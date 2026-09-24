/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "mfcc.h"
#include "kws_model.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* ---- Tampon boyutları ------------------------------------------------------
 * DMA tamponu : 2048 paket (16 bit), iki yarı x 1024 paket
 * PDM2PCM     : her çağrıda 64 paket (1024 PDM biti) alır, 16 ses örneği (1 ms) verir
 * Bir yarı    : 1024 / 64 = 16 çağrı -> 16 x 16 = 256 ses örneği (16 ms)
 */
#define PDM_BUF_LEN      2048u
#define PDM_WORDS_PER_MS 64u
#define PCM_PER_MS       16u
#define PCM_PER_HALF     256u
#define REC_LEN          16000u          /* 1 saniye @ 16 kHz */

uint16_t pdm_buf[PDM_BUF_LEN];           /* DMA'nın mikrofon bitlerini yazdığı tampon */
int16_t  pcm_half[PCM_PER_HALF];         /* son yarım tamponun GERÇEK sesi (16 kHz)   */
int16_t  rec_buf[REC_LEN];               /* 1 saniyelik kayıt (32 KB RAM)             */

volatile uint8_t  g_rec_done   = 0;      /* 1 = 1 sn'lik pencere hazır, işlenmeyi bekliyor */

/* ---- Sesle tetiklenen kayıt (VAD) ------------------------------------------
 * Kart sürekli dinler, son 1,25 sn'yi halka tamponda tutar. Seviye tabanın
 * 2,5 katını geçince "konuşma başladı" der ve konuşmanın 250 ms öncesinden
 * başlayan 1 sn'lik pencereyi modele verir. Böylece kelime pencerenin ortasına düşer.
 */
#define RING_LEN      20000u    /* 1,25 sn halka tampon                        */
#define PRE_ROLL       4000u    /* konuşmanın 250 ms öncesini de al            */
#define VAD_ON_X10       45u    /* taban seviyenin 4,5 katı = konuşma olabilir  */
#define VAD_MIN_BLK       6u    /* eşik arka arkaya 6 blok (~96 ms) aşılmalı    */
#define COOLDOWN_BLK     20u    /* sonuçtan sonra ~320 ms dinlemeye ara ver     */
#define CONF_MIN         70     /* %70'in altındaki tahminler "emin değil"      */

/* ---- Yanlış kabulü azaltan kapılar (L5) ----------------------------------
 * Komutlar tek kelimelik ve öncesi/sonrası sessiz; sohbet ise akıcı ve uzun. */
#define SND_X10          35u    /* "ses var" sayılan seviye: tabanın 3,5 katı       */
#define PRE_MAX_ACTIVE    4u    /* öncesindeki 16 blokta en fazla 4 gürültü bloğu   */
#define POST_MAX_ACTIVE   2u    /* son 5 blokta en fazla 2 ses bloğu                */
#define HOLD_MAX_ACTIVE   2u    /* bekleme: son 12 blokta en fazla 2 ses bloğu      */
#define PRE_QUIET_BLK    16u    /* tetikten önceki ~256 ms sessiz olmalı            */
#define EDGE_SKIP_BLK     3u    /* kelimenin yükselen kenarı (~48 ms) kontrol dışı */
#define POST_QUIET_BLK    5u    /* pencerenin son ~80 ms'i sessiz olmalı (kısa söz) */
#define HOLD_QUIET_BLK   12u    /* reddedince ~200 ms sessizlik gelene kadar bekle  */
#define MARGIN_MIN       40     /* en iyi iki tahmin arasında en az 40 puan fark    */

/* 1 = VERİ TOPLAMA modu: mavi düğmeye bas -> 0,4 sn sonra turuncu LED yanar ->
 *     sonraki 3 sn içindeki İLK konuşma yakalanıp Mac'e gönderilir.
 * 0 = DEMO modu: sürekli dinler, tanır, LED yakar (mavi düğme debug gönderimini açar/kapatır). */
#define RECORD_MODE       0

enum { ST_WAIT, ST_IDLE, ST_CAPTURE, ST_BUSY, ST_COOLDOWN, ST_HOLD };

int16_t ring[RING_LEN];                  /* son 1,25 sn'lik ses                      */
volatile uint32_t total_samples = 0;     /* açılıştan beri gelen toplam örnek sayısı */
volatile uint8_t  vad_state     = ST_WAIT;
volatile uint32_t cap_start     = 0;     /* yakalanan pencerenin başlangıç örneği    */
volatile uint32_t cooldown      = 0;
volatile uint32_t g_trig_ratio  = 0;     /* tetiklendiği andaki seviye oranı x10     */
volatile uint32_t on_blocks     = 0;     /* eşiğin arka arkaya aşıldığı blok sayısı  */
volatile uint32_t g_idle_peak   = 0;     /* dinlerken görülen en yüksek oran x10     */
volatile uint32_t g_hist        = 0;     /* son 32 bloğun ses geçmişi (1 = ses var)   */
volatile uint32_t g_rej_pre     = 0;     /* red: öncesinde sessizlik yok (cümle ortası) */
volatile uint32_t g_rej_long    = 0;     /* red: konuşma pencereden uzun (cümle)      */
volatile uint32_t g_last_pre    = 0;     /* son redde öncesindeki dolu blok sayısı     */
volatile uint32_t g_last_post   = 0;     /* son redde sondaki dolu blok sayısı         */
volatile int32_t  g_base_acc    = 0;     /* taban x 64 (küçük değişimler kaybolmasın) */
volatile uint8_t  g_send_debug  = 0;     /* 1 = sesi ve MFCC'yi de Mac'e gönder      */
uint32_t led_off_at = 0;                 /* LED'lerin söneceği zaman (ms)            */
uint32_t arm_at      = 0;                /* kayıt modu: dinlemenin başlayacağı zaman */
uint32_t armed_until = 0;                /* kayıt modu: dinlemenin biteceği zaman    */

volatile uint32_t g_blocks  = 0;         /* işlenen yarım tampon sayısı            */
volatile uint32_t g_level   = 0;         /* anlık ses seviyesi (ortalama |örnek|)   */
volatile uint32_t g_base    = 0;         /* sessiz ortamın seviyesi (taban)         */
volatile uint32_t g_ratio10 = 0;         /* anlık / taban oranı x 10               */
volatile uint32_t g_peak10  = 10;        /* tepe değeri: hızlı yükselir, yavaş iner */
volatile uint32_t g_proc_cycles = 0;     /* bir yarım tamponu işleme süresi (çevrim) */

extern PDM_Filter_Handler_t PDM1_filter_handler;  /* pdm2pcm.c'de tanımlı */

float mfcc_out[MFCC_OUT_FRAMES * MFCC_OUT_COEF];   /* kaydın MFCC'si: 49 x 10 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf köprüsü: her karakteri USART2'den gönder */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* Çevrim sayacı (kronometre): işleme süresini ölçmek için */
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Bir yarım tamponu işle: PDM bitleri -> gerçek ses -> seviye (+ kayıt) */
static void process_half(const uint16_t *src)
{
  uint32_t t0 = DWT->CYCCNT;
  uint16_t tmp[PDM_WORDS_PER_MS];

  /* 1) PDM -> PCM: 16 parça x (64 paket -> 16 ses örneği) */
  for (uint32_t c = 0; c < PCM_PER_HALF / PCM_PER_MS; c++) {
    /* ST'nin örneğindeki gibi her 16 bitlik paketin iki byte'ının yerini değiştir */
    for (uint32_t j = 0; j < PDM_WORDS_PER_MS; j++)
      tmp[j] = (uint16_t)__REV16(src[c * PDM_WORDS_PER_MS + j]);
    PDM_Filter(tmp, &pcm_half[c * PCM_PER_MS], &PDM1_filter_handler);
  }

  /* 2) Ses seviyesi: örneklerin mutlak değerlerinin ortalaması */
  uint32_t sumabs = 0;
  for (uint32_t i = 0; i < PCM_PER_HALF; i++)
    sumabs += (uint32_t)abs(pcm_half[i]);
  g_level = sumabs / PCM_PER_HALF;

  /* 3) Sesi halka tampona yaz */
  for (uint32_t i = 0; i < PCM_PER_HALF; i++)
    ring[(total_samples + i) % RING_LEN] = pcm_half[i];
  total_samples += PCM_PER_HALF;

  /* 4) Ses algılama (VAD) durum makinesi */
  uint32_t ratio = g_base ? (g_level * 10u) / g_base : 0;
  g_hist = (g_hist << 1) | (ratio >= SND_X10 ? 1u : 0u);   /* ses geçmişi */

  switch (vad_state) {
    case ST_IDLE:                                   /* dinliyor, konuşma bekliyor */
      if (ratio > g_idle_peak) g_idle_peak = ratio; /* sessizlik istatistiği */
      if (ratio >= VAD_ON_X10) {
        if (++on_blocks >= VAD_MIN_BLK) {           /* yeterince uzun sürdü: konuşma */
#if !RECORD_MODE
          /* KAPI 1: kelimeden önce sessizlik var mı? Yoksa bu bir cümlenin ortası. */
          uint32_t before = (g_hist >> (VAD_MIN_BLK + EDGE_SKIP_BLK)) & ((1u << PRE_QUIET_BLK) - 1u);
          uint32_t n_before = (uint32_t)__builtin_popcount(before);
          if (n_before > PRE_MAX_ACTIVE) {
            g_last_pre = n_before;
            g_rej_pre++;
            on_blocks = 0;
            vad_state = ST_HOLD;                    /* konuşma bitene kadar bekle */
            break;
          }
#endif
          cap_start = total_samples - VAD_MIN_BLK * PCM_PER_HALF - PRE_ROLL;
          g_trig_ratio = ratio;
          on_blocks = 0;
          vad_state = ST_CAPTURE;
        }
      } else {
        on_blocks = 0;
        /* Uyarlanan taban: ortam sessizken taban yavaşça ortama uyar (~1 sn) */
        if (ratio < 20u) {
          int32_t d = (int32_t)g_level - g_base_acc / 64;
          g_base_acc += (d < 0) ? d * 4 : d / 4;   /* inerken hızlı, çıkarken yavaş */
          int32_t nb = g_base_acc / 64;
          g_base = (nb < 1) ? 1u : (uint32_t)nb;
        }
      }
      break;
    case ST_CAPTURE:                                /* pencere dolana kadar bekle */
      if (total_samples >= cap_start + REC_LEN) {
#if !RECORD_MODE
        /* KAPI 2: pencerenin sonunda hâlâ konuşma var mı? Varsa bu bir cümle. */
        uint32_t n_after = (uint32_t)__builtin_popcount(g_hist & ((1u << POST_QUIET_BLK) - 1u));
        if (n_after > POST_MAX_ACTIVE) {
          g_last_post = n_after;
          g_rej_long++;
          vad_state = ST_HOLD;
          break;
        }
#endif
        for (uint32_t i = 0; i < REC_LEN; i++)
          rec_buf[i] = ring[(cap_start + i) % RING_LEN];
        g_rec_done = 1;
        vad_state = ST_BUSY;                        /* ana döngü işleyecek */
      }
      break;
    case ST_HOLD:                                   /* sessizlik gelene kadar bekle */
      if ((uint32_t)__builtin_popcount(g_hist & ((1u << HOLD_QUIET_BLK) - 1u)) <= HOLD_MAX_ACTIVE)
        vad_state = ST_IDLE;
      break;
    case ST_COOLDOWN:                               /* kısa ara, sonra tekrar dinle */
      if (cooldown) cooldown--;
      else          vad_state = ST_IDLE;
      break;
    default:                                        /* ST_WAIT, ST_BUSY */
      break;
  }

  g_blocks++;
  g_proc_cycles = DWT->CYCCNT - t0;
}

/* Tahmini LED'lerle göster (LED'ler kartta çember şeklinde):
 *   yeşil = sol (LD4), turuncu = üst (LD3), kırmızı = sağ (LD5), mavi = alt (LD6)
 * Sınıf sırası: 0 down, 1 go, 2 left, 3 no, 4 right, 5 stop, 6 up, 7 yes */
static void show_result(int k)
{
  static const uint16_t led_for[KWS_NCLASS] = {
    LD6_Pin,                             /* down  -> alt           */
    LD4_Pin | LD5_Pin,                   /* go    -> sol + sağ     */
    LD4_Pin,                             /* left  -> sol           */
    0,                                   /* no    -> hiçbiri       */
    LD5_Pin,                             /* right -> sağ           */
    LD3_Pin | LD6_Pin,                   /* stop  -> üst + alt     */
    LD3_Pin,                             /* up    -> üst           */
    LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin /* yes -> hepsi          */
  };
  HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);
  if (led_for[k]) HAL_GPIO_WritePin(GPIOD, led_for[k], GPIO_PIN_SET);
  led_off_at = HAL_GetTick() + 1500;     /* 1,5 sn sonra söndür */
}

/* DMA: tamponun ilk yarısı doldu -> ilk yarıyı işle (DMA şu an ikinci yarıya yazıyor) */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI2)
    process_half(&pdm_buf[0]);
}

/* DMA: tamponun ikinci yarısı doldu -> ikinci yarıyı işle (DMA başa döndü) */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == SPI2)
    process_half(&pdm_buf[PDM_BUF_LEN / 2]);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_PDM2PCM_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);   /* printf beklemeden hemen göndersin */
  dwt_init();
  mfcc_init();
  printf("\r\n[mic_test L6] basladi, SYSCLK = %lu Hz\r\n", (unsigned long)SystemCoreClock);

  HAL_I2S_Receive_DMA(&hi2s2, pdm_buf, PDM_BUF_LEN);   /* mikrofonu dinlemeye başla */

  /* İlk ~0,5 sn: mikrofonun ve süzgecin açılış geçişi, yok say */
  uint32_t seen = g_blocks;
  while (g_blocks < seen + 32) {}

  /* Sonraki ~0,5 sn: ortam SESSİZKEN taban seviyeyi ölç */
  seen = g_blocks;
  uint32_t acc = 0;
  for (int k = 0; k < 32; k++) {
    while (g_blocks == seen) {}        /* yeni yarım tampon bekle */
    seen = g_blocks;
    acc += g_level;
  }
  g_base = acc / 32;
  if (g_base == 0) g_base = 1;         /* sıfıra bölmeyi önle */
  g_base_acc = (int32_t)g_base * 64;
  printf("taban olculdu: g_base = %lu\r\n", (unsigned long)g_base);
  if (kws_init() == 0) printf("MODEL: hazir\r\n");
  else                 printf("MODEL: baslatilamadi!\r\n");
#if RECORD_MODE
  g_send_debug = 1;
  vad_state = ST_WAIT;                 /* düğme beklenir */
  printf("KAYIT MODU: mavi dugmeye bas, turuncu LED yaninca kelimeyi soyle\r\n");
#else
  vad_state = ST_IDLE;                 /* dinlemeye başla */
  printf("DINLIYOR: bir kelime soyle (yes, no, up, down, left, right, go, stop)\r\n");
  printf("(Mavi dugme: sesi ve MFCC'yi Mac'e gonderme modunu acar/kapatir)\r\n");
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* ---- Mavi düğme: hata ayıklama gönderimini aç/kapa ---- */
    static GPIO_PinState prev_btn = GPIO_PIN_RESET;
    GPIO_PinState btn = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
    if (btn == GPIO_PIN_SET && prev_btn == GPIO_PIN_RESET) {
#if RECORD_MODE
      if (vad_state == ST_WAIT && !g_rec_done && !arm_at) {
        arm_at = HAL_GetTick() + 400;              /* düğmenin tık sesini atla */
        HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);
      }
#else
      g_send_debug ^= 1;
      printf("DEBUG gonderim: %s\r\n", g_send_debug ? "ACIK" : "KAPALI");
#endif
    }
    prev_btn = btn;

#if RECORD_MODE
    /* 0,4 sn doldu: dinlemeye başla, turuncu LED = "SÖYLE" */
    if (arm_at && HAL_GetTick() >= arm_at) {
      arm_at = 0;
      on_blocks = 0;
      armed_until = HAL_GetTick() + 3000;
      vad_state = ST_IDLE;
      HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
    }
    /* 3 sn içinde konuşma gelmediyse vazgeç */
    if (armed_until && vad_state == ST_IDLE && HAL_GetTick() >= armed_until) {
      vad_state = ST_WAIT;
      armed_until = 0;
      HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
      printf("ZAMAN ASIMI: konusma algilanmadi\r\n");
    }
#endif

    /* ---- Konuşma yakalandıysa: MFCC -> model -> karar ---- */
    if (g_rec_done) {
      uint32_t t0 = DWT->CYCCNT;
      for (int f = 0; f < MFCC_OUT_FRAMES; f++)
        mfcc_frame(&rec_buf[f * MFCC_FRAME_HOP], &mfcc_out[f * MFCC_OUT_COEF]);
      uint32_t mfcc_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000u);

      int8_t sc[KWS_NCLASS];
      uint32_t t1 = DWT->CYCCNT;
      int best = kws_run(mfcc_out, sc);
      uint32_t model_us = (DWT->CYCCNT - t1) / (SystemCoreClock / 1000000u);

      if (best < 0) {
        printf("TAHMIN: model hatasi!\r\n");
      } else {
        /* en yüksek 3 skor */
        int top[3];
        for (int r = 0; r < 3; r++) {
          int b = -1;
          for (int k = 0; k < KWS_NCLASS; k++) {
            int used = 0;
            for (int j = 0; j < r; j++) if (top[j] == k) used = 1;
            if (!used && (b < 0 || sc[k] > sc[b])) b = k;
          }
          top[r] = b;
        }
        int conf   = kws_pct(sc[best]);
        int margin = conf - kws_pct(sc[top[1]]);          /* KAPI 3: kararlılık farkı */
        if (best >= KWS_NKEYWORD) {
          /* Model "komut değil" dedi: sessizlik ya da bilinmeyen kelime */
          printf("KOMUT DEGIL (%-10s %3d%%)", KWS_LABELS[best], conf);
        } else if (conf >= CONF_MIN && margin >= MARGIN_MIN) {
          show_result(best);
          printf("TAHMIN: %-5s %3d%%", KWS_LABELS[best], conf);
        } else {
          printf("EMIN DEGIL     ");
        }
        printf(" | ilk 3: %s=%d%% %s=%d%% %s=%d%% | ses x%lu.%lu | mfcc=%lu us model=%lu us\r\n",
               KWS_LABELS[top[0]], kws_pct(sc[top[0]]),
               KWS_LABELS[top[1]], kws_pct(sc[top[1]]),
               KWS_LABELS[top[2]], kws_pct(sc[top[2]]),
               (unsigned long)(g_trig_ratio / 10), (unsigned long)(g_trig_ratio % 10),
               (unsigned long)mfcc_us, (unsigned long)model_us);
      }

      /* İstenirse doğrulama için sesi ve MFCC'yi de gönder */
      if (g_send_debug) {
        printf("BEGIN %u\r\n", (unsigned)REC_LEN);
        HAL_UART_Transmit(&huart2, (uint8_t *)rec_buf, REC_LEN * 2, HAL_MAX_DELAY);
        printf("\r\nMFCC %u\r\n", (unsigned)(MFCC_OUT_FRAMES * MFCC_OUT_COEF));
        HAL_UART_Transmit(&huart2, (uint8_t *)mfcc_out, sizeof(mfcc_out), HAL_MAX_DELAY);
        printf("\r\nEND mfcc_49_pencere_us=%lu\r\n", (unsigned long)mfcc_us);
      }

      g_rec_done = 0;
#if RECORD_MODE
      armed_until = 0;
      vad_state   = ST_WAIT;             /* bir sonraki düğmeyi bekle */
      HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);
      led_off_at  = 0;
#else
      cooldown   = COOLDOWN_BLK;
      vad_state  = ST_COOLDOWN;          /* kısa ara, sonra tekrar dinle */
#endif
    }

    /* ---- Dinlerken her 2 sn'de bir sessizlik istatistiği ---- */
    static uint32_t next_stat = 0;
    if (!RECORD_MODE && vad_state == ST_IDLE && HAL_GetTick() >= next_stat) {
      next_stat = HAL_GetTick() + 2000;
      printf("dinliyor | son 2 sn en yuksek oran x%lu.%lu (esik x%lu.%lu) | taban %lu\r\n",
             (unsigned long)(g_idle_peak / 10), (unsigned long)(g_idle_peak % 10),
             (unsigned long)(VAD_ON_X10 / 10), (unsigned long)(VAD_ON_X10 % 10),
             (unsigned long)g_base);
      g_idle_peak = 0;
    }

    /* ---- Kapıların reddettiklerini yaz (ayar yaparken işe yarar) ---- */
    static uint32_t seen_pre = 0, seen_long = 0;
    if (g_rej_pre != seen_pre) {
      seen_pre = g_rej_pre;
      printf("RED: cumlenin ortasi (oncesi %lu/16 blok dolu)  [toplam %lu]\r\n", (unsigned long)g_last_pre, (unsigned long)seen_pre);
    }
    if (g_rej_long != seen_long) {
      seen_long = g_rej_long;
      printf("RED: uzun konusma (son %lu/5 blok dolu)  [toplam %lu]\r\n", (unsigned long)g_last_post, (unsigned long)seen_long);
    }

    /* ---- LED'leri zamanı gelince söndür ---- */
    if (led_off_at && HAL_GetTick() >= led_off_at) {
      HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);
      led_off_at = 0;
    }

    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_CRC_DR_RESET(&hcrc);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_LSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
