/* mfcc.c — Python'daki mfcc_ref() fonksiyonunun birebir C tercümesi.
 *
 *   ses (640) -> /32768 -> pencere -> 1024'e sıfırla tamamla -> FFT
 *   -> genlik (0-4 kHz, 257 kutu) -> mel filtreleri (40) -> log -> DCT (10)
 *
 * Bütün sabitler (pencere, mel filtreleri, DCT) mfcc_consts.h içinde ve
 * doğrudan TensorFlow'dan kopyalandı.
 */
#include "mfcc.h"
#include "arm_math.h"
#include "mfcc_consts.h"
#include <math.h>

static arm_rfft_fast_instance_f32 fft;

/* Büyük dizileri static tutuyoruz: yığın (stack) varsayılan olarak sadece
 * 1 KB, bu dizileri fonksiyon içinde tanımlasaydık taşardı. */
static float32_t buf[MFCC_NFFT];     /* FFT girişi (FFT bunu çalışma alanı olarak da kullanır) */
static float32_t spec[MFCC_NFFT];    /* FFT çıkışı (paketlenmiş karmaşık sayılar)             */
static float32_t mag[MFCC_NBINS];    /* frekans kutularının genliği                            */
static float32_t mel[MFCC_NMEL];     /* mel filtre çıkışları                                   */

void mfcc_init(void)
{
  arm_rfft_fast_init_f32(&fft, MFCC_NFFT);
}

void mfcc_frame(const int16_t *x, float *out)
{
  int i, k, m, c;

  /* 1) int16 -> [-1, 1] float, pencereyle çarp, 1024'e sıfırla tamamla */
  for (i = 0; i < MFCC_WIN_LEN; i++)
    buf[i] = ((float32_t)x[i] / 32768.0f) * MFCC_WINDOW[i];
  for (; i < MFCC_NFFT; i++)
    buf[i] = 0.0f;

  /* 2) FFT. CMSIS çıktı formatı:
   *    spec[0] = 0 Hz (gerçek), spec[1] = 8 kHz (gerçek),
   *    k. kutu için spec[2k] = gerçek kısım, spec[2k+1] = sanal kısım */
  arm_rfft_fast_f32(&fft, buf, spec, 0);

  /* 3) Genlik: |X[k]|, sadece 0-4 kHz (257 kutu) */
  mag[0] = fabsf(spec[0]);
  for (k = 1; k < MFCC_NBINS; k++) {
    float32_t re = spec[2 * k], im = spec[2 * k + 1];
    mag[k] = sqrtf(re * re + im * im);
  }

  /* 4) Mel filtreleri: mel = mag (1x257) @ MELW (257x40) */
  for (m = 0; m < MFCC_NMEL; m++)
    mel[m] = 0.0f;
  for (k = 0; k < MFCC_NBINS; k++) {
    const float *w = &MFCC_MELW[k * MFCC_NMEL];
    float32_t a = mag[k];
    for (m = 0; m < MFCC_NMEL; m++)
      mel[m] += a * w[m];
  }

  /* 5) log (TensorFlow'daki gibi +1e-6) */
  for (m = 0; m < MFCC_NMEL; m++)
    mel[m] = logf(mel[m] + 1e-6f);

  /* 6) DCT: out = logmel (1x40) @ DCT (40x10) */
  for (c = 0; c < MFCC_NCOEF; c++) {
    float32_t s = 0.0f;
    for (m = 0; m < MFCC_NMEL; m++)
      s += mel[m] * MFCC_DCT[m * MFCC_NCOEF + c];
    out[c] = s;
  }
}
