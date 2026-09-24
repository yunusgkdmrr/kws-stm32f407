/* kws_model.c — MFCC -> normalizasyon -> int8 -> model -> skorlar
 *
 * Colab'daki adımların birebir aynısı:
 *   m = (mfcc - mu) / sd                      (normalizasyon)
 *   q = round(m / scale + zero_point)         (int8'e çevirme, [-128, 127])
 * mu, sd, scale ve zero_point mfcc_consts.h içinde, modelle birlikte üretildi.
 */
#include "kws_model.h"
#include "network.h"
#include "network_data.h"
#include "mfcc_consts.h"
#include <math.h>
#include <stddef.h>

/* Sınıf sırası notebook 03 ile aynı: 8 komut (alfabetik), sonra _silence_ ve _unknown_ */
const char *const KWS_LABELS[KWS_NCLASS] = {
  "down", "go", "left", "no", "right", "stop", "up", "yes", "sessiz", "bilinmeyen"
};

/* Modelin çalışma alanı (9,4 KB). Giriş ve çıkış tamponları da bunun içinde. */
AI_ALIGNED(32) static uint8_t pool0[AI_NETWORK_DATA_ACTIVATION_1_SIZE];
static ai_handle activations[] = { pool0 };

static ai_handle net = AI_HANDLE_NULL;
static ai_buffer *in_buf;
static ai_buffer *out_buf;

int kws_init(void)
{
  ai_error err = ai_network_create_and_init(&net, activations, NULL);
  if (err.type != AI_ERROR_NONE)
    return -1;
  in_buf  = ai_network_inputs_get(net, NULL);
  out_buf = ai_network_outputs_get(net, NULL);
  return 0;
}

int kws_run(const float *mfcc, int8_t *scores)
{
  /* Giriş tamponu aktivasyon alanının içinde ve model çalışırken üzerine
   * yazılıyor; bu yüzden her çıkarımdan önce yeniden dolduruyoruz. */
  int8_t *in = (int8_t *)in_buf[0].data;

  for (int i = 0; i < MFCC_NFRAMES * MFCC_NCOEF; i++) {
    int c = i % MFCC_NCOEF;                              /* hangi katsayı (0..9) */
    float z = (mfcc[i] - MFCC_MU[c]) / MFCC_SD[c];       /* normalizasyon        */
    float q = roundf(z / MODEL_IN_SCALE) + MODEL_IN_ZP;  /* int8'e çevir         */
    if (q > 127.0f)  q = 127.0f;
    if (q < -128.0f) q = -128.0f;
    in[i] = (int8_t)q;
  }

  if (ai_network_run(net, in_buf, out_buf) != 1)
    return -1;

  const int8_t *out = (const int8_t *)out_buf[0].data;
  int best = 0;
  for (int k = 0; k < KWS_NCLASS; k++) {
    scores[k] = out[k];
    if (out[k] > out[best]) best = k;
  }
  return best;
}
