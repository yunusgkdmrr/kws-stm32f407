/* kws_model.h — KWS modelini çalıştıran küçük sarmalayıcı */
#pragma once
#include <stdint.h>

#define KWS_NCLASS 10
#define KWS_NKEYWORD 8     /* 0..7 komut, 8 = sessizlik, 9 = bilinmeyen */
extern const char *const KWS_LABELS[KWS_NCLASS];   /* down, go, left, no, right, stop, up, yes, sessiz, bilinmeyen */

/* Açılışta bir kez çağır. 0 = başarılı. */
int kws_init(void);

/* mfcc   : 49 x 10 float MFCC (mfcc_frame çıktıları arka arkaya)
 * scores : 8 adet int8 skor (olasılık = (skor + 128) / 256)
 * dönüş  : en yüksek skorlu sınıfın numarası, hata olursa -1 */
int kws_run(const float *mfcc, int8_t *scores);

/* int8 skoru yüzdeye çevir (0..100) */
static inline int kws_pct(int8_t s) { return ((int)s + 128) * 100 / 256; }
