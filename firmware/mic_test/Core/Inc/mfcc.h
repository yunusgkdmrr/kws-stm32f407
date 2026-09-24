/* mfcc.h — Kartta MFCC hesabı (Colab/TensorFlow ile birebir aynı tarif) */
#pragma once
#include <stdint.h>

#define MFCC_FRAME_LEN   640   /* 40 ms @ 16 kHz */
#define MFCC_FRAME_HOP   320   /* 20 ms @ 16 kHz */
#define MFCC_OUT_COEF    10
#define MFCC_OUT_FRAMES  49    /* 1 saniyede 49 pencere */

/* Açılışta bir kez çağır (FFT tablolarını hazırlar). */
void mfcc_init(void);

/* Tek bir 40 ms'lik pencerenin MFCC'si.
 * x   : 640 adet int16 ses örneği (PDM2PCM çıktısı, 16 kHz)
 * out : 10 adet float MFCC katsayısı
 */
void mfcc_frame(const int16_t *x, float *out);
