// Released on July 9, 2004

#ifndef FFT_H
#define FFT_H

// xfft分别是复型数组的FFT变换
// ixfft分别是复型数组的FFT反变换
void inifft(long n);
void xfft(float *xRe, float *xIm,long n, 
          float *yRe, float *yIm);
void ixfft(float *xRe, float *xIm,long n, 
           float *yRe, float *yIm);
void freeFFT(void);


#endif