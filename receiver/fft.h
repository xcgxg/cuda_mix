// Released on July 9, 2004

#ifndef FFT_H
#define FFT_H

// xfft�ֱ��Ǹ��������FFT�任
// ixfft�ֱ��Ǹ��������FFT���任
void inifft(long n);
void xfft(float *xRe, float *xIm,long n, 
          float *yRe, float *yIm);
void ixfft(float *xRe, float *xIm,long n, 
           float *yRe, float *yIm);
void freeFFT(void);


#endif