// Released on July 9, 2004
#ifndef FFT_H
#define FFT_H

#include <vector>

#include "def.h"
using namespace std;

// xfft分别是复型数组的FFT变换
// ixfft分别是复型数组的FFT反变换
void inifft(long n);
void xfft(double *xRe, double *xIm, long n,
	double *yRe, double *yIm);
void xfft(vector<double>& xRe, vector<double>& xIm);
void ixfft(double *xRe, double *xIm, long n,
	double *yRe, double *yIm);
void ixfft(vector<double>& xRe, vector<double>& xIm);

void xfft(float *xRe, float *xIm, long n,
	float *yRe, float *yIm);
void xfft(vector<float>& xRe, vector<float>& xIm);
void ixfft(float *xRe, float *xIm, long n,
	float *yRe, float *yIm);
void ixfft(vector<float>& xRe, vector<float>& xIm);
void freeFFT(void);


#endif