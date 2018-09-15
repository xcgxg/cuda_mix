#include "../StdAfx.h"
#include <stdlib.h>
#include <stdio.h>
#include <conio.h>
#include <memory.h>
#include "fftw3.h"
#include "fft.h"

#include <vector>

#include "def.h"


fftwf_complex *in;
fftwf_complex *out;
fftwf_plan p;
//FFT初始化标志
bool FFT_init_flag = false;

void ixfft(double *xRe, double *xIm, long n,
	double *yRe, double *yIm)
{
	double dv;
	long int i;

	//inv_const();
	for (i = 0; i<n; i++) xIm[i] = -xIm[i];
	xfft(xRe, xIm, n, yRe, yIm);
	//inv_const();
	dv = 1.0 / double(n);
	for (i = 0; i<n; i++)
	{
		yRe[i] = yRe[i] * dv;
		yIm[i] = -yIm[i] * dv;
	}
}

void ixfft(vector<double>& xRe, vector<double>& xIm)
{
	int n = xRe.size();

	double dv;
	long int i;

	//inv_const();
	for (i = 0; i<n; i++) xIm[i] = -xIm[i];
	xfft(xRe, xIm);
	//inv_const();
	dv = 1.0 / double(n);
	for (i = 0; i<n; i++)
	{
		xRe[i] = xRe[i] * dv;
		xIm[i] = -xIm[i] * dv;
	}
}

void xfft(double *xRe, double *xIm, long n,
	double *yRe, double *yIm)
{
	int i;
	for (i = 0; i<n; i++)
	{
		in[i][0] = xRe[i];
		in[i][1] = xIm[i];
	}
	fftwf_execute(p);
	for (i = 0; i<n; i++)
	{
		yRe[i] = out[i][0];
		yIm[i] = out[i][1];
	}
}   /* fft */

void xfft(vector<double>& xRe, vector<double>& xIm)
{
	int n = xRe.size();
	int i;
	for (i = 0; i<n; i++)
	{
		in[i][0] = xRe[i];
		in[i][1] = xIm[i];
	}
	fftwf_execute(p);
	for (i = 0; i<n; i++)
	{
		xRe[i] = out[i][0];
		xIm[i] = out[i][1];
	}
}

void ixfft(float *xRe, float *xIm, long n,
	float *yRe, float *yIm)
{
	float dv;
	long int i;

	//inv_const();
	for (i = 0; i<n; i++) xIm[i] = -xIm[i];
	xfft(xRe, xIm, n, yRe, yIm);
	//inv_const();
	dv = 1.0 / float(n);
	for (i = 0; i<n; i++)
	{
		yRe[i] = yRe[i] * dv;
		yIm[i] = -yIm[i] * dv;
	}
}

void ixfft(vector<float>& xRe, vector<float>& xIm)
{
	int n = xRe.size();

	float dv;
	long int i;

	//inv_const();
	for (i = 0; i<n; i++) xIm[i] = -xIm[i];
	xfft(xRe, xIm);
	//inv_const();
	dv = 1.0 / float(n);
	for (i = 0; i<n; i++)
	{
		xRe[i] = xRe[i] * dv;
		xIm[i] = -xIm[i] * dv;
	}
}

void inifft(long n)//inifft(nMax);
{
	//(fftw_complex *)这个强制类型转换必须加，否则编译报错
	in = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*n);
	out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex)*n);
	p = fftwf_plan_dft_1d(n, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

}

void xfft(float *xRe, float *xIm, long n,
	float *yRe, float *yIm)
{
	int i;
	for (i = 0; i<n; i++)
	{
		in[i][0] = xRe[i];
		in[i][1] = xIm[i];
	}
	fftwf_execute(p);
	for (i = 0; i<n; i++)
	{
		yRe[i] = out[i][0];
		yIm[i] = out[i][1];
	}
}   /* fft */

void xfft(vector<float>& xRe, vector<float>& xIm)
{
	int n = xRe.size();
	int i;
	for (i = 0; i<n; i++)
	{
		in[i][0] = xRe[i];
		in[i][1] = xIm[i];
	}
	fftwf_execute(p);
	for (i = 0; i<n; i++)
	{
		xRe[i] = out[i][0];
		xIm[i] = out[i][1];
	}
}


void freeFFT(void)
{
	fftwf_destroy_plan(p);
	fftwf_free(in);
	fftwf_free(out);
}


