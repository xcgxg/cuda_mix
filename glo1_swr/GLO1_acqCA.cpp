// Released on July 9, 2004
#include "../stdafx.h"
#include <math.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>

#include "GLO1_CAcode.h"
#include "GLO1_gpsconst.h"
#include "GLO1_acqCA.h"
#include "../receiver/fft.h"

extern long GLO1_FFTSIZE;
extern double GLO1_SAMPLING_FREQ;
extern long GLO1_SAMPLESPMS;

//#define GLO1_ACQ_DEBUG

// 全局堆指针，用于在堆上分配全局数组
float	**GLO1_corr;
float	*GLO1_rCa, *GLO1_iCa;
float	*GLO1_rInData, *GLO1_iInData;
float	*GLO1_rProduct, *GLO1_iProduct;
float	*GLO1_cosPhase, *GLO1_sinPhase;

// 全局数组分配
void GLO1_initSignalSearch()
{
	long i;
	long numFreqBin;
	long nMax;

	nMax = GLO1_FFTSIZE;
	inifft(nMax);

	numFreqBin = (long)((GLO1_MAXF - GLO1_MINF) / GLO1_DELTF + 1.5);

	if ((GLO1_corr = (float**) malloc(numFreqBin*sizeof(float*)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	for (i=0;i<numFreqBin;i++)
	{
		if ((GLO1_corr[i] = (float*) malloc(nMax*sizeof(float)))==NULL)
		{
			puts("memory allocation error in acqca.cpp");
			exit(0);
		}
	}
	if ((GLO1_rCa = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((GLO1_iCa = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}

	if ((GLO1_cosPhase = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((GLO1_sinPhase = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((GLO1_rInData = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((GLO1_iInData = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((GLO1_rProduct = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((GLO1_iProduct = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
}
extern void  GLO1_write_prn();

void GLO1_freeSignalSearch()
{
	long i;
	long numFreqBin;

	numFreqBin = (long)((GLO1_MAXF - GLO1_MINF) / GLO1_DELTF + 1.5);				// 要搜索的频率总数
	GLO1_write_prn();
	// 释放全部堆变量
	for (i=0;i<numFreqBin;i++)
	{
		free(GLO1_corr[i]);
	}
	free(GLO1_corr);
	free(GLO1_rCa);
	free(GLO1_iCa);
	free(GLO1_sinPhase);
	free(GLO1_cosPhase);
	free(GLO1_rInData);
	free(GLO1_iInData);
	free(GLO1_rProduct);
	free(GLO1_iProduct);
	freeFFT();
}

int GLO1_acqCA(double *data,			// input data sequence       
          long num,				// data length			
          char** caTable,		// CA CODE TABLE	
          short sv,				// sv number: 1-37 
          double fs,			// sampling frequency			   
          double *freqOffset,	// frequency offset array in Hz	
          long numFreqBin,		// length of freqOffset	
          double freqCarrier,	// nomial carrier frequence in Hz
          double *tau,			// detected code phase in code chip: 
          double *doppler,		// detected doppler in Hz: output
          long *nTrial,			// number of trials: output
          double *Rmax,			// returned correlation peak: output 
          double *threshold)	// threshold of signal detection: input/output
{
	long	nMax, nLocal;
	long	nMsec;
    long    lSampleOffset;   
    double  dSampleOffset;
    double  sampleOffsetPerMilliSecond;
	double	deltaT;
	float	*pCorr;
	double	phase, deltaPhase;
	float	*rIn, *iIn;
	float	*pROut, *pIOut;
	float	*pRCa, *pICa;
	double	*pData, *p1Data;
	float	*pRProduct, *pIProduct;
	long	i, j, index, k;
	double	peak, abs2Product;
	float	*pCos, *pSin;
#ifdef GLO1_ACQ_DEBUG
 	FILE	*fCorrTest, *fSignalTest;						//add by ddkk
 	fCorrTest = fopen("E:\\corr.dat","w");						//add by ddkk
// 	fSignalTest = fopen("signal.dat","w");					//add by ddkk
#endif
	nMax = (long)(GLO1_SAMPLING_FREQ*1e-3+0.5);		
    nLocal = (long)(GLO1_SAMPLING_FREQ*2e-3+0.5);    
	nMsec= num/nMax;			               
    sampleOffsetPerMilliSecond = GLO1_SAMPLING_FREQ*1e-3 - nMax;

	deltaT=1/fs;

	for (j=0; j<numFreqBin; j++)
	{
		pCorr = GLO1_corr[j];
		for (i=0; i<nMax; i++) *pCorr++ = 0.0;
	}
	
	GLO1_genCaTimeSequence(caTable, sv, deltaT*0.5, deltaT, nLocal, GLO1_rCa);
/*
	for (i = 0; i<nLocal; i++)						//add by ddkk
	{												//add by ddkk
		fprintf(fSignalTest, "%f ", rCa[i]);		//add by ddkk
	}												//add by ddkk
	fprintf(fSignalTest, "\n");						//add by ddkk
*/
	for (i=	nMax+GLO1_SAMPLESPMS; i<GLO1_FFTSIZE; i++)
	{
		GLO1_rCa[i]=0.0;
	}
	
	for(i=0;i<GLO1_FFTSIZE;i++)
	{
		GLO1_iCa[i]=0.0;
	}    
	xfft(GLO1_rCa,GLO1_iCa,GLO1_FFTSIZE,GLO1_rCa,GLO1_iCa);

    *nTrial = nMsec;

	for (k=0; k<numFreqBin; k++)
	{
		phase = GLO1_twoPI*(freqCarrier + freqOffset[k])*deltaT;
		deltaPhase = phase;					
		
		for (j=0; j<nMax;j++)
		{
			GLO1_cosPhase[j]= cos(phase);
			GLO1_sinPhase[j]= sin(phase);
/*			fprintf(fSignalTest, "%f ", cos(phase));*/
			phase += deltaPhase;
		}

        dSampleOffset = 0.0;
        lSampleOffset = 0;
		for (i=0;i<nMsec;i++)
		{
			pData = data + i*nMax + lSampleOffset;
			rIn = GLO1_rInData;
			iIn = GLO1_iInData;
			p1Data = pData;

			pCos = GLO1_cosPhase;
			pSin = GLO1_sinPhase;
			for (j=0; j<nMax;j++)
			{
				*rIn++ =  (*p1Data)  * (*pCos++);
				*iIn++ = -(*p1Data++)* (*pSin++);
				//fprintf(fSignalTest, "%f ", rInData[j]);
				//fprintf(fSignalTest, "%f ", pData[j]);
			}
			for (j=	nMax; j<GLO1_FFTSIZE; j++)
			{
				*rIn++ = *iIn++ = 0.0;
			}

			ixfft(GLO1_rInData,GLO1_iInData,GLO1_FFTSIZE,GLO1_rInData,GLO1_iInData);

			pROut = GLO1_rInData;
			pIOut = GLO1_iInData;
			pRCa = GLO1_rCa;
			pICa = GLO1_iCa;
			pRProduct= GLO1_rProduct;
			pIProduct= GLO1_iProduct;
			for (j=0; j<GLO1_FFTSIZE;j++)
			{
				*pRProduct++ = (*pROut) * (*pRCa) - (*pIOut) * (*pICa);
				*pIProduct++ = (*pIOut++) * (*pRCa++) + (*pROut++) * (*pICa++);
			}

			ixfft(GLO1_rProduct,GLO1_iProduct,GLO1_FFTSIZE,GLO1_rProduct,GLO1_iProduct); //FFT CALL

            pROut = GLO1_rProduct;
			pIOut = GLO1_iProduct;
			pCorr = GLO1_corr[k];
			for (j=0; j<GLO1_SAMPLESPMS;j++)
			{
				abs2Product = (*pROut) * (*pROut++) + (*pIOut) * (*pIOut++);
				*pCorr++ += abs2Product;
			}
            dSampleOffset += sampleOffsetPerMilliSecond;
            if(dSampleOffset>0.5)
            {
                dSampleOffset-=1.0;
                lSampleOffset++;
            }
            else if (dSampleOffset<-0.5)
            {
                dSampleOffset+=1.0;
                lSampleOffset--;
            }
		}
	}
#ifdef GLO1_ACQ_DEBUG
	//if (sv == 8) {
		for (k = 0; k < numFreqBin; k++)								//add by ddkk
		{															//add by ddkk
			for (i = 0; i < nMax; i++)								//add by ddkk
			{														//add by ddkk
				fprintf(fCorrTest, "%f ", GLO1_corr[k][i]);				//add by ddkk
			}														//add by ddkk
			fprintf(fCorrTest, "\n");								//add by ddkk
		}															//add by ddkk
		fclose(fCorrTest);											//add by ddkk
	// 	fclose(fSignalTest);										//add by ddkk
	//}
#endif

	peak = 0.0;	
	index = 0;
	for (i=0;i<numFreqBin;i++)
	{
		pCorr = GLO1_corr[i];
		for (j=0; j<GLO1_SAMPLESPMS;j++)
		{
			abs2Product = *pCorr++;
			if(abs2Product>peak)
			{
				peak = abs2Product;
				index = j;
				*doppler = freqOffset[i];
			}
		}
	}
	
	*Rmax = peak;

	if (peak>(*threshold))
	{
		*tau = (double)(index)/GLO1_SAMPLESPMS*1e-3;  
		
		return(GLO1_DETECTED);
	}
	else if (peak>*threshold*0.8)
	{
		return(GLO1_DETECTING);
	}
	else
	{
		return(GLO1_UNDETECTED);
	}
}
