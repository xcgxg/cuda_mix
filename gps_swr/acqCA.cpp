// Released on July 9, 2004
#include "../stdafx.h"
#include <math.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>

#include "CAcode.h"
#include "gpsconst.h"
#include "acqCA.h"
#include "../receiver/fft.h"

extern long FFTSIZE;
extern double SAMPLING_FREQ;
extern long SAMPLESPMS;

//#define ACQ_DEBUG

// 全局堆指针，用于在堆上分配全局数组
float	**corr;
float	*rCa, *iCa;
float	*rInData, *iInData;
float	*rProduct, *iProduct;
float	*cosPhase, *sinPhase;

// 全局数组分配
void initSignalSearch()
{
	long i;
	long numFreqBin;
	long nMax;

	nMax = FFTSIZE;
	inifft(nMax);

	numFreqBin = (long)((MAXF-MINF)/DELTF+1.5);	

	if ((corr = (float**) malloc(numFreqBin*sizeof(float*)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	for (i=0;i<numFreqBin;i++)
	{
		if ((corr[i] = (float*) malloc(nMax*sizeof(float)))==NULL)
		{
			puts("memory allocation error in acqca.cpp");
			exit(0);
		}
	}
	if ((rCa = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((iCa = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}

	if ((cosPhase = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((sinPhase = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((rInData = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((iInData = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((rProduct = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((iProduct = (float*) malloc(nMax*sizeof(float)))==NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
}
extern void  write_prn();

void freeSignalSearch()
{
	long i;
	long numFreqBin;

	numFreqBin = (long)((MAXF-MINF)/DELTF+1.5);				// 要搜索的频率总数
	write_prn();
	// 释放全部堆变量
	for (i=0;i<numFreqBin;i++)
	{
		free(corr[i]);
	}
	free(corr);
	free(rCa);
	free(iCa);
	free(sinPhase);
	free(cosPhase);
	free(rInData);
	free(iInData);
	free(rProduct);
	free(iProduct);
	freeFFT();
}

int acqCA(double *data,			// input data sequence       
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
#ifdef ACQ_DEBUG
 	FILE	*fCorrTest, *fSignalTest;						//add by ddkk
 	fCorrTest = fopen("K:\\corr.dat","w");						//add by ddkk
// 	fSignalTest = fopen("signal.dat","w");					//add by ddkk
#endif
	nMax = (long)(SAMPLING_FREQ*1e-3+0.5);		
    nLocal = (long)(SAMPLING_FREQ*2e-3+0.5);    
	nMsec= num/nMax;			               
    sampleOffsetPerMilliSecond = SAMPLING_FREQ*1e-3 - nMax;

	deltaT=1/fs;

	for (j=0; j<numFreqBin; j++)
	{
		pCorr = corr[j];
		for (i=0; i<nMax; i++) *pCorr++ = 0.0;
	}
	
	genCaTimeSequence(caTable,sv,deltaT*0.5,deltaT,nLocal,rCa);
/*
	for (i = 0; i<nLocal; i++)						//add by ddkk
	{												//add by ddkk
		fprintf(fSignalTest, "%f ", rCa[i]);		//add by ddkk
	}												//add by ddkk
	fprintf(fSignalTest, "\n");						//add by ddkk
*/
	for (i=	nMax+SAMPLESPMS; i<FFTSIZE; i++)
	{
		rCa[i]=0.0;
	}
	
	for(i=0;i<FFTSIZE;i++)
	{
		iCa[i]=0.0;
	}    
	xfft(rCa,iCa,FFTSIZE,rCa,iCa);

    *nTrial = nMsec;

	for (k=0; k<numFreqBin; k++)
	{
		phase = twoPI*(freqCarrier+freqOffset[k])*deltaT;
		deltaPhase = phase;					
		
		for (j=0; j<nMax;j++)
		{
			cosPhase[j]= cos(phase);
			sinPhase[j]= sin(phase);
/*			fprintf(fSignalTest, "%f ", cos(phase));*/
			phase += deltaPhase;
		}

        dSampleOffset = 0.0;
        lSampleOffset = 0;
		for (i=0;i<nMsec;i++)
		{
			pData = data + i*nMax + lSampleOffset;
			rIn = rInData;
			iIn = iInData;
			p1Data = pData;

			pCos = cosPhase;
			pSin = sinPhase;
			for (j=0; j<nMax;j++)
			{
				*rIn++ =  (*p1Data)  * (*pCos++);
				*iIn++ = -(*p1Data++)* (*pSin++);
				//fprintf(fSignalTest, "%f ", rInData[j]);
				//fprintf(fSignalTest, "%f ", pData[j]);
			}
			for (j=	nMax; j<FFTSIZE; j++)
			{
				*rIn++ = *iIn++ = 0.0;
			}

			ixfft(rInData,iInData,FFTSIZE,rInData,iInData);

			pROut = rInData;
			pIOut = iInData;
			pRCa = rCa;
			pICa = iCa;
			pRProduct= rProduct;
			pIProduct= iProduct;
			for (j=0; j<FFTSIZE;j++)
			{
				*pRProduct++ = (*pROut) * (*pRCa) - (*pIOut) * (*pICa);
				*pIProduct++ = (*pIOut++) * (*pRCa++) + (*pROut++) * (*pICa++);
			}

			ixfft(rProduct,iProduct,FFTSIZE,rProduct,iProduct); //FFT CALL

            pROut = rProduct;
			pIOut = iProduct;
			pCorr = corr[k];
			for (j=0; j<SAMPLESPMS;j++)
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
#ifdef ACQ_DEBUG
	for (k=0; k<numFreqBin; k++)								//add by ddkk
	{															//add by ddkk
		for (i = 0; i <nMax; i++)								//add by ddkk
		{														//add by ddkk
			fprintf(fCorrTest,"%f ",corr[k][i]);				//add by ddkk
		}														//add by ddkk
		fprintf(fCorrTest,"\n");								//add by ddkk
	}															//add by ddkk
	fclose(fCorrTest);											//add by ddkk
// 	fclose(fSignalTest);										//add by ddkk
#endif

	peak = 0.0;	
	index = 0;
	for (i=0;i<numFreqBin;i++)
	{
		pCorr = corr[i];
		for (j=0; j<SAMPLESPMS;j++)
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
		*tau = (double)(index)/SAMPLESPMS*1e-3;  
		
		return(DETECTED);
	}
	else if (peak>*threshold*0.8)
	{
		return(DETECTING);
	}
	else
	{
		return(UNDETECTED);
	}
}
