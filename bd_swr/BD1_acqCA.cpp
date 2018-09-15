// Released on July 9, 2004
#include "../stdafx.h"
#include <math.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>

#include "BD1_CAcode.h"
#include "BD1_gpsconst.h"
#include "BD1_acqCA.h"
#include "../receiver/fft.h"

extern long BD1_FFTSIZE;
extern double BD1_SAMPLING_FREQ;
extern long BD1_SAMPLESPMS;
extern double BD1_ACQ_SAMPLING_FREQ;

float	**BD1_corr;
float	*BD1_rCa, *BD1_iCa;
float	*BD1_rInData, *BD1_iInData;
float	*BD1_rProduct, *BD1_iProduct;
float	*BD1_cosPhase, *BD1_sinPhase;
size_t	*BD1_chipIndex;
// 全局数组分配
void BD1_initSignalSearch()
{
	long i;
	long numFreqBin;
	long nMax, nSample;
	double	chipPhase = 0.0;
	double	samplingRatio = BD1_SAMPLING_FREQ / BD1_ACQ_SAMPLING_FREQ;

	nMax = BD1_FFTSIZE;
	inifft(nMax);
	nSample = (long)(long long)(BD1_SAMPLING_FREQ*2e-3 + 0.5);

	numFreqBin = (long)(long long)((BD1_MAXF - BD1_MINF) / BD1_DELTF + 1.5);
	if ((BD1_corr = (float**)malloc(numFreqBin*sizeof(float*))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	for (i = 0; i<numFreqBin; i++)
	{
		if ((BD1_corr[i] = (float*)malloc(nMax*sizeof(float))) == NULL)//nMax = FFTSIZE;
		{
			puts("memory allocation error in acqca.cpp");
			exit(0);
		}
	}

	if ((BD1_rCa = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}

	if ((BD1_iCa = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}

	if ((BD1_cosPhase = (float*)malloc(nSample*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((BD1_sinPhase = (float*)malloc(nSample*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}

	if ((BD1_rInData = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((BD1_iInData = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}

	if ((BD1_rProduct = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((BD1_iProduct = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((BD1_chipIndex = (size_t*)malloc(nMax*sizeof(size_t))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	for (i = 0; i<nMax; i++)
	{
		BD1_chipIndex[i] = (size_t)chipPhase;
		chipPhase += samplingRatio;
	}
}

void BD1_freeSignalSearch()
{
	long i;
	long numFreqBin;

	numFreqBin = (long)(long long)((BD1_MAXF - BD1_MINF) / BD1_DELTF + 1.5);

	for (i = 0; i<numFreqBin; i++)
	{
		free(BD1_corr[i]);
	}
	free(BD1_corr);
	free(BD1_rCa);
	free(BD1_iCa);
	free(BD1_sinPhase);
	free(BD1_cosPhase);
	free(BD1_rInData);
	free(BD1_iInData);
	free(BD1_rProduct);
	free(BD1_iProduct);
	free(BD1_chipIndex);
	freeFFT();
}


int BD1_acqCA(double *data,			// input data sequence                  
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
	long	nMax, nLocal, nSample;
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
	double	samplingRatio = BD1_SAMPLING_FREQ / BD1_ACQ_SAMPLING_FREQ;
	// 	FILE	*fCorrTest;
	double  r_result, i_result, result0, result1;
	long    accurate_flag = 0;

	// 	FILE	*fSignalTest;						//add by ddkk
	//	fCorrTest = fopen("D:\\corr.dat","w");						//add by ddkk
	// 	fSignalTest = fopen("signal.dat","w");					//add by ddkk

	nMax = BD1_FFTSIZE;//nMax = (long)(SAMPLING_FREQ*1e-3+0.5);//4750 //changed by ddkk		
	nLocal = BD1_FFTSIZE / 2;//nLocal = (long)(SAMPLING_FREQ*2e-3+0.5);  //9500 //changed by ddkk 
	nSample = (long)(long long)(BD1_SAMPLING_FREQ*1e-3 + 0.5);
	nMsec = num / nSample;
	sampleOffsetPerMilliSecond = BD1_SAMPLING_FREQ*1e-3 - nSample;

	deltaT = 1 / fs;

	for (j = 0; j<numFreqBin; j++)
	{
		pCorr = BD1_corr[j];
		for (i = 0; i<nMax; i++) *pCorr++ = 0.0;
	}

	BD1_genCaTimeSequence(caTable, sv, deltaT*0.5, deltaT, nLocal, BD1_rCa);//2
	// 	for (i = 0; i<nLocal; i++)						//add by ddkk
	// 	{												//add by ddkk
	// 		fprintf(fSignalTest, "%f ", rCa[i]);		//add by ddkk
	// 	}												//add by ddkk
	// 	fprintf(fSignalTest, "\n");						//add by ddkk
	/*
	for (i=	nMax+SAMPLESPMS; i<FFTSIZE; i++)
	{
	rCa[i]=0.0;
	}
	*/	for (i = nLocal; i < BD1_FFTSIZE; i++)
	{
		BD1_rCa[i] = 0.0;
	}

	for (i = 0; i<BD1_FFTSIZE; i++)
	{
		BD1_iCa[i] = 0.0;
	}
	xfft(BD1_rCa, BD1_iCa, BD1_FFTSIZE, BD1_rCa, BD1_iCa);

	*nTrial = nMsec;

	phase = BD1_twoPI*(freqCarrier + freqOffset[0]) / BD1_SAMPLING_FREQ;
	deltaPhase = phase;
	for (j = 0; j<nSample * 2; j++)
	{
		BD1_cosPhase[j] = cos(phase);
		BD1_sinPhase[j] = sin(phase);
		/*fprintf(fSignalTest, "%f ", cos(phase));*/
		phase += deltaPhase;
	}

	dSampleOffset = 0.0;
	lSampleOffset = 0;
	for (i = 0; i<nMsec - 1; i++)
	{
		pData = data + i*nSample + lSampleOffset;
		rIn = BD1_rInData;
		iIn = BD1_iInData;
		p1Data = pData;

		pCos = BD1_cosPhase;
		pSin = BD1_sinPhase;
		for (j = 0; j<nMax; j++)
		{
			/*	index = (size_t)(j*samplingRatio);*/
			*rIn++ = (p1Data[BD1_chipIndex[j]]) * (pCos[BD1_chipIndex[j]]);
			*iIn++ = (p1Data[BD1_chipIndex[j]]) * (pSin[BD1_chipIndex[j]]);//*iIn++ = -(p1Data[chipIndex[j]]) * (pSin[chipIndex[j]]);
			//fprintf(fSignalTest, "%f ", rInData[j]);
			//fprintf(fSignalTest, "%f ", p1Data[(size_t)(j*samplingRatio)]);
		}
		// 		for (j=	nMax; j<FFTSIZE; j++)
		// 		{
		// 			*rIn++ = *iIn++ = 0.0;
		// 		}

		xfft(BD1_rInData, BD1_iInData, BD1_FFTSIZE, BD1_rInData, BD1_iInData);//ixfft(rInData,iInData,FFTSIZE,rInData,iInData);

		for (k = 0; k<numFreqBin; k++)
		{
			pROut = BD1_rInData;
			pIOut = BD1_iInData;
			pRCa = BD1_rCa;
			pICa = BD1_iCa;
			pRProduct = BD1_rProduct;
			pIProduct = BD1_iProduct;
			index = 0;
			for (j = BD1_FFTSIZE - k; j<2 * BD1_FFTSIZE - k; j++)
			{
				index = fmod((long double)j, (long double)BD1_FFTSIZE);
				*pRProduct++ = (pROut[index]) * (*pRCa) + (pIOut[index]) * (*pICa);
				*pIProduct++ = (pIOut[index]) * (*pRCa++) - (pROut[index]) * (*pICa++);
			}

			ixfft(BD1_rProduct, BD1_iProduct, BD1_FFTSIZE, BD1_rProduct, BD1_iProduct); //FFT CALL

			pROut = BD1_rProduct;
			pIOut = BD1_iProduct;
			pCorr = BD1_corr[k];
			for (j = 0; j<nLocal; j++)	//	for (j=nLocal; j<nMax;j++)
			{
				abs2Product = (*pROut) * (*pROut++) + (*pIOut) * (*pIOut++);
				*pCorr++ += abs2Product;
				//				fprintf(fCorrTest,"%f ",abs2Product);
			}
			//			fprintf(fCorrTest,"\n");
			dSampleOffset += sampleOffsetPerMilliSecond;
			if (dSampleOffset>0.5)
			{
				dSampleOffset -= 1.0;
				lSampleOffset++;
			}
			else if (dSampleOffset<-0.5)
			{
				dSampleOffset += 1.0;
				lSampleOffset--;
			}
		}
	}

	// 		for (k=0; k<numFreqBin; k++)								//add by ddkk
	//  		{															//add by ddkk
	//  			for (j = 0; j <nLocal; j++)								//add by ddkk
	//  			{														//add by ddkk
	//  				fprintf(fCorrTest,"%f ",corr[k][j]);				//add by ddkk
	// 			}														//add by ddkk
	//  			fprintf(fCorrTest,"\n");								//add by ddkk
	//  		}					
	// 										//add by ddkk
	// 	fclose(fCorrTest);											//add by ddkk
	// 	fclose(fSignalTest);										//add by ddkk

	peak = 0.0;
	index = 0;
	for (i = 0; i<numFreqBin; i++)
	{
		pCorr = BD1_corr[i];
		for (j = 0; j<nLocal; j++)
		{
			abs2Product = *pCorr++;
			if (abs2Product>peak)
			{
				peak = abs2Product;
				index = (long)(long long)(j*samplingRatio + 0.5);
				*doppler = freqOffset[i];
			}
		}
	}
	*Rmax = peak;

	/////////////////////////////////////////////////////////
	/* add by ddkk, accurate search*/
	if (peak > *threshold*0.8 && sv >= 6 && sv < 11)
	{
		BD1_genCaTimeSequence(caTable, sv, deltaT*0.5, deltaT, nLocal, BD1_rCa);
		r_result = 0.0;
		i_result = 0.0;
		phase = BD1_twoPI*(freqCarrier + *doppler) / BD1_SAMPLING_FREQ;
		deltaPhase = phase;
		for (j = 0; j<nSample; j++)
		{
			BD1_cosPhase[j] = cos(phase);
			BD1_sinPhase[j] = sin(phase);
			/*fprintf(fSignalTest, "%f ", cos(phase));*/
			phase += deltaPhase;
		}
		pData = data + index;//data + nSample - index;
		for (j = 0; j<nLocal; j++)
		{
			r_result += (pData[BD1_chipIndex[j]]) * (BD1_cosPhase[BD1_chipIndex[j]]) * BD1_rCa[j];
			i_result += (pData[BD1_chipIndex[j]]) * (BD1_sinPhase[BD1_chipIndex[j]]) * BD1_rCa[j];
		}
		result0 = r_result*r_result + i_result * i_result;

		while (1)
		{
			r_result = 0.0;
			i_result = 0.0;
			if (accurate_flag == 0)
			{
				phase = BD1_twoPI*(freqCarrier + *doppler + BD1_DELTF / 2) / BD1_SAMPLING_FREQ;
			}
			else if (accurate_flag == 1)
			{
				phase = BD1_twoPI*(freqCarrier + *doppler - BD1_DELTF / 2) / BD1_SAMPLING_FREQ;
			}
			deltaPhase = phase;
			for (j = 0; j<nSample; j++)
			{
				BD1_cosPhase[j] = cos(phase);
				BD1_sinPhase[j] = sin(phase);
				/*fprintf(fSignalTest, "%f ", cos(phase));*/
				phase += deltaPhase;
			}
			pData = data + index;
			for (j = 0; j<nLocal; j++)
			{
				r_result += (pData[BD1_chipIndex[j]]) * (BD1_cosPhase[BD1_chipIndex[j]]) * BD1_rCa[j];
				i_result += (pData[BD1_chipIndex[j]]) * (BD1_sinPhase[BD1_chipIndex[j]]) * BD1_rCa[j];
			}
			result1 = r_result*r_result + i_result * i_result;

			if (result1 > result0)
			{
				if (accurate_flag == 0)
					*doppler = *doppler + BD1_DELTF / 2;
				else if (accurate_flag == 1)
					*doppler = *doppler - BD1_DELTF / 2;
				break;
			}
			accurate_flag++;
			if (accurate_flag == 2)
			{
				break;
			}
		}
	}
	/* add by ddkk, accurate search*/
	//////////////////////////////////////////////////////////////

	if (peak>(*threshold * 0.8))
	{
		*tau = 1e-3 - (double)(index) / BD1_SAMPLESPMS*1e-3;

		return(BD1_DETECTED);
	}
	else if (peak>*threshold*0.7)
	{
		return(BD1_DETECTING);
	}
	else
	{
		return(BD1_UNDETECTED);
	}
}
