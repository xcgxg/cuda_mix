// Released on July 9, 2004
#include "../stdafx.h"
#include <math.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <iostream>
#include "BD3_CAcode.h"
#include "BD3_gpsconst.h"
#include "BD3_acqCA.h"
#include "../receiver/fft.h"

extern long BD3_FFTSIZE;
extern double BD3_SAMPLING_FREQ;
extern long BD3_SAMPLESPMS;
extern double BD3_ACQ_SAMPLING_FREQ;

float	**BD3_corr;
float	*BD3_rCa, *BD3_iCa;
float	*BD3_rInData, *BD3_iInData;
float	*BD3_rProduct, *BD3_iProduct;
float	*BD3_cosPhase, *BD3_sinPhase;
size_t	*BD3_chipIndex;
// 全局数组分配
void BD3_initSignalSearch()
{
	long i;
	long numFreqBin;
	long nMax,nSample;
	double	chipPhase = 0.0;
	double	samplingRatio = BD3_SAMPLING_FREQ / BD3_ACQ_SAMPLING_FREQ;

	nMax = BD3_FFTSIZE;
	inifft(nMax);
	nSample = (long)(long long)(BD3_SAMPLING_FREQ*2e-3 + 0.5);

	numFreqBin = (long)(long long)((BD3_MAXF - BD3_MINF) / BD3_DELTF + 1.5);
	if ((BD3_corr = (float**)malloc(numFreqBin*sizeof(float*))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	for (i=0;i<numFreqBin;i++)
	{
		if ((BD3_corr[i] = (float*)malloc(nMax*sizeof(float))) == NULL)//nMax = FFTSIZE;
		{
			puts("memory allocation error in acqca.cpp");
			exit(0);
		}
	}
    
	if ((BD3_rCa = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	
	if ((BD3_iCa = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	
	if ((BD3_cosPhase = (float*)malloc(nSample*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((BD3_sinPhase = (float*)malloc(nSample*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	
	if ((BD3_rInData = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((BD3_iInData = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	
	if ((BD3_rProduct = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((BD3_iProduct = (float*)malloc(nMax*sizeof(float))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	if ((BD3_chipIndex = (size_t*)malloc(nMax*sizeof(size_t))) == NULL)
	{
		puts("memory allocation error in acqca.cpp");
		exit(0);
	}
	for (i=0;i<nMax;i++)
	{
		BD3_chipIndex[i] = (size_t)chipPhase;
		chipPhase +=samplingRatio;
	}
}

void BD3_freeSignalSearch()
{
	long i;
	long numFreqBin;

	numFreqBin = (long)(long long)((BD3_MAXF - BD3_MINF) / BD3_DELTF + 1.5);

	for (i=0;i<numFreqBin;i++)
	{
		free(BD3_corr[i]);
	}
	free(BD3_corr);
	free(BD3_rCa);
	free(BD3_iCa);
	free(BD3_sinPhase);
	free(BD3_cosPhase);
	free(BD3_rInData);
	free(BD3_iInData);
	free(BD3_rProduct);
	free(BD3_iProduct);
	free(BD3_chipIndex);
	freeFFT();
}


int BD3_acqCA(double *data,			// input data sequence                  
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
	double	samplingRatio = BD3_SAMPLING_FREQ / BD3_ACQ_SAMPLING_FREQ;
 	FILE	*fCorrTest;
	double  r_result, i_result, result0, result1;
	long    accurate_flag = 0;

 	FILE	*fSignalTest;						//add by ddkk
	//fCorrTest = fopen("D:\\corr.dat","w");						//add by ddkk
 	fSignalTest = fopen("signal.dat","w");					//add by ddkk

	nMax = BD3_FFTSIZE;//nMax = (long)(SAMPLING_FREQ*1e-3+0.5);//4750 //changed by ddkk		
	nLocal = BD3_FFTSIZE / 2;//nLocal = (long)(SAMPLING_FREQ*2e-3+0.5);  //9500 //changed by ddkk 
	nSample = (long)(long long)(BD3_SAMPLING_FREQ*1e-3 + 0.5);//1ms的采样点数
	nMsec= num/nSample;			               
	sampleOffsetPerMilliSecond = BD3_SAMPLING_FREQ*1e-3 - nSample;

	deltaT=1/fs; 

	for (j=0; j<numFreqBin; j++)
	{
		pCorr = BD3_corr[j];
		for (i=0; i<nMax; i++) *pCorr++ = 0.0;//相关峰值清零
	}
	
	BD3_genCaTimeSequence(caTable, sv, deltaT*0.5, deltaT, nLocal, BD3_rCa);//2
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
	*/	for (i = nLocal; i < BD3_FFTSIZE; i++)
	{
		BD3_rCa[i] = 0.0;//本地伪码实部补零至FFTSIZE点，
	}
	
	for (i = 0; i<BD3_FFTSIZE; i++)
	{
		BD3_iCa[i] = 0.0;//本地伪码虚部初始化为0
	}    
	xfft(BD3_rCa, BD3_iCa, BD3_FFTSIZE, BD3_rCa, BD3_iCa);//计算本地伪码频谱

    *nTrial = nMsec;

	phase = BD3_twoPI*(freqCarrier + freqOffset[0]) / BD3_SAMPLING_FREQ;
	deltaPhase = phase;					
	for (j=0; j<nSample*2;j++)
	{
		BD3_cosPhase[j] = cos(phase);//产生0.5ms本地同相和正交载波
		BD3_sinPhase[j] = sin(phase);
		/*fprintf(fSignalTest, "%f ", cos(phase));*/
		phase += deltaPhase;
	}

	dSampleOffset = 0.0;
    lSampleOffset = 0;
	for (i=0;i<nMsec-1;i++)
	{
		pData = data + i*nSample + lSampleOffset;
		rIn = BD3_rInData;
		iIn = BD3_iInData;
		p1Data = pData;
		
		pCos = BD3_cosPhase;
		pSin = BD3_sinPhase;
		for (j=0; j<nMax;j++)
		{
		/*	index = (size_t)(j*samplingRatio);*/
			*rIn++ = (p1Data[BD3_chipIndex[j]]) * (pCos[BD3_chipIndex[j]]);
			*iIn++ = (p1Data[BD3_chipIndex[j]]) * (pSin[BD3_chipIndex[j]]);//*iIn++ = -(p1Data[chipIndex[j]]) * (pSin[chipIndex[j]]);
			//fprintf(fSignalTest, "%f ", rInData[j]);
			//fprintf(fSignalTest, "%f ", p1Data[(size_t)(j*samplingRatio)]);
		}
// 		for (j=	nMax; j<FFTSIZE; j++)
// 		{
// 			*rIn++ = *iIn++ = 0.0;
// 		}
			
		xfft(BD3_rInData, BD3_iInData, BD3_FFTSIZE, BD3_rInData, BD3_iInData);//ixfft(rInData,iInData,FFTSIZE,rInData,iInData);

		for (k=0; k<numFreqBin; k++)
		{
			pROut = BD3_rInData;
			pIOut = BD3_iInData;
			pRCa = BD3_rCa;
			pICa = BD3_iCa;
			pRProduct = BD3_rProduct;
			pIProduct = BD3_iProduct;
			index = 0;
			for (j = BD3_FFTSIZE - k; j<2 * BD3_FFTSIZE - k; j++)
			{
				index = fmod((long double)j, (long double)BD3_FFTSIZE);
				*pRProduct++ = (pROut[index]) * (*pRCa) + (pIOut[index]) * (*pICa);
				*pIProduct++ = (pIOut[index]) * (*pRCa++) - (pROut[index]) * (*pICa++);
			}
			
			ixfft(BD3_rProduct, BD3_iProduct, BD3_FFTSIZE, BD3_rProduct, BD3_iProduct); //FFT CALL
			
			pROut = BD3_rProduct;
			pIOut = BD3_iProduct;
			pCorr = BD3_corr[k];
			for (j=0; j<nLocal;j++)	//	for (j=nLocal; j<nMax;j++)
			{
				abs2Product = (*pROut) * (*pROut++) + (*pIOut) * (*pIOut++);
				*pCorr++ += abs2Product;
//				fprintf(fCorrTest,"%f ",abs2Product);
			}
//			fprintf(fCorrTest,"\n");
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
	//if(sv==11)
	//{
		//int a = 6;

		//char path[100];
		//sprintf(path, "E:\\cy\\receiver\\test_result\\matlab test\\matlab test\\BD2B3\\acquisition_result\\1bit\\corr%d.dat", sv);
		//fCorrTest = fopen(path,"w");
 	//	for (k=0; k<numFreqBin; k++)								//add by ddkk
  //		{															//add by ddkk
  //			for (j = 0; j <nLocal; j++)								//add by ddkk
  //			{														//add by ddkk
  //				fprintf(fCorrTest,"%f ",corr[k][j]);				//add by ddkk
 	//		}														//add by ddkk
  //			fprintf(fCorrTest,"\n");								//add by ddkk
  //		}					
 	//									//add by ddkk
 	//fclose(fCorrTest);											//add by ddkk
 	//fclose(fSignalTest);										//add by ddkk

	//}
	peak = 0.0;	
	index = 0;
	for (i=0;i<numFreqBin;i++)
	{
		pCorr = BD3_corr[i];
		for (j=0; j<nLocal;j++)
		{
			abs2Product = *pCorr++;
			if(abs2Product>peak)
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
		BD3_genCaTimeSequence(caTable, sv, deltaT*0.5, deltaT, nLocal, BD3_rCa);
		r_result = 0.0;
		i_result = 0.0;
		phase = BD3_twoPI*(freqCarrier + *doppler) / BD3_SAMPLING_FREQ;
		deltaPhase = phase;					
		for (j=0; j<nSample;j++)
		{
			BD3_cosPhase[j] = cos(phase);
			BD3_sinPhase[j] = sin(phase);
			/*fprintf(fSignalTest, "%f ", cos(phase));*/
			phase += deltaPhase;
		}
		pData = data + index;//data + nSample - index;
		for (j=0; j<nLocal;j++)
		{
			r_result += (pData[BD3_chipIndex[j]]) * (BD3_cosPhase[BD3_chipIndex[j]]) * BD3_rCa[j];
			i_result += (pData[BD3_chipIndex[j]]) * (BD3_sinPhase[BD3_chipIndex[j]]) * BD3_rCa[j];
		}
		result0 = r_result*r_result + i_result * i_result;

		while(1)
		{
			r_result = 0.0;
			i_result = 0.0;
			if (accurate_flag == 0)
			{
				phase = BD3_twoPI*(freqCarrier + *doppler + BD3_DELTF / 2) / BD3_SAMPLING_FREQ;
			}
			else if(accurate_flag == 1)
			{
				phase = BD3_twoPI*(freqCarrier + *doppler - BD3_DELTF / 2) / BD3_SAMPLING_FREQ;
			}
			deltaPhase = phase;					
			for (j=0; j<nSample;j++)
			{
				BD3_cosPhase[j] = cos(phase);
				BD3_sinPhase[j] = sin(phase);
				/*fprintf(fSignalTest, "%f ", cos(phase));*/
				phase += deltaPhase;
			}
			pData = data + index;
			for (j=0; j<nLocal;j++)
			{
				r_result += (pData[BD3_chipIndex[j]]) * (BD3_cosPhase[BD3_chipIndex[j]]) * BD3_rCa[j];
				i_result += (pData[BD3_chipIndex[j]]) * (BD3_sinPhase[BD3_chipIndex[j]]) * BD3_rCa[j];
			}
			result1 = r_result*r_result + i_result * i_result;

			if (result1 > result0)
			{
				if (accurate_flag == 0)
					*doppler = *doppler + BD3_DELTF / 2;
				else if(accurate_flag == 1)
					*doppler = *doppler - BD3_DELTF / 2;
				break;
			}
			accurate_flag++;
			if(accurate_flag == 2)
			{
				break;
			}
		}
	}
	/* add by ddkk, accurate search*/
	//////////////////////////////////////////////////////////////

	if (peak>(*threshold * 0.8))
	{
		*tau = 1e-3 - (double)(index) / BD3_SAMPLESPMS*1e-3;
		
		return(BD3_DETECTED);
	}
	else if (peak>*threshold*0.7)
	{
		return(BD3_DETECTING);
	}
	else
	{
		return(BD3_UNDETECTED);
	}
}
