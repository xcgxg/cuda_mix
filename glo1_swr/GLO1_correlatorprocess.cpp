// Released on July 9, 2004
#include "../stdafx.h"
// 系统头文件
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <emmintrin.h> 

#include <windows.h>

#define NLASM

// 用户头文件
#include "GLO1_gpsstruct.h"
#include "GLO1_gpsconst.h"
#include "GLO1_cagen.h"
#include "GLO1_correlatorprocess.h"
#include "GLO1_cpudetect.h"
//#include "TimeCounterEx.h"
#include "GLO1_gp2021.h"
#include "GLO1_cpudetect.h"

#define GLO1_CAExtTableResolution (int)32
#define GLO1_CAExtTableShift      (int)3  

#define VIRTUAL_CHAR __m128i

#define GLO1_ASM_ALONE 1

extern "C" void plsToXmm5(long*);
extern "C" void xmm5ToPls(long*);
extern "C" void xorXmm5();
extern "C" void xmm5_xmm4(__m128i*);

// 存储载波相位的sin, cos表，全局变量，本地初始化
char GLO1_sinTable[GLO1_CARRIER_TABLE_LENGTH];
char GLO1_cosTable[GLO1_CARRIER_TABLE_LENGTH];

int GLO1_ADNumber;

// 外部定义的全局变量
extern GLO1_CHANNEL GLO1_chan[GLO1_chmax + 1];			// 信道的状态变量数组
extern GLO1_CORRELATOR GLO1_correlator[GLO1_chmax + 1];	// 相关器通道状态变量数组
extern long GLO1_correlatorDataLength;		// 某次缓存的输入数据个数＝采样率*中断间隔
										// 如5MHz*0.5ms = 2500
extern long	GLO1_globalBufLength[GLO1_chmax+1];	// 用于保存每个通道缓存的数据长度，需初始化
extern double *GLO1_buffer[GLO1_chmax+1];		// 用于保存每个通道的缓存数据，需初始化分配空间
										// 相关器缓存的数字化数据数组，动态分配，最大长度10000
extern long GLO1_TIC_RT_CNTR;				// 测量中断的实时计数器: 0～测量中断的最大计数值-1
extern long GLO1_TIC_CNTR;					// 测量中断的最大计数值
extern double GLO1_TIC_CNTR_RES;
extern double GLO1_TIC_CNTR_DBL;
extern long GLO1_TIC_OCCUR_FLAG;				// 测量中断标志，在相关器仿真程序中被置1
										// 在测量中断服务程序中被复位为0
extern char** GLO1_caTable;					// CA码表：37×1023，取值为1,-1 <-> 1,0
										// CA码表在CAGEN程序中被分配和初始化
char** GLO1_CAExtendedTable;					// 扩展的CA码表，37×1023字节，动态分配
										// 存储的是EPL支路的CA码，表示用0,1
										// E支路在MSB，L支路在LSB，char型数最多表示8个支路
VIRTUAL_CHAR**** GLO1_productTable;					// 存储输入信号×载波×CA码的结果
										

extern int GLO1_gTotAvailCore;

extern float GLO1_CARRIER_FREQ;
extern double GLO1_deltaPhaseConst;
extern double GLO1_deltaCodePhaseConst;
extern long GLO1_DETECTSIZE;

#ifndef REAL_TIME

void GLO1_initCorrelator()
{
	double	phase0, deltaPhase,phase;
	double  amplitude, value, absValue, signValue;
	long	i;
	
	// generate two tables to look up for carrier phase
	phase0 = GLO1_twoPI / (2 * GLO1_CARRIER_TABLE_LENGTH);
	deltaPhase = GLO1_twoPI / GLO1_CARRIER_TABLE_LENGTH;
	amplitude = GLO1_CARRIER_TABLE_LENGTH / 2.0;
	for (i = 0; i<GLO1_CARRIER_TABLE_LENGTH; i++)
	{
		phase = phase0 + deltaPhase*i;
		value = sin(phase);
		absValue = fabs(value);
		signValue = value>=0 ? 1 : -1;
		GLO1_sinTable[i] = (char)(signValue*ceil(absValue*amplitude));
		value = cos(phase);
		absValue = fabs(value);
		signValue = value>=0 ? 1 : -1;
		GLO1_cosTable[i] = (char)(signValue*ceil(absValue*amplitude));
	}

	// generate CA code table to look up
	GLO1_caTable = GLO1_caGen();
	GLO1_CAExtendedTable = GLO1_caGenExtended(GLO1_caTable);
	GLO1_genProductTable();

	for (i=0;i<=GLO1_chmax;i++)
	{
        long long carr = (long long)(GLO1_CARRIER_FREQ*GLO1_deltaPhaseConst + 0.5);
		GLO1_correlator[i].fCarr = (long)(carr & 0xffffffffUL);
		GLO1_correlator[i].fCode = (__int64)(1.0*GLO1_deltaCodePhaseConst+0.5);
		GLO1_correlator[i].state = GLO1_CHN_OFF;
		GLO1_correlator[i].carrierCycle = 0;
		GLO1_correlator[i].ready = 1;			
	}

	for (i=0;i<=GLO1_chmax;i++)
	{
		GLO1_globalBufLength[i] = 0;	
		GLO1_buffer[i] = (double *) _aligned_malloc((GLO1_DETECTSIZE+30000)*sizeof(double),sizeof(double));
		if(GLO1_buffer[i]==NULL)
		{
			perror(" memory allocation error");
			exit(0);
		}
	}

	GLO1_CPUDetection();
}

/************************************************************************/
/*
先把相关器.ready置1，通道.state设为捕获，使能进到ch_acq里，只有在ch_acq里捕获到了，
才把相关器.state设为CHN_oN，把相关器.ready置0，通道.state设牵引
这样相关器才能运行，进行积分清除，到了1毫秒，就把相关器.ready设为1，这样才能在下次进到通道.state选择时进到ch_pull_in里
*/
/************************************************************************/

void GLO1_correlatorChannelProcess(char* data,	// data array 
			long dataLength,				// data length 每次进0.5毫秒的数据			
			char ch)						// channel number	
{
	GLO1_CORRELATOR* pCorr;
	pCorr = &GLO1_correlator[ch];
	if (pCorr->state != GLO1_CHN_ON)
		return;
	__int64	tau;
	register long	phaseLocal;

	__m128i	s128;
	long*	pls128=(long*)&s128;

	long	deltaPhase;
	__int64	deltaCodePhase;
	long	counter;
	char*	localCaTable;
	long	ep_1ms;
	long	ep_20ms;
	long	carrierCycle;
	__m128i*	pChar=0;

	long*	pTau = (long *)&tau+1;
	char*	pData = data;

	// choose a CA code Table according to the PRN number
	localCaTable = GLO1_CAExtendedTable[pCorr->sv-1];
	phaseLocal = pCorr->phaseLocal;	
	deltaPhase = pCorr->fCarr;		

	carrierCycle = pCorr->carrierCycle;

	tau = pCorr->tau;		

	deltaCodePhase = pCorr->fCode;	
	*(pls128) = (pCorr->i_early);		
	*(pls128+1) = (pCorr->q_early);		
	*(pls128+2) = (pCorr->i_prompt);	
	*(pls128+3) = (pCorr->q_prompt);	

#ifdef NLASM//NING LUO ASM
#if GLO1_ASM_ALONE
	plsToXmm5(pls128);
#else
	__asm
	{
		mov eax, pls128
			movdqa      xmm5, xmmword ptr[eax]
	}
#endif
#endif

	for (counter=0; counter<dataLength; counter++)
	{    
		tau += deltaCodePhase;
		//if (*pTau>=261888)
		if (*pTau>=130816)//511*256=511<<8
		{
			//*pTau -=261888;	
			*pTau -=130816;
			#ifdef NLASM         
#if GLO1_ASM_ALONE
			xmm5ToPls(pls128);
#else
			__asm
			{
				mov eax, pls128
					// *(xmmword*)eax
					// b - 8
					// w - 16
					// d/l - 32
					// q - 64
					movdqa      xmmword ptr[eax], xmm5
			}
#endif
			#endif
			pCorr->latchedSER = *(pls128);
			pCorr->latchedSEI = *(pls128+1);
			pCorr->latchedSPR = *(pls128+2);
			pCorr->latchedSPI = *(pls128+3);

			#ifdef NLASM
#if GLO1_ASM_ALONE
			xorXmm5();
#else

			__asm
			{
				xorps  xmm5, xmm5
			}
#endif
			#else
			*psR=*(psR+1)=*(psR+2)=0;
			*psI=*(psI+1)=*(psI+2)=0;
			#endif


			ep_1ms = (pCorr->epochCounterCheck & 0x001F)+1;
			//ep_20ms = (pCorr->epochCounterCheck & 0x3F00)>>8;
			ep_20ms = (pCorr->epochCounterCheck & 0x7F00)>>8;//100= 110 0100   111 1111  //0x3F00
			/*if (ep_1ms>=20)
			{
				ep_1ms = 0;
				ep_20ms++;
				if (ep_20ms>=50)
				{
					ep_20ms = 0;
				}
			}*/
			if (ep_1ms>=10)
			{
				ep_1ms = 0;
				ep_20ms++;
				if (ep_20ms>=100)
				{
					ep_20ms = 0;
				}
			}
			pCorr->epochCounterCheck = (ep_20ms<<8) + ep_1ms;
			pCorr->ready = 1;
		}
		phaseLocal += deltaPhase;
		carrierCycle += (phaseLocal>>30);	
		phaseLocal &= 0x3FFFFFFF;

		if (GLO1_ADNumber == 2)
		{
			pChar = GLO1_productTable[data[counter]][phaseLocal >> 26][localCaTable[*pTau >> 3]];
		}
		else if (GLO1_ADNumber == 4) {
			int dd = 0;
			switch (data[counter]) {
			case -3:	dd = 0;	break;
			case -1:	dd = 1;	break;
			case 1:		dd = 2;	break;
			case 3:		dd = 3;	break;
			}
			pChar = GLO1_productTable[dd][phaseLocal >> 26][localCaTable[*pTau >> 3]];
		}
		else if (GLO1_ADNumber == 16)
		{
			int dd2 = 0;
			dd2 = data[counter] + 8;
			/*switch (data[counter])
			{
			case 0: 	dd = 1;	break;
			case 1: 	dd = 2;	break;
			case 2:		dd = 3;	break;
			case 3:		dd = 4;	break;
			case 4: 	dd = 5;	break;
			case 6: 	dd = 7;	break;
			case 7:		dd = 8;	break;
			*/
			pChar = GLO1_productTable[dd2][phaseLocal >> 26][localCaTable[*pTau >> 3]];
		}

		
#ifdef NLASM
#if GLO1_ASM_ALONE
		xmm5_xmm4(pChar);
#else

		__asm
		{
			mov         eax, dword ptr[pChar]
				movdqa      xmm4, xmmword ptr[eax]
				psubd       xmm5, xmm4	// xmm5 -= xmm4
		}
#endif
#else
      sI = _mm_sub_epi32 (sI,*pChar);
      sR = _mm_add_epi32 (sR,*(pChar+1));


#endif

	}	
#ifdef NLASM
#if GLO1_ASM_ALONE

	xmm5ToPls(pls128);
#else

	__asm
	{
		mov eax, pls128
			movdqa      xmmword ptr[eax], xmm5
	}
#endif
#endif
	// 还原数据

	pCorr->i_early = *(pls128);
	pCorr->q_early = *(pls128+1);
	pCorr->i_prompt = *(pls128+2);
	pCorr->q_prompt = *(pls128+3);

	pCorr->carrierCycle = carrierCycle;

	pCorr->tau = tau;
	pCorr->phaseLocal = phaseLocal;
}

// The following session supports multithreaded correlator
long GLO1_lgDataLength;
char* GLO1_cgData;
HANDLE GLO1_chn_start_event[GLO1_chmax + 1];
HANDLE GLO1_chn_finish_event[GLO1_chmax + 1];
HANDLE GLO1_chn_thread[GLO1_chmax + 1];
int GLO1_chn_id[GLO1_chmax + 1];
void GLO1_CorrProc(LPVOID lpData)
{
	int i = *(int*)lpData;
	while(TRUE)
	{
		WaitForSingleObject(GLO1_chn_start_event[i], INFINITE);
		GLO1_correlatorChannelProcess(GLO1_cgData, GLO1_lgDataLength, i);

		SetEvent(GLO1_chn_finish_event[i]);
	}
}

bool GLO1_bFirst = true;
void GLO1_correlatorProcess(char* data,
			long dataLength)	
{
	
	char i;

	GLO1_correlatorDataLength = dataLength;

	GLO1_cgData = data;
	GLO1_lgDataLength = dataLength;

   if (GLO1_gTotAvailCore > 1) 
   {
	   if (GLO1_bFirst)
      {
		  for (i=0; i<=GLO1_chmax; i++)
         {
			  GLO1_chn_id[i] = i;
			  GLO1_chn_finish_event[i] = CreateEvent(NULL, FALSE, FALSE, NULL);
			  GLO1_chn_start_event[i] = CreateEvent(NULL, FALSE, TRUE, NULL);
			
			  GLO1_chn_thread[i] =
			CreateThread((LPSECURITY_ATTRIBUTES) NULL,
            0,
			(LPTHREAD_START_ROUTINE)(GLO1_CorrProc),
			(LPVOID)&GLO1_chn_id[i],
            0,
            NULL );

			  SetThreadPriority(GLO1_chn_thread[i], THREAD_PRIORITY_NORMAL);
         }
		  GLO1_bFirst = false;
      }
      else
      {
         for (i=0; i<=GLO1_chmax; i++)
			 SetEvent(GLO1_chn_start_event[i]);
      }
	  WaitForMultipleObjects(GLO1_chmax + 1, GLO1_chn_finish_event, TRUE, INFINITE);
   }
   else
   {
      for (i=0;i<=GLO1_chmax;i++)
	   {
		  GLO1_correlatorChannelProcess(GLO1_cgData, GLO1_lgDataLength, i);
	   }
   }
	GLO1_TIC_RT_CNTR += dataLength;
	if (GLO1_TIC_RT_CNTR == (GLO1_TIC_CNTR+1))
	{
		for (i=0;i<=GLO1_chmax;i++)
		{
			GLO1_correlator[i].carrierCycleLatch = GLO1_correlator[i].carrierCycle;
			GLO1_correlator[i].carrierCycle = 0;
			GLO1_correlator[i].phaseLocalLatch = GLO1_correlator[i].phaseLocal;
			GLO1_correlator[i].tauLatch = GLO1_correlator[i].tau;
			GLO1_correlator[i].epochCounter = GLO1_correlator[i].epochCounterCheck;
		}
		GLO1_TIC_RT_CNTR = 0;
		GLO1_TIC_OCCUR_FLAG = 1;	
	}
}

void GLO1_shutCorrelator()
{
	long i;

   if(GLO1_gTotAvailCore > 1)
   {
      for (i=0; i<=GLO1_chmax+1; i++)
		  TerminateThread(GLO1_chn_thread[i], 0);
   }
   GLO1_freeCaTable(GLO1_caTable);
   GLO1_freeCAExtendedTable(GLO1_CAExtendedTable);
   GLO1_freeProductTable();

	for (i=0;i<=GLO1_chmax;i++)
	{
		_aligned_free(GLO1_buffer[i]);	
	}

}	

char** GLO1_caGenExtended(char** localCATable)
{
	char **GLO1_CAExtendedTable;
	int i;
	long deltaPhase;
	long phaseP, phaseE, phaseL;
	char eValue, pValue, lValue; 

	GLO1_CAExtendedTable=(char**)malloc(sizeof(char*)*37);
	if(GLO1_CAExtendedTable==NULL)
	{
		printf("memory allocation error in CA extended table generation");
		exit(0);
	}							
	//CAExtendedTable[0]=(char *)malloc(1023*CAExtTableResolution*37*sizeof(char));
	GLO1_CAExtendedTable[0]=(char *)malloc(511*GLO1_CAExtTableResolution*37*sizeof(char));
	if (GLO1_CAExtendedTable[0]==NULL) 
	{
		printf("memory allocation error in CA extended table generation");
		exit(0);
	}
	for (i=1;i<37;i++)
	{
     // CAExtendedTable[i]=CAExtendedTable[i-1]+1023*CAExtTableResolution;
	  GLO1_CAExtendedTable[i]=GLO1_CAExtendedTable[i-1]+511*GLO1_CAExtTableResolution;
	}
	deltaPhase = (long)(GLO1_DLLdT*0.5*GLO1_CAExtTableResolution + 0.5);
	for (i=0; i<37; i++)
	{
		//for (phaseP=0; phaseP<1023*CAExtTableResolution; phaseP++)
		for (phaseP=0; phaseP<511*GLO1_CAExtTableResolution; phaseP++)
		{ 
			phaseE = phaseP+deltaPhase;
			if(phaseE>=511*GLO1_CAExtTableResolution)
			{
				phaseE-=511*GLO1_CAExtTableResolution;
			}
			eValue = localCATable[i][phaseE>>5];
			pValue = localCATable[i][phaseP>>5];
			phaseL = phaseP-deltaPhase;
			if(phaseL<0)
			{
				phaseL+=511*GLO1_CAExtTableResolution;
			}
			lValue = localCATable[i][phaseL>>5];
			GLO1_CAExtendedTable[i][phaseP] = 0;
			if(eValue>0)
			{
				GLO1_CAExtendedTable[i][phaseP]++;
			}
			GLO1_CAExtendedTable[i][phaseP]<<=1;
			if(pValue>0)
			{
				GLO1_CAExtendedTable[i][phaseP]++;
			}
			GLO1_CAExtendedTable[i][phaseP]<<=1;
			if(lValue>0)
			{
				GLO1_CAExtendedTable[i][phaseP]++;
			}
		
		}
	}

	return GLO1_CAExtendedTable;
}


void GLO1_freeCAExtendedTable(char** caExtendedTable)
{
   free(caExtendedTable[0]);
	free(caExtendedTable);
}

void GLO1_genProductTable()
{
	long int i,j,k,l;
	char *ADOutput = new char[GLO1_ADNumber];
	char code;
    long EPL2E_LP[8]={0,2,0,2,-2,0,-2,0};
	if (GLO1_ADNumber == 2)  //SetQB(1.0),则GLO1_ADNumber=2；SetQB(2.0),则GLO1_ADNumber=4
	{
		ADOutput[0] = 0;
		ADOutput[1] = 1;
	}
	else if (GLO1_ADNumber == 4){
		ADOutput[0] = -3;
		ADOutput[1] = -1;
		ADOutput[2] = 1;
		ADOutput[3] = 3;
	}
	else if (GLO1_ADNumber == 16)
	{
		ADOutput[0] = -8;
		ADOutput[1] = -7;
		ADOutput[2] = -6;
		ADOutput[3] = -5;
		ADOutput[4] = -4;
		ADOutput[5] = -3;
		ADOutput[6] = -2;
		ADOutput[7] = -1;
		ADOutput[8] = 0;
		ADOutput[9] = 1;
		ADOutput[10] = 2;
		ADOutput[11] = 3;
		ADOutput[12] = 4;
		ADOutput[13] = 5;
		ADOutput[14] = 6;
		ADOutput[15] = 7;
	}

	GLO1_productTable = (VIRTUAL_CHAR****) malloc(sizeof(VIRTUAL_CHAR***)*GLO1_ADNumber);
    // __m128i ***bigTable = (__m128i***) productTable;
	if(GLO1_productTable==NULL)
	{
		perror("Product table allocation error 1");
		exit(0);
	}
	for (i=0; i<GLO1_ADNumber; i++)
	{
		GLO1_productTable[i] = (VIRTUAL_CHAR***)malloc(sizeof(VIRTUAL_CHAR**)*GLO1_CARRIER_TABLE_LENGTH);
		if(GLO1_productTable[i]==NULL)
		{
			perror("Product table allocation error 2");
			exit(0);
		}
		for (j = 0; j<GLO1_CARRIER_TABLE_LENGTH; j++)
		{
			GLO1_productTable[i][j] = (VIRTUAL_CHAR**)malloc(sizeof(VIRTUAL_CHAR*)*GLO1_CAExtOutput);
			if(GLO1_productTable[i][j]==NULL)
			{
				perror("Product table allocation error 3");
				exit(0);
			}
			for (k = 0; k<GLO1_CAExtOutput; k++)
			{
				GLO1_productTable[i][j][k]=(VIRTUAL_CHAR*)_aligned_malloc(sizeof(VIRTUAL_CHAR),16);
				if(GLO1_productTable[i][j][k]==NULL)
				{
					perror("Product table allocation error 4");
					exit(0);
				}
				code=EPL2E_LP[k];
				long* pLong=(long*)GLO1_productTable[i][j][k];
				*pLong++  = ADOutput[i]*GLO1_cosTable[j]*code; 
				*pLong++  =-ADOutput[i]*GLO1_sinTable[j]*code;
				code = (k<<30)>=0 ? 1 : -1;
				*pLong++  = ADOutput[i]*GLO1_cosTable[j]*code; 
				*pLong++  =-ADOutput[i]*GLO1_sinTable[j]*code;
                /*s-tatic TCHAR buf[130];
                pLong = (long*)productTable[i][j][k];
                wsprintf(buf, TEXT("productTable[%d][%d][%d] = %d, %d, %d, %d.\n"),
                    i, j, k, pLong[0], pLong[1], pLong[2], pLong[3]);
                OutputDebugString(buf);*/
			}
		}
	}
	delete []ADOutput;
}

void GLO1_freeProductTable()
{
	int i,j,k;
	for(i=0; i<GLO1_ADNumber; i++)
	{
		for (j = 0; j<GLO1_CARRIER_TABLE_LENGTH; j++)
		{
			for (k = 0; k<GLO1_CAExtOutput; k++)
			{
				_aligned_free(GLO1_productTable[i][j][k]);
			}
			free(GLO1_productTable[i][j]);
		}
		free(GLO1_productTable[i]);
	}
	free(GLO1_productTable);
}
#endif