// Released on July 9, 2004
#include "../StdAfx.h"
// 系统头文件
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <emmintrin.h> 

#include <windows.h>

#define NLASM
#define BD1_ASM_ALONE 1


// 用户头文件
#include "BD1_gpsstruct.h"
#include "BD1_gpsconst.h"
#include "BD1_cagen.h"
#include "BD1_correlatorprocess.h"
#include "BD1_cpudetect.h"
//#include "TimeCounterEx.h"
#include "BD1_gp2021.h"
#include "BD1_cpudetect.h"

#define CAExtTableResolution (int)32

#define VIRTUAL_CHAR __m128i

extern "C" void plsToXmm5(long*);
extern "C" void xmm5ToPls(long*);
extern "C" void xorXmm5();
extern "C" void xmm5_xmm4(__m128i*);


// 存储载波相位的sin, cos表，全局变量，本地初始化
char BD1_sinTable[BD1_CARRIER_TABLE_LENGTH];
char BD1_cosTable[BD1_CARRIER_TABLE_LENGTH];

int BD1_ADNumber;

// 外部定义的全局变量
extern BD1_CHANNEL BD1_chan[BD1_chmax + 1];			// 信道的状态变量数组
extern BD1_CORRELATOR BD1_correlator[BD1_chmax + 1];	// 相关器通道状态变量数组
extern long BD1_correlatorDataLength;		// 某次缓存的输入数据个数＝采样率*中断间隔
// 如5MHz*0.5ms = 2500
extern long	BD1_globalBufLength[BD1_chmax + 1];	// 用于保存每个通道缓存的数据长度，需初始化
extern double *BD1_buffer[BD1_chmax + 1];		// 用于保存每个通道的缓存数据，需初始化分配空间
// 相关器缓存的数字化数据数组，动态分配，最大长度10000
extern long BD1_TIC_RT_CNTR;				// 测量中断的实时计数器: 0～测量中断的最大计数值-1
extern long BD1_TIC_CNTR;					// 测量中断的最大计数值
extern double BD1_TIC_CNTR_RES;
extern double BD1_TIC_CNTR_DBL;
extern long BD1_TIC_OCCUR_FLAG;				// 测量中断标志，在相关器仿真程序中被置1
// 在测量中断服务程序中被复位为0
extern char** BD1_caTable;					// CA码表：37×1023，取值为1,-1 <-> 1,0
// CA码表在CAGEN程序中被分配和初始化
char** BD1_CAExtendedTable;					// 扩展的CA码表，37×1023字节，动态分配
// 存储的是EPL支路的CA码，表示用0,1
// E支路在MSB，L支路在LSB，char型数最多表示8个支路
VIRTUAL_CHAR**** BD1_productTable;					// 存储输入信号×载波×CA码的结果


extern int BD1_gTotAvailCore;

extern float BD1_CARRIER_FREQ;
extern double BD1_deltaPhaseConst;
extern double BD1_deltaCodePhaseConst;
extern long BD1_DETECTSIZE;

#ifndef REAL_TIME

// 对相关器进行初始化
void BD1_initCorrelator()
{
	double	phase0, deltaPhase, phase;
	double  amplitude, value, absValue, signValue;
	long	i;

	// generate two tables to look up for carrier phase
	phase0 = BD1_twoPI / (2 * BD1_CARRIER_TABLE_LENGTH);
	deltaPhase = BD1_twoPI / BD1_CARRIER_TABLE_LENGTH;//pi/8
	amplitude = BD1_CARRIER_TABLE_LENGTH / 2.0;
	for (i = 0; i<BD1_CARRIER_TABLE_LENGTH; i++)
	{
		phase = phase0 + deltaPhase*i;
		value = sin(phase);
		absValue = fabs(value);
		signValue = value >= 0 ? 1 : -1;
		BD1_sinTable[i] = (char)(signValue*ceil(absValue*amplitude));
		value = cos(phase);
		absValue = fabs(value);
		signValue = value >= 0 ? 1 : -1;
		BD1_cosTable[i] = (char)(signValue*ceil(absValue*amplitude));
	}

	// generate CA code table to look up
	BD1_caTable = BD1_bdd2_caGen();
	BD1_CAExtendedTable = BD1_caGenExtended(BD1_caTable);
	BD1_genProductTable();

	// 如果采用FFT进行信号捕获，则以下的初始化过程并非必要，因为在信号捕获到之前
	// 没有必要打开相关器对应的通道。
	for (i = 0; i <= BD1_chmax; i++)
	{
		BD1_correlator[i].fCarr = (long)(long long)(BD1_CARRIER_FREQ*BD1_deltaPhaseConst + 0.5);//载波频率字

		BD1_correlator[i].fCode = (__int64)(1.0*BD1_deltaCodePhaseConst + 0.5);//码频率字

		BD1_correlator[i].state = BD1_CHN_OFF;
		BD1_correlator[i].carrierCycle = 0;
		BD1_correlator[i].ready = 1;

	}

	for (i = 0; i <= BD1_chmax; i++)
	{
		BD1_globalBufLength[i] = 0;	// 初始化数据长度为0

		BD1_buffer[i] = (double *)_aligned_malloc((BD1_DETECTSIZE + 30000)*sizeof(double), sizeof(double));

		if (BD1_buffer[i] == NULL)
		{
			perror(" memory allocation error");
			exit(0);
		}
	}

	BD1_CPUDetection();
}



// 以下函数用于仿真相关器的单个通道 correlatorChannelProcess(cgData,lgDataLength,i);

void BD1_correlatorChannelProcess(char* data,
	long dataLength,
	char ch)
{
	BD1_CORRELATOR* pCorr;
	pCorr = &BD1_correlator[ch];
	if (pCorr->state != BD1_CHN_ON)
		return;

	__int64	tau;
	register long	phaseLocal;
	__m128i	s128;
	long*	pls128 = (long*)&s128;

	long	deltaPhase;//载波相位的步进
	__int64	deltaCodePhase;//码相位的步进
	long	counter;
	char*	localCaTable;
	long	ep_1ms;
	long	ep_20ms;
	long    ep_1ms_d2;
	long    ep_2ms;
	long	carrierCycle;//载波相位整周数
	__m128i*	pChar = 0;

	long*	pTau = (long *)&tau + 1;

	// choose a CA code Table according to the PRN number
	localCaTable = BD1_CAExtendedTable[pCorr->sv - 1];
	phaseLocal = pCorr->phaseLocal;
	deltaPhase = pCorr->fCarr;			// 读取载波相位的步进  载波频率字

	carrierCycle = pCorr->carrierCycle;	// 读取载波相位的整数部分

	tau = pCorr->tau;					// 读取相关器通道的起始码相位

	deltaCodePhase = pCorr->fCode;		// 读取码相位的步进 码频率字

	*(pls128) = (pCorr->i_early);
	*(pls128 + 1) = (pCorr->q_early);
	*(pls128 + 2) = (pCorr->i_prompt);
	*(pls128 + 3) = (pCorr->q_prompt);

#ifdef NLASM//NING LUO ASM
#if BD1_ASM_ALONE
	plsToXmm5(pls128);
#else
	__asm
	{
		mov eax, pls128
			movdqa      xmm5, xmmword ptr[eax]
	}
#endif
#endif

	for (counter = 0; counter<dataLength; counter++)
	{
		// first update code phase
		tau += deltaCodePhase;
		if (*pTau >= 523776)
		{
			*pTau -= 523776;
#ifdef NLASM         
#if BD1_ASM_ALONE
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
			pCorr->latchedSEI = *(pls128 + 1);
			pCorr->latchedSPR = *(pls128 + 2);
			pCorr->latchedSPI = *(pls128 + 3);

#ifdef NLASM
#if BD1_ASM_ALONE
			xorXmm5();
#else

			__asm
			{
				xorps  xmm5, xmm5
			}
#endif
#else
#endif

			// 修正ms计数器
			if ((BD1_chan[ch].prn >= 6) && (BD1_chan[ch].prn <= 12))
			{
				ep_1ms = (pCorr->epochCounterCheck & 0x001F) + 1;
				ep_20ms = (pCorr->epochCounterCheck & 0x3F00) >> 8;// 50 = 11 0010
			}
			if ((BD1_chan[ch].prn >= 1) && (BD1_chan[ch].prn <= 5))
			{
				ep_1ms_d2 = (pCorr->epochCounterCheck & 0x001F) + 1;
				ep_2ms = (pCorr->epochCounterCheck & 0x1FF00) >> 8;	//500 = 1 1111 0100	
			}

			////////////////2010.11.22////////////////////////////////
			if (ep_1ms_d2 >= 2)
			{
				ep_1ms_d2 = 0;
				ep_2ms++;
				if (ep_2ms >= 500)
				{
					ep_2ms = 0;
				}
			}
			if (ep_1ms >= 20)
			{
				ep_1ms = 0;
				ep_20ms++;
				if (ep_20ms >= 50)
				{
					ep_20ms = 0;
				}
			}
			//////////////////////////////////////////////////////////
			if ((BD1_chan[ch].prn >= 6) && (BD1_chan[ch].prn <= 12))
			{
				pCorr->epochCounterCheck = (ep_20ms << 8) + ep_1ms;

			}
			if ((BD1_chan[ch].prn >= 1) && (BD1_chan[ch].prn <= 5))
			{
				pCorr->epochCounterCheck = (ep_2ms << 8) + ep_1ms_d2;

			}
			pCorr->ready = 1;
		}

		phaseLocal += deltaPhase;
		carrierCycle += (phaseLocal >> 30);
		phaseLocal &= 0x3FFFFFFF;

		//1比特和2比特量化
		if (BD1_ADNumber == 2)
		{
			pChar = BD1_productTable[data[counter]][phaseLocal >> 26][localCaTable[*pTau >> 3]];
		}
		else if (BD1_ADNumber == 4)
		{

			int dd1 = 0;
			switch (data[counter])
			{
			case -3:	dd1 = 0;	break;
			case -1:	dd1 = 1;	break;
			case 1:		dd1 = 2;	break;
			case 3:		dd1 = 3;	break;
			}
			pChar = BD1_productTable[dd1][phaseLocal >> 26][localCaTable[*pTau >> 3]];
		}
		else if (BD1_ADNumber == 16)
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
			pChar = BD1_productTable[dd2][phaseLocal >> 26][localCaTable[*pTau >> 3]];
		}

		//}
		//pChar = productTable[data[counter]][phaseLocal>>26][localCaTable[*pTau>>3]];

#ifdef NLASM
#if BD1_ASM_ALONE
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


#endif

	}
#ifdef NLASM
#if BD1_ASM_ALONE

	xmm5ToPls(pls128);
#else

	__asm
	{
		mov eax, pls128
			movdqa      xmmword ptr[eax], xmm5
	}
#endif
#endif


	pCorr->i_early = *(pls128);
	pCorr->q_early = *(pls128 + 1);
	pCorr->i_prompt = *(pls128 + 2);
	pCorr->q_prompt = *(pls128 + 3);

	pCorr->carrierCycle = carrierCycle;

	pCorr->tau = tau;
	pCorr->phaseLocal = phaseLocal;
}

// The following session supports multithreaded correlator
long BD1_lgDataLength;
char* BD1_cgData;
HANDLE BD1_chn_start_event[BD1_chmax + 1];
HANDLE BD1_chn_finish_event[BD1_chmax + 1];
HANDLE BD1_chn_thread[BD1_chmax + 1];
int BD1_chn_id[BD1_chmax + 1];
bool BD1_bFirst = true;
void BD1_CorrProc(LPVOID lpData)
{
	int i = *(int*)lpData;
	while (TRUE)
	{

		WaitForSingleObject(BD1_chn_start_event[i], INFINITE);

		BD1_correlatorChannelProcess(BD1_cgData, BD1_lgDataLength, i);
		SetEvent(BD1_chn_finish_event[i]);
	}
}



void BD1_correlatorProcess(char* data,
	long dataLength)
{

	char i;

	BD1_cgData = data;
	BD1_lgDataLength = dataLength;

	BD1_correlatorDataLength = dataLength;

	if (BD1_gTotAvailCore > 1)
	{		
		if (BD1_bFirst)
		{
			for (i = 0; i <= BD1_chmax; i++)
			{
				BD1_chn_id[i] = i;
				BD1_chn_finish_event[i] = CreateEvent(NULL, FALSE, FALSE, NULL);
				BD1_chn_start_event[i] = CreateEvent(NULL, FALSE, TRUE, NULL);

				BD1_chn_thread[i] =
					CreateThread((LPSECURITY_ATTRIBUTES)NULL,
					0,
					(LPTHREAD_START_ROUTINE)(BD1_CorrProc),
					(LPVOID)&BD1_chn_id[i],
					0,
					NULL);

				SetThreadPriority(BD1_chn_thread[i], THREAD_PRIORITY_ABOVE_NORMAL);
			}
			BD1_bFirst = false;
		}
		else
		{
			for (i = 0; i <= BD1_chmax; i++)
				SetEvent(BD1_chn_start_event[i]);
		}
		WaitForMultipleObjects(BD1_chmax + 1, BD1_chn_finish_event, TRUE, INFINITE);
	}
	else
	{
		for (i = 0; i <= BD1_chmax; i++)
		{
			BD1_correlatorChannelProcess(BD1_cgData, BD1_lgDataLength, i);
		}
	}
	BD1_TIC_RT_CNTR += dataLength;
	if (BD1_TIC_RT_CNTR == (BD1_TIC_CNTR + 1))
	{
		for (i = 0; i <= BD1_chmax; i++)
		{
			BD1_correlator[i].carrierCycleLatch = BD1_correlator[i].carrierCycle;
			BD1_correlator[i].carrierCycle = 0;
			BD1_correlator[i].phaseLocalLatch = BD1_correlator[i].phaseLocal;
			BD1_correlator[i].tauLatch = BD1_correlator[i].tau;
			BD1_correlator[i].epochCounter = BD1_correlator[i].epochCounterCheck;
			//correlator[i].epochCounter_d2 = correlator[i].epochCounterCheck_d2;//2010.11.22
		}
		BD1_TIC_RT_CNTR = 0;
		BD1_TIC_OCCUR_FLAG = 1;
	}
}

void BD1_shutCorrelator()
{
	long i;

	if (BD1_gTotAvailCore > 1)
	{
		for (i = 0; i <= BD1_chmax + 1; i++)
			TerminateThread(BD1_chn_thread[i], 0);
	}
	BD1_freeCaTable(BD1_caTable);
	BD1_freeCAExtendedTable(BD1_CAExtendedTable);
	BD1_freeProductTable();

	for (i = 0; i <= BD1_chmax; i++)
	{
		_aligned_free(BD1_buffer[i]);
	}

}

char** BD1_caGenExtended(char** localCATable)
{
	char **CAExtendedTable;
	int i;
	long deltaPhase;
	long phaseP, phaseE, phaseL;
	char eValue, pValue, lValue;

	CAExtendedTable = (char**)malloc(sizeof(char*)* 14);
	if (CAExtendedTable == NULL)
	{
		printf("memory allocation error in CA extended table generation");
		exit(0);
	}

	CAExtendedTable[0] = (char *)malloc(2046 * CAExtTableResolution * 14 * sizeof(char));
	if (CAExtendedTable[0] == NULL)
	{
		printf("memory allocation error in CA extended table generation");
		exit(0);
	}
	for (i = 1; i<14; i++)
	{
		CAExtendedTable[i] = CAExtendedTable[i - 1] + 2046 * CAExtTableResolution;
	}

	deltaPhase = (long)(long long)(BD1_DLLdT*0.5*CAExtTableResolution + 0.5);
	for (i = 0; i<14; i++)
	{
		for (phaseP = 0; phaseP<2046 * CAExtTableResolution; phaseP++)
		{
			phaseE = phaseP + deltaPhase;
			if (phaseE >= 2046 * CAExtTableResolution)
			{
				phaseE -= 2046 * CAExtTableResolution;
			}
			eValue = localCATable[i][phaseE >> 5];
			pValue = localCATable[i][phaseP >> 5];
			phaseL = phaseP - deltaPhase;
			if (phaseL<0)
			{
				phaseL += 2046 * CAExtTableResolution;
			}
			lValue = localCATable[i][phaseL >> 5];
			CAExtendedTable[i][phaseP] = 0;

			if (eValue>0)
			{
				CAExtendedTable[i][phaseP]++;
			}
			CAExtendedTable[i][phaseP] <<= 1;
			if (pValue>0)
			{
				CAExtendedTable[i][phaseP]++;
			}
			CAExtendedTable[i][phaseP] <<= 1;
			if (lValue>0)
			{
				CAExtendedTable[i][phaseP]++;
			}

		}
	}

	return CAExtendedTable;
}


void BD1_freeCAExtendedTable(char** caExtendedTable)
{
	free(caExtendedTable[0]);
	free(caExtendedTable);
}

void BD1_genProductTable()
{
	long int i, j, k, l;
	char *ADOutput = new char[BD1_ADNumber];
	char code;
	long EPL2E_LP[8] = { 0, 2, 0, 2, -2, 0, -2, 0 };
	//ADOutput[0] = 1;
	//ADOutput[1] = -1;
	//ADOutput[0] = 0;
	//ADOutput[1] = 1;
	if (BD1_ADNumber == 2)  //SetQB(1.0),则ADNumber=2；SetQB(2.0),则ADNumber=4;SetQB(4.0),则ADNumber=16
	{
		ADOutput[0] = 0;
		ADOutput[1] = 1;
	}
	else if (BD1_ADNumber == 4)
	{
		ADOutput[0] = -3;
		ADOutput[1] = -1;
		ADOutput[2] = 1;
		ADOutput[3] = 3;
	}
	else if (BD1_ADNumber == 16)
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


	BD1_productTable = (VIRTUAL_CHAR****)malloc(sizeof(VIRTUAL_CHAR***)*BD1_ADNumber);
	if (BD1_productTable == NULL)
	{
		perror("Product table allocation error 1");
		exit(0);
	}
	for (i = 0; i<BD1_ADNumber; i++)
	{
		BD1_productTable[i] = (VIRTUAL_CHAR***)malloc(sizeof(VIRTUAL_CHAR**)*BD1_CARRIER_TABLE_LENGTH);
		if (BD1_productTable[i] == NULL)
		{
			perror("Product table allocation error 2");
			exit(0);
		}
		for (j = 0; j<BD1_CARRIER_TABLE_LENGTH; j++)
		{
			BD1_productTable[i][j] = (VIRTUAL_CHAR**)malloc(sizeof(VIRTUAL_CHAR*)*CAExtOutput);
			if (BD1_productTable[i][j] == NULL)
			{
				perror("Product table allocation error 3");
				exit(0);
			}
			for (k = 0; k<CAExtOutput; k++)
			{
				BD1_productTable[i][j][k] = (VIRTUAL_CHAR*)_aligned_malloc(sizeof(VIRTUAL_CHAR), 16);
				/************************************************************************/
				if (BD1_productTable[i][j][k] == NULL)
				{
					perror("Product table allocation error 4");
					exit(0);
				}
				code = EPL2E_LP[k];
				long* pLong = (long*)BD1_productTable[i][j][k];
				*pLong++ = ADOutput[i] * BD1_cosTable[j] * code; // E-L:I
				*pLong++ = -ADOutput[i] * BD1_sinTable[j] * code; // E-L:Q
				code = (k << 30) >= 0 ? 1 : -1;
				*pLong++ = ADOutput[i] * BD1_cosTable[j] * code; // P:I
				*pLong++ = -ADOutput[i] * BD1_sinTable[j] * code; // P:Q
			}
		}
	}
	delete[]ADOutput;
}

void BD1_freeProductTable()
{
	int i, j, k;
	for (i = 0; i<BD1_ADNumber; i++)
	{
		for (j = 0; j<BD1_CARRIER_TABLE_LENGTH; j++)
		{
			for (k = 0; k<CAExtOutput; k++)
			{
				_aligned_free(BD1_productTable[i][j][k]);
			}
			free(BD1_productTable[i][j]);
		}
		free(BD1_productTable[i]);
	}
	free(BD1_productTable);
}
#endif
