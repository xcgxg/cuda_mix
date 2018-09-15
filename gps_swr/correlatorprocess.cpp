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
#include "gpsstruct.h"
#include "gpsconst.h"
#include "cagen.h"
#include "correlatorprocess.h"
#include "cpudetect.h"
//#include "TimeCounterEx.h"
#include "gp2021.h"
#include "cpudetect.h"

#define CAExtTableResolution (int)32
#define CAExtTableShift      (int)3  

#define VIRTUAL_CHAR __m128i

#define ASM_ALONE 1

extern "C" void plsToXmm5(long*);
extern "C" void xmm5ToPls(long*);
extern "C" void xorXmm5();
extern "C" void xmm5_xmm4(__m128i*);

bool bFirst = true;

// 存储载波相位的sin, cos表，全局变量，本地初始化
char sinTable[CARRIER_TABLE_LENGTH];
char cosTable[CARRIER_TABLE_LENGTH];

int ADNumber;

// 外部定义的全局变量
extern CHANNEL chan[chmax + 1];			// 信道的状态变量数组
extern CORRELATOR correlator[chmax + 1];	// 相关器通道状态变量数组
extern long correlatorDataLength;		// 某次缓存的输入数据个数＝采样率*中断间隔
// 如5MHz*0.5ms = 2500
extern long	globalBufLength[chmax + 1];	// 用于保存每个通道缓存的数据长度，需初始化
extern double *buffer[chmax + 1];		// 用于保存每个通道的缓存数据，需初始化分配空间
// 相关器缓存的数字化数据数组，动态分配，最大长度10000
extern long TIC_RT_CNTR;				// 测量中断的实时计数器: 0～测量中断的最大计数值-1
extern long TIC_CNTR;					// 测量中断的最大计数值
extern double TIC_CNTR_RES;
extern double TIC_CNTR_DBL;
extern long TIC_OCCUR_FLAG;				// 测量中断标志，在相关器仿真程序中被置1
// 在测量中断服务程序中被复位为0
extern char** caTable;					// CA码表：37×1023，取值为1,-1 <-> 1,0
// CA码表在CAGEN程序中被分配和初始化
char** CAExtendedTable;					// 扩展的CA码表，37×1023字节，动态分配
// 存储的是EPL支路的CA码，表示用0,1
// E支路在MSB，L支路在LSB，char型数最多表示8个支路
VIRTUAL_CHAR**** productTable;					// 存储输入信号×载波×CA码的结果

extern double DLLdT;            //DLLdt

extern int gTotAvailCore;

extern float CARRIER_FREQ;
extern double deltaPhaseConst;
extern double deltaCodePhaseConst;
extern long DETECTSIZE;

#ifndef REAL_TIME

void initCorrelator()
{
	double	phase0, deltaPhase, phase;
	double  amplitude, value, absValue, signValue;
	long	i;

	// generate two tables to look up for carrier phase
	phase0 = twoPI / (2 * CARRIER_TABLE_LENGTH);
	deltaPhase = twoPI / CARRIER_TABLE_LENGTH;
	amplitude = CARRIER_TABLE_LENGTH / 2.0;
	for (i = 0; i<CARRIER_TABLE_LENGTH; i++)
	{
		phase = phase0 + deltaPhase*i;
		value = sin(phase);
		absValue = fabs(value);
		signValue = value >= 0 ? 1 : -1;
		sinTable[i] = (char)(signValue*ceil(absValue*amplitude));
		value = cos(phase);
		absValue = fabs(value);
		signValue = value >= 0 ? 1 : -1;
		cosTable[i] = (char)(signValue*ceil(absValue*amplitude));
	}

	// generate CA code table to look up
	caTable = caGen();
	CAExtendedTable = caGenExtended(caTable);
	genProductTable();

	for (i = 0; i <= chmax; i++)
	{
		long long carr = (long long)(CARRIER_FREQ*deltaPhaseConst + 0.5);
		correlator[i].fCarr = (long)(carr & 0xffffffffUL);
		correlator[i].fCode = (__int64)(1.0*deltaCodePhaseConst + 0.5);
		correlator[i].state = CHN_OFF;
		correlator[i].carrierCycle = 0;
		correlator[i].ready = 1;
	}

	for (i = 0; i <= chmax; i++)
	{
		globalBufLength[i] = 0;
		buffer[i] = (double *)_aligned_malloc((DETECTSIZE + 5000)*sizeof(double), sizeof(double));
		if (buffer[i] == NULL)
		{
			perror(" memory allocation error");
			exit(0);
		}
	}

	CPUDetection();
}

/************************************************************************/
/*
先把相关器.ready置1，通道.state设为捕获，使能进到ch_acq里，只有在ch_acq里捕获到了，
才把相关器.state设为CHN_oN，把相关器.ready置0，通道.state设牵引
这样相关器才能运行，进行积分清除，到了1毫秒，就把相关器.ready设为1，这样才能在下次进到通道.state选择时进到ch_pull_in里
*/
/************************************************************************/

void correlatorChannelProcess(char* data,	// data array 
	long dataLength,				// data length 每次进0.5毫秒的数据			
	char ch)						// channel number	
{
	CORRELATOR* pCorr;
	pCorr = &correlator[ch];
	if (pCorr->state != CHN_ON)
		return;
	__int64	tau;
	register long	phaseLocal;

	__m128i	s128;
	long*	pls128 = (long*)&s128;

	long	deltaPhase;
	__int64	deltaCodePhase;
	long	counter;
	char*	localCaTable;
	long	ep_1ms;
	long	ep_20ms;
	long	carrierCycle;
	__m128i*	pChar = 0;

	long*	pTau = (long *)&tau + 1;
	char*	pData = data;

	// choose a CA code Table according to the PRN number
	localCaTable = CAExtendedTable[pCorr->sv - 1];
	phaseLocal = pCorr->phaseLocal;
	deltaPhase = pCorr->fCarr;

	carrierCycle = pCorr->carrierCycle;

	tau = pCorr->tau;

	deltaCodePhase = pCorr->fCode;
	*(pls128) = (pCorr->i_early);
	*(pls128 + 1) = (pCorr->q_early);
	*(pls128 + 2) = (pCorr->i_prompt);
	*(pls128 + 3) = (pCorr->q_prompt);

#ifdef NLASM//NING LUO ASM
#if ASM_ALONE
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
		tau += deltaCodePhase;
		if (*pTau >= 261888)
		{
			*pTau -= 261888;
#ifdef NLASM         
#if ASM_ALONE
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
#if ASM_ALONE
			xorXmm5();
#else

			__asm
			{
				xorps  xmm5, xmm5
			}
#endif
#else
			*psR = *(psR + 1) = *(psR + 2) = 0;
			*psI = *(psI + 1) = *(psI + 2) = 0;
#endif


			ep_1ms = (pCorr->epochCounterCheck & 0x001F) + 1;
			ep_20ms = (pCorr->epochCounterCheck & 0x3F00) >> 8;
			if (ep_1ms >= 20)
			{
				ep_1ms = 0;
				ep_20ms++;
				if (ep_20ms >= 50)
				{
					ep_20ms = 0;
				}
			}
			pCorr->epochCounterCheck = (ep_20ms << 8) + ep_1ms;
			pCorr->ready = 1;
		}
		phaseLocal += deltaPhase;
		carrierCycle += (phaseLocal >> 30);
		phaseLocal &= 0x3FFFFFFF;

		if (ADNumber == 2)
		{
			pChar = productTable[data[counter]][phaseLocal >> 26][localCaTable[*pTau >> 3]];
		}
		else if (ADNumber == 4)
		{
			int dd1 = 0;
			switch (data[counter])
			{
			case -3:	dd1 = 0;	break;
			case -1:	dd1 = 1;	break;
			case 1:		dd1 = 2;	break;
			case 3:		dd1 = 3;	break;
			}
			pChar = productTable[dd1][phaseLocal >> 26][localCaTable[*pTau >> 3]];
		}
		else if (ADNumber == 16)
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
			pChar = productTable[dd2][phaseLocal >> 26][localCaTable[*pTau >> 3]];
		}

#ifdef NLASM
#if ASM_ALONE
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
		sI = _mm_sub_epi32(sI, *pChar);
		sR = _mm_add_epi32(sR, *(pChar + 1));


#endif

	}
#ifdef NLASM
#if ASM_ALONE

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
	pCorr->q_early = *(pls128 + 1);
	pCorr->i_prompt = *(pls128 + 2);
	pCorr->q_prompt = *(pls128 + 3);

	pCorr->carrierCycle = carrierCycle;

	pCorr->tau = tau;
	pCorr->phaseLocal = phaseLocal;
}

// The following session supports multithreaded correlator
long lgDataLength;
char* cgData;
HANDLE chn_start_event[chmax + 1];
HANDLE chn_finish_event[chmax + 1];
HANDLE chn_thread[chmax + 1];
int chn_id[chmax + 1];
void CorrProc(LPVOID lpData)
{
	int i = *(int*)lpData;
	while (TRUE)
	{
		WaitForSingleObject(chn_start_event[i], INFINITE);
		correlatorChannelProcess(cgData, lgDataLength, i);

		SetEvent(chn_finish_event[i]);
	}
}

void correlatorProcess(char* data,
	long dataLength)
{

	char i;

	correlatorDataLength = dataLength;

	cgData = data;
	lgDataLength = dataLength;

	if (gTotAvailCore > 1)
	{
		//s-tatic bool bFirst = true;
		
		if (bFirst)
		{
			for (i = 0; i <= chmax; i++)
			{
				chn_id[i] = i;
				chn_finish_event[i] = CreateEvent(NULL, FALSE, FALSE, NULL);
				chn_start_event[i] = CreateEvent(NULL, FALSE, TRUE, NULL);

				chn_thread[i] =
					CreateThread((LPSECURITY_ATTRIBUTES)NULL,
					0,
					(LPTHREAD_START_ROUTINE)(CorrProc),
					(LPVOID)&chn_id[i],
					0,
					NULL);

				SetThreadPriority(chn_thread[i], THREAD_PRIORITY_NORMAL);
			}
			bFirst = false;
		}
		else
		{
			for (i = 0; i <= chmax; i++)
				SetEvent(chn_start_event[i]);
		}
		WaitForMultipleObjects(chmax + 1, chn_finish_event, TRUE, INFINITE);
	}
	else
	{
		for (i = 0; i <= chmax; i++)
		{
			correlatorChannelProcess(cgData, lgDataLength, i);
		}
	}
	TIC_RT_CNTR += dataLength;
	if (TIC_RT_CNTR == (TIC_CNTR + 1))
	{
		for (i = 0; i <= chmax; i++)
		{
			correlator[i].carrierCycleLatch = correlator[i].carrierCycle;
			correlator[i].carrierCycle = 0;
			correlator[i].phaseLocalLatch = correlator[i].phaseLocal;
			correlator[i].tauLatch = correlator[i].tau;
			correlator[i].epochCounter = correlator[i].epochCounterCheck;
		}
		TIC_RT_CNTR = 0;
		TIC_OCCUR_FLAG = 1;
	}
}

void shutCorrelator()
{
	long i;

	if (gTotAvailCore > 1)
	{
		for (i = 0; i <= chmax + 1; i++)
			TerminateThread(chn_thread[i], 0);
	}
	freeCaTable(caTable);
	freeCAExtendedTable(CAExtendedTable);
	freeProductTable();

	for (i = 0; i <= chmax; i++)
	{
		_aligned_free(buffer[i]);
	}

}

char** caGenExtended(char** localCATable)
{
	char **CAExtendedTable;
	int i;
	long deltaPhase;
	long phaseP, phaseE, phaseL;
	char eValue, pValue, lValue;

	CAExtendedTable = (char**)malloc(sizeof(char*)* 37);
	if (CAExtendedTable == NULL)
	{
		printf("memory allocation error in CA extended table generation");
		exit(0);
	}
	CAExtendedTable[0] = (char *)malloc(1023 * CAExtTableResolution * 37 * sizeof(char));
	if (CAExtendedTable[0] == NULL)
	{
		printf("memory allocation error in CA extended table generation");
		exit(0);
	}
	for (i = 1; i<37; i++)
	{
		CAExtendedTable[i] = CAExtendedTable[i - 1] + 1023 * CAExtTableResolution;
	}
	deltaPhase = (long)(DLLdT*0.5*CAExtTableResolution + 0.5);
	for (i = 0; i<37; i++)
	{
		for (phaseP = 0; phaseP<1023 * CAExtTableResolution; phaseP++)
		{
			phaseE = phaseP + deltaPhase;
			if (phaseE >= 1023 * CAExtTableResolution)
			{
				phaseE -= 1023 * CAExtTableResolution;
			}
			eValue = localCATable[i][phaseE >> 5];
			pValue = localCATable[i][phaseP >> 5];
			phaseL = phaseP - deltaPhase;
			if (phaseL<0)
			{
				phaseL += 1023 * CAExtTableResolution;
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


void freeCAExtendedTable(char** caExtendedTable)
{
	free(caExtendedTable[0]);
	free(caExtendedTable);
}

void genProductTable()
{
	long int i, j, k, l;
	char *ADOutput = new char[ADNumber];
	char code;
	long EPL2E_LP[8] = { 0, 2, 0, 2, -2, 0, -2, 0 };
	if (ADNumber == 2)  //SetQB(1.0),则ADNumber=2；SetQB(2.0),则ADNumber=4
	{
		ADOutput[0] = 0;
		ADOutput[1] = 1;
	}
	else if (ADNumber == 4){
		ADOutput[0] = -3;
		ADOutput[1] = -1;
		ADOutput[2] = 1;
		ADOutput[3] = 3;
	}
	else if (ADNumber == 16)
	{
		/*ADOutput[0] = 0;
		ADOutput[1] = 1;
		ADOutput[2] = 2;
		ADOutput[3] = 3;
		ADOutput[4] = 4;
		ADOutput[5] = 5;
		ADOutput[6] = 6;
		ADOutput[7] = 7;
		ADOutput[8] = 8;
		ADOutput[9] = 9;
		ADOutput[10] = 10;
		ADOutput[11] = 11;
		ADOutput[12] = 12;
		ADOutput[13] = 13;
		ADOutput[14] = 14;
		ADOutput[15] = 15;*/
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

	productTable = (VIRTUAL_CHAR****)malloc(sizeof(VIRTUAL_CHAR***)*ADNumber);
	// __m128i ***bigTable = (__m128i***) productTable;
	if (productTable == NULL)
	{
		perror("Product table allocation error 1");
		exit(0);
	}
	for (i = 0; i<ADNumber; i++)
	{
		productTable[i] = (VIRTUAL_CHAR***)malloc(sizeof(VIRTUAL_CHAR**)*CARRIER_TABLE_LENGTH);
		if (productTable[i] == NULL)
		{
			perror("Product table allocation error 2");
			exit(0);
		}
		for (j = 0; j<CARRIER_TABLE_LENGTH; j++)
		{
			productTable[i][j] = (VIRTUAL_CHAR**)malloc(sizeof(VIRTUAL_CHAR*)*CAExtOutput);
			if (productTable[i][j] == NULL)
			{
				perror("Product table allocation error 3");
				exit(0);
			}
			for (k = 0; k<CAExtOutput; k++)
			{
				productTable[i][j][k] = (VIRTUAL_CHAR*)_aligned_malloc(sizeof(VIRTUAL_CHAR), 16);
				if (productTable[i][j][k] == NULL)
				{
					perror("Product table allocation error 4");
					exit(0);
				}
				code = EPL2E_LP[k];
				long* pLong = (long*)productTable[i][j][k];
				*pLong++ = ADOutput[i] * cosTable[j] * code;
				*pLong++ = -ADOutput[i] * sinTable[j] * code;
				code = (k << 30) >= 0 ? 1 : -1;
				*pLong++ = ADOutput[i] * cosTable[j] * code;
				*pLong++ = -ADOutput[i] * sinTable[j] * code;
				/*s-tatic TCHAR buf[130];
				pLong = (long*)productTable[i][j][k];
				wsprintf(buf, TEXT("productTable[%d][%d][%d] = %d, %d, %d, %d.\n"),
				i, j, k, pLong[0], pLong[1], pLong[2], pLong[3]);
				OutputDebugString(buf);*/
			}
		}
	}
	delete[]ADOutput;
}

void freeProductTable()
{
	int i, j, k;
	for (i = 0; i<ADNumber; i++)
	{
		for (j = 0; j<CARRIER_TABLE_LENGTH; j++)
		{
			for (k = 0; k<CAExtOutput; k++)
			{
				_aligned_free(productTable[i][j][k]);
			}
			free(productTable[i][j]);
		}
		free(productTable[i]);
	}
	free(productTable);
}
#endif