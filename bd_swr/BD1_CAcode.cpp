// Released on July 9, 2004
#include "../StdAfx.h"
#ifndef REAL_TIME

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "BD1_cacode.h"

// 该函数仅用于FFT法捕获信号时，产生本地参考的CA码序列
void BD1_genCaTimeSequence(
					   char** caTable,	// CA码表的首地址: input    caTable
					   short sv,		// 卫星标号1-37: input    sv
					   double tBegin,	// 起始时间，单位：秒: input   deltaT*0.5
					   double deltaT,	// 样点间的时间间隔，单位：秒: input    deltaT
					   long num,		// 样点个数: input   nLocal  2毫秒的点数
					   float* code		// 返回的码序列,值为1，-1: output   rCa
					   )
{
	double floatCodePhase, deltaCodePhase;
	long codeIndex;
	long i;

	if (tBegin<0)
	{
		puts("Error in time arrangement");
		exit(0);
	}

	floatCodePhase = tBegin*2046e3;						// 计算起始的浮点码相位
	floatCodePhase = fmod(floatCodePhase,2046.0);		// 限制码相位为0-1022.999999999...
	deltaCodePhase = deltaT*2046e3;						// 计算样点间的码相位差

	for (i=0;i<num;i++)//long num,		// 样点个数: input   nLocal  2毫秒的点数
	{
		codeIndex = (long)(long long)floatCodePhase;				// 计算码相位的索引
		code[i] = (float) caTable[sv-1][codeIndex];	// 取值
		floatCodePhase += deltaCodePhase;				
		if (floatCodePhase >= 2046.0)
		{
			floatCodePhase -= 2046.0;
		}
	}
}

#endif
