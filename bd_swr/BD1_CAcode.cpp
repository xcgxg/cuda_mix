// Released on July 9, 2004
#include "../StdAfx.h"
#ifndef REAL_TIME

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "BD1_cacode.h"

// �ú���������FFT�������ź�ʱ���������زο���CA������
void BD1_genCaTimeSequence(
					   char** caTable,	// CA�����׵�ַ: input    caTable
					   short sv,		// ���Ǳ��1-37: input    sv
					   double tBegin,	// ��ʼʱ�䣬��λ����: input   deltaT*0.5
					   double deltaT,	// ������ʱ��������λ����: input    deltaT
					   long num,		// �������: input   nLocal  2����ĵ���
					   float* code		// ���ص�������,ֵΪ1��-1: output   rCa
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

	floatCodePhase = tBegin*2046e3;						// ������ʼ�ĸ�������λ
	floatCodePhase = fmod(floatCodePhase,2046.0);		// ��������λΪ0-1022.999999999...
	deltaCodePhase = deltaT*2046e3;						// ��������������λ��

	for (i=0;i<num;i++)//long num,		// �������: input   nLocal  2����ĵ���
	{
		codeIndex = (long)(long long)floatCodePhase;				// ��������λ������
		code[i] = (float) caTable[sv-1][codeIndex];	// ȡֵ
		floatCodePhase += deltaCodePhase;				
		if (floatCodePhase >= 2046.0)
		{
			floatCodePhase -= 2046.0;
		}
	}
}

#endif
