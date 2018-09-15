// Released on July 9, 2004
#include "../stdafx.h"
#ifndef REAL_TIME

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cacode.h"


void genCaTimeSequence(
					   char** caTable,
					   short sv,	
					   double tBegin,
					   double deltaT,
					   long num,	
					   float* code
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

	floatCodePhase = tBegin*1023e3;				
	floatCodePhase = fmod(floatCodePhase,1023.0);	
	deltaCodePhase = deltaT*1023e3;				
	
	for (i=0;i<num;i++)
	{
		codeIndex = (long)floatCodePhase;	
		code[i] = (float) caTable[sv-1][codeIndex];
		floatCodePhase += deltaCodePhase;				
		if (floatCodePhase >= 1023.0)
		{
			floatCodePhase -= 1023.0;
		}
	}
}

#endif
