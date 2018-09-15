// Released on July 9, 2004
#include "../stdafx.h"
#ifndef REAL_TIME

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "GLO1_cacode.h"


void GLO1_genCaTimeSequence(
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

	floatCodePhase = tBegin*511e3;				
	floatCodePhase = fmod(floatCodePhase,511.0);	
	deltaCodePhase = deltaT*511e3;				
	
	for (i=0;i<num;i++)
	{
		codeIndex = (long)floatCodePhase;	
		code[i] = (float) caTable[sv-1][codeIndex];
		floatCodePhase += deltaCodePhase;				
		if (floatCodePhase >= 511.0)
		{
			floatCodePhase -= 511.0;
		}
	}
}

#endif
