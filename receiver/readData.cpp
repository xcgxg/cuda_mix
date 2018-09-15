// Released on July 9, 2004
#include "../stdafx.h"
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include "../gps_swr/gpsconst.h"



void readFileHeader(FILE* fp)
{

}

long readData(char* data, long num, FILE* fp)
{
	long n;
	n = fread(data,sizeof(char),num,fp);
	return n;
}