// Released on July 9, 2004
#include "../stdafx.h"
#ifndef REAL_TIME

// 系统头文件
#include <malloc.h>
#include <stdlib.h>
#include <stdio.h>

#include "GLO1_cagen.h"

//void g1Gen(char* g1)
//{
//	short i;
//
//	for (i=0;i<10;i++)
//	{
//		g1[i]=1;
//	}
//	for (i=0;i<1023;i++)
//	{
//		g1[i+10]=g1[i+7]^g1[i];
//	}
//
//}
//
//void g2Gen(char* g2)
//{
//	short i;
//
//	for (i=0;i<10;i++)
//	{
//		g2[i]=1;
//	}
//
//	for (i=0;i<1023;i++)
//	{
//		g2[i+10]=g2[i]^g2[i+1]^g2[i+2]^g2[i+4]^g2[i+7]^g2[i+8];
//	}
//
//
//}


//char** caGen(void)
//{
//	char g1[1023+10], g2[1023+10];//char g1[2046+11], g2[2046+11];
//	short sv[37]={5,6,7,8,17,18,139,140,141,251,252,254,255,256,257,258,469,
//		470,471,472,473,474,509,512,513,514,515,516,859,860,861,862,863,950,947,948,950};
//	short i, j, index;
//	char* pTable;
//	char** LocalCATable;
//
//	LocalCATable = (char**)malloc(37*sizeof(char*));
//	if(LocalCATable==NULL)
//	{
//		printf("memory allocation error in CA table generation");
//		exit(0);
//	}
//	for (i=0;i<37;i++)
//	{
//		LocalCATable[i]=(char *)malloc(1023*sizeof(char));
//		if (LocalCATable[i]==NULL) 
//		{
//			printf("memory allocation error in CA table generation");
//			exit(0);
//		}
//	}
//
//	g1Gen(g1);
//	g2Gen(g2);
//	
//	for (i=0;i<37;i++)
//	{
//		pTable = LocalCATable[i];
//		for (j=0; j<1023; j++)
//		{
//			index = (j+1023-sv[i])%1023;
//			*pTable++ = (g2[index]^g1[j])*2-1;
//		}
//	}
//	/************************************************************************/
//	return LocalCATable;
//}

char** GLO1_caGen(void)
{
	char** LocalCATable;

	LocalCATable = (char**)malloc(37*sizeof(char*));
	if(LocalCATable==NULL)
	{
		printf("memory allocation error in CA table generation");
		exit(0);
	}
	for (int i=0;i<37;i++)
	{
		LocalCATable[i]=(char *)malloc(511*sizeof(char));

		char D1 = 1, D2 = 1, D3 = 1, D4 = 1, D5 = 1, D6 = 1, D7 = 1, D8 = 1, D9 = 1;
		char tmp;
		for (int j = 0;j < 511;++j)
		{
			LocalCATable[i][j]=D7;
			// tmp = D5*D9;
		    tmp = D5^D9;
			D9 = D8;
			D8 = D7;
			D7 = D6;
			D6 = D5;
			D5 = D4;
			D4 = D3;
			D3 = D2;
			D2 = D1;
			D1 = tmp;
		}
	}
	return LocalCATable;
}

void GLO1_freeCaTable(char** caTable)
{
	short i;
	for (i=0;i<37;i++)
	{
		free(caTable[i]);
	}
	free(caTable);
}

#endif