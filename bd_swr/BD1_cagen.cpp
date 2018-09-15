// Released on July 9, 2004
#include "../StdAfx.h"
#ifndef REAL_TIME

// 系统头文件
#include <malloc.h>
#include <stdlib.h>
#include <stdio.h>

#include "BD1_cagen.h"

char** BD1_bdd2_caGen(void)
{
	short i;
	char* pTable;
	char** LocalCATable;
	
	LocalCATable = (char**)malloc(14*sizeof(char*));//LocalCATable = (char**)malloc(37*sizeof(char*));
	if(LocalCATable==NULL)
	{
		printf("memory allocation error in CA table generation");
		exit(0);
	}
	for (i=0;i<14;i++)//for (i=0;i<37;i++)
	{
		LocalCATable[i]=(char *)malloc(2046*sizeof(char));
		if (LocalCATable[i]==NULL) 
		{
			printf("memory allocation error in CA table generation");
			exit(0);
		}
	}
	
	
	
	for (i=0;i<14;i++)//for (i=0;i<37;i++)
	{
		pTable = LocalCATable[i];
		BD1_CreateCACodeTable(pTable, i + 1);
		
	}
	return LocalCATable;


}

void BD1_CreateCACodeTable(char svcode[], int prn)
{
    int Sel1,Sel2;        
    int G1_array[2046];   
    int G2_array[2046];
    int G1[11]={1,-1,1,-1,1,-1,1,-1,1,-1,1};           
	int G2[11]={1,-1,1,-1,1,-1,1,-1,1,-1,1};
	switch (prn)
	{
	case 1:            
		Sel1=1;       
		Sel2=3;
		break;
	case 2:
		Sel1=1;
		Sel2=4;
		break;
	case 3:
		Sel1=1;
		Sel2=5;
		break;
	case 4:
		Sel1=1;
		Sel2=6;
		break;
	case 5:
		Sel1=1;
		Sel2=8;
		break;
	case 6:
		Sel1=1;
		Sel2=9;
		break;
	case 7:
		Sel1=1;
		Sel2=10;
		break;
	case 8:
		Sel1=1;
		Sel2=11;
		break;
	case 9:
		Sel1=2;
		Sel2=7;
		break;
	case 10:
		Sel1=3;
		Sel2=4;
		break;
	case 11:
		Sel1=3;
		Sel2=5;
		break;
	case 12:
		Sel1=3;
		Sel2=6;
		break;
	case 13:
		Sel1=3;
		Sel2=8;
		break;
	case 14:
		Sel1=3;
		Sel2=9;
		break;
	case 15:
		Sel1=3;
		Sel2=10;
	    break;
	case 16:
		Sel1=3;
		Sel2=11;
		break;
	case 17:
		Sel1=4;
		Sel2=5;
		break;
	case 18:
		Sel1=4;
		Sel2=6;
		break;
	case 19:
		Sel1=4;
		Sel2=8;
		break;
    case 20:
		Sel1=4;
		Sel2=9;
		break;
	case 21:
		Sel1=4;
		Sel2=10;
		break;
	case 22:
		Sel1=4;
		Sel2=11;
		break;
	case 23:
		Sel1=5;
		Sel2=6;
		break;
	case 24:
		Sel1=5;
		Sel2=8;
		break;
	case 25:
		Sel1=5;
		Sel2=9;
		break;
	case 26:
		Sel1=5;
		Sel2=10;
		break;
	case 27:
		Sel1=5;
		Sel2=11;
		break;
    case 28:
		Sel1=6;
		Sel2=8;
		break;
	case 29:
		Sel1=6;
		Sel2=9;
		break;
	case 30:
		Sel1=6;
		Sel2=10;
		break;
	case 31:
		Sel1=6;
		Sel2=11;
		break;
	case 32:
		Sel1=8;
		Sel2=9;
		break;
	case 33:
		Sel1=8;
		Sel2=10;
		break;
	case 34:
		Sel1=8;
		Sel2=11;
		break;
	case 35:
		Sel1=9;
		Sel2=10;
		break;
	case 36:
		Sel1=9;
		Sel2=11;
		break;
	case 37:
		Sel1=10;
		Sel2=11;
		break;
	default :
		printf("The satellite number is invalid. \n");
	}//end switch
	for (int i=0;i<2046;i++)              
	{
		G1_array[i]=G1[10]; 
		int temp1=G1[0];                        
		G1[0]=G1[0]*G1[6]*G1[7]*G1[8]*G1[9]*G1[10];  
		G1[10]=G1[9];                      
		G1[9]=G1[8];
		G1[8]=G1[7]; 
		G1[7]=G1[6]; 
		G1[6]=G1[5]; 
		G1[5]=G1[4]; 
		G1[4]=G1[3]; 
		G1[3]=G1[2]; 
		G1[2]=G1[1]; 
		G1[1]=temp1;     
		
		
		G2_array[i]=G2[Sel1-1]*G2[Sel2-1];
		int temp2=G2[0];
		G2[0]=G2[0]*G2[1]*G2[2]*G2[3]*G2[4]*G2[7]*G2[8]*G2[10];
		G2[10]=G2[9];                          
		G2[9]=G2[8];
		G2[8]=G2[7]; 
		G2[7]=G2[6]; 
		G2[6]=G2[5]; 
		G2[5]=G2[4]; 
		G2[4]=G2[3]; 
		G2[3]=G2[2]; 
		G2[2]=G2[1]; 
		G2[1]=temp2; 
	}

	for (int l=0;l<2046;l++)
	{
		
		svcode[l]=(char)G1_array[l]*G2_array[l];   
	} 

	return;

}


// 在主程序最后必须释放caTable
void BD1_freeCaTable(char** caTable)
{
	short i;
	for (i=0;i<14;i++)
	{
		free(caTable[i]);
	}
	free(caTable);
}

#endif