// Released on July 9, 2004

#ifndef CORRELATORPROCESS_H
#define CORRELATORPROCESS_H

#define l_2p26  (long) 67108864
#define d_2p26	(double) 67108864
#define d_2m26	(double) 1.490116119384765625e-8


#define carrierNCOMax (long) CARRIER_TABLE_LENGTH*l_2p26
#define carrierNCORes (double) 9.31322574615478515625e-10  //2^-30

#define l_2p40 (__int64) 1099511627776
#define d_2p40 (double) 1099511627776.0
#define d_2m40 (double) 9.094947017729282379150390625e-13
#define codeNCOMax  (__int64) 1023*1099511627776


#define tauConst	(double)1.023e6*d_2p40

#define dllDTConst (double) DLLdT*0.5*d_2p40
#define dllDT (__int64)(dllDTConst+0.5)
#define tauLate   (__int64) (codeNCOMax - dllDT)

#define	branchesPerChan (int)6
#define CAExtOutput (int)8

void initCorrelator();


void correlatorChannelProcess(char* data,
			long dataLength,				
			char ch
			);				

// This function simulates the whole correlator
void correlatorProcess(char* data,
					   long dataLength);	

void shutCorrelator();

char** caGenExtended(char** localCATable);

void freeCAExtendedTable(char** caExtendedTable);

void genProductTable();

void freeProductTable();

#endif