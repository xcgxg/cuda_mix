// Released on July 9, 2004

#ifndef BD1_CORRELATORPROCESS_H
#define BD1_CORRELATORPROCESS_H

#define l_2p26  ((long) 67108864)
#define l_2p30  ((long) 1073741824)
#define d_2p26	((double) 67108864)
#define d_2m26	((double) 1.490116119384765625e-8)

#define carrierNCOMax ((long) (long long) BD1_CARRIER_TABLE_LENGTH*l_2p26)
#define carrierNCORes ((double) 9.31322574615478515625e-10)  //2^-30

#define l_2p40 ((__int64) 1099511627776)
#define d_2p40 ((double) 1099511627776.0)
#define d_2m40 ((double) 9.094947017729282379150390625e-13)
#define codeNCOMax  ((__int64) 2046*1099511627776)

#define tauConst	((double)2.046e6*d_2p40) //add by jh 3-13
#define dllDTConst ((double) DLLdT*0.5*d_2p40)
#define dllDT ((__int64)(dllDTConst+0.5))
#define tauLate   ((__int64) (codeNCOMax - dllDT))

#define CAExtOutput (int)8

void BD1_initCorrelator();


void BD1_correlatorChannelProcess(char* data,
	long dataLength,
	char ch
	);

// This function simulates the whole correlator
void BD1_correlatorProcess(char* data,
	long dataLength);

void BD1_shutCorrelator();

char** BD1_caGenExtended(char** localCATable);

void BD1_freeCAExtendedTable(char** caExtendedTable);

void BD1_genProductTable();

void BD1_freeProductTable();

#endif