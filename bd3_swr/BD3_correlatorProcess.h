// Released on July 9, 2004

#ifndef BD3_CORRELATORPROCESS_H
#define BD3_CORRELATORPROCESS_H

#define BD3_l_2p26  ((long) 67108864)
#define BD3_l_2p30  ((long) 1073741824)
#define BD3_d_2p26	((double) 67108864)
#define BD3_d_2m26	((double) 1.490116119384765625e-8)

#define BD3_carrierNCOMax ((long) (long long) BD3_CARRIER_TABLE_LENGTH*BD3_l_2p26)
#define BD3_carrierNCORes ((double) 9.31322574615478515625e-10)  //2^-30

#define BD3_l_2p40 ((__int64) 1099511627776)
#define BD3_d_2p40 ((double) 1099511627776.0)
#define BD3_d_2m40 ((double) 9.094947017729282379150390625e-13)
//#define codeNCOMax  ((__int64) 2046*1099511627776)
#define BD3_codeNCOMax  ((__int64) 10230*1099511627776) //done

//#define tauConst	((double)2.046e6*d_2p40) //add by jh 3-13
#define BD3_tauConst	((double)10.23e6*BD3_d_2p40)//done
#define BD3_dllDTConst ((double) BD3_DLLdT*0.5*BD3_d_2p40)
#define BD3_dllDT ((__int64)(BD3_dllDTConst+0.5))
#define BD3_tauLate   ((__int64) (BD3_codeNCOMax - BD3_dllDT))

#define BD3_CAExtOutput (int)8

void BD3_initCorrelator();


void BD3_correlatorChannelProcess(char* data,
			long dataLength,			
			char ch
			);				

// This function simulates the whole correlator
void BD3_correlatorProcess(char* data,
			long dataLength);	

void BD3_shutCorrelator();

char** BD3_caGenExtended(char** localCATable);

void BD3_freeCAExtendedTable(char** caExtendedTable);

void BD3_genProductTable();

void BD3_freeProductTable();

#endif