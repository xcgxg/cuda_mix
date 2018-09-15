// Released on July 9, 2004

#ifndef GLO1_CORRELATORPROCESS_H
#define GLO1_CORRELATORPROCESS_H

#define GLO1_l_2p26  (long) 67108864
#define GLO1_d_2p26	(double) 67108864
#define GLO1_d_2m26	(double) 1.490116119384765625e-8


#define GLO1_carrierNCOMax (long) GLO1_CARRIER_TABLE_LENGTH*GLO1_l_2p26
#define GLO1_carrierNCORes (double) 9.31322574615478515625e-10  //2^-30

#define GLO1_l_2p40 (__int64) 1099511627776
#define GLO1_d_2p40 (double) 1099511627776.0
#define GLO1_d_2m40 (double) 9.094947017729282379150390625e-13
#define GLO1_codeNCOMax  (__int64) 511*1099511627776



#define GLO1_tauConst	(double)0.511e6*GLO1_d_2p40

#define GLO1_dllDTConst (double) GLO1_DLLdT*0.5*GLO1_d_2p40
#define GLO1_dllDT (__int64)(GLO1_dllDTConst+0.5)
#define GLO1_tauLate   (__int64) (GLO1_codeNCOMax - GLO1_dllDT)

#define	GLO1_branchesPerChan (int)6
#define GLO1_CAExtOutput (int)8

void GLO1_initCorrelator();


void GLO1_correlatorChannelProcess(char* data,
			long dataLength,				
			char ch
			);				

// This function simulates the whole correlator
void GLO1_correlatorProcess(char* data,
					   long dataLength);	

void GLO1_shutCorrelator();

char** GLO1_caGenExtended(char** localCATable);

void GLO1_freeCAExtendedTable(char** caExtendedTable);

void GLO1_genProductTable();

void GLO1_freeProductTable();

#endif