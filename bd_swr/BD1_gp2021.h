// Released on July 9, 2004
#ifndef BD1_GP2021_H
#define BD1_GP2021_H

inline void BD1_to_gps(int add, int data);
inline int BD1_from_gps(int add);
inline int BD1_accum_status(void);
void BD1_all_accum_reset(void);
void BD1_data_tst(int data);
//2010.11.22
unsigned int BD1_readEpoch(char ch);
unsigned int BD1_readEpochCheck(char ch);
void BD1_writeEpoch(char ch, unsigned data);
/*
unsigned int readEpoch(char ch, int d);
unsigned int readEpochCheck(char ch,int d);
void writeEpoch(char ch, unsigned data, int d);
*/
double BD1_readCodePhase(char ch);
void BD1_writeCodeFreq(char ch, double data);
void BD1_writeCodePhase(char ch, double data);
double BD1_readCarrierPhase(char ch);
void BD1_writeCarrierFreq(char ch, double data);

void BD1_ReadAccumData(char ch);		// new function written by Ning LUO
void BD1_ch_accum_reset(char ch);
void BD1_ch_code_slew(char ch, int data);
void BD1_all_code_slew(int data);
void BD1_programTIC(long data);
//void writeEpoch(char ch,unsigned int data);

#endif