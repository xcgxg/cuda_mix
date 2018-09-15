// Released on July 9, 2004
#ifndef BD3_GP2021_H
#define BD3_GP2021_H

inline void BD3_to_gps(int add, int data);
inline int BD3_from_gps(int add);
inline int BD3_accum_status(void);
void BD3_all_accum_reset(void);
void BD3_data_tst(int data);
//2010.11.22
unsigned int BD3_readEpoch(char ch);
unsigned int BD3_readEpochCheck(char ch);
void BD3_writeEpoch(char ch, unsigned data);
/*
unsigned int readEpoch(char ch, int d);
unsigned int readEpochCheck(char ch,int d);
void writeEpoch(char ch, unsigned data, int d);
*/
double BD3_readCodePhase(char ch);
void BD3_writeCodeFreq(char ch, double data);
void BD3_writeCodePhase(char ch, double data);
double BD3_readCarrierPhase(char ch);
void BD3_writeCarrierFreq(char ch, double data);

void BD3_ReadAccumData(char ch);		// new function written by Ning LUO
void BD3_ch_accum_reset(char ch);
void BD3_ch_code_slew(char ch, int data);
void BD3_all_code_slew(int data);
void BD3_programTIC(long data);
//void writeEpoch(char ch,unsigned int data);

#endif