// Released on July 9, 2004
#ifndef GP2021_H
#define GP2021_H

inline void to_gps(int add,int data);
inline int from_gps(int add);
inline int accum_status(void);
void all_accum_reset(void);
void data_tst(int data);
unsigned int readEpoch(char ch);
unsigned int readEpochCheck(char ch);
void writeEpoch(char ch, unsigned data);
double readCodePhase(char ch);
void writeCodeFreq(char ch, double data);
void writeCodePhase(char ch, double data);
double readCarrierPhase(char ch);
void writeCarrierFreq(char ch, double data);
void ch_cntl(char ch,int data);
void ReadAccumData(char ch);		// new function written by Ning LUO
void ch_accum_reset(char ch);
void ch_code_slew(char ch,int data);
void all_code_slew(int data);
void programTIC(long data);
void writeEpoch(char ch,unsigned int data);

#endif