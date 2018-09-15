// Released on July 9, 2004
#ifndef GLO1_GP2021_H
#define GLO1_GP2021_H

inline void GLO1_to_gps(int add, int data);
inline int GLO1_from_gps(int add);
inline int GLO1_accum_status(void);
void GLO1_all_accum_reset(void);
void GLO1_data_tst(int data);
unsigned int GLO1_readEpoch(char ch);
unsigned int GLO1_readEpochCheck(char ch);
void GLO1_writeEpoch(char ch, unsigned data);
double GLO1_readCodePhase(char ch);
void GLO1_writeCodeFreq(char ch, double data);
void GLO1_writeCodePhase(char ch, double data);
double GLO1_readCarrierPhase(char ch);
void GLO1_writeCarrierFreq(char ch, double data);
void GLO1_ch_cntl(char ch, int data);
void GLO1_ReadAccumData(char ch);		// new function written by Ning LUO
void GLO1_ch_accum_reset(char ch);
void GLO1_ch_code_slew(char ch, int data);
void GLO1_all_code_slew(int data);
void GLO1_programTIC(long data);
void GLO1_writeEpoch(char ch, unsigned int data);

#endif