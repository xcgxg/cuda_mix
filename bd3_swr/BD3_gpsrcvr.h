// Released on July 9, 2004
#ifndef BD3_GPSRCVR_H
#define BD3_GPSRCVR_H

#include "BD3_gpsstruct.h"
#include <stdio.h>

long BD3_sim_main_init();
void BD3_sim_main(long);
void BD3_sim_end();

void BD3_Sim_BDD2B1_Interrupt();

void BD3_read_rcvr_par(void);
void BD3_display(void);
void BD3_send_sim_res_socket();
void BD3_bdd2b1pream(char, char);
void BD3_nav_fix(void);
void BD3_initTrackLoopPar();
void BD3_ch_alloc();
void BD3_hot_ch_alloc();

void  BD3_velocity(void);
inline int BD3_bit_test(int, char);
void BD3_ch_acq(char);
void BD3_ch_confirm(char);
void BD3_ch_pull_in(char);
void BD3_bitSync(char);
void BD3_ch_track(char);
inline int BD3_sign(long);
int BD3_xors(long);

void BD3_undointerlace(unsigned long * x);
int BD3_checkf(unsigned long x, int flag);
int BD3_check(unsigned long x, int flag);
void BD3_shift_data(unsigned long *data30bit);
void BD3_shift_data2(unsigned long *data30bit);
int BD3_bch_decode(unsigned long shift_datas);
////////////////////////////
////////////////////////////
////////////////////////////
void BD3_bitSync_D1(char);
void BD3_NHDecoderd_D1(int, char);
void BD3_pream_D1(char, char);
void BD3_ch_pull_in_D1(char);
void BD3_ch_track_D1(char);
int BD3_checkf_D1(unsigned long x, int flag);
int BD3_check_D1(unsigned long x, int flag);
////////////////////////////
////////////////////////////
////////////////////////////
void BD3_get_nav_orb(FILE *RinexEPP_file, BD3_EPHEMERIS *snv);
int BD3_read_RinexEPP(FILE *RinexEPP_file, BD3_EPHEMERIS *snv);//*

////////////////////////////
//void navmessd1(char prn,char ch);
//void navmessd2(char prn,char ch);
////////////////////////////
#endif

