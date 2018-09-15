// Released on July 9, 2004
#ifndef BD1_GPSRCVR_H
#define BD1_GPSRCVR_H

#include "BD1_gpsstruct.h"


void BD1_Sim_BDD2B1_Interrupt();

void BD1_read_rcvr_par(void);
void BD1_send_sim_res_socket();
void BD1_display(void);
void BD1_bdd2b1pream(char, char);
void BD1_nav_fix(void);
void BD1_initTrackLoopPar();
void BD1_ch_alloc();
void BD1_hot_ch_alloc();

void  BD1_velocity(void);
inline int BD1_bit_test(int, char);
void BD1_ch_acq(char);
void BD1_ch_confirm(char);
void BD1_ch_pull_in(char);
void BD1_bitSync(char);
void BD1_ch_track(char);
inline int BD1_sign(long);
int BD1_xors(long);

void BD1_undointerlace(unsigned long * x);
int BD1_checkf(unsigned long x, int flag);
int BD1_check(unsigned long x, int flag);
void BD1_shift_data(unsigned long *data30bit);
void BD1_shift_data2(unsigned long *data30bit);
int BD1_bch_decode(unsigned long shift_datas);
////////////////////////////
////////////////////////////
////////////////////////////
void BD1_bitSync_D1(char);
void BD1_NHDecoderd_D1(int, char);
void BD1_pream_D1(char, char);
void BD1_ch_pull_in_D1(char);
void BD1_ch_track_D1(char);
int BD1_checkf_D1(unsigned long x, int flag);
int BD1_check_D1(unsigned long x, int flag);
////////////////////////////
////////////////////////////
////////////////////////////
void BD1_get_nav_orb(FILE *RinexEPP_file, BD1_EPHEMERIS *snv);
int BD1_read_RinexEPP(FILE *RinexEPP_file, BD1_EPHEMERIS *snv);//*


long BD1_sim_main_init();
void BD1_sim_main(long);
void BD1_sim_end();

////////////////////////////
//void navmessd1(char prn,char ch);
//void navmessd2(char prn,char ch);
////////////////////////////
#endif

