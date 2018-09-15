// Released on July 9, 2004
#ifndef GPSRCVR_H
#define GPSRCVR_H

#ifdef REAL_TIME
void interrupt GPS_Interrupt(...);
#else
void Sim_GPS_Interrupt();
#endif
void read_rcvr_par(void);
void display(void);
void send_sim_res_socket();
void pream(char ,char );
void nav_fix(void);
void initTrackLoopPar();
void ch_alloc();
void hot_ch_alloc();
void  velocity(void);
inline int bit_test(int,char);
void ch_acq(char);
void ch_confirm(char);
void ch_pull_in(char);
void bitSync(char);
void ch_track(char);
inline int sign(long);
int xors(long);

long GPSL1_sim_main_init();
void GPSL1_sim_main(long);
void GPSL1_sim_end();

#endif