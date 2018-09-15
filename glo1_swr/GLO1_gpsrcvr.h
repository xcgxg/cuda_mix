// Released on July 9, 2004
#ifndef GLO1_GPSRCVR_H
#define GLO1_GPSRCVR_H


#ifdef REAL_TIME
void interrupt GPS_Interrupt(...);
#else
void GLO1_Sim_GPS_Interrupt();
#endif

long GLO1_sim_main_init();
void GLO1_sim_main(long);
void GLO1_sim_end();
void GLO1_send_sim_res_socket();

void GLO1_read_rcvr_par(void);
void GLO1_display(void);
void GLO1_pream(char, char);
void GLO1_nav_fix(void);
void GLO1_initTrackLoopPar();
int GLO1_freq_alloc(char j);//todo:ÉùÃ÷
void GLO1_ch_alloc();
void GLO1_hot_ch_alloc();
void  GLO1_velocity(void);
inline int GLO1_bit_test(int, char);
void GLO1_ch_acq(char);
void GLO1_ch_confirm(char);
void GLO1_ch_pull_in(char);
void GLO1_bitSync(char);
void GLO1_ch_track(char);
inline int GLO1_sign(long);
int GLO1_xors(long);

#endif