// Released on July 9, 2004

#ifndef BD3_GPSFUNCS_H
#define BD3_GPSFUNCS_H

#include "BD3_gpsstruct.h"

inline int  BD3_bit_test_l(unsigned long, char);
BD3_SATVIS BD3_satfind(char i);
BD3_XYZ BD3_satpos_almanac(double time, char n);

BD3_ECEFT BD3_satpos_ephemeris(double t, char n);
//void get_nav_orb(FILE *RinexEPP_file, EPHEMERIS *snv);      //*debug to verify whether nav and time right or not
double BD3_atan_2(double y, double x);                           //*
BD3_ECEFT BD3_SatProcess(double settime, int n, BD3_EPHEMERIS G_ephmsg[]);  //*
BD3_ECEFT BD3_Sat_Cal_accord_orign(double t, int n, BD3_EPHEMERIS ephsv[]); //*


void BD3_read_initial_data(void);

BD3_LLH BD3_receiver_loc(void);

//void navmess(char prn,char ch);
void BD3_navmessd1(char prn, char ch);
void BD3_navmessd2(char prn, char ch);

void BD3_resolution(char ch);


int BD3_exor(char bit, long parity);

BD3_LLH BD3_ecef_to_llh(BD3_XYZ pos);

BD3_XYZ BD3_llh_to_ecef(BD3_LLH pos);

BD3_PVT  BD3_pos_vel_time(int nsl);
BD3_PVT BD3_kalman_pos_vel(int nsl);
void  BD3_dops(int nsl);

double BD3_tropo_iono(char ch, double az, double el, double gps_time);

void  BD3_read_ion_utc(void);

void BD3_read_almanac(void);
void BD3_read_prn();
void BD3_write_prn();
void  BD3_read_ephemeris();

void BD3_write_almanac();

void BD3_write_ephemeris();
void BD3_write_Debug_ephemeris(int i);

void BD3_write_ion_utc();

long BD3_fix_sqrt(long x);

#define BD3_SCALED_PI_ON_2  25736L
#define BD3_SCALED_PI       51472L

int BD3_matinv(double **q, int n);
void BD3_mult_ATB(double **A, int arows, int acols, double **B, int brows,
                                  int bcols, double **C);
double** BD3_dmatrix(unsigned row, unsigned column);

void BD3_free_dmatrix(double **m, unsigned row);
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
int BD3_CoRelation_D1(int fifo[], int n, int v[]);       //2007.05.06
int BD3_judge_D1(int v);         //2007.05.06
int BD3_check_D1(int v[], int n);         //2007.05.06
void BD3_NHDecoder_D1(int n, char bit);     //2007.05.06
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
#endif

