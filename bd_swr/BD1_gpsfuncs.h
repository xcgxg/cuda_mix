// Released on July 9, 2004

#ifndef BD1_GPSFUNCS_H
#define BD1_GPSFUNCS_H

#include "BD1_gpsstruct.h"

//inline int  BD1_bit_test_l(unsigned long, char);
BD1_SATVIS BD1_satfind(char i);
BD1_XYZ BD1_satpos_almanac(double time, char n);

BD1_ECEFT BD1_satpos_ephemeris(double t, char n);
//void get_nav_orb(FILE *RinexEPP_file, EPHEMERIS *snv);      //*debug to verify whether nav and time right or not
double BD1_atan_2(double y, double x);                           //*
BD1_ECEFT BD1_SatProcess(double settime, int n, BD1_EPHEMERIS G_ephmsg[]);  //*
BD1_ECEFT BD1_Sat_Cal_accord_orign(double t, int n, BD1_EPHEMERIS ephsv[]); //*


void BD1_read_initial_data(void);

BD1_LLH BD1_receiver_loc(void);

//void navmess(char prn,char ch);
void BD1_navmessd1(char prn, char ch);
void BD1_navmessd2(char prn, char ch);

void BD1_resolution(char ch);


int BD1_exor(char bit, long parity);

BD1_LLH BD1_ecef_to_llh(BD1_XYZ pos);

BD1_XYZ BD1_llh_to_ecef(BD1_LLH pos);

BD1_PVT BD1_pos_vel_time(int nsl);
BD1_PVT BD1_kalman_pos_vel(int nsl);
void  BD1_dops(int nsl);

double BD1_tropo_iono(char ch, double az, double el, double gps_time);

void  BD1_read_ion_utc(void);

void BD1_read_almanac(void);
void BD1_read_prn();
void BD1_write_prn();
void  BD1_read_ephemeris();

void BD1_write_almanac();

void BD1_write_ephemeris();
void BD1_write_Debug_ephemeris(int i);

void BD1_write_ion_utc();

long BD1_fix_sqrt(long x);

#define SCALED_PI_ON_2  25736L
#define SCALED_PI       51472L

int BD1_matinv(double **q, int n);
void BD1_mult_ATB(double **A, int arows, int acols, double **B, int brows,
                                  int bcols, double **C);
double** BD1_dmatrix(unsigned row, unsigned column);

void BD1_free_dmatrix(double **m, unsigned row);
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
int BD1_CoRelation_D1(int fifo[], int n, int v[]);       //2007.05.06
int BD1_judge_D1(int v);         //2007.05.06
int BD1_check_D1(int v[], int n);         //2007.05.06
void BD1_NHDecoder_D1(int n, char bit);     //2007.05.06
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
#endif

