// Released on July 9, 2004

#ifndef GPSFUNCS_H
#define GPSFUNCS_H

inline int  bit_test_l(unsigned long ,char);
SATVIS satfind(char i);
XYZ satpos_almanac(double time, char n);

ECEFT satpos_ephemeris(double t,char n);

void read_initial_data(void);

LLH receiver_loc(void);

void navmess(char prn,char ch);

void  parity_check(void);
int exor(char bit, long parity);

LLH ecef_to_llh(XYZ pos);

XYZ llh_to_ecef(LLH pos);

PVT  pos_vel_time(int nsl);
PVT kalman_pos_vel(int nsl);
void  dops( int nsl);

double tropo_iono(char ch,double az,double el,double gps_time);

void  read_ion_utc(void);

void read_almanac(void);
void read_prn();
void write_prn();
void  read_ephemeris();

void write_almanac();

void write_ephemeris();
void write_Debug_ephemeris(int i);

void write_ion_utc();

long fix_sqrt(long x);

#define SCALED_PI_ON_2  25736L
#define SCALED_PI       51472L

int matinv( double **q, int n);
void mult_ATB(double **A, int arows, int acols, double **B, int brows,
                                  int bcols, double **C);
double** dmatrix(unsigned row, unsigned column);

void free_dmatrix( double **m, unsigned row);

#endif

