// Released on July 9, 2004

#ifndef GLO1_GPSFUNCS_H
#define GLO1_GPSFUNCS_H

#include "GLO1_gpsstruct.h"


extern GLO1_glonass_ephemeris  GLO1_glonass_sv_id_ephemeris[GLONASS_SV_TOTAL_NUM];
extern GLO1_glonass_almanac_str5 GLO1_glonass_sv_id_almanac_str5[GLONASS_SV_TOTAL_NUM];

inline int  GLO1_bit_test_l(unsigned long, char);
GLO1_SATVIS GLO1_satfind(char i);
GLO1_XYZ GLO1_satpos_almanac(double time, char n);

GLO1_ECEFT GLO1_satpos_ephemeris(double t, char n);

GLO1_ECEFT GLO1_glonass_process_sat_pvt_L(int k, GLO1_glonass_ephemeris *pEph,
	GLO1_glonass_almanac_str5 *palm_str5,
	fp64 xmit_time);

void GLO1_read_initial_data(void);

GLO1_LLH GLO1_receiver_loc(void);

void GLO1_navmess(char prn, char ch);

void GLO1_glonass_explain_string_data(int CH, int32u origin_string_data[], GLO1_glonass_ephemeris *pephem,
	GLO1_glonass_almanac *palc,
	GLO1_glonass_almanac_str5 *palc_str5,
	GLO1_glonass_almanac_global *palc_glob,
	GLO1_CHANNEL *pchan,
	int16s	*unpack_glonass_flag,

	int16s *glonass_ephemeris_processing,
	int32u *glonass_frame_id,
	int16s *glonass_almanac_processing,
	int16s *glonass_almanac_global_processing,
	bool *string1);

void  GLO1_parity_check(void);
int GLO1_exor(char bit, long parity);

GLO1_LLH GLO1_ecef_to_llh(GLO1_XYZ pos);

GLO1_XYZ llh_to_ecef(GLO1_LLH pos);

GLO1_PVT  GLO1_pos_vel_time(int nsl);
GLO1_PVT GLO1_kalman_pos_vel(int nsl);
void  GLO1_dops(int nsl);

double GLO1_tropo_iono(char ch, double az, double el, double gps_time);

void  GLO1_read_ion_utc(void);

void GLO1_read_almanac(void);
void GLO1_read_prn();
void GLO1_write_prn();
void  GLO1_read_ephemeris();

void GLO1_write_almanac();

void GLO1_write_ephemeris();
void GLO1_write_Debug_ephemeris(int i);

void GLO1_write_ion_utc();

long GLO1_fix_sqrt(long x);

#define GLO1_SCALED_PI_ON_2  25736L
#define GLO1_SCALED_PI       51472L

int GLO1_matinv(double **q, int n);
void GLO1_mult_ATB(double **A, int arows, int acols, double **B, int brows,
                                  int bcols, double **C);
double** GLO1_dmatrix(unsigned row, unsigned column);

void GLO1_free_dmatrix(double **m, unsigned row);

#endif

