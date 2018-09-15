// Released on July 9, 2004
#include "../stdafx.h"
#include "../core/Eigen/Eigen"
#include "../core/Eigen/Dense"
using namespace Eigen;
// system header files
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <io.h>
#include <math.h>

// user header files
#include "GLO1_gpsstruct.h"
#include "GLO1_gpsconst.h"
#include "GLO1_gpsfuncs.h"
#include "GLO1_gp2021.h"

// external variables
extern GLO1_CHANNEL	GLO1_chan[12];
extern int		GLO1_out_debug;
extern FILE*	GLO1_fpeph;

// global variables
char			GLO1_tzstr[40];				
time_t			GLO1_thetime;					
FILE			*GLO1_stream,*GLO1_debug,*GLO1_in,*GLO1_out,*GLO1_kalm;	
FILE            *GLO1_fprn_in,*GLO1_fprn_out;
GLO1_ALMANAC			GLO1_gps_alm[33];
GLO1_EPHEMERIS		GLO1_gps_eph[33];



GLO1_AVAILABLESV      GLO1_availablesv;

GLO1_glonass_ephemeris  GLO1_glonass_sv_id_ephemeris[GLONASS_SV_TOTAL_NUM] = {};
GLO1_glonass_almanac_str5 GLO1_glonass_sv_id_almanac_str5[GLONASS_SV_TOTAL_NUM] = {};

int				GLO1_SVh[33],GLO1_ASV[33];			
double			GLO1_b0,GLO1_b1,GLO1_b2,GLO1_b3,GLO1_al0,GLO1_al1,GLO1_al2,GLO1_al3;	
double			GLO1_a0,GLO1_a1,GLO1_tot,GLO1_WNt,GLO1_dtls,GLO1_WNlsf,GLO1_DN,GLO1_dtlsf;

GLO1_PVT				GLO1_rpvt;
GLO1_STATE			GLO1_receiver;

double			GLO1_gdop, GLO1_pdop, GLO1_hdop, GLO1_vdop, GLO1_tdop, GLO1_alm_toa;
unsigned long	GLO1_clock_tow;
GLO1_LLH				GLO1_rec_pos_llh;
GLO1_LLH				GLO1_current_loc, GLO1_rp_llh;
GLO1_ECEFT			GLO1_track_sat[13];
GLO1_XYZ				GLO1_rec_pos_xyz;
int				GLO1_alm_gps_week,GLO1_gps_week,GLO1_almanac_valid,GLO1_almanac_flag,GLO1_handle;
unsigned long	GLO1_sf[6][11];						

int				GLO1_p_error[6];						
												

int				GLO1_status;							

unsigned long	GLO1_test_l[33]={0x00000000L,            // single bit set numbers
  0x00000001L,0x00000002L,0x00000004L,0x00000008L,  // for testing bit positions
  0x00000010L,0x00000020L,0x00000040L,0x00000080L,	// 0000 0001 0010 0100 1000
  0x00000100L,0x00000200L,0x00000400L,0x00000800L,	// 32位的第1位到第32位
  0x00001000L,0x00002000L,0x00004000L,0x00008000L,
  0x00010000L,0x00020000L,0x00040000L,0x00080000L,
  0x00100000L,0x00200000L,0x00400000L,0x00800000L,
  0x01000000L,0x02000000L,0x04000000L,0x08000000L,
  0x10000000L,0x20000000L,0x40000000L,0x80000000L};

double		GLO1_mask_angle;
char		GLO1_header[45],GLO1_ttext[27],GLO1_trailer;
double		GLO1_meas_dop[13];							// 测量的每个通道的多普勒
GLO1_XYZ			GLO1_d_sat[13];								// 每个通道的卫星的速度
double		GLO1_carrier_ref;				// 载波标称中频，单位：Hz
//double		code_ref=1023000;						// CA码速率标频，单位：chip/s
double		GLO1_code_ref=511000;						// CA码速率标频，单位：chip/s

double		GLO1_dt[13];									// 从卫星到接收机的传播时间差
double		GLO1_cbias;									// 接收机钟差
double      GLO1_lambda[13];
// binary constants for nav message decoding

double GLO1_c_2p12 = 4096;
double GLO1_c_2p4  = 16;
double GLO1_c_2m5  = 0.03125;
double GLO1_c_2m11 = 4.8828125e-4;
double GLO1_c_2m19 = 1.9073486328125e-6;
double GLO1_c_2m20 = 9.5367431640625e-7;
double GLO1_c_2m21 = 4.76837158203125e-7;
double GLO1_c_2m23 = 1.19209289550781e-7;
double GLO1_c_2m24 = 5.96046447753906e-8;
double GLO1_c_2m27 = 7.45058059692383e-9;
double GLO1_c_2m29 = 1.86264514923096e-9;
double GLO1_c_2m30 = 9.31322574615479e-10;
double GLO1_c_2m31 = 4.65661287307739e-10;
double GLO1_c_2m33 = 1.16415321826935E-10;
double GLO1_c_2m38 = 3.63797880709171e-12;
double GLO1_c_2m43 = 1.13686837721616e-13;
double GLO1_c_2m50 = 8.881784197e-16;
double GLO1_c_2m55 = 2.77555756156289e-17;

int			GLO1_m_tropo, GLO1_m_iono;				// flags for using tropo and iono models
int			GLO1_align_t;						// 将接收机时间和GPS时间对准的标志
GLO1_SATVIS		GLO1_xyz[33];						// 卫星的位置、方位和多普勒

char GLO1_last_prn[12];
/*******************************************************************************
FUNCTION satfind()
RETURNS  None.

PARAMETERS None.

PURPOSE

	THIS FUNCTION DETERMINES THE SATELLITES TO SEARCH FOR
	WHEN ALMANAC DATA IS AVAILABLE
	输入：卫星的标号
	输出：可视卫星的位置、仰角以及多普勒
	这个函数在历书有效时决定哪些去搜索哪些卫星
*******************************************************************************/
GLO1_SATVIS GLO1_satfind(char i)
{
	double	tdot,az;
	double	satang,alm_time,almanac_date;
	double	range1,range2,xls,yls,zls,xaz,yaz, rangeInv;
	long	jd_yr;
	GLO1_XYZ		gpspos1,gpspos2,north,east,up;
	GLO1_SATVIS	result;
	int		jd_m;
	struct	tm *gmt;
	double  time_s;

    
	// INITIALIZE ALL THE CONSTANTS
	// 防止返回结果混乱
	result.azimuth = result.doppler = result.elevation = 0.0;
	result.x = result.y = result.z = 0.0;

	_putenv(GLO1_tzstr);

	_tzset();
	gmt=gmtime(&GLO1_thetime);/*转换为struct tm结构的UTC时间*/
	if (gmt->tm_mon <= 1)// set up the correct time
	{	
		jd_yr = (long) (365.25*(gmt->tm_year-1.+1900.));
		jd_m  = (int) (30.6001*(gmt->tm_mon+14.));
	}
	else
	{
		jd_yr = (long) (365.25*(gmt->tm_year+1900.));
		jd_m = (int) (30.6001*(gmt->tm_mon+2.));
	}
	time_s=gmt->tm_min/1440.+gmt->tm_sec/86400.+1720981.5+gmt->tm_hour/24.
		+jd_yr+jd_m+gmt->tm_mday;
	GLO1_gps_week=int((time_s-2444244.5)/7.);// 估计GPS的星期数
	almanac_date=GLO1_gps_alm[i].week*7.0+2444244.5;

	if (GLO1_gps_week-GLO1_gps_alm[i].week>512) almanac_date+=1024*7.0;
	alm_time=(time_s-almanac_date)*86400.;//86400是一天的秒数(24*60*60)  这里计算TOW，在本周内的第多少秒
	GLO1_clock_tow=(long)((time_s-GLO1_gps_week*7.-2444244.5)*86400.);// 估计GPS的秒数,值应在0-604799之间，即一周的秒数.
	if (GLO1_gps_alm[i].inc > 0.0 && i>0)
    {
		gpspos1 = GLO1_satpos_almanac(alm_time, i);
		gpspos2 = GLO1_satpos_almanac(alm_time + 1.0, i);

		GLO1_rec_pos_xyz=llh_to_ecef(GLO1_current_loc);
		// 计算将WGS-84的坐标转换为本地坐标系的旋转矩阵
		north.x = -cos(GLO1_current_loc.lon)*sin(GLO1_current_loc.lat);
		north.y = -sin(GLO1_current_loc.lon)*sin(GLO1_current_loc.lat);
		north.z = cos(GLO1_current_loc.lat);
		east.x = -sin(GLO1_current_loc.lon);
		east.y = cos(GLO1_current_loc.lon);
		up.x = cos(GLO1_current_loc.lon)*cos(GLO1_current_loc.lat);
		up.y = sin(GLO1_current_loc.lon)*cos(GLO1_current_loc.lat);
		up.z = sin(GLO1_current_loc.lat);

		xls =gpspos1.x-GLO1_rec_pos_xyz.x;
		yls =gpspos1.y-GLO1_rec_pos_xyz.y;
		zls =gpspos1.z-GLO1_rec_pos_xyz.z;
		range1=sqrt(xls*xls+yls*yls+zls*zls);
		rangeInv = 1.0/range1;
		xls *= rangeInv;	// 计算卫星位置相对与接收机位置的单位矢量
		yls *= rangeInv;
		zls *= rangeInv;

		tdot=up.x*xls+up.y*yls+up.z*zls;
		range2=sqrt(pow(gpspos2.x-GLO1_rec_pos_xyz.x-GLO1_rpvt.xv,2)+
					pow(gpspos2.y-GLO1_rec_pos_xyz.y-GLO1_rpvt.yv,2)+
					pow(gpspos2.z-GLO1_rec_pos_xyz.z-GLO1_rpvt.zv,2));
		if ( tdot >= 1.00 ) 
			satang=GLO1_pi/2.0;
		else if ( tdot <= -1.00 ) 
			satang=-GLO1_pi/2.0;
		else  
			satang=asin(tdot);

		xaz=east.x*xls+east.y*yls;
		yaz=north.x*xls+north.y*yls+north.z*zls;
		if (xaz !=0.0 || yaz !=0.0) 
			az=atan2(xaz,yaz);
		else 
			az=0.0; 

		result.x=gpspos1.x;
		result.y=gpspos1.y;
		result.z=gpspos1.z;
		result.elevation=satang;
		result.azimuth  =az;
		result.doppler  =(range1-range2)*5.2514;
	}
	return(result);
}


/*******************************************************************************
FUNCTION satpos_almanac(float time, char n)
RETURNS  None.

PARAMETERS
			time   float  time of week
			n      char   satellite prn

PURPOSE

	  THIS SUBROUTINE CALCULATES THE SATELLITE POSITION
	  BASED ON ALMANAC DATA
******************************************************************************/

GLO1_XYZ GLO1_satpos_almanac(double time, char n)
{
	double ei,ea,diff,r,ta,la,aol,xp,yp,d_toa;
	GLO1_XYZ result;

	
	d_toa=time-GLO1_gps_alm[n].toa;
	if (d_toa>302400.0) d_toa=d_toa-604800.0;
	else if (d_toa<-302400.0)d_toa=d_toa+604800.0;
	ei=GLO1_gps_alm[n].ma+d_toa*GLO1_gps_alm[n].w;
    ea=ei;	
	do
	{
       diff= (ei-(ea-GLO1_gps_alm[n].ety*sin(ea)))/(1.-GLO1_gps_alm[n].ety*cos(ea));
       ea += diff;
	} while (fabs(diff) > 1.0e-6);
      
	if (GLO1_gps_alm[n].ety != 0.0 )	
		ta=atan2(sqrt(1.-pow(GLO1_gps_alm[n].ety,2))*sin(ea),cos(ea)-GLO1_gps_alm[n].ety);
	else ta=ea;
	r=pow(GLO1_gps_alm[n].sqa,2)*(1.-pow(GLO1_gps_alm[n].ety,2)*cos(ea));
    //  AOL IS THE ARGUMENT OF LATITUDE
    aol=ta+GLO1_gps_alm[n].aop;

	//LA IS THE LONGITUDE OF THE ASCENDING NODE
	la=GLO1_gps_alm[n].lan+(GLO1_gps_alm[n].rra-GLO1_omegae)*d_toa-GLO1_gps_alm[n].toa*GLO1_omegae;
    xp=r*cos(aol);
    yp=r*sin(aol);
    result.x=xp*cos(la)-yp*cos(GLO1_gps_alm[n].inc)*sin(la);
    result.y=xp*sin(la)+yp*cos(GLO1_gps_alm[n].inc)*cos(la);
    result.z=yp*sin(GLO1_gps_alm[n].inc);
	return(result);
}

/*******************************************************************************
FUNCTION satpos_ephemeris(double t, char n)
RETURNS  None.

PARAMETERS
			t  double   time of week
			n  char     satellite prn

PURPOSE

     THIS SUBROUTINE CALCULATES THE SATELLITE POSITION
     BASED ON BROADCAST EPHEMERIS DATA

	  R    - RADIUS OF SATELLITE AT TIME T
     Crc  - RADIUS COSINE CORRECTION TERM
     Crs  - RADIUS SINE   CORRECTION TERM
     SLAT - SATELLITE LATITUDE
     SLONG- SATELLITE LONGITUDE
     TOE  - TIME OF EPHEMERIS FROM START OF WEEKLY EPOCH
	  ETY  - ORBITAL INITIAL ECCENTRICITY
	  TOA  - TIME OF APPLICABILITY FROM START OF WEEKLY EPOCH
     INC  - ORBITAL INCLINATION
     IDOT - RATE OF INCLINATION ANGLE
     CUC  - ARGUMENT OF LATITUDE COSINE CORRECTION TERM
     CUS  - ARGUMENT OF LATITUDE SINE   CORRECTION TERM
     CIC  - INCLINATION COSINE CORRECTION TERM
     CIS  - INCLINATION SINE   CORRECTION TERM
     RRA  - RATE OF RIGHT ASCENSION
     SQA  - SQUARE ROOT OF SEMIMAJOR AXIS
     LAN  - LONGITUDE OF NODE AT WEEKLY EPOCH
     AOP  - ARGUMENT OF PERIGEE
     MA   - MEAN ANOMALY AT TOA
     DN   - MEAN MOTION DIFFERENCE

*******************************************************************************/

inline fp64 GLO1_correct_angle(fp64 angle)
{
	fp64 new_angle;
	int32s integral_part;
	int32s sign_of_angle = 1;
	/*presume that the input angle's input is more than 0 */
	if (angle < 0.0)
	{
		sign_of_angle = -1;//angle;s value is negative
		angle = -angle;
	}
	integral_part = angle / (2 * GLO1_pi);
	new_angle = angle - integral_part * (2 * GLO1_pi);
	if (sign_of_angle == -1)
	{
		new_angle = (2 * GLO1_pi) - new_angle;
	}
	return new_angle;
}
/*
*　把PZ-90下的卫星坐标变换到WGS-84下，具体参见系统坐标统一相关文档
*/
void GLO1_PZ90_to_WGS84(GLO1_ECEFT *pPos, GLO1_ECEFT *pVel)
{
	GLO1_ECEFT tpos, tvel;
	fp64 compact = GLO1_UNIT + GLO1_MM;

	tpos.x = GLO1_DX + compact * (pPos->x + GLO1_WZ * pPos->y - GLO1_WY * pPos->z);
	tpos.y = GLO1_DY + compact * (-GLO1_WZ * pPos->x + pPos->y + GLO1_WX * pPos->z);
	tpos.z = GLO1_DZ + compact * (GLO1_WY * pPos->x - GLO1_WX * pPos->y + pPos->z);

	tvel.vx = compact * (pVel->vx + GLO1_WZ * pVel->vy - GLO1_WY * pVel->vz);
	tvel.vy = compact * (-GLO1_WZ * pVel->vx + pVel->vy + GLO1_WX * pVel->vz);
	tvel.vz = compact * (GLO1_WY * pVel->vx - GLO1_WX * pVel->vy + pVel->vz);

	pPos->x = tpos.x;
	pPos->y = tpos.y;
	pPos->z = tpos.z;
	pVel->vx = tvel.vx;
	pVel->vy = tvel.vy;
	pVel->vz = tvel.vz;
}

#define GLO1_U_GLO			398600.44e+09	/* 立方米/平方秒 Gravitational constant */
#define GLO1_Ae_GLO			6378136			/* 米 Semi-major axis of Earth */
#define GLO1_J02_GLO			1082625.7e-09	/*  */
#define GLO1_OMIGA_GLO		7.29115e-05		/* radian/s Earth rotation rate */
#define GLO1_C20         	-1082.63e-6
#define 	GLO1_RK_STEP_LEN					1	/*龙格库塔步长 （秒）*/

/*
* 根据星历计算GLONASS卫星的位置、速度、钟差和钟漂——具体参见GLONASS ICD文件
*/
fp64 GLO1_cx[5], GLO1_cy[5], GLO1_cz[5];
fp64 GLO1_dx[5], GLO1_dy[5], GLO1_dz[5];
fp64 GLO1_ux[5], GLO1_uy[5], GLO1_uz[5];
fp64 GLO1_px[5], GLO1_py[5], GLO1_pz[5];
GLO1_ECEFT GLO1_glonass_process_sat_pvt_L(int k, GLO1_glonass_ephemeris *pEph,
	GLO1_glonass_almanac_str5 *palm_str5,
	fp64 xmit_time)
{
	double	la, xp, yp, bclk, tc, d_toc, d_toe;
	double	xls, yls, zls, range1, tdot, satang, xaz, yaz;
	double	az;
	GLO1_XYZ		north, east, up;
	//GLO1_ECEFT *pSat_pos;
	//GLO1_ECEFT *pSat_vel;
	GLO1_ECEFT *pSat_pos = new GLO1_ECEFT;
	GLO1_ECEFT *pSat_vel = new GLO1_ECEFT;
	//fp64 pax = pEph->doudot_Xn, pay = pEph->doudot_Yn, paz = pEph->doudot_Zn;
	fp64 pax = GLO1_glonass_sv_id_ephemeris[k+10].doudot_Xn,
		pay = GLO1_glonass_sv_id_ephemeris[k+10].doudot_Yn,
		paz = GLO1_glonass_sv_id_ephemeris[k+10].doudot_Zn;
	fp64 x, y, z;
	fp64 vx, vy, vz;
	fp64 up3, Jup, r, pzpr;

	/*st-atic fp64 cx[5], cy[5], cz[5];
	st-atic fp64 dx[5], dy[5], dz[5];
	sta-tic fp64 ux[5], uy[5], uz[5];
	st-atic fp64 px[5], py[5], pz[5];*/

	fp64 M, M1;
	fp64 JupC;
	fp64 h;
	fp64 dw;

	int32s 	i, j, nTime;
	fp64 	GLO1_dt;

	fp64	dt_SV;

	fp64   	nt_tb, nt, odd;
	fp64 	J02 = GLO1_J02_GLO;
	fp64 	ae = GLO1_Ae_GLO;
	fp64 	u = GLO1_U_GLO;
	fp64 	w = GLO1_OMIGA_GLO;
	fp64 	pw2 = GLO1_OMIGA_GLO*GLO1_OMIGA_GLO;
	GLO1_ECEFT	result;
	//	xyzt_t  old_pos;

	//	old_pos.x=pSat_pos->x;
	//	old_pos.y=pSat_pos->y;
	//	old_pos.z=pSat_pos->z;

	result.x = 0.0;
	result.y = 0.0;
	result.z = 0.0;
	result.tb = 0.0;
	//pSat_pos->iftrue = 0;

	result.vx = 0.0;
	result.vy = 0.0;
	result.vz = 0.0;
	//pSat_vel->tb = 0.0;
	//pSat_vel->iftrue = 0;

	if ((GLO1_glonass_sv_id_ephemeris[k + 10].Bn & 0x0001) == 1)
	{

	}
	nt = xmit_time;
	if (nt<0)
		nt += 86400;

	/*x = pEph->Xn;
	y = pEph->Yn;
	z = pEph->Zn;

	vx = pEph->dot_Xn;
	vy = pEph->dot_Yn;
	vz = pEph->dot_Zn;
	nt_tb = nt - pEph->tb;*/

	x = GLO1_glonass_sv_id_ephemeris[k + 10].Xn;
	y = GLO1_glonass_sv_id_ephemeris[k + 10].Yn;
	z = GLO1_glonass_sv_id_ephemeris[k + 10].Zn;

	vx = GLO1_glonass_sv_id_ephemeris[k + 10].dot_Xn;
	vy = GLO1_glonass_sv_id_ephemeris[k + 10].dot_Yn;
	vz = GLO1_glonass_sv_id_ephemeris[k + 10].dot_Zn;
	nt_tb = nt - GLO1_glonass_sv_id_ephemeris[k + 10].tb;


	if (GLO1_glonass_sv_id_almanac_str5[k + 10].valid == 1)
	{
		dt_SV = -GLO1_glonass_sv_id_ephemeris[k + 10].taun + GLO1_glonass_sv_id_ephemeris[k + 10].gamman * nt_tb /*- GLO1_glonass_sv_id_almanac_str5[k + 10].tauc*/;
	}
	else
	{
		dt_SV = -GLO1_glonass_sv_id_ephemeris[k + 10].taun + GLO1_glonass_sv_id_ephemeris[k + 10].gamman * nt_tb;
	}

	//	dt_SV = -pEph->taun + pEph->gamman * nt_tb;

	nt_tb = nt_tb - dt_SV;

	dt_SV = -GLO1_glonass_sv_id_ephemeris[k + 10].taun + GLO1_glonass_sv_id_ephemeris[k + 10].gamman * nt_tb;	//返回以修正信号发射时间

	if (nt_tb > 43200.0)
	{
		nt_tb -= 86400.0;
	}
	else if (nt_tb < -43200.0)
	{
		nt_tb += 86400.0;
	}

	if (nt_tb>0)
		GLO1_dt = GLO1_RK_STEP_LEN;
	else
		GLO1_dt = -1 * GLO1_RK_STEP_LEN;



	JupC = 3.0 / 2.0 * J02 * u * ae*ae;
	dw = 2.0 * w;

	nTime = floor((double)((int32s)(fabs(nt_tb))) / GLO1_RK_STEP_LEN);
	odd = fabs(nt_tb) - nTime*GLO1_RK_STEP_LEN;

	GLO1_ux[1] = vx;
	GLO1_uy[1] = vy;
	GLO1_uz[1] = vz;
	GLO1_px[1] = x;
	GLO1_py[1] = y;
	GLO1_pz[1] = z;

	for (i = 0; i <= nTime; i++)
	{
		if (i == nTime)
		{
			if (GLO1_dt > 0)
				GLO1_dt = odd;
			else
				GLO1_dt = -1 * odd;
		}

		h = GLO1_dt;

		for (j = 1; j<5; j++)
		{
			switch (j) {
			case 1:
				break;
			case 2:
				GLO1_ux[j] = GLO1_ux[1] + h *  GLO1_cx[j - 1] * 0.5;
				GLO1_uy[j] = GLO1_uy[1] + h *  GLO1_cy[j - 1] * 0.5;
				GLO1_uz[j] = GLO1_uz[1] + h * GLO1_cz[j - 1] * 0.5;

				GLO1_px[j] = GLO1_px[1] + h *  GLO1_dx[j - 1] * 0.5;
				GLO1_py[j] = GLO1_py[1] + h *  GLO1_dy[j - 1] * 0.5;
				GLO1_pz[j] = GLO1_pz[1] + h * GLO1_dz[j - 1] * 0.5;
				break;
			case 3:
				GLO1_ux[j] = GLO1_ux[1] + h *  GLO1_cx[j - 1] * 0.5;
				GLO1_uy[j] = GLO1_uy[1] + h *  GLO1_cy[j - 1] * 0.5;
				GLO1_uz[j] = GLO1_uz[1] + h *  GLO1_cz[j - 1] * 0.5;

				GLO1_px[j] = GLO1_px[1] + h *  GLO1_dx[j - 1] * 0.5;
				GLO1_py[j] = GLO1_py[1] + h *  GLO1_dy[j - 1] * 0.5;
				GLO1_pz[j] = GLO1_pz[1] + h *  GLO1_dz[j - 1] * 0.5;
				break;
			case 4:
				GLO1_ux[j] = GLO1_ux[1] + h *  GLO1_cx[j - 1];
				GLO1_uy[j] = GLO1_uy[1] + h *  GLO1_cy[j - 1];
				GLO1_uz[j] = GLO1_uz[1] + h *  GLO1_cz[j - 1];

				GLO1_px[j] = GLO1_px[1] + h *  GLO1_dx[j - 1];
				GLO1_py[j] = GLO1_py[1] + h *  GLO1_dy[j - 1];
				GLO1_pz[j] = GLO1_pz[1] + h *  GLO1_dz[j - 1];
				break;
			}

			r = sqrt(GLO1_px[j] * GLO1_px[j] + GLO1_py[j] * GLO1_py[j] + GLO1_pz[j] * GLO1_pz[j]);
			up3 = u * 1.0 / (r*r*r);
			Jup = JupC / (r*r*r*r*r);
			pzpr = 5.0 *  GLO1_pz[j] * GLO1_pz[j] / (r*r);
			M = Jup * (1.0 - pzpr);
			M1 = Jup * (3.0 - pzpr);
			GLO1_dx[j] = GLO1_ux[j];
			GLO1_dy[j] = GLO1_uy[j];
			GLO1_dz[j] = GLO1_uz[j];
			GLO1_cx[j] = (-up3 - M + pw2) *  GLO1_px[j] + dw *  GLO1_uy[j] + pax;//cx[j] = (-up3 - M + pw2) * px[j] + dw * uy[j] + pax;
			GLO1_cy[j] = (-up3 - M + pw2) *  GLO1_py[j] - dw *  GLO1_ux[j] + pay;//cy[j] = (-up3 - M + pw2) * py[j] - dw * ux[j] + pay;
			GLO1_cz[j] = (-up3 - M1) *  GLO1_pz[j] + paz;//cz[j] = (-up3 - M1     ) * pz[j]  + paz;
		}

		GLO1_ux[1] = GLO1_ux[1] + h * (GLO1_cx[1] + 2.0 * (GLO1_cx[2] + GLO1_cx[3]) + GLO1_cx[4]) / 6.0;
		GLO1_uy[1] = GLO1_uy[1] + h * (GLO1_cy[1] + 2.0 * (GLO1_cy[2] + GLO1_cy[3]) + GLO1_cy[4]) / 6.0;
		GLO1_uz[1] = GLO1_uz[1] + h * (GLO1_cz[1] + 2.0 * (GLO1_cz[2] + GLO1_cz[3]) + GLO1_cz[4]) / 6.0;

		GLO1_px[1] = GLO1_px[1] + h * (GLO1_dx[1] + 2.0 * (GLO1_dx[2] + GLO1_dx[3]) + GLO1_dx[4]) / 6.0;
		GLO1_py[1] = GLO1_py[1] + h * (GLO1_dy[1] + 2.0 * (GLO1_dy[2] + GLO1_dy[3]) + GLO1_dy[4]) / 6.0;
		GLO1_pz[1] = GLO1_pz[1] + h * (GLO1_dz[1] + 2.0 * (GLO1_dz[2] + GLO1_dz[3]) + GLO1_dz[4]) / 6.0;

		vx = GLO1_ux[1];
		vy = GLO1_uy[1];
		vz = GLO1_uz[1];

		x = GLO1_px[1];
		y = GLO1_py[1];
		z = GLO1_pz[1];
	}
	

	pSat_pos->x = x;
	pSat_pos->y = y;
	pSat_pos->z = z;
	pSat_pos->tb = dt_SV;
	/*result.x = x;
	result.y = y;
	result.z = z;
	result.tb = dt_SV;*/
	//pSat_pos->iftrue = 1;

	pSat_vel->vx = vx;
	pSat_vel->vy = vy;
	pSat_vel->vz = vz;
	/*result.vx = vx;
	result.vy = vy;
	result.vz = vz;*/
	//pSat_vel->tb = 0.0;
	//pSat_vel->iftrue = 1;

	GLO1_PZ90_to_WGS84(pSat_pos, pSat_vel);

	result.x = pSat_pos->x;
	result.y = pSat_pos->y;
	result.z = pSat_pos->z;
	result.vx = pSat_vel->vx;
	result.vy = pSat_vel->vy;
	result.vz = pSat_vel->vz;
	result.tb = pSat_pos->tb;
	delete  pSat_vel;
	delete  pSat_pos;

	////保存伪距
	//char str[10];
	////itoa(sv, str, 10);
	//sprintf(str, "%d", k);
	//char s[100] = "K_";
	//char *pfilename = strcat(s, str);

	//FILE *fd = fopen(pfilename, "a");
	//fprintf(fd, "%f %f %f %f %f %f\n", result.x, result.y, result.z, result.vx, result.vy, result.vz);
	//fclose(fd);

	result.az = 0.0;
	result.el = 0.0;
	if (GLO1_rec_pos_xyz.x != 0.0 || GLO1_rec_pos_xyz.y != 0.0 || GLO1_rec_pos_xyz.z != 0.0)
	{

		north.x = -cos(GLO1_rec_pos_llh.lon)*sin(GLO1_rec_pos_llh.lat);
		north.y = -sin(GLO1_rec_pos_llh.lon)*sin(GLO1_rec_pos_llh.lat);
		north.z = cos(GLO1_rec_pos_llh.lat);
		east.x = -sin(GLO1_rec_pos_llh.lon);
		east.y = cos(GLO1_rec_pos_llh.lon);
		east.z = 0.0;
		up.x = cos(GLO1_rec_pos_llh.lon)*cos(GLO1_rec_pos_llh.lat);
		up.y = sin(GLO1_rec_pos_llh.lon)*cos(GLO1_rec_pos_llh.lat);
		up.z = sin(GLO1_rec_pos_llh.lat);


		xls = result.x - GLO1_rec_pos_xyz.x;
		yls = result.y - GLO1_rec_pos_xyz.y;
		zls = result.z - GLO1_rec_pos_xyz.z;
		range1 = sqrt(xls*xls + yls*yls + zls*zls);
		tdot = (up.x*xls + up.y*yls + up.z*zls) / range1;


		if (tdot >= 1.00) satang = GLO1_pi / 2.0;
		else if (tdot <= -1.00) satang = -GLO1_pi / 2.0;
		else satang = asin(tdot);


		xaz = east.x*xls + east.y*yls;
		yaz = north.x*xls + north.y*yls + north.z*zls;
		if (xaz != 0.0 || yaz != 0.0) az = atan2(xaz, yaz);
		else az = 0.0;
		result.el = satang;
		result.az = az;
	}
	return (result);

}

GLO1_ECEFT GLO1_satpos_ephemeris(double t, char n)
{
	double	ei,ea,diff,ta,aol,delr,delal,delinc,r,inc;
	double	la,xp,yp,bclk,tc,d_toc,d_toe;
	double	xls,yls,zls,range1,tdot,satang,xaz,yaz;
	double	az;
	GLO1_XYZ		north,east,up;
	GLO1_ECEFT	result;
	d_toc=t-GLO1_gps_eph[n].toc;
    if (d_toc>302400.0) d_toc=d_toc-604800.0;
    else if (d_toc<-302400.0)d_toc=d_toc+604800.0;
	bclk=GLO1_gps_eph[n].af0+GLO1_gps_eph[n].af1*d_toc+GLO1_gps_eph[n].af2*d_toc*d_toc
		-GLO1_gps_eph[n].tgd;
	tc=t-bclk;
    
	d_toe=tc-GLO1_gps_eph[n].toe;
    if (d_toe>302400.0) d_toe=d_toe-604800.0;
	else if (d_toe<-302400.0)d_toe=d_toe+604800.0;
	
	// mean anomaly
	ei=GLO1_gps_eph[n].ma+d_toe*(GLO1_gps_eph[n].wm+GLO1_gps_eph[n].dn);
	// iterate eccentric anomaly
    ea=ei;
    do
    {
		diff=(ei-(ea-GLO1_gps_eph[n].ety*sin(ea)))/(1.0E0-GLO1_gps_eph[n].ety*cos(ea));
		ea += diff;
	} while (fabs(diff) > 1.0e-12 );
      

	bclk -= 4.442807633E-10*GLO1_gps_eph[n].ety*GLO1_gps_eph[n].sqra*sin(ea);
	result.tb=bclk;


	ta=atan2(sqrt(1.00-pow(GLO1_gps_eph[n].ety,2))*sin(ea),cos(ea)-GLO1_gps_eph[n].ety);
	// AOL IS THE ARGUMENT OF LATITUDE OF THE SATELLITE
    aol=ta+GLO1_gps_eph[n].w;

	// calculate the second harmonic perturbations of the orbit
	delr  =GLO1_gps_eph[n].crc*cos(2.0*aol)+GLO1_gps_eph[n].crs*sin(2.0*aol);
	delal =GLO1_gps_eph[n].cuc*cos(2.0*aol)+GLO1_gps_eph[n].cus*sin(2.0*aol);
	delinc=GLO1_gps_eph[n].cic*cos(2.0*aol)+GLO1_gps_eph[n].cis*sin(2.0*aol);


    r=pow(GLO1_gps_eph[n].sqra,2)*(1.00-GLO1_gps_eph[n].ety*cos(ea))+delr;
	aol=aol+delal;
	inc=GLO1_gps_eph[n].inc0+delinc+GLO1_gps_eph[n].idot*d_toe;

	// LA IS THE CORRECTED LONGITUDE OF THE ASCENDING NODE
    la=GLO1_gps_eph[n].w0+(GLO1_gps_eph[n].omegadot-GLO1_omegae)*d_toe-
		GLO1_omegae*GLO1_gps_eph[n].toe;
    xp=r*cos(aol);
    yp=r*sin(aol);
    result.x=xp*cos(la)-yp*cos(inc)*sin(la);
    result.y=xp*sin(la)+yp*cos(inc)*cos(la);
    result.z=yp*sin(inc);
    result.az=0.0;
	result.el=0.0;
    if (GLO1_rec_pos_xyz.x != 0.0 || GLO1_rec_pos_xyz.y != 0.0 || GLO1_rec_pos_xyz.z != 0.0)
	{

		north.x=-cos(GLO1_rec_pos_llh.lon)*sin(GLO1_rec_pos_llh.lat);
		north.y=-sin(GLO1_rec_pos_llh.lon)*sin(GLO1_rec_pos_llh.lat);
		north.z= cos(GLO1_rec_pos_llh.lat);
		east.x=-sin(GLO1_rec_pos_llh.lon);
		east.y= cos(GLO1_rec_pos_llh.lon);
		east.z=0.0;
		up.x=cos(GLO1_rec_pos_llh.lon)*cos(GLO1_rec_pos_llh.lat);
		up.y=sin(GLO1_rec_pos_llh.lon)*cos(GLO1_rec_pos_llh.lat);
		up.z=sin(GLO1_rec_pos_llh.lat);
		

		xls =result.x-GLO1_rec_pos_xyz.x;
		yls =result.y-GLO1_rec_pos_xyz.y;
		zls =result.z-GLO1_rec_pos_xyz.z;
		range1=sqrt(xls*xls+yls*yls+zls*zls);
		tdot=(up.x*xls+up.y*yls+up.z*zls)/range1;
   

		if ( tdot >= 1.00 ) satang=GLO1_pi/2.0;
		else if ( tdot <= -1.00 ) satang=-GLO1_pi/2.0;
		else satang=asin(tdot);


		xaz=east.x*xls+east.y*yls;
		yaz=north.x*xls+north.y*yls+north.z*zls;
		if (xaz !=0.0 || yaz !=0.0) az=atan2(xaz,yaz);
		else az=0.0;
		result.el=satang;
		result.az=az;
   }
   return(result);
}

/*******************************************************************************
FUNCTION read_initial_data(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
		  To read in all of the receiver initialization files
		  读入所有接收机初始化文件
*******************************************************************************/

void GLO1_read_initial_data(void)
{
	int		id;
	GLO1_SATVIS	dummy;//jh 方位角，仰角，多扑了 X Y Z 

	for (id=1;id<=32;id++) GLO1_gps_alm[id].inc=0.0;
	  
	GLO1_status = GLO1_cold_start;	// 先设置为冷启动模式
						// for initialization, if we have enough data we
	GLO1_read_ion_utc();	   // will switch to warm or hot start
	GLO1_read_almanac();	   // status在read_almanac可能被更改，如果almanac数据有效的话
	GLO1_thetime=time(NULL); //GLO1_thetime第一次出现，time函数：Gets the system time.
	//此函数会返回从公元1970年1月1日的UTC时间从0时0分0秒算起到现在所经过的秒数。如果t 并非空指针的话，此函数也会将返回值存到t指针所指的内存
	dummy = GLO1_satfind(0);//估计GPS周数和TOW
	//此语句的作用是什么？ 传入0，防止在第一次运行进入时用历书计算卫星位置，这里只是用satfind里的估计GPS周数和TOW     找到可视卫星位置，XYZ 俯仰角 多普勒
	 //dummy哑巴，假的，虚拟，这个量没有用//因为通道尚未分配，而用历书算的卫星位置是用来分配通道的，所以在通道分配的时候再按历书去求卫星位置，然后分配通道
}

/*******************************************************************************
FUNCTION receiver_loc(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
		To read in the last location of the receiver from file "curloc.dat"
		to help in warm and hot starts
		从curloc.dat中读最新的接收机位置，以帮助暖启动或热启动
*******************************************************************************/
char GLO1_curloc_file[4096];
GLO1_LLH GLO1_receiver_loc(void)
{
	float latitude,longitude,height;
    char  text[10];
	GLO1_LLH result;

	result.lat=0.0;      
	result.lon=0.0;
	result.hae=-6378137;

    // READ THE CURRENT LOCATION DATA FILE

	if ((GLO1_in = fopen(GLO1_curloc_file, "rt")) == NULL)
    {
		printf("Cannot open curloc.dat file.\n");
		GLO1_status = GLO1_cold_start;
    }
	else
    {
		// 从文件中读出接收机的经纬高，用于辅助热启动模式
		fscanf(GLO1_in,"%10s",text);//latitude
		fscanf(GLO1_in,"%f",&latitude);
		fscanf(GLO1_in,"%10s",text);//longitude
		fscanf(GLO1_in,"%f",&longitude);
		fscanf(GLO1_in,"%10s",text);//hae
		fscanf(GLO1_in,"%f",&height);
		result.lat=latitude/57.296;	//从度转到弧度 convert unit from degree to radian
		result.lon=longitude/57.296;
		result.hae=height;
		fclose(GLO1_in);
	}
	return(result);
}

/*******************************************************************************
FUNCTION navmess()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function assembles and decodes the 1500 bit nav message
	into almanac and ephemeris messages


*******************************************************************************/
//电文格式索引项
/*string 1*/
#define		glonass_m				0
#define		glonass_KX				1
#define		glonass_resv1			2

#define		glonass_e_P1			3
#define		glonass_e_tk_hour		4
#define		glonass_e_tk_min		5
#define		glonass_e_tk_sec		6
#define		glonass_e_dot_Xn		7
#define		glonass_e_doudot_Xn		8
#define		glonass_e_Xn			9

/*string 2*/
#define		glonass_e_Bn			3
#define		glonass_e_P2			4
#define		glonass_e_tb			5
#define		glonass_e_dot_Yn		6
#define		glonass_e_doudot_Yn		7
#define		glonass_e_Yn			8

/*string 3*/
#define		glonass_e_P3			3
#define		glonass_e_gamman		4
#define		glonass_e_P				5
#define		glonass_e_ln			6
#define		glonass_e_dot_Zn		7
#define		glonass_e_doudot_Zn		8
#define		glonass_e_Zn			9

/*string 4*/
#define		glonass_resv2			3
#define		glonass_e_taun			4
#define		glonass_e_deltataun		5
#define		glonass_e_En			6
#define		glonass_e_P4			7
#define		glonass_e_FT			8
#define		glonass_e_NT			9
#define		glonass_e_n				10
#define		glonass_e_Mod			11

/*string 5*/
#define		glonass_a_daynum		3
#define		glonass_a_tauc			4
#define		glonass_a_N4			5
#define		glonass_a_taugps		6
#define		glonass_a_str5_ln		7

/*string 6 8 10 12 14,frame 1 2 3 4 & string 6 8 10 12,frame 5*/
#define		glonass_a_Cn			2
#define		glonass_a_Mn			3
#define		glonass_a_n				4
#define		glonass_a_taun			5
#define		glonass_a_lambdan		6
#define		glonass_a_deltain		7
#define		glonass_a_epsilonn		8

/*string 7 9 11 13 15,frame 1 2 3 4 &string 7 9 11 13,frame 5*/
#define		glonass_a_omegan		2
#define		glonass_a_tlambdan		3
#define		glonass_a_deltaTn		4
#define		glonass_a_dot_deltaTn	5
#define		glonass_a_Hn			6
#define		glonass_a_ln			7

/*string 14,frame 5 */
#define		glonass_B1				3
#define		glonass_B2				4
#define		glonass_KP				5

/*string 15,frame 5 */
#define		glonass_str15_ln		3



//电文格式查找索引表
unsigned char  GLO1_decode_glonass_index[] = { 0, 10, 19, 29, 41 };
unsigned char  GLO1_decode_glonass_frame[][5] = {
	{ 49,49,49,49,49 },		//string 6
	{ 58,58,58,58,58 },		//string 7
	{ 49,49,49,49,49 },		//string 8
	{ 58,58,58,58,58 },		//string 9
	{ 49,49,49,49,49 },		//string 10
	{ 58,58,58,58,58 },		//string 11
	{ 49,49,49,49,49 },		//string 12
	{ 58,58,58,58,58 },		//string 13
	{ 49,49,49,49,66 },		//string 14
	{ 58,58,58,58,72 },		//string 15
};

//定义电文格式************电文位计数:  85--1
 GLO1_decode_glonass_info GLO1_glonass_formats[] = {
	/*string 1*/
	{ glonass_m,				0,	0,	81,	4 },
	{ glonass_KX,			0,	0,	1,	8 },
	{ glonass_resv1,			0,	0,	79,	2 },
	{ glonass_e_P1,			0,	0,	77,	2 },
	{ glonass_e_tk_hour,		0,	0,	72,	5 },
	{ glonass_e_tk_min,		0,	0,	66,	6 },
	{ glonass_e_tk_sec,		30,	0,	65,	1 },
	{ glonass_e_dot_Xn,		-20,1,	41,	24 },
	{ glonass_e_doudot_Xn,	-30,1,	36,	5 },
	{ glonass_e_Xn,			-11,1,	9,	27 },

	/*string 2*/
	{ glonass_m,				0,	0,	81,	4 },
	{ glonass_KX,			0,	0,	1,	8 },
	{ glonass_resv1,			0,	0,	65,	5 },
	{ glonass_e_Bn,			0,	0,	78,	3 },
	{ glonass_e_P2,			0,	0,	77,	1 },
	{ glonass_e_tb,			15,	0,	70,	7 },
	{ glonass_e_dot_Yn,		-20,1,	41,	24 },
	{ glonass_e_doudot_Yn,	-30,1,	36,	5 },
	{ glonass_e_Yn,			-11,1,	9,	27 },

	/*string 3*/
	{ glonass_m,				0,	0,	81,	4 },
	{ glonass_KX,			0,	0,	1,	8 },
	{ glonass_resv1,			0,	0,	68,	1 },
	{ glonass_e_P3,			0,	0,	80,	1 },
	{ glonass_e_gamman,		-40,1,	69,	11 },
	{ glonass_e_P,			0,	0,	66,	2 },
	{ glonass_e_ln,			0,	0,	65,	1 },
	{ glonass_e_dot_Zn,		-20,1,	41,	24 },
	{ glonass_e_doudot_Zn,	-30,1,	36,	5 },
	{ glonass_e_Zn,			-11,1,	9,	27 },

	/*string 4*/
	{ glonass_m,				0,	0,	81,	4 },
	{ glonass_KX,			0,	0,	1,	8 },
	{ glonass_resv1,			0,	0,	35,	14 },
	{ glonass_resv2,			0,	0,	27,	3 },
	{ glonass_e_taun,		-30,1,	59,	22 },
	{ glonass_e_deltataun,	-30,1,	54,	5 },
	{ glonass_e_En,			0,	0,	49,	5 },
	{ glonass_e_P4,			0,	0,	34,	1 },
	{ glonass_e_FT,			0,	0,	30,	4 },
	{ glonass_e_NT,			0,	0,	16,	11 },
	{ glonass_e_n,			0,	0,	11,	5 },
	{ glonass_e_Mod,			0,	0,	9,	2 },

	/*string 5*/
	{ glonass_m,				0,	0,	81,	4 },
	{ glonass_KX,			0,	0,	1,	8 },
	{ glonass_resv1,			0,	0,	37,	1 },
	{ glonass_a_daynum,		0,	0,	70,	11 },
	{ glonass_a_tauc,		-31,1,	38,	32 },
	{ glonass_a_N4,			0,	0,	32,	5 },
	{ glonass_a_taugps,		-30,1,	10,	22 },
	{ glonass_a_str5_ln,		0,	0,	9,	1 },

	/*string 6 8 10 12 14,frame 1 2 3 4 & string 6 8 10 12,frame 5*/
	{ glonass_m,				0,	0,	81,	4 },
	{ glonass_KX,			0,	0,	1,	8 },
	{ glonass_a_Cn,			0,	0,	80,	1 },
	{ glonass_a_Mn,			0,	0,	78,	2 },
	{ glonass_a_n,			0,	0,	73,	5 },
	{ glonass_a_taun,		-18,1,	63,	10 },
	{ glonass_a_lambdan,		-20,1,	42,	21 },
	{ glonass_a_deltain,		-20,1,	24,	18 },
	{ glonass_a_epsilonn,	-20,0,	9,	15 },

	/*string 7 9 11 13 15,frame 1 2 3 4 &string 7 9 11 13,frame 5*/
	{ glonass_m,				0,	0,	81,	4 },
	{ glonass_KX,			0,	0,	1,	8 },
	{ glonass_a_omegan,		-15,1,	65,	16 },
	{ glonass_a_tlambdan,	-5,	0,	44,	21 },
	{ glonass_a_deltaTn,		-9,	1,	22,	22 },
	{ glonass_a_dot_deltaTn,	-14,1,	15,	7 },
	{ glonass_a_Hn,			0,	0,	10,	5 },
	{ glonass_a_ln,			0,	0,	9,	1 },

	/*string 14,frame 5 */
	{ glonass_m,				0,	0,	81,	4 },
	{ glonass_KX,			0,	0,	1,	8 },
	{ glonass_resv1,			0,	0,	9,	49 },
	{ glonass_B1,			-10,1,	70,	11 },
	{ glonass_B2,			-16,1,	60,	10 },
	{ glonass_KP,			0,	0,	58,	2 },

	/*string 15,frame 5 */
	{ glonass_m,				0,	0,	81,	4 },
	{ glonass_KX,			0,	0,	1,	8 },
	{ glonass_resv1,			0,	0,	10,	79 },
	{ glonass_str15_ln,		0,	0,	9,	1 }
};
//实现把int32u(unsigned int)的数根据ICD通过位操作得到结果
unsigned int GLO1_get_u_bits(unsigned int src, unsigned char start, unsigned char end)
{
	unsigned char shift_left, shift_right;
	unsigned char length;
	unsigned int ret;
	length = end - start;
	shift_left = 32 - end;
	shift_right = 32 - length;
	//#ifdef NO_INSTRUCTION_OPERATE
	ret = (src << shift_left) >> shift_right;
	//#else
	//   ret= _extu(src,shift_left,shift_right);
	//#endif
	return ret;
}
//typedef  unsigned int 			int32u;
inline  unsigned int GLO1_glonass_decode(GLO1_decode_glonass_info *pfmt, unsigned int frame, unsigned int string, unsigned char flag, unsigned int pbin[], unsigned char strindex[], unsigned char *pfraindex)
{
	unsigned int ret = 0x00000000;
	unsigned char start_word, end_word;
	unsigned char start_bit;
	unsigned char end_bit;
	unsigned int temp_word;
	GLO1_decode_glonass_info *pinfo;

	if (string <= 5)
	{
		pinfo = &pfmt[strindex[string - 1] + flag];
	}
	else
	{
		pinfo = &pfmt[*(pfraindex + (string - 6) * 5 + (frame - 1)) + flag];
	}

	if (pinfo->num_bits != 0)
	{
		start_word = 2 - (pinfo->startbit + 4) / 30;
		end_word = 2 - (pinfo->startbit + pinfo->num_bits + 4 - 1) / 30;
		start_bit = (pinfo->startbit + 4) % 30;
		end_bit = (pinfo->startbit + pinfo->num_bits + 4 - 1) % 30;
		if (start_word == end_word)
		{
			if (string <= 5)
			{
				temp_word = pbin[start_word];

			}
			else
			{
				temp_word = pbin[start_word];
			}
			ret |= GLO1_get_u_bits(temp_word, start_bit, end_bit + 1);
			if (pinfo->pow2>1)
			{
				ret *= pinfo->pow2;
			}
		}
		else
		{
			if (string <= 5)
			{
				temp_word = pbin[end_word];
				ret |= GLO1_get_u_bits(temp_word, 0, end_bit + 1) << (pinfo->num_bits - end_bit - 1);
				temp_word = pbin[start_word];

				ret |= GLO1_get_u_bits(temp_word, start_bit, 30);

			}
			else
			{
				temp_word = pbin[end_word];
				ret |= GLO1_get_u_bits(temp_word, 0, end_bit + 1) << (pinfo->num_bits - end_bit - 1);
				temp_word = pbin[start_word];
				ret |= GLO1_get_u_bits(temp_word, start_bit, 30);
			}
			if (pinfo->pow2>1)
			{
				ret *= pinfo->pow2;
			}
		}
	}
	return ret;
}


inline int GLO1_glonass_decode_signed(GLO1_decode_glonass_info *pfmt, unsigned int frame, unsigned int string, unsigned char flag, unsigned int pbin[], unsigned char strindex[], unsigned char *pfraindex)
{
	unsigned char bits = 0;
	unsigned int data = 0;
	int ret;
	GLO1_decode_glonass_info *pinfo;

	//	int16u sign_flag;
	unsigned int mask = 0x00000001;

	if (string <= 5)
	{
		pinfo = &pfmt[strindex[string - 1] + flag];
	}
	else
	{
		pinfo = &pfmt[*(pfraindex + (string - 6) * 5 + (frame - 1)) + flag];
	}
	bits += pinfo->num_bits;
	data = GLO1_glonass_decode(pfmt, frame, string, flag, pbin, strindex, pfraindex);
	/*
	sign_flag = (data & (0x01<<(bits-1)))>>(bits-1);

	data = data & (~(0x01<<(bits-1)));

	if(sign_flag == 1)
	{
	ret = data * -1;
	}
	else
	{
	ret = data;
	}
	*/
	ret = GLO1_get_u_bits(data, 0, bits - 1);
	mask = 0x00000001 << (bits - 1);
	if ((data&mask) == mask)
	{
		ret = -ret;
	}

	return ret;
}


inline double GLO1_glonass_decode_float(GLO1_decode_glonass_info *pfmt, unsigned int frame, unsigned int string, unsigned char flag, unsigned int pbin[], unsigned char strindex[], unsigned char *pfraindex)
{
	int data;
	unsigned int data1;
	double ret, power2;
	GLO1_decode_glonass_info *pinfo;
	if (string <= 5)
	{
		pinfo = &pfmt[strindex[string - 1] + flag];
	}
	else
	{
		pinfo = &pfmt[*(pfraindex + (string - 6) * 5 + (frame - 1)) + flag];
	}
	if (pinfo->signq == 1)
	{
		data = GLO1_glonass_decode_signed(pfmt, frame, string, flag, pbin, strindex, pfraindex);
		ret = data;
		power2 = pow(2.0, pinfo->pow2);
		ret *= power2;
	}
	else
	{
		data1 = GLO1_glonass_decode(pfmt, frame, string, flag, pbin, strindex, pfraindex);
		ret = data1;
		power2 = pow(2.0, pinfo->pow2);
		ret *= power2;
	}
	return ret;

}

void GLO1_glonass_explain_string1_4(unsigned int frame, unsigned int string, GLO1_glonass_ephemeris *pephem, int32u pbin[])
{
	//    int16u ln_debug;
	if (string == 1)
	{
		//glonass_decode_signed(pfmt, frame, string, flag, pbin, strindex, pfraindex);
		pephem->P1 = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_P1, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->tk_hour = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_tk_hour, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->tk_min = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_tk_min, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->tk_sec = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_tk_sec, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->dot_Xn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_dot_Xn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 1000.0;
		pephem->doudot_Xn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_doudot_Xn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 1000.0;
		pephem->Xn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_Xn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 1000.0;
		///*保存卫星位置*/
		//char str[10];
		////itoa(sv, str, 10);
		//sprintf_s(str, "%d", string);
		//char s[100] = "sv_";
		//strcat_s(s, str);
		//char *pfilename = s;
		//FILE *fj;
		//fopen_s(&fj, pfilename, "a+");

		//fprintf(fj, "%d %d %d %d %f %f %f\n", pephem->P1, pephem->tk_hour, pephem->tk_min, pephem->tk_sec, pephem->dot_Xn, pephem->doudot_Xn, pephem->Xn);
		//fclose(fj);// 输出完毕后关闭这个文件
	}

	if (string == 2)
	{
		pephem->Bn = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_Bn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->P2 = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_P2, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->tb = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_tb, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 60;
		pephem->dot_Yn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_dot_Yn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 1000.0;
		pephem->doudot_Yn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_doudot_Yn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 1000.0;
		pephem->Yn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_Yn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 1000.0;

		///*保存卫星位置*/
		//char str[10];
		////itoa(sv, str, 10);
		//sprintf_s(str, "%d", string);
		//char s[100] = "sv_";
		//strcat_s(s, str);
		//char *pfilename = s;
		//FILE *fj;
		//fopen_s(&fj, pfilename, "a+");

		//fprintf(fj, "%d %d %d %f %f %f\n", pephem->Bn, pephem->P2, pephem->tb, pephem->dot_Yn, pephem->doudot_Yn, pephem->Yn);
		//fclose(fj);// 输出完毕后关闭这个文件
	}

	if (string == 3)
	{
		pephem->P3 = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_P3, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->gamman = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_gamman, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->P = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_P, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->ln = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_ln, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->dot_Zn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_dot_Zn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 1000.0;
		pephem->doudot_Zn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_doudot_Zn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 1000.0;
		pephem->Zn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_Zn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame) * 1000.0;

		///*保存卫星位置*/
		//char str[10];
		////itoa(sv, str, 10);
		//sprintf_s(str, "%d", string);
		//char s[100] = "sv_";
		//strcat_s(s, str);
		//char *pfilename = s;
		//FILE *fj;
		//fopen_s(&fj, pfilename, "a+");

		//fprintf(fj, "%d %f %d %d %f %f %f\n", pephem->P3, pephem->gamman, pephem->P, pephem->ln, pephem->dot_Zn, pephem->doudot_Zn, pephem->Zn);
		//fclose(fj);// 输出完毕后关闭这个文件
	}

	if (string == 4)
	{
		pephem->taun = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_taun, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->deltataun = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_e_deltataun, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->En = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_En, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->P4 = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_P4, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->FT = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_FT, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->NT = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_NT, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->n = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_n, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		pephem->Mod = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_e_Mod, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);

		///*保存卫星位置*/
		//char str[10];
		////itoa(sv, str, 10);
		//sprintf_s(str, "%d", string);
		//char s[100] = "sv_";
		//strcat_s(s, str);
		//char *pfilename = s;
		//FILE *fj;
		//fopen_s(&fj, pfilename, "a+");

		//fprintf(fj, "%f %f %d %d %d %d %d %d\n", pephem->taun, pephem->deltataun, pephem->En, pephem->P4, pephem->FT, pephem->NT, pephem->n, pephem->Mod);
		//fclose(fj);// 输出完毕后关闭这个文件
	}

	return;
}

void GLO1_glonass_explain_string5_14_15(int32u frame,
	int32u string,
	GLO1_glonass_almanac_str5 *palc_str5,
	GLO1_glonass_almanac_global *palc,
	int32u pbin[])
{

	if (string == 5)
	{
		palc_str5->daynum = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_a_daynum, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc_str5->tauc = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_tauc, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc_str5->N4 = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_a_N4, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc_str5->taugps = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_taugps, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc_str5->str5_ln = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_a_str5_ln, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);

		///*保存卫星位置*/
		//char str[10];
		////itoa(sv, str, 10);
		//sprintf_s(str, "%d", string);
		//char s[100] = "sv_";
		//strcat_s(s, str);
		//char *pfilename = s;
		//FILE *fj;
		//fopen_s(&fj, pfilename, "a+");

		//fprintf(fj, "%d %f %d %f %d\n", palc_str5->daynum, palc_str5->tauc, palc_str5->N4, palc_str5->taugps, palc_str5->str5_ln);
		//fclose(fj);// 输出完毕后关闭这个文件
	}
	if (string == 14 && frame == 5)
	{
		palc->B1 = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_B1, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->B2 = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_B2, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->KP = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_KP, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
	}

	if (string == 15 && frame == 5)
	{
		palc->str15_ln = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_str15_ln, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
	}


	return;
}

void GLO1_glonass_explain_string6_7(int32u frame, int32u string, GLO1_glonass_almanac*palc, int32u pbin[])
{
	if ((string % 2) == 0)
	{
		palc->Cn = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_a_Cn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->Mn = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_a_Mn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->n = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_a_n, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->taun = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_taun, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->lambdan = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_lambdan, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->deltain = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_deltain, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->epsilonn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_epsilonn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
	}
	else
	{
		palc->omegan = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_omegan, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->tlambdan = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_tlambdan, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->deltaTn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_deltaTn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->dot_deltaTn = GLO1_glonass_decode_float(GLO1_glonass_formats, frame, string, glonass_a_dot_deltaTn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->Hn = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_a_Hn, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
		palc->ln = GLO1_glonass_decode(GLO1_glonass_formats, frame, string, glonass_a_ln, pbin, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
	}



	return;
}



void GLO1_glonass_ephemeris_store(GLO1_glonass_ephemeris *pephem, GLO1_glonass_ephemeris *pephem_temp)
{
	


	pephem->star = pephem_temp->star;
	pephem->P1 = pephem_temp->P1;
	pephem->tk_hour = pephem_temp->tk_hour;
	pephem->tk_min = pephem_temp->tk_min;
	pephem->tk_sec = pephem_temp->tk_sec;
	pephem->dot_Xn = pephem_temp->dot_Xn;
	pephem->doudot_Xn = pephem_temp->doudot_Xn;
	pephem->Xn = pephem_temp->Xn;

	pephem->Bn = pephem_temp->Bn;       //not use
	pephem->P2 = pephem_temp->P2;       //not use
	pephem->tb = pephem_temp->tb;
	pephem->dot_Yn = pephem_temp->dot_Yn;
	pephem->doudot_Yn = pephem_temp->doudot_Yn;
	pephem->Yn = pephem_temp->Yn;

	pephem->P3 = pephem_temp->P3;
	pephem->gamman = pephem_temp->gamman;
	pephem->P = pephem_temp->P;
	pephem->ln = pephem_temp->ln;        //not use
	pephem->dot_Zn = pephem_temp->dot_Zn;
	pephem->doudot_Zn = pephem_temp->doudot_Zn;
	pephem->Zn = pephem_temp->Zn;

	pephem->taun = pephem_temp->taun;
	pephem->deltataun = pephem_temp->deltataun;
	pephem->En = pephem_temp->En;
	pephem->P4 = pephem_temp->P4;        //not use
	pephem->FT = pephem_temp->FT;
	pephem->URA = pephem_temp->URA;
	pephem->NT = pephem_temp->NT;
	pephem->n = pephem_temp->n;
	pephem->Mod = pephem_temp->Mod;

	pephem->valid = 1;

	return;
}

void GLO1_glonass_almanac_store(GLO1_glonass_almanac *palc, GLO1_glonass_almanac *palc_temp)
{
	palc->Cn = palc_temp->Cn;
	palc->Mn = palc_temp->Mn;
	palc->n = palc_temp->n;
	palc->taun = palc_temp->taun;
	palc->lambdan = palc_temp->lambdan;
	palc->deltain = palc_temp->deltain;
	palc->epsilonn = palc_temp->epsilonn;

	palc->omegan = palc_temp->omegan;
	palc->tlambdan = palc_temp->tlambdan;
	palc->deltaTn = palc_temp->deltaTn;
	palc->dot_deltaTn = palc_temp->dot_deltaTn;
	palc->Hn = palc_temp->Hn;
	palc->ln = palc_temp->ln;

	palc->valid = 1;

	return;
}

void GLO1_glonass_almanac_str5_store(GLO1_glonass_almanac_str5 *palc, GLO1_glonass_almanac_str5 *palc_temp)
{
	palc->daynum = palc_temp->daynum;
	palc->tauc = palc_temp->tauc;
	palc->N4 = palc_temp->N4;
	palc->taugps = palc_temp->taugps;
	palc->str5_ln = palc_temp->str5_ln;

	palc->valid = 1;

	return;
}

void GLO1_glonass_almanac_global_store(GLO1_glonass_almanac_global *palc, GLO1_glonass_almanac_global *palc_temp)
{

	palc->B1 = palc_temp->B1;
	palc->B2 = palc_temp->B2;
	palc->KP = palc_temp->KP;

	palc->str15_ln = palc_temp->str15_ln;

	palc->valid = 1;

	return;
}



void GLO1_unpack_glonass_all_data(int CH, GLO1_glonass_ephemeris *pephem,
	GLO1_glonass_almanac *palc,
	GLO1_glonass_almanac_str5 *palc_str5,
	GLO1_glonass_almanac_global *palc_glob,
	int32u origin_data[],
	GLO1_CHANNEL *pchan,
	int16s	 *unpack_glonass_flag,

	int16s *glonass_ephemeris_processing,
	int32u *glonass_frame_id,
	int16s *glonass_almanac_processing,
	int16s *glonass_almanac_global_processing,
	bool *string1)
{
	int32u	string_id;
	int32u glonass_soday = 0;

	//*********************判定星历有效性，此处位置需详细讨论*************************************
	/*fp64	eph_effect_time;
	fp64	cmp_time;


	//	fp64	for_set_time;

	if(*unpack_glonass_flag == 1 && pchan->Sv_id >0)
	{
	if(GLO1_glonass_sv_id_ephemeris[glonass_chan[channel].Sv_id-1].P1 == 0)
	{
	eph_effect_time = 30.0*60.0;
	}
	else if(GLO1_glonass_sv_id_ephemeris[glonass_chan[channel].Sv_id-1].P1 == 1)
	{
	eph_effect_time = 30.0*60.0;
	}
	else if(GLO1_glonass_sv_id_ephemeris[glonass_chan[channel].Sv_id-1].P1 == 2)
	{
	eph_effect_time = 45.0*60.0;
	}
	else if(GLO1_glonass_sv_id_ephemeris[glonass_chan[channel].Sv_id-1].P1 == 3)
	{
	eph_effect_time = 60.0*60.0;
	}

	convert_time(&cmp_time,Local_tow,GLONASS_SYS);
	cmp_time = fabs(cmp_time - GLO1_glonass_sv_id_ephemeris[glonass_chan[channel].Sv_id-1].tb);
	if(Local_tow_vail_flag == 1 && cmp_time > eph_effect_time)
	{
	unpack_glonass_flag[glonass_chan[channel].Sv_id-1] = 0;
	glonass_ephemeris_processing[channel] = -1;
	}

	}*/

	//******************************解析电文****************************************
	string_id = GLO1_glonass_decode(GLO1_glonass_formats, 1, 1, glonass_m, origin_data, GLO1_decode_glonass_index, *GLO1_decode_glonass_frame);
	//string_id_debug = glonass_decode_debug(GLO1_glonass_formats,1,1,glonass_m,origin_data);


	switch (string_id)
	{
	case 1:
	{   GLO1_glonass_explain_string1_4(1, 1, pephem, origin_data);
	*string1 = true;
	//GLO1_writeEpoch(CH, (0x1f & readEpochCheck(CH)) | 0x000);
	//if (GLO1_chan[CH].n_frame == 0)
	//{
	//	glonass_soday = (pephem->tk_hour * 3600 + pephem->tk_min * 60 + pephem->tk_sec) + 8;//+8
	//	GLO1_chan[CH].tr_bit_time = glonass_soday * 100;
	//}
	//else
	//{
	//	glonass_soday = (pephem->tk_hour * 3600 + pephem->tk_min * 60 + pephem->tk_sec) + 8 - 2;
	//	GLO1_chan[CH].tr_bit_time = glonass_soday * 100;
	//}
	if (*glonass_ephemeris_processing == -1)
	{
		*glonass_frame_id++;
		if (*glonass_frame_id >5)
			*glonass_frame_id = 1;
		//glonass_explain_string1_4(1,1,pephem,origin_data);
		*glonass_ephemeris_processing = 1;
	}

	}
	break;

	case 2:
	{   GLO1_glonass_explain_string1_4(1, 2, pephem, origin_data);
	if (*glonass_ephemeris_processing == 1)
	{
		//glonass_explain_string1_4(1,2,pephem,origin_data);
		*glonass_ephemeris_processing = 2;
	}
	*glonass_ephemeris_processing = 2;
	//else
	//*glonass_ephemeris_processing = -1;
	}
	break;

	case 3:
	{   GLO1_glonass_explain_string1_4(1, 3, pephem, origin_data);
	if (pephem->P3 == 0)
	{
		*glonass_frame_id = 5;
	}
	else if (*glonass_frame_id == 5 && pephem->P3 == 1)
		*glonass_frame_id--;

	if (*glonass_ephemeris_processing == 2)
	{
		//glonass_explain_string1_4(1,3,pephem,origin_data);
		*glonass_ephemeris_processing = 3;

		//if(pephem->P3 == 0)
		//{
		//	*glonass_frame_id = 5;
		//}
		//else if(*glonass_frame_id == 5 && pephem->P3 ==1)
		//	*glonass_frame_id--;
	}
	//else
	//*glonass_ephemeris_processing = -1;
	}
	break;

	case 4:
	{
		GLO1_glonass_explain_string1_4(1, 4, pephem, origin_data);
		pephem->valid = 1;
		pchan->prn = pephem->n;

		//if(palc_str5->valid==1)
		//{
		//	glonass_soday = pephem->tk_hour*3600 + pephem->tk_min*60 + pephem->tk_sec + 8;

		//	set_local_time(glonass_soday,78,pephem->NT,GLONASS_SYS);	//置入本地时
		//set_local_time_glo(glonass_soday,78,pephem->NT,palc_str5->N4);	//置入本地时
		//}
		//if(palc_str5->valid==1)

		//{
		//	glonass_soday = pephem->tk_hour*3600 + pephem->tk_min*60 + pephem->tk_sec + 8;

		//set_local_time(glonass_soday,78,pephem->NT,GLONASS_SYS);	//置入本地时
		//	set_local_time_glo(glonass_soday,78,pephem->NT,palc_str5->N4);	//置入本地时
		//}


		//Global_WN = pephem->NT;
		switch (pephem->FT)
		{
		case 0:  pephem->URA = 1.0; break;
		case 1:  pephem->URA = 2.0; break;
		case 2:  pephem->URA = 2.5; break;
		case 3:  pephem->URA = 4.0; break;
		case 4:  pephem->URA = 5.0; break;
		case 5:  pephem->URA = 7.0; break;
		case 6:  pephem->URA = 10.0; break;
		case 7:  pephem->URA = 12.0; break;
		case 8:  pephem->URA = 14.0; break;
		case 9:  pephem->URA = 16.0; break;
		case 10: pephem->URA = 32.0; break;
		case 11: pephem->URA = 64.0; break;
		case 12: pephem->URA = 128.0; break;
		case 13: pephem->URA = 256.0; break;
		case 14: pephem->URA = 512.0; break;
		case 15: {pephem->URA = 0.0; pephem->Bn = 1; }break;
		default:break;

		}

		if (*glonass_ephemeris_processing == 3)
		{

				glonass_soday = (pephem->tk_hour * 3600 + pephem->tk_min * 60 + pephem->tk_sec)+8;//+8
				GLO1_chan[CH].tr_bit_time = glonass_soday * 100;
		}
		//set_local_time(glonass_soday, 87, pephem->NT, GLONASS_SYS);	//置入本地时
		//set_local_time_glo(glonass_soday,78,pephem->NT,palc_str5->N4);	//置入本地时
		//Global_WN = pephem->NT;
		//if (palc_str5->valid == 1)
		//{
		//	glonass_soday = pephem->tk_hour * 3600 + pephem->tk_min * 60 + pephem->tk_sec ;

		//	//set_local_time(glonass_soday,78,pephem->NT,GLONASS_SYS);	//置入本地时
		//	//if(pephem->NT!=0)
		//	//set_local_time_glo(glonass_soday, 78, pephem->NT, palc_str5->N4);	//置入本地时
		//}


		if (pephem->n>0 && pephem->n <= 24)
		*unpack_glonass_flag = 1;
		GLO1_glonass_ephemeris_store(&GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10], pephem);
		////保存伪距
		//char str[10];
		////itoa(sv, str, 10);
		//sprintf(str, "%d", GLO1_chan[CH].K + 10);
		//char s[100] = "dw_";
		//char *pfilename = strcat(s, str);

		//FILE *fd = fopen(pfilename, "a");
		//fprintf(fd, " %d %d %d %d %d %10.9f %10.9f %10.9f  %d %10.9f %10.9f %10.9f\n", GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].star, GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].P1,
		//	GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].tk_hour, GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].tk_min,
		//	GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].tk_sec, GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].dot_Xn,
		//	GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].doudot_Xn, GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].Xn,
		//	GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].tb, GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].dot_Yn,
		//	GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].doudot_Yn, GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K + 10].Yn );
		//fclose(fd);

		//glonass_ephemeris_store(&GLO1_glonass_sv_id_ephemeris[pephem->n - 1], pephem);

		//if(sys_selected_num==1)
		//if (g1_need && sv_need == pephem->n)
		//	/******/		    	Get_XLXX_2F(GLONASS_SYS, &GLO1_glonass_sv_id_ephemeris[pephem->n - 1]);

		//Global_WN = pephem->NT;
		*glonass_ephemeris_processing = -1;



	}
	break;

	case 5:
	{
		GLO1_glonass_explain_string5_14_15(1, 5, palc_str5, palc_glob, origin_data);
		//glonass_almanac_str5_store(&GLO1_glonass_sv_id_almanac_str5[pephem->n-1],palc_str5);
		//if(pephem->valid ==1)
		//{
		//	glonass_soday = pephem->tk_hour*3600 + pephem->tk_min*60 + pephem->tk_sec + 8;

		//	set_local_time_glo(glonass_soday,78,pephem->NT,palc_str5->N4);	//置入本地时
		// }

		palc_str5->valid = 1;
		/*
		if(GLO1_glonass_sv_id_almanac_str5[pephem->n].valid ==1)
		{
		for_set_time = GLO1_glonass_sv_id_almanac_str5[pephem->n].tauc + 0.2;
		if(glonass_soday[channel] > 0)
		{
		glonass_soday[channel] += (int32s)for_set_time + 10;
		if(glonass_soday[channel] >86400)
		{
		glonass_soday[channel] -=86400;
		}
		for_set_time = for_set_time - (int32s)for_set_time;

		if(for_set_time < 0.0)
		{
		for_set_time += 1;
		glonass_soday[channel] -= 1;
		}
		for_set_time *= 1000.0;

		set_local_time(glonass_soday[channel],(int32s)for_set_time,pephem->NT,GLONASS_SYS);	//置入本地时
		}
		*/


	}
	break;

	case 6:
	{
		if (*glonass_almanac_processing == -1)
		{
			GLO1_glonass_explain_string6_7(1, 6, palc, origin_data);
			*glonass_almanac_processing = 1;
		}
	}
	break;

	case 7:
	{
		if (*glonass_almanac_processing == 1)
		{
			GLO1_glonass_explain_string6_7(1, 7, palc, origin_data);
			*glonass_almanac_processing = 2;

			//glonass_almanac_store(&glonass_sv_id_almanac[palc->n-1],palc);
			palc->valid = 1;
		}
		else
			*glonass_almanac_processing = -1;
	}
	break;

	case 8:
	{
		GLO1_glonass_explain_string6_7(1, 8, palc, origin_data);
		*glonass_almanac_processing = 3;
	}
	break;

	case 9:
	{
		if (*glonass_almanac_processing == 3)
		{
			GLO1_glonass_explain_string6_7(1, 9, palc, origin_data);
			*glonass_almanac_processing = 4;

			//glonass_almanac_store(&glonass_sv_id_almanac[palc->n-1],palc);
			palc->valid = 1;
		}
		else
			*glonass_almanac_processing = -1;
	}
	break;

	case 10:
	{
		GLO1_glonass_explain_string6_7(1, 10, palc, origin_data);
		*glonass_almanac_processing = 5;
	}
	break;

	case 11:
	{
		if (*glonass_almanac_processing == 5)
		{
			GLO1_glonass_explain_string6_7(1, 11, palc, origin_data);
			*glonass_almanac_processing = 6;

			//glonass_almanac_store(&glonass_sv_id_almanac[palc->n-1],palc);
			palc->valid = 1;
		}
		else
			*glonass_almanac_processing = -1;
	}
	break;

	case 12:
	{
		GLO1_glonass_explain_string6_7(1, 12, palc, origin_data);
		*glonass_almanac_processing = 7;
	}
	break;

	case 13:
	{
		if (*glonass_almanac_processing == 7)
		{
			GLO1_glonass_explain_string6_7(1, 13, palc, origin_data);
			*glonass_almanac_processing = 8;

			//glonass_almanac_store(&glonass_sv_id_almanac[palc->n-1],palc);
			palc->valid = 1;
		}
		else
			*glonass_almanac_processing = -1;
	}
	break;

	case 14:
	{
		if (*glonass_frame_id == 5)
		{
			if (*glonass_almanac_global_processing == -1)
			{
				GLO1_glonass_explain_string5_14_15(5, 14, palc_str5, palc_glob, origin_data);
				*glonass_almanac_global_processing = 1;
			}
			else
				*glonass_almanac_global_processing = -1;
		}
		else
		{
			GLO1_glonass_explain_string6_7(1, 14, palc, origin_data);
			*glonass_almanac_processing = 9;
		}

	}
	break;

	case 15:
	{
		if (*glonass_frame_id == 5)
		{
			if (*glonass_almanac_global_processing == 1)
			{
				GLO1_glonass_explain_string5_14_15(5, 14, palc_str5, palc_glob, origin_data);
				//glonass_almanac_global_store(&glonass_sv_id_almanac_global[pephem->n-1],palc_glob);
				palc_glob->valid = 1;
			}
			*glonass_almanac_global_processing = -1;

		}
		else
		{
			if (*glonass_almanac_processing == 9)
			{
				GLO1_glonass_explain_string6_7(1, 15, palc, origin_data);
				//glonass_almanac_store(&glonass_sv_id_almanac[palc->n-1],palc);
				palc->valid = 1;
			}
			*glonass_almanac_processing = -1;
		}
	}
	break;

	}
	if (pephem->n>0 && pephem->n <= 24)

	//glonass_ephemeris_store(&GLO1_glonass_sv_id_ephemeris[GLO1_chan[CH].K+10], pephem);
	GLO1_glonass_almanac_str5_store(&GLO1_glonass_sv_id_almanac_str5[GLO1_chan[CH].K +10], palc_str5);

	return;
}



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
	bool *string1)
{
	//int16s		glonass_verify_flag;

	//relative_decode(origin_string_data);

	/////////////////glonass_verify_flag	0:正确;		1:已修正；	2:舍弃;		3:其他//////////////////////////
	//glonass_verify_flag = glonass_verify_data(t);

	//	glonass_chan_tdxx[channel].FraID = (char)((origin_string_data[0] & 0x1E000000)>>25);

	//if (glonass_verify_flag == 0 || glonass_verify_flag == 1)		//电文无误码
	//{
	//提取string号
	//		glonass_chan_tdxx[channel].FraID = (char)((origin_string_data[0] & 0x1E000000)>>25);

	//解析电文


	GLO1_unpack_glonass_all_data(CH, pephem,
		palc,
		palc_str5,
		palc_glob,
		origin_string_data,
		pchan,
		unpack_glonass_flag,

		glonass_ephemeris_processing,
		glonass_frame_id,
		glonass_almanac_processing,
		glonass_almanac_global_processing,
		string1);
	//glonass_ephemeris_store(&GLO1_glonass_sv_id_ephemeris[pephem->n], pephem, &GLO1_glonass_sv_id_almanac_str5[pephem->n], palc_str5);
	//}
	//else		//电文有误码，舍弃本string
	//{

	//}


	origin_string_data[0] = 0;
	origin_string_data[1] = 0;
	origin_string_data[2] = 0;


}
int 	GLO1_i4page, GLO1_i5page;
void GLO1_navmess(char prn, char ch)
{
	int				i,j,k;
	unsigned long	isqra,ie,iomega0;
	long			iaf0,iomegadot;
	char			itgd,iaf2;
	short			iweek,iura,ihealth,iodc,iaf1;
	unsigned short	itoe,itoc;
	short			iode,icrs,idn,icuc,icus,icic,iomegad;
	short			icis,icrc,idoe,idot;
	unsigned short	iae,iatoa;
	//st-atic	int 	i4page,i5page;
	short			i4data,i5data,isv,iaomegad;
	long			iaaf0,iaaf1,iadeli,iaomega0,im0,inc0,iw;
	unsigned long	iasqr;
	long			iaw,iam0,scale,ia0,ia1;
	char			ial0,ial1,ial2,ial3,ibt0,ibt1,ibt2,ibt3;
	short			itot,iWNt,idtls,iWNlsf,iDN,idtlsf;//WNa
	short			sfr,word,i4p,i5p;
	double			r_sqra,r_inc0,r_ety;
	double			d_toe;

	double			doublesv;


    d_toe=GLO1_clock_tow-GLO1_gps_eph[prn].toe;

    if (d_toe>302400.0) d_toe=d_toe-604800.0;
    else if (d_toe<-302400.0)d_toe=d_toe+604800.0;
	
	j=0;
	for (sfr=1;sfr<=5;sfr++)
	{
		for (word=1;word<=10;word++)
		{
			scale=0;	
			for (i=0;i<=29;i++)
			{
				scale=scale<<1;	
				if (GLO1_chan[ch].message[(j+GLO1_chan[ch].offset)%1500]==1)
				{
					scale++; 
				}
				j++;
			}
			GLO1_sf[sfr][word]=scale;
		}
	}

	GLO1_parity_check();
	if ((GLO1_p_error[1]==0 || GLO1_p_error[1]==0x200) && 
		(GLO1_p_error[2]==0 || GLO1_p_error[2]==0x200) && 
		(GLO1_p_error[3]==0 || GLO1_p_error[3]==0x200))
	{
		iodc=int(((GLO1_sf[1][3] & 0x3) <<8 ) | ((GLO1_sf[1][8] & 0xFF0000L) >>16));
		iode=int(GLO1_sf[2][3]  >> 16);
		idoe=int(GLO1_sf[3][10] >> 16);
		if (iode==idoe)// && ((iode!=GLO1_gps_eph[prn].iode) || (iodc!=GLO1_gps_eph[prn].iodc)))
						//modified by daicy
		{
			iweek= int(GLO1_sf[1][3] >> 14);
			iura=int(( GLO1_sf[1][3] & 0xF00 ) >> 8);
			ihealth=int(( GLO1_sf[1][3] & 0xFC ) >> 2);
			itgd=int(GLO1_sf[1][7] & 0xFF);
			itoc=int(GLO1_sf[1][8] & 0xFFFF);
			iaf2=int(GLO1_sf[1][9] >> 16);
			iaf1=int(GLO1_sf[1][9] & 0xFFFF);
			iaf0=GLO1_sf[1][10] >> 2;
			if (GLO1_bit_test_l(iaf0,22)) iaf0=iaf0 | 0xFFC00000L;
			
			//   subframe 2
			icrs=int(GLO1_sf[2][3] & 0xFFFF);
			idn=int(GLO1_sf[2][4] >> 8);
			im0=((GLO1_sf[2][4] & 0xFF) << 24) | GLO1_sf[2][5];
			icuc=int(GLO1_sf[2][6] >>8);
			ie=((GLO1_sf[2][6] & 0xFF) << 24) | GLO1_sf[2][7];
			icus=int(GLO1_sf[2][8] >> 8);
			isqra=(((GLO1_sf[2][8] & 0xFF) << 24) | GLO1_sf[2][9]);
			itoe=int(GLO1_sf[2][10] >> 8);
			
			// subframe 3
			icic=int(GLO1_sf[3][3] >> 8);
			icis=int(GLO1_sf[3][5] >> 8);
			inc0=((GLO1_sf[3][5] & 0xFF) << 24) | GLO1_sf[3][6];
			iomega0=((GLO1_sf[3][3] & 0xFF) << 24) | GLO1_sf[3][4];
			icrc=int(GLO1_sf[3][7] >> 8);
			iw=((GLO1_sf[3][7] & 0xFF) << 24) | GLO1_sf[3][8];
			iomegadot=GLO1_sf[3][9];
			if (GLO1_bit_test_l(iomegadot,24)) iomegadot=iomegadot | 0xFF000000L;
			idot=int((GLO1_sf[3][10] & 0xFFFC) >> 2);
			if (GLO1_bit_test_l(idot,14)) idot=idot | 0xC000;

			r_sqra=isqra*GLO1_c_2m19;
			r_inc0=inc0*GLO1_c_2m31*GLO1_pi;
			r_ety=ie*GLO1_c_2m33;

			if ((r_inc0<1.05 && r_inc0>0.873) && (r_sqra>5100.0 && r_sqra<5200.0) &&
				(r_ety <.05 && r_ety>0.0))
			{
				GLO1_gps_eph[prn].valid=1;
				GLO1_gps_eph[prn].iode=iode;
				GLO1_gps_eph[prn].iodc=iodc;
				GLO1_gps_eph[prn].week=iweek;
				GLO1_gps_eph[prn].ura=iura;
				GLO1_gps_eph[prn].health=ihealth;
				GLO1_gps_eph[prn].tgd=itgd*GLO1_c_2m31;
				GLO1_gps_eph[prn].toc=itoc*16.0;
				GLO1_gps_eph[prn].af2=iaf2*GLO1_c_2m55;
				GLO1_gps_eph[prn].af1=iaf1*GLO1_c_2m43;
				GLO1_gps_eph[prn].af0=iaf0*GLO1_c_2m31;
				GLO1_gps_eph[prn].crs=icrs*GLO1_c_2m5;
				GLO1_gps_eph[prn].dn=idn*GLO1_c_2m43*GLO1_pi;
				GLO1_gps_eph[prn].ma=im0*GLO1_c_2m31*GLO1_pi;
				GLO1_gps_eph[prn].cuc=icuc*GLO1_c_2m29;
				GLO1_gps_eph[prn].ety=r_ety;
				GLO1_gps_eph[prn].cus=icus*GLO1_c_2m29;
				GLO1_gps_eph[prn].sqra=r_sqra;
				GLO1_gps_eph[prn].wm=19964981.84/pow(r_sqra,3);
				GLO1_gps_eph[prn].toe=itoe*GLO1_c_2p4;
				GLO1_gps_eph[prn].cic=icic*GLO1_c_2m29;
				GLO1_gps_eph[prn].cis=icis*GLO1_c_2m29;
				GLO1_gps_eph[prn].inc0=r_inc0;
				GLO1_gps_eph[prn].w0=iomega0*GLO1_c_2m31*GLO1_pi;
				GLO1_gps_eph[prn].crc=icrc*GLO1_c_2m5;
				GLO1_gps_eph[prn].w=iw*GLO1_c_2m31*GLO1_pi;
				GLO1_gps_eph[prn].omegadot=iomegadot*GLO1_c_2m43*GLO1_pi;
				GLO1_gps_eph[prn].idot=idot*GLO1_c_2m43*GLO1_pi;

				  doublesv=(double)prn;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=(double)iweek;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=0.0;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].tgd;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=0;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].toc;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].af2;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].af1;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].af0;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=0;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].crs;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].dn;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].ma;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].cuc;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].ety;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].cus;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].sqra;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].toe;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].cic;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].w0;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].cis;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].inc0;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].crc;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].w;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].omegadot;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  doublesv=GLO1_gps_eph[prn].idot;
				  fwrite(&doublesv, sizeof(double),1,GLO1_fpeph);

				  fflush(GLO1_fpeph);

				  if (GLO1_out_debug)  GLO1_write_Debug_ephemeris(prn);
			} 
		    else if (GLO1_gps_eph[prn].valid==1 && d_toe>7200.0) GLO1_gps_eph[prn].valid=0;
		} 
	}

	if ((GLO1_p_error[4]==0 || GLO1_p_error[4] == 0x200) && 
		(GLO1_p_error[5]==0 || GLO1_p_error[5] == 0x200) && GLO1_almanac_valid==0 && GLO1_almanac_flag==0)
	{
		GLO1_almanac_flag=1;

		i4data= int(GLO1_sf[4][3] >> 22);	
		i4p= int((GLO1_sf[4][3] & 0x3F0000L) >> 16);	
		if (i4p != GLO1_i4page && i4data == 1)
		{								
			GLO1_i4page = i4p;
			if (GLO1_i4page > 24 && GLO1_i4page < 33)
			{
				isv = GLO1_i4page;
				GLO1_gps_alm[isv].week=GLO1_gps_week%1024;
				iae=int(GLO1_sf[4][3] & 0x00FFFFL);
				GLO1_gps_alm[isv].ety=iae*GLO1_c_2m21;
				iatoa=int(GLO1_sf[4][4] >> 16);
				GLO1_gps_alm[isv].toa=iatoa*GLO1_c_2p12;
				iadeli=GLO1_sf[4][4] & 0x00FFFFL;
				if (GLO1_bit_test_l(iadeli,16)) iadeli=iadeli | 0xFFFF0000L;
				GLO1_gps_alm[isv].inc=(iadeli*GLO1_c_2m19+0.3)*GLO1_pi;
				iomegad=int(GLO1_sf[4][5] >> 8);
				GLO1_gps_alm[isv].rra=iomegad*GLO1_c_2m38*GLO1_pi;
				GLO1_gps_alm[isv].health=int(GLO1_sf[4][5] & 0x0000FF);
				iasqr=GLO1_sf[4][6];
				GLO1_gps_alm[isv].sqa=iasqr*GLO1_c_2m11;
				if (GLO1_gps_alm[isv].sqa>0.0) 
					GLO1_gps_alm[isv].w=19964981.84/pow(GLO1_gps_alm[isv].sqa,3);
				iaomega0=GLO1_sf[4][7];
				if (GLO1_bit_test_l(iaomega0,24)) iaomega0=iaomega0 | 0xFF000000L;
				GLO1_gps_alm[isv].lan=iaomega0*GLO1_c_2m23*GLO1_pi;
				iaw=GLO1_sf[4][8];
				if (GLO1_bit_test_l(iaw,24)) iaw=iaw | 0xFF000000L;
				GLO1_gps_alm[isv].aop=iaw*GLO1_c_2m23*GLO1_pi;
				iam0=GLO1_sf[4][9];
				if (GLO1_bit_test_l(iam0,24)) iam0=iam0 | 0xFF000000L;
				GLO1_gps_alm[isv].ma=iam0*GLO1_c_2m23*GLO1_pi;
				iaaf0=(GLO1_sf[4][10] >> 13) | ((GLO1_sf[4][10] & 0x1C)>>2);
				if (GLO1_bit_test_l(iaaf0,11)) iaaf0=iaaf0 | 0xFFFFF800L;
				GLO1_gps_alm[isv].af0=iaaf0*GLO1_c_2m20;
				iaaf1=(GLO1_sf[4][10] | 0xFFE0) >> 5;
				if (GLO1_bit_test_l(iaaf1,11)) iaaf1=iaaf1 | 0xFFFFF800L;
				GLO1_gps_alm[isv].af1=iaaf1*GLO1_c_2m38;
			}

			else if (GLO1_i4page == 55)
			{
				GLO1_gps_alm[prn].text_message[0]=char((GLO1_sf[4][3] & 0x00FF00) >> 8);
				GLO1_gps_alm[prn].text_message[1]=char( GLO1_sf[4][3] & 0x0000FF);
				for ( k=1;k<=7;k++)
				{
				  GLO1_gps_alm[prn].text_message[3*k-1]= char(GLO1_sf[4][k+3] >> 16);
				  GLO1_gps_alm[prn].text_message[3*k  ]= char((GLO1_sf[4][k+3] & 0x00FF00) >> 8);
				  GLO1_gps_alm[prn].text_message[3*k+1]= char(GLO1_sf[4][k+3] & 0x0000FF);
				}
			}

			else if (GLO1_i4page == 56)
			{
				ial0=int((GLO1_sf[4][3] & 0x00FF00) >> 8);
				GLO1_al0=ial0*GLO1_c_2m30;
				ial1= int(GLO1_sf[4][3] & 0x0000FF);
				GLO1_al1=ial1*GLO1_c_2m27;
				ial2= int(GLO1_sf[4][4] >> 16);
				GLO1_al2=ial2*GLO1_c_2m24;
				ial3=int((GLO1_sf[4][4] & 0x00FF00) >> 8);
				GLO1_al3=ial3*GLO1_c_2m24;
				ibt0= int(GLO1_sf[4][4] & 0x0000FF);
				GLO1_b0=ibt0*2048.;
				ibt1= int(GLO1_sf[4][5] >> 16);
				GLO1_b1=ibt1*16384.;
				ibt2=int((GLO1_sf[4][5] & 0x00FF00) >> 8);
				GLO1_b2=ibt2*65536.;
				ibt3= int(GLO1_sf[4][5] & 0x00FF);
				GLO1_b3=ibt3*65536.;
				ia1=   GLO1_sf[4][6];
				if (GLO1_bit_test_l(ia1,24)) ia1=ia1 | 0xFF000000L;
				GLO1_a1=ia1*GLO1_c_2m50;
				ia0=  (GLO1_sf[4][7] << 8) | (GLO1_sf[4][8] >> 16);
				GLO1_a0=ia0*GLO1_c_2m30;
				itot=  int((GLO1_sf[4][8] & 0x00FF00) >> 8);
				GLO1_tot=itot*4096;
				iWNt=  int(GLO1_sf[4][8] & 0xFF);
				GLO1_WNt=iWNt;
				idtls=  int(GLO1_sf[4][10] >> 16);
				if (idtls >128) idtls=idtls |0xFF00;
				GLO1_dtls=idtls;
				iWNlsf=int((GLO1_sf[4][9] & 0x00FF00) >> 8);
				GLO1_WNlsf=iWNlsf;
				iDN   = int(GLO1_sf[4][9] & 0x0000FF);
				GLO1_DN=iDN;
				idtlsf= int(GLO1_sf[4][9] >> 16);
				if (idtlsf >128) idtlsf=idtlsf |0xFF00;
				GLO1_dtlsf=idtlsf;
			}

			else if (GLO1_i4page == 63)
			{
				GLO1_ASV[1]= int((GLO1_sf[4][3] & 0x00F000) >>12);
				GLO1_ASV[2]= int((GLO1_sf[4][3] & 0x000F00) >>8);
				GLO1_ASV[3]= int((GLO1_sf[4][3] & 0x0000F0) >>4);
				GLO1_ASV[4]= int( GLO1_sf[4][3] & 0x00000F);
				GLO1_ASV[5]= int( GLO1_sf[4][4] >>20);
				GLO1_ASV[6]= int((GLO1_sf[4][4] & 0x0F0000L) >>16);
				GLO1_ASV[7]= int((GLO1_sf[4][4] & 0x00F000) >>12);
				GLO1_ASV[8]= int((GLO1_sf[4][4] & 0x000F00) >> 8);
				GLO1_ASV[9]= int((GLO1_sf[4][4] & 0x0000F0) >> 4);
				GLO1_ASV[10]=int(GLO1_sf[4][4] & 0x00000F);
				GLO1_ASV[11]=int(GLO1_sf[4][5] >>20);
				GLO1_ASV[12]=int((GLO1_sf[4][5] & 0x0F0000L) >>16);
				GLO1_ASV[13]=int((GLO1_sf[4][5] & 0x00F000) >>12);
				GLO1_ASV[14]=int((GLO1_sf[4][5] & 0x000F00) >> 8);
				GLO1_ASV[15]=int((GLO1_sf[4][5] & 0x0000F0) >> 4);
				GLO1_ASV[16]=int(GLO1_sf[4][5] & 0x00000F);
				GLO1_ASV[17]=int(GLO1_sf[4][6] >>20);
				GLO1_ASV[18]=int((GLO1_sf[4][6] & 0x0F0000L) >>16);
				GLO1_ASV[19]=int((GLO1_sf[4][6] & 0x00F000) >>12);
				GLO1_ASV[20]=int((GLO1_sf[4][6] & 0x000F00) >> 8);
				GLO1_ASV[21]=int((GLO1_sf[4][6] & 0x0000F0) >> 4);
				GLO1_ASV[22]=int(GLO1_sf[4][6] & 0x00000F);
				GLO1_ASV[23]=int(GLO1_sf[4][7] >>20);
				GLO1_ASV[24]=int((GLO1_sf[4][7] & 0x0F0000L) >>16);
				GLO1_ASV[25]=int((GLO1_sf[4][7] & 0x00F000) >>12);
				GLO1_ASV[26]=int((GLO1_sf[4][7] & 0x000F00) >> 8);
				GLO1_ASV[27]=int((GLO1_sf[4][7] & 0x0000F0) >> 4);
				GLO1_ASV[28]=int( GLO1_sf[4][7] & 0x00000F);
				GLO1_ASV[29]=int( GLO1_sf[4][8] >>20);
				GLO1_ASV[30]=int((GLO1_sf[4][8] & 0x0F0000L) >>16);
				GLO1_ASV[31]=int((GLO1_sf[4][8] & 0x00F000) >>12);
				GLO1_ASV[32]=int((GLO1_sf[4][8] & 0x000F00) >> 8);
				GLO1_SVh[25]=int(GLO1_sf[4][8] & 0x00003F);
				if( GLO1_SVh[25]==0x3f) GLO1_gps_alm[25].inc=0.0;
				GLO1_SVh[26]=int(GLO1_sf[4][9] >>18);
				if( GLO1_SVh[26]==0x3f) GLO1_gps_alm[26].inc=0.0;
				GLO1_SVh[27]=int((GLO1_sf[4][9] & 0x03F000L) >>12);
				if( GLO1_SVh[27]==0x3f) GLO1_gps_alm[27].inc=0.0;
				GLO1_SVh[28]=int((GLO1_sf[4][9] & 0x000FC0) >>6);
				if( GLO1_SVh[28]==0x3f) GLO1_gps_alm[28].inc=0.0;
				GLO1_SVh[29]= int(GLO1_sf[4][9] & 0x00003F);
				if( GLO1_SVh[29]==0x3f) GLO1_gps_alm[29].inc=0.0;
				GLO1_SVh[30]= int(GLO1_sf[4][10] >>18);
				if( GLO1_SVh[30]==0x3f) GLO1_gps_alm[30].inc=0.0;
				GLO1_SVh[31]=int((GLO1_sf[4][10]& 0x03F000L) >>12);
				if( GLO1_SVh[31]==0x3f) GLO1_gps_alm[31].inc=0.0;
				GLO1_SVh[32]=int((GLO1_sf[4][10]& 0x000FC0) >>6);
				if( GLO1_SVh[32]==0x3f) GLO1_gps_alm[32].inc=0.0;
			}
		}
		
		i5data=int(GLO1_sf[5][3] >> 22);	// DATA ID
		i5p=int((GLO1_sf[5][3] & 0x3F0000L) >> 16);	// PAGE/SV ID
		GLO1_chan[ch].page5=i5p;
		if (GLO1_i5page != i5p && i5data == 1)
		{
			GLO1_i5page = i5p;


			if (GLO1_i5page == 51)
			{
				iatoa=int((GLO1_sf[5][3] & 0xFF00) >>8);
				GLO1_SVh[1]=int(GLO1_sf[5][4] >>18);
				if( GLO1_SVh[1]==0x3f) GLO1_gps_alm[1].inc=0.0;
				GLO1_SVh[2]=int((GLO1_sf[5][4] & 0x03F000L)>>12);
				if( GLO1_SVh[2]==0x3f) GLO1_gps_alm[2].inc=0.0;
				GLO1_SVh[3]=int((GLO1_sf[5][4] & 0x000FC0)>>6);
				if( GLO1_SVh[3]==0x3f) GLO1_gps_alm[3].inc=0.0;
				GLO1_SVh[4]= int(GLO1_sf[5][4] & 0x00003F);
				if( GLO1_SVh[4]==0x3f) GLO1_gps_alm[4].inc=0.0;
				GLO1_SVh[5]= int(GLO1_sf[5][5] >>18);
				if( GLO1_SVh[5]==0x3f) GLO1_gps_alm[5].inc=0.0;
				GLO1_SVh[6]=int((GLO1_sf[5][5] & 0x03F000L)>>12);
				if( GLO1_SVh[6]==0x3f) GLO1_gps_alm[6].inc=0.0;
				GLO1_SVh[7]=int((GLO1_sf[5][5] & 0x000FC0)>>6);
				if( GLO1_SVh[7]==0x3f) GLO1_gps_alm[7].inc=0.0;
				GLO1_SVh[8]= int(GLO1_sf[5][5] & 0x00003F);
				if( GLO1_SVh[8]==0x3f) GLO1_gps_alm[8].inc=0.0;
				GLO1_SVh[9]= int(GLO1_sf[5][6] >>18);
				if( GLO1_SVh[9]==0x3f) GLO1_gps_alm[9].inc=0.0;
				GLO1_SVh[10]=int((GLO1_sf[5][6] & 0x03F000L)>>12);
				if( GLO1_SVh[10]==0x3f) GLO1_gps_alm[10].inc=0.0;
				GLO1_SVh[11]=int((GLO1_sf[5][6] & 0x000FC0)>>6);
				if( GLO1_SVh[11]==0x3f) GLO1_gps_alm[11].inc=0.0;
				GLO1_SVh[12]= int(GLO1_sf[5][6] & 0x00003F);
				if( GLO1_SVh[12]==0x3f) GLO1_gps_alm[12].inc=0.0;
				GLO1_SVh[13]= int(GLO1_sf[5][7] >>18);
				if( GLO1_SVh[13]==0x3f) GLO1_gps_alm[13].inc=0.0;
				GLO1_SVh[14]=int((GLO1_sf[5][7] & 0x03F000L)>>12);
				if( GLO1_SVh[14]==0x3f) GLO1_gps_alm[14].inc=0.0;
				GLO1_SVh[15]=int((GLO1_sf[5][7] & 0x000FC0)>>6);
				if( GLO1_SVh[15]==0x3f) GLO1_gps_alm[15].inc=0.0;
				GLO1_SVh[16]= int(GLO1_sf[5][7] & 0x00003F);
				if( GLO1_SVh[16]==0x3f) GLO1_gps_alm[16].inc=0.0;
				GLO1_SVh[17]= int(GLO1_sf[5][8] >>18);
				if( GLO1_SVh[17]==0x3f) GLO1_gps_alm[17].inc=0.0;
				GLO1_SVh[18]=int((GLO1_sf[5][8] & 0x03F000L)>>12);
				if( GLO1_SVh[18]==0x3f) GLO1_gps_alm[18].inc=0.0;
				GLO1_SVh[19]=int((GLO1_sf[5][8] & 0x000FC0)>>6);
				if( GLO1_SVh[19]==0x3f) GLO1_gps_alm[19].inc=0.0;
				GLO1_SVh[20]= int(GLO1_sf[5][8] & 0x00003F);
				if( GLO1_SVh[20]==0x3f) GLO1_gps_alm[20].inc=0.0;
				GLO1_SVh[21]= int(GLO1_sf[5][9] >>18);
				if( GLO1_SVh[21]==0x3f) GLO1_gps_alm[21].inc=0.0;
				GLO1_SVh[22]=int((GLO1_sf[5][9] & 0x03F000L)>>12);
				if( GLO1_SVh[22]==0x3f) GLO1_gps_alm[22].inc=0.0;
				GLO1_SVh[23]=int((GLO1_sf[5][9] & 0x000FC0)>>6);
				if( GLO1_SVh[23]==0x3f) GLO1_gps_alm[23].inc=0.0;
				GLO1_SVh[24]= int(GLO1_sf[5][9] & 0x00003F);
				if( GLO1_SVh[24]==0x3f) GLO1_gps_alm[24].inc=0.0;
			}

			else
			{
				isv = GLO1_i5page;
				GLO1_gps_alm[isv].week=GLO1_gps_week%1024;
				iae=int(GLO1_sf[5][3] & 0xFFFF);
				GLO1_gps_alm[isv].ety=iae*GLO1_c_2m21;
				iatoa=int(GLO1_sf[5][4] >> 16);
				GLO1_gps_alm[isv].toa=iatoa*4096.0;
				iadeli=int(GLO1_sf[5][4] & 0xFFFF);
				GLO1_gps_alm[isv].inc=(iadeli*GLO1_c_2m19+0.3)*GLO1_pi;
				iaomegad=int(GLO1_sf[5][5] >> 8);
				GLO1_gps_alm[isv].rra=iaomegad*GLO1_c_2m38*GLO1_pi;
				GLO1_gps_alm[isv].health=int(GLO1_sf[5][5] & 0xFF);
				iasqr=GLO1_sf[5][6];
				GLO1_gps_alm[isv].sqa=iasqr*GLO1_c_2m11;
				if (GLO1_gps_alm[isv].sqa>0.0) GLO1_gps_alm[isv].w=19964981.84/pow(GLO1_gps_alm[isv].sqa,3);
				iaomega0=GLO1_sf[5][7];
				if (GLO1_bit_test_l(iaomega0,24)) iaomega0=iaomega0 | 0xFF000000L;
				GLO1_gps_alm[isv].lan=iaomega0*GLO1_c_2m23*GLO1_pi;
				iaw=GLO1_sf[5][8];
				if (GLO1_bit_test_l(iaw,24)) iaw=iaw | 0xFF000000L;
				GLO1_gps_alm[isv].aop=iaw*GLO1_c_2m23*GLO1_pi;
				iam0=GLO1_sf[5][9];
				if (GLO1_bit_test_l(iam0,24)) iam0=iam0 | 0xFF000000L;
				GLO1_gps_alm[isv].ma=iam0*GLO1_c_2m23*GLO1_pi;
				iaaf0=int((GLO1_sf[5][10] >> 13) | ((GLO1_sf[5][10] & 0x1C)>>2));
				if (GLO1_bit_test_l(iaaf0,11)) iaaf0=iaaf0 | 0xF800;
				GLO1_gps_alm[isv].af0=iaaf0*GLO1_c_2m20;
				iaaf1=int((GLO1_sf[5][10] & 0xFFE0) >> 5);
				if (GLO1_bit_test_l(iaaf1,11)) iaaf1=iaaf1 | 0xF800;
				GLO1_gps_alm[isv].af1=iaaf1*GLO1_c_2m38;
			}  
		}
	} 
}

/*******************************************************************************
FUNCTION bit_test_l(unsigned long data, char bit_n)
RETURNS  int

PARAMETERS
	  data    unsigned long
	  bit_n   char

PURPOSE
	This function returns a 1 if bit number bit_n of data is 1
	else it returns a 0

*******************************************************************************/

inline int  GLO1_bit_test_l(unsigned long data, char bit_n)
{
	int result;
	result= (data & GLO1_test_l[bit_n]) ? 1 : 0;
	return(result);
}

/*******************************************************************************
FUNCTION parity_check(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  checks the parity of the 5 subframes of the nav message

*******************************************************************************/

void  GLO1_parity_check(void)
{
	long pb1=0x3b1f3480L,pb2=0x1d8f9a40L,pb3=0x2ec7cd00L;
	long pb4=0x1763e680L,pb5=0x2bb1f340L,pb6=0x0b7a89c0L;
	int  parity,m_parity;
	char d29=0,d30=0,sfm,word,b_1,b_2,b_3,b_4,b_5,b_6;
	int err_bit;

	for (sfm=1;sfm<=5;sfm++)
	{
		GLO1_p_error[sfm]=0;
		for (word=1;word<=10;word++)
		{
			m_parity=int(GLO1_sf[sfm][word] & 0x3f);
			b_1=GLO1_exor(d29,GLO1_sf[sfm][word] & pb1) << 5;
			b_2=GLO1_exor(d30,GLO1_sf[sfm][word] & pb2) << 4;
			b_3=GLO1_exor(d29,GLO1_sf[sfm][word] & pb3) << 3;
			b_4=GLO1_exor(d30,GLO1_sf[sfm][word] & pb4) << 2;
			b_5=GLO1_exor(0,GLO1_sf[sfm][word] & pb5) << 1;
			b_6=GLO1_exor(d29^d30,GLO1_sf[sfm][word] & pb6);
			parity=b_1+b_2+b_3+b_4+b_5+b_6;

			err_bit = (parity != m_parity) ? 1: 0;
			GLO1_p_error[sfm]=(GLO1_p_error[sfm] << 1) + err_bit;
			if (d30==1) GLO1_sf[sfm][word]=0x03fffffc0L & ~GLO1_sf[sfm][word];
			GLO1_sf[sfm][word]=GLO1_sf[sfm][word]>>6;

			d29=(m_parity & 0x2) >>1;
			d30=m_parity & 0x1;
		}
	}
}

/*******************************************************************************
FUNCTION exor(char bit, long parity)
RETURNS  None.

PARAMETERS
			bit     char
			parity  long

PURPOSE
			count the number of bits set in the parameter parity and
			do an exclusive or with the parameter bit

*******************************************************************************/
int GLO1_exor(char bit, long parity)
{
	char i;
	int result;
	result=0;
	for (i=7;i<=30;i++)
	{
		if (GLO1_bit_test_l(parity,i)) result++;
	}
	result=result%2;
	result=(bit ^ result) & 0x1;
	return(result);
}

/*******************************************************************************
FUNCTION ecef_to_llh(ecef pos)
RETURNS  position in llh structure

PARAMETERS
			pos   ecef

PURPOSE    Convert a position in in cartesian ecef coordinates to
			  Geodetic WGS 84 coordinates

Based on equations found in Hoffman-Wellinhoff

*******************************************************************************/

GLO1_LLH GLO1_ecef_to_llh(GLO1_XYZ pos)
{
	double p,n,thet,esq,epsq;
	GLO1_LLH result;

	p=sqrt(pos.x*pos.x+pos.y*pos.y);
	thet=atan(pos.z*GLO1_a/(p*GLO1_b));
	esq =1.0-GLO1_b*GLO1_b/(GLO1_a*GLO1_a);
	epsq=GLO1_a*GLO1_a/(GLO1_b*GLO1_b)-1.0;
	result.lat=atan((pos.z+epsq*GLO1_b*pow(sin(thet),3))/(p-esq*GLO1_a*pow(cos(thet),3)));
	result.lon=atan2(pos.y,pos.x);
	n=GLO1_a*GLO1_a/sqrt(GLO1_a*GLO1_a*cos(result.lat)*cos(result.lat) +
		GLO1_b*GLO1_b*sin(result.lat)*sin(result.lat));
	result.hae=p/cos(result.lat)-n;
	return(result);
}

/*******************************************************************************
FUNCTION llh_to_ecef(llh pos)
RETURNS  position in ecef structure

PARAMETERS
			pos   llh

PURPOSE    Convert a position in Geodetic WGS 84 coordinates to cartesian
			  ecef coordinates

Based on equations found in Hoffman-Wellinhoff

*******************************************************************************/

GLO1_XYZ llh_to_ecef(GLO1_LLH pos)
{
	double n;
	GLO1_XYZ result;
	n=GLO1_a*GLO1_a/sqrt(GLO1_a*GLO1_a*cos(pos.lat)*cos(pos.lat)+GLO1_b*GLO1_b*sin(pos.lat)*sin(pos.lat));
	result.x=(n+pos.hae)*cos(pos.lat)*cos(pos.lon);
	result.y=(n+pos.hae)*cos(pos.lat)*sin(pos.lon);
	result.z=(GLO1_b*GLO1_b/(GLO1_a*GLO1_a)*n+pos.hae)*sin(pos.lat);
	return(result);
}

/*******************************************************************************
FUNCTION pos_vel_time(int nsl)
RETURNS  None.

PARAMETERS
			nsl   int

PURPOSE

	This routine processes the all-in-view pseudorange to arrive
	at a receiver position

INPUTS:
    pseudo_range[nsl] Vector of measured range from satellites to the receiver
	sat_location[nsl][3] Array of satellite locations in ECEF when the signal
								 was sent
    nsl      number of satellites used

OUTPUTS:
    RP[3]    VECTOR OF RECEIVER POSITION IN ECEF (X,Y,Z)
    CBIAS    RECEIVER CLOCK BIAS FROM GPS TIME

VARIABLES USED:
    C        SPEED OF LIGHT IN VACUUM IN M/S
    S[6][5]  MATRIX USED FOR SOLVING FOR RECEIVER POSITION CORRECTION
    B[5]     RESULTING RECEIVER CLOCK BIAS & POSITION CORRECTIONS
    X,Y,Z    TEMPORARY RECEIVER POSITION
    T        TEMPORARY RECEIVER CLOCK BIAS
    R[5]     RANGE FROM RECEIVER TO SATELLITES

IF THE POSITION CANNOT BE DETERMINED THE RESULT OF RP
WILL BE (0,0,0) THE CENTER OF THE EARTH
// needs debugging ???
*******************************************************************************/

int		GLO1_daicy_temp_cnt;
extern double		GLO1_m_time[3];;
extern FILE *GLO1_daicy_file_pr;
extern FILE *GLO1_daicy_file_pos;
extern FILE   *GLO1_out_trtime;

GLO1_PVT  GLO1_pos_vel_time(int nsl)
{
	double	dd[5][5],r,ms[5][13],pm[5][13],bm[13],br[5],correct_mag,x,y,z,t;
	double	GLO1_a1,GLO1_b1,c1,d1,e1,f1,g1,h1,i1,j1,k1,l1,m1,n1,o1,p1,denom,alpha;
	int		i,j,k,nits;
	GLO1_PVT		result;

	double	daicy_delta_pr[13];
	double	daicy_x,daicy_y,daicy_z;

	nits=0;
	t=0.0;

	x=GLO1_rec_pos_xyz.x;
	y=GLO1_rec_pos_xyz.y;
	z=GLO1_rec_pos_xyz.z;

	GLO1_daicy_temp_cnt = 0;

	do
	{
		GLO1_daicy_temp_cnt++;
		for (i=1;i<=nsl;i++)
		{
			alpha=(t-GLO1_dt[i])*GLO1_omegae;
			r=sqrt(pow(GLO1_track_sat[i].x*cos(alpha)-GLO1_track_sat[i].y*sin(alpha)-x,2)+
				 pow(GLO1_track_sat[i].y*cos(alpha)+GLO1_track_sat[i].x*sin(alpha)-y,2)+
				 pow(GLO1_track_sat[i].z-z,2));
			bm[i]=r-(GLO1_dt[i]-t)*GLO1_c;
			ms[1][i]=(GLO1_track_sat[i].x*cos(alpha)-GLO1_track_sat[i].y*sin(alpha)-x)/r;
			ms[2][i]=(GLO1_track_sat[i].y*cos(alpha)+GLO1_track_sat[i].x*sin(alpha)-y)/r;
			ms[3][i]=(GLO1_track_sat[i].z-z)/r;
			ms[4][i]=1.0;

			//输出伪距比对
			if (GLO1_daicy_temp_cnt>0)
			{
				#if 1
				//(39,116,18000)
				daicy_x=GLO1_track_sat[i].x*cos(alpha)-GLO1_track_sat[i].y*sin(alpha)-(-2181911.708);
				daicy_y=GLO1_track_sat[i].y*cos(alpha)+GLO1_track_sat[i].x*sin(alpha)-4473581.957;
				daicy_z=GLO1_track_sat[i].z-4003644.790;
				#else
				//(39,116,100)
				daicy_x=GLO1_track_sat[i].x*cos(alpha)-GLO1_track_sat[i].y*sin(alpha)-(-2175813.565);
				daicy_y=GLO1_track_sat[i].y*cos(alpha)+GLO1_track_sat[i].x*sin(alpha)-4461078.911;
				daicy_z=GLO1_track_sat[i].z-3992379.955;
				#endif
				
				daicy_delta_pr[i]=sqrt(daicy_x*daicy_x+daicy_y*daicy_y+daicy_z*daicy_z);
				fprintf(GLO1_daicy_file_pr, "m_time[1]: %.9f	%d	dt[i]: %.9f	R: %.2f	", GLO1_m_time[1], i, GLO1_dt[i], daicy_delta_pr[i]);
				
				daicy_delta_pr[i]=sqrt(daicy_x*daicy_x+daicy_y*daicy_y+daicy_z*daicy_z)-GLO1_dt[i]*GLO1_c;
				fprintf(GLO1_daicy_file_pr, "trans_t: %.2f	delta_t: %.2f	", GLO1_dt[i] * GLO1_c, daicy_delta_pr[i]);
				
				if(i==nsl)
					fprintf(GLO1_daicy_file_pr, "		diff		%.2f	%.2f	%.2f	%.2f	%.2f	%.2f	%.2f	\n",
										 daicy_delta_pr[2]-daicy_delta_pr[1],daicy_delta_pr[3]-daicy_delta_pr[1],
										 daicy_delta_pr[4]-daicy_delta_pr[1],daicy_delta_pr[5]-daicy_delta_pr[1],
										 daicy_delta_pr[6]-daicy_delta_pr[1],daicy_delta_pr[7]-daicy_delta_pr[1],
										 daicy_delta_pr[8]-daicy_delta_pr[1]);
			}
		}
		GLO1_a1=0.;GLO1_b1=0.;c1=0.;d1=0.;
		e1=0.;f1=0.;g1=0.;h1=0.;
		i1=0.;j1=0.;k1=0.;l1=0.;
		m1=0.;n1=0.;o1=0.;p1=0.;
		for (k=1;k<=nsl;k++)
		{
			GLO1_a1+=ms[1][k]*ms[1][k];
			GLO1_b1+=ms[1][k]*ms[2][k];
			c1+=ms[1][k]*ms[3][k];
			d1+=ms[1][k]*ms[4][k];
			f1+=ms[2][k]*ms[2][k];
			g1+=ms[2][k]*ms[3][k];
			h1+=ms[2][k]*ms[4][k];
			k1+=ms[3][k]*ms[3][k];
			l1+=ms[3][k]*ms[4][k];
			p1+=ms[4][k];
		}
		o1=l1;m1=d1;n1=h1;e1=GLO1_b1;i1=c1;j1=g1;

		denom=(k1*p1-l1*o1)*(GLO1_a1*f1-GLO1_b1*e1) + (l1*n1-j1*p1)*(GLO1_a1*g1-c1*e1) +
				(j1*o1-k1*n1)*(GLO1_a1*h1-d1*e1) + (l1*m1-i1*p1)*(c1*f1-GLO1_b1*g1) +
				(i1*o1-k1*m1)*(d1*f1-GLO1_b1*h1) + (i1*n1-j1*m1)*(c1*h1-d1*g1);
		
		dd[1][1]=f1*(k1*p1-l1*o1)+g1*(l1*n1-j1*p1)+h1*(j1*o1-k1*n1);
		dd[1][2]=e1*(l1*o1-k1*p1)+g1*(i1*p1-l1*m1)+h1*(k1*m1-i1*o1);
		dd[1][3]=e1*(j1*p1-n1*l1)-i1*(f1*p1-n1*h1)+m1*(f1*l1-j1*h1);
		dd[1][4]=e1*(n1*k1-j1*o1)+i1*(f1*o1-n1*g1)+m1*(j1*g1-f1*k1);
		dd[2][1]=dd[1][2];
		dd[2][2]=GLO1_a1*(k1*p1-l1*o1)+c1*(l1*m1-i1*p1)+d1*(i1*o1-k1*m1);
		dd[2][3]=GLO1_a1*(l1*n1-j1*p1)+i1*(GLO1_b1*p1-n1*d1)+m1*(j1*d1-GLO1_b1*l1);
		dd[2][4]=GLO1_a1*(j1*o1-n1*k1)-i1*(GLO1_b1*o1-n1*c1)+m1*(GLO1_b1*k1-c1*j1);
//		dd[3][1]=GLO1_b1*(g1*p1-h1*o1)-f1*(c1*p1-o1*d1)+n1*(c1*h1-d1*g1);
		dd[3][1]=dd[1][3];
//		dd[3][2]=GLO1_a1*(o1*h1-g1*p1)+e1*(c1*p1-o1*d1)+m1*(d1*g1-c1*h1);
		dd[3][2]=dd[2][3];
		dd[3][3]=GLO1_a1*(f1*p1-h1*n1)+GLO1_b1*(h1*m1-e1*p1)+d1*(e1*n1-f1*m1);
		dd[3][4]=GLO1_a1*(n1*g1-f1*o1)+e1*(GLO1_b1*o1-c1*n1)+m1*(c1*f1-GLO1_b1*g1);
//		dd[4][1]=GLO1_b1*(h1*k1-g1*l1)+f1*(c1*l1-d1*k1)+j1*(d1*g1-c1*h1);
		dd[4][1]=dd[1][4];
//		dd[4][2]=GLO1_a1*(g1*l1-h1*k1)-e1*(c1*l1-d1*k1)+i1*(c1*h1-d1*g1);
		dd[4][2]=dd[2][4];
//		dd[4][3]=GLO1_a1*(j1*h1-f1*l1)+e1*(GLO1_b1*l1-d1*j1)+i1*(d1*f1-GLO1_b1*h1);
		dd[4][3]=dd[3][4];
		dd[4][4]=GLO1_a1*(f1*k1-g1*j1)+GLO1_b1*(g1*i1-e1*k1)+c1*(e1*j1-f1*i1);
		if ( denom<=0.0 )
		{
			result.x=1.0;      // something went wrong
			result.y=1.0;      // set solution to near center of earth
			result.z=1.0;
			result.dt=0.0;
		}
		else
		{
			for (i=1;i<=4;i++)
			{
				for (j=1;j<=4;j++) dd[i][j]=dd[i][j]/denom;
			}
			for (i=1;i<=nsl;i++)
			{
				for (j=1;j<=4;j++)
				{
					pm[j][i]=0.0;
					for (k=1;k<=4;k++)pm[j][i]+=dd[j][k]*ms[k][i];
				}
			}
			for (i=1;i<=4;i++)
			{
				br[i]=0.0;
				for (j=1;j<=nsl;j++)br[i]+=bm[j]*pm[i][j];
			}
			nits++;
			x += br[1];
			y += br[2];
			z += br[3];
			t -= br[4]/GLO1_c;	
			correct_mag=sqrt(br[1]*br[1]+br[2]*br[2]+br[3]*br[3]);
		}
	} while ( correct_mag > 0.01 && correct_mag < 1.e8 && nits < 10);


	result.x=x;
	result.y=y;
	result.z=z;
	result.dt=t;
	
	//  Now for Velocity
	for (i=1;i<=nsl;i++)
	{
		alpha=(GLO1_dt[i]-t)*GLO1_omegae;


		r=sqrt(pow(GLO1_track_sat[i].x*cos(alpha)-GLO1_track_sat[i].y*sin(alpha)-x,2)+
			pow(GLO1_track_sat[i].y*cos(alpha)+GLO1_track_sat[i].x*sin(alpha)-y,2)+
			pow(GLO1_track_sat[i].z-z,2));

		bm[i]=((GLO1_track_sat[i].x*cos(alpha)-GLO1_track_sat[i].y*sin(alpha)-x)*GLO1_d_sat[i].x+
			(GLO1_track_sat[i].y*cos(alpha)+GLO1_track_sat[i].x*sin(alpha)-y)*GLO1_d_sat[i].y+
			(GLO1_track_sat[i].z-z)*GLO1_d_sat[i].z)/r+GLO1_meas_dop[i]*GLO1_lambda[i];
	}


	for (i=1;i<=4;i++)
	{
		br[i]=0.0;
		for (j=1;j<=nsl;j++)br[i]+=bm[j]*pm[i][j];
	}

	result.xv=br[1]+y*GLO1_omegae;  
	result.yv=br[2]-x*GLO1_omegae;  
	result.zv=br[3];
	result.df=br[4]/GLO1_c*1000000.0; 
	return(result);
}

GLO1_PVT GLO1_kalman_pos_vel(int nsl)
{
	double r, x, y, z, t;
	int nits;
	double Ts = 0.01;
	double Sf = 1.0e-4;
	double St = 1.0e-5;
	double Q_detaT11 = St*Ts + Sf*Ts*Ts*Ts / 3;
	double Q_detaT12 = Sf*Ts*Ts / 2;
	double Q_detaT21 = Sf*Ts*Ts / 2;
	double Q_detaT22 = Sf*Ts;
	int NumState = 8;  //卡尔曼状态个数
	double Qx = 0.005;
	double Qy = 0.001;
	double Qz = 0.001;
	/*ax,ay.az:加速度方差乘以三个方向的加速度时间常数的倒数得到的结果*/
	double ax = 4.0e-2;
	double ay = 4.0e-2;
	double az = 4.0e-2;
	GLO1_PVT		result;

	MatrixXd A(8, 8);/*状态转移矩阵A*/
	MatrixXd Q(8, 8);/*过程噪声协方差阵Q*/

	/*观测噪声协方差阵 可根据俯仰角得到*/
	double R_t1 = 0.001;
	double R_t2 = 0.002;

	MatrixXd R(2 * nsl, 2 * nsl);           // zeros(rows,cols) R:0矩阵
	MatrixXd P0(NumState, NumState);       // eye(rows,cols)P0单位矩阵
	MatrixXd Pk(NumState, NumState);        //Pk初始0矩阵
	MatrixXd detaX(NumState, 1);         //差值
	MatrixXd detaX_t(NumState, 1);        //先验预测值
	MatrixXd Y(2 * nsl, 1);         //观测向量
	MatrixXd DisX2(2 * nsl, 1);
	MatrixXd Z(2 * nsl, 1);
	MatrixXd H(2 * nsl, NumState);
	MatrixXd I(NumState, NumState);
	MatrixXd K(NumState, 2 * nsl);   //kalmanFilter增益
	VectorXd range_Final_2(nsl); //伪距率
	VectorXd satx(nsl);  //卫星校正后位置
	VectorXd saty(nsl);
	VectorXd satz(nsl);
	VectorXd alpha(nsl);
	VectorXd r0(nsl);      //伪距计算值
	VectorXd dVx(nsl);
	VectorXd dVy(nsl);
	VectorXd dVz(nsl);
	VectorXd dx(nsl);         //卫星位置
	VectorXd dy(nsl);
	VectorXd dz(nsl);
	VectorXd satVx(nsl);   //卫星速度
	VectorXd satVy(nsl);
	VectorXd satVz(nsl);
	VectorXd range_Final_1(nsl);  //伪距
	VectorXd dop(nsl);           //多普勒
	VectorXd old_receiver(NumState);       /*接收机初始状态*/
	/*初始化各矩阵*/
	A << 1, Ts, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 0,
		0, 0, 1, Ts, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 1, Ts, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 1, Ts,
		0, 0, 0, 0, 0, 0, 0, 1;

	Q <<
		Qx*Ts*Ts*Ts / 3, Qx*Ts*Ts / 2, 0, 0, 0, 0, 0, 0,
		Qx*Ts*Ts / 2, Qx*Ts, 0, 0, 0, 0, 0, 0,
		0, 0, Qy*Ts*Ts*Ts / 3, Qy*Ts*Ts / 2, 0, 0, 0, 0,
		0, 0, Qy*Ts*Ts / 2, Qy*Ts, 0, 0, 0, 0,
		0, 0, 0, 0, Qz*Ts*Ts*Ts / 3, Qx*Ts*Ts / 2, 0, 0,
		0, 0, 0, 0, Qz*Ts*Ts / 2, Qz*Ts, 0, 0,
		0, 0, 0, 0, 0, 0, Q_detaT11, Q_detaT12,
		0, 0, 0, 0, 0, 0, Q_detaT21, Q_detaT22;




	R.setZero(2 * nsl, 2 * nsl);
	P0.setIdentity(NumState, NumState);
	Pk.setZero(NumState, NumState);
	detaX.setZero(NumState, 1);
	detaX_t.setZero(NumState, 1);
	Y.setZero(2 * nsl, 1);
	DisX2.setZero(nsl, 1);
	Z.setZero(2 * nsl, 1);
	H.setZero(2 * nsl, NumState);
	I.setIdentity(NumState, NumState);      // eye(rows,cols)


	nits = 0;
	t = 0;

	x = GLO1_rec_pos_xyz.x;
	y = GLO1_rec_pos_xyz.y;
	z = GLO1_rec_pos_xyz.z;
	old_receiver(0) = /*0;//*/ x;
	old_receiver(1) = /*0;// */GLO1_rpvt.xv;
	old_receiver(2) = /*0;//*/ y;
	old_receiver(3) = /*0;// */ GLO1_rpvt.yv;
	old_receiver(4) = /*0;//*/ z;
	old_receiver(5) = /*0;// */ GLO1_rpvt.zv;
	old_receiver(6) = t;
	old_receiver(7) = 0;


	//do
	//{

	for (int j = 0; j < nsl; j++)  //vector转换成eigen
	{
		dx(j) = GLO1_track_sat[j + 1].x;
		dy(j) = GLO1_track_sat[j + 1].y;
		dz(j) = GLO1_track_sat[j + 1].z;
		satVx(j) = GLO1_d_sat[j + 1].x;
		satVy(j) = GLO1_d_sat[j + 1].y;
		satVz(j) = GLO1_d_sat[j + 1].z;
		range_Final_1(j) = GLO1_dt[j + 1];
		dop(j) = GLO1_meas_dop[j + 1];
	}

	Pk = A * 1000 * P0*(A.transpose()) + Q;
	detaX_t = A*old_receiver;// .transpose();

	/*得到观测矩阵H*/

	for (int k = 0; k < nsl; k++)
	{
		range_Final_2(k) = GLO1_lambda[k+1]* dop(k);

		alpha(k) = (range_Final_1(k)*GLO1_c - detaX_t(6))*GLO1_omegae / GLO1_c;
	}
	/*得到卫星位置信息*/
	for (int i = 0; i < nsl; i++)
	{
		satx(i) = dx(i) + dy(i)*alpha(i);
		saty(i) = dy(i) - dx(i)*alpha(i);
		satz(i) = dz(i);

		dx(i) = detaX_t(0) - satx(i);
		dy(i) = detaX_t(2) - saty(i);
		dz(i) = detaX_t(4) - satz(i);
		r0(i) = sqrt(dx(i)*dx(i) + dy(i)*dy(i) + dz(i)*dz(i));
		/*得到卫星速度*/
		dVx(i) = detaX_t(1) - satVx(i);
		dVy(i) = detaX_t(3) - satVy(i);
		dVz(i) = detaX_t(5) - satVz(i);


		H(2 * (i), 0) = dx(i) / r0(i);
		H(2 * (i), 2) = dy(i) / r0(i);
		H(2 * (i), 4) = dz(i) / r0(i);
		H(2 * (i), 6) = 1;
		//H(2 * (i)+1, 1) = dx(i) / r0(i);
		//H(2 * (i)+1, 3) = dy(i) / r0(i);
		//H(2 * (i)+1, 5) = dz(i) / r0(i);
		H(2 * (i)+1, 1) = 0;
		H(2 * (i)+1, 3) = 0;
		H(2 * (i)+1, 5) = 0;
		H(2 * (i)+1, 7) = 1;
		Y(2 * (i)) = GLO1_c*range_Final_1(i);
		Y(2 * i + 1) = range_Final_2(i);
		DisX2(i) = ((detaX_t(0) - satx(i))*(detaX_t(1) - satVx(i)) + (detaX_t(2) - saty(i))*(detaX_t(3) - satVy(i)) + (detaX_t(4) - satz(i))*(detaX_t(5) - satVz(i))) / r0(i) + detaX_t(7);
		Z(2 * i) = r0(i) + detaX_t(6);
		Z(2 * i + 1) = DisX2(i);
		R(2 * i, 2 * i) = R_t1;
		R(2 * i + 1, 2 * i + 1) = R_t2;
	}
	K = Pk*(H.transpose())*(H*Pk*(H.transpose()) + R).inverse();
	detaX = K*(Y - Z);
	Pk = (I - K*H)*Pk;
	old_receiver = detaX_t + detaX;
	//nits++;
	//} while (nits<10);
	result.x = old_receiver(0);
	result.y = old_receiver(2);
	result.z = old_receiver(4);
	result.xv = old_receiver(1)/*+ old_receiver(2)*omegae*/;
	result.yv = old_receiver(3)/*- old_receiver(0)*omegae*/;
	result.zv = old_receiver(5);
	result.dt = old_receiver(6) / GLO1_c;
	result.df = old_receiver(7) / GLO1_c*1000000.0;

	return(result);
}


/*******************************************************************************
FUNCTION dops(int nsl)

RETURNS  None.

PARAMETERS
			nsl  int

PURPOSE

	This routine computes the dops

INPUTS:
	sat_location[nsl][3] Array of satellite locations in ECEF when the signal
								 was sent
    nsl      number of satellites used
    receiver position

OUTPUTS:
	hdop = horizontal dilution of precision (rss of ndop & edop)
    vdop = vertical dilution of precision
    tdop = time dilution of precision
	pdop = position dilution of precision (rss of vdop & hdop)
    gdop = geometric dilution of precision (rss of pdop & tdop)

*******************************************************************************/
void  GLO1_dops( int nsl)
{
	double r,xls,yls,zls;
	int i;
	double **H, **G;
	
	H = GLO1_dmatrix(nsl, 4);
	G = GLO1_dmatrix(4, 4);
	
	GLO1_receiver.north.x=-cos(GLO1_rec_pos_llh.lon)*sin(GLO1_rec_pos_llh.lat);
	GLO1_receiver.north.y=-sin(GLO1_rec_pos_llh.lon)*sin(GLO1_rec_pos_llh.lat);
	GLO1_receiver.north.z= cos(GLO1_rec_pos_llh.lat);
	GLO1_receiver.east.x=-sin(GLO1_rec_pos_llh.lon);
	GLO1_receiver.east.y= cos(GLO1_rec_pos_llh.lon);
	GLO1_receiver.up.x=cos(GLO1_rec_pos_llh.lon)*cos(GLO1_rec_pos_llh.lat);
	GLO1_receiver.up.y=sin(GLO1_rec_pos_llh.lon)*cos(GLO1_rec_pos_llh.lat);
	GLO1_receiver.up.z=sin(GLO1_rec_pos_llh.lat);
	for (i=1;i<=nsl;i++)
	{
		//   Compute line of sight vectors
		xls=GLO1_track_sat[i].x-GLO1_rec_pos_xyz.x;
		yls=GLO1_track_sat[i].y-GLO1_rec_pos_xyz.y;
		zls=GLO1_track_sat[i].z-GLO1_rec_pos_xyz.z;

		r=sqrt(xls*xls+yls*yls+zls*zls);

		H[i-1][0]=(xls*GLO1_receiver.north.x+yls*GLO1_receiver.north.y+zls*GLO1_receiver.north.z)/r;
		H[i-1][1]=(xls*GLO1_receiver.east.x+yls*GLO1_receiver.east.y)/r;
		H[i-1][2]=(xls*GLO1_receiver.up.x+yls*GLO1_receiver.up.y+zls*GLO1_receiver.up.z)/r;
		H[i-1][3]=1.0;
	}

	GLO1_mult_ATB(H, nsl, 4, H, nsl, 4, G);
	GLO1_matinv(G, 4);
	GLO1_hdop = sqrt(G[0][0] + G[1][1]);
	GLO1_vdop = sqrt(G[2][2]);
	GLO1_tdop = sqrt(G[3][3]);
	GLO1_pdop = sqrt(G[0][0] + G[1][1] + G[2][2]);
	GLO1_gdop=sqrt(G[0][0]+G[1][1]+G[2][2]+G[3][3]);
	GLO1_free_dmatrix(G, 4);
	GLO1_free_dmatrix(H, nsl);
}

/*******************************************************************************
FUNCTION tropo_iono(float az, float el, double gps_time)
RETURNS  signal time delay due to troposphere and ionosphere (single frequency)

PARAMETERS
			az         float
			el         float
			gps_time   double

PURPOSE
	This function corrects the pseudoranges with a tropospheric model
	and the broadcast ionospheric message corrections.

*******************************************************************************/

double GLO1_tropo_iono(char ch, double az, double el, double gps_time)
{
	double d_Trop,alt_factor;
	double d_Ion,psi,phi,lambdai,phim,per,x,F,amp,t;

	if (GLO1_current_loc.hae>200000.0) alt_factor=0.0;
	else if (GLO1_current_loc.hae<0.0) alt_factor=1.0;
	else alt_factor=exp(-GLO1_current_loc.hae*1.33e-4);

	d_Trop = (GLO1_m_tropo==1) ? 2.47/(sin(el)+.0121)*alt_factor/GLO1_c : 0.0;
	GLO1_chan[ch].Tropo=d_Trop;

	if (GLO1_m_iono==1)
	{
		psi=0.0137/(el/GLO1_pi+0.11)-.022;
		phi=GLO1_current_loc.lat/GLO1_pi+psi*cos(az);
		if (phi > 0.416) phi= 0.416;
		else if (phi <-0.416 ) phi=-0.416;
		lambdai=GLO1_current_loc.lon/GLO1_pi+psi*sin(az)/cos(phi*GLO1_pi);
		t=43200.0*lambdai+gps_time-int((43200.0*lambdai+gps_time)/86400.)*86400.;
		if (t<0.0) t=t+86400.0;
		phim=phi+0.064*cos((lambdai-1.617)*GLO1_pi);
		
		//  If available from the nav message use its Ionosphere model
		if (GLO1_b0 != 0.0 && GLO1_al0 != 0.0)
		{
			per=GLO1_b0+GLO1_b1*phim+GLO1_b2*phim*phim+GLO1_b3*phim*phim*phim;
			amp=GLO1_al0+GLO1_al1*phim+GLO1_al2*phim*phim+GLO1_al3*phim*phim*phim;
		}
		else // else try this set of default iono model parameters
		{
			per=141312.0-32768.0*phim-131072.0*phim*phim-65536.0*phim*phim*phim;
			amp=3.46e-8+7.45e-9*phim-1.19e-7*phim*phim+5.96e-8*phim*phim*phim;
		}
		if ( per <72000.0 ) per=72000.0;
		x=2.*GLO1_pi*(t-50400.)/per;
		F=1.0+16.0*pow(0.53-el/GLO1_pi,3);
		if ( amp < 0.0 ) amp=0.0;
		if (fabs(x) < 1.5707)  d_Ion=F*(5.0e-9+amp*(1.0-x*x/2.+x*x*x*x/24.0));
		else                   d_Ion=F*5.0e-9;
	}
	else d_Ion=0.0;
	GLO1_chan[ch].Iono=d_Ion;
	return(d_Trop+d_Ion);
}

/*******************************************************************************
FUNCTION read_ion_utc(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function reads the broadcast ionospheric correction model and the
	gps time to UTC conversion parameters from "ion_utc.dat" which is
	created by the program when the data has been read from the satellites

*******************************************************************************/
char GLO1_ion_utc_file[4096];
void  GLO1_read_ion_utc(void)
{
	if ((GLO1_in = fopen(GLO1_ion_utc_file, "rt")) == NULL)
	{
		printf("Cannot open ion_utc.dat file.\n");
	}
	else
	{
		GLO1_handle=_fileno(GLO1_in);
		while (!_eof(GLO1_handle))
		{
			//ion
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_al0);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_al1);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_al2);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_al3);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_b0);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_b1);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_b2);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_b3);
			//utc
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_a0);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_a1);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_dtls);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_tot);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_WNt);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_WNlsf);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_DN);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%lf",&GLO1_dtlsf);
		}
		fclose(GLO1_in);
	}
	
}

/*******************************************************************************
FUNCTION read_almanac(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function reads the almanac parameters from "current.alm" which is
	created by the program when the data has been read from the satellites

*******************************************************************************/
char GLO1_current_alm_file[4096];
void GLO1_read_almanac(void)
{
	int   id,health,week;
	float eccen,rinc,rras,sqra;
	float ratoa,aopg,rma,af0,af1,toa;

	if ((GLO1_in = fopen(GLO1_current_alm_file, "rt")) == NULL)
	{
		printf("Cannot open currrent.rcv file.\n");
		for (id=1;id<=32;id++)
		{
			GLO1_gps_alm[id].week=GLO1_gps_week-1;
			GLO1_gps_alm[id].inc=1.0;
		}
	}
	else
	{
		GLO1_status=GLO1_warm_start;
		GLO1_handle=_fileno(GLO1_in);
		while (!_eof(GLO1_handle))
		{
			fscanf(GLO1_in,"%46c",&GLO1_header);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%d",&id);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%i",&health);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&eccen);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&toa);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&rinc);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&rras);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&sqra);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&ratoa);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&aopg);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&rma);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&af0);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%f",&af1);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%i",&week);
			fscanf(GLO1_in,"%c",&GLO1_trailer);
			GLO1_gps_alm[id].health=health;
			GLO1_gps_alm[id].week=week;
			GLO1_gps_alm[id].toa=toa;
			GLO1_gps_alm[id].ety=eccen;
			GLO1_gps_alm[id].toa=toa;
			GLO1_gps_alm[id].inc=rinc;
			GLO1_gps_alm[id].rra=rras;
			GLO1_gps_alm[id].sqa=sqra;
			GLO1_gps_alm[id].lan=ratoa;
			GLO1_gps_alm[id].aop=aopg;
			GLO1_gps_alm[id].ma=rma;
			GLO1_gps_alm[id].af0=af0;
			GLO1_gps_alm[id].af1=af1;
			GLO1_gps_alm[id].sat_file=0;
			if (GLO1_gps_alm[id].sqa>0.0) GLO1_gps_alm[id].w=19964981.84/pow(GLO1_gps_alm[id].sqa,3);
		}
		fclose(GLO1_in);
		GLO1_alm_gps_week=week;
		GLO1_alm_toa=toa;
	}
	
}

/*******************************************************************************
FUNCTION read_ephemeris(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function reads the ephemeris parameters from "current.eph" which is
	created by the program when the data has been read from the satellites

*******************************************************************************/
char GLO1_current_eph_file[4096];
void GLO1_read_ephemeris()
{
	int   id,health,week,ura,iode,iodc;
	double  toc,toe;
	double crc,crs,cic,cis,cuc,cus,tgd,ety,inc0,omegadot,w0,w,ma,dn,idot;
	double daf0,daf1,daf2,esqra;
	double d_toe;
	FILE   *ephfile;
	int i;
	if ((ephfile = fopen(GLO1_current_eph_file, "rt")) == NULL)
	{
		printf("Cannot open currrent.eph file.\n");
	}
	else
	{
		GLO1_availablesv.availablesvnumber=0;//统计可见星数量用于热启动。
		i=0;

		while (!feof(ephfile))
		{
			fscanf(GLO1_in,"%37c",&GLO1_header);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%i",&id);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%i",&health);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%i",&week);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%i",&ura);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&toe);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%i",&iode);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&toc);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%i",&iodc);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&tgd);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&daf0);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&daf1);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&daf2);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&ety);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&inc0);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&idot);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&omegadot);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&esqra);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&dn);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&w0);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&w);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&ma);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&cuc);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&cus);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&crc);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&crs);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&cic);
			fscanf(GLO1_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_in,"%le",&cis);
			fscanf(GLO1_in,"%c",&GLO1_trailer);

			d_toe=GLO1_clock_tow-toe;
			if (d_toe>302400.0) d_toe=d_toe-604800.0;
			else if (d_toe<-302400.0)d_toe=d_toe+604800.0;

			//    If week is current and time is less than 2 hours old use for hot start
			//    note: GLO1_gps_week is computed from the PCs real time clock
			//          and does not roll over at 1024
			if ( fabs(d_toe)<7200.0)//如果存储的星历TOE和当前时间相差在2小时以内，则可以用
			{
				GLO1_gps_eph[id].valid=1;
				GLO1_gps_eph[id].health=health;
				GLO1_gps_eph[id].week=week;
				GLO1_gps_eph[id].ura=ura;
				GLO1_gps_eph[id].toe=toe;
				GLO1_gps_eph[id].iode=iode;
				GLO1_gps_eph[id].toc=toc;
				GLO1_gps_eph[id].iodc=iodc;
				GLO1_gps_eph[id].tgd=tgd;
				GLO1_gps_eph[id].af0=daf0;
				GLO1_gps_eph[id].af1=daf1;
				GLO1_gps_eph[id].af2=daf2;
				GLO1_gps_eph[id].ety=ety;
				GLO1_gps_eph[id].inc0=inc0;
				GLO1_gps_eph[id].idot=idot;
				GLO1_gps_eph[id].omegadot=omegadot;
				GLO1_gps_eph[id].sqra=esqra;
				GLO1_gps_eph[id].dn=dn;
				GLO1_gps_eph[id].w0=w0;
				GLO1_gps_eph[id].w=w;
				GLO1_gps_eph[id].ma=ma;
				GLO1_gps_eph[id].cuc=cuc;
				GLO1_gps_eph[id].cus=cus;
				GLO1_gps_eph[id].crc=crc;
				GLO1_gps_eph[id].crs=crs;
				GLO1_gps_eph[id].cic=cic;
				GLO1_gps_eph[id].cis=cis;
				if (GLO1_gps_eph[id].sqra >0.0) GLO1_gps_eph[id].wm=19964981.84/pow(GLO1_gps_eph[id].sqra,3);
				if (GLO1_out_debug)  GLO1_write_Debug_ephemeris(id);

				GLO1_availablesv.availablesvnumber ++;
	         	GLO1_availablesv.availablesvprn[i] = id;
	         	i++;
			}
		} 
		fclose(GLO1_in);
	}
}
char GLO1_last_prn_file[4096];
void GLO1_read_prn(void)
{
	int   i = 0,id;
	
	if ((GLO1_fprn_in = fopen(GLO1_last_prn_file, "rt")) == NULL)
	{
		printf("Cannot open lastime.prn file.\n");
	}
	else
	{
		for (i = 0;i <= GLO1_chmax;i++)
		{
			fscanf(GLO1_fprn_in,"%27c",&GLO1_ttext);
			fscanf(GLO1_fprn_in,"%d",&id);
		}
		fclose(GLO1_fprn_in);
	}
}


void GLO1_write_prn()
{
	int i;
	GLO1_fprn_out = fopen(GLO1_last_prn_file, "w+");
	for (i=0;i<=GLO1_chmax;i++)
	{
		fprintf(GLO1_fprn_out,"prn:                       %4d\n",GLO1_chan[i].prn);

	}
	fclose(GLO1_fprn_out);
}
/*******************************************************************************
FUNCTION write_almanac()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast almanac data to a file for later
	use.  In particular to support a warm or hot start.

*******************************************************************************/

void GLO1_write_almanac()
{
	int i;
	GLO1_out = fopen(GLO1_current_alm_file, "w+");
	for (i=1;i<=32;i++)
	{
		if ( GLO1_gps_alm[i].inc >0.0)
		{
			fprintf(GLO1_out,"**** Week %4d almanac for PRN-%2d ***********\n",
				GLO1_gps_alm[i].week%1024,i);
			fprintf(GLO1_out,"ID:                         %3d\n",i);
			fprintf(GLO1_out,"Health:                     %3d\n",GLO1_gps_alm[i].health);
			fprintf(GLO1_out,"Eccentricity:               %10.9e\n",GLO1_gps_alm[i].ety);
			fprintf(GLO1_out,"Time of Applicability(s):   %10.9e\n",GLO1_gps_alm[i].toa);
			fprintf(GLO1_out,"Orbital Inclination(rad):   %10.9e\n",GLO1_gps_alm[i].inc);
			fprintf(GLO1_out,"Rate of Right Ascen(R/s):   %10.9e\n",GLO1_gps_alm[i].rra);
			fprintf(GLO1_out,"SQRT(A) (m^1/2):            %10.9e\n",GLO1_gps_alm[i].sqa);
			fprintf(GLO1_out,"Right Ascen at TOA(rad):    %10.9e\n",GLO1_gps_alm[i].lan);
			fprintf(GLO1_out,"Argument of Perigee(rad):   %10.9e\n",GLO1_gps_alm[i].aop);
			fprintf(GLO1_out,"Mean Anom(rad):             %10.9e\n",GLO1_gps_alm[i].ma);
			fprintf(GLO1_out,"Af0(s):                     %10.9e\n",GLO1_gps_alm[i].af0);
			fprintf(GLO1_out,"Af1(s/s):                   %10.9e\n",GLO1_gps_alm[i].af1);
			fprintf(GLO1_out,"week:                       %4d   \n",GLO1_gps_alm[i].week%1024);
			fprintf(GLO1_out,"\n");
		}
	}
	fclose(GLO1_out);
}

/*******************************************************************************
FUNCTION write_ephemeris()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ephemeris data to a file for later
	use.  In particular to support a hot start.

*******************************************************************************/

void GLO1_write_ephemeris()
{
	int i;
	GLO1_out = fopen(GLO1_current_eph_file, "w+");
	for (i=1;i<=32;i++)
	{
		if ( GLO1_gps_eph[i].inc0 >0.0)
		{
			fprintf(GLO1_out,"**** Ephemeris for PRN-%2d ***********\n",i);
			fprintf(GLO1_out,"ID:                         %3d\n",i);
			fprintf(GLO1_out,"Health:                     %3d\n",GLO1_gps_eph[i].health);
			fprintf(GLO1_out,"Week:                       %4d\n",   GLO1_gps_eph[i].week);
			fprintf(GLO1_out,"URA:                        %3d\n",   GLO1_gps_eph[i].ura);
			fprintf(GLO1_out,"E Time of Applic(s):        %12.11e\n",GLO1_gps_eph[i].toe);
			fprintf(GLO1_out,"IODE:                       %4d\n",   GLO1_gps_eph[i].iode);
			fprintf(GLO1_out,"C Time of Applic(s):        %12.11e\n",GLO1_gps_eph[i].toc);
			fprintf(GLO1_out,"IODC:                       %4d\n",   GLO1_gps_eph[i].iodc);
			fprintf(GLO1_out,"Tgd(s):                     %12.11e\n",GLO1_gps_eph[i].tgd);
			fprintf(GLO1_out,"Af0(s):                     %12.11e\n",GLO1_gps_eph[i].af0);
			fprintf(GLO1_out,"Af1(s/s):                   %12.11e\n",GLO1_gps_eph[i].af1);
			fprintf(GLO1_out,"Af2(s/s/s):                 %12.11e\n",GLO1_gps_eph[i].af2);
			fprintf(GLO1_out,"Eccentricity:               %12.11e\n",GLO1_gps_eph[i].ety);
			fprintf(GLO1_out,"Orbital Inclination(rad):   %12.11e\n",GLO1_gps_eph[i].inc0);
			fprintf(GLO1_out,"inc rate (r/s)              %12.11e\n",GLO1_gps_eph[i].idot);
			fprintf(GLO1_out,"Rate of Right Ascen(R/s):   %12.11e\n",GLO1_gps_eph[i].omegadot);
			fprintf(GLO1_out,"SQRT(A) (m^1/2):            %12.11e\n",GLO1_gps_eph[i].sqra);
			fprintf(GLO1_out,"dn                          %12.11e\n",GLO1_gps_eph[i].dn);
			fprintf(GLO1_out,"Right Ascen at TOE(rad):    %12.11e\n",GLO1_gps_eph[i].w0);
			fprintf(GLO1_out,"Argument of Perigee(rad):   %12.11e\n",GLO1_gps_eph[i].w);
			fprintf(GLO1_out,"Mean Anom(rad):             %12.11e\n",GLO1_gps_eph[i].ma);
			fprintf(GLO1_out,"Cuc(rad):                   %12.11e\n",GLO1_gps_eph[i].cuc);
			fprintf(GLO1_out,"Cus(rad):                   %12.11e\n",GLO1_gps_eph[i].cus);
			fprintf(GLO1_out,"Crc(m):                     %12.11e\n",GLO1_gps_eph[i].crc);
			fprintf(GLO1_out,"Crs(m):                     %12.11e\n",GLO1_gps_eph[i].crs);
			fprintf(GLO1_out,"Cic(rad):                   %12.11e\n",GLO1_gps_eph[i].cic);
			fprintf(GLO1_out,"Cis(rad):                   %12.11e\n",GLO1_gps_eph[i].cis);
			fprintf(GLO1_out,"\n");
		}
	}
	fclose(GLO1_out);
}

/*******************************************************************************
FUNCTION write_ephemeris()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ephemeris data to a file for later
	use.  In particular to support a hot start.

*******************************************************************************/
void GLO1_write_Debug_ephemeris(int i)
{
	fprintf(GLO1_debug, "**** Ephemeris for PRN-%2d ***********\n", i);
	fprintf(GLO1_debug,"ID:                         %3d\n",i);
	fprintf(GLO1_debug,"Health:                     %3d\n",GLO1_gps_eph[i].health);
	fprintf(GLO1_debug,"Week:                       %4d\n",   GLO1_gps_eph[i].week);
	fprintf(GLO1_debug,"URA:                        %3d\n",   GLO1_gps_eph[i].ura);
	fprintf(GLO1_debug,"E Time of Applic(s):        %12.11e\n",GLO1_gps_eph[i].toe);
	fprintf(GLO1_debug,"IODE:                       %4d\n",   GLO1_gps_eph[i].iode);
	fprintf(GLO1_debug,"C Time of Applic(s):        %12.11e\n",GLO1_gps_eph[i].toc);
	fprintf(GLO1_debug,"IODC:                       %4d\n",   GLO1_gps_eph[i].iodc);
	fprintf(GLO1_debug,"Tgd(s):                     %12.11e\n",GLO1_gps_eph[i].tgd);
	fprintf(GLO1_debug,"Af0(s):                     %12.11e\n",GLO1_gps_eph[i].af0);
	fprintf(GLO1_debug,"Af1(s/s):                   %12.11e\n",GLO1_gps_eph[i].af1);
	fprintf(GLO1_debug,"Af2(s/s/s):                 %12.11e\n",GLO1_gps_eph[i].af2);
	fprintf(GLO1_debug,"Eccentricity:               %12.11e\n",GLO1_gps_eph[i].ety);
	fprintf(GLO1_debug,"Orbital Inclination(rad):   %12.11e\n",GLO1_gps_eph[i].inc0);
	fprintf(GLO1_debug,"inc rate (r/s)              %12.11e\n",GLO1_gps_eph[i].idot);
	fprintf(GLO1_debug,"Rate of Right Ascen(R/s):   %12.11e\n",GLO1_gps_eph[i].omegadot);
	fprintf(GLO1_debug,"SQRT(A) (m^1/2):            %12.11e\n",GLO1_gps_eph[i].sqra);
	fprintf(GLO1_debug,"dn                          %12.11e\n",GLO1_gps_eph[i].dn);
	fprintf(GLO1_debug,"Right Ascen at TOE(rad):    %12.11e\n",GLO1_gps_eph[i].w0);
	fprintf(GLO1_debug,"Argument of Perigee(rad):   %12.11e\n",GLO1_gps_eph[i].w);
	fprintf(GLO1_debug,"Mean Anom(rad):             %12.11e\n",GLO1_gps_eph[i].ma);
	fprintf(GLO1_debug,"Cuc(rad):                   %12.11e\n",GLO1_gps_eph[i].cuc);
	fprintf(GLO1_debug,"Cus(rad):                   %12.11e\n",GLO1_gps_eph[i].cus);
	fprintf(GLO1_debug,"Crc(m):                     %12.11e\n",GLO1_gps_eph[i].crc);
	fprintf(GLO1_debug,"Crs(m):                     %12.11e\n",GLO1_gps_eph[i].crs);
	fprintf(GLO1_debug,"Cic(rad):                   %12.11e\n",GLO1_gps_eph[i].cic);
	fprintf(GLO1_debug,"Cis(rad):                   %12.11e\n",GLO1_gps_eph[i].cis);
	fprintf(GLO1_debug,"\n");
}


/*******************************************************************************
FUNCTION write_ion_utc()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ionospheric correction data and the
      parameters to tie GPS time to UTC to a file for later use.

*******************************************************************************/

void GLO1_write_ion_utc()
{
	GLO1_out = fopen(GLO1_ion_utc_file, "w+");
	fprintf(GLO1_out,"GLO1_al0:                        %e\n",GLO1_al0);
	fprintf(GLO1_out,"GLO1_al1:                        %e\n",GLO1_al1);
	fprintf(GLO1_out,"GLO1_al2:                        %e\n",GLO1_al2);
	fprintf(GLO1_out,"GLO1_al3:                        %e\n",GLO1_al3);
	fprintf(GLO1_out,"b0:                         %f\n",GLO1_b0);
	fprintf(GLO1_out,"b1:                         %f\n",GLO1_b1);
	fprintf(GLO1_out,"b2:                         %f\n",GLO1_b2);
	fprintf(GLO1_out,"GLO1_b3:                         %f\n",GLO1_b3);
	fprintf(GLO1_out,"GLO1_a0                          %f\n",GLO1_a0);
	fprintf(GLO1_out,"GLO1_a1                          %f\n",GLO1_a1);
	fprintf(GLO1_out,"GLO1_dtls                        %f\n",GLO1_dtls);
	fprintf(GLO1_out,"GLO1_tot                         %f\n",GLO1_tot);
	fprintf(GLO1_out,"GLO1_WNt                         %f\n",GLO1_WNt);
	fprintf(GLO1_out,"GLO1_WNlsf                       %f\n",GLO1_WNlsf);
	fprintf(GLO1_out,"GLO1_DN                          %f\n",GLO1_DN);
	fprintf(GLO1_out,"GLO1_dtlsf                       %f\n",GLO1_dtlsf);
	fclose(GLO1_out);
}

/*******************************************************************************
FUNCTION rss(long a, long b)
RETURNS  long integer

PARAMETERS
      a  long integer
		b  long integer

PURPOSE
	This function finds the fixed point magnitude of a 2 dimensional vector

*******************************************************************************/

/*******************************************************************************
FUNCTION fix_sqrt(long x)
RETURNS  long integer

PARAMETERS
      x long integer

PURPOSE
	This function finds the fixed point square root of a long integer

*******************************************************************************/

long GLO1_fix_sqrt(long x)
{
	long xt,scr;
	int i;
	i=0;
	xt=x;
	do
	{
		xt=xt>>1;
		i++;
	} while (xt>0);

	i=(i>>1)+1;
	xt=x>>i;
	do
	{
		scr=xt*xt;
		scr=x-scr;
		scr=scr>>1;
		scr=scr/xt;
		xt=scr+xt;
	} while (scr!=0);
	xt=xt<<7;
	return(xt);
}

/*******************************************************************************
FUNCTION fix_atan2(long y,long x)
RETURNS  long integer

PARAMETERS 
		x  long   in-phase fixed point value
		y  long   quadrature fixed point value

PURPOSE
      This function computes the fixed point arctangent represented by
      x and y in the parameter list
      1 radian = 16384
      based on the power series  f-f^3*2/9

	Fixed for y==x  added special code for x==0 suggested by Joel Barnes, UNSW
*******************************************************************************/

#define GLO1_SCALED_PI_ON_2  25736L
#define GLO1_SCALED_PI       51472L

inline long GLO1_fix_atan2(long y,long x)
{
	long result,n,n3;
	if ((x==0) && (y==0))
		return(0); 

	if (x>0 &&  x>=labs(y))
	{
		n=(y<<14)/x;
		n3=((((n*n)>>14)*n)>>13)/9;
		result=n-n3;
	}
	else if (x<=0 && -x>=labs(y))
	{
		n=(y<<14)/x;
		n3=((((n*n)>>14)*n)>>13)/9;
		result = (y>0) ? n - n3 + GLO1_SCALED_PI : n - n3 - GLO1_SCALED_PI;
	}
	else if (y>0 &&  y>labs(x))
	{
		n=(x<<14)/y;
		n3=((((n*n)>>14)*n)>>13)/9;
		result = GLO1_SCALED_PI_ON_2 - n + n3;
	}
	else if (y<0 && -y>labs(x))
	{
		n=(x<<14)/y;
		n3=((((n*n)>>14)*n)>>13)/9;
		result = -n + n3 - GLO1_SCALED_PI_ON_2;
	}
	return(result);
}


int GLO1_matinv(double **q, int n)
{
	int i, j, k, n1;
	double s;

	for (i=1; i< n; i++) q[i][0] /= q[0][0];
	for (i=1; i< n; i++)
	{
		s = 0.0;
		for (k=0; k< i; k++) s += q[i][k] * q[i][k] * q[k][k];
		q[i][i] -= s;
        n1 = n -1;
        if (i < n1)
        {
			for(j=i+1; j< n; j++)
			{
				s = 0.0;
				for( k=0; k< i; k++) s += q[i][k] * q[j][k] * q[k][k];
				q[j][i] = (q[j][i] - s)/q[i][i];
			}
		}
	}

	n1 = n - 2;
	for (i=0; i<= n1; i++) 
	{
		q[i+1][i] = -q[i+1][i];
		if (i < n1)
		{
			for(j=i+2; j< n; j++)
			{
				s = q[j][i];
				for(k=i+1; k<j; k++) s += q[j][k] * q[k][i];
				q[j][i] = -s;
			}
		}
	}

	n1 = n - 1;
	for (i=0; i< n; i++)
	{
		for (j=i; j< n; j++) 
		{
			s = (i == j) ? 1.0/q[j][j] : q[j][i]/q[j][j];
			if (i == n1) q[j][i] = s;
			else if (j < n1) 
			{
				for(k=j+1; k< n; k++) s += q[k][i]*q[k][j]/q[k][k];
				q[j][i] = s;
			}
			else  q[n1][i] /= q[n1][n1]; 
		}
	}
	for (i=0; i< n; i++) for (j=i; j< n; j++) q[i][j] = q[j][i];
	return 0;
}



void GLO1_mult_ATB(double **A, int arows, int acols, double **B, int brows,
                                  int bcols, double **C)
{
	int i, j, k;

	if(arows!=brows)
	{
		printf("matrices not compatible in mult_ABT()!!   bye\n");
		exit(0);
	}

	for(i=0; i<acols; i++)
	{
		for(j=0; j<bcols; j++)
		{
			C[i][j] = 0.0;
			for(k=0; k<arows; k++)	C[i][j] += A[k][i]*B[k][j];
		}
	}
}

double** GLO1_dmatrix(unsigned row, unsigned column)
{
	unsigned i;
	double  **m;

	m = (double **) malloc(row * sizeof(double *));
	if (m==NULL)  
	{
		printf("Allocation failure 1 in dmatrix()\n");
		exit(0);
	}
	for (i=0; i<row; i++)
	{
		m[i] = (double *) malloc(column * sizeof(double));
		if (m[i]==NULL)
		{
           printf(" Allocation failure 2 in dmatrix()\n ");
		   exit(0);
		}
	}
	return m;
}


void GLO1_free_dmatrix(double **m, unsigned row)
{
	unsigned i;
	for (i=0; i<row; i++) free((double*)m[i]);
	free((double **)m);
}
