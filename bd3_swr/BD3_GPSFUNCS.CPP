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
#include "BD3_gpsstruct.h"
#include "BD3_gpsconst.h"
#include "BD3_gpsfuncs.h"
#include "BD3_gpsrcvr.h"//add bj jh 3-16
// external variables
extern BD3_CHANNEL	BD3_chan[12];
extern int		BD3_out_debug;
extern FILE*	BD3_fpeph;
extern int sel;

// global variables
char			BD3_tzstr[40];
time_t			BD3_thetime;
FILE			*BD3_stream, *BD3_debug, *BD3_in, *BD3_out, *BD3_kalm;
FILE            *BD3_fprn_in, *BD3_fprn_out;
BD3_ALMANAC			BD3_gps_alm[13];
BD3_EPHEMERIS		BD3_gps_eph[13];

BD3_AVAILABLESV      BD3_availablesv;


int				BD3_SVh[13], BD3_ASV[13];
double			BD3_b0, BD3_b1, BD3_b2, BD3_b3, BD3_al0, BD3_al1, BD3_al2, BD3_al3;		// broadcast ionospheric delay model
double			BD3_a0, BD3_a1, BD3_tot, BD3_WNt, BD3_dtls, BD3_WNlsf, BD3_DN, BD3_dtlsf;	//broadcast UTC data

BD3_PVT				BD3_rpvt;
BD3_STATE			BD3_receiver;

double			BD3_gdop, BD3_pdop, BD3_hdop, BD3_vdop, BD3_tdop, BD3_alm_toa;
unsigned long	BD3_clock_tow;
BD3_LLH				BD3_rec_pos_llh;
BD3_LLH				BD3_current_loc, BD3_rp_llh;
BD3_ECEFT			BD3_track_sat[13];
BD3_XYZ				BD3_rec_pos_xyz;
int				BD3_alm_gps_week, BD3_gps_week, BD3_almanac_valid, BD3_almanac_flag, BD3_handle;
						
unsigned long	BD3_sf[13][11][6][11];							// 用于存储5个子帧10个字/页)的导航电文，13表示通道数，11表示页面数，6表示子桢数，11表示字数
////////2010.11.16//////////////////////////////////////////////////////////////////
unsigned long	BD3_sfd1[6][11];							// 用于存储5个子帧10个字/页)的导航电文，sf[0][]无效，sf[][0]无效

int				BD3_p_error[6];							// 用于存储5个子帧(10个字/页)的导航电文的校验状态
													// LSB 10位有效，每个比特对应一个字的校验状态，0：正确，1：错误

int				BD3_status;								// 表征接收机的状态，只有三个值:
													// cold_start, warm_start和navigating

unsigned long	BD3_test_l[33] = { 0x00000000L,            // single bit set numbers
  0x00000001L,0x00000002L,0x00000004L,0x00000008L,  // for testing bit positions
  0x00000010L,0x00000020L,0x00000040L,0x00000080L,	// 0000 0001 0010 0100 1000
  0x00000100L,0x00000200L,0x00000400L,0x00000800L,	// 32位的第1位到第32位
  0x00001000L,0x00002000L,0x00004000L,0x00008000L,
  0x00010000L,0x00020000L,0x00040000L,0x00080000L,
  0x00100000L,0x00200000L,0x00400000L,0x00800000L,
  0x01000000L,0x02000000L,0x04000000L,0x08000000L,
  0x10000000L,0x20000000L,0x40000000L,0x80000000L};

double		BD3_mask_angle;
char		BD3_header[45], BD3_ttext[27], BD3_trailer;
double		BD3_meas_dop[13];							// 测量的每个通道的多普勒
BD3_XYZ			BD3_d_sat[13];								// 每个通道的卫星的速度
double		BD3_carrier_ref;				// 载波标称中频，单位：Hz
//double		code_ref=2046000;						// CA码速率标频，单位：chip/s
double		BD3_code_ref = 10230000;
double		BD3_dt[13];									// 从卫星到接收机的传播时间差
double		BD3_cbias;									// 接收机钟差

// binary constants for nav message decoding

double BD3_c_2p12 = 4096;
double BD3_c_2p4 = 16;
double BD3_c_2m5 = 0.03125;
double BD3_c_2m6 = 0.015625;//add by jh 3-16
double BD3_c_2m11 = 4.8828125e-4;
double BD3_c_2m19 = 1.9073486328125e-6;
double BD3_c_2m20 = 9.5367431640625e-7;
double BD3_c_2m21 = 4.76837158203125e-7;
double BD3_c_2m23 = 1.19209289550781e-7;
double BD3_c_2m24 = 5.96046447753906e-8;
double BD3_c_2m27 = 7.45058059692383e-9;
double BD3_c_2m29 = 1.86264514923096e-9;
double BD3_c_2m30 = 9.31322574615479e-10;
double BD3_c_2m31 = 4.65661287307739e-10;
double BD3_c_2m33 = 1.16415321826935E-10;
double BD3_c_2m38 = 3.63797880709171e-12;
double BD3_c_2m43 = 1.13686837721616e-13;
double BD3_c_2m50 = 8.881784197e-16;
double BD3_c_2m55 = 2.77555756156289e-17;
double BD3_c_2m66 = 1.35525271560688e-20;

int			BD3_m_tropo, BD3_m_iono;				// flags for using tropo and iono models
int			BD3_align_t;						// 将接收机时间和GPS时间对准的标志
BD3_SATVIS		BD3_xyz[13];//33						// 卫星的位置、方位和多普勒

#define BD3_bit_test_l(BD3_data,BD3_bit_n)	(BD3_data & BD3_test_l[BD3_bit_n]) ? 1 : 0
char BD3_last_prn[12];
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
BD3_SATVIS BD3_satfind(char i)
{
	double	tdot,az;
	double	satang,alm_time,almanac_date;
	double	range1,range2,xls,yls,zls,xaz,yaz, rangeInv;
	long	jd_yr;
	BD3_XYZ		gpspos1, gpspos2, north, east, up;
	BD3_SATVIS	result;
	int		jd_m;
	struct	tm *gmt;
	double  time_s;

    
	// INITIALIZE ALL THE CONSTANTS
	// 防止返回结果混乱
	result.azimuth = result.doppler = result.elevation = 0.0;
	result.x = result.y = result.z = 0.0;

	_putenv(BD3_tzstr);
	_tzset();
	gmt = gmtime(&BD3_thetime);
	
	if (gmt->tm_mon <= 1)
	{	
		jd_yr = (long) ((long long)(365.25*(gmt->tm_year-1.+1900.)));
		jd_m  = (int) (long long) (30.6001*(gmt->tm_mon+14.));
	}
	else
	{
		jd_yr = (long)(long long)(365.25*(gmt->tm_year+1900.));
		jd_m = (int)(long long)(30.6001*(gmt->tm_mon+2.));
	}
	time_s=gmt->tm_min/1440.+gmt->tm_sec/86400.+1720981.5+gmt->tm_hour/24.
		+jd_yr+jd_m+gmt->tm_mday;
	// 估计GPS的星期数
	BD3_gps_week = int((time_s - 2444244.5) / 7.);
	almanac_date = BD3_gps_alm[i].week*7.0 + 2444244.5;

	if (BD3_gps_week - BD3_gps_alm[i].week>512) almanac_date += 1024 * 7.0;
	alm_time=(time_s-almanac_date)*86400.;

	BD3_clock_tow = (long)(long long)((time_s - BD3_gps_week*7. - 2444244.5)*86400.);

	// CALCULATE THE POSITION OF THE SATELLITES 
	// use inclincation angle and sv number as the flag of validity
	if (BD3_gps_alm[i].inc > 0.0 && i>0)
    {
		gpspos1 = BD3_satpos_almanac(alm_time, i);
		
		gpspos2 = BD3_satpos_almanac(alm_time + 1.0, i);
		BD3_rec_pos_xyz = BD3_llh_to_ecef(BD3_current_loc);
		north.x = -cos(BD3_current_loc.lon)*sin(BD3_current_loc.lat);
		north.y = -sin(BD3_current_loc.lon)*sin(BD3_current_loc.lat);
		north.z = cos(BD3_current_loc.lat);
		east.x = -sin(BD3_current_loc.lon);
		east.y = cos(BD3_current_loc.lon);
		up.x = cos(BD3_current_loc.lon)*cos(BD3_current_loc.lat);
		up.y = sin(BD3_current_loc.lon)*cos(BD3_current_loc.lat);
		up.z = sin(BD3_current_loc.lat);

		// DETERMINE IF A CLEAR LINE OF SIGHT EXISTS 
		xls = gpspos1.x - BD3_rec_pos_xyz.x;
		yls = gpspos1.y - BD3_rec_pos_xyz.y;
		zls = gpspos1.z - BD3_rec_pos_xyz.z;
		range1=sqrt(xls*xls+yls*yls+zls*zls);
		rangeInv = 1.0/range1;
		xls *= rangeInv;		
		yls *= rangeInv;
		zls *= rangeInv;

		tdot=up.x*xls+up.y*yls+up.z*zls;
		
		range2 = sqrt(pow(gpspos2.x - BD3_rec_pos_xyz.x - BD3_rpvt.xv, 2) +
			pow(gpspos2.y - BD3_rec_pos_xyz.y - BD3_rpvt.yv, 2) +
			pow(gpspos2.z - BD3_rec_pos_xyz.z - BD3_rpvt.zv, 2));
		// to avoid numerical error
		if ( tdot >= 1.00 ) 
			satang = BD3_pi / 2.0;
		else if ( tdot <= -1.00 ) 
			satang = -BD3_pi / 2.0;
		else  
			satang=asin(tdot);

		// 计算卫星位置相对与接收机位置的单位矢量从WGS84投影到本地坐标系时的东向和北向分量
		xaz=east.x*xls+east.y*yls;
		yaz=north.x*xls+north.y*yls+north.z*zls;
		if (xaz !=0.0 || yaz !=0.0) 
			az=atan2(xaz,yaz);	// 计算卫星的方位角(和北的夹角)
		else 
			az=0.0;  // tp avoid being divided by Zero

		result.x=gpspos1.x;
		result.y=gpspos1.y;
		result.z=gpspos1.z;
		result.elevation=satang;
		result.azimuth  =az;
		result.doppler  =(range1-range2)*5.2073;	// 5.2514 is the inverse of L1 wavelength	// doppler的单位为：米/秒  波长lamda = c/f
	}//end if (gps_alm[i].inc > 0.0 && i>0)
	return(result);
}


/*******************************************************************************
FUNCTION satpos_almanac(float time, char n)
RETURNS  None.

PARAMETERS
			time   float  time of week是TOW吗？是t，是计算轨道的历元时刻，是GPS信号发射的系统时
			n      char   satellite prn

PURPOSE

	  THIS SUBROUTINE CALCULATES THE SATELLITE POSITION
	  BASED ON ALMANAC DATA
	  这个子程序用历书数据计算卫星位置
******************************************************************************/

BD3_XYZ BD3_satpos_almanac(double time, char n)
{
	double ei,ea,diff,r,ta,la,aol,xp,yp,d_toa;
	BD3_XYZ result;
	
	d_toa = time - BD3_gps_alm[n].toa;
	if (d_toa>302400.0) d_toa=d_toa-604800.0;
	else if (d_toa<-302400.0)d_toa=d_toa+604800.0;
			
				
	ei = BD3_gps_alm[n].ma + d_toa*BD3_gps_alm[n].w;

    ea=ei;
	do
	{
		diff = (ei - (ea - BD3_gps_alm[n].ety*sin(ea))) / (1. - BD3_gps_alm[n].ety*cos(ea));
	   
       ea += diff;
	} while (fabs(diff) > 1.0e-6);
      
	if (BD3_gps_alm[n].ety != 0.0)
		ta = atan2(sqrt(1. - pow(BD3_gps_alm[n].ety, 2))*sin(ea), cos(ea) - BD3_gps_alm[n].ety);
	else ta=ea;
      
	//  R IS THE CORRECTED RADIUS OF SATELLITE ORBIT AT TIME T

	r = pow(BD3_gps_alm[n].sqa, 2)*(1. - pow(BD3_gps_alm[n].ety, 2)*cos(ea));

    //  AOL IS THE ARGUMENT OF LATITUDE
	aol = ta + BD3_gps_alm[n].aop;

	//LA IS THE LONGITUDE OF THE ASCENDING NODE
	la = BD3_gps_alm[n].lan + (BD3_gps_alm[n].rra - BD3_omegae)*d_toa - BD3_gps_alm[n].toa*BD3_omegae;

    xp=r*cos(aol);
    yp=r*sin(aol);
	result.x = xp*cos(la) - yp*cos(BD3_gps_alm[n].inc)*sin(la);
	result.y = xp*sin(la) + yp*cos(BD3_gps_alm[n].inc)*cos(la);
	result.z = yp*sin(BD3_gps_alm[n].inc);
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
// t is the 
BD3_ECEFT BD3_satpos_ephemeris(double t, char n)//track_sat[i]=satpos_ephemeris(tr_time[i],chan[tr_ch[i]].prn);
{
	double	ei,ea,diff,ta,aol,delr,delal,delinc,r,inc;
	double	la,xp,yp,bclk,tc,d_toc,d_toe;
	double	xls,yls,zls,range1,tdot,satang,xaz,yaz;
	double	az;
	BD3_XYZ		north, east, up;
	BD3_ECEFT	result;
    double  temp_x=0.0,temp_y=0.0,temp_z=0.0;
	double  five=0.0,phi=0.0;

	d_toc = t - BD3_gps_eph[n].toc;
    if (d_toc>302400.0) d_toc=d_toc-604800.0;
    else if (d_toc<-302400.0)d_toc=d_toc+604800.0;
	/***********************ICD103页*************************************************/
	/* The value of t must account for beginning or end of week
	crossovers. That is, if the quantity t - toc is greater than 302,400 seconds,
	subtract 604,800 seconds from t. If thequantity t - toc is less than -302,
	400 seconds, add 604,800 seconds to t.                                  */
	/************************************************************************/
    
	//总的时间修正项deltat，在此为bclk
	bclk = BD3_gps_eph[n].af0 + BD3_gps_eph[n].af1*d_toc + BD3_gps_eph[n].af2*d_toc*d_toc;
	//	-gps_eph[n].tgd;  // invalid  when debug bd
	tc=t-bclk;
    
	d_toe = tc - BD3_gps_eph[n].toe;
    if (d_toe>302400.0) d_toe=d_toe-604800.0;
	else if (d_toe<-302400.0)d_toe=d_toe+604800.0;
	
	// mean anomaly
	ei = BD3_gps_eph[n].ma + d_toe*(BD3_gps_eph[n].wm + BD3_gps_eph[n].dn);
	// iterate eccentric anomaly
    ea=ei;
    do
    {
		diff = (ei - (ea - BD3_gps_eph[n].ety*sin(ea))) / (1.0E0 - BD3_gps_eph[n].ety*cos(ea));
		ea += diff;
	} while (fabs(diff) > 1.0e-12 );
      
	// relativistic effects corrections: Pakinson, p684. vol 1
	//bclk -= 4.442807633E-10*gps_eph[n].ety*gps_eph[n].sqra*sin(ea);  //debug data1 ,no relativistic effects
	result.tb=bclk;

	// true anomaly
	ta = atan2(sqrt(1.00 - pow(BD3_gps_eph[n].ety, 2))*sin(ea), cos(ea) - BD3_gps_eph[n].ety);
	// AOL IS THE ARGUMENT OF LATITUDE OF THE SATELLITE
	aol = ta + BD3_gps_eph[n].w;

	// calculate the second harmonic perturbations of the orbit
	delr = BD3_gps_eph[n].crc*cos(2.0*aol) + BD3_gps_eph[n].crs*sin(2.0*aol);
	delal = BD3_gps_eph[n].cuc*cos(2.0*aol) + BD3_gps_eph[n].cus*sin(2.0*aol);
	delinc = BD3_gps_eph[n].cic*cos(2.0*aol) + BD3_gps_eph[n].cis*sin(2.0*aol);


	r = pow(BD3_gps_eph[n].sqra, 2)*(1.00 - BD3_gps_eph[n].ety*cos(ea)) + delr;
	aol=aol+delal;
	inc = BD3_gps_eph[n].inc0 + delinc + BD3_gps_eph[n].idot*d_toe;

	// LA IS THE CORRECTED LONGITUDE OF THE ASCENDING NODE
	/////////////////// 修改2010.11.17 ///////////////////////////////////////////////////////
/*    la=gps_eph[n].w0+(gps_eph[n].omegadot-omegae)*d_toe-
		omegae*gps_eph[n].toe;
*/	
    xp=r*cos(aol);
    yp=r*sin(aol);

	five = -5.0*BD3_DtoR;
	phi = BD3_omegae*d_toe;

	if (n>=6)
	{
		la = BD3_gps_eph[n].w0 + (BD3_gps_eph[n].omegadot - BD3_omegae)*d_toe - BD3_omegae*BD3_gps_eph[n].toe;
		result.x=xp*cos(la)-yp*cos(inc)*sin(la);
		result.y=xp*sin(la)+yp*cos(inc)*cos(la);
        result.z=yp*sin(inc);
	} 
	else if (n>=1 && n<=5)
	{
		la = BD3_gps_eph[n].w0 + (BD3_gps_eph[n].omegadot)*d_toe - BD3_omegae*BD3_gps_eph[n].toe;
		temp_x=xp*cos(la)-yp*cos(inc)*sin(la);
		temp_y=xp*sin(la)+yp*cos(inc)*cos(la);
        temp_z=yp*sin(inc);
		
        
        result.x=temp_x*cos(phi) + (temp_y*cos(five)+temp_z*sin(five)) * sin(phi);
		result.y=-temp_x*sin(phi)+ (temp_y*cos(five)+temp_z*sin(five)) * cos(phi);
        result.z=-temp_y*sin(five)+temp_z*cos(five);
	}
//////////////////////////////////////////////////////////////////////////

    result.az=0.0;
	result.el=0.0;
	if (BD3_rec_pos_xyz.x != 0.0 || BD3_rec_pos_xyz.y != 0.0 || BD3_rec_pos_xyz.z != 0.0)
	{
		// 计算将WGS-84的坐标转换为本地坐标系的旋转矩阵
		north.x = -cos(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
		north.y = -sin(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
		north.z = cos(BD3_rec_pos_llh.lat);
		east.x = -sin(BD3_rec_pos_llh.lon);
		east.y = cos(BD3_rec_pos_llh.lon);
		east.z=0.0;
		up.x = cos(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
		up.y = sin(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
		up.z = sin(BD3_rec_pos_llh.lat);
		
		// 计算卫星位置相对与接收机位置的单位矢量
		xls = result.x - BD3_rec_pos_xyz.x;
		yls = result.y - BD3_rec_pos_xyz.y;
		zls = result.z - BD3_rec_pos_xyz.z;
		range1=sqrt(xls*xls+yls*yls+zls*zls);
		tdot=(up.x*xls+up.y*yls+up.z*zls)/range1;
   
		// 计算卫星仰角
		if (tdot >= 1.00) satang = BD3_pi / 2.0;
		else if (tdot <= -1.00) satang = -BD3_pi / 2.0;
		else satang=asin(tdot);

		// 计算卫星方位角
		xaz=east.x*xls+east.y*yls;
		yaz=north.x*xls+north.y*yls+north.z*zls;
		if (xaz !=0.0 || yaz !=0.0) az=atan2(xaz,yaz);
		else az=0.0;
		result.el=satang;
		result.az=az;
   }
   return(result);
}

/*double  ComputeSatelliteInfo(const double* User, int sv, const double t,
						   double* SvPos, double* SvVel, double* clkoffset) */						   
BD3_ECEFT BD3_Sat_Cal_accord_orign(double t, int sat, BD3_EPHEMERIS ephsv[])
{
	int		j,k;			// counters
	double 	tr_time;		// transmit time
	double	delta_t;		// difference between computation and reference times
	double	bao;				// semi-major axis of orbit
	double	n0;			// computed mean motion
	double 	n;				// corrected mean motion
	double 	Mk;			// mean anomaly
	double	Ek;			// eccentric anomaly
	double	vk;			// true anomaly
	double	phik;			// argument of latitude
	double	roote;		// dummy variable
	double 	deluk;		// argument of latitude correction
	double	delrk;		// radius correction
	double	delik;		// inclination correction
	double	uk;			// corrected argument of latitude
	double	rk;			// corrected radius
	double	ik;			// corrected inclination
	double	uk_dot;		// rate of corrected argument of latitude
	double	rk_dot;		// rate of radius
	double	ik_dot;		// rate of inclination
	double 	phik_dot;	// rate of aargument of latitude
	double	xkp;			// in-plane x-coordinate
	double	ykp;			// in-plane y-coordinate
	double 	xkp_dot;		// in-plane x-velocity
	double	ykp_dot;		// in-plane y-velocity
	double	a1,a2;		// dummy variables
	double	omegak;		// corrected longitude of ascending node
	double  range;
	double  range_rate;
	double  dx,dy,dz;
	double  relative;

	double  m_sx1,m_sy1,m_sz1;	//计算的发射时刻的卫星坐标
	double  m_sx2,m_sy2,m_sz2;
	double  m_svx1,m_svy1,m_svz1;
	double  cos_5,sin_5;
	double  cos_OMEGAEDeltat,sin_OMEGAEDeltat;
    double  coef1,coef2,coef11;

	double  mu=3.986004418e14;		// WGS84 gravitational constant;
	double  f=-4.442809305e-10;

	int sv;
	int d_toc=0;
	double bclk=0.0;
	BD3_ECEFT	result;
	BD3_XYZ		north, east, up;
	double	xls,yls,zls,range1,tdot,satang,xaz,yaz;
	double	az;

	// assign time of computation, satellite position at transmission time
	//The input t is measurement epoch, i.e. receiving time
	//tr_time = t - range;

		// compute time difference from computation time
    sv=sat;  // sat prn

	d_toc=t-ephsv[sv].toc;
    if (d_toc>302400.0) d_toc=d_toc-604800.0;
    else if (d_toc<-302400.0)d_toc=d_toc+604800.0;

	bclk=ephsv[sv].af0+ephsv[sv].af1*d_toc+ephsv[sv].af2*d_toc*d_toc;
	result.tb=bclk;

	tr_time=t-bclk;
	
		delta_t = tr_time - ephsv[sv].toe;

		// correct for end of week crossover
		if(delta_t>302400.0)  delta_t-=604800.0;
		if(delta_t<-302400.0) delta_t+=604800.0;

		// semi-major axis of orbit
		bao = ephsv[sv].sqra*ephsv[sv].sqra;
		// computed mean motion
		n0 = sqrt(mu/(bao*bao*bao));
		// corrected mean motion
		n = n0 + ephsv[sv].dn;

		// mean anomaly
		Mk = ephsv[sv].ma + n*delta_t;
		//Mk = fmod(Mk,PI+PI);

		// iterate to get eccentric anomaly
		Ek=Mk;
		for(k=0;k<10;k++)	// 5 iterations
			Ek = Mk + ephsv[sv].ety*sin(Ek);
		relative = f*ephsv[sv].ety*ephsv[sv].sqra*sin(Ek);

		// compute the true anomaly
		roote = sqrt(1.0 -ephsv[sv].ety*ephsv[sv].ety);
		vk = atan2(roote*sin(Ek) , cos(Ek)-ephsv[sv].ety);

		// recompute the eccentric anomaly
		//Ek = acos((ephsv[sv].ec+cos(vk))/(1.0+ephsv[sv].ec*cos(vk)));

		// compute argument of latitude
		phik = vk + ephsv[sv].w;


		//************************************************************************
		//**            Compute second order harmonic partubations              **
		//************************************************************************

		// argument of latitude correction
		deluk=ephsv[sv].cuc*cos(2.0*phik) + ephsv[sv].cus*sin(2.0*phik);

		// radius correction
		delrk=ephsv[sv].crc*cos(2.0*phik) + ephsv[sv].crs*sin(2.0*phik);

		// correction to inclination
		delik=ephsv[sv].cic*cos(2.0*phik) + ephsv[sv].cis*sin(2.0*phik);


		//************************************************************************
		//**        Correct argument of latitude, radius and inclination        **
		//************************************************************************

		// corrected argument of latitude
		uk = phik + deluk;

		// corrected radius
		rk = bao*(1.0-ephsv[sv].ety*cos(Ek)) + delrk;

		// corrected inclination
		ik = ephsv[sv].inc0 + ephsv[sv].idot*delta_t + delik;


		//************************************************************************
		//**   Compute time derivatives of argument of latitude, inclination    **
		//**                             and radius                             **
		//************************************************************************

		// time derivative of argument of latitude
/*		phik_dot = n*pow(1.0+ephsv[sv].ety*cos(vk),2.0) / pow(1.0-ephsv[sv].ety*ephsv[sv].ety,1.5);

		// time derivative of corrected argument of latitude
		uk_dot = phik_dot + 2.0*phik_dot*(ephsv[sv].cus*cos(2.0*phik)-ephsv[sv].cuc*sin(2.0*phik));

		// time derivative of radius
		rk_dot = a*n*ephsv[sv].ety*sin(vk)/sqrt(1.0-ephsv[sv].ety*ephsv[sv].ety)
						+ 2.0*phik_dot*(ephsv[sv].crs*cos(2.0*phik)-epchsv[sv].crc*sin(2.0*phik));

		// time derivative of inclination
		ik_dot = ephsv[sv].idot + 2.0*phik_dot*(ephsv[sv].cis*cos(2.0*phik)-ephsv[sv].cic*sin(2.0*phik));*/


		//************************************************************************
		//**           Compute positions and velocities in the plane            **
		//************************************************************************

		// in-plane positions
		xkp = rk*cos(uk);
		ykp = rk*sin(uk);

		// in-plane velocities
		//xkp_dot = rk_dot*cos(uk) - ykp*uk_dot;
		//ykp_dot = rk_dot*sin(uk) + xkp*uk_dot;


		//************************************************************************
		//**   Correct longitude of ascending node and compute positions and    **
		//**                             velocities                             **
		//************************************************************************

		// correct longitude of ascending node
	//	omegak = ephsv[sv].W0 + (ephsv[sv].omegdot-OMEGA_EARTH)*delta_t - OMEGA_EARTH*(ephsv[sv].Toe + range);
		if((sv>=1) && (sv<=5))
		{
			omegak = ephsv[sv].w0 + ephsv[sv].omegadot*delta_t - BD3_omegae*ephsv[sv].toe;
			
		    m_sx1 = xkp*cos(omegak) - ykp*cos(ik)*sin(omegak);
			m_sy1 = xkp*sin(omegak) + ykp*cos(ik)*cos(omegak);
			m_sz1 = ykp*sin(ik);

			cos_5 = cos((-5.0)*BD3_pi / 180.0);
			sin_5 = sin((-5.0)*BD3_pi / 180.0);

			m_sx2 = m_sx1;
			m_sy2 = m_sy1*cos_5 + m_sz1*sin_5;
			m_sz2 = -m_sy1*sin_5 + m_sz1*cos_5;

			cos_OMEGAEDeltat = cos(BD3_omegae*delta_t);
			sin_OMEGAEDeltat = sin(BD3_omegae*delta_t);

			result.x = cos_OMEGAEDeltat*m_sx2 + sin_OMEGAEDeltat*m_sy2;
			result.y = -sin_OMEGAEDeltat*m_sx2 + cos_OMEGAEDeltat*m_sy2;
			result.z = m_sz2;
		}
		else if(sv>=6)
		{
			// correct longitude of ascending node
			omegak = ephsv[sv].w0 + (ephsv[sv].omegadot - BD3_omegae)*delta_t - BD3_omegae*ephsv[sv].toe;

			// satellite position
			result.x =xkp*cos(omegak) - ykp*cos(ik)*sin(omegak);
			result.y =xkp*sin(omegak) + ykp*cos(ik)*cos(omegak);
			result.z =ykp*sin(ik);	

		}
		
	result.az=0.0;
	result.el=0.0;
	if (BD3_rec_pos_xyz.x != 0.0 || BD3_rec_pos_xyz.y != 0.0 || BD3_rec_pos_xyz.z != 0.0)
	{
		// 计算将WGS-84的坐标转换为本地坐标系的旋转矩阵
		north.x = -cos(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
		north.y = -sin(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
		north.z = cos(BD3_rec_pos_llh.lat);
		east.x = -sin(BD3_rec_pos_llh.lon);
		east.y = cos(BD3_rec_pos_llh.lon);
		east.z=0.0;
		up.x = cos(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
		up.y = sin(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
		up.z = sin(BD3_rec_pos_llh.lat);
		
		// 计算卫星位置相对与接收机位置的单位矢量
		xls = result.x - BD3_rec_pos_xyz.x;
		yls = result.y - BD3_rec_pos_xyz.y;
		zls = result.z - BD3_rec_pos_xyz.z;
		range1=sqrt(xls*xls+yls*yls+zls*zls);
		tdot=(up.x*xls+up.y*yls+up.z*zls)/range1;
   
		// 计算卫星仰角
		if (tdot >= 1.00) satang = BD3_pi / 2.0;
		else if (tdot <= -1.00) satang = -BD3_pi / 2.0;
		else satang=asin(tdot);

		// 计算卫星方位角
		xaz=east.x*xls+east.y*yls;
		yaz=north.x*xls+north.y*yls+north.z*zls;
		if (xaz !=0.0 || yaz !=0.0) az=atan2(xaz,yaz);
		else az=0.0;
		result.el=satang;
		result.az=az;
   }
   return(result);
		
}


//ECEFT satpos_ephemeris(double t,char n)
//ECEFT SatProcess(double settime,int satnum,eph_t G_ephmsg[])
BD3_ECEFT BD3_SatProcess(double settime, int n, BD3_EPHEMERIS G_ephmsg[])
{
	int j,svid,sv;
	double dsqrta2,dn0,dtk,detoc;
	double dMk,dEk,dEktemp,detemp,dVk,dfaK;
	double ddeltauk,ddeltark,ddeltaik;
	double duk,drk,dik,dxk,dyk,dOMEGk;
	BD3_xyz_t tpos;
	double svcl,atime;
	double dfive,sinfive,cosfive;
	double phi,sinphi,cosphi;
	
	double sin_Ek,cos_Ek,e_2,s2phi,c2phi;
	double e_cosEk,sin_duk,cos_duk,sin_domegk,cos_domegk;
	double sin_dik,cos_dik;

	BD3_ECEFT	result;
	BD3_XYZ		north, east, up;
	double	xls,yls,zls,range1,tdot,satang,xaz,yaz;
	double	az;


	detoc  = 0.0;
	tpos.x = 0.0;
	tpos.y = 0.0;
	tpos.z = 0.0;

	//svid = obs->satID;
	svid=n+1;  //下标问题，特添加此变量
	sv=n;
	svcl = 0.0;
	
	atime = settime;

	//卫星钟差改正
    detoc = settime - G_ephmsg[svid-1].toc;
	
		if (detoc > 302400.0)
		{
			detoc-=604800.0;
		}
		else if (detoc < -302400.0)
		{
			detoc +=604800.0;
		}
		
		svcl     = G_ephmsg[svid-1].af0 + G_ephmsg[svid-1].af1 * detoc + G_ephmsg[svid-1].af2 * detoc * detoc;// - G_ephmsg[svid-1].dTGD1;//YUYUAN
		atime    = settime;
		settime -= svcl;
		detoc    = settime - G_ephmsg[svid-1].toc;
		svcl     = G_ephmsg[svid-1].af0 + G_ephmsg[svid-1].af1 * detoc + G_ephmsg[svid-1].af2 * detoc * detoc;//- G_ephmsg[svid-1].dTGD1;//YUYUAN
    
	result.tb=svcl;

	settime = atime - svcl;
	  
	dsqrta2  = G_ephmsg[svid-1].sqra * G_ephmsg[svid-1].sqra;  //存星历时即已做此工作
	//dn0 = _SQRT_GM / (dsqrta2 * G_ephmsg[svid-1].dsqrtA); 
	
    dn0=G_ephmsg[svid-1].wm;
	
	//-------------2007.02.08--------------------------------
	dtk = settime - G_ephmsg[svid-1].toe;
	
	if (dtk > 302400)
	{
		dtk-=604800.0;
	}
	else if (dtk < -302400.0)
	{
		dtk +=604800.0;
	}
	//计算平近点角
	dMk    = G_ephmsg[svid-1].ma + (dn0 + G_ephmsg[svid-1].dn) * dtk;
	//计算偏近点角
	detemp  = G_ephmsg[svid-1].ety;		
	dEktemp = 0.0; 
	dEk     = 0.0;		
	dEk     = dMk;
	for (j=0;j<15;j++)
	{					
		dEktemp = dEk;			
		dEk     = dMk + detemp * sin(dEktemp);						
		if(fabs(dEktemp - dEk) < 1.0e-13) break;
	}

	sin_Ek = 1.0 * sin(dEk);
	cos_Ek = 1.0 * cos(dEk);
	
	e_2 = detemp*detemp;
	
	//计算真近点角
	dVk = BD3_atan_2((sqrt(1 - e_2) * sin_Ek), (cos_Ek - detemp));
	
	//考虑相对论效应(源没有做此效应，暂时取消)
	//G_satellite_pos[satnum].t = G_satellite_pos[satnum].t +    //秒
		//                        _F * detemp * G_ephmsg[svid-1].dsqrtA * sin_Ek;

	//纬度幅角参数
	dfaK = dVk + G_ephmsg[svid-1].w;

	//周期改正项
	s2phi = sin(dfaK+dfaK);
	c2phi = cos(dfaK+dfaK);		   	
	ddeltauk = G_ephmsg[svid-1].cus * s2phi + G_ephmsg[svid-1].cuc * c2phi;
    ddeltark = G_ephmsg[svid-1].crs * s2phi + G_ephmsg[svid-1].crc * c2phi;
    ddeltaik = G_ephmsg[svid-1].cis * s2phi + G_ephmsg[svid-1].cic * c2phi;

	e_cosEk = 1 - detemp * cos_Ek;
	duk = dfaK + ddeltauk;
	drk = dsqrta2 * e_cosEk + ddeltark;
	dik = G_ephmsg[svid-1].inc0 + ddeltaik + G_ephmsg[svid-1].idot * dtk;

	sin_duk = sin(duk);
	cos_duk = cos(duk);
	dxk = drk * cos_duk;
	dyk = drk * sin_duk;

	sin_dik = sin(dik);
	cos_dik = cos(dik);

	dfive = -5.0*BD3_DtoR;
	phi = BD3_omegae*dtk;
	sinfive = sin(dfive);
	cosfive = cos(dfive);
	sinphi  = sin(phi);
	cosphi  = cos(phi);
	
	if (sv > 5 )//*
	{
		dOMEGk = G_ephmsg[svid - 1].w0 + (G_ephmsg[svid - 1].omegadot - BD3_omegae) * dtk
			- BD3_omegae * G_ephmsg[svid - 1].toe;

		sin_domegk = sin(dOMEGk);
		cos_domegk = cos(dOMEGk);

		result.x = dxk * cos_domegk - dyk * cos_dik * sin_domegk;
		result.y = dxk * sin_domegk + dyk * cos_dik * cos_domegk;
		result.z = dyk * sin_dik;
    }
	else if(sv>1.e-6 && sv<6)
	{
		dOMEGk = G_ephmsg[svid - 1].w0 + G_ephmsg[svid - 1].omegadot * dtk - BD3_omegae * G_ephmsg[svid - 1].toe;
		sin_domegk = sin(dOMEGk);
		cos_domegk = cos(dOMEGk);

		tpos.x = dxk * cos_domegk - dyk * cos_dik * sin_domegk;
		tpos.y = dxk * sin_domegk + dyk * cos_dik * cos_domegk;
		tpos.z = dyk * sin_dik;

		result.x = tpos.x *cosphi+sinphi*(tpos.y*cosfive+tpos.z*sinfive);
		result.y = -tpos.x*sinphi+cosphi*(tpos.y*cosfive+tpos.z*sinfive);
		result.z = -tpos.y*sinfive+tpos.z*cosfive;
	}

	//////////////////////////////////
	result.az=0.0;
	result.el=0.0;
	if (BD3_rec_pos_xyz.x != 0.0 || BD3_rec_pos_xyz.y != 0.0 || BD3_rec_pos_xyz.z != 0.0)
	{
		// 计算将WGS-84的坐标转换为本地坐标系的旋转矩阵
		north.x = -cos(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
		north.y = -sin(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
		north.z = cos(BD3_rec_pos_llh.lat);
		east.x = -sin(BD3_rec_pos_llh.lon);
		east.y = cos(BD3_rec_pos_llh.lon);
		east.z=0.0;
		up.x = cos(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
		up.y = sin(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
		up.z = sin(BD3_rec_pos_llh.lat);
		
		// 计算卫星位置相对与接收机位置的单位矢量
		xls = result.x - BD3_rec_pos_xyz.x;
		yls = result.y - BD3_rec_pos_xyz.y;
		zls = result.z - BD3_rec_pos_xyz.z;
		range1=sqrt(xls*xls+yls*yls+zls*zls);
		tdot=(up.x*xls+up.y*yls+up.z*zls)/range1;
   
		// 计算卫星仰角
		if (tdot >= 1.00) satang = BD3_pi / 2.0;
		else if (tdot <= -1.00) satang = -BD3_pi / 2.0;
		else satang=asin(tdot);

		// 计算卫星方位角
		xaz=east.x*xls+east.y*yls;
		yaz=north.x*xls+north.y*yls+north.z*zls;
		if (xaz !=0.0 || yaz !=0.0) az=atan2(xaz,yaz);
		else az=0.0;
		result.el=satang;
		result.az=az;
   }
   return(result);
}

double BD3_atan_2(double y, double x)
{
    double arctan = 0;
	if ((x == 0.0) && (y >= 0.0))    arctan = 0.5 * BD3_pi;
	else if ((x == 0.0) && (y < 0.0)) arctan = 1.5*BD3_pi;
    else
    {
       arctan = atan( y / x );
	   if (x < 0.0)  arctan = arctan + BD3_pi;
	   else if ((x > 0.0) && (y < 0.0)) arctan = arctan + 2.0 * BD3_pi;
    }
    return arctan;
}
/*******************************************************************************
FUNCTION read_initial_data(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
		  To read in all of the receiver initialization files
		读入所有接收机初始化文件
*******************************************************************************/

void BD3_read_initial_data(void)
{
	int		id;
	BD3_SATVIS	dummy;

	for (id=1;id<=12;id++)
		BD3_gps_alm[id].inc = 0.0;

	  
	BD3_status = BD3_cold_start;
	BD3_read_ion_utc();
	//read_almanac();		
	BD3_thetime = time(NULL);
	dummy = BD3_satfind(0);
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
char BD3_curloc_file[4096];//add by jh ui
BD3_LLH BD3_receiver_loc(void)
{
	float latitude,longitude,height;
    char  text[10];
	BD3_LLH result;

	result.lat=0.0;      
	result.lon=0.0;
	result.hae=-6378137;

    // READ THE CURRENT LOCATION DATA FILE
	if ((BD3_in = fopen(BD3_curloc_file, "rt")) == NULL)
    {
		printf("Cannot open curloc.dat file.\n");
		BD3_status = BD3_cold_start;
    }
	else
    {
		
		fscanf(BD3_in, "%10s", text);//latitude  59.997679
		fscanf(BD3_in, "%f", &latitude);
		fscanf(BD3_in, "%10s", text);//longitude 60.006548
		fscanf(BD3_in, "%f", &longitude);
		fscanf(BD3_in, "%10s", text);//hae       62.963341
		fscanf(BD3_in, "%f", &height);
		result.lat=latitude/57.296;	
		result.lon=longitude/57.296;
		result.hae=height;
		fclose(BD3_in);
	}
	return(result);
}


void BD3_resolution(char ch)
{
	short	sfr,word;
	long   scale;
	int    i,j;

	BD3_chan[ch].pnumID = (BD3_chan[ch].message[(33 + BD3_chan[ch].offset) % 1500]) * 8 + (BD3_chan[ch].message[(35 + BD3_chan[ch].offset) % 1500]) * 4 + (BD3_chan[ch].message[(37 + BD3_chan[ch].offset) % 1500]) * 2 + (BD3_chan[ch].message[(39 + BD3_chan[ch].offset) % 1500]);
	
	if (BD3_chan[ch].reverseflag == 1)
		BD3_chan[ch].pnumID = (~BD3_chan[ch].pnumID) & 0x000f;
	
	if (BD3_chan[ch].pnumID == 1)
		BD3_chan[ch].start = 1;
	if (BD3_chan[ch].start == 1)
	{
		j=0;
			for (word=1;word<11;word++)
			{
				scale = 0;	
				for (i=0;i<30;i++)
				{
					scale <<= 1;
					if (BD3_chan[ch].message[(j + BD3_chan[ch].offset) % 1500] == 1)
					{
						scale++; 
					}
					j++;
				}

				BD3_sf[ch][BD3_chan[ch].pnumID][1][word] = scale;
				if (BD3_chan[ch].reverseflag)

					BD3_sf[ch][BD3_chan[ch].pnumID][1][word] = (~scale) & 0x3fffffffL;
			}

			if (BD3_chan[ch].pnumID == 10)
		{
				BD3_chan[ch].frame_ready = 1;
				BD3_chan[ch].start = 0;
		}
	}
	
}
void BD3_navmessd2(char prn, char ch)   //2010.11.16
									//void navmess(char prn,char ch) 
{
	unsigned long	isqra, ie;
	long			iaf0, iomegadot, iomega0;
	int			    itgd1, itgd2, iaf2;
	int 			iweek, iura, ihealth, iodc, iaf1;
	unsigned short	itoe, itoc;
	int 			iode, icrs, idn, icuc, icus, icic;
	int 			icis, icrc, idot;
	long            iAA1, iBB, iA0, iA1, iA2, iA3, iB0, iB1, iB2, iB3, ir0, ir1, ir2, ir3;

	long			im0, inc0, iw;;

	short			sfr, Pnum, word;
	double			r_sqra, r_inc0, r_ety;
	double			d_toe;

	double			doublesv;

	int wordd;

	int freamcheck[13][11][1] = { 0 };
	int result1, result2;

	wordd = 0;


	d_toe = BD3_clock_tow - BD3_gps_eph[prn].toe;
	if (d_toe>302400.0) d_toe = d_toe - 604800.0;
	else if (d_toe<-302400.0)d_toe = d_toe + 604800.0;

	for (Pnum = 1;Pnum<11;Pnum++)
	{
		if (BD3_bch_decode(BD3_sf[ch][Pnum][1][1] & 0x00007fff) == 0)
			break;
		for (word = 2;word<11;word++)
		{
			BD3_shift_data(&BD3_sf[ch][Pnum][1][word]);
			if ((BD3_bch_decode(BD3_sf[ch][Pnum][1][word] & 0x3fff8000) == 0)
				&& (BD3_bch_decode(BD3_sf[ch][Pnum][1][word] & 0x00007fff) == 0))
				break;
			else
			{
				wordd++;
				BD3_shift_data2(&BD3_sf[ch][Pnum][1][word]);
				if (wordd == 9)
				{
					freamcheck[ch][Pnum][1] = 1;
					wordd = 0;
				}
			}
		}
	}


	if (freamcheck[ch][1][1] && freamcheck[ch][2][1] && freamcheck[ch][3][1])
	{
		iodc = int((BD3_sf[ch][1][1][2] & 0x1f00L) >> 8);
		iode = int((BD3_sf[ch][4][1][4] & 0x1f000000L) >> 24);
		if ((iode != BD3_gps_eph[prn].iode) || (iodc != BD3_gps_eph[prn].iodc))
		{
			//  subframe 1
			//pnum1
			ihealth = int((BD3_sf[ch][1][1][2] & 0x2000L) >> 13);
			iweek = int((BD3_sf[ch][1][1][3] & 0x3ffe000L) >> 13);
			iura = int(BD3_sf[ch][1][1][3] >> 26);
			itoc = int(((BD3_sf[ch][1][1][3] & 0x1f00L) << 4) | (BD3_sf[ch][1][1][4] >> 18));
			itgd1 = int((BD3_sf[ch][1][1][4] & 0x3ff00L) >> 8);
			if (BD3_bit_test_l(itgd1, 10)) itgd1 = itgd1 | 0xfffffc00L;
			itgd2 = int(BD3_sf[ch][1][1][5] >> 20);
			if (BD3_bit_test_l(itgd2, 10)) itgd2 = itgd2 | 0xfffffc00L;
			iAA1 = (BD3_sf[ch][1][1][5] & 0xf0000L) >> 16;//???

													  //pnum2
			iA0 = ((BD3_sf[ch][2][1][2] & 0x3f00L) >> 6) | (BD3_sf[ch][2][1][3] >> 28);
			if (BD3_bit_test_l(iA0, 8))  iA0 = iA0 | 0xffffff00L;
			iA1 = (BD3_sf[ch][2][1][3] & 0xff00000L) >> 20;
			if (BD3_bit_test_l(iA1, 8))  iA1 = iA1 | 0xffffff00L;
			iA2 = (BD3_sf[ch][2][1][3] & 0xff000L) >> 12;
			if (BD3_bit_test_l(iA2, 8))  iA2 = iA2 | 0xffffff00L;
			iA3 = ((BD3_sf[ch][2][1][3] & 0xf00L) >> 4) | (BD3_sf[ch][2][1][4] >> 26);
			if (BD3_bit_test_l(iA3, 8))  iA3 = iA3 | 0xffffff00L;
			iB0 = (BD3_sf[ch][2][1][4] & 0x3fc0000L) >> 18;
			if (BD3_bit_test_l(iB0, 8))  iB0 = iB0 | 0xffffff00L;
			iB1 = (BD3_sf[ch][2][1][4] & 0x3fc00L) >> 10;
			if (BD3_bit_test_l(iB1, 8))  iB1 = iB1 | 0xffffff00L;
			iB2 = ((BD3_sf[ch][2][1][4] & 0x300L) >> 2) | (BD3_sf[ch][2][1][5] >> 24);
			if (BD3_bit_test_l(iB2, 8))  iB2 = iB2 | 0xffffff00L;
			iB3 = ((BD3_sf[ch][2][1][5] & 0xff0000L) >> 16);
			if (BD3_bit_test_l(iB3, 8))  iB3 = iB3 | 0xffffff00L;

			//pnum3
			ir0 = ((BD3_sf[ch][3][1][2] & 0x3f00L) >> 6) | (BD3_sf[ch][3][1][3] >> 28);
			ir1 = (BD3_sf[ch][3][1][3] & 0xff00000L) >> 20;
			ir2 = (BD3_sf[ch][3][1][3] & 0xff000L) >> 12;
			ir3 = ((BD3_sf[ch][3][1][3] & 0xf00L) >> 4) | (BD3_sf[ch][3][1][4] >> 26);
			iBB = (BD3_sf[ch][3][1][4] & 0x3c00000L) >> 22;
			iaf0 = ((BD3_sf[ch][3][1][4] & 0xfff00L) << 4) | (BD3_sf[ch][3][1][5] >> 18);//(( sf[ch][3][1][4] & 0x3fff00L ) << 2) | (sf[ch][3][1][5]>>20);
			if (BD3_bit_test_l(iaf0, 24)) iaf0 = iaf0 | 0xff000000L;
			iaf1 = int(((BD3_sf[ch][3][1][5] & 0x3c000L) << 2) | ((BD3_sf[ch][4][1][2] & 0x3f00L) << 4) | (BD3_sf[ch][4][1][3] >> 18));//((( sf[ch][3][1][5] & 0xf0000L ) << 2) | ((sf[ch][4][1][2] & 0x3f00L ) << 4) |( sf[ch][4][1][3] >> 18));
			if (BD3_bit_test_l(iaf1, 22)) iaf1 = iaf1 | 0xffc00000L;

			//pnum4
			iaf2 = int(((BD3_sf[ch][4][1][3] & 0x3ff00L) >> 7) | (BD3_sf[ch][4][1][4] >> 29));
			if (BD3_bit_test_l(iaf2, 11)) iaf2 = iaf2 | 0xfffff800L;
			idn = int((BD3_sf[ch][4][1][4] & 0x00ffff00L) >> 8);
			if (BD3_bit_test_l(idn, 16)) idn = idn | 0xffff0000L;
			icuc = int(((BD3_sf[ch][4][1][5] >> 16) << 4) | ((BD3_sf[ch][5][1][2] & 0x3c00L) >> 10));//int(sf[ch][4][1][5] >>12) | ((sf[ch][5][1][2] & 0x3c00L) >>10);
			if (BD3_bit_test_l(icuc, 18)) icuc = icuc | 0xfffc0000L;

			//pnum5
			im0 = ((BD3_sf[ch][5][1][2] & 0x300L) << 22) | (BD3_sf[ch][5][1][3] & 0x3fffff00L) | (BD3_sf[ch][5][1][4] >> 22);
			icus = int(((BD3_sf[ch][5][1][4] & 0x3fff00L) >> 4) | (BD3_sf[ch][5][1][5] >> 26));
			if (BD3_bit_test_l(icus, 18)) icus = icus | 0xfffc0000L;
			ie = ((BD3_sf[ch][5][1][5] & 0x3ff0000L) << 6) | ((BD3_sf[ch][6][1][2] & 0x3f00L) << 8) | (BD3_sf[ch][6][1][3] >> 14);

			//pnum6
			isqra = ((BD3_sf[ch][6][1][3] & 0x3f00L) << 18) | ((BD3_sf[ch][6][1][4] >> 8) << 4) | (BD3_sf[ch][6][1][5] >> 26);
			icic = int(((BD3_sf[ch][6][1][5] & 0x3ff0000L) >> 8) | (BD3_sf[ch][7][1][2] & 0x3f00L) >> 6) | (BD3_sf[ch][7][1][3] >> 28);
			if (BD3_bit_test_l(icic, 18)) icic = icic | 0xfffc0000L;

			//pnum7
			icis = int((BD3_sf[ch][7][1][3] & 0xffffc00L) >> 10);
			if (BD3_bit_test_l(icis, 18)) icis = icis | 0xFFFC0000L;
			itoe = int(((BD3_sf[ch][7][1][3] & 0x300L) << 7) | (BD3_sf[ch][7][1][4] >> 15));
			inc0 = ((BD3_sf[ch][7][1][4] & 0x7f00L) << 17) | ((BD3_sf[ch][7][1][5] >> 16) << 11) | ((BD3_sf[ch][8][1][2] & 0x3f00L) >> 3) | (BD3_sf[ch][8][1][3] >> 25);

			//pnum8
			icrc = int((BD3_sf[ch][8][1][3] & 0x1ffff00L) >> 7 | (BD3_sf[ch][8][1][4] >> 29));
			if (BD3_bit_test_l(icrc, 18)) icrc = icrc | 0xfffc0000L;
			icrs = int((BD3_sf[ch][8][1][4] & 0x1ffff800L) >> 11);
			if (BD3_bit_test_l(icrs, 18)) icrs = icrs | 0xfffc0000L;
			iomegadot = ((BD3_sf[ch][8][1][4] & 0x700L) << 13) | ((BD3_sf[ch][8][1][5] >> 14) << 5) | ((BD3_sf[ch][9][1][2] & 0x3e00L) >> 9);
			if (BD3_bit_test_l(iomegadot, 24)) iomegadot = iomegadot | 0xff000000L;

			//pnum9
			iomega0 = ((BD3_sf[ch][9][1][2] & 0x100L) << 23) | ((BD3_sf[ch][9][1][3] >> 8) << 9) | (BD3_sf[ch][9][1][4] >> 21);
			iw = ((BD3_sf[ch][9][1][4] & 0x1fff00L) << 11) | ((BD3_sf[ch][9][1][5] >> 16) << 5) | ((BD3_sf[ch][10][1][2] & 0x3e00L) >> 9);

			//pnum10
			idot = int(((BD3_sf[ch][10][1][2] & 0x100L) << 5) | (BD3_sf[ch][10][1][3] >> 17));
			if (BD3_bit_test_l(idot, 14)) idot = idot | 0xffffc000L;

			r_sqra = isqra*BD3_c_2m19;
			r_inc0 = inc0*BD3_c_2m31*BD3_pi;
			r_ety = ie*BD3_c_2m33;

			// Does this ephemeris make sense?
			/*			if ((r_inc0<1.05 && r_inc0>0.873) && (r_sqra>5100.0 && r_sqra<5200.0) &&
			(r_ety <.05 && r_ety>=0.0))
			{
			*/
			//2010.11.17
			BD3_gps_eph[prn].valid = 1;
			BD3_gps_eph[prn].iode = iode;
			BD3_gps_eph[prn].iodc = iodc;
			BD3_gps_eph[prn].week = iweek;
			BD3_gps_eph[prn].ura = iura;
			BD3_gps_eph[prn].health = ihealth;
			BD3_gps_eph[prn].tgd = itgd1*pow(10.0, -10.0);
			//gps_eph[prn].tgd2=itgd2*pow(10.0, -10.0);
			BD3_gps_eph[prn].toc = itoc*8.0;
			BD3_gps_eph[prn].af2 = iaf2*BD3_c_2m66;
			BD3_gps_eph[prn].af1 = iaf1*BD3_c_2m50;
			BD3_gps_eph[prn].af0 = iaf0*BD3_c_2m33;
			BD3_gps_eph[prn].crs = icrs*BD3_c_2m6;
			BD3_gps_eph[prn].dn = idn*BD3_c_2m43*BD3_pi;
			BD3_gps_eph[prn].ma = im0*BD3_c_2m31*BD3_pi;
			BD3_gps_eph[prn].cuc = icuc*BD3_c_2m31;
			BD3_gps_eph[prn].ety = r_ety;
			BD3_gps_eph[prn].cus = icus*BD3_c_2m31;
			BD3_gps_eph[prn].sqra = r_sqra;
			BD3_gps_eph[prn].wm = 19964981.84 / pow(r_sqra, 3);
			BD3_gps_eph[prn].toe = itoe*8.0;
			BD3_gps_eph[prn].cic = icic*BD3_c_2m31;
			BD3_gps_eph[prn].cis = icis*BD3_c_2m31;
			BD3_gps_eph[prn].inc0 = r_inc0;
			BD3_gps_eph[prn].w0 = iomega0*BD3_c_2m31*BD3_pi;
			BD3_gps_eph[prn].crc = icrc*BD3_c_2m6;
			BD3_gps_eph[prn].w = iw*BD3_c_2m31*BD3_pi;
			BD3_gps_eph[prn].omegadot = iomegadot*BD3_c_2m43*BD3_pi;
			BD3_gps_eph[prn].idot = idot*BD3_c_2m43*BD3_pi;

			doublesv = (double)prn;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = (double)iweek;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = 0.0;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].tgd;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			//doublesv=gps_eph[prn].tgd2;
			//fwrite(&doublesv, sizeof(double),1,fpeph);
			doublesv = 0;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].toc;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].af2;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].af1;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].af0;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = 0;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].crs;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].dn;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].ma;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].cuc;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].ety;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].cus;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].sqra;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].toe;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].cic;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].w0;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].cis;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].inc0;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].crc;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].w;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].omegadot;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			doublesv = BD3_gps_eph[prn].idot;
			fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
			fflush(BD3_fpeph);

			if (BD3_out_debug)  BD3_write_Debug_ephemeris(prn);
			//			} 
			//			else if (gps_eph[prn].valid==1 && d_toe>7200.0) gps_eph[prn].valid=0;
		}
	}
	else printf("there is a mistake in parity check\n");
}
//void navmessd2(char prn,char ch)   //2010.11.16
////void navmess(char prn,char ch) 
//{
//	unsigned long	isqra,ie;
//	long			iaf0,iomegadot,iomega0;
//	int			    itgd1,itgd2,iaf2;
//	int 			iweek,iura,ihealth,iodc,iaf1;
//	unsigned short	itoe,itoc;
//	int 			iode,icrs,idn,icuc,icus,icic;
//	int 			icis,icrc,idot;
//	long            iAA1,iBB,iA0,iA1,iA2,iA3,iB0,iB1,iB2,iB3,ir0,ir1,ir2,ir3;
//
//    long			im0,inc0,iw;;
//
//	short			sfr,Pnum,word;
//	double			r_sqra,r_inc0,r_ety;
//	double			d_toe;
//
//	double			doublesv;
//
//    int wordd;          
//  
//	int freamcheck[13][11][1] = {0};
//	int result1,result2; 
//    
//	wordd=0;
//
//
//    d_toe=clock_tow-gps_eph[prn].toe;
//    if (d_toe>302400.0) d_toe=d_toe-604800.0;
//    else if (d_toe<-302400.0)d_toe=d_toe+604800.0;
//
//	for (Pnum=1;Pnum<11;Pnum++)
//	{
//		if (bch_decode(sf[ch][Pnum][1][1] & 0x00007fff) == 0)
//			break;
//		for (word=2;word<11;word++)
//		{
//			shift_data(&sf[ch][Pnum][1][word]);
//			if( (bch_decode(sf[ch][Pnum][1][word] & 0x3fff8000) == 0) 
//				&& (bch_decode(sf[ch][Pnum][1][word] & 0x00007fff) == 0) )
//					break; 
//			else 
//			{ 
//				wordd++;
//				shift_data2(&sf[ch][Pnum][1][word]);
//				if(wordd==9)
//				{
//					freamcheck[ch][Pnum][1]=1;
//					wordd=0;
//				}
//			}
//		}
//	}
//
//
//	if(freamcheck[ch][1][1] && freamcheck[ch][2][1] && freamcheck[ch][3][1])
//	{
//		iodc=int((sf[ch][1][1][2] & 0x1f00L) >>8 ) ;
//		iode=int((sf[ch][4][1][4] & 0x1f000000L) >> 24);
//		if ((iode!=gps_eph[prn].iode) || (iodc!=gps_eph[prn].iodc))
//		{
//            //  subframe 1
//			   //pnum1
//			ihealth=int(( sf[ch][1][1][2] & 0x2000L ) >> 13);
//			iweek= int((sf[ch][1][1][3] & 0x3ffe000L ) >> 13);
//			iura=int( sf[ch][1][1][3] >> 26);
//			itoc=int(((sf[ch][1][1][3] & 0x1f00L) << 4 ) | ( sf[ch][1][1][4]>>18 ));
//			itgd1=int((sf[ch][1][1][4] & 0x3ff00L) >> 8 );
//			if (bit_test_l(itgd1,10)) itgd1=itgd1 | 0xfffffc00L;
//			itgd2=int(sf[ch][1][1][5] >> 20 );
//			if (bit_test_l(itgd2,10)) itgd2=itgd2 | 0xfffffc00L;
//            iAA1= ( sf[ch][1][1][5] &0xf0000L ) >> 16;
//
//			   //pnum2
//            iA0=(( sf[ch][2][1][2] & 0x3f00L) >> 6) | (sf[ch][2][1][3]>>28) ;
//			if (bit_test_l(iA0,8))  iA0=iA0 | 0xffffff00L;
//            iA1=( sf[ch][2][1][3] & 0xff00000L ) >> 20;
//			if (bit_test_l(iA1,8))  iA1=iA1 | 0xffffff00L;
//			iA2=( sf[ch][2][1][3] & 0xff000L ) >> 12;
//			if (bit_test_l(iA2,8))  iA2=iA2 | 0xffffff00L;
//			iA3=(( sf[ch][2][1][3] & 0xf00L ) >> 4 ) | (sf[ch][2][1][4]>>26);
//			if (bit_test_l(iA3,8))  iA3=iA3 | 0xffffff00L;
//			iB0=( sf[ch][2][1][4] & 0x3fc0000L ) >> 18 ;
//			if (bit_test_l(iB0,8))  iB0=iB0 | 0xffffff00L;
//            iB1=( sf[ch][2][1][4] & 0x3fc00L ) >> 10;
//			if (bit_test_l(iB1,8))  iB1=iB1 | 0xffffff00L;
//			iB2=(( sf[ch][2][1][4] & 0x300L ) >> 2) | (sf[ch][2][1][5]>>24);
//			if (bit_test_l(iB2,8))  iB2=iB2 | 0xffffff00L;
//			iB3=(( sf[ch][2][1][5] & 0xff0000L ) >> 16 );
//			if (bit_test_l(iB3,8))  iB3=iB3 | 0xffffff00L;
//
//			  //pnum3
//            ir0=(( sf[ch][3][1][2] & 0x3f00L ) >> 6) | (sf[ch][3][1][3]>>28) ;
//            ir1=( sf[ch][3][1][3] & 0xff00000L ) >> 20;
//			ir2=( sf[ch][3][1][3] & 0xff000L ) >> 12;
//			ir3=(( sf[ch][3][1][3] & 0xf00L ) >> 4 ) | (sf[ch][3][1][4]>>26);
//            iBB=( sf[ch][3][1][4] & 0x3c00000L ) >> 22;
//			iaf0=(( sf[ch][3][1][4] & 0x3fff00L ) << 2) | (sf[ch][3][1][5]>>20);
//			if (bit_test_l(iaf0,24)) iaf0=iaf0 | 0xff000000L;
//			iaf1=int((( sf[ch][3][1][5] & 0xf0000L ) << 2) | ((sf[ch][4][1][2] & 0x3f00L ) << 4) |( sf[ch][4][1][3] >> 18));
//            if (bit_test_l(iaf1,22)) iaf1=iaf1 | 0xffc00000L;
//			
//			  //pnum4
//            iaf2=int((( sf[ch][4][1][3] & 0x3ff00L ) >> 7 ) | (sf[ch][4][1][4]>>29));
//			if (bit_test_l(iaf2,11)) iaf2=iaf2 | 0xfffff800L;
//            idn=int(( sf[ch][4][1][4] & 0x00ffff00L) >> 8);
//			if (bit_test_l(idn,16)) idn=idn | 0xffff0000L;
//           // icuc=int(sf[ch][4][1][5] >>12) | ((sf[ch][5][1][2] & 0x3c00L) >>10);
//			icuc = int(sf[ch][4][1][5]&0x3fff0000L >> 12) | ((sf[ch][5][1][2] & 0x3c00L) >> 10);//todo
//			if (bit_test_l(icuc,18)) icuc=icuc | 0xfffc0000L;
//
//			  //pnum5
//			im0=((sf[ch][5][1][2] & 0x300L) << 22) | (sf[ch][5][1][3] & 0x3fffff00L) | (sf[ch][5][1][4]>>22);
//           // icus=int(((sf[ch][5][1][4] &0x3fff00L) >> 4) | (sf[ch][5][1][5]>>26));
//			icus = int(((sf[ch][5][1][4] & 0x3fff00L) >> 4) | (sf[ch][5][1][5]&0x3c000000L >> 26));//todo
//			if (bit_test_l(icus,18)) icus=icus | 0xfffc0000L;
//            ie=((sf[ch][5][1][5] & 0x3ff0000L) << 6) | ((sf[ch][6][1][2]&0x3f00L)<<8)| (sf[ch][6][1][3] >> 14);
//			 
//			  //pnum6
//			isqra=((sf[ch][6][1][3] & 0x3f00L) << 18) | (sf[ch][6][1][4]>>4) | (sf[ch][6][1][5]>>26);
//            icic=int(((sf[ch][6][1][5] & 0x3ff0000L)>>8) | (sf[ch][7][1][2] & 0x3f00L)>>6) | (sf[ch][7][1][3]>>28);
//			if (bit_test_l(icic,18)) icic=icic | 0xfffc0000L;
//
//			  //pnum7
//			icis=int((sf[ch][7][1][3] & 0xffffc00L)>>10);
//			if (bit_test_l(icis,18)) icis=icis | 0xFFFC0000L;
//            itoe=int(((sf[ch][7][1][3] & 0x300L) << 7) | (sf[ch][7][1][4] >>15));
//            inc0=((sf[ch][7][1][4] & 0x7f00L ) << 17) | (sf[ch][7][1][5] >> 5) | ((sf[ch][8][1][2] & 0x3f00L) >> 3) | (sf[ch][8][1][3] >> 25 );
//
//			  //pnum8
//			icrc=int((sf[ch][8][1][3] & 0x1ffff00L ) >> 8);
//			if (bit_test_l(icrc,17)) icrc=icrc | 0xfffe0000L;
//			icrs=int(sf[ch][8][1][4] >>13);
//			if (bit_test_l(icrs,17)) icrs=icrs | 0xfffe0000L;
//            iomegadot=((sf[ch][8][1][4] & 0x1f00L ) << 11) | (sf[ch][8][1][5] >> 11) | ((sf[ch][9][1][2] & 0x3e00L) >> 9);
//            if (bit_test_l(iomegadot,24)) iomegadot=iomegadot | 0xff000000L;
//
//			  //pnum9
//            iomega0=((sf[ch][9][1][2] & 0x100) << 23) | (sf[ch][9][1][3] << 1) | (sf[ch][9][1][4]>> 21);
//            iw=((sf[ch][9][1][4] & 0x1fff00L) << 11) | (sf[ch][9][1][5] >> 11 ) | ((sf[ch][10][1][2] & 0x3e00L) >> 9);
//
//			  //pnum10
//			idot=int(((sf[ch][10][1][2] & 0x100L) << 5) | (sf[ch][10][1][3] >> 17));
//			if (bit_test_l(idot,14)) idot=idot | 0xffffc000L;
//
//			r_sqra=isqra*c_2m19;
//			r_inc0=inc0*c_2m31*pi;
//			r_ety=ie*c_2m33;
//			
//			// Does this ephemeris make sense?
///*			if ((r_inc0<1.05 && r_inc0>0.873) && (r_sqra>5100.0 && r_sqra<5200.0) &&
//				(r_ety <.05 && r_ety>=0.0))
//			{
//*/
////2010.11.17
//			    gps_eph[prn].valid=1;
//				gps_eph[prn].iode=iode;
//				gps_eph[prn].iodc=iodc;
//				gps_eph[prn].week=iweek;
//				gps_eph[prn].ura=iura;
//				gps_eph[prn].health=ihealth;
//				gps_eph[prn].tgd=itgd1*pow(10.0, -10.0);
//		    	//gps_eph[prn].tgd2=itgd2*pow(10.0, -10.0);
//				gps_eph[prn].toc=itoc*8.0;
//				gps_eph[prn].af2=iaf2*c_2m66;
//				gps_eph[prn].af1=iaf1*c_2m50;
//				gps_eph[prn].af0=iaf0*c_2m33;
//				gps_eph[prn].crs=icrs*c_2m6;
//				gps_eph[prn].dn=idn*c_2m43*pi;
//				gps_eph[prn].ma=im0*c_2m31*pi;
//				gps_eph[prn].cuc=icuc*c_2m31;
//				gps_eph[prn].ety=r_ety;
//				gps_eph[prn].cus=icus*c_2m31;
//				gps_eph[prn].sqra=r_sqra;
//				gps_eph[prn].wm=19964981.84/pow(r_sqra,3);
//				gps_eph[prn].toe=itoe*8.0;
//				gps_eph[prn].cic=icic*c_2m31;
//				gps_eph[prn].cis=icis*c_2m31;
//				gps_eph[prn].inc0=r_inc0;
//				gps_eph[prn].w0=iomega0*c_2m31*pi;
//				gps_eph[prn].crc=icrc*c_2m6;
//				gps_eph[prn].w=iw*c_2m31*pi;
//				gps_eph[prn].omegadot=iomegadot*c_2m43*pi;
//				gps_eph[prn].idot=idot*c_2m43*pi;
//				
//				doublesv=(double)prn;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=(double)iweek;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=0.0;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].tgd;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				//doublesv=gps_eph[prn].tgd2;
//				//fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=0;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].toc;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].af2;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].af1;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].af0;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=0;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].crs;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].dn;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].ma;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].cuc;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].ety;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].cus;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].sqra;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].toe;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].cic;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].w0;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].cis;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].inc0;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].crc;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].w;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].omegadot;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				doublesv=gps_eph[prn].idot;
//				fwrite(&doublesv, sizeof(double),1,fpeph);
//				fflush(fpeph);
//				
//				if (out_debug)  write_Debug_ephemeris(prn);
////			} 
////			else if (gps_eph[prn].valid==1 && d_toe>7200.0) gps_eph[prn].valid=0;
//		} 
//	} 
//    else printf("there is a mistake in parity check\n");
//}
/////////////////////////////添加2010.11.16/////////////////////////////////////////////
void BD3_navmessd1(char prn, char ch)
{
	int				i,j;
	unsigned long	isqra,ie;
	long			iaf0,iomegadot;
	int			    itgd,iaf2;
	int			    iweek,iura,ihealth,iodc,iaf1;
	unsigned int	itoe,itoc;
	int			    iode,icrs,idn,icuc,icus,icic;
	int			    icis,icrc,idot;
	long			im0,inc0,iw,iomega0;
	long			scale;
	short			sfr,word;
	double			r_sqra,r_inc0,r_ety;
	double			d_toe;
	double			doublesv;


    int wqword;          //xxnavmess 
	int wordcheck[6][11];   //xxnavmess
	int freamcheck[6];   //xxnavmess
	int result1,result2; //xxnavmess
	wqword=0;
	//xxnavmess
	for(sfr=1;sfr<6;sfr++)
		for(word=1;word<11;word++)
			wordcheck[sfr][word]=1;  //1--error 
	//xxnavmess
	for(sfr=1;sfr<6;sfr++)
		freamcheck[sfr]=0; //xxnavmess ----0---error

	// 检查当前时刻和星历时刻的时间差
	d_toe = BD3_clock_tow - BD3_gps_eph[prn].toe;
    if (d_toe>302400.0) d_toe=d_toe-604800.0;
    else if (d_toe<-302400.0)d_toe=d_toe+604800.0;
	
	j=0;

	// sf[][]是一个以帧序号和字序号为索引的二维数组
	// sf每个单元的为4字节的long型，有效位数为低30位

	for (sfr=1;sfr<=5;sfr++)
	{
		for (word=1;word<=10;word++)
		{
			scale=0;	
			for (i=0;i<=29;i++)
			{
				// chan结构中的message是一个循环地址索引的数组
				// message的每个字节实际只存放一个bit的导航电文
				// message[0]并不一定存放的就是第一帧
				// chan[ch].offset用于指示第一子帧、第一个字、第一个bit的位置
				scale=scale<<1;
				if (BD3_chan[ch].message[(j + BD3_chan[ch].offset) % 1500] == 1)
				{
					// set the corresponding bit to 1 by adding
					// or set to 0 without adding
					scale++; 
				}
				j++;
			}
			BD3_sfd1[sfr][word] = scale;
			if (BD3_chan[ch].reverseflag)
				BD3_sfd1[sfr][word] = (~scale) & 0x3fffffffL;
		}
	}

	//----xxnavmess--------
	for (sfr=1;sfr<=5;sfr++)
	{
		for (word=2;word<=10;word++)  
        {
			BD3_undointerlace(&BD3_sfd1[sfr][word]); //1~5帧，每帧2~10字去交织   //xxnavmess
		} //for-word-end
	} //for-sfr-end
//	parity_check(); // check the parity of the 1500 bit message

    //wq   xxcheck
	for (sfr=1;sfr<=5;sfr++)
	{
		wordcheck[sfr][1] = BD3_checkf(BD3_sfd1[sfr][1], 2);  //wq 第一个字校验  //xxnavmess
	}  
    for (sfr=1;sfr<=5;sfr++)
	{
		for (word=2;word<=10;word++) 
		{ 
			result1 = BD3_check(BD3_sfd1[sfr][word], 1);
			result2 = BD3_check(BD3_sfd1[sfr][word], 2);
			if(result1==0&&result2==0)
				wordcheck[sfr][word]=0; //wq 1--error  所以当result==0且result2==0时才意味着这个字的正确
			//wq 即将wordcheck[]设为0表示正确
		} //for-end
		for(word=1;word<11;word++)
			if(wordcheck[sfr][word]) break;   //xxnavmess   只有当wordcheck[word]都为0时才行
			//xxnavmess   如果一个字有问题则wordcheck[word]就会为1，则没有必要执行，
			//xxnavmess   该字的校验freamcheck[sfr]=0表示该子帧错误
			else 
			{ 
				wqword++;  
				if(wqword==10)
				{
					freamcheck[sfr]=1;  
					wqword=0;
				}
			}
			//wq 只有十个字都正确，才标志一个子帧正确
			//wq 通过查询freamcheck[]可以知道这五个子帧哪个是正确的
	}//for-end
/////////////////////xxnavmess------end///////////////////////////////
	if(freamcheck[1]&&freamcheck[2]&&freamcheck[3])  
	{
		iodc = int(((BD3_sfd1[1][2] & 0x0001f000L) >> 12));
		iode = int(((BD3_sfd1[1][10] & 0x00001f00L) >> 8));

		if ((iode != BD3_gps_eph[prn].iode) || (iodc != BD3_gps_eph[prn].iodc))
		{
			//   subframe 1
			iweek = int(BD3_sfd1[1][3] >> 17);
			ihealth = int((BD3_sfd1[1][2] & 0x00020000L) >> 17);
			iura = int((BD3_sfd1[1][2] & 0x0f00) >> 8);
			//itoc=int(((sfd1[1][3] & 0x0001ff00L)|(sfd1[1][4] >> 22)));
			itoc = int(((BD3_sfd1[1][3] & 0x0001ff00L) | ((BD3_sfd1[1][4] & 0x3fc00000) >> 22))); //todo:加了&0x3fc00000
			itgd = int((BD3_sfd1[1][4] & 0x003ff000L) >> 12);
			if (BD3_bit_test_l(itgd, 10)) itgd = itgd | 0xfffffc00L;
			iaf2 = int((BD3_sfd1[1][8] & 0x03ff8000L) >> 15);
			if (BD3_bit_test_l(iaf2, 11)) iaf2 = iaf2 | 0xfffff800L;
			iaf1 = int(((BD3_sfd1[1][9] & 0x1F00) << 9) | (BD3_sfd1[1][10] >> 13));
			if (BD3_bit_test_l(iaf1, 22)) iaf1 = iaf1 | 0xFFC00000L;
			iaf0 = int(((BD3_sfd1[1][8] & 0x7F00) << 9) | (BD3_sfd1[1][9] >> 13));
			if (BD3_bit_test_l(iaf0, 24)) iaf0 = iaf0 | 0xFF000000L;
			
			//   subframe 2
			idn = int(((BD3_sfd1[2][2] & 0x0003ff00L) >> 2) | (BD3_sfd1[2][3] >> 24));
			if (BD3_bit_test_l(idn, 16)) idn = idn | 0xFFFF0000L;
			icuc = int(((BD3_sfd1[2][3] & 0x00ffff00L) >> 6) | (BD3_sfd1[2][4] >> 28));
			if (BD3_bit_test_l(icuc, 18)) icuc = icuc | 0xFFFC0000L;
			im0 = (((BD3_sfd1[2][4] & 0x0fffff00L) << 4) | (BD3_sfd1[2][5] >> 18));
			ie = (((BD3_sfd1[2][5] & 0x0003ff00L) << 14) | (BD3_sfd1[2][6] >> 8));
			icus = int(BD3_sfd1[2][7] >> 12);
			if (BD3_bit_test_l(icus, 18)) icus = icus | 0xFFFC0000L;
			icrc = int(((BD3_sfd1[2][7] & 0x0f00) << 6) | ((BD3_sfd1[2][8] >> 16) & 0x3fff)); //(((sfd1[2][7] & 0x0f00) << 6)|(sfd1[2][8] >> 17));
			if (BD3_bit_test_l(icrc, 18)) icrc = icrc | 0xFFFC0000L;

			icrs = int(((BD3_sfd1[2][8] & 0x000ff00L) << 2) | (BD3_sfd1[2][9] >> 20));//((sfd1[2][8] & 0x0001ff00L)|(sfd1[2][9] >> 22));
			if (BD3_bit_test_l(icrs, 18)) icrs = icrs | 0xFFFC0000L;

			isqra = (((BD3_sfd1[2][9] & 0x000fff00L) << 12) | (BD3_sfd1[2][10] >> 10));//(((sfd1[2][9] & 0x003fff00L) << 10) | (sfd1[2][10] >> 12));
			itoe = int(((BD3_sfd1[2][10] & 0x00300) << 7) | ((BD3_sfd1[3][2] & 0x0003ff00L) >> 3) | (BD3_sfd1[3][3] >> 25));//(((sfd1[2][10] & 0x0f00) << 5)|((sfd1[3][2] & 0x0003ff00L) >> 5)|(sfd1[3][3] >> 27));
			
			// subframe 3
			inc0 = ((BD3_sfd1[3][3] & 0x01ffff00L) << 7) | (BD3_sfd1[3][4] >> 15);//((sfd1[3][3] & 0x07ffff00L) << 5) | (sfd1[3][4] >> 17);
			icic = int(((BD3_sfd1[3][4] & 0x0007f00L) << 3) | (BD3_sfd1[3][5] >> 19));//(((sfd1[3][4] & 0x0001ff00L) << 1)|(sfd1[3][5] >> 21));
			if (BD3_bit_test_l(icic, 18)) icic = icic | 0xFFFC0000L;
			iomegadot = int(((BD3_sfd1[3][5] & 0x007ff00L) << 5) | (BD3_sfd1[3][6] >> 17));//(((sfd1[3][5] & 0x001fff00L) << 3)|(sfd1[3][6] >> 17));
			if (BD3_bit_test_l(iomegadot, 24)) iomegadot = iomegadot | 0xFF000000L;
			icis = int(((BD3_sfd1[3][6] & 0x0001ff00L) << 1) | (BD3_sfd1[3][7] >> 21));//(((sfd1[3][6] & 0x0007ff00L) >> 1)|(sfd1[3][7] >> 23));
			if (BD3_bit_test_l(icis, 18)) icis = icis | 0xFFFC0000L;
			idot = int(((BD3_sfd1[3][7] & 0x001fff00L) >> 7) | (BD3_sfd1[3][8] >> 29));//((sfd1[3][7] & 0x007ffe00L) >> 9);
			if (BD3_bit_test_l(idot, 14)) idot = idot | 0xFFFFC000L;
			iomega0 = ((BD3_sfd1[3][8] & 0x001fffff00L) << 3) | ((BD3_sfd1[3][9] >> 19));//((sfd1[3][7] & 0x00000100L) << 23) | ((sfd1[3][8] >> 8) << 9) | (sfd1[3][9] >> 21);
			iw = ((BD3_sfd1[3][9] & 0x007ff00L) << 13) | (BD3_sfd1[3][10] >> 9);//((sfd1[3][9] & 0x001fff00L) << 11) | (sfd1[3][10] >> 11);
			
			r_sqra = isqra*BD3_c_2m19;
			r_inc0 = inc0*BD3_c_2m31*BD3_pi;
			r_ety = ie*BD3_c_2m33;
			
			// Does this ephemeris make sense?
/*			if ((r_inc0<1.05 && r_inc0>0.873) && (r_sqra>5100.0 && r_sqra<5200.0) &&
				(r_ety <.05 && r_ety>=0.0))
			{
*/	
//2010.11.17
			BD3_gps_eph[prn].valid = 1;
			BD3_gps_eph[prn].iode = iode;
			BD3_gps_eph[prn].iodc = iodc;
			BD3_gps_eph[prn].week = iweek;
			BD3_gps_eph[prn].ura = iura;
			BD3_gps_eph[prn].health = ihealth;
	//			gps_eph[prn].tgd=itgd*pow(2.0, -31.0);    //GPS ICD
			BD3_gps_eph[prn].tgd = itgd*pow(10.0, -10.0);    //GPS ICD 
			BD3_gps_eph[prn].toc = itoc*8.0;
			BD3_gps_eph[prn].af2 = iaf2*BD3_c_2m66;
			BD3_gps_eph[prn].af1 = iaf1*BD3_c_2m50;
			BD3_gps_eph[prn].af0 = iaf0*BD3_c_2m33;
			BD3_gps_eph[prn].crs = icrs*BD3_c_2m6;
			BD3_gps_eph[prn].dn = idn*BD3_c_2m43*BD3_pi;
			BD3_gps_eph[prn].ma = im0*BD3_c_2m31*BD3_pi;
			BD3_gps_eph[prn].cuc = icuc*BD3_c_2m31;
			BD3_gps_eph[prn].ety = r_ety;
			BD3_gps_eph[prn].cus = icus*BD3_c_2m31;
			BD3_gps_eph[prn].sqra = r_sqra;
			BD3_gps_eph[prn].wm = 19964981.84 / pow(r_sqra, 3);
			BD3_gps_eph[prn].toe = itoe*8.0;
			BD3_gps_eph[prn].cic = icic*BD3_c_2m31;
			BD3_gps_eph[prn].cis = icis*BD3_c_2m31;
			BD3_gps_eph[prn].inc0 = r_inc0;
			BD3_gps_eph[prn].w0 = iomega0*BD3_c_2m31*BD3_pi;
			BD3_gps_eph[prn].crc = icrc*BD3_c_2m6;
			BD3_gps_eph[prn].w = iw*BD3_c_2m31*BD3_pi;
			BD3_gps_eph[prn].omegadot = iomegadot*BD3_c_2m43*BD3_pi;
			BD3_gps_eph[prn].idot = idot*BD3_c_2m43*BD3_pi;
				
				doublesv=(double)prn;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv=(double)iweek;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv=0.0;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].tgd;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv=0;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].toc;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].af2;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].af1;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].af0;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv=0;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].crs;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].dn;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].ma;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].cuc;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].ety;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].cus;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].sqra;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].toe;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].cic;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].w0;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].cis;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].inc0;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].crc;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].w;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].omegadot;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				doublesv = BD3_gps_eph[prn].idot;
				fwrite(&doublesv, sizeof(double), 1, BD3_fpeph);
				fflush(BD3_fpeph);
				
				if (BD3_out_debug)  BD3_write_Debug_ephemeris(prn);
//			} // end of making sense checking
//			else if (gps_eph[prn].valid==1 && 0/*d_toe>7200.0*/) gps_eph[prn].valid=0;
		} // end of if iode, idoe agreement
	} // end of if, correct subframe 1, 2, 3
    else 
		printf("there is a mistake in parity check\n");
}
//////////////////////////////////////////////////////////////////////////

/*******************************************************************************
FUNCTION navmess()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function assembles and decodes the 1500 bit nav message
	into almanac and ephemeris messages
	解调导航电文

*******************************************************************************/

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

/*******************************************************************************
FUNCTION parity_check(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  checks the parity of the 5 subframes of the nav message
函数的输入为long型的全局数组sf[帧序号:1-5][字序号:1-10]
函数的输出为int型的全局数组p_error[帧序号:1-5]
p_error的低10bit表示某个子帧的10个字的校验是否正确，正确时为0，否则为1。
注意：在parity_check之前，sf字中包含有6bit的校验位，而之后则没有
		因此校验前，数据是低30位有效，而校验后是低24位有效
*******************************************************************************/

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
int BD3_exor(char bit, long parity)
{
	char i;
	int result;
	result=0;
	for (i=7;i<=30;i++)//取parity的前24位，最后6位不取
	{
		if (BD3_bit_test_l(parity, i)) result++;
		
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
NO iteration is used ???
*******************************************************************************/

BD3_LLH BD3_ecef_to_llh(BD3_XYZ pos)
{
	double p,n,thet,esq,epsq;
	BD3_LLH result;

	p=sqrt(pos.x*pos.x+pos.y*pos.y);
	thet = atan(pos.z*BD3_a / (p*BD3_b));
	esq = 1.0 - BD3_b*BD3_b / (BD3_a*BD3_a);
	epsq = BD3_a*BD3_a / (BD3_b*BD3_b) - 1.0;
	result.lat = atan((pos.z + epsq*BD3_b*pow(sin(thet), 3)) / (p - esq*BD3_a*pow(cos(thet), 3)));
	result.lon=atan2(pos.y,pos.x);
	n = BD3_a*BD3_a / sqrt(BD3_a*BD3_a*cos(result.lat)*cos(result.lat) +
		BD3_b*BD3_b*sin(result.lat)*sin(result.lat));
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

BD3_XYZ BD3_llh_to_ecef(BD3_LLH pos)
{
	double n;
	BD3_XYZ result;
	n = BD3_a*BD3_a / sqrt(BD3_a*BD3_a*cos(pos.lat)*cos(pos.lat) + BD3_b*BD3_b*sin(pos.lat)*sin(pos.lat));
	result.x=(n+pos.hae)*cos(pos.lat)*cos(pos.lon);
	result.y=(n+pos.hae)*cos(pos.lat)*sin(pos.lon);
	result.z = (BD3_b*BD3_b / (BD3_a*BD3_a)*n + pos.hae)*sin(pos.lat);
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
extern double		BD3_m_time[3];
extern 	FILE *BD3_daicy_file_pr;
extern 	FILE *BD3_daicy_file_pos;
int		BD3_daicy_temp_cnt;

BD3_PVT  BD3_pos_vel_time(int nsl)
{
	double	dd[5][5],r,ms[5][13],pm[5][13],bm[13],br[5],correct_mag,x,y,z,t;
	double	a1,b1,c1,d1,e1,f1,g1,h1,i1,j1,k1,l1,m1,n1,o1,p1,denom,alpha;
	int		i,j,k,nits;
	BD3_PVT		result;
	
	double sx[1][20], sy[1][20], sz[1][20];

	double	daicy_delta_pr[13];
	double	daicy_x,daicy_y,daicy_z;


	//  USE ITERATIVE迭代 APPROACH TO SOLVING FOR THE POSITION OF THE RECEIVER
	nits=0;
	t=0.0;

	
	x = BD3_rec_pos_xyz.x;
	y = BD3_rec_pos_xyz.y;
	z = BD3_rec_pos_xyz.z;
	
	BD3_daicy_temp_cnt = 0;
	
	do
	{
		BD3_daicy_temp_cnt++;
		for (i=1;i<=nsl;i++)
		{
			/*sx[i - 1] = track_sat[i].x;
			sy[i - 1] = track_sat[i].y;
			sz[i - 1] = track_sat[i].z;*/

			t=0.0;//added by daicy
			alpha = (t - BD3_dt[i])*BD3_omegae;
			
			r = sqrt(pow(BD3_track_sat[i].x*cos(alpha) - BD3_track_sat[i].y*sin(alpha) - x, 2) +
				pow(BD3_track_sat[i].y*cos(alpha) + BD3_track_sat[i].x*sin(alpha) - y, 2) +
				pow(BD3_track_sat[i].z - z, 2));

			// compute range residual
			bm[i] = r - (BD3_dt[i] - t)*BD3_c;
			ms[1][i] = (BD3_track_sat[i].x*cos(alpha) - BD3_track_sat[i].y*sin(alpha) - x) / r;
			ms[2][i] = (BD3_track_sat[i].y*cos(alpha) + BD3_track_sat[i].x*sin(alpha) - y) / r;
			ms[3][i] = (BD3_track_sat[i].z - z) / r;
			ms[4][i]=1.0;

			//输出伪距比对
			if (BD3_daicy_temp_cnt>0)
			{
				#if 1 //todo
				//#if 0
				//(39,116,18000)
				daicy_x = BD3_track_sat[i].x*cos(alpha) - BD3_track_sat[i].y*sin(alpha) - (-2181911.708);
				daicy_y = BD3_track_sat[i].y*cos(alpha) + BD3_track_sat[i].x*sin(alpha) - 4473581.957;
				daicy_z = BD3_track_sat[i].z - 4003644.790;
				#else
				//(39,116,100)
				daicy_x=track_sat[i].x*cos(alpha)-track_sat[i].y*sin(alpha)-(-2175813.565);
				daicy_y=track_sat[i].y*cos(alpha)+track_sat[i].x*sin(alpha)-4461078.911;
				daicy_z=track_sat[i].z-3992379.955;
				#endif
				
				daicy_delta_pr[i]=sqrt(daicy_x*daicy_x+daicy_y*daicy_y+daicy_z*daicy_z);
				fprintf(BD3_daicy_file_pr, "m_time[1]: %.9f	%d	dt[i]: %.9f	R: %.2f	", BD3_m_time[1], i, BD3_dt[i], daicy_delta_pr[i]);
				
				daicy_delta_pr[i] = sqrt(daicy_x*daicy_x + daicy_y*daicy_y + daicy_z*daicy_z) - BD3_dt[i] * BD3_c;
				fprintf(BD3_daicy_file_pr, "trans_t: %.2f	delta_t: %.2f	", BD3_dt[i] * BD3_c, daicy_delta_pr[i]);
				
				if(i==nsl)
					fprintf(BD3_daicy_file_pr, "		diff		%.2f	%.2f	%.2f	%.2f	%.2f	%.2f	\n",
										 daicy_delta_pr[2]-daicy_delta_pr[1],daicy_delta_pr[3]-daicy_delta_pr[1],
										 daicy_delta_pr[4]-daicy_delta_pr[1],daicy_delta_pr[5]-daicy_delta_pr[1],
										 daicy_delta_pr[6]-daicy_delta_pr[1],daicy_delta_pr[7]-daicy_delta_pr[1]);
			}
		}
		/*
		//保存卫星x位置结果
		char str0[20];
		char s0[] = "SX";
		char *pfilename0 = strcat(s0, str0);

		ofstream SX;
		SX.open(pfilename0, ios::app);
		SX << sx << "\n";
		SX.close(); // 输出完毕后关闭这个文件

		//保存卫星x位置结果
		char str1[20];
		char s1[] = "SY";
		char *pfilename1 = strcat(s1, str1);

		ofstream SY;
		SY.open(pfilename1, ios::app);
		SY << sy << "\n";
		SY.close(); // 输出完毕后关闭这个文件

					//保存卫星x位置结果
		char str2[20];
		char s2[] = "SZ";
		char *pfilename2 = strcat(s2, str2);

		ofstream SZ;
		SZ.open(pfilename2, ios::app);
		SZ << sz << "\n";
		SZ.close(); // 输出完毕后关闭这个文件
		*/
			
		a1=0.;b1=0.;c1=0.;d1=0.;
		e1=0.;f1=0.;g1=0.;h1=0.;
		i1=0.;j1=0.;k1=0.;l1=0.;
		m1=0.;n1=0.;o1=0.;p1=0.;
		for (k=1;k<=nsl;k++)
		{
			a1+=ms[1][k]*ms[1][k];
			b1+=ms[1][k]*ms[2][k];
			c1+=ms[1][k]*ms[3][k];
			d1+=ms[1][k]*ms[4][k];
			f1+=ms[2][k]*ms[2][k];
			g1+=ms[2][k]*ms[3][k];
			h1+=ms[2][k]*ms[4][k];
			k1+=ms[3][k]*ms[3][k];
			l1+=ms[3][k]*ms[4][k];
			p1+=ms[4][k];
		}
		o1=l1;m1=d1;n1=h1;e1=b1;i1=c1;j1=g1;

		// 求解矩阵的行列式
		denom=(k1*p1-l1*o1)*(a1*f1-b1*e1) + (l1*n1-j1*p1)*(a1*g1-c1*e1) +
				(j1*o1-k1*n1)*(a1*h1-d1*e1) + (l1*m1-i1*p1)*(c1*f1-b1*g1) +
				(i1*o1-k1*m1)*(d1*f1-b1*h1) + (i1*n1-j1*m1)*(c1*h1-d1*g1);
		
		// 求解矩阵的逆
		dd[1][1]=f1*(k1*p1-l1*o1)+g1*(l1*n1-j1*p1)+h1*(j1*o1-k1*n1);
		dd[1][2]=e1*(l1*o1-k1*p1)+g1*(i1*p1-l1*m1)+h1*(k1*m1-i1*o1);
		dd[1][3]=e1*(j1*p1-n1*l1)-i1*(f1*p1-n1*h1)+m1*(f1*l1-j1*h1);
		dd[1][4]=e1*(n1*k1-j1*o1)+i1*(f1*o1-n1*g1)+m1*(j1*g1-f1*k1);

		dd[2][1]=dd[1][2];
		dd[2][2]=a1*(k1*p1-l1*o1)+c1*(l1*m1-i1*p1)+d1*(i1*o1-k1*m1);
		dd[2][3]=a1*(l1*n1-j1*p1)+i1*(b1*p1-n1*d1)+m1*(j1*d1-b1*l1);
		dd[2][4]=a1*(j1*o1-n1*k1)-i1*(b1*o1-n1*c1)+m1*(b1*k1-c1*j1);

		dd[3][1]=dd[1][3];

		dd[3][2]=dd[2][3];
		dd[3][3]=a1*(f1*p1-h1*n1)+b1*(h1*m1-e1*p1)+d1*(e1*n1-f1*m1);
		dd[3][4]=a1*(n1*g1-f1*o1)+e1*(b1*o1-c1*n1)+m1*(c1*f1-b1*g1);

		dd[4][1]=dd[1][4];

		dd[4][2]=dd[2][4];

		dd[4][3]=dd[3][4];
		dd[4][4]=a1*(f1*k1-g1*j1)+b1*(g1*i1-e1*k1)+c1*(e1*j1-f1*i1);

		if ( denom<=0.0 )
		{
			result.x=1.0;    
			result.y=1.0;      
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
			t -= br[4] / BD3_c;
			correct_mag=sqrt(br[1]*br[1]+br[2]*br[2]+br[3]*br[3]);
		}
	} while ( correct_mag > 0.01 && correct_mag < 1.e8 && nits < 10);
	

	result.x=x;
	result.y=y;
	result.z=z;
	result.dt=t;

	fprintf(BD3_daicy_file_pos, "m_time[1]: %.4f	x: %.2f	y: %.2f		z: %.2f		delta_t: %.9f\n", BD3_m_time[1], x, y, z, t);
	
	//  Now for Velocity
	for (i=1;i<=nsl;i++)
	{
		alpha = (BD3_dt[i] - t)*BD3_omegae;

		r = sqrt(pow(BD3_track_sat[i].x*cos(alpha) - BD3_track_sat[i].y*sin(alpha) - x, 2) +
			pow(BD3_track_sat[i].y*cos(alpha) + BD3_track_sat[i].x*sin(alpha) - y, 2) +
			pow(BD3_track_sat[i].z - z, 2));

		bm[i] = ((BD3_track_sat[i].x*cos(alpha) - BD3_track_sat[i].y*sin(alpha) - x)*BD3_d_sat[i].x +
			(BD3_track_sat[i].y*cos(alpha) + BD3_track_sat[i].x*sin(alpha) - y)*BD3_d_sat[i].y +
			(BD3_track_sat[i].z - z)*BD3_d_sat[i].z) / r + BD3_meas_dop[i] * BD3_lambda;
	}


	for (i=1;i<=4;i++)
	{
		br[i]=0.0;
		for (j=1;j<=nsl;j++)br[i]+=bm[j]*pm[i][j];
	}

	result.xv = br[1] + y*BD3_omegae;
	result.yv = br[2] - x*BD3_omegae;
	result.zv=br[3];
	result.df = br[4] / BD3_c*1000000.0;
	return(result);
}

BD3_PVT BD3_kalman_pos_vel(int nsl)
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
	BD3_PVT		result;

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

	x = BD3_rec_pos_xyz.x;
	y = BD3_rec_pos_xyz.y;
	z = BD3_rec_pos_xyz.z;
	old_receiver(0) = /*0;//*/ x;
	old_receiver(1) = /*0;// */BD3_rpvt.xv;
	old_receiver(2) = /*0;//*/ y;
	old_receiver(3) = /*0;// */ BD3_rpvt.yv;
	old_receiver(4) = /*0;//*/ z;
	old_receiver(5) = /*0;// */ BD3_rpvt.zv;
	old_receiver(6) = t;
	old_receiver(7) = 0;


	//do
	//{

	for (int j = 0; j < nsl; j++)  //vector转换成eigen
	{
		dx(j) = BD3_track_sat[j + 1].x;
		dy(j) = BD3_track_sat[j + 1].y;
		dz(j) = BD3_track_sat[j + 1].z;
		satVx(j) = BD3_d_sat[j + 1].x;
		satVy(j) = BD3_d_sat[j + 1].y;
		satVz(j) = BD3_d_sat[j + 1].z;
		range_Final_1(j) = BD3_dt[j + 1];
		dop(j) = BD3_meas_dop[j + 1];
	}

	Pk = A * 1000 * P0*(A.transpose()) + Q;
	detaX_t = A*old_receiver;// .transpose();

	/*得到观测矩阵H*/

	for (int k = 0; k < nsl; k++)
	{
		range_Final_2(k) = BD3_lambda* dop(k);

		alpha(k) = (range_Final_1(k)*BD3_c - detaX_t(6))*BD3_omegae / BD3_c;
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
		Y(2 * (i)) = BD3_c*range_Final_1(i);
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
	result.dt = old_receiver(6) / BD3_c;
	result.df = old_receiver(7) / BD3_c*1000000.0;

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
void  BD3_dops(int nsl)
{
	double r,xls,yls,zls;
	int i;
	double **H, **G;
	
	H = BD3_dmatrix(nsl, 4);
	G = BD3_dmatrix(4, 4);
	
	BD3_receiver.north.x = -cos(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
	BD3_receiver.north.y = -sin(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
	BD3_receiver.north.z = cos(BD3_rec_pos_llh.lat);
	BD3_receiver.east.x = -sin(BD3_rec_pos_llh.lon);
	BD3_receiver.east.y = cos(BD3_rec_pos_llh.lon);
	BD3_receiver.up.x = cos(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
	BD3_receiver.up.y = sin(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
	BD3_receiver.up.z = sin(BD3_rec_pos_llh.lat);
	for (i=1;i<=nsl;i++)
	{
		//   Compute line of sight vectors
		xls = BD3_track_sat[i].x - BD3_rec_pos_xyz.x;
		yls = BD3_track_sat[i].y - BD3_rec_pos_xyz.y;
		zls = BD3_track_sat[i].z - BD3_rec_pos_xyz.z;

		r=sqrt(xls*xls+yls*yls+zls*zls);

		H[i - 1][0] = (xls*BD3_receiver.north.x + yls*BD3_receiver.north.y + zls*BD3_receiver.north.z) / r;
		H[i - 1][1] = (xls*BD3_receiver.east.x + yls*BD3_receiver.east.y) / r;
		H[i - 1][2] = (xls*BD3_receiver.up.x + yls*BD3_receiver.up.y + zls*BD3_receiver.up.z) / r;
		H[i-1][3]=1.0;
	}

	BD3_mult_ATB(H, nsl, 4, H, nsl, 4, G);
	BD3_matinv(G, 4);
	BD3_hdop = sqrt(G[0][0] + G[1][1]);
	BD3_vdop = sqrt(G[2][2]);
	BD3_tdop = sqrt(G[3][3]);
	BD3_pdop = sqrt(G[0][0] + G[1][1] + G[2][2]);
	BD3_gdop = sqrt(G[0][0] + G[1][1] + G[2][2] + G[3][3]);
	BD3_free_dmatrix(G, 4);
	BD3_free_dmatrix(H, nsl);
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

double BD3_tropo_iono(char ch, double az, double el, double gps_time)
{
	double d_Trop,alt_factor;
	double d_Ion,psi,phi,lambdai,phim,per,x,F,amp,t;

	//  Try a simple troposphere model
	if (BD3_current_loc.hae>200000.0) alt_factor = 0.0;
	else if (BD3_current_loc.hae<0.0) alt_factor = 1.0;
	else alt_factor = exp(-BD3_current_loc.hae*1.33e-4);

	// to check if troposphere model is needed or not
	d_Trop = (BD3_m_tropo == 1) ? 2.47 / (sin(el) + .0121)*alt_factor / BD3_c : 0.0;
	BD3_chan[ch].Tropo = d_Trop;

	//  Use an ionosphere model
//	if (m_iono==1)  //try to test ion effective
	{
		psi = 0.0137 / (el / BD3_pi + 0.11) - .022;
		phi = BD3_current_loc.lat / BD3_pi + psi*cos(az);
		if (phi > 0.416) phi= 0.416;
		else if (phi <-0.416 ) phi=-0.416;
		lambdai = BD3_current_loc.lon / BD3_pi + psi*sin(az) / cos(phi*BD3_pi);
		t=43200.0*lambdai+gps_time-int((43200.0*lambdai+gps_time)/86400.)*86400.;
		if (t<0.0) t=t+86400.0;
		phim = phi + 0.064*cos((lambdai - 1.617)*BD3_pi);
		
		//  If available from the nav message use its Ionosphere model
		if (BD3_b0 != 0.0 && BD3_al0 != 0.0)
		{
			per = BD3_b0 + BD3_b1*phim + BD3_b2*phim*phim + BD3_b3*phim*phim*phim;
			amp = BD3_al0 + BD3_al1*phim + BD3_al2*phim*phim + BD3_al3*phim*phim*phim;
		}
		else // else try this set of default iono model parameters
		{
			per=141312.0-32768.0*phim-131072.0*phim*phim-65536.0*phim*phim*phim;
			amp=3.46e-8+7.45e-9*phim-1.19e-7*phim*phim+5.96e-8*phim*phim*phim;
		}
		if ( per <72000.0 ) per=72000.0;
		x = 2.*BD3_pi*(t - 50400.) / per;
		F = 1.0 + 16.0*pow(0.53 - el / BD3_pi, 3);
		if ( amp < 0.0 ) amp=0.0;
		if (fabs(x) < 1.5707)  d_Ion=F*(5.0e-9+amp*(1.0-x*x/2.+x*x*x*x/24.0));
		else                   d_Ion=F*5.0e-9;
	}
	//else d_Ion=0.0;
	BD3_chan[ch].Iono = d_Ion;
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
这个函数从ion_utc.dat读广播电离层校正模型和GPS时到UTC的转换的参数
ion_utc.dat在数据从卫星读到时由程序创建
*******************************************************************************/
char BD3_ion_utc_file[4096];
void  BD3_read_ion_utc(void)
{
	if ((BD3_in = fopen(BD3_ion_utc_file, "rt")) == NULL)
	{
		printf("Cannot open ion_utc.dat file.\n");
	}
	else
	{
		BD3_handle = _fileno(BD3_in);
		while (!_eof(BD3_handle))
		{
			//ion
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_al0);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_al1);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_al2);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_al3);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_b0);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_b1);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_b2);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_b3);
			//utc
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_a0);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_a1);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_dtls);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_tot);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_WNt);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_WNlsf);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_DN);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &BD3_dtlsf);
		}
		fclose(BD3_in);
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
char BD3_current_alm_file[4096];
void BD3_read_almanac(void)
{
	int   id,health,week;
	float eccen,rinc,rras,sqra;
	float ratoa,aopg,rma,af0,af1,toa;

	if ((BD3_in = fopen(BD3_current_alm_file, "rt")) == NULL)
	{
		printf("Cannot open current.rcv file.\n");
		for (id=1;id<=12;id++)
		{
			BD3_gps_alm[id].week = BD3_gps_week - 1;
			BD3_gps_alm[id].inc = 1.0;
		}
	}
	else
	{
		BD3_status = BD3_warm_start;
		BD3_handle = _fileno(BD3_in);
		while (!_eof(BD3_handle))
		{
			fscanf(BD3_in, "%46c", &BD3_header);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%d", &id);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%i", &health);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &eccen);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &toa);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &rinc);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &rras);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &sqra);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &ratoa);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &aopg);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &rma);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &af0);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%f", &af1);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%i", &week);
			fscanf(BD3_in, "%c", &BD3_trailer);
			BD3_gps_alm[id].health = health;
			BD3_gps_alm[id].week = week;
			BD3_gps_alm[id].toa = toa;
			BD3_gps_alm[id].ety = eccen;
			BD3_gps_alm[id].toa = toa;
			BD3_gps_alm[id].inc = rinc;
			BD3_gps_alm[id].rra = rras;
			BD3_gps_alm[id].sqa = sqra;
			BD3_gps_alm[id].lan = ratoa;
			BD3_gps_alm[id].aop = aopg;
			BD3_gps_alm[id].ma = rma;
			BD3_gps_alm[id].af0 = af0;
			BD3_gps_alm[id].af1 = af1;
			BD3_gps_alm[id].sat_file = 0;
			if (BD3_gps_alm[id].sqa>0.0) BD3_gps_alm[id].w = 19964981.84 / pow(BD3_gps_alm[id].sqa, 3);
		}
		fclose(BD3_in);

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
char BD3_current_eph_file[4096];
void BD3_read_ephemeris()
{
	int   id,health,week,ura,iode,iodc;
	double  toc,toe;
	double crc,crs,cic,cis,cuc,cus,tgd,ety,inc0,omegadot,w0,w,ma,dn,idot;
	double daf0,daf1,daf2,esqra;
	double d_toe;
	FILE   *ephfile;

	int i;

	if ((ephfile = fopen(BD3_current_eph_file, "rt")) == NULL)//if ((ephfile = fopen("current.eph", "rt")) == NULL)
	{
		printf("Cannot open currrent.eph file.\n");
	}
	else
	{
		BD3_availablesv.availablesvnumber = 0;
		i=0;
//		handle=fileno(in);
		while (!feof(ephfile))
		{
			fscanf(BD3_in, "%37c", &BD3_header);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%i", &id);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%i", &health);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%i", &week);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%i", &ura);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &toe);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%i", &iode);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &toc);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%i", &iodc);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &tgd);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &daf0);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &daf1);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &daf2);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &ety);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &inc0);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &idot);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &omegadot);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &esqra);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &dn);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &w0);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &w);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &ma);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &cuc);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &cus);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &crc);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &crs);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &cic);
			fscanf(BD3_in, "%27c", &BD3_ttext);
			fscanf(BD3_in, "%le", &cis);
			fscanf(BD3_in, "%c", &BD3_trailer);

			d_toe = BD3_clock_tow - toe;
			if (d_toe>302400.0) d_toe=d_toe-604800.0;
			else if (d_toe<-302400.0)d_toe=d_toe+604800.0;

			if ( fabs(d_toe)<7200.0)
			{
				BD3_gps_eph[id].valid = 1;
				BD3_gps_eph[id].health = health;
				BD3_gps_eph[id].week = week;
				BD3_gps_eph[id].ura = ura;
				BD3_gps_eph[id].toe = toe;
				BD3_gps_eph[id].iode = iode;
				BD3_gps_eph[id].toc = toc;
				BD3_gps_eph[id].iodc = iodc;
				BD3_gps_eph[id].tgd = tgd;
				BD3_gps_eph[id].af0 = daf0;
				BD3_gps_eph[id].af1 = daf1;
				BD3_gps_eph[id].af2 = daf2;
				BD3_gps_eph[id].ety = ety;
				BD3_gps_eph[id].inc0 = inc0;
				BD3_gps_eph[id].idot = idot;
				BD3_gps_eph[id].omegadot = omegadot;
				BD3_gps_eph[id].sqra = esqra;
				BD3_gps_eph[id].dn = dn;
				BD3_gps_eph[id].w0 = w0;
				BD3_gps_eph[id].w = w;
				BD3_gps_eph[id].ma = ma;
				BD3_gps_eph[id].cuc = cuc;
				BD3_gps_eph[id].cus = cus;
				BD3_gps_eph[id].crc = crc;
				BD3_gps_eph[id].crs = crs;
				BD3_gps_eph[id].cic = cic;
				BD3_gps_eph[id].cis = cis;
				if (BD3_gps_eph[id].sqra >0.0) BD3_gps_eph[id].wm = 19964981.84 / pow(BD3_gps_eph[id].sqra, 3);
				if (BD3_out_debug)  BD3_write_Debug_ephemeris(id);

				BD3_availablesv.availablesvnumber++;
				BD3_availablesv.availablesvprn[i] = id;
	         	i++;
			}  
		}  
		fclose(BD3_in);
	} 
}
char BD3_last_prn_file[4096];
void BD3_read_prn(void)
{
	int   i = 0,id;
	
	if ((BD3_fprn_in = fopen(BD3_last_prn_file, "rt")) == NULL)
	{
		printf("Cannot open lastime.prn file.\n");
	}
	else
	{
	
		for (i = 0; i <= BD3_chmax; i++)
		{
			fscanf(BD3_fprn_in, "%27c", &BD3_ttext);
			fscanf(BD3_fprn_in, "%d", &id);
		
		}
		fclose(BD3_fprn_in);
	}

}


void BD3_write_prn()
{
	int i;
	BD3_fprn_out = fopen(BD3_last_prn_file, "w+");
	for (i = 0; i <= BD3_chmax; i++)
	{
		fprintf(BD3_fprn_out, "prn:                       %4d\n", BD3_chan[i].prn);

	}
	fclose(BD3_fprn_out);
}
/*******************************************************************************
FUNCTION write_almanac()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast almanac data to a file for later
	use.  In particular to support a warm or hot start.

*******************************************************************************/

void BD3_write_almanac()
{
	int i;
	BD3_out = fopen(BD3_current_alm_file, "w+");
	for (i=1;i<=12;i++)
	{
		if (BD3_gps_alm[i].inc >0.0)
		{
			fprintf(BD3_out, "**** Week %4d almanac for PRN-%2d ***********\n",
				BD3_gps_alm[i].week % 1024, i);
			fprintf(BD3_out, "ID:                         %3d\n", i);
			fprintf(BD3_out, "Health:                     %3d\n", BD3_gps_alm[i].health);
			fprintf(BD3_out, "Eccentricity:               %10.9e\n", BD3_gps_alm[i].ety);
			fprintf(BD3_out, "Time of Applicability(s):   %10.9e\n", BD3_gps_alm[i].toa);
			fprintf(BD3_out, "Orbital Inclination(rad):   %10.9e\n", BD3_gps_alm[i].inc);
			fprintf(BD3_out, "Rate of Right Ascen(R/s):   %10.9e\n", BD3_gps_alm[i].rra);
			fprintf(BD3_out, "SQRT(A) (m^1/2):            %10.9e\n", BD3_gps_alm[i].sqa);
			fprintf(BD3_out, "Right Ascen at TOA(rad):    %10.9e\n", BD3_gps_alm[i].lan);
			fprintf(BD3_out, "Argument of Perigee(rad):   %10.9e\n", BD3_gps_alm[i].aop);
			fprintf(BD3_out, "Mean Anom(rad):             %10.9e\n", BD3_gps_alm[i].ma);
			fprintf(BD3_out, "Af0(s):                     %10.9e\n", BD3_gps_alm[i].af0);
			fprintf(BD3_out, "Af1(s/s):                   %10.9e\n", BD3_gps_alm[i].af1);
			fprintf(BD3_out, "week:                       %4d   \n", BD3_gps_alm[i].week % 1024);
			fprintf(BD3_out, "\n");
		}
	}
	fclose(BD3_out);
}

/*******************************************************************************
FUNCTION write_ephemeris()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ephemeris data to a file for later
	use.  In particular to support a hot start.

*******************************************************************************/

void BD3_write_ephemeris()
{
	int i;
	BD3_out = fopen(BD3_current_eph_file, "w+");
	for (i=1;i<=12;i++)
	{
		if (BD3_gps_eph[i].inc0 >0.0)
		{
			fprintf(BD3_out, "**** Ephemeris for PRN-%2d ***********\n", i);
			fprintf(BD3_out, "ID:                         %3d\n", i);
			fprintf(BD3_out, "Health:                     %3d\n", BD3_gps_eph[i].health);
			fprintf(BD3_out, "Week:                       %4d\n", BD3_gps_eph[i].week);
			fprintf(BD3_out, "URA:                        %3d\n", BD3_gps_eph[i].ura);
			fprintf(BD3_out, "E Time of Applic(s):        %12.11e\n", BD3_gps_eph[i].toe);
			fprintf(BD3_out, "IODE:                       %4d\n", BD3_gps_eph[i].iode);
			fprintf(BD3_out, "C Time of Applic(s):        %12.11e\n", BD3_gps_eph[i].toc);
			fprintf(BD3_out, "IODC:                       %4d\n", BD3_gps_eph[i].iodc);
			fprintf(BD3_out, "Tgd(s):                     %12.11e\n", BD3_gps_eph[i].tgd);
			fprintf(BD3_out, "Af0(s):                     %12.11e\n", BD3_gps_eph[i].af0);
			fprintf(BD3_out, "Af1(s/s):                   %12.11e\n", BD3_gps_eph[i].af1);
			fprintf(BD3_out, "Af2(s/s/s):                 %12.11e\n", BD3_gps_eph[i].af2);
			fprintf(BD3_out, "Eccentricity:               %12.11e\n", BD3_gps_eph[i].ety);
			fprintf(BD3_out, "Orbital Inclination(rad):   %12.11e\n", BD3_gps_eph[i].inc0);
			fprintf(BD3_out, "inc rate (r/s)              %12.11e\n", BD3_gps_eph[i].idot);
			fprintf(BD3_out, "Rate of Right Ascen(R/s):   %12.11e\n", BD3_gps_eph[i].omegadot);
			fprintf(BD3_out, "SQRT(A) (m^1/2):            %12.11e\n", BD3_gps_eph[i].sqra);
			fprintf(BD3_out, "dn                          %12.11e\n", BD3_gps_eph[i].dn);
			fprintf(BD3_out, "Right Ascen at TOE(rad):    %12.11e\n", BD3_gps_eph[i].w0);
			fprintf(BD3_out, "Argument of Perigee(rad):   %12.11e\n", BD3_gps_eph[i].w);
			fprintf(BD3_out, "Mean Anom(rad):             %12.11e\n", BD3_gps_eph[i].ma);
			fprintf(BD3_out, "Cuc(rad):                   %12.11e\n", BD3_gps_eph[i].cuc);
			fprintf(BD3_out, "Cus(rad):                   %12.11e\n", BD3_gps_eph[i].cus);
			fprintf(BD3_out, "Crc(m):                     %12.11e\n", BD3_gps_eph[i].crc);
			fprintf(BD3_out, "Crs(m):                     %12.11e\n", BD3_gps_eph[i].crs);
			fprintf(BD3_out, "Cic(rad):                   %12.11e\n", BD3_gps_eph[i].cic);
			fprintf(BD3_out, "Cis(rad):                   %12.11e\n", BD3_gps_eph[i].cis);
			fprintf(BD3_out, "\n");
		}
	}
	fclose(BD3_out);
}

/*******************************************************************************
FUNCTION write_ephemeris()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ephemeris data to a file for later
	use.  In particular to support a hot start.

*******************************************************************************/
void BD3_write_Debug_ephemeris(int i)
{
	fprintf(BD3_debug, "**** Ephemeris for PRN-%2d ***********\n", i);
	fprintf(BD3_debug, "ID:                         %3d\n", i);
	fprintf(BD3_debug, "Health:                     %3d\n", BD3_gps_eph[i].health);
	fprintf(BD3_debug, "Week:                       %4d\n", BD3_gps_eph[i].week);
	fprintf(BD3_debug, "URA:                        %3d\n", BD3_gps_eph[i].ura);
	fprintf(BD3_debug, "E Time of Applic(s):        %12.11e\n", BD3_gps_eph[i].toe);
	fprintf(BD3_debug, "IODE:                       %4d\n", BD3_gps_eph[i].iode);
	fprintf(BD3_debug, "C Time of Applic(s):        %12.11e\n", BD3_gps_eph[i].toc);
	fprintf(BD3_debug, "IODC:                       %4d\n", BD3_gps_eph[i].iodc);
	fprintf(BD3_debug, "Tgd(s):                     %12.11e\n", BD3_gps_eph[i].tgd);
	fprintf(BD3_debug, "Af0(s):                     %12.11e\n", BD3_gps_eph[i].af0);
	fprintf(BD3_debug, "Af1(s/s):                   %12.11e\n", BD3_gps_eph[i].af1);
	fprintf(BD3_debug, "Af2(s/s/s):                 %12.11e\n", BD3_gps_eph[i].af2);
	fprintf(BD3_debug, "Eccentricity:               %12.11e\n", BD3_gps_eph[i].ety);
	fprintf(BD3_debug, "Orbital Inclination(rad):   %12.11e\n", BD3_gps_eph[i].inc0);
	fprintf(BD3_debug, "inc rate (r/s)              %12.11e\n", BD3_gps_eph[i].idot);
	fprintf(BD3_debug, "Rate of Right Ascen(R/s):   %12.11e\n", BD3_gps_eph[i].omegadot);
	fprintf(BD3_debug, "SQRT(A) (m^1/2):            %12.11e\n", BD3_gps_eph[i].sqra);
	fprintf(BD3_debug, "dn                          %12.11e\n", BD3_gps_eph[i].dn);
	fprintf(BD3_debug, "Right Ascen at TOE(rad):    %12.11e\n", BD3_gps_eph[i].w0);
	fprintf(BD3_debug, "Argument of Perigee(rad):   %12.11e\n", BD3_gps_eph[i].w);
	fprintf(BD3_debug, "Mean Anom(rad):             %12.11e\n", BD3_gps_eph[i].ma);
	fprintf(BD3_debug, "Cuc(rad):                   %12.11e\n", BD3_gps_eph[i].cuc);
	fprintf(BD3_debug, "Cus(rad):                   %12.11e\n", BD3_gps_eph[i].cus);
	fprintf(BD3_debug, "Crc(m):                     %12.11e\n", BD3_gps_eph[i].crc);
	fprintf(BD3_debug, "Crs(m):                     %12.11e\n", BD3_gps_eph[i].crs);
	fprintf(BD3_debug, "Cic(rad):                   %12.11e\n", BD3_gps_eph[i].cic);
	fprintf(BD3_debug, "Cis(rad):                   %12.11e\n", BD3_gps_eph[i].cis);
	fprintf(BD3_debug, "\n");
}


/*******************************************************************************
FUNCTION write_ion_utc()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ionospheric correction data and the
      parameters to tie GPS time to UTC to a file for later use.

*******************************************************************************/

void BD3_write_ion_utc()
{
	BD3_out = fopen(BD3_ion_utc_file, "w+");
	fprintf(BD3_out, "al0:                        %e\n", BD3_al0);
	fprintf(BD3_out, "al1:                        %e\n", BD3_al1);
	fprintf(BD3_out, "al2:                        %e\n", BD3_al2);
	fprintf(BD3_out, "al3:                        %e\n", BD3_al3);
	fprintf(BD3_out, "b0:                         %f\n", BD3_b0);
	fprintf(BD3_out, "b1:                         %f\n", BD3_b1);
	fprintf(BD3_out, "b2:                         %f\n", BD3_b2);
	fprintf(BD3_out, "b3:                         %f\n", BD3_b3);
	fprintf(BD3_out, "a0                          %f\n", BD3_a0);
	fprintf(BD3_out, "a1                          %f\n", BD3_a1);
	fprintf(BD3_out, "dtls                        %f\n", BD3_dtls);
	fprintf(BD3_out, "tot                         %f\n", BD3_tot);
	fprintf(BD3_out, "WNt                         %f\n", BD3_WNt);
	fprintf(BD3_out, "WNlsf                       %f\n", BD3_WNlsf);
	fprintf(BD3_out, "DN                          %f\n", BD3_DN);
	fprintf(BD3_out, "dtlsf                       %f\n", BD3_dtlsf);
	fclose(BD3_out);
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
/*
inline long rss(long a,long b )
{
	long result,c,d;
	c=labs(a);
	d=labs(b);
	if (c==0 && d==0) result=0;
	else
	{
		if (c>d) result=(d>>1)+c;
		else     result=(c>>1)+d;
	}
	return (result);
}
*/
/*******************************************************************************
FUNCTION fix_sqrt(long x)
RETURNS  long integer

PARAMETERS
      x long integer

PURPOSE
	This function finds the fixed point square root of a long integer

*******************************************************************************/

long BD3_fix_sqrt(long x)
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

#define BD3_SCALED_PI_ON_2  25736L
#define BD3_SCALED_PI       51472L

inline long fix_atan2(long y,long x)
{
	long result,n,n3;
	if ((x==0) && (y==0))
		return(0); // invalid case

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
		result = (y>0) ? n - n3 + BD3_SCALED_PI : n - n3 - BD3_SCALED_PI;
	}
	else if (y>0 &&  y>labs(x))
	{
		n=(x<<14)/y;
		n3=((((n*n)>>14)*n)>>13)/9;
		result = BD3_SCALED_PI_ON_2 - n + n3;
	}
	else if (y<0 && -y>labs(x))
	{
		n=(x<<14)/y;
		n3=((((n*n)>>14)*n)>>13)/9;
		result = -n + n3 - BD3_SCALED_PI_ON_2;
	}
	return(result);
}

// 计算n×n的对称矩阵q的逆矩阵
int BD3_matinv(double **q, int n)
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


// 计算arows行，acols列的A矩阵的转置和brows行，bcols列的B矩阵的乘积
void BD3_mult_ATB(double **A, int arows, int acols, double **B, int brows,
                                  int bcols, double **C)
{
	int i, j, k;

	// 首先检测两个矩阵的行列是否匹配
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

// 分配一个row行，column列的double型数组
double** BD3_dmatrix(unsigned row, unsigned column)
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


// FREES A MATRIX ALLOCATED WITH DMATRIX
void BD3_free_dmatrix(double **m, unsigned row)
{
	unsigned i;
	for (i=0; i<row; i++) free((double*)m[i]);
	free((double **)m);
}
