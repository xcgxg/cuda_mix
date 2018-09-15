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
#include "gpsstruct.h"
#include "gpsconst.h"
#include "gpsfuncs.h"

// external variables
extern CHANNEL	chan[12];
extern int		out_debug;
extern FILE*	fpeph;
extern int sel;

// global variables
char			tzstr[40];				
time_t			thetime;					
FILE			*stream,*debug,*in,*out,*kalm;	
FILE            *fprn_in,*fprn_out;
ALMANAC			gps_alm[33];			
EPHEMERIS		gps_eph[33];			

AVAILABLESV      availablesv;


int				SVh[33],ASV[33];			
double			b0,b1,b2,b3,al0,al1,al2,al3;	
double			a0,a1,tot,WNt,dtls,WNlsf,DN,dtlsf;

PVT				rpvt;
STATE			receiver;

double			gdop,pdop,hdop,vdop,tdop,alm_toa;
unsigned long	clock_tow;
LLH				rec_pos_llh;
LLH				current_loc,rp_llh;
ECEFT			track_sat[13];					
XYZ				rec_pos_xyz;
int				alm_gps_week,gps_week,almanac_valid,almanac_flag,handle;
unsigned long	sf[6][11];						

int				p_error[6];						
												

int				status;							

unsigned long	test_l[33]={0x00000000L,            // single bit set numbers
  0x00000001L,0x00000002L,0x00000004L,0x00000008L,  // for testing bit positions
  0x00000010L,0x00000020L,0x00000040L,0x00000080L,	// 0000 0001 0010 0100 1000
  0x00000100L,0x00000200L,0x00000400L,0x00000800L,	// 32位的第1位到第32位
  0x00001000L,0x00002000L,0x00004000L,0x00008000L,
  0x00010000L,0x00020000L,0x00040000L,0x00080000L,
  0x00100000L,0x00200000L,0x00400000L,0x00800000L,
  0x01000000L,0x02000000L,0x04000000L,0x08000000L,
  0x10000000L,0x20000000L,0x40000000L,0x80000000L};

double		mask_angle;
char		header[45],ttext[27],trailer;
double		meas_dop[13];							// 测量的每个通道的多普勒
XYZ			d_sat[13];								// 每个通道的卫星的速度
double		carrier_ref;				// 载波标称中频，单位：Hz
double		code_ref=1023000;						// CA码速率标频，单位：chip/s
double		dt[13];									// 从卫星到接收机的传播时间差
double		cbias;									// 接收机钟差

// binary constants for nav message decoding

double c_2p12 = 4096;
double c_2p4  = 16;
double c_2m5  = 0.03125;
double c_2m11 = 4.8828125e-4;
double c_2m19 = 1.9073486328125e-6;
double c_2m20 = 9.5367431640625e-7;
double c_2m21 = 4.76837158203125e-7;
double c_2m23 = 1.19209289550781e-7;
double c_2m24 = 5.96046447753906e-8;
double c_2m27 = 7.45058059692383e-9;
double c_2m29 = 1.86264514923096e-9;
double c_2m30 = 9.31322574615479e-10;
double c_2m31 = 4.65661287307739e-10;
double c_2m33 = 1.16415321826935E-10;
double c_2m38 = 3.63797880709171e-12;
double c_2m43 = 1.13686837721616e-13;
double c_2m50 = 8.881784197e-16;
double c_2m55 = 2.77555756156289e-17;

int			m_tropo, m_iono;				// flags for using tropo and iono models
int			align_t;						// 将接收机时间和GPS时间对准的标志
SATVIS		xyz[33];						// 卫星的位置、方位和多普勒

char last_prn[12];
/*******************************************************************************
FUNCTION satfind()
RETURNS  None.

PARAMETERS None.

PURPOSE

	THIS FUNCTION DETERMINES THE SATELLITES TO SEARCH FOR
	WHEN ALMANAC DATA IS AVAILABLE

*******************************************************************************/
SATVIS satfind(char i)
{
	double	tdot,az;
	double	satang,alm_time,almanac_date;
	double	range1,range2,xls,yls,zls,xaz,yaz, rangeInv;
	long	jd_yr;
	XYZ		gpspos1,gpspos2,north,east,up;
	SATVIS	result;
	int		jd_m;
	struct	tm *gmt;
	double  time_s;

    
	// INITIALIZE ALL THE CONSTANTS
	// 防止返回结果混乱
	result.azimuth = result.doppler = result.elevation = 0.0;
	result.x = result.y = result.z = 0.0;

	_putenv(tzstr);

	_tzset();
	gmt=gmtime(&thetime);
	if (gmt->tm_mon <= 1)
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
	gps_week=int((time_s-2444244.5)/7.);
	almanac_date=gps_alm[i].week*7.0+2444244.5;

	if (gps_week-gps_alm[i].week>512) almanac_date+=1024*7.0;
	alm_time=(time_s-almanac_date)*86400.;
	//clock_tow=(long)((time_s-gps_week*7.-2444244.5)*86400.); //周内秒计算，结果不正确，不再使用。
	if (gps_alm[i].inc > 0.0 && i>0)
    {
		gpspos1=satpos_almanac(alm_time,i);
		gpspos2=satpos_almanac(alm_time+1.0,i);

		rec_pos_xyz=llh_to_ecef(current_loc);
		north.x = -cos(current_loc.lon)*sin(current_loc.lat);
		north.y = -sin(current_loc.lon)*sin(current_loc.lat);
		north.z = cos(current_loc.lat);
		east.x = -sin(current_loc.lon);
		east.y = cos(current_loc.lon);
		up.x = cos(current_loc.lon)*cos(current_loc.lat);
		up.y = sin(current_loc.lon)*cos(current_loc.lat);
		up.z = sin(current_loc.lat);

		xls =gpspos1.x-rec_pos_xyz.x;
		yls =gpspos1.y-rec_pos_xyz.y;
		zls =gpspos1.z-rec_pos_xyz.z;
		range1=sqrt(xls*xls+yls*yls+zls*zls);
		rangeInv = 1.0/range1;
		xls *= rangeInv;	
		yls *= rangeInv;
		zls *= rangeInv;

		tdot=up.x*xls+up.y*yls+up.z*zls;
		range2=sqrt(pow(gpspos2.x-rec_pos_xyz.x-rpvt.xv,2)+
					pow(gpspos2.y-rec_pos_xyz.y-rpvt.yv,2)+
					pow(gpspos2.z-rec_pos_xyz.z-rpvt.zv,2));
		if ( tdot >= 1.00 ) 
			satang=pi/2.0;
		else if ( tdot <= -1.00 ) 
			satang=-pi/2.0;
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

XYZ satpos_almanac(double time, char n)
{
	double ei,ea,diff,r,ta,la,aol,xp,yp,d_toa;
	XYZ result;

	
	d_toa=time-gps_alm[n].toa;
	if (d_toa>302400.0) d_toa=d_toa-604800.0;
	else if (d_toa<-302400.0)d_toa=d_toa+604800.0;
	ei=gps_alm[n].ma+d_toa*gps_alm[n].w;
    ea=ei;	
	do
	{
       diff= (ei-(ea-gps_alm[n].ety*sin(ea)))/(1.-gps_alm[n].ety*cos(ea));
       ea += diff;
	} while (fabs(diff) > 1.0e-6);
      
	if (gps_alm[n].ety != 0.0 )	
		ta=atan2(sqrt(1.-pow(gps_alm[n].ety,2))*sin(ea),cos(ea)-gps_alm[n].ety);
	else ta=ea;
	r=pow(gps_alm[n].sqa,2)*(1.-pow(gps_alm[n].ety,2)*cos(ea));
    //  AOL IS THE ARGUMENT OF LATITUDE
    aol=ta+gps_alm[n].aop;

	//LA IS THE LONGITUDE OF THE ASCENDING NODE
	la=gps_alm[n].lan+(gps_alm[n].rra-omegae)*d_toa-gps_alm[n].toa*omegae;
    xp=r*cos(aol);
    yp=r*sin(aol);
    result.x=xp*cos(la)-yp*cos(gps_alm[n].inc)*sin(la);
    result.y=xp*sin(la)+yp*cos(gps_alm[n].inc)*cos(la);
    result.z=yp*sin(gps_alm[n].inc);
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

ECEFT satpos_ephemeris(double t,char n)
{
	double	ei,ea,diff,ta,aol,delr,delal,delinc,r,inc;
	double	la,xp,yp,bclk,tc,d_toc,d_toe;
	double	xls,yls,zls,range1,tdot,satang,xaz,yaz;
	double	az;
	XYZ		north,east,up;
	ECEFT	result;
	d_toc=t-gps_eph[n].toc;
    if (d_toc>302400.0) d_toc=d_toc-604800.0;
    else if (d_toc<-302400.0)d_toc=d_toc+604800.0;
	bclk=gps_eph[n].af0+gps_eph[n].af1*d_toc+gps_eph[n].af2*d_toc*d_toc
		-gps_eph[n].tgd;
	tc=t-bclk;
    
	d_toe=tc-gps_eph[n].toe;
    if (d_toe>302400.0) d_toe=d_toe-604800.0;
	else if (d_toe<-302400.0)d_toe=d_toe+604800.0;
	
	// mean anomaly
	ei=gps_eph[n].ma+d_toe*(gps_eph[n].wm+gps_eph[n].dn);
	// iterate eccentric anomaly
    ea=ei;
    do
    {
		diff=(ei-(ea-gps_eph[n].ety*sin(ea)))/(1.0E0-gps_eph[n].ety*cos(ea));
		ea += diff;
	} while (fabs(diff) > 1.0e-12 );
      

	bclk -= 4.442807633E-10*gps_eph[n].ety*gps_eph[n].sqra*sin(ea);
	result.tb=bclk;


	ta=atan2(sqrt(1.00-pow(gps_eph[n].ety,2))*sin(ea),cos(ea)-gps_eph[n].ety);
	// AOL IS THE ARGUMENT OF LATITUDE OF THE SATELLITE
    aol=ta+gps_eph[n].w;

	// calculate the second harmonic perturbations of the orbit
	delr  =gps_eph[n].crc*cos(2.0*aol)+gps_eph[n].crs*sin(2.0*aol);
	delal =gps_eph[n].cuc*cos(2.0*aol)+gps_eph[n].cus*sin(2.0*aol);
	delinc=gps_eph[n].cic*cos(2.0*aol)+gps_eph[n].cis*sin(2.0*aol);


    r=pow(gps_eph[n].sqra,2)*(1.00-gps_eph[n].ety*cos(ea))+delr;
	aol=aol+delal;
	inc=gps_eph[n].inc0+delinc+gps_eph[n].idot*d_toe;

	// LA IS THE CORRECTED LONGITUDE OF THE ASCENDING NODE
    la=gps_eph[n].w0+(gps_eph[n].omegadot-omegae)*d_toe-
		omegae*gps_eph[n].toe;
    xp=r*cos(aol);
    yp=r*sin(aol);
    result.x=xp*cos(la)-yp*cos(inc)*sin(la);
    result.y=xp*sin(la)+yp*cos(inc)*cos(la);
    result.z=yp*sin(inc);
    result.az=0.0;
	result.el=0.0;
    if (rec_pos_xyz.x != 0.0 || rec_pos_xyz.y != 0.0 || rec_pos_xyz.z != 0.0)
	{

		north.x=-cos(rec_pos_llh.lon)*sin(rec_pos_llh.lat);
		north.y=-sin(rec_pos_llh.lon)*sin(rec_pos_llh.lat);
		north.z= cos(rec_pos_llh.lat);
		east.x=-sin(rec_pos_llh.lon);
		east.y= cos(rec_pos_llh.lon);
		east.z=0.0;
		up.x=cos(rec_pos_llh.lon)*cos(rec_pos_llh.lat);
		up.y=sin(rec_pos_llh.lon)*cos(rec_pos_llh.lat);
		up.z=sin(rec_pos_llh.lat);
		

		xls =result.x-rec_pos_xyz.x;
		yls =result.y-rec_pos_xyz.y;
		zls =result.z-rec_pos_xyz.z;
		range1=sqrt(xls*xls+yls*yls+zls*zls);
		tdot=(up.x*xls+up.y*yls+up.z*zls)/range1;
   

		if ( tdot >= 1.00 ) satang=pi/2.0;
		else if ( tdot <= -1.00 ) satang=-pi/2.0;
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

*******************************************************************************/

void read_initial_data(void)
{
	int		id;
	SATVIS	dummy;

	for (id=1;id<=32;id++) gps_alm[id].inc=0.0;
	  
	status=cold_start;	
	read_ion_utc();	
	read_almanac();	
	thetime=time(NULL);
	dummy=satfind(0);
}

/*******************************************************************************
FUNCTION receiver_loc(void)
RETURNS  None.

PARAMETERS None.

PURPOSE
		To read in the last location of the receiver from file "curloc.dat"
		to help in warm and hot starts

*******************************************************************************/
char curloc_file[4096];
LLH receiver_loc(void)
{
	float latitude,longitude,height;
    char  text[10];
	LLH result;

	result.lat=0.0;      
	result.lon=0.0;
	result.hae=-6378137;

    // READ THE CURRENT LOCATION DATA FILE

	if ((in = fopen(curloc_file, "rt")) == NULL)
    {
		printf("Cannot open curloc.dat file.\n");
		status=cold_start;	
    }
	else
    {

		fscanf(in,"%10s",text);
		fscanf(in,"%f",&latitude);
		fscanf(in,"%10s",text);
		fscanf(in,"%f",&longitude);
		fscanf(in,"%10s",text);
		fscanf(in,"%f",&height);
		result.lat=latitude/57.296;	
		result.lon=longitude/57.296;
		result.hae=height;
		fclose(in);
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
int 	i4page, i5page;
void navmess(char prn,char ch)
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


    d_toe=clock_tow-gps_eph[prn].toe;

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
				if (chan[ch].message[(j+chan[ch].offset)%1500]==1)
				{
					scale++; 
				}
				j++;
			}
			sf[sfr][word]=scale;
		}
	}

	parity_check();
	if ((p_error[1]==0 || p_error[1]==0x200) && 
		(p_error[2]==0 || p_error[2]==0x200) && 
		(p_error[3]==0 || p_error[3]==0x200))
	{
		iodc=int(((sf[1][3] & 0x3) <<8 ) | ((sf[1][8] & 0xFF0000L) >>16));
		iode=int(sf[2][3]  >> 16);
		idoe=int(sf[3][10] >> 16);
		if (iode==idoe)// && ((iode!=gps_eph[prn].iode) || (iodc!=gps_eph[prn].iodc)))
						//modified by daicy
		{
			iweek= int(sf[1][3] >> 14);
			iura=int(( sf[1][3] & 0xF00 ) >> 8);
			ihealth=int(( sf[1][3] & 0xFC ) >> 2);
			itgd=int(sf[1][7] & 0xFF);
			itoc=int(sf[1][8] & 0xFFFF);
			iaf2=int(sf[1][9] >> 16);
			iaf1=int(sf[1][9] & 0xFFFF);
			iaf0=sf[1][10] >> 2;
			if (bit_test_l(iaf0,22)) iaf0=iaf0 | 0xFFC00000L;
			
			//   subframe 2
			icrs=int(sf[2][3] & 0xFFFF);
			idn=int(sf[2][4] >> 8);
			im0=((sf[2][4] & 0xFF) << 24) | sf[2][5];
			icuc=int(sf[2][6] >>8);
			ie=((sf[2][6] & 0xFF) << 24) | sf[2][7];
			icus=int(sf[2][8] >> 8);
			isqra=(((sf[2][8] & 0xFF) << 24) | sf[2][9]);
			itoe=int(sf[2][10] >> 8);
			
			// subframe 3
			icic=int(sf[3][3] >> 8);
			icis=int(sf[3][5] >> 8);
			inc0=((sf[3][5] & 0xFF) << 24) | sf[3][6];
			iomega0=((sf[3][3] & 0xFF) << 24) | sf[3][4];
			icrc=int(sf[3][7] >> 8);
			iw=((sf[3][7] & 0xFF) << 24) | sf[3][8];
			iomegadot=sf[3][9];
			if (bit_test_l(iomegadot,24)) iomegadot=iomegadot | 0xFF000000L;
			idot=int((sf[3][10] & 0xFFFC) >> 2);
			if (bit_test_l(idot,14)) idot=idot | 0xC000;

			r_sqra=isqra*c_2m19;
			r_inc0=inc0*c_2m31*pi;
			r_ety=ie*c_2m33;

			if ((r_inc0<1.05 && r_inc0>0.873) && (r_sqra>5100.0 && r_sqra<5200.0) &&
				(r_ety <.05 && r_ety>0.0))
			{
				gps_eph[prn].valid=1;
				gps_eph[prn].iode=iode;
				gps_eph[prn].iodc=iodc;
				gps_eph[prn].week=iweek;
				gps_eph[prn].ura=iura;
				gps_eph[prn].health=ihealth;
				gps_eph[prn].tgd=itgd*c_2m31;
				gps_eph[prn].toc=itoc*16.0;
				gps_eph[prn].af2=iaf2*c_2m55;
				gps_eph[prn].af1=iaf1*c_2m43;
				gps_eph[prn].af0=iaf0*c_2m31;
				gps_eph[prn].crs=icrs*c_2m5;
				gps_eph[prn].dn=idn*c_2m43*pi;
				gps_eph[prn].ma=im0*c_2m31*pi;
				gps_eph[prn].cuc=icuc*c_2m29;
				gps_eph[prn].ety=r_ety;
				gps_eph[prn].cus=icus*c_2m29;
				gps_eph[prn].sqra=r_sqra;
				gps_eph[prn].wm=19964981.84/pow(r_sqra,3);
				gps_eph[prn].toe=itoe*c_2p4;
				gps_eph[prn].cic=icic*c_2m29;
				gps_eph[prn].cis=icis*c_2m29;
				gps_eph[prn].inc0=r_inc0;
				gps_eph[prn].w0=iomega0*c_2m31*pi;
				gps_eph[prn].crc=icrc*c_2m5;
				gps_eph[prn].w=iw*c_2m31*pi;
				gps_eph[prn].omegadot=iomegadot*c_2m43*pi;
				gps_eph[prn].idot=idot*c_2m43*pi;

				  doublesv=(double)prn;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=(double)iweek;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=0.0;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].tgd;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=0;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].toc;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].af2;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].af1;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].af0;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=0;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].crs;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].dn;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].ma;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].cuc;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].ety;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].cus;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].sqra;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].toe;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].cic;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].w0;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].cis;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].inc0;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].crc;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].w;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].omegadot;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  doublesv=gps_eph[prn].idot;
				  fwrite(&doublesv, sizeof(double),1,fpeph);

				  fflush(fpeph);

				if (out_debug)  write_Debug_ephemeris(prn);
			} 
		    else if (gps_eph[prn].valid==1 && d_toe>7200.0) gps_eph[prn].valid=0;
		} 
	}

	if ((p_error[4]==0 || p_error[4] == 0x200) && 
		(p_error[5]==0 || p_error[5] == 0x200) && almanac_valid==0 && almanac_flag==0)
	{
		almanac_flag=1;

		i4data= int(sf[4][3] >> 22);	
		i4p= int((sf[4][3] & 0x3F0000L) >> 16);	
		if (i4p != i4page && i4data==1)	
		{								
			i4page=i4p;
			if (i4page > 24 && i4page < 33)
			{
				isv=i4page;
				gps_alm[isv].week=gps_week%1024;
				iae=int(sf[4][3] & 0x00FFFFL);
				gps_alm[isv].ety=iae*c_2m21;
				iatoa=int(sf[4][4] >> 16);
				gps_alm[isv].toa=iatoa*c_2p12;
				iadeli=sf[4][4] & 0x00FFFFL;
				if (bit_test_l(iadeli,16)) iadeli=iadeli | 0xFFFF0000L;
				gps_alm[isv].inc=(iadeli*c_2m19+0.3)*pi;
				iomegad=int(sf[4][5] >> 8);
				gps_alm[isv].rra=iomegad*c_2m38*pi;
				gps_alm[isv].health=int(sf[4][5] & 0x0000FF);
				iasqr=sf[4][6];
				gps_alm[isv].sqa=iasqr*c_2m11;
				if (gps_alm[isv].sqa>0.0) 
					gps_alm[isv].w=19964981.84/pow(gps_alm[isv].sqa,3);
				iaomega0=sf[4][7];
				if (bit_test_l(iaomega0,24)) iaomega0=iaomega0 | 0xFF000000L;
				gps_alm[isv].lan=iaomega0*c_2m23*pi;
				iaw=sf[4][8];
				if (bit_test_l(iaw,24)) iaw=iaw | 0xFF000000L;
				gps_alm[isv].aop=iaw*c_2m23*pi;
				iam0=sf[4][9];
				if (bit_test_l(iam0,24)) iam0=iam0 | 0xFF000000L;
				gps_alm[isv].ma=iam0*c_2m23*pi;
				iaaf0=(sf[4][10] >> 13) | ((sf[4][10] & 0x1C)>>2);
				if (bit_test_l(iaaf0,11)) iaaf0=iaaf0 | 0xFFFFF800L;
				gps_alm[isv].af0=iaaf0*c_2m20;
				iaaf1=(sf[4][10] | 0xFFE0) >> 5;
				if (bit_test_l(iaaf1,11)) iaaf1=iaaf1 | 0xFFFFF800L;
				gps_alm[isv].af1=iaaf1*c_2m38;
			}

			else if ( i4page == 55 )
			{
				gps_alm[prn].text_message[0]=char((sf[4][3] & 0x00FF00) >> 8);
				gps_alm[prn].text_message[1]=char( sf[4][3] & 0x0000FF);
				for ( k=1;k<=7;k++)
				{
				  gps_alm[prn].text_message[3*k-1]= char(sf[4][k+3] >> 16);
				  gps_alm[prn].text_message[3*k  ]= char((sf[4][k+3] & 0x00FF00) >> 8);
				  gps_alm[prn].text_message[3*k+1]= char(sf[4][k+3] & 0x0000FF);
				}
			}

			else if ( i4page == 56 )
			{
				ial0=int((sf[4][3] & 0x00FF00) >> 8);
				al0=ial0*c_2m30;
				ial1= int(sf[4][3] & 0x0000FF);
				al1=ial1*c_2m27;
				ial2= int(sf[4][4] >> 16);
				al2=ial2*c_2m24;
				ial3=int((sf[4][4] & 0x00FF00) >> 8);
				al3=ial3*c_2m24;
				ibt0= int(sf[4][4] & 0x0000FF);
				b0=ibt0*2048.;
				ibt1= int(sf[4][5] >> 16);
				b1=ibt1*16384.;
				ibt2=int((sf[4][5] & 0x00FF00) >> 8);
				b2=ibt2*65536.;
				ibt3= int(sf[4][5] & 0x00FF);
				b3=ibt3*65536.;
				ia1=   sf[4][6];
				if (bit_test_l(ia1,24)) ia1=ia1 | 0xFF000000L;
				a1=ia1*c_2m50;
				ia0=  (sf[4][7] << 8) | (sf[4][8] >> 16);
				a0=ia0*c_2m30;
				itot=  int((sf[4][8] & 0x00FF00) >> 8);
				tot=itot*4096;
				iWNt=  int(sf[4][8] & 0xFF);
				WNt=iWNt;
				idtls=  int(sf[4][10] >> 16);
				if (idtls >128) idtls=idtls |0xFF00;
				dtls=idtls;
				iWNlsf=int((sf[4][9] & 0x00FF00) >> 8);
				WNlsf=iWNlsf;
				iDN   = int(sf[4][9] & 0x0000FF);
				DN=iDN;
				idtlsf= int(sf[4][9] >> 16);
				if (idtlsf >128) idtlsf=idtlsf |0xFF00;
				dtlsf=idtlsf;
			}

			else if ( i4page == 63 )
			{
				ASV[1]= int((sf[4][3] & 0x00F000) >>12);
				ASV[2]= int((sf[4][3] & 0x000F00) >>8);
				ASV[3]= int((sf[4][3] & 0x0000F0) >>4);
				ASV[4]= int( sf[4][3] & 0x00000F);
				ASV[5]= int( sf[4][4] >>20);
				ASV[6]= int((sf[4][4] & 0x0F0000L) >>16);
				ASV[7]= int((sf[4][4] & 0x00F000) >>12);
				ASV[8]= int((sf[4][4] & 0x000F00) >> 8);
				ASV[9]= int((sf[4][4] & 0x0000F0) >> 4);
				ASV[10]=int(sf[4][4] & 0x00000F);
				ASV[11]=int(sf[4][5] >>20);
				ASV[12]=int((sf[4][5] & 0x0F0000L) >>16);
				ASV[13]=int((sf[4][5] & 0x00F000) >>12);
				ASV[14]=int((sf[4][5] & 0x000F00) >> 8);
				ASV[15]=int((sf[4][5] & 0x0000F0) >> 4);
				ASV[16]=int(sf[4][5] & 0x00000F);
				ASV[17]=int(sf[4][6] >>20);
				ASV[18]=int((sf[4][6] & 0x0F0000L) >>16);
				ASV[19]=int((sf[4][6] & 0x00F000) >>12);
				ASV[20]=int((sf[4][6] & 0x000F00) >> 8);
				ASV[21]=int((sf[4][6] & 0x0000F0) >> 4);
				ASV[22]=int(sf[4][6] & 0x00000F);
				ASV[23]=int(sf[4][7] >>20);
				ASV[24]=int((sf[4][7] & 0x0F0000L) >>16);
				ASV[25]=int((sf[4][7] & 0x00F000) >>12);
				ASV[26]=int((sf[4][7] & 0x000F00) >> 8);
				ASV[27]=int((sf[4][7] & 0x0000F0) >> 4);
				ASV[28]=int( sf[4][7] & 0x00000F);
				ASV[29]=int( sf[4][8] >>20);
				ASV[30]=int((sf[4][8] & 0x0F0000L) >>16);
				ASV[31]=int((sf[4][8] & 0x00F000) >>12);
				ASV[32]=int((sf[4][8] & 0x000F00) >> 8);
				SVh[25]=int(sf[4][8] & 0x00003F);
				if( SVh[25]==0x3f) gps_alm[25].inc=0.0;
				SVh[26]=int(sf[4][9] >>18);
				if( SVh[26]==0x3f) gps_alm[26].inc=0.0;
				SVh[27]=int((sf[4][9] & 0x03F000L) >>12);
				if( SVh[27]==0x3f) gps_alm[27].inc=0.0;
				SVh[28]=int((sf[4][9] & 0x000FC0) >>6);
				if( SVh[28]==0x3f) gps_alm[28].inc=0.0;
				SVh[29]= int(sf[4][9] & 0x00003F);
				if( SVh[29]==0x3f) gps_alm[29].inc=0.0;
				SVh[30]= int(sf[4][10] >>18);
				if( SVh[30]==0x3f) gps_alm[30].inc=0.0;
				SVh[31]=int((sf[4][10]& 0x03F000L) >>12);
				if( SVh[31]==0x3f) gps_alm[31].inc=0.0;
				SVh[32]=int((sf[4][10]& 0x000FC0) >>6);
				if( SVh[32]==0x3f) gps_alm[32].inc=0.0;
			}
		}
		
		i5data=int(sf[5][3] >> 22);	// DATA ID
		i5p=int((sf[5][3] & 0x3F0000L) >> 16);	// PAGE/SV ID
		chan[ch].page5=i5p;
		if (i5page != i5p && i5data==1)
		{
			i5page=i5p;


			if ( i5page == 51 )
			{
				iatoa=int((sf[5][3] & 0xFF00) >>8);
				SVh[1]=int(sf[5][4] >>18);
				if( SVh[1]==0x3f) gps_alm[1].inc=0.0;
				SVh[2]=int((sf[5][4] & 0x03F000L)>>12);
				if( SVh[2]==0x3f) gps_alm[2].inc=0.0;
				SVh[3]=int((sf[5][4] & 0x000FC0)>>6);
				if( SVh[3]==0x3f) gps_alm[3].inc=0.0;
				SVh[4]= int(sf[5][4] & 0x00003F);
				if( SVh[4]==0x3f) gps_alm[4].inc=0.0;
				SVh[5]= int(sf[5][5] >>18);
				if( SVh[5]==0x3f) gps_alm[5].inc=0.0;
				SVh[6]=int((sf[5][5] & 0x03F000L)>>12);
				if( SVh[6]==0x3f) gps_alm[6].inc=0.0;
				SVh[7]=int((sf[5][5] & 0x000FC0)>>6);
				if( SVh[7]==0x3f) gps_alm[7].inc=0.0;
				SVh[8]= int(sf[5][5] & 0x00003F);
				if( SVh[8]==0x3f) gps_alm[8].inc=0.0;
				SVh[9]= int(sf[5][6] >>18);
				if( SVh[9]==0x3f) gps_alm[9].inc=0.0;
				SVh[10]=int((sf[5][6] & 0x03F000L)>>12);
				if( SVh[10]==0x3f) gps_alm[10].inc=0.0;
				SVh[11]=int((sf[5][6] & 0x000FC0)>>6);
				if( SVh[11]==0x3f) gps_alm[11].inc=0.0;
				SVh[12]= int(sf[5][6] & 0x00003F);
				if( SVh[12]==0x3f) gps_alm[12].inc=0.0;
				SVh[13]= int(sf[5][7] >>18);
				if( SVh[13]==0x3f) gps_alm[13].inc=0.0;
				SVh[14]=int((sf[5][7] & 0x03F000L)>>12);
				if( SVh[14]==0x3f) gps_alm[14].inc=0.0;
				SVh[15]=int((sf[5][7] & 0x000FC0)>>6);
				if( SVh[15]==0x3f) gps_alm[15].inc=0.0;
				SVh[16]= int(sf[5][7] & 0x00003F);
				if( SVh[16]==0x3f) gps_alm[16].inc=0.0;
				SVh[17]= int(sf[5][8] >>18);
				if( SVh[17]==0x3f) gps_alm[17].inc=0.0;
				SVh[18]=int((sf[5][8] & 0x03F000L)>>12);
				if( SVh[18]==0x3f) gps_alm[18].inc=0.0;
				SVh[19]=int((sf[5][8] & 0x000FC0)>>6);
				if( SVh[19]==0x3f) gps_alm[19].inc=0.0;
				SVh[20]= int(sf[5][8] & 0x00003F);
				if( SVh[20]==0x3f) gps_alm[20].inc=0.0;
				SVh[21]= int(sf[5][9] >>18);
				if( SVh[21]==0x3f) gps_alm[21].inc=0.0;
				SVh[22]=int((sf[5][9] & 0x03F000L)>>12);
				if( SVh[22]==0x3f) gps_alm[22].inc=0.0;
				SVh[23]=int((sf[5][9] & 0x000FC0)>>6);
				if( SVh[23]==0x3f) gps_alm[23].inc=0.0;
				SVh[24]= int(sf[5][9] & 0x00003F);
				if( SVh[24]==0x3f) gps_alm[24].inc=0.0;
			}

			else
			{
				isv=i5page;
				gps_alm[isv].week=gps_week%1024;
				iae=int(sf[5][3] & 0xFFFF);
				gps_alm[isv].ety=iae*c_2m21;
				iatoa=int(sf[5][4] >> 16);
				gps_alm[isv].toa=iatoa*4096.0;
				iadeli=int(sf[5][4] & 0xFFFF);
				gps_alm[isv].inc=(iadeli*c_2m19+0.3)*pi;
				iaomegad=int(sf[5][5] >> 8);
				gps_alm[isv].rra=iaomegad*c_2m38*pi;
				gps_alm[isv].health=int(sf[5][5] & 0xFF);
				iasqr=sf[5][6];
				gps_alm[isv].sqa=iasqr*c_2m11;
				if (gps_alm[isv].sqa>0.0) gps_alm[isv].w=19964981.84/pow(gps_alm[isv].sqa,3);
				iaomega0=sf[5][7];
				if (bit_test_l(iaomega0,24)) iaomega0=iaomega0 | 0xFF000000L;
				gps_alm[isv].lan=iaomega0*c_2m23*pi;
				iaw=sf[5][8];
				if (bit_test_l(iaw,24)) iaw=iaw | 0xFF000000L;
				gps_alm[isv].aop=iaw*c_2m23*pi;
				iam0=sf[5][9];
				if (bit_test_l(iam0,24)) iam0=iam0 | 0xFF000000L;
				gps_alm[isv].ma=iam0*c_2m23*pi;
				iaaf0=int((sf[5][10] >> 13) | ((sf[5][10] & 0x1C)>>2));
				if (bit_test_l(iaaf0,11)) iaaf0=iaaf0 | 0xF800;
				gps_alm[isv].af0=iaaf0*c_2m20;
				iaaf1=int((sf[5][10] & 0xFFE0) >> 5);
				if (bit_test_l(iaaf1,11)) iaaf1=iaaf1 | 0xF800;
				gps_alm[isv].af1=iaaf1*c_2m38;
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

inline int  bit_test_l(unsigned long data,char bit_n)
{
	int result;
	result= (data & test_l[bit_n]) ? 1 : 0;
	return(result);
}

/*******************************************************************************
FUNCTION parity_check(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  checks the parity of the 5 subframes of the nav message

*******************************************************************************/

void  parity_check(void)
{
	long pb1=0x3b1f3480L,pb2=0x1d8f9a40L,pb3=0x2ec7cd00L;
	long pb4=0x1763e680L,pb5=0x2bb1f340L,pb6=0x0b7a89c0L;
	int  parity,m_parity;
	char d29=0,d30=0,sfm,word,b_1,b_2,b_3,b_4,b_5,b_6;
	int err_bit;

	for (sfm=1;sfm<=5;sfm++)
	{
		p_error[sfm]=0;
		for (word=1;word<=10;word++)
		{
			m_parity=int(sf[sfm][word] & 0x3f);
			b_1=exor(d29,sf[sfm][word] & pb1) << 5;
			b_2=exor(d30,sf[sfm][word] & pb2) << 4;
			b_3=exor(d29,sf[sfm][word] & pb3) << 3;
			b_4=exor(d30,sf[sfm][word] & pb4) << 2;
			b_5=exor(0,sf[sfm][word] & pb5) << 1;
			b_6=exor(d29^d30,sf[sfm][word] & pb6);
			parity=b_1+b_2+b_3+b_4+b_5+b_6;

			err_bit = (parity != m_parity) ? 1: 0;
			p_error[sfm]=(p_error[sfm] << 1) + err_bit;
			if (d30==1) sf[sfm][word]=0x03fffffc0L & ~sf[sfm][word];
			sf[sfm][word]=sf[sfm][word]>>6;

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
int exor(char bit, long parity)
{
	char i;
	int result;
	result=0;
	for (i=7;i<=30;i++)
	{
		if (bit_test_l(parity,i)) result++;
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

LLH ecef_to_llh(XYZ pos)
{
	double p,n,thet,esq,epsq;
	LLH result;

	p=sqrt(pos.x*pos.x+pos.y*pos.y);
	thet=atan(pos.z*a/(p*b));
	esq =1.0-b*b/(a*a);
	epsq=a*a/(b*b)-1.0;
	result.lat=atan((pos.z+epsq*b*pow(sin(thet),3))/(p-esq*a*pow(cos(thet),3)));
	result.lon=atan2(pos.y,pos.x);
	n=a*a/sqrt(a*a*cos(result.lat)*cos(result.lat) +
		b*b*sin(result.lat)*sin(result.lat));
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

XYZ llh_to_ecef(LLH pos)
{
	double n;
	XYZ result;
	n=a*a/sqrt(a*a*cos(pos.lat)*cos(pos.lat)+b*b*sin(pos.lat)*sin(pos.lat));
	result.x=(n+pos.hae)*cos(pos.lat)*cos(pos.lon);
	result.y=(n+pos.hae)*cos(pos.lat)*sin(pos.lon);
	result.z=(b*b/(a*a)*n+pos.hae)*sin(pos.lat);
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

int		daicy_temp_cnt;
extern double		m_time[3];;
extern FILE *daicy_file_pr;
extern FILE *daicy_file_pos;
extern FILE   *out_trtime;

PVT  pos_vel_time(int nsl)
{
	double	dd[5][5],r,ms[5][13],pm[5][13],bm[13],br[5],correct_mag,x,y,z,t;
	double	a1,b1,c1,d1,e1,f1,g1,h1,i1,j1,k1,l1,m1,n1,o1,p1,denom,alpha;
	int		i,j,k,nits;
	PVT		result;

	double	daicy_delta_pr[13];
	double	daicy_x,daicy_y,daicy_z;

	nits=0;
	t=0.0;

	x=rec_pos_xyz.x;
	y=rec_pos_xyz.y;
	z=rec_pos_xyz.z;

	daicy_temp_cnt = 0;

	do
	{
		daicy_temp_cnt++;
		for (i=1;i<=nsl;i++)
		{
			alpha=(t-dt[i])*omegae;
			r=sqrt(pow(track_sat[i].x*cos(alpha)-track_sat[i].y*sin(alpha)-x,2)+
				 pow(track_sat[i].y*cos(alpha)+track_sat[i].x*sin(alpha)-y,2)+
				 pow(track_sat[i].z-z,2));
			bm[i]=r-(dt[i]-t)*c;
			ms[1][i]=(track_sat[i].x*cos(alpha)-track_sat[i].y*sin(alpha)-x)/r;
			ms[2][i]=(track_sat[i].y*cos(alpha)+track_sat[i].x*sin(alpha)-y)/r;
			ms[3][i]=(track_sat[i].z-z)/r;
			ms[4][i]=1.0;

			//输出伪距比对
			if(daicy_temp_cnt>0)
			{
				#if 1
				//(39,116,18000)
				daicy_x=track_sat[i].x*cos(alpha)-track_sat[i].y*sin(alpha)-(-2181911.708);
				daicy_y=track_sat[i].y*cos(alpha)+track_sat[i].x*sin(alpha)-4473581.957;
				daicy_z=track_sat[i].z-4003644.790;
				#else
				//(39,116,100)
				daicy_x=track_sat[i].x*cos(alpha)-track_sat[i].y*sin(alpha)-(-2175813.565);
				daicy_y=track_sat[i].y*cos(alpha)+track_sat[i].x*sin(alpha)-4461078.911;
				daicy_z=track_sat[i].z-3992379.955;
				#endif
				
				daicy_delta_pr[i]=sqrt(daicy_x*daicy_x+daicy_y*daicy_y+daicy_z*daicy_z);
				fprintf(daicy_file_pr,"m_time[1]: %.9f	%d	dt[i]: %.9f	R: %.2f	",m_time[1],i,dt[i],daicy_delta_pr[i]);
				
				daicy_delta_pr[i]=sqrt(daicy_x*daicy_x+daicy_y*daicy_y+daicy_z*daicy_z)-dt[i]*c;
				fprintf(daicy_file_pr,"trans_t: %.2f	delta_t: %.2f	",dt[i]*c,daicy_delta_pr[i]);
				
				if(i==nsl)
					fprintf(daicy_file_pr,"		diff		%.2f	%.2f	%.2f	%.2f	%.2f	%.2f	%.2f	\n",
										 daicy_delta_pr[2]-daicy_delta_pr[1],daicy_delta_pr[3]-daicy_delta_pr[1],
										 daicy_delta_pr[4]-daicy_delta_pr[1],daicy_delta_pr[5]-daicy_delta_pr[1],
										 daicy_delta_pr[6]-daicy_delta_pr[1],daicy_delta_pr[7]-daicy_delta_pr[1],
										 daicy_delta_pr[8]-daicy_delta_pr[1]);
			}
		}
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

		denom=(k1*p1-l1*o1)*(a1*f1-b1*e1) + (l1*n1-j1*p1)*(a1*g1-c1*e1) +
				(j1*o1-k1*n1)*(a1*h1-d1*e1) + (l1*m1-i1*p1)*(c1*f1-b1*g1) +
				(i1*o1-k1*m1)*(d1*f1-b1*h1) + (i1*n1-j1*m1)*(c1*h1-d1*g1);
		
		dd[1][1]=f1*(k1*p1-l1*o1)+g1*(l1*n1-j1*p1)+h1*(j1*o1-k1*n1);
		dd[1][2]=e1*(l1*o1-k1*p1)+g1*(i1*p1-l1*m1)+h1*(k1*m1-i1*o1);
		dd[1][3]=e1*(j1*p1-n1*l1)-i1*(f1*p1-n1*h1)+m1*(f1*l1-j1*h1);
		dd[1][4]=e1*(n1*k1-j1*o1)+i1*(f1*o1-n1*g1)+m1*(j1*g1-f1*k1);
		dd[2][1]=dd[1][2];
		dd[2][2]=a1*(k1*p1-l1*o1)+c1*(l1*m1-i1*p1)+d1*(i1*o1-k1*m1);
		dd[2][3]=a1*(l1*n1-j1*p1)+i1*(b1*p1-n1*d1)+m1*(j1*d1-b1*l1);
		dd[2][4]=a1*(j1*o1-n1*k1)-i1*(b1*o1-n1*c1)+m1*(b1*k1-c1*j1);
//		dd[3][1]=b1*(g1*p1-h1*o1)-f1*(c1*p1-o1*d1)+n1*(c1*h1-d1*g1);
		dd[3][1]=dd[1][3];
//		dd[3][2]=a1*(o1*h1-g1*p1)+e1*(c1*p1-o1*d1)+m1*(d1*g1-c1*h1);
		dd[3][2]=dd[2][3];
		dd[3][3]=a1*(f1*p1-h1*n1)+b1*(h1*m1-e1*p1)+d1*(e1*n1-f1*m1);
		dd[3][4]=a1*(n1*g1-f1*o1)+e1*(b1*o1-c1*n1)+m1*(c1*f1-b1*g1);
//		dd[4][1]=b1*(h1*k1-g1*l1)+f1*(c1*l1-d1*k1)+j1*(d1*g1-c1*h1);
		dd[4][1]=dd[1][4];
//		dd[4][2]=a1*(g1*l1-h1*k1)-e1*(c1*l1-d1*k1)+i1*(c1*h1-d1*g1);
		dd[4][2]=dd[2][4];
//		dd[4][3]=a1*(j1*h1-f1*l1)+e1*(b1*l1-d1*j1)+i1*(d1*f1-b1*h1);
		dd[4][3]=dd[3][4];
		dd[4][4]=a1*(f1*k1-g1*j1)+b1*(g1*i1-e1*k1)+c1*(e1*j1-f1*i1);
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
			t -= br[4]/c;	
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
		alpha=(dt[i]-t)*omegae;


		r=sqrt(pow(track_sat[i].x*cos(alpha)-track_sat[i].y*sin(alpha)-x,2)+
			pow(track_sat[i].y*cos(alpha)+track_sat[i].x*sin(alpha)-y,2)+
			pow(track_sat[i].z-z,2));

		bm[i]=((track_sat[i].x*cos(alpha)-track_sat[i].y*sin(alpha)-x)*d_sat[i].x+
			(track_sat[i].y*cos(alpha)+track_sat[i].x*sin(alpha)-y)*d_sat[i].y+
			(track_sat[i].z-z)*d_sat[i].z)/r+meas_dop[i]*lambda;
	}


	for (i=1;i<=4;i++)
	{
		br[i]=0.0;
		for (j=1;j<=nsl;j++)br[i]+=bm[j]*pm[i][j];
	}

	result.xv=br[1]+y*omegae;  
	result.yv=br[2]-x*omegae;  
	result.zv=br[3];
	result.df=br[4]/c*1000000.0; 
	return(result);
}

PVT kalman_pos_vel(int nsl)
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
	PVT		result;

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

	x = rec_pos_xyz.x;
	y = rec_pos_xyz.y;
	z = rec_pos_xyz.z;
	old_receiver(0) = /*0;//*/ x;
	old_receiver(1) = /*0;// */rpvt.xv;
	old_receiver(2) = /*0;//*/ y;
	old_receiver(3) = /*0;// */ rpvt.yv;
	old_receiver(4) = /*0;//*/ z;
	old_receiver(5) = /*0;// */ rpvt.zv;
	old_receiver(6) = t;
	old_receiver(7) = 0;


	//do
	//{

	for (int j = 0; j < nsl; j++)  //vector转换成eigen
	{
		dx(j) = track_sat[j + 1].x;
		dy(j) = track_sat[j + 1].y;
		dz(j) = track_sat[j + 1].z;
		satVx(j) = d_sat[j + 1].x;
		satVy(j) = d_sat[j + 1].y;
		satVz(j) = d_sat[j + 1].z;
		range_Final_1(j) = dt[j + 1];
		dop(j) = meas_dop[j + 1];
	}

	Pk = A * 1000 * P0*(A.transpose()) + Q;
	detaX_t = A*old_receiver;// .transpose();

	/*得到观测矩阵H*/

	for (int k = 0; k < nsl; k++)
	{
		range_Final_2(k) = lambda* dop(k);

		alpha(k) = (range_Final_1(k)*c - detaX_t(6))*omegae / c;
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
		Y(2 * (i)) = c*range_Final_1(i);
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
	result.dt = old_receiver(6) / c;
	result.df = old_receiver(7) / c*1000000.0;

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
void  dops( int nsl)
{
	double r,xls,yls,zls;
	int i;
	double **H, **G;
	
	H = dmatrix(nsl,4);
	G = dmatrix(4,4);
	
	receiver.north.x=-cos(rec_pos_llh.lon)*sin(rec_pos_llh.lat);
	receiver.north.y=-sin(rec_pos_llh.lon)*sin(rec_pos_llh.lat);
	receiver.north.z= cos(rec_pos_llh.lat);
	receiver.east.x=-sin(rec_pos_llh.lon);
	receiver.east.y= cos(rec_pos_llh.lon);
	receiver.up.x=cos(rec_pos_llh.lon)*cos(rec_pos_llh.lat);
	receiver.up.y=sin(rec_pos_llh.lon)*cos(rec_pos_llh.lat);
	receiver.up.z=sin(rec_pos_llh.lat);
	for (i=1;i<=nsl;i++)
	{
		//   Compute line of sight vectors
		xls=track_sat[i].x-rec_pos_xyz.x;
		yls=track_sat[i].y-rec_pos_xyz.y;
		zls=track_sat[i].z-rec_pos_xyz.z;

		r=sqrt(xls*xls+yls*yls+zls*zls);

		H[i-1][0]=(xls*receiver.north.x+yls*receiver.north.y+zls*receiver.north.z)/r;
		H[i-1][1]=(xls*receiver.east.x+yls*receiver.east.y)/r;
		H[i-1][2]=(xls*receiver.up.x+yls*receiver.up.y+zls*receiver.up.z)/r;
		H[i-1][3]=1.0;
	}

	mult_ATB(H,nsl,4,H,nsl,4,G);
	matinv(G,4);
	hdop=sqrt(G[0][0]+G[1][1]);
	vdop=sqrt(G[2][2]);
	tdop=sqrt(G[3][3]);
	pdop=sqrt(G[0][0]+G[1][1]+G[2][2]);
	gdop=sqrt(G[0][0]+G[1][1]+G[2][2]+G[3][3]);
	free_dmatrix(G,4);
	free_dmatrix(H,nsl);
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

double tropo_iono(char ch,double az,double el,double gps_time)
{
	double d_Trop,alt_factor;
	double d_Ion,psi,phi,lambdai,phim,per,x,F,amp,t;

	if (current_loc.hae>200000.0) alt_factor=0.0;
	else if (current_loc.hae<0.0) alt_factor=1.0;
	else alt_factor=exp(-current_loc.hae*1.33e-4);

	d_Trop = (m_tropo==1) ? 2.47/(sin(el)+.0121)*alt_factor/c : 0.0;
	chan[ch].Tropo=d_Trop;

	if (m_iono==1)
	{
		psi=0.0137/(el/pi+0.11)-.022;
		phi=current_loc.lat/pi+psi*cos(az);
		if (phi > 0.416) phi= 0.416;
		else if (phi <-0.416 ) phi=-0.416;
		lambdai=current_loc.lon/pi+psi*sin(az)/cos(phi*pi);
		t=43200.0*lambdai+gps_time-int((43200.0*lambdai+gps_time)/86400.)*86400.;
		if (t<0.0) t=t+86400.0;
		phim=phi+0.064*cos((lambdai-1.617)*pi);
		
		//  If available from the nav message use its Ionosphere model
		if (b0 != 0.0 && al0 != 0.0)
		{
			per=b0+b1*phim+b2*phim*phim+b3*phim*phim*phim;
			amp=al0+al1*phim+al2*phim*phim+al3*phim*phim*phim;
		}
		else // else try this set of default iono model parameters
		{
			per=141312.0-32768.0*phim-131072.0*phim*phim-65536.0*phim*phim*phim;
			amp=3.46e-8+7.45e-9*phim-1.19e-7*phim*phim+5.96e-8*phim*phim*phim;
		}
		if ( per <72000.0 ) per=72000.0;
		x=2.*pi*(t-50400.)/per;
		F=1.0+16.0*pow(0.53-el/pi,3);
		if ( amp < 0.0 ) amp=0.0;
		if (fabs(x) < 1.5707)  d_Ion=F*(5.0e-9+amp*(1.0-x*x/2.+x*x*x*x/24.0));
		else                   d_Ion=F*5.0e-9;
	}
	else d_Ion=0.0;
	chan[ch].Iono=d_Ion;
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
char ion_utc_file[4096];
void  read_ion_utc(void)
{
	if ((in = fopen(ion_utc_file, "rt")) == NULL)
	{
		printf("Cannot open ion_utc.dat file.\n");
	}
	else
	{
		handle=_fileno(in);
		while (!_eof(handle))
		{
			//ion
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&al0);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&al1);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&al2);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&al3);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&b0);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&b1);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&b2);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&b3);
			//utc
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&a0);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&a1);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&dtls);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&tot);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&WNt);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&WNlsf);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&DN);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%lf",&dtlsf);
		}
		fclose(in);
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
char current_alm_file[4096];
void read_almanac(void)
{
	int   id,health,week;
	float eccen,rinc,rras,sqra;
	float ratoa,aopg,rma,af0,af1,toa;

	if ((in = fopen(current_alm_file, "rt")) == NULL)
	{
		printf("Cannot open currrent.rcv file.\n");
		for (id=1;id<=32;id++)
		{
			gps_alm[id].week=gps_week-1;
			gps_alm[id].inc=1.0;
		}
	}
	else
	{
		status=warm_start;
		handle=_fileno(in);
		while (!_eof(handle))
		{
			fscanf(in,"%46c",&header);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%d",&id);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%i",&health);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&eccen);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&toa);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&rinc);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&rras);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&sqra);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&ratoa);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&aopg);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&rma);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&af0);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%f",&af1);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%i",&week);
			fscanf(in,"%c",&trailer);
			gps_alm[id].health=health;
			gps_alm[id].week=week;
			gps_alm[id].toa=toa;
			gps_alm[id].ety=eccen;
			gps_alm[id].toa=toa;
			gps_alm[id].inc=rinc;
			gps_alm[id].rra=rras;
			gps_alm[id].sqa=sqra;
			gps_alm[id].lan=ratoa;
			gps_alm[id].aop=aopg;
			gps_alm[id].ma=rma;
			gps_alm[id].af0=af0;
			gps_alm[id].af1=af1;
			gps_alm[id].sat_file=0;
			if (gps_alm[id].sqa>0.0) gps_alm[id].w=19964981.84/pow(gps_alm[id].sqa,3);
		}
		fclose(in);
		alm_gps_week=week;
		alm_toa=toa;
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
char current_eph_file[4096];
void read_ephemeris()
{
	int   id,health,week,ura,iode,iodc;
	double  toc,toe;
	double crc,crs,cic,cis,cuc,cus,tgd,ety,inc0,omegadot,w0,w,ma,dn,idot;
	double daf0,daf1,daf2,esqra;
	double d_toe;
	FILE   *ephfile;
	int i;
	if ((ephfile = fopen(current_eph_file, "rt")) == NULL)
	{
		printf("Cannot open currrent.eph file.\n");
	}
	else
	{
		availablesv.availablesvnumber=0;
		i=0;

		while (!feof(ephfile))
		{
			fscanf(in,"%37c",&header);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%i",&id);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%i",&health);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%i",&week);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%i",&ura);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&toe);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%i",&iode);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&toc);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%i",&iodc);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&tgd);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&daf0);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&daf1);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&daf2);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&ety);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&inc0);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&idot);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&omegadot);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&esqra);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&dn);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&w0);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&w);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&ma);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&cuc);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&cus);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&crc);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&crs);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&cic);
			fscanf(in,"%27c",&ttext);
			fscanf(in,"%le",&cis);
			fscanf(in,"%c",&trailer);

			d_toe=clock_tow-toe;
			if (d_toe>302400.0) d_toe=d_toe-604800.0;
			else if (d_toe<-302400.0)d_toe=d_toe+604800.0;

			if ( fabs(d_toe)<7200.0)
			{
				gps_eph[id].valid=1;
				gps_eph[id].health=health;
				gps_eph[id].week=week;
				gps_eph[id].ura=ura;
				gps_eph[id].toe=toe;
				gps_eph[id].iode=iode;
				gps_eph[id].toc=toc;
				gps_eph[id].iodc=iodc;
				gps_eph[id].tgd=tgd;
				gps_eph[id].af0=daf0;
				gps_eph[id].af1=daf1;
				gps_eph[id].af2=daf2;
				gps_eph[id].ety=ety;
				gps_eph[id].inc0=inc0;
				gps_eph[id].idot=idot;
				gps_eph[id].omegadot=omegadot;
				gps_eph[id].sqra=esqra;
				gps_eph[id].dn=dn;
				gps_eph[id].w0=w0;
				gps_eph[id].w=w;
				gps_eph[id].ma=ma;
				gps_eph[id].cuc=cuc;
				gps_eph[id].cus=cus;
				gps_eph[id].crc=crc;
				gps_eph[id].crs=crs;
				gps_eph[id].cic=cic;
				gps_eph[id].cis=cis;
				if (gps_eph[id].sqra >0.0) gps_eph[id].wm=19964981.84/pow(gps_eph[id].sqra,3);
				if (out_debug)  write_Debug_ephemeris(id);

				availablesv.availablesvnumber ++;
	         	availablesv.availablesvprn[i] = id;
	         	i++;
			}
		} 
		fclose(in);
	}
}
char last_prn_file[4096];
void read_prn(void)
{
	int   i = 0,id;
	
	if ((fprn_in = fopen(last_prn_file, "rt")) == NULL)
	{
		printf("Cannot open lastime.prn file.\n");
	}
	else
	{
		for (i = 0;i <= chmax;i++)
		{
			fscanf(fprn_in,"%27c",&ttext);
			fscanf(fprn_in,"%d",&id);
		}
		fclose(fprn_in);
	}
}


void write_prn()
{
	int i;
	fprn_out=fopen(last_prn_file,"w+");
	for (i=0;i<=chmax;i++)
	{
		fprintf(fprn_out,"prn:                       %4d\n",chan[i].prn);

	}
	fclose(fprn_out);
}
/*******************************************************************************
FUNCTION write_almanac()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast almanac data to a file for later
	use.  In particular to support a warm or hot start.

*******************************************************************************/

void write_almanac()
{
	int i;
	out=fopen(current_alm_file,"w+");
	for (i=1;i<=32;i++)
	{
		if ( gps_alm[i].inc >0.0)
		{
			fprintf(out,"**** Week %4d almanac for PRN-%2d ***********\n",
				gps_alm[i].week%1024,i);
			fprintf(out,"ID:                         %3d\n",i);
			fprintf(out,"Health:                     %3d\n",gps_alm[i].health);
			fprintf(out,"Eccentricity:               %10.9e\n",gps_alm[i].ety);
			fprintf(out,"Time of Applicability(s):   %10.9e\n",gps_alm[i].toa);
			fprintf(out,"Orbital Inclination(rad):   %10.9e\n",gps_alm[i].inc);
			fprintf(out,"Rate of Right Ascen(R/s):   %10.9e\n",gps_alm[i].rra);
			fprintf(out,"SQRT(A) (m^1/2):            %10.9e\n",gps_alm[i].sqa);
			fprintf(out,"Right Ascen at TOA(rad):    %10.9e\n",gps_alm[i].lan);
			fprintf(out,"Argument of Perigee(rad):   %10.9e\n",gps_alm[i].aop);
			fprintf(out,"Mean Anom(rad):             %10.9e\n",gps_alm[i].ma);
			fprintf(out,"Af0(s):                     %10.9e\n",gps_alm[i].af0);
			fprintf(out,"Af1(s/s):                   %10.9e\n",gps_alm[i].af1);
			fprintf(out,"week:                       %4d   \n",gps_alm[i].week%1024);
			fprintf(out,"\n");
		}
	}
	fclose(out);
}

/*******************************************************************************
FUNCTION write_ephemeris()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ephemeris data to a file for later
	use.  In particular to support a hot start.

*******************************************************************************/

void write_ephemeris()
{
	int i;
	out=fopen(current_eph_file,"w+");
	for (i=1;i<=32;i++)
	{
		if ( gps_eph[i].inc0 >0.0)
		{
			fprintf(out,"**** Ephemeris for PRN-%2d ***********\n",i);
			fprintf(out,"ID:                         %3d\n",i);
			fprintf(out,"Health:                     %3d\n",gps_eph[i].health);
			fprintf(out,"Week:                       %4d\n",   gps_eph[i].week);
			fprintf(out,"URA:                        %3d\n",   gps_eph[i].ura);
			fprintf(out,"E Time of Applic(s):        %12.11e\n",gps_eph[i].toe);
			fprintf(out,"IODE:                       %4d\n",   gps_eph[i].iode);
			fprintf(out,"C Time of Applic(s):        %12.11e\n",gps_eph[i].toc);
			fprintf(out,"IODC:                       %4d\n",   gps_eph[i].iodc);
			fprintf(out,"Tgd(s):                     %12.11e\n",gps_eph[i].tgd);
			fprintf(out,"Af0(s):                     %12.11e\n",gps_eph[i].af0);
			fprintf(out,"Af1(s/s):                   %12.11e\n",gps_eph[i].af1);
			fprintf(out,"Af2(s/s/s):                 %12.11e\n",gps_eph[i].af2);
			fprintf(out,"Eccentricity:               %12.11e\n",gps_eph[i].ety);
			fprintf(out,"Orbital Inclination(rad):   %12.11e\n",gps_eph[i].inc0);
			fprintf(out,"inc rate (r/s)              %12.11e\n",gps_eph[i].idot);
			fprintf(out,"Rate of Right Ascen(R/s):   %12.11e\n",gps_eph[i].omegadot);
			fprintf(out,"SQRT(A) (m^1/2):            %12.11e\n",gps_eph[i].sqra);
			fprintf(out,"dn                          %12.11e\n",gps_eph[i].dn);
			fprintf(out,"Right Ascen at TOE(rad):    %12.11e\n",gps_eph[i].w0);
			fprintf(out,"Argument of Perigee(rad):   %12.11e\n",gps_eph[i].w);
			fprintf(out,"Mean Anom(rad):             %12.11e\n",gps_eph[i].ma);
			fprintf(out,"Cuc(rad):                   %12.11e\n",gps_eph[i].cuc);
			fprintf(out,"Cus(rad):                   %12.11e\n",gps_eph[i].cus);
			fprintf(out,"Crc(m):                     %12.11e\n",gps_eph[i].crc);
			fprintf(out,"Crs(m):                     %12.11e\n",gps_eph[i].crs);
			fprintf(out,"Cic(rad):                   %12.11e\n",gps_eph[i].cic);
			fprintf(out,"Cis(rad):                   %12.11e\n",gps_eph[i].cis);
			fprintf(out,"\n");
		}
	}
	fclose(out);
}

/*******************************************************************************
FUNCTION write_ephemeris()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ephemeris data to a file for later
	use.  In particular to support a hot start.

*******************************************************************************/
void write_Debug_ephemeris(int i)
{
	fprintf(debug,"**** Ephemeris for PRN-%2d ***********\n",i);
	fprintf(debug,"ID:                         %3d\n",i);
	fprintf(debug,"Health:                     %3d\n",gps_eph[i].health);
	fprintf(debug,"Week:                       %4d\n",   gps_eph[i].week);
	fprintf(debug,"URA:                        %3d\n",   gps_eph[i].ura);
	fprintf(debug,"E Time of Applic(s):        %12.11e\n",gps_eph[i].toe);
	fprintf(debug,"IODE:                       %4d\n",   gps_eph[i].iode);
	fprintf(debug,"C Time of Applic(s):        %12.11e\n",gps_eph[i].toc);
	fprintf(debug,"IODC:                       %4d\n",   gps_eph[i].iodc);
	fprintf(debug,"Tgd(s):                     %12.11e\n",gps_eph[i].tgd);
	fprintf(debug,"Af0(s):                     %12.11e\n",gps_eph[i].af0);
	fprintf(debug,"Af1(s/s):                   %12.11e\n",gps_eph[i].af1);
	fprintf(debug,"Af2(s/s/s):                 %12.11e\n",gps_eph[i].af2);
	fprintf(debug,"Eccentricity:               %12.11e\n",gps_eph[i].ety);
	fprintf(debug,"Orbital Inclination(rad):   %12.11e\n",gps_eph[i].inc0);
	fprintf(debug,"inc rate (r/s)              %12.11e\n",gps_eph[i].idot);
	fprintf(debug,"Rate of Right Ascen(R/s):   %12.11e\n",gps_eph[i].omegadot);
	fprintf(debug,"SQRT(A) (m^1/2):            %12.11e\n",gps_eph[i].sqra);
	fprintf(debug,"dn                          %12.11e\n",gps_eph[i].dn);
	fprintf(debug,"Right Ascen at TOE(rad):    %12.11e\n",gps_eph[i].w0);
	fprintf(debug,"Argument of Perigee(rad):   %12.11e\n",gps_eph[i].w);
	fprintf(debug,"Mean Anom(rad):             %12.11e\n",gps_eph[i].ma);
	fprintf(debug,"Cuc(rad):                   %12.11e\n",gps_eph[i].cuc);
	fprintf(debug,"Cus(rad):                   %12.11e\n",gps_eph[i].cus);
	fprintf(debug,"Crc(m):                     %12.11e\n",gps_eph[i].crc);
	fprintf(debug,"Crs(m):                     %12.11e\n",gps_eph[i].crs);
	fprintf(debug,"Cic(rad):                   %12.11e\n",gps_eph[i].cic);
	fprintf(debug,"Cis(rad):                   %12.11e\n",gps_eph[i].cis);
	fprintf(debug,"\n");
}


/*******************************************************************************
FUNCTION write_ion_utc()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function writes the broadcast ionospheric correction data and the
      parameters to tie GPS time to UTC to a file for later use.

*******************************************************************************/

void write_ion_utc()
{
	out=fopen(ion_utc_file,"w+");
	fprintf(out,"al0:                        %e\n",al0);
	fprintf(out,"al1:                        %e\n",al1);
	fprintf(out,"al2:                        %e\n",al2);
	fprintf(out,"al3:                        %e\n",al3);
	fprintf(out,"b0:                         %f\n",b0);
	fprintf(out,"b1:                         %f\n",b1);
	fprintf(out,"b2:                         %f\n",b2);
	fprintf(out,"b3:                         %f\n",b3);
	fprintf(out,"a0                          %f\n",a0);
	fprintf(out,"a1                          %f\n",a1);
	fprintf(out,"dtls                        %f\n",dtls);
	fprintf(out,"tot                         %f\n",tot);
	fprintf(out,"WNt                         %f\n",WNt);
	fprintf(out,"WNlsf                       %f\n",WNlsf);
	fprintf(out,"DN                          %f\n",DN);
	fprintf(out,"dtlsf                       %f\n",dtlsf);
	fclose(out);
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

long fix_sqrt(long x)
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

#define SCALED_PI_ON_2  25736L
#define SCALED_PI       51472L

inline long fix_atan2(long y,long x)
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
		result = ( y>0) ? n-n3+SCALED_PI : n-n3-SCALED_PI;
	}
	else if (y>0 &&  y>labs(x))
	{
		n=(x<<14)/y;
		n3=((((n*n)>>14)*n)>>13)/9;
		result=SCALED_PI_ON_2-n+n3;
	}
	else if (y<0 && -y>labs(x))
	{
		n=(x<<14)/y;
		n3=((((n*n)>>14)*n)>>13)/9;
		result=-n+n3-SCALED_PI_ON_2;
	}
	return(result);
}


int matinv( double **q, int n)
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



void mult_ATB(double **A, int arows, int acols, double **B, int brows,
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

double** dmatrix(unsigned row, unsigned column)
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


void free_dmatrix( double **m, unsigned row)
{
	unsigned i;
	for (i=0; i<row; i++) free((double*)m[i]);
	free((double **)m);
}
