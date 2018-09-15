// Released on July 9, 2004

#include "GLO1_gpsconst.h"
#ifndef GLO1_GPS_STRUCT_H
#define GLO1_GPS_STRUCT_H

#define GLONASS_SV_TOTAL_NUM  30

#define GLO1_SGS90_MU  3.9860044E14       
#define GLO1_SGS90_WE  7.292115E-5
#define GLO1_SGS90_J2  0.00108263
#define GLO1_SGS90_A   6378136.0
#define GLO1_SGS90_EE  6.694385E-3
#define GLO1_SGS90_FL  298.256999982   
#define GLO1_FC2      2.0L
#define GLO1_FC3      3.0L
#define GLO1_FC4      4.0L
#define GLO1_FC5      5.0L
#define GLO1_FC6      6.0L
#define GLO1_FC7      7.0L
#define GLO1_FC8      8.0L
#define GLO1_FC10     10.0L
#define GLO1_FC11     11.0L
#define GLO1_FC16     16.0L
#define GLO1_FC35     35.0L
#define GLO1_FC120    120.0L 
#define GLO1_UNIT     1.0 

/**/
//PZ-90 -> WGS84 ԭʼ����ת������
#define GLO1_DX       -1.1L //-0.47L//
#define GLO1_DY       -0.3L//-0.51L//
#define GLO1_DZ       -0.9L//-1.56L//
#define GLO1_WX       0.0L//0.076E-6L//    
#define GLO1_WY       0.0L//0.017E-6L//
#define GLO1_WZ       -0.82E-6L//1.728E-6L//
#define GLO1_MM       -0.12E-6L //22E-9L//   	//   coefficient of compression  
/**/

typedef struct
{
	unsigned long sv;
	
	unsigned long epochCounter;			// �����epochֵ
	unsigned long epochCounterCheck;	// ʵʱepoch������

	unsigned long carrierCycle;			// ʵʱ���ز���λ������
	unsigned long carrierCycleLatch;	// ������ز���λ������
	long phaseLocal;					// ʵʱ���ز���λ��С������, max value: CARRIER_TABLE_LENGTH -> 2*pi
	long phaseLocalLatch;				// ������ز���λ��С������

	__int64 tau;						// ʵʱ������λ,max value: 1022.99999999 -> code length 1023
	__int64 tauLatch;					// ���������λ

	long fCarr;							// carrier frequency in Hz of carrier NCO
	__int64 fCode;						// code frequency

	long state;							// �����ͨ���Ƿ����еı�־

	long i_prompt;
	long q_prompt;
	long i_early;
	long q_early;
	long i_late;
	long q_late;
	long latchedSPR;
	long latchedSPI;
	long latchedSER;
	long latchedSEI;
	long latchedSLR;
	long latchedSLI;

	//long codeSlewCounter;
	//long iqInterrupt;
	long ready;							// �����Dump�ı�־

} GLO1_CORRELATOR;


typedef struct
{
	int  reverseflag;                   //ͨ�����Ƿ��ࣨdone��
	int  state;							// ͨ����״̬
	double code_freq;					// ��Ҫ���õ������ʣ���λ��chip/s
	double carrier_freq;				// ��Ҫ���õ��ز���ƵƵ�ʣ���λ��Hz
	long tow_sync;						// �յ�TLM��HOW�ֺ��ʱ��ͬ����־
	int unpack_glonass_flag;
	long frame_ready;					// һ��������֡������ı�־
	long  offset;						// ��һ��֡ͷ��message�����е�����
	long  t_count;						// �յ��ı��ؼ���������ѭ����ʽ����0-1499
	long  ms_count;						// ���1bitʱ�õ�ms������(ģ20)������������IQֵ�ۼ��ٽ��
	long  n_frame;						// �Ѿ��յ���֡����
	long ch_time;						// ��·����pull-in״̬��ʱ��,ֻ��ch_pull_in���ۼ�
	unsigned long trackTime;			// ��·��������trackingģʽ��ʱ��
	int  sfid;							// ��֡�ı��
	int  page5;							// ���������е�page ID
	long  i_early,q_early,i_prompt,q_prompt,i_late,q_late;	// ������IQͨ������ֵ
	long  q_prom_20,i_prom_20;								// 20msIQͨ��ֵ�ĺ�
	long  q_early_20,i_early_20;								// 20msIQͨ��ֵ�ĺ�
	long  q_late_20,i_late_20;								// 20msIQͨ��ֵ�ĺ�
	long  i_old, q_old;										// ������FLL����¼��һ�ε�IQ����ֵ
	
	// ����һ�����������λͬ��
	long IBuf[GLO1_BUFSIZE];					// ���ڻ���Iͨ����ֵ
	long QBuf[GLO1_BUFSIZE];					// ���ڻ���Qͨ����ֵ
	char signBuf[GLO1_BUFSIZE];				// ����IQͨ���ķ��ű仯
	long kCell[20];						// �ۼ�ÿ��cell��ķ��ű仯
	long BitSyncIndex;					// IQBuf�������һ���ɴ�����ݵ�λ�ã�ѭ������
	long IQBufSize;						// IQBuf�����ݵĸ���

	double dllS0;						// DLL�ĵ�ͨ�˲����Ļ�����״̬
	double dllS1;						// CA��Ƶ�ʵ������������ڼ�¼������м�ֵ
	double cLoopS0, cLoopS1;			// PLL��FLL��·�˲���������������״̬
	double fc;							// �ز��������Ŀ���Ƶ��
	double doppler;						// ͬfc��������fc��updateʱ������ֵ����λ��Hz
	double vDoppler;					// doppler�������˶�������doppler����
	double carrier_corr;				// �ز���Ƶ��������������Ư�Ͷ����գ���λ��Hz

	long FLLFlag;						// �Ƿ�ʹ���ز���
	long FLLIndex;						// ����׼���ñ�־
	double freqErr;						// Ƶ�����

	long tr_bit_time;					// �����GPS���ڵ���ʼ�㣬�Ѿ������bit��
	long meas_bit_time;
	long TOW;							// ����¼
	long TLM;							// ����¼
	long carrier_counter;				// ��һ��update�����ڻ����ز���λ�����ܲ���(����)
	long cycle_sum;						// ��һ��update�����ڻ����ز���λ�����ܲ���(ʵʱ)
	double carr_dco_phase, old_carr_dco_phase;	// �����ز���λ��С������
	double d_carr_phase;						// ��������������Ԫ��С�����ֵ��ز���λ��
	
	double int_carr_phase;				// ���β�����Ļ����ز���λ
	double Pr,dPr;						// α���α������
	double Tropo,Iono;					// ģ�ͼ�����Ķ�����͵�����ӳ�
	unsigned long fifo0,fifo1,fifo2,fifo3,fifo4,fifo5,fifo6,fifo7; // ���ڼ����ͬ��ͷ���ֻ�����
	unsigned long buf0,buf1,buf2; // ������

	unsigned int epoch;					// ��ȡ��ms����
	double carrier_output;
	
	double CNo;							// ���Ƶ��ŵ������
	double WBP, NBP, NP, NBD;			// ���ڹ�������ȵ��м�������μ���Ƥ��GPS RECEIVERһ��
	double phaseLockDetector;			// �ز���λ�Ƿ�������ָʾ
	double codePhase;					// ��ȡ����NCO����λ

	double az, el;						// ���ǵķ�λ�Ǻ����ǣ���λ����

	long subFrameSyncFlag;				// ��֡ͬ����־
	long expectedFrameHead;				// �ϴ���֡ͷ������message�������е�λ��
	long expectedSubFrameID;
	char message[3200];					// ѭ��������������1��֡������
	char prn;
	int  K;                            //GLONASSƵ���� todo
	float IF;                          //ÿ��Ƶ������Ƶ todo
	char bit;							// ��������1bit


	char searchDirection;
	double currentHighFreq;
	double currentLowFreq;
	char integratedMs;
	long ic[3], qc[3];
	char dwellTime;
	char kc[3];
	char searchStatus[3];
	long halfCodeChip;

} GLO1_CHANNEL;

typedef struct
{
	char state;							// USING��HOLD��AVAILABLE
	long undetectedCounter;
	
	int NumToTryAcq;
	int NumBTWEachTrial;
	double maxf;
	double minf;
} GLO1_SVSTRUCT;

typedef struct           // Approximate orbital parameters
{
	double w,ety,inc,rra,sqa,lan,aop,ma,toa,af0,af1;
	char text_message[23];
	long health,week,sat_file;
} GLO1_ALMANAC;

typedef struct           // Precise orbital parameters
{
	int iode,iodc,ura,valid,health,week;
	double dn,tgd,toe,toc,omegadot,idot,cuc,cus,crc,crs,cic,cis;
	double ma,e,sqra,w0,inc0,w,wm,ety,af0,af1,af2;
} GLO1_EPHEMERIS;

typedef struct
{
	int availablesvnumber;
	int availablesvprn[GLO1_chmax];
}GLO1_AVAILABLESV;

typedef struct
{
	int deg,min;
	float sec;
} GLO1_DMS;

typedef struct
{
	double azimuth,elevation,doppler;
	double x,y,z;
} GLO1_SATVIS;

typedef struct
{
	double x,y,z;
} GLO1_XYZ;

typedef struct
{
  double x,y,z,tb,vx,vy,vz;
  double az,el;
} GLO1_ECEFT;

typedef struct
{
  double lat,lon,hae;
} GLO1_LLH;

typedef struct 
{
   double x,y,z,dt;
   double xv,yv,zv,df;
} GLO1_PVT;

typedef struct
{
  double  east,north,up;
  double  clock_err;
  GLO1_XYZ     x, y, z;
} GLO1_VELOCITY;

typedef struct
{
	GLO1_VELOCITY  vel;
	GLO1_ECEFT     pos;
	GLO1_LLH       loc;
	GLO1_XYZ		north, east, up;
} GLO1_STATE;

//GLONASS
typedef struct
{
	unsigned short index;    //���
							 //  int16s pow10;    //10���ݴ�����
	short pow2;     //2���ݴ�����
					//  int16s powpi;    //pi���ݴ�����
	short signq;    //�Ƿ��з���
	short startbit;//�εĿ�ʼ
	short num_bits;//��λ��
}GLO1_decode_glonass_info;
//MY TYPE define
typedef  unsigned char 			boolean;
typedef  unsigned char 			int8u;
typedef  char 					int8s;
typedef  unsigned short 		int16u;
typedef  short 					int16s;
typedef  int 					int32s;
typedef  unsigned int 			int32u;
typedef  long 					int40s;
typedef  unsigned long 			int40u;
//typedef  long long 				int64s;
//typedef  unsigned long long 	int64u;
typedef  float 					fp32;
typedef  double 				fp64;
typedef  unsigned char          uchar;
typedef  unsigned short int     usint;
typedef struct
{
	unsigned short	star;
	unsigned short	P1;
	unsigned int	tk_hour;
	unsigned int	tk_min;
	unsigned int	tk_sec;
	double dot_Xn;
	double doudot_Xn;
	double	Xn;

	unsigned short	Bn;       //health flag  0:ok  1:malfuntion(not consider second&third bits)  
	int16u	P2;
	int16u	tb;
	fp64	dot_Yn;
	fp64	doudot_Yn;
	fp64	Yn;

	int16u	P3;      //alc included sat num    0: 4sats  1: 5sats
	fp64	gamman;
	int16u	P;
	int16u	ln;      //health of **n-th** sat  0:health  1:malfuntion
	fp64	dot_Zn;
	fp64	doudot_Zn;
	fp64	Zn;

	fp64	taun;
	fp64	deltataun;
	int16u	En;
	int16u	P4;
	int16u	FT;
	fp64    URA;     //�û���ྫ��
	int16u	NT;
	int16u	n;       //index of satllite
	int16u	Mod;

	int16u	valid;

}GLO1_glonass_ephemeris;

typedef struct		//frame 5 string 14 15�еĵ�������
{
	int16u	daynum;
	fp64	tauc;
	int16u	N4;
	fp64	taugps;
	int16u	str5_ln;

	int16u	valid;
}GLO1_glonass_almanac_str5;

typedef struct		//string 5 �Լ� frame 5 string 14 15�еĵ�������
{
	fp64	B1;
	fp64	B2;
	int16u	KP;

	int16u	str15_ln;

	int16u	valid;
}GLO1_glonass_almanac_global;

typedef struct
{
	int16u	Cn;
	int16u	Mn;
	int16u	n;
	fp64	taun;
	fp64	lambdan;
	fp64	deltain;
	fp64	epsilonn;

	fp64	omegan;
	fp64	tlambdan;
	fp64	deltaTn;
	fp64	dot_deltaTn;
	int16u	Hn;
	int16u	ln;

	int16u	valid;

}GLO1_glonass_almanac;

#endif	