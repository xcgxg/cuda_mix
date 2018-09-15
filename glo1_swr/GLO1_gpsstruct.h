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
//PZ-90 -> WGS84 原始坐标转换参量
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
	
	unsigned long epochCounter;			// 锁存的epoch值
	unsigned long epochCounterCheck;	// 实时epoch计数器

	unsigned long carrierCycle;			// 实时的载波相位整周数
	unsigned long carrierCycleLatch;	// 锁存的载波相位整周数
	long phaseLocal;					// 实时的载波相位的小数部分, max value: CARRIER_TABLE_LENGTH -> 2*pi
	long phaseLocalLatch;				// 锁存的载波相位的小数部分

	__int64 tau;						// 实时的码相位,max value: 1022.99999999 -> code length 1023
	__int64 tauLatch;					// 锁存的码相位

	long fCarr;							// carrier frequency in Hz of carrier NCO
	__int64 fCode;						// code frequency

	long state;							// 相关器通道是否运行的标志

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
	long ready;							// 相关器Dump的标志

} GLO1_CORRELATOR;


typedef struct
{
	int  reverseflag;                   //通道码是否反相（done）
	int  state;							// 通道的状态
	double code_freq;					// 需要设置的码速率，单位：chip/s
	double carrier_freq;				// 需要设置的载波中频频率，单位：Hz
	long tow_sync;						// 收到TLM和HOW字后的时间同步标志
	int unpack_glonass_flag;
	long frame_ready;					// 一个完整的帧接收完的标志
	long  offset;						// 第一子帧头在message数组中的索引
	long  t_count;						// 收到的比特计数，采用循环方式，从0-1499
	long  ms_count;						// 解调1bit时用的ms计数器(模20)，计满后将所有IQ值累加再解调
	long  n_frame;						// 已经收到的帧计数
	long ch_time;						// 环路进入pull-in状态的时间,只在ch_pull_in中累加
	unsigned long trackTime;			// 环路锁定进入tracking模式的时间
	int  sfid;							// 子帧的标号
	int  page5;							// 导航电文中的page ID
	long  i_early,q_early,i_prompt,q_prompt,i_late,q_late;	// 读出的IQ通道积分值
	long  q_prom_20,i_prom_20;								// 20msIQ通道值的和
	long  q_early_20,i_early_20;								// 20msIQ通道值的和
	long  q_late_20,i_late_20;								// 20msIQ通道值的和
	long  i_old, q_old;										// 仅用于FLL，记录上一次的IQ积分值
	
	// 以下一组变量仅用于位同步
	long IBuf[GLO1_BUFSIZE];					// 用于缓存I通道的值
	long QBuf[GLO1_BUFSIZE];					// 用于缓存Q通道的值
	char signBuf[GLO1_BUFSIZE];				// 缓存IQ通道的符号变化
	long kCell[20];						// 累计每个cell里的符号变化
	long BitSyncIndex;					// IQBuf数组里，下一个可存放数据的位置，循环索引
	long IQBufSize;						// IQBuf中数据的个数

	double dllS0;						// DLL的低通滤波器的积分器状态
	double dllS1;						// CA码频率的增量，仅用于记录运算的中间值
	double cLoopS0, cLoopS1;			// PLL和FLL环路滤波器的两个积分器状态
	double fc;							// 载波环产生的控制频率
	double doppler;						// 同fc，但它是fc在update时的锁存值，单位：Hz
	double vDoppler;					// doppler中由于运动产生的doppler分量
	double carrier_corr;				// 载波中频修正量，包括钟漂和多普勒，单位：Hz

	long FLLFlag;						// 是否使用载波环
	long FLLIndex;						// 数据准备好标志
	double freqErr;						// 频率误差

	long tr_bit_time;					// 相对于GPS星期的起始点，已经传输的bit数
	long meas_bit_time;
	long TOW;							// 仅记录
	long TLM;							// 仅记录
	long carrier_counter;				// 在一个update周期内积分载波相位的整周部分(锁存)
	long cycle_sum;						// 在一个update周期内积分载波相位的整周部分(实时)
	double carr_dco_phase, old_carr_dco_phase;	// 读出载波相位的小数部分
	double d_carr_phase;						// 相邻两个测量历元的小数部分的载波相位差
	
	double int_carr_phase;				// 两次测量间的积分载波相位
	double Pr,dPr;						// 伪距和伪距增量
	double Tropo,Iono;					// 模型计算出的对流层和电离层延迟
	unsigned long fifo0,fifo1,fifo2,fifo3,fifo4,fifo5,fifo6,fifo7; // 用于检测字同步头的字缓冲区
	unsigned long buf0,buf1,buf2; // 缓冲区

	unsigned int epoch;					// 对取的ms计数
	double carrier_output;
	
	double CNo;							// 估计的信道信噪比
	double WBP, NBP, NP, NBD;			// 用于估计信噪比的中间变量，参见蓝皮书GPS RECEIVER一章
	double phaseLockDetector;			// 载波相位是否锁定的指示
	double codePhase;					// 读取的码NCO的相位

	double az, el;						// 卫星的方位角和仰角，单位弧度

	long subFrameSyncFlag;				// 子帧同步标志
	long expectedFrameHead;				// 上次子帧头出现在message缓存区中的位置
	long expectedSubFrameID;
	char message[3200];					// 循环缓存区，保持1个帧的数据
	char prn;
	int  K;                            //GLONASS频道号 todo
	float IF;                          //每个频道的中频 todo
	char bit;							// 最近解调的1bit


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
	char state;							// USING、HOLD、AVAILABLE
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
	unsigned short index;    //序号
							 //  int16s pow10;    //10的幂次因子
	short pow2;     //2的幂次因子
					//  int16s powpi;    //pi的幂次因子
	short signq;    //是否有符号
	short startbit;//段的开始
	short num_bits;//段位数
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
	fp64    URA;     //用户测距精度
	int16u	NT;
	int16u	n;       //index of satllite
	int16u	Mod;

	int16u	valid;

}GLO1_glonass_ephemeris;

typedef struct		//frame 5 string 14 15中的电文数据
{
	int16u	daynum;
	fp64	tauc;
	int16u	N4;
	fp64	taugps;
	int16u	str5_ln;

	int16u	valid;
}GLO1_glonass_almanac_str5;

typedef struct		//string 5 以及 frame 5 string 14 15中的电文数据
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