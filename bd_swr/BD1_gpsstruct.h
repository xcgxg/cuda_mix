// Released on July 9, 2004

#ifndef BD1_GPS_STRUCT_H
#define BD1_GPS_STRUCT_H

#include "BD1_gpsconst.h"

typedef struct
{
	unsigned long sv;

	unsigned long epochCounter;			// 锁存的epoch值
	unsigned long epochCounterCheck;	// 实时epoch计数器
	//unsigned long epochCounter_d2;	
	//unsigned long epochCounterCheck_d2;	

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

} BD1_CORRELATOR;


typedef struct
{
	int  reverseflag;
	int  state;							// 通道的状态
	double code_freq;					// 需要设置的码速率，单位：chip/s
	double carrier_freq;				// 需要设置的载波中频频率，单位：Hz
	long tow_sync;						// 收到TLM和HOW字后的时间同步标志
	long frame_ready;					// 一个完整的帧接收完的标志
	long  offset;						// 第一子帧头在message数组中的索引
	long  t_count;						// 收到的比特计数，采用循环方式，从0-1499
	long  ms_count;						// 解调1bit时用的ms计数器(模20)，计满后将所有IQ值累加再解调
	long  n_frame;						// 已经收到的帧计数
	long ch_time;						// 环路进入pull-in状态的时间,只在ch_pull_in中累加
	unsigned long trackTime;			// 环路锁定进入tracking模式的时间
	int  sfid;							// 子帧的标号
	int  page5;							// 导航电文中的page ID
	long  i_early, q_early, i_prompt, q_prompt, i_late, q_late;	// 读出的IQ通道积分值
	long  q_prom_20, i_prom_20;								// 20msIQ通道值的和
	long  q_early_20, i_early_20;								// 20msIQ通道值的和
	long  q_late_20, i_late_20;								// 20msIQ通道值的和
	long  i_old, q_old;										// 仅用于FLL，记录上一次的IQ积分值

	// 以下一组变量仅用于位同步
	long IBuf[BD1_BUFSIZE];					// 用于缓存I通道的值
	long QBuf[BD1_BUFSIZE];					// 用于缓存Q通道的值
	char signBuf[BD1_BUFSIZE];				// 缓存IQ通道的符号变化
	long kCell[2];//add by jh 3-16 long kCell[20];						// 累计每个cell里的符号变化
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
	double Pr, dPr;						// 伪距和伪距增量
	double Tropo, Iono;					// 模型计算出的对流层和电离层延迟
	unsigned long fifo0, fifo1;			// 用于检测字同步头的字缓冲区
	unsigned int epoch;					// 对取的ms计数
	///////////1123////////////
	//unsigned int epoch_d2;		
	double carrier_output;

	double CNo;							// 估计的信道信噪比
	double WBP, NBP, NP, NBD;			// 用于估计信噪比的中间变量，参见蓝皮书GPS RECEIVER一章
	double phaseLockDetector;			// 载波相位是否锁定的指示
	double codePhase;					// 读取的码NCO的相位

	double az, el;						// 卫星的方位角和仰角，单位弧度

	long subFrameSyncFlag;				// 子帧同步标志
	long expectedFrameHead;				// 上次子帧头出现在message缓存区中的位置
	long expectedSubFrameID;
	char message[1600];					// 循环缓存区，保持1个帧的数据
	char prn;
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

	long  start;
	long frame_ready_1;
	long pnumID;
	////////////////////////////////////////////////////
	////////////////////////////////////////////////////
	////////////////////////////////////////////////////
	//以下参数 NH 解码时需要             //2007.05.06
	int fifo[BD1_FIFO_NUM]; //相关器
	int v[BD1_FIFO_NUM];    //每相关器输出值
	int p;
	int count;            //当前收到的含比特数(NH码未剥离)
	int NH_count;         //0-19
	int NHMatchFlag;      //NH码匹配标志
	double itemp[20];                      //2007.05.08 用于缓存I通道的积分值
	double qtemp[20];                   //2007.05.08  用于缓存Q通道的积分值
	////////////////////////////////////////////////////
	////////////////////////////////////////////////////
	////////////////////////////////////////////////////
} BD1_CHANNEL;

typedef struct
{
	char state;							// USING、HOLD、AVAILABLE
	long undetectedCounter;

	int NumToTryAcq;
	int NumBTWEachTrial;
	double maxf;
	double minf;
} BD1_SVSTRUCT;

typedef struct           // Approximate orbital parameters
{
	double w, ety, inc, rra, sqa, lan, aop, ma, toa, af0, af1;

	char text_message[23];
	long health, week, sat_file;
} BD1_ALMANAC;

typedef struct           // Precise orbital parameters
{
	int iode, iodc, ura, valid, health, week;
	double dn, tgd, toe, toc, omegadot, idot, cuc, cus, crc, crs, cic, cis;
	double ma, e, sqra, w0, inc0, w, wm, ety, af0, af1, af2;
} BD1_EPHEMERIS;

typedef struct
{
	int availablesvnumber;
	int availablesvprn[BD1_chmax];
}BD1_AVAILABLESV;

typedef struct
{
	int deg, min;
	float sec;
} BD1_DMS;

typedef struct
{
	double azimuth, elevation, doppler;
	double x, y, z;
} BD1_SATVIS;

typedef struct
{
	double x, y, z;
} BD1_XYZ;

typedef struct
{
	double x, y, z, tb;
	double az, el;
} BD1_ECEFT;

typedef struct
{
	double lat, lon, hae;
} BD1_LLH;

typedef struct
{
	double x, y, z, dt;
	double xv, yv, zv, df;
} BD1_PVT;

typedef struct
{
	double  east, north, up;
	double  clock_err;
	BD1_XYZ     x, y, z;
} BD1_VELOCITY;

typedef struct
{
	BD1_VELOCITY  vel;
	BD1_ECEFT     pos;
	BD1_LLH       loc;
	BD1_XYZ		north, east, up;
} BD1_STATE;

typedef struct
{
	int doy;  //day of the year
	int Year;
	int Month;
	int Day;
	int Hour;
	int Min;
	double Second;

}BD1_TIME;
typedef struct
{
	int    iftrue; //20070515 lcchxy 修改
	double x;
	double y;
	double z;

}BD1_xyz_t;

#endif	