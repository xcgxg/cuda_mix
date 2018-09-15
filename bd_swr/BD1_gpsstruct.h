// Released on July 9, 2004

#ifndef BD1_GPS_STRUCT_H
#define BD1_GPS_STRUCT_H

#include "BD1_gpsconst.h"

typedef struct
{
	unsigned long sv;

	unsigned long epochCounter;			// �����epochֵ
	unsigned long epochCounterCheck;	// ʵʱepoch������
	//unsigned long epochCounter_d2;	
	//unsigned long epochCounterCheck_d2;	

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

} BD1_CORRELATOR;


typedef struct
{
	int  reverseflag;
	int  state;							// ͨ����״̬
	double code_freq;					// ��Ҫ���õ������ʣ���λ��chip/s
	double carrier_freq;				// ��Ҫ���õ��ز���ƵƵ�ʣ���λ��Hz
	long tow_sync;						// �յ�TLM��HOW�ֺ��ʱ��ͬ����־
	long frame_ready;					// һ��������֡������ı�־
	long  offset;						// ��һ��֡ͷ��message�����е�����
	long  t_count;						// �յ��ı��ؼ���������ѭ����ʽ����0-1499
	long  ms_count;						// ���1bitʱ�õ�ms������(ģ20)������������IQֵ�ۼ��ٽ��
	long  n_frame;						// �Ѿ��յ���֡����
	long ch_time;						// ��·����pull-in״̬��ʱ��,ֻ��ch_pull_in���ۼ�
	unsigned long trackTime;			// ��·��������trackingģʽ��ʱ��
	int  sfid;							// ��֡�ı��
	int  page5;							// ���������е�page ID
	long  i_early, q_early, i_prompt, q_prompt, i_late, q_late;	// ������IQͨ������ֵ
	long  q_prom_20, i_prom_20;								// 20msIQͨ��ֵ�ĺ�
	long  q_early_20, i_early_20;								// 20msIQͨ��ֵ�ĺ�
	long  q_late_20, i_late_20;								// 20msIQͨ��ֵ�ĺ�
	long  i_old, q_old;										// ������FLL����¼��һ�ε�IQ����ֵ

	// ����һ�����������λͬ��
	long IBuf[BD1_BUFSIZE];					// ���ڻ���Iͨ����ֵ
	long QBuf[BD1_BUFSIZE];					// ���ڻ���Qͨ����ֵ
	char signBuf[BD1_BUFSIZE];				// ����IQͨ���ķ��ű仯
	long kCell[2];//add by jh 3-16 long kCell[20];						// �ۼ�ÿ��cell��ķ��ű仯
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
	double Pr, dPr;						// α���α������
	double Tropo, Iono;					// ģ�ͼ�����Ķ�����͵�����ӳ�
	unsigned long fifo0, fifo1;			// ���ڼ����ͬ��ͷ���ֻ�����
	unsigned int epoch;					// ��ȡ��ms����
	///////////1123////////////
	//unsigned int epoch_d2;		
	double carrier_output;

	double CNo;							// ���Ƶ��ŵ������
	double WBP, NBP, NP, NBD;			// ���ڹ�������ȵ��м�������μ���Ƥ��GPS RECEIVERһ��
	double phaseLockDetector;			// �ز���λ�Ƿ�������ָʾ
	double codePhase;					// ��ȡ����NCO����λ

	double az, el;						// ���ǵķ�λ�Ǻ����ǣ���λ����

	long subFrameSyncFlag;				// ��֡ͬ����־
	long expectedFrameHead;				// �ϴ���֡ͷ������message�������е�λ��
	long expectedSubFrameID;
	char message[1600];					// ѭ��������������1��֡������
	char prn;
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

	long  start;
	long frame_ready_1;
	long pnumID;
	////////////////////////////////////////////////////
	////////////////////////////////////////////////////
	////////////////////////////////////////////////////
	//���²��� NH ����ʱ��Ҫ             //2007.05.06
	int fifo[BD1_FIFO_NUM]; //�����
	int v[BD1_FIFO_NUM];    //ÿ��������ֵ
	int p;
	int count;            //��ǰ�յ��ĺ�������(NH��δ����)
	int NH_count;         //0-19
	int NHMatchFlag;      //NH��ƥ���־
	double itemp[20];                      //2007.05.08 ���ڻ���Iͨ���Ļ���ֵ
	double qtemp[20];                   //2007.05.08  ���ڻ���Qͨ���Ļ���ֵ
	////////////////////////////////////////////////////
	////////////////////////////////////////////////////
	////////////////////////////////////////////////////
} BD1_CHANNEL;

typedef struct
{
	char state;							// USING��HOLD��AVAILABLE
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
	int    iftrue; //20070515 lcchxy �޸�
	double x;
	double y;
	double z;

}BD1_xyz_t;

#endif	