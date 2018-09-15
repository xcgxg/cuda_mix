// Released on July 9, 2004
#include "../stdafx.h"
#include "../core/Eigen/Eigen"
#include "../core/Eigen/Dense"
using namespace Eigen;
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <math.h>
#include <time.h>
#include <assert.h>


#include "gpsconst.h"
#include "gpsstruct.h"
#include "../receiver/readData.h"
#include "correlatorProcess.h"
#include "acqCA.h"
#include "cagen.h"
#include "gp2021.h"
#include "gpsfuncs.h"
#include "gpsrcvr.h"
#include "../receiver/fft.h"

#include "../receiver/UI.h"
#include "../receiver/UpdateUIMessages.h"

#include "../core/global_var.h"
#include "../core/global_STAP.h"


#include "../receiver/anti_receiver_data.h"
#include "../core/socket.h"

//#define DEMO_CONSOLE
#ifdef DEMO_CONSOLE
#endif

#include <windows.h>

//#include "TimeCounterEx.h"

#define	IRQLEVEL        0

CHANNEL		   chan[chmax + 1];				// ȫ�̱��������ڴ洢ÿ��ͨ�����ٻ�·��״̬
CORRELATOR	   correlator[chmax + 1];		// ȫ�̱��������ڴ洢ÿ��ͨ���������״̬
SVSTRUCT	      svStruct[MaxSvNumber + 1];	// ��ʾ���ǿ�����
long		      globalBufLength[chmax + 1];	// ���ڱ���ÿ��ͨ����������ݳ��ȣ����ʼ��
double  *buffer[chmax + 1];			// ���ڱ���ÿ��ͨ���Ļ������ݣ����ʼ������ռ�
long		      correlatorDataLength;	// ���л����������ݳ���
long		      TIC_RT_CNTR;				// TICʵʱ��������
long		      TIC_CNTR;					// TIC������������Ӧ��TIC�жϵķ�������
// ע�⣺TIC_CNTR�ĸ�ֵֻ����programTIC����ʵ��
double         TIC_CNTR_RES;
double         TIC_CNTR_DBL;
long		      TIC_OCCUR_FLAG;			// TIC�жϷ����ı�־

int			   display_page = 0;			// ������ʾ��i��ҳ��
unsigned	test[16] =
{ 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };
unsigned int tr_ch[13];					   // ����0���ã�����Ϊ������Ч�۲����ݵ�chan�ı��
int			out_debug, out_pos, out_vel, out_time;		// �Ƿ����debug��PVT��Ϣ�ı�־��
DMS			cur_lat, cur_long;			   // ������ʾ��γ��

// ���¶����������ȱʡֵ����ͨ���ı��ļ�rcvr_par.dat���趨ʵ��
int			   nav_tic;						// һ��nav_up��ʱ������Ӧ��TIC�жϵĴ���
int            ICP_CTL = 0;		         // ����ۻ��ز���λ���Ǹ��ٻ�·��ֵ����־
double         code_corr;              // CA�����ʵ����������֣�1.023MHz��Ӧ1.0
long           time_on = 0;	            // ������������Ϊ�룬����ͣ��������ֹ
long		      cc_scale = 1540;       	// �ز���Ƶ(1575.42MHz)/CA���Ƶ(1.023MHz)
double         nav_up = 1.0;					// ������Ļ�����λ����λ����
double		   speed, heading;          // ���ջ��ٶȵľ���ֵ�ͷ�λ��
long		d_tow;								// ���ջ�ʱ����λ����
int			key = 0;								// �����̵İ�����Ϣ
int			tic_count = 0;						// TIC��������Ҳ�ǲ����жϷ��������ļ�����
// ע�⣺�����жϵ�Ƶ������ȱʡΪ��100ms
// ������ΧΪ1s[0-9]
int			hms_count = 0;						// TIC��������������ΧΪ1����[0-599]
int			nav_count;							// TIC��������������ΧΪ[0,nav_tic-1]
int			min_flag;							// ���Ӽ�������־
int			nav_flag;							// ���㵼�����־
int			sec_flag;							// ���־
int			n_track;							// ����(ͨ������trackingģʽ)�����Ǹ���
unsigned int interr_int = 512;
double		clock_offset = 0.0;					// ���ջ�����Ư����λ��ppm,��ʼֵδ֪���ٶ�Ϊ0
// ��ֵ��ʾ���ջ���Ƶ�ʵ��ڱ��ֵ
XYZ			rec_pos_ecef;						// ���ջ�������
long		i_TIC_dt;							// ���μ��㵼����ļ��,��λ���������
double		TIC_dt;								// ���μ��㵼����ļ��,��λ����
// ����TIC_dt=i_TIC_dt*�������
double		m_time[3];							// ���ջ�ʱ��,[1]�ǵ�ǰʱ�̵ģ�[0]����һ�ε�
// [2]�ƺ�û����
// m_time���ڱ�ʾ���ջ���GPSʱ��
// ��ԭ���ĳ�����m_timeû����Ч�ĸ���,
// ��Ӧ�ÿ����жϱ�־sec_flag��ά��
double		delta_m_time;						// ���ջ�ʱ���С������
double		m_error;							// ���ջ��Ĳ���ʱ���GPS��������
long		TIC_sum;							// ��������ж��������ۻ��ļ���ֵ
//int			astat;							// ������GP2021��������dump ready��־�ļĴ���
//int			mstat;							// ������GP2021��������miss data��־�ļĴ���
char		**caTable;							// ��ʵʱ����ʱ���洢37��1023��CA���

double		DLLc1, DLLc2;						// DLL��ϵ��
double		PLLc0, PLLc1, PLLc2;				// PLL��ϵ��
double		FLLa1, FLLa2;						// FLL��ϵ��

// �����õ�ȫ�̱���
double		*IQQ, *IQI;
double		IBUF[3000], QBUF[3000];
FILE		*fpIQ;
long		IQCounter;
FILE		*fpobs;
FILE		*fpeph;

long		detectNum[chmax + 1];					// ����ĳ�����ǵĳ���ʱ��
double		threshold_sig_search;				// �źż������							

//char		data[SAMPLESPMS+1];							// �źŻ�����
//char		data[6000];							// �źŻ�����
double		minFreqErr[chmax + 1], maxFreqErr[chmax + 1];

unsigned long uLastPCTick, uCurrentPCTick;

__int64 iTotalBitCounter = 0;

bool bLocalTimeSetByTow = false;
bool bTowDecoded = false;

// �ⲿ����
double DLLdT = 0.5;
extern XYZ			rec_pos_xyz;
extern FILE			*stream, *debug, *in, *out, *kalm;
extern LLH			current_loc, rp_llh;
extern LLH			rec_pos_llh;
extern time_t		thetime;
extern int			status;
extern unsigned long  clock_tow;
extern int			alm_gps_week, gps_week, almanac_valid, almanac_flag, handle;
extern SATVIS		xyz[33];
extern EPHEMERIS	gps_eph[33];
extern double		gdop, pdop, hdop, vdop, tdop, alm_toa;
extern ECEFT		track_sat[13];
extern double		dt[13], cbias;
extern XYZ			d_sat[13];
extern double		meas_dop[13];
extern PVT			rpvt;
extern STATE		receiver;
extern int			m_tropo, m_iono, align_t;	// flags for using tropo and iono models	
extern ALMANAC		gps_alm[33];
extern double		carrier_ref, code_ref;
extern char			tzstr[40];
extern double		mask_angle;
extern double		b0, b1, b2, b3, al0, al1, al2, al3;	// broadcast ionospheric delay model
extern double		a0, a1, tot, WNt, dtls, WNlsf, DN, dtlsf;//broadcast UTC data

extern int ADNumber;   //0624
float   CARRIER_FREQ;   //�����Ƶ  0625
double  SAMPLING_FREQ;     //0626
double  SAMPLING_INT;      //0626
long    DETECTSIZE;        //0626
long	TIC_ref;           //0626
double  deltaPhaseConst;   //0626
double  deltaCodePhaseConst;    //0626
long    SAMPLESPMS;         //0627
long    ACC_INT_PERIOD;     //0627
char    *GPSL1_data;              //0627
long    FFTSIZE;            //0627
long    bufSize;            //0627

FILE   *fpCodeRange1, *fpCodeRange2;     //0628
FILE   *fpCarrRange1, *fpCarrRange2;     //0628
long   ldcount = 0;                     //0628
FILE   *fpdoppler;                      //0628
FILE   *fpCNo;                          //0628

char   if_data_file[4096];
/*******************************************************************************
FUNCTION main()
RETURNS  None.

PARAMETERS None.

PURPOSE
This is the main program to control the GPS receiver

*******************************************************************************/

extern char last_prn[12];
char hot_cold;
extern char curloc_file[];
FILE *daicy_file_pr;
FILE *daicy_file_pos;
FILE   *out_trtime;

long GPSL1_sim_main_init()
{
	char  ch;
	long num, TIC_DIF;
	long numRead;
	//FILE* fpData;
	//FILE* fpOption;
	char fileName[200];
	int i;

	ADNumber = (int)pow(2.0, ADNumber);//ADNumber������ı�������λ��
	carrier_ref = CARRIER_FREQ;//��ƵƵ��

	SAMPLING_INT = 1.0 / SAMPLING_FREQ;//SAMPLING_FREQ:����Ƶ��
	DETECTSIZE = (long)((long long)(SAMPLING_FREQ*0.01 + 0.5) & 0xffffffffUL);//10��������ݵ���

	TIC_ref = (long)((long long)(SAMPLING_FREQ*0.1 + 0.5 - 1) & 0xffffffffUL);// 100ms��Ӧ���жϼ���ֵ 100��������ݵ���  Ԥ��TIC_CNTR�ļ�����100����

	deltaPhaseConst = SAMPLING_INT*carrierNCOMax;// carrierNCOMax��Ӧ���ز�DCO��ʵ�ʷֱ���2^30   ���ٳ��Ա���ز��ͱ�ɱ���ز�Ƶ������
	// deltaPhaseConst�Ͷ�Ӧ�����ز�Ƶ���ֵ� 2^30/����Ƶ��
	deltaCodePhaseConst = SAMPLING_INT*1.023e6*d_2p40;// �����Ƶ��*2^40/����Ƶ�� = ��Ƶ����
	//SAMPLING_INT*1.023e6*d_2p40;

	SAMPLESPMS = (long)((long long)(SAMPLING_FREQ*0.001) & 0xffffffffUL);   //0627//lyq
	//jh SAMPLESPMSÿ��������ݵ���

	ACC_INT_PERIOD = SAMPLESPMS / 2 + 1;//+1֮����Ȼ���Ǹպõ�0.5���룬������ֻ�Ƕ��ĵ����������ݴ����ʱ�򣬻��ǰ�������Ķ�����ȡ�ģ�����ûӰ��
	//#define ACC_INT_PERIOD  SAMPLESPMS/2+1			// 0.5ms��������
	//jh ACC_INT_PERIOD:ÿ0.5������֣�������Ȼ��1���룩��0.5������ж���
	GPSL1_data = new char[SAMPLESPMS + 1];

	fopen_s(&out_trtime, "tr_time.txt", "w");//������
	fopen_s(&daicy_file_pr, "daicy_pr.txt", "w");
	fopen_s(&daicy_file_pos, "daicy_pos.txt", "w");


	float fnumber = (float)SAMPLING_FREQ / 500;//2����ĵ���  ��ΪҪȡ2MS�ı���α�룬ΪʲôҪ2ms��GPS�źŵ�CA�볤1023��Ƭ������1ms �����ڸ����������£�1ms�ڲɵ��ĵ�����һ����2n
	int fftcount = 0;
	//lyq
	do
	{
		fnumber = fnumber / 2;//fnumber�Ǹ�����
		fftcount++;
	} while (fnumber > 2);

	if (fnumber <= 1.5)
		FFTSIZE = (int)(pow(2.0, fftcount)*1.5);//��������ܴ���1.5�� 2�Ķ��ٴη�  ��ע�ⶼ��ȡ��
	else
		FFTSIZE = (int)pow(2.0, fftcount + 1);//������if else���϶�ȡ��������2ms�ĵ���

	bufSize = (long)((long long)(SAMPLING_FREQ*0.0005) & 0xffffffffUL);//0.5��������ݵ�������ACC_INT_PERIOD��Ӧ

	fopen_s(&fpCodeRange1, "CodeRangeSample.txt", "w");      //0628   
	fopen_s(&fpCodeRange2, "CodeRangeFix.txt", "w");
	fopen_s(&fpCarrRange1, "CarrRangeSample.txt", "w");
	fopen_s(&fpCarrRange2, "CarrRangeFix.txt", "w");
	fopen_s(&fpdoppler, "doppler.txt", "w");
	fopen_s(&fpCNo, "CNo.txt", "w");

#ifdef DEMO_CONSOLE      
	sharemem_start();
	sharemem_write(1);
#endif

	read_rcvr_par(); //��rcvr_par�ļ�������Ʋ�����ٵĽ��ջ�����
	rec_pos_xyz.x = 0.0;// set up user position at earth center, if last valid position is
	// stored in a file, the position will be updated
	rec_pos_xyz.y = 0.0;
	//	rec_pos_xyz.z=0.0;

	//׼������ļ�
	if (out_pos == 1 || out_vel == 1 || out_time == 1)
		fopen_s(&stream, "gpsrcvr.log", "w");//gpsrcvr.log��Ž��ջ��ĵ�ǰ��λʱ�䣬γ���ߣ�XYZ,�������ٶȣ���Ư��H/V/TDOP
	if (out_debug == 1)
		fopen_s(&debug, "debug.log", "w+");


	read_initial_data();

	initSignalSearch();// Ϊ�ź�������̬��������

	//current_loc=receiver_loc();
	rec_pos_llh.lon = current_loc.lon;// ��鱣����ļ����Ƿ���н��ջ���λ��
	rec_pos_llh.lat = current_loc.lat;
	rec_pos_llh.hae = current_loc.hae;

	// ����navFix�ĸ���ʱ������
	nav_tic = (int)(nav_up / 0.1 + 0.5);	// ��Ӧ��nav_up���ڵĲ����ж�����һ��10�Σ���Ϊ�����ж�100����һ�Σ�������Ϊ10����1000�������һ��
	//������Ͳ����ж�ʱ��һ����������Ƶ��
	//�����ж�һ�ξ��ܵõ�һ��α��Ӷ��õ�һ�ζ�λ���������1��10���жϼ��������1��10�ε�ˢ���ʣ�ˢ���ʿ���Ϊ1~10
	programTIC(TIC_ref);	// ��ЧΪ: TIC_CNTR = TIC_ref // TIC������������Ӧ��TIC�жϵķ�������// ע�⣺TIC_CNTR�ĸ�ֵֻ����programTIC����ʵ��

	for (ch = 0; ch <= chmax; ch++)
	{
		chan[ch].state = CHN_OFF;//����ر�״̬����ͨ������
		chan[ch].prn = 0;
	}
	for (ch = MinSvNumber; ch <= MaxSvNumber; ch++)
	{
		svStruct[ch].state = AVAILABLE;//�����������Ǿ�����
		svStruct[ch].undetectedCounter = 0;
		svStruct[ch].NumBTWEachTrial = ReAcqTime;//#define ReAcqTime 1000000LTrialʵ����   �������ز������ޣ�����ĳ���Ǳ��ز���ʱ��
		//NumBTWEachTrial��number between each Trial  BTW:between
		svStruct[ch].NumToTryAcq = 1;
		svStruct[ch].maxf = MAXF;
		svStruct[ch].minf = MINF;
		xyz[ch].azimuth = xyz[ch].doppler = xyz[ch].elevation = 0.0;
		xyz[ch].x = xyz[ch].y = xyz[ch].z = 0.0;
	}

	time(&thetime);
#ifndef REAL_TIME
	initCorrelator();
#endif
	// ��ʼ���ŵ��ķ��䣬���������������ķ��䷽ʽ��ͬ
	hot_cold = 1;
	if (hot_cold == 1)
	{
		ch_alloc();
	}
	else if (hot_cold == 2)
	{
		hot_ch_alloc();
	}
	// double	m_time[3]; // ���ջ�ʱ��,[1]�ǵ�ǰʱ�̵ģ�[0]����һ�ε� [2]�ƺ�û���� 
	//m_time���ڱ�ʾ���ջ���GPSʱ�� ��ԭ���ĳ�����m_timeû����Ч�ĸ���,��Ӧ�ÿ����жϱ�־sec_flag��ά��
	m_time[1] = clock_tow;	// �����Ƶ�GPS�븳ֵ�����ջ�����ʱ��,ע���ⲻ��PC����ʱ�� 

	read_ephemeris();//��"current.eph"�ж�����

#ifndef REAL_TIME
	TIC_RT_CNTR = 0;	// simulator tic counter in correlator
	num = ACC_INT_PERIOD;	// simulator acc_int counter in correlator
	//#define ACC_INT_PERIOD  SAMPLESPMS/2+1			// 0.5ms��������	
	//ÿ��ȡ0.5ms��������ȡ������ۼӣ�����ÿ1ms�����һ����صĽ����������һ������ۼӣ�����������1ms�ۼ�������������ȡ0.5

	initTrackLoopPar();//��ʼ���ز���
#endif

	/*fopen_s(&fpData, if_data_file, "rb");
	if (fpData == NULL)
	{
	perror("Cannot open data file! Program Exited");
	exit(0);
	}

	readFileHeader(fpData);*/
	fopen_s(&fpeph, "monitor.eph", "wb");

	/*if (!fpeph)
	{
		printf("error monitor\n");
	}*/

	return num;
}
int is_state_3 = 0;
void GPSL1_sim_main(long num)
{
	long TIC_DIF;
	long tmp_num;

	//printf("%d %d\n", GPSL1_receiver_buf.buf_remain_len, num);
	while (key != 'x' && key != 'X' && GPSL1_receiver_buf.buf_remain_len >= num)
	{
#ifndef REAL_TIME
		//printf("%d %d\n", GPSL1_receiver_buf.buf_remain_len, num);
		//socket���䶨λ��������������㷨����ʱ��
		if (0 == is_state_3)
		{
			send_res_L1.var1 += num;

			for (int i = 0; i <= chmax; i++)
			{
				if (3 == chan[i].state)
				{
					is_state_3 = 1;
					send_res_L1.var1 /= sameple_rate;

					break;
				}
			}
		}

		tmp_num = num;
		//numRead = fread(data, sizeof(char), num, fpData);//add by jh 4-2

		/*for (int i = 0; i < tmp_num; i++)
		{
			printf("%d ", GPSL1_receiver_buf.buf_current_ptr[i]);
		}
		printf("123\n");*/
		/*if (num == 5001)
		{
		tmp_count++;
		}
		else
		{
		printf("%d\n", tmp_count);
		tmp_count = 0;
		}*/
		//printf("%d\n", numRead);
		/*if (numRead < num)
		{
		write_prn();
		printf("\n reach the end of the data file, process end!");
		break;
		}*/
		// compute how many samples should be read next time
		// generally, the datalength corresponds to an interval of a dump interrupt
		// but when tic interrupt occurs, the data length can be shorter
		// TIC_CNTR+1��TIC_RT_CNTR���Դﵽ�����ֵ���ﵽ���������0
		// TIC_RT_CNTR+num�ǵ����λ�������жϴ����TIC_RT_CNTR����ﵽ��ֵ
		TIC_DIF = TIC_CNTR + 1 - (TIC_RT_CNTR + num);
		//TIC_RT_CNTR TICʵʱ��������
		//TIC_DIF����Ϊ�˼���num
		// �´�Ӧ�ö�ȡ�����������Ȼ�����Щ��ȥ�����
		GPSL1_receiver_init_num = num = TIC_DIF < ACC_INT_PERIOD ? TIC_DIF : ACC_INT_PERIOD;
		//��֤��һ��ʱ������������ݸպôճ�100���룬�����100��������

		GPSL1_data = GPSL1_receiver_buf.buf_current_ptr; 
		//memcpy(data, GPSL1_receiver_buf.buf_current_ptr, tmp_num);

		/*for (int i = 0; i < tmp_num; i++)
		{
			printf("%d ", data[i]);
		}*/
		//printf("123\n");
		correlatorProcess(GPSL1_data, tmp_num);// simulate correlator processing  numRead = read_share_memory(data, num);
		//printf("%d %d\n", GPSL1_receiver_buf.buf_remain_len, num);
		GPSL1_receiver_buf.buf_remain_len -= tmp_num;
		GPSL1_receiver_buf.buf_current_ptr += tmp_num; 
		
		Sim_GPS_Interrupt();	// simulate interrupt processing

#endif
		// ע�⣺�ŵ��������������˷�ʱ�䣬û�б�Ҫÿ����������ж�
		// ����ʱ�ͼ��һ���ŵ�����������֮���Է������ǿ��Ǻ���ʱ
		// �ܹ����ٵر��������ŵ���ʵʱʹ��ʱ������������Ҫ��д��λ��
		// ҲҪ���°���
		for (char ch = 0; ch <= chmax; ch++)
		{
			if (chan[ch].frame_ready == 1)
			{
				navmess(chan[ch].prn, ch);  // decode the navigation message
				chan[ch].frame_ready = 0;   // for this channel
			}
		}


		if (sec_flag == 1)//����һ�����жϱ�־������ά��ϵͳ����ʱ��ͼ�黷·״̬����Sim_GPS_Interrupt��if (tic_count==0) sec_flag=1;		// one second has passed
		{
			almanac_flag = 0;

			thetime++;// ʱ���ȥ��1��  ���⣺����ʵ�ʹ�ȥ��Ӧ�ò���������1�룬Ӧ��С��1�룿
			clock_tow = (++clock_tow) % 604800L;
			time_on++;// ������������Ϊ�룬����ͣ��������ֹ
			sec_flag = 0;
		}
		if (nav_flag == 1)// ���㵼�����ʱ�䵽
		{

			if (_kbhit()) key = _getch();

			nav_fix();
			nav_flag = 0;

			display();

			send_sim_res_socket();
		}

		if (min_flag == 1)// ���������͵���ģʽ�µ��ŵ����亯��
		{
			min_flag = 0;
#ifdef BCPP
			clrscr();
#endif
		}

#ifdef DEMO_CONSOLE
		sharemem_write(1);
#endif



		if (key == 'p' || key == 'P')
		{
			display_page++;
			display_page = display_page % 4;
#ifdef BCPP
			clrscr();
#endif
		}
	}

}

void GPSL1_sim_end()
{

#ifdef DEMO_CONSOLE
	sharemem_write(0);
	sharemem_end();
#endif

	// Remove our interrupt and restore the old one
#ifdef REAL_TIME
	Interrupt_Remove();
#endif

#ifndef REAL_TIME
	shutCorrelator();
#endif

#ifndef REAL_TIME
	// �ͷŶ�������
	freeSignalSearch();
#endif
	write_prn();
	// Update the Almanac Data file
	if (almanac_valid == 1)  write_almanac();
	//  Update the Ephemeris Data file
	write_ephemeris();
	//  Update the ionospheric model and UTC parameters
	write_ion_utc();
	{
		fopen_s(&out, curloc_file, "w+");
		fprintf(out, "latitude  %f\n", rec_pos_llh.lat*r_to_d);
		fprintf(out, "longitude %f\n", rec_pos_llh.lon*r_to_d);
		fprintf(out, "hae       %f\n", rec_pos_llh.hae);
		fclose(out);
	}
	_fcloseall();
	::PostMessage(MainWnd, WM_SIMULATION_STOP, 0, 0);
}

void send_sim_res_socket()
{
	/*���ջ���ʶ*/
	send_res_L1.receiver_id = RECEIVER_L1;

	//�������㷨����ʱ��ֻ����һ��
	//send_res_L1.var1 = 0;

	send_res_L1.dim = m*n;

	//Ȩֵȡ��ǰ��������ݵ����һ��
	if (CONFIG_STAP == (ANTI_CONFIG_SEL & sel))
	{
		
		/*float temp[4] = {};
		cudaMemcpy(temp, STAP_array_matrix_R_inver[0], 4 * sizeof(SIGNAL_TYPE),
			cudaMemcpyDeviceToHost);*/
		cudaMemcpy(send_res_L1.var2_real, STAP_array_matrix_R_inver[0],
			send_res_L1.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);
		
		//��ʼ��ʱ��Ϊ0
		//memset(send_res_L1.var2_image, 0, send_res_L1.dim * sizeof(SIGNAL_TYPE));
	}
	else if (CONFIG_BEAM == (ANTI_CONFIG_SEL & sel))
	{
		cudaMemcpy(send_res_L1.var2_real, STAP_array_matrix_R_inver[0],
			send_res_L1.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);

		cudaMemcpy(send_res_L1.var2_image, STAP_array_matrix_R_inver[0] + 
			send_res_L1.dim, send_res_L1.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);
	}
	else
	{
		//��ʼ��ʱ��Ϊ0
		/*memset(send_res_L1.var2_real, 0, send_res_L1.dim * sizeof(SIGNAL_TYPE));
		memset(send_res_L1.var2_image, 0, send_res_L1.dim * sizeof(SIGNAL_TYPE));*/
	}

	{
		int tmp_count = sameple_rate *1e-3;
		send_res_L1.var3 = 0;
		for (int i = 0; i <tmp_count; i++)
		{
			send_res_L1.var3 += ((SIMRESVAR)GPSL1_receiver_buf.buf_ptr[i]) * 
				((SIMRESVAR)GPSL1_receiver_buf.buf_ptr[i]);
		}
		send_res_L1.var3 /= tmp_count;
		send_res_L1.var3 = 10 * log(send_res_L1.var3);
	}
	
	for (int i = 0; i < SAT_NUM; i++)
	{
		send_res_L1.var4[i] = chan[i].i_prompt;
		send_res_L1.var5[i] = chan[i].CNo;
		send_res_L1.var6[i] = ((long long)(chan[i].phaseLockDetector + 0.5)) & 0xffffffffUL;

		send_res_L1.var7_az[i] = chan[i].az*r_to_d;
		send_res_L1.var7_el[i] = chan[i].el*r_to_d;
		send_res_L1.var8_prn[i] = chan[i].prn;

		send_res_L1.var10_x[i] = track_sat[i + 1].x;
		send_res_L1.var10_y[i] = track_sat[i + 1].y;
		send_res_L1.var10_z[i] = track_sat[i + 1].z;
	}
	send_res_L1.var9_GDOP = gdop;
	send_res_L1.var9_HDOP = hdop;
	send_res_L1.var9_TDOP = vdop;
	send_res_L1.var9_VDOP = tdop;
	send_res_L1.var9_PDOP = pdop;

	send_res_L1.var11_x = navout.X;
	send_res_L1.var11_y = navout.Y;
	send_res_L1.var11_z = navout.Z;

	send_res_L1.var12_he = navout.height;
	send_res_L1.var12_la[0] = cur_lat.deg, send_res_L1.var12_la[1] = abs(cur_lat.min), send_res_L1.var12_la[2] = fabs(cur_lat.sec);
	send_res_L1.var12_lo[0] = cur_long.deg, send_res_L1.var12_lo[1] = abs(cur_long.min), send_res_L1.var12_lo[2] = fabs(cur_long.sec);
	
	send_res_L1.var13[0] = rpvt.xv, send_res_L1.var13[1] = rpvt.yv, send_res_L1.var13[2] = rpvt.zv;
	send_res_L1.var14 = clock_offset;

	localtime_s(&local_time, &thetime);
	send_res_L1.time_y = local_time.tm_year;
	send_res_L1.time_mon = local_time.tm_mon;
	send_res_L1.time_d = local_time.tm_mday;
	send_res_L1.time_h = local_time.tm_hour;
	send_res_L1.time_min = local_time.tm_min;
	send_res_L1.time_s = local_time.tm_sec;

	send_res_L1.var16 = gps_week % 1024;
	send_res_L1.var17 = clock_tow;
	

	sendto(socket_client, (char *)&send_res_L1, sizeof(SimRes), 0, (sockaddr *)&socket_sin, socket_len);
}

/*******************************************************************************
FUNCTION display()
RETURNS  None.

PARAMETERS None.

PURPOSE
This function displays the current status of the receiver on the
computer screen.  It is called when there is nothing else to do

*******************************************************************************/
void display(void)
{

	char ch;
#ifdef BCPP
	gotoxy(1, 1);
#endif

	printf("%s", ctime(&thetime));
	printf("TOW  %6ld\n", clock_tow);                         //������
	printf("meas time %f  error %f \n", m_time[1], m_error);   //                                                                                             [1]:����ʱ�䡣m_error�����ջ��Ĳ���ʱ���GPS��������
	cur_lat.deg = int(rec_pos_llh.lat*r_to_d);
	cur_lat.min = int((rec_pos_llh.lat*r_to_d - cur_lat.deg) * 60);
	cur_lat.sec = float((rec_pos_llh.lat*r_to_d - cur_lat.deg - cur_lat.min / 60.)*3600.);
	cur_long.deg = int(rec_pos_llh.lon*r_to_d);
	cur_long.min = int((rec_pos_llh.lon*r_to_d - cur_long.deg) * 60);
	cur_long.sec = float((rec_pos_llh.lon*r_to_d - cur_long.deg - cur_long.min / 60.)*3600.);
	printf("   latitude    longitude          HAE      clock error (ppm)\n");
	printf("  %4d:%2d:%5.2f  %4d:%2d:%5.2f  %10.2f  %f\n",
		cur_lat.deg, abs(cur_lat.min), fabs(cur_lat.sec), cur_long.deg, abs(cur_long.min),
		fabs(cur_long.sec), rec_pos_llh.hae, clock_offset); //clock_offset:���ջ���Ư
	printf(" Speed      vx		vy	  vz      Heading      TIC_dt\n");
	printf(" %lf   %lf   %lf   %lf   %lf   %lf\n", speed, rpvt.xv, rpvt.yv, rpvt.zv, heading*r_to_d, TIC_dt);  //speed:���ջ��ٶȵľ���ֵ;heading�����ջ��ķ�λ�ǣ�TIC_dt�����μ��㵼����ļ��
	printf("   \n");

	printf("tracking %2d status %1d almanac valid %1d gps week %4d\n",
		n_track, status, almanac_valid, gps_week % 1024);

	if (display_page == 0)
	{
		printf(" ch prn state az  el  doppler  t_count n_frame sfid ura page CNo  PL\n");
		for (ch = 0; ch <= chmax; ch++)
		{
			printf(" %2d %2d  %2d  %4.0f  %3.0f   %7.1f   %4d  %4d  %2d  %3d  %3d   %4.1f  %1d\n",
				ch, chan[ch].prn, chan[ch].state,
				chan[ch].az*r_to_d, chan[ch].el*r_to_d,                                //az:���ǵķ�λ�ǣ�el�����ǵ�����
				chan[ch].vDoppler, chan[ch].t_count, chan[ch].n_frame, chan[ch].sfid,    //vDoppler:�����˶�������doppler����;t_count:�յ��ı��ؼ���������ѭ����ʽ����0-1499
				gps_eph[chan[ch].prn].ura, chan[ch].page5, chan[ch].CNo, (long)((long long)(chan[ch].phaseLockDetector + 0.5) & 0xffffffffUL));
			//ura���û����뾫��ָ����page5�����������е�page�ţ�CNO�����Ƶ��ŵ�����ȣ�phaseLockDetector���ز���λ�Ƿ�������ָʾ

			chinfo[ch].prn = chan[ch].prn;
			chinfo[ch].state = chan[ch].state;
			chinfo[ch].az = chan[ch].az;
			chinfo[ch].el = chan[ch].el;
			chinfo[ch].doppler = chan[ch].doppler;
			chinfo[ch].t_count = chan[ch].t_count;
			chinfo[ch].n_frames = chan[ch].n_frame;     // �Ѿ��յ���֡����
			chinfo[ch].sfid = chan[ch].sfid;        // ��֡�ı��
			chinfo[ch].ura = gps_eph[chan[ch].prn].ura;
			chinfo[ch].page = chan[ch].page5;
			chinfo[ch].CN0 = chan[ch].CNo;

			CNR[ch] = chinfo[ch].CN0;

			corrpeak[ch].i_early = chan[ch].i_early;
			corrpeak[ch].i_prompt = chan[ch].i_prompt;//��ط�ֵ
			corrpeak[ch].i_late = chan[ch].i_late;
		}

		/*BEAM*/
		if (CONFIG_BEAM == (ANTI_CONFIG_SEL & sel))
		{ 
			bool is_EL_AZ_changed = false;
		
			int tmp_AZ_sj;
			int tmp_EL_si;

			for (ch = 0; ch <= chmax; ch++)
			{
				//�޸�ȫ�ֵķ�λ�Ǻ͸�����
				tmp_AZ_sj = chan[ch].az*r_to_d;
				tmp_EL_si = 90 - chan[ch].el*r_to_d;
				if ((tmp_AZ_sj != AZ_sj[ch]) || (tmp_EL_si != EL_si[ch]))
				{
					is_EL_AZ_changed = true;

					AZ_sj[ch] = tmp_AZ_sj;
					EL_si[ch] = tmp_EL_si;
				}
			}

			if (is_EL_AZ_changed)
			{
				beam_s_vector(EL_si, AZ_sj, lv / TF / 2, m, n, array_antenna, sameple_rate, IF);
			}
		}
		/*BEAM*/

		printf("  prn     Track_sat X      Track_sat Y      Track_sat Z\n");

		for (ch = 1; ch <= chmax + 1; ch++)
		{

			printf("   %2d    %lf    %lf    %lf\n",
				chan[tr_ch[ch]].prn, track_sat[ch].x, track_sat[ch].y, track_sat[ch].z);
		}

		printf(" GDOP=%6.3f  HDOP=%6.3f  VDOP=%6.3f  TDOP=%6.3f PDOP=%6.3f\n", gdop, hdop, vdop, tdop, pdop);
	}
	else if (display_page == 1)
	{
		printf(" ch prn state TLM      TOW  Health  Valid  TOW_sync offset\n");
		for (ch = 0; ch <= chmax; ch++)
		{
			printf(" %2d %2d  %2d  %6ld   %6ld   %2d     %2d     %2d   %4d\n",
				ch, chan[ch].prn, chan[ch].state, chan[ch].TLM, chan[ch].TOW,
				gps_eph[chan[ch].prn].health, gps_eph[chan[ch].prn].valid, chan[ch].tow_sync,
				chan[ch].offset);
		}
	}
	else if (display_page == 2)
	{
		printf(" ch prn state n_freq az  el        tropo        iono\n");

		for (ch = 0; ch <= chmax; ch++)
		{
			printf(" %2d %2d  %2d  %4.0f  %3.0f   %10.4lf   %10.4lf\n",
				ch, chan[ch].prn, chan[ch].state,
				xyz[chan[ch].prn].azimuth*r_to_d, xyz[chan[ch].prn].elevation*r_to_d,
				chan[ch].Tropo*SPEEDOFLIGHT, chan[ch].Iono*SPEEDOFLIGHT);
		}		
	}
	else if (display_page == 3)
	{
		printf(" ch prn state      Pseudorange     delta Pseudorange\n");
		for (ch = 0; ch <= chmax; ch++)
		{
			printf(" %2d %2d  %2d  %20.10lf   %15.10lf\n",
				ch, chan[ch].prn, chan[ch].state, chan[ch].Pr, chan[ch].dPr);
		}
	}
	else if (display_page == 4)
	{
	}
	//add by jh ui
	//::PostMessage(ChannelWnd, WM_CHANNEL_INFORMATION, (WPARAM)chinfo, (LPARAM)12);

	sprintf(navout.lat, "%4d:%2d:%5.2f", cur_lat.deg, abs(cur_lat.min), fabs(cur_lat.sec));
	sprintf(navout.lon, "%4d:%2d:%5.2f", cur_long.deg, abs(cur_long.min), fabs(cur_long.sec));
	navout.height = rec_pos_llh.hae;
	navout.gdop = gdop;
	navout.hdop = hdop;
	navout.vdop = vdop;
	navout.tdop = tdop;
	strcpy(navout.time, ctime(&thetime));
	navout.ve = receiver.vel.east;
	navout.vn = receiver.vel.north;
	navout.vu = receiver.vel.up;

	XYZ wgs;

	wgs = llh_to_ecef(rec_pos_llh);
	navout.X = wgs.x;
	navout.Y = wgs.y;
	navout.Z = wgs.z;
	printf("X: %lf, Y: %lf, Z: %lf.\n", navout.X, navout.Y, navout.Z);

	//::PostMessage(NavOutWnd, WM_NAVIGATION_OUT, (WPARAM)&navout, 0);

	//::PostMessage(CNRatioWnd,WM_CNRATIO, (WPARAM)CNR, 0);

	//::PostMessage(CPWnd, WM_CP, (WPARAM)corrpeak, (LPARAM)3);

}

/*******************************************************************************
FUNCTION Interrupt_Install()
RETURNS  None.

PARAMETERS None.

PURPOSE
This function replaces the current IRQ0 Interrupt service routine with
our own custom function. The old vector is stored in a global variable
and will be reinstalled at the end of program execution. IRQ0 is
enabled by altering the interrupt mask stored by the 8259 interrupt
handler.

*******************************************************************************/
#ifdef REAL_TIME
void Interrupt_Install()
{
	unsigned char     int_mask, i_high, i_low;
	i_high = interr_int >> 8;
	i_low = interr_int & 0xff;
	Old_Interrupt = getvect(8 + IRQLEVEL);
	disable();
	setvect(8 + IRQLEVEL, GPS_Interrupt);
	int_mask = inportb(0x21);
	int_mask = int_mask & ~(1 << IRQLEVEL);
	outportb(0x21, int_mask);
	enable();

	outportb(0x43, 0x34);
	outportb(0x40, i_low);
	outportb(0x40, i_high);
	outportb(0x20, 0x20);
}
#endif

/*******************************************************************************
FUNCTION Interrupt_Remove()
RETURNS  None.

PARAMETERS None.

PURPOSE
This function removes the custom interrupt vector from the vector
table and restores the previous vector.

*******************************************************************************/
#ifdef REAL_TIME
void Interrupt_Remove()
{
	unsigned char     int_mask;

	outportb(0x20, 0x20);           // clear interrupt and allow next one
	int_mask = inportb(0x21);      // get hardware interrupt mask
	int_mask = int_mask | (1 << IRQLEVEL);
	disable();
	//outportb(0x21,int_mask);       // send new mask to 8259
	setvect(8 + IRQLEVEL, Old_Interrupt);
	enable();                      // allow hardware interrupts
	outportb(0x20, 0x20);           // clear interrupt and allow next one
	outportb(0x43, 0x34);           // reset clock
	outportb(0x40, 0xff);
	outportb(0x40, 0xff);
}
#endif

/*******************************************************************************
FUNCTION GPS_Interrupt()

RETURNS  None.

PARAMETERS None.

PURPOSE
This function replaces the current IRQ0 Interrupt service routine with
our GPS function which will perform the acquisition - tracking functions
������������ǵ�ִ�в�����ٵ�GPS����������Ŀǰ��IRQ0�жϷ������
*******************************************************************************/
#ifdef REAL_TIME
void interrupt GPS_Interrupt(...)
#else
void Sim_GPS_Interrupt()
#endif
{
	unsigned int add;
	char ch;
	double carrierPhase;



	bool bAcq = false;
	for (ch = 0; ch <= chmax; ch++)
	{
		if (correlator[ch].ready == 1)
		{
			ReadAccumData(ch);//���������1����Ļ���ֵ
			/************************************************************************/
			correlator[ch].ready = 0;

			switch (chan[ch].state)
			{
			case acquisition:
				ch_acq(ch);
				break;
			case confirm:
				break;
			case pull_in:	// pull-inģ��Ҫ���λͬ���Ĺ���
				ch_pull_in(ch);
				break;
			case track:
				ch_track(ch);
				break;
			}
		}
	}

	if (TIC_OCCUR_FLAG == 1)
	{
		tic_count = (++tic_count) % 10;
		if (tic_count == 0) sec_flag = 1;

		hms_count = (++hms_count) % 600;
		if (hms_count == 0) min_flag = 1;

		nav_count = (++nav_count) % nav_tic;
		if (nav_count == 0) nav_flag = 1;

		TIC_sum += TIC_CNTR + 1;
		add = 1;
		for (ch = 0; ch <= chmax; ch++)
		{
			if (chan[ch].state != track)
			{
				continue;
			}
			carrierPhase = readCarrierPhase(ch);
			chan[ch].cycle_sum += ((long)((long long)carrierPhase & 0xffffffffUL));
			chan[ch].carr_dco_phase = fmod(carrierPhase, 1.0);
		}

		if (nav_count == 0)
		{
			for (ch = 0; ch <= chmax; ch++)
			{
				if (chan[ch].state != track)
				{
					continue;
				}
				chan[ch].codePhase = readCodePhase(ch);

				chan[ch].epoch = readEpoch(ch);

				chan[ch].meas_bit_time = chan[ch].tr_bit_time;

				chan[ch].doppler = chan[ch].fc;
				chan[ch].carrier_counter = chan[ch].cycle_sum;
				chan[ch].cycle_sum = 0;
				chan[ch].d_carr_phase = chan[ch].carr_dco_phase - chan[ch].old_carr_dco_phase;
				chan[ch].old_carr_dco_phase = chan[ch].carr_dco_phase;
			}

#ifdef REAL_TIME
#else
			i_TIC_dt = TIC_sum;
#endif

			TIC_sum = 0;
		}
		TIC_OCCUR_FLAG = 0;
		{
			ch_alloc();
		}
	}

}

void initTrackLoopPar()
{
	double DLLK1, DLLK2;
	double BDLL, DLLkesi;
	double BPLL, PLLomega0;
	double BFLL, FLLkesi;

	long i, ch;
	// DLL parameters
	DLLK1 = 1.0;
	DLLK2 = 1.0;
	BDLL = 0.5;
	DLLkesi = pow(2.0, -0.5);

	DLLc1 = pow((8 * DLLkesi*BDLL / (1 + 4 * DLLkesi*DLLkesi)), 2.0) / (DLLK1*DLLK2);
	DLLc2 = 16 * DLLkesi*DLLkesi*BDLL / (1 + 4 * DLLkesi*DLLkesi) / DLLK1 / DLLK2;

	BPLL = 15.0;
	PLLomega0 = BPLL / 0.78445;
	PLLc0 = pow(PLLomega0, 3.0);
	PLLc1 = 1.1*PLLomega0*PLLomega0;
	PLLc2 = 2.4*PLLomega0;
	FLLkesi = 0.707;
	BFLL = 10.0;
	//changed by Ning Luo in Sept/04, 5.0Hz->10Hz

	FLLa1 = pow((8 * FLLkesi*BFLL / (1 + 4 * FLLkesi*FLLkesi)), 2.0);
	FLLa2 = 16 * FLLkesi*FLLkesi*BFLL / (1 + 4 * FLLkesi*FLLkesi);

	for (ch = 0; ch <= chmax; ch++)
	{
		chan[ch].BitSyncIndex = 0;
		chan[ch].IQBufSize = 0;
		for (i = 0; i<BUFSIZE; i++)
		{
			chan[ch].signBuf[i] = 0;
		}
		for (i = 0; i<20; i++)
		{
			chan[ch].kCell[i] = 0;
		}
	}
}
void hot_ch_alloc()
{
	char i;
	short allocFlag;

	read_prn();
	for (i = 0; i <= chmax; i++)
	{
		if (chan[i].state != CHN_OFF)
		{
			continue;
		}
		allocFlag = 0;

		if (last_prn[i]>0)
		{
			allocFlag = 1;
			svStruct[i].state = USING;
			chan[i].prn = last_prn[i];
			chan[i].state = acquisition;
			svStruct[i].NumToTryAcq = 20;
			svStruct[i].NumBTWEachTrial = 1000;
			correlator[i].state = CHN_OFF;
			correlator[i].ready = 1;
			correlator[i].sv = last_prn[i];
		}

	}

}
void ch_alloc()//һ�������жϽ���һ��
{
	char i, j;
	short allocFlag;

	almanac_valid = 1;// ���Almanac�Ƿ���Ч
	for (i = MinSvNumber; i <= MaxSvNumber; i++)//��32��ͨ��
	{
		xyz[i] = satfind(i);//�ڴ�ch_alloc����֮ǰ��time(&thetime);
		if (gps_alm[i].inc>0.0 && gps_alm[i].week != gps_week % 1024)//�ж�������Ч��
		{
			almanac_valid = 0;
			break;
		}
	}
	if (al0 == 0.0 && b0 == 0.0) almanac_valid = 0;

	// ������е����ǣ����ĳЩ����δ����⵽�źţ�����Щ���Ǳ��뱻����һ��ʱ��
	// �Ա�֤�����������л��ᱻ����������
	for (i = MinSvNumber; i <= MaxSvNumber; i++)
	{
		if (svStruct[i].state == HOLD)//HOLD����fft����û��ֱ�Ӳ��񵽷�ֵ�����õ�
		{
			if (svStruct[i].NumToTryAcq == 0)//�����������
			{
				svStruct[i].NumToTryAcq = 1;
				svStruct[i].NumBTWEachTrial = ReAcqTime;
				svStruct[i].maxf = MAXF;
				svStruct[i].minf = MINF;
			}
			svStruct[i].undetectedCounter++;
			if (svStruct[i].undetectedCounter>svStruct[i].NumBTWEachTrial)
			{
				svStruct[i].state = AVAILABLE;
				svStruct[i].undetectedCounter = 0;
			}
		}
	}

	for (i = 0; i <= chmax; i++)
	{
		if (chan[i].state != CHN_OFF)// ����ŵ��Ѿ���⵽���ź�/��ռ�ã����������ŵ��ļ��
		{
			continue;
		}

		// Ϊ�����ŵ�ѡ��һ������
		// check all satellites
		allocFlag = 0;
		for (j = MinSvNumber; j <= MaxSvNumber; j++)
		{
			if (almanac_valid == 1)// ���Almanac�Ƿ���Ч
			{
				if (xyz[j].elevation > mask_angle &&
					gps_alm[j].health == 0 &&
					gps_alm[j].ety != 0.00 &&
					svStruct[j].state == AVAILABLE)
				{
					allocFlag = 1;
					svStruct[j].state = USING;
					chan[i].prn = j;
					chan[i].state = acquisition;
					svStruct[j].NumToTryAcq = 20;//���������Ϣ�У���ÿ�����ǵ��Բ�����Ϊ20�Σ�����ֻ�Բ�һ�Σ�ֱ���´�ch_alloc�����ڸ����ﵱ�����<30ʱ����Ϊ20��
					svStruct[j].NumBTWEachTrial = 1000;
					correlator[i].state = CHN_OFF;
					correlator[i].ready = 1;
					ch_cntl(i, j);
					break;
				}
			}
			else
			{
				if (svStruct[j].state == AVAILABLE)
				{
					allocFlag = 1;
					svStruct[j].state = USING;
					chan[i].prn = j;
					chan[i].state = acquisition;
					if (svStruct[j].NumToTryAcq == 0)
					{
						svStruct[j].NumToTryAcq = 1;
						svStruct[j].NumBTWEachTrial = ReAcqTime;//1000000
						svStruct[j].maxf = MAXF;
						svStruct[j].minf = MINF;
					}
					correlator[i].state = CHN_OFF;
					correlator[i].ready = 1;
					ch_cntl(i, j);
					break;
				}
			}
		}
		if (allocFlag == 0) break;
	}
}



// linear FFT detector
char first_flag = 1;
void ch_acq(char ch)
{
	double Rmax;
	double tau, doppler;
	long dataLength;
	int flag;
	long bufOffset;
	long numFreqBin, i;
	long nTrial;
	double freqOffset[100];
	char first_flag = 1;
	double *pBuf;
	char *pData;
	//st-atic double BUFTABLE[2] = { -1.0, 1.0 };//add by jh ui

	pBuf = &buffer[ch][globalBufLength[ch]];
	pData = GPSL1_data; 
	for (i = 0; i<correlatorDataLength; i++)
	{
		//*pBuf++ = BUFTABLE[*pData++];
		*pBuf++ = *pData++; 
	}

	globalBufLength[ch] += correlatorDataLength;

#ifndef REAL_TIME 
	correlator[ch].ready = 1;
#endif

	if (globalBufLength[ch] < DETECTSIZE)
		return;
	if (globalBufLength[ch] > (DETECTSIZE + 5000))
	{
		printf("computation abnormal, system exits");
		exit(0);
	}

	bufOffset = globalBufLength[ch] - DETECTSIZE;
	dataLength = DETECTSIZE;
	numFreqBin = (long)((long long)((svStruct[chan[ch].prn].maxf - svStruct[chan[ch].prn].minf) / DELTF + 1.5) & 0xffffffffUL);
	for (i = 0; i<numFreqBin; i++)
		freqOffset[i] = i*DELTF + svStruct[chan[ch].prn].minf;

	if (first_flag == 1)
	{
		double threshold = 0.0;
		flag = acqCA(&buffer[ch][bufOffset], dataLength, caTable, 37, SAMPLING_FREQ, freqOffset, numFreqBin,
			CARRIER_FREQ, &tau, &doppler, &nTrial, &Rmax, &threshold);
		threshold_sig_search = Rmax*1.5f;
		first_flag = 0;
	}
	flag = acqCA(&buffer[ch][bufOffset], dataLength, caTable, chan[ch].prn, SAMPLING_FREQ, freqOffset, numFreqBin,
		CARRIER_FREQ, &tau, &doppler, &nTrial, &Rmax, &threshold_sig_search);

	globalBufLength[ch] = 0;
	if (flag == DETECTED)
	{

		writeCodePhase(ch, tau);

		writeCodeFreq(ch, 1.0);

		writeCarrierFreq(ch, carrier_ref + doppler);
#ifndef REAL_TIME
		correlator[ch].state = CHN_ON;
		correlator[ch].ready = 0;
#endif
		chan[ch].state = pull_in;
		chan[ch].cLoopS1 = doppler;
		chan[ch].cLoopS0 = 0;
		chan[ch].dllS0 = 0.0;
		chan[ch].dllS1 = 0.0;
		detectNum[ch] = 0;
		chan[ch].FLLIndex = 1;
		chan[ch].FLLFlag = 1;
		chan[ch].ch_time = 0;
		chan[ch].freqErr = 0.0;
		chan[ch].fc = doppler;
	}
	else if (flag == UNDETECTED)
	{
		svStruct[chan[ch].prn].NumToTryAcq--;
		{
			chan[ch].state = CHN_OFF;
			correlator[ch].state = CHN_OFF;
			correlator[ch].ready = 1;
			svStruct[chan[ch].prn].state = HOLD;
			svStruct[chan[ch].prn].undetectedCounter = 0;
			detectNum[ch] = 0;
			chan[ch].prn = 0;
		}
	}
}



void ch_pull_in(char ch)
{
	double sER, sEI, sLR, sLI, sPR, sPI;
	double absSE, absSL;
	double codeErr, T, phaseErr, freqErr;
	double *dllS0, *dllS1;
	double *cLoopS0, *cLoopS1;
	double cross, dot, phaseDiff;

	chan[ch].ch_time++;//��Ϊ��������ж���1����һ�Σ�����һ���������һ��
	// ���tau�ǳ��ӽ���1023,���һ��IQ����ֵ�ܿ���ֻ�����˺��ټ������㣬���
	// ������ܴ󣬲����á�    ��Ϊ������������ж�tau�ĸ�32�Ƿ�216888����tau�ǲ��񵽵Ĵӳ�ʼ����λ��ʼ�ۼӵģ����Կ��ܽӽ�1023����������-216888�������´ν�����ӽ�1023
	if (chan[ch].ch_time == 1)
	{
		return;
	}

	sER = chan[ch].i_early;
	sEI = chan[ch].q_early;
	sLR = chan[ch].i_late;
	sLI = chan[ch].q_late;
	sPR = chan[ch].i_prompt;
	sPI = chan[ch].q_prompt;

	// below added by Ning Luo in Sept/04
	if (chan[ch].ch_time == 2)
	{
		chan[ch].freqErr = 0;
		chan[ch].i_old = chan[ch].i_prompt;
		chan[ch].q_old = chan[ch].q_prompt;
		minFreqErr[ch] = M_PI;
		maxFreqErr[ch] = -M_PI;
		return;
	}

	if (chan[ch].ch_time<FINE_FREQ_RESOLUTION + 2)
	{
		cross = chan[ch].i_old*sPI - chan[ch].q_old*sPR;
		dot = chan[ch].i_old*sPR + chan[ch].q_old*sPI;
		phaseDiff = atan2(cross, dot);

		if (phaseDiff>maxFreqErr[ch]) maxFreqErr[ch] = phaseDiff;
		if (phaseDiff<minFreqErr[ch]) minFreqErr[ch] = phaseDiff;
		chan[ch].freqErr += phaseDiff;
		chan[ch].i_old = chan[ch].i_prompt;
		chan[ch].q_old = chan[ch].q_prompt;
	}
	else if (chan[ch].ch_time == FINE_FREQ_RESOLUTION + 2)
	{

		T = 1e-3;
		freqErr = (chan[ch].freqErr - maxFreqErr[ch] - minFreqErr[ch]) / (FINE_FREQ_RESOLUTION - 3.0);
		freqErr = freqErr / (2 * M_PI*T);
		chan[ch].fc += freqErr;
		chan[ch].cLoopS1 = chan[ch].fc;
		chan[ch].carrier_freq = carrier_ref + chan[ch].fc;
		writeCarrierFreq(ch, chan[ch].carrier_freq);
	}
	// above added by Ning Luo in Sept/04

	else if (chan[ch].ch_time>FINE_FREQ_RESOLUTION + 2)
	{
		/////////////////////////////////////////////////////////////////////////////////
		T = 1e-3;

		codeErr = (sER*sPR + sEI*sPI) / (sER*sER + sPR*sPR + sEI*sEI + sPI*sPI)*(DLLdT / (2.0*1.023e6));
		/************************************************************************/
		dllS0 = &(chan[ch].dllS0);
		dllS1 = &(chan[ch].dllS1);
		*dllS0 += DLLc1*T*codeErr;
		*dllS1 = *dllS0 + DLLc2*codeErr + chan[ch].fc / 1575.42e6;
		chan[ch].code_freq = (1 + (*dllS1));
		writeCodeFreq(ch, chan[ch].code_freq);
		/////////////////////////////////////////////////////////////////////////////////

		if (chan[ch].ch_time>PULL_IN_TIME) //#define PULL_IN_TIME (long) 1000		// FLL������ʱ�䣬��λ��ms
		{
			freqErr = 0.0;
		}
		else
		{
			chan[ch].FLLIndex = 1 - chan[ch].FLLIndex;
			//����ch_acq����1��Ȼ���������0������һ��1-0�ֱ�1��Ȼ�����´��ֱ�0�����������ifÿ��һ�ν�һ��

			if (chan[ch].FLLFlag == 1 && chan[ch].FLLIndex == 1)
			{
				cross = chan[ch].i_old*sPI - chan[ch].q_old*sPR;
				dot = chan[ch].i_old*sPR + chan[ch].q_old*sPI;
				if ((cross*cross + dot*dot)<1.0)
					phaseDiff = 0.0;
				else
					phaseDiff = dot >= 0.0 ? cross / sqrt(dot*dot + cross*cross) : -cross / sqrt(dot*dot + cross*cross);
				freqErr = phaseDiff / (twoPI*T);
				chan[ch].freqErr = freqErr;
			}
			else
			{

				freqErr = 0;
			}
			chan[ch].i_old = chan[ch].i_prompt;
			chan[ch].q_old = chan[ch].q_prompt;
		}
		if (fabs(sPR)<1e-3)
		{
			phaseErr = 0.0;
		}
		else
			phaseErr = atan(sPI / sPR);

		cLoopS0 = &(chan[ch].cLoopS0);
		cLoopS1 = &(chan[ch].cLoopS1);

		*cLoopS0 = *cLoopS0 + (phaseErr*PLLc0 + freqErr*FLLa1)*T;
		*cLoopS1 = *cLoopS1 + (PLLc1*phaseErr + (*cLoopS0) + FLLa2*freqErr)*T;
		chan[ch].fc = *cLoopS1 + PLLc2*phaseErr;
		chan[ch].carrier_freq = carrier_ref + chan[ch].fc;
		writeCarrierFreq(ch, chan[ch].carrier_freq);


		if (chan[ch].ch_time>PULL_IN_TIME)//1s
		{
			bitSync(ch);
			if (chan[ch].state == track)
				return;
		}
	}
}


void bitSync(char ch)
{
	long	*index;
	char	oldSign, newSign;
	short	counter1, counter2, counter, signIndex;
	short	i;
	double  absAngle;
	double  realP, imagP;
	long	tempIndex;
	long    sPI, sPR;
	double  sPI20, sPR20;

	index = &(chan[ch].BitSyncIndex);
	chan[ch].IBuf[*index] = chan[ch].i_prompt;
	chan[ch].QBuf[*index] = chan[ch].q_prompt;

	oldSign = chan[ch].signBuf[*index];

	tempIndex = (*index + BUFSIZE - 1) % BUFSIZE;
	realP = (double)chan[ch].i_prompt*(double)chan[ch].IBuf[tempIndex]
		+ (double)chan[ch].q_prompt*(double)chan[ch].QBuf[tempIndex];
	imagP = (double)chan[ch].i_prompt*(double)chan[ch].QBuf[tempIndex]
		- (double)chan[ch].q_prompt*(double)chan[ch].IBuf[tempIndex];
	absAngle = fabs(atan2(imagP, realP)) - 3.1415926 / 2;
	newSign = absAngle >= 0 ? 1 : 0;
	chan[ch].signBuf[*index] = newSign;
	chan[ch].kCell[*index % 20] += (newSign - oldSign);
	*index = (*index + 1) % BUFSIZE;
	chan[ch].IQBufSize++;

	if (chan[ch].IQBufSize<BUFSIZE) return;
	chan[ch].IQBufSize = BUFSIZE;

	counter1 = 0;
	counter2 = 0;
	for (i = 0; i<20; i++)
	{
		if (chan[ch].kCell[i] >= NBS2)
		{
			if (chan[ch].kCell[i] >= NBS1)
			{
				counter1++;
				signIndex = i;
			}
			else
			{
				counter2++;
			}
		}
	}

	if (counter2 >= 2)
		return;

	if (counter1 != 1)
		return;
	chan[ch].ms_count = (*index - 1 + BUFSIZE - signIndex) % 20;

	if (chan[ch].ms_count != 19) return;

	writeEpoch(ch, chan[ch].ms_count);

	counter = BUFSIZE;
	chan[ch].t_count = 0;
	chan[ch].subFrameSyncFlag = SUBFRAME_SYNCHRONIZING;
	chan[ch].WBP = chan[ch].NP = 0.0;


	while (counter>0)
	{
		chan[ch].i_prom_20 = 0;
		chan[ch].q_prom_20 = 0;
		do
		{
			sPR = chan[ch].IBuf[*index];
			sPI = chan[ch].QBuf[*index];
			chan[ch].i_prom_20 += sPR;
			chan[ch].q_prom_20 += sPI;

			chan[ch].WBP += (double)sPR*sPR + (double)sPI*sPI;

			(*index)++;
			*index %= BUFSIZE;
			counter--;
		} while ((*index) % 20 != signIndex && counter>0);
		if (counter == 0 && (*index) % 20 != signIndex)
		{

			break;
		}
		else
		{
			sPI20 = chan[ch].q_prom_20;
			sPR20 = chan[ch].i_prom_20;
			chan[ch].NBP = sPR20*sPR20 + sPI20*sPI20;
			chan[ch].NP += (chan[ch].NBP / chan[ch].WBP / (BUFSIZE / 20));
			chan[ch].WBP = 0.0;

			absAngle = fabs(atan2(sPI20, sPR20));
			chan[ch].bit = absAngle> 3.1415926 / 2 ? 0 : 1;
			pream(ch, chan[ch].bit);
			chan[ch].message[chan[ch].t_count++] = chan[ch].bit;

		}
	}

	if (chan[ch].NP - 1.0<0)
		chan[ch].CNo = 0.0;
	else
		chan[ch].CNo = 10 * log10(1000.0*(chan[ch].NP - 1.0) / (20 - chan[ch].NP));

	chan[ch].BitSyncIndex = 0;
	chan[ch].IQBufSize = 0;
	for (i = 0; i<BUFSIZE; i++)
	{
		chan[ch].signBuf[i] = 0;
	}
	for (i = 0; i<20; i++)
	{
		chan[ch].kCell[i] = 0;
	}

	chan[ch].i_prom_20 = 0;
	chan[ch].q_prom_20 = 0;
	chan[ch].i_early_20 = 0;
	chan[ch].q_early_20 = 0;
	chan[ch].i_late_20 = 0;
	chan[ch].q_late_20 = 0;
	chan[ch].WBP = chan[ch].NBP = chan[ch].NP = chan[ch].NBD = 0.0;

	chan[ch].trackTime = 0;
	chan[ch].state = track;
}


/*******************************************************************************
FUNCTION ch_track(char ch)
RETURNS  None.

PARAMETERS
ch  char  channel number

PURPOSE  to track in carrier and code the GPS satellite and partially
decode the navigation message (to determine TOW, subframe etc.)

*******************************************************************************/
void ch_track(char ch)
{
	double sER, sEI, sLR, sLI, sPR, sPI;
	double sPR20, sPI20, sER20, sEI20, sLR20, sLI20;
	double absSE, absSL;
	double codeErr, T, phaseErr;
	double *dllS0, *dllS1;
	double *cLoopS0, *cLoopS1;
	double absAngle;
	CHANNEL* pChan = &chan[ch];

	chan[ch].trackTime++;

	sER = pChan->i_early;
	sEI = pChan->q_early;
	sLR = pChan->i_late;
	sLI = pChan->q_late;
	sPR = pChan->i_prompt;
	sPI = pChan->q_prompt;

	T = 1e-3;


	if (fabs(sPR)<1e-3)
	{
		phaseErr = 0.0;
	}
	else
		phaseErr = atan(sPI / sPR);


	cLoopS0 = &(pChan->cLoopS0);
	cLoopS1 = &(pChan->cLoopS1);

	*cLoopS0 = *cLoopS0 + phaseErr*PLLc0*T;
	*cLoopS1 = *cLoopS1 + (PLLc1*phaseErr + (*cLoopS0))*T;
	pChan->fc = *cLoopS1 + PLLc2*phaseErr;
	pChan->carrier_freq = carrier_ref + pChan->fc;
	writeCarrierFreq(ch, pChan->carrier_freq);
	pChan->ms_count = (++pChan->ms_count) % 20;

	pChan->i_prom_20 += pChan->i_prompt;
	pChan->q_prom_20 += pChan->q_prompt;
	pChan->i_early_20 += pChan->i_early;
	pChan->q_early_20 += pChan->q_early;
	pChan->i_late_20 += pChan->i_late;
	pChan->q_late_20 += pChan->q_late;
	pChan->WBP += sPR*sPR + sPI*sPI;

	if (pChan->ms_count == 19)
	{

		T = 0.02;
		sPI20 = pChan->q_prom_20;
		sPR20 = pChan->i_prom_20;
		sLI20 = pChan->q_late_20;
		sLR20 = pChan->i_late_20;
		sEI20 = pChan->q_early_20;
		sER20 = pChan->i_early_20;

		codeErr = (sER20*sPR20 + sEI20*sPI20) / (sER20*sER20 + sPR20*sPR20 + sEI20*sEI20 + sPI20*sPI20)*(DLLdT / (2.0*1.023e6));

		dllS0 = &(pChan->dllS0);
		dllS1 = &(pChan->dllS1);
		*dllS0 += DLLc1*T*codeErr;
		*dllS1 = *dllS0 + DLLc2*codeErr
			+ pChan->fc / 1575.42e6;
		pChan->code_freq = 1 + (*dllS1);
		writeCodeFreq(ch, pChan->code_freq);


		pChan->NBP = sPI20*sPI20 + sPR20*sPR20;
		pChan->NP += (0.02 * pChan->NBP / pChan->WBP);
		pChan->WBP = 0.0;


		pChan->NBD = sPR20*sPR20 - sPI20*sPI20;
		pChan->phaseLockDetector = pChan->NBD / pChan->NBP;

		if (pChan->trackTime % 1000 == 0 && pChan->trackTime>0)
		{

			if (chan[ch].NP - 1.0<0)
				chan[ch].CNo = 0.0;
			else
				pChan->CNo = 10 * log10(1000.0*(pChan->NP - 1.0) / (20.0 - pChan->NP));
			pChan->NP = 0.0;
			if (pChan->CNo<30)
			{
				pChan->state = acquisition;
				pChan->phaseLockDetector = 0.0;
				pChan->az = 0;
				pChan->el = 0;
				svStruct[pChan->prn].NumToTryAcq = 20;
				svStruct[pChan->prn].NumBTWEachTrial = 1000;
				svStruct[pChan->prn].maxf = (pChan->fc + DELTF);
				svStruct[pChan->prn].minf = (pChan->fc - DELTF);
				correlator[ch].state = CHN_OFF;
				correlator[ch].ready = 1;
			}
		}


		pChan->tr_bit_time++;
		absAngle = fabs(atan2((double)pChan->q_prom_20, (double)pChan->i_prom_20));
		pChan->bit = absAngle>3.1415926 / 2 ? 0 : 1;

		pream(ch, pChan->bit);

		pChan->message[pChan->t_count++] = pChan->bit;

		pChan->i_prom_20 = 0;
		pChan->q_prom_20 = 0;
		pChan->i_early_20 = 0;
		pChan->q_early_20 = 0;
		pChan->i_late_20 = 0;
		pChan->q_late_20 = 0;
	}


	if (pChan->t_count == 1500)
	{
		pChan->n_frame++;
		pChan->t_count = 0;
	}

}

/*******************************************************************************
FUNCTION xors(long pattern)
RETURNS  Integer

PARAMETERS
pattern  long 32 bit data

PURPOSE
count the number of bits set in "pattern"

*******************************************************************************/

int xors(long pattern)
{
	int count, i;
	count = 0;
	pattern = pattern >> 6;
	for (i = 0; i <= 25; i++)
	{
		count += int(pattern & 0x1);
		pattern = pattern >> 1;
	}
	count = count % 2;
	return(count);
}

/*******************************************************************************
FUNCTION sign(long data)
RETURNS  Integer

PARAMETERS
data Long

PURPOSE
This function returns
+1 when positive
0 when zero
-1 when negative
*******************************************************************************/
inline int sign(long data)
{
	int result;

	if (data  > 0) result = 1;
	else if (data == 0) result = 0;
	else if (data  < 0) result = -1;
	return (result);
}


/*******************************************************************************
FUNCTION bit_test()
RETURNS  None.

PARAMETERS None.

PURPOSE
Determine if a bit in the data word has been set
*******************************************************************************/
inline int  bit_test(int data, char bit_n)
{
	return(data & test[bit_n]);
}

/*******************************************************************************
FUNCTION read_rcvr_par(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  To read in from the rcvr_par file the receiver parameters that control
acquisition, tracking etc.


*******************************************************************************/
char rcvr_par_file[4096];
void read_rcvr_par(void)
{
	char intext[40];

	if ((in = fopen(rcvr_par_file, "rt")) == NULL)
	{
		printf("Cannot open rcvr_par.dat file.\n");
		exit(0);
	}
	else
	{
		fscanf(in, "%s %s", intext, tzstr);
		fscanf(in, "%s %lf", intext, &mask_angle);
		mask_angle /= r_to_d;
		fscanf(in, "%s %lf", intext, &clock_offset);
		fscanf(in, "%s %d", intext, &interr_int);
		fscanf(in, "%s %d", intext, &ICP_CTL);
		fscanf(in, "%s %lf", intext, &nav_up);
		fscanf(in, "%s %d", intext, &out_pos);
		fscanf(in, "%s %d", intext, &out_vel);
		fscanf(in, "%s %d", intext, &out_time);
		fscanf(in, "%s %d", intext, &out_debug);
		fscanf(in, "%s %d", intext, &m_tropo);
		fscanf(in, "%s %d", intext, &m_iono);
		fscanf(in, "%s %d", intext, &align_t);
	}
	fclose(in);
}

/*******************************************************************************
FUNCTION nav_fix()
RETURNS  None.

PARAMETERS None.

PURPOSE
This function determines the pseudorange and doppler to each
satellite and calls pos_vel_time to determine the position and
velocity of the receiver

*******************************************************************************/
#define DAICYCOR
double wgs[3];
int	pos_out_flag = 0;
int judge_nits = 0;
double	t_cor[13];
short	towFlag = 0;
void  nav_fix(void)
{
	//sta-tic int judge_nits = 0;

	char			ch, n, bit;
	double			tr_time[13], tr_avg, ipart, clock_error;
	//st-atic double	t_cor[13];
	int				i, ms;
	double			chip;
	int				meas_bit_time_rem;
	long			meas_bit_time_offset;
	double			tic_CNTR;
	XYZ				rp_ecef;
	ECEFT			dm_gps_sat[13], dp_gps_sat[13];
	double			clock_error_in_Hz;
	//s-tatic short	towFlag = 0;
	double             const_pram = 0.0;

	tr_avg = 0.0;
	n = 1;
	for (ch = 0; ch <= chmax; ch++)
	{
		if (chan[ch].state != track)
			continue;
		meas_bit_time_offset = 0;
		ms = chan[ch].epoch & 0x1f;
		bit = chan[ch].epoch >> 8;

		chip = chan[ch].codePhase;
		chan[ch].int_carr_phase = chan[ch].carrier_counter + chan[ch].d_carr_phase;

		chan[ch].carrier_output += (chan[ch].int_carr_phase - CARRIER_FREQ);

		if (out_debug)
		{
			fprintf(debug, " ch= %d PRN =%d bit time= %ld  bit= %d  ms= %d chip= %f",
				ch, chan[ch].prn, chan[ch].meas_bit_time, bit, ms, chip);
			if (ICP_CTL == 0)
				fprintf(debug, "CTL = %lf\n", chan[ch].doppler);
			else
				fprintf(debug, "ICP = %lf\n", chan[ch].int_carr_phase);
		}

		meas_bit_time_rem = chan[ch].meas_bit_time % 50;
		if (meas_bit_time_rem == bit + 1) meas_bit_time_offset = -1;
		if (meas_bit_time_rem == bit - 1) meas_bit_time_offset = +1;
		if (meas_bit_time_rem == 0 && bit == 49) meas_bit_time_offset = -1;
		if (meas_bit_time_rem == 49 && bit == 0) meas_bit_time_offset = +1;
		if ((chan[ch].meas_bit_time + meas_bit_time_offset) % 50 == bit &&
			chan[ch].state == track && chan[ch].CNo>30 &&
			gps_eph[chan[ch].prn].valid == 1 &&
			gps_eph[chan[ch].prn].health == 0 &&
			chan[ch].tow_sync == 1 && chip <= 1e-3)
		{
			tr_time[n] = (chan[ch].meas_bit_time + meas_bit_time_offset)*.02 +
				ms / 1000.0 + chip - 0.019 + SAMPLING_INT;

			tr_ch[n] = ch;
			tr_avg += tr_time[n];
			n++;
		}
	}

	n_track = n - 1;

	TIC_dt = i_TIC_dt*SAMPLING_INT;
	if (out_debug) fprintf(debug, "n_track= %d\n", n_track);
	if (towFlag == 0)
	{
		if (n_track>0)
		{
			m_time[1] = tr_time[1] + 0.075;
			towFlag = 1;
		}
		else if (bLocalTimeSetByTow)
		{
			m_time[1] += nav_up;
		}
		else if (bTowDecoded)
		{
			m_time[1] = clock_tow;
			bLocalTimeSetByTow = true;
		}
		else
			m_time[1] += nav_up;
	}
	else if (towFlag == 1)
		m_time[1] += nav_up;

	for (i = 1; i <= n_track; i++)
	{

		track_sat[i] = satpos_ephemeris(tr_time[i], chan[tr_ch[i]].prn);
		chan[tr_ch[i]].az = track_sat[i].az;
		chan[tr_ch[i]].el = track_sat[i].el;
		if (ICP_CTL == 0)
		{
			dm_gps_sat[i] = satpos_ephemeris(tr_time[i] - TIC_dt / 2.0, chan[tr_ch[i]].prn);
			dp_gps_sat[i] = satpos_ephemeris(tr_time[i] + TIC_dt / 2.0, chan[tr_ch[i]].prn);
		}
		else
		{
			dm_gps_sat[i] = satpos_ephemeris(tr_time[i] - TIC_dt, chan[tr_ch[i]].prn);
			dp_gps_sat[i] = track_sat[i];
		}
		t_cor[i] = track_sat[i].tb -
			tropo_iono(tr_ch[i], track_sat[i].az, track_sat[i].el, tr_time[i]);

#ifdef	DAICYCOR
		if (chan[tr_ch[i]].prn == 13)	const_pram = 0.0 / c;
		else if (chan[tr_ch[i]].prn == 3)	const_pram = 2.3 / c;
		else if (chan[tr_ch[i]].prn == 23)	const_pram = -6.2 / c;
		else if (chan[tr_ch[i]].prn == 16)	const_pram = -1.1 / c;
		else if (chan[tr_ch[i]].prn == 25)	const_pram = 2.8 / c;
		else if (chan[tr_ch[i]].prn == 8)	const_pram = -2.0 / c;
		else if (chan[tr_ch[i]].prn == 27)	const_pram = 1.5 / c;
		else if (chan[tr_ch[i]].prn == 11)	const_pram = -3.9 / c;
		else const_pram = 0.0 / c;
#endif

		dt[i] = m_time[1] - (tr_time[i] - t_cor[i]) + const_pram;
		d_sat[i].x = (dp_gps_sat[i].x - dm_gps_sat[i].x) / TIC_dt - track_sat[i].y*omegae;
		d_sat[i].y = (dp_gps_sat[i].y - dm_gps_sat[i].y) / TIC_dt + track_sat[i].x*omegae;
		d_sat[i].z = (dp_gps_sat[i].z - dm_gps_sat[i].z) / TIC_dt;
		if (ICP_CTL == 0) meas_dop[i] = chan[tr_ch[i]].doppler;
		else meas_dop[i] = chan[tr_ch[i]].int_carr_phase / TIC_dt - CARRIER_FREQ;
	}



	if (n_track >= 4)
	{
		switch (sel & USING_POS_METHOD_SEL)
		{
		case USING_POS_VEL_TIME:
			rpvt = pos_vel_time(n_track);
			break;
		case USING_KALMAN_POS_VEL:
			if (judge_nits == 0)
			{
				rpvt = pos_vel_time(n_track);
			}
			else
			{
				rpvt = kalman_pos_vel(n_track);
			}
			break;
		default:
			break;
		}
		

		cbias = rpvt.dt;
		clock_error = rpvt.df;
		clock_error_in_Hz = clock_error*1575.42;
		m_time[1] = m_time[1] - cbias;
		rp_ecef.x = rpvt.x;
		rp_ecef.y = rpvt.y;
		rp_ecef.z = rpvt.z;
		rp_llh = ecef_to_llh(rp_ecef);

		wgs[0] = rpvt.x;
		wgs[1] = rpvt.y;
		wgs[2] = rpvt.z;
		::PostMessage(NavOutWnd, WM_NAVIGATION_OUT, (WPARAM)wgs, 1);


		for (ch = 0; ch <= chmax; ch++)
		{
			chan[ch].vDoppler = 0.0;
		}
		for (ch = 1; ch <= n_track; ch++)
		{
			chan[tr_ch[ch]].vDoppler = meas_dop[ch] - clock_error_in_Hz;
		}
		if (rp_llh.hae>-2000.0 && rp_llh.hae< 28000 && pos_out_flag >= 0) //modified by daicy
		{
			velocity();

			if (speed < 514.0)
			{
				judge_nits++;
				if (fabs(clock_error)<5.0) clock_offset = clock_error;
				if (almanac_valid == 1) status = navigating;
				if (align_t == 1)
				{
					delta_m_time = modf(m_time[1], &ipart);
					if (nav_up<1.0)
					{
						delta_m_time = modf(delta_m_time / nav_up, &ipart);
						if (delta_m_time>0.5) m_error = (delta_m_time - 1.0)*nav_up;
						else m_error = delta_m_time*nav_up;
					}
					else
					{
						if (delta_m_time>0.5) m_error = (delta_m_time - 1.0) / nav_up;
						else m_error = delta_m_time / nav_up;
					}

#ifndef REAL_TIME
					tic_CNTR = TIC_ref*(1.0 - m_error / nav_up) / (1.0 + clock_offset*1.0e-6);
#endif
					programTIC(tic_CNTR);	//modifid by daicy
				}

				rec_pos_llh.lon = rp_llh.lon;
				rec_pos_llh.lat = rp_llh.lat;
				rec_pos_llh.hae = rp_llh.hae;
				current_loc.lon = rp_llh.lon;
				current_loc.lat = rp_llh.lat;
				current_loc.hae = rp_llh.hae;
				rec_pos_xyz.x = rp_ecef.x;
				rec_pos_xyz.y = rp_ecef.y;
				rec_pos_xyz.z = rp_ecef.z;

				dops(n_track);
				m_time[0] = m_time[1];
			}
		}
		pos_out_flag++;

	}
	else
	{

		m_time[1] = fmod(m_time[1], 604800);
		rp_ecef.x = 0.0;
		rp_ecef.y = 0.0;
		rp_ecef.z = 0.0;
		rpvt.xv = 0.0;
		rpvt.yv = 0.0;
		rpvt.zv = 0.0;
	}

	if (n_track >= 4)
	{
		for (i = 1; i <= n_track; i++)
		{

			chan[tr_ch[i]].Pr = (m_time[1] - tr_time[i])*SPEEDOFLIGHT;

			chan[tr_ch[i]].dPr = chan[tr_ch[i]].vDoppler*lambda;

			////����α��
			//char str[10];
			////itoa(sv, str, 10);
			//sprintf(str, "%d", chan[tr_ch[i]].prn);
			//char s[100] = "PRN_";
			//char *pfilename = strcat(s, str);

			//FILE *fd = fopen(pfilename, "a");
			//fprintf(fd, "TOW: % 6ld, Pr: %10.9f\n", clock_tow, chan[tr_ch[i]].Pr);
			//fclose(fd);
		}
	}
}


/*******************************************************************************
FUNCTION velocity(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  To convert velocity from ecef to local level (WGS-84) axes

*******************************************************************************/
void velocity(void)
{

	// x,y,z -> North
	receiver.north.x = -cos(rec_pos_llh.lon)*sin(rec_pos_llh.lat);
	receiver.north.y = -sin(rec_pos_llh.lon)*sin(rec_pos_llh.lat);
	receiver.north.z = cos(rec_pos_llh.lat);

	// x,y,z -> East
	receiver.east.x = -sin(rec_pos_llh.lon);
	receiver.east.y = cos(rec_pos_llh.lon);
	//  receiver.east.z=0.0;

	// x,y,z -> Up
	receiver.up.x = cos(rec_pos_llh.lon)*cos(rec_pos_llh.lat);
	receiver.up.y = sin(rec_pos_llh.lon)*cos(rec_pos_llh.lat);
	receiver.up.z = sin(rec_pos_llh.lat);

	receiver.vel.north = rpvt.xv*receiver.north.x + rpvt.yv*receiver.north.y +
		rpvt.zv*receiver.north.z;
	receiver.vel.east = rpvt.xv*receiver.east.x + rpvt.yv*receiver.east.y;
	receiver.vel.up = rpvt.xv*receiver.up.x + rpvt.yv*receiver.up.y +
		rpvt.zv*receiver.up.z;
	speed = sqrt(receiver.vel.north*receiver.vel.north + receiver.vel.east*receiver.vel.east);
	if (speed == 0.0) heading = 0.0;
	else heading = atan2(receiver.vel.east, receiver.vel.north);

	//	 gotoxy(1,24);
	//	 printf("velocity->");
}

/*******************************************************************************
FUNCTION pream(char ch, char bit)
RETURNS  None.

PARAMETERS

ch   channel number (0-11)
bit  the latest data bit from the satellite

PURPOSE
This function finds the preamble in the navigation message and synchronizes
to the nav message

*******************************************************************************/
unsigned long GPSL1_pream = 0x22c00000L;
unsigned long pb1 = 0xbb1f3480L, pb2 = 0x5d8f9a40L, pb3 = 0xaec7cd00L;
unsigned long pb4 = 0x5763e680L, pb5 = 0x6bb1f340L, pb6 = 0x8b7a89c0L;
void pream(char ch, char bit)
{
	//st-atic unsigned long pream = 0x22c00000L;
	unsigned long  parity0, parity1;
	//sta-tic unsigned long pb1 = 0xbb1f3480L, pb2 = 0x5d8f9a40L, pb3 = 0xaec7cd00L;
	//sta-tic unsigned long pb4 = 0x5763e680L, pb5 = 0x6bb1f340L, pb6 = 0x8b7a89c0L;
	unsigned long TOWs, HOW, TLM, TLMs;
	unsigned long HOWLastTwo;
	int sfid_s;
	long currentBitPos, frameLength;
	if (chan[ch].fifo1 & 0x20000000L)//0010
	{
		chan[ch].fifo0 = (chan[ch].fifo0 << 1) + 1;
	}
	else
	{
		chan[ch].fifo0 = chan[ch].fifo0 << 1;
	}

	chan[ch].fifo1 = (chan[ch].fifo1 << 1) + bit;

	if (chan[ch].fifo0 & 0x40000000L)
	{
		TLM = chan[ch].fifo0 ^ 0x3fffffc0L;
	}
	else
	{
		TLM = chan[ch].fifo0;
	}

	if (chan[ch].fifo1 & 0x40000000L)
	{
		HOW = chan[ch].fifo1 ^ 0x3fffffc0L;
	}
	else
	{
		HOW = chan[ch].fifo1;
	}
	TOWs = (HOW & 0x3fffffffL) >> 13;
	//printf("%d", TOWs);
	sfid_s = int((HOW & 0x700) >> 8);
	HOWLastTwo = (HOW & 0x3);
	currentBitPos = chan[ch].t_count + 1;

	if (chan[ch].subFrameSyncFlag == SUBFRAME_SYNCHRONIZED)
	{
		frameLength = currentBitPos >= chan[ch].expectedFrameHead ?
			currentBitPos - chan[ch].expectedFrameHead : currentBitPos + 1500 - chan[ch].expectedFrameHead;
		if (frameLength == 60)
		{
			chan[ch].subFrameSyncFlag = SUBFRAME_SYNCHRONIZING;
			if (((GPSL1_pream^TLM) & 0x3fc00000L) == 0)
			{
				parity0 = (xors(TLM & pb1) << 5) + (xors(TLM & pb2) << 4) +
					(xors(TLM & pb3) << 3) + (xors(TLM & pb4) << 2) +
					(xors(TLM & pb5) << 1) + (xors(TLM & pb6));

				if (parity0 == (TLM & 0x3f))
				{

					parity1 = (xors(HOW & pb1) << 5) + (xors(HOW & pb2) << 4) +
						(xors(HOW & pb3) << 3) + (xors(HOW & pb4) << 2) +
						(xors(HOW & pb5) << 1) + (xors(HOW & pb6));

					if (parity1 == (HOW & 0x3f)
						&& (HOWLastTwo == 0 || HOWLastTwo == 0x3)
						&& TOWs<100800
						&& sfid_s == chan[ch].expectedSubFrameID)
					{
						chan[ch].subFrameSyncFlag = SUBFRAME_SYNCHRONIZED;
						chan[ch].expectedFrameHead = (currentBitPos - 60 + 300) % 1500;
						if (++chan[ch].expectedSubFrameID>5)
						{
							chan[ch].expectedSubFrameID = 1;
						}

						TLMs = (TLM >> 2) & 0x3fff;
						if (sfid_s == 1)
						{
							d_tow = clock_tow - TOWs * 6 + 5;
#ifdef REAL_TIME
							if (labs(d_tow)<300)
#endif
							{
								chan[ch].offset = chan[ch].t_count - 59;
								writeEpoch(ch, (0x1f & readEpochCheck(ch)) | 0xa00);
								if (chan[ch].offset<0.0) chan[ch].offset += 1500;
								chan[ch].tr_bit_time = TOWs * 300 - 240;

								chan[ch].TOW = TOWs * 6;
								chan[ch].tow_sync = 1;
								thetime = thetime - d_tow;

								clock_tow = TOWs * 6 - 5;
								bTowDecoded = true;
								chan[ch].sfid = sfid_s;
								chan[ch].TLM = TLMs;
							}
						}
						else if (sfid_s>1 && sfid_s<6)
						{

							d_tow = clock_tow - TOWs * 6 + 5;
#ifdef REAL_TIME
							if (labs(d_tow)<3)
#endif
							{

								chan[ch].offset = chan[ch].t_count - 59 - (sfid_s - 1) * 300;
								writeEpoch(ch, (0x1f & readEpochCheck(ch)) | 0xa00);

								if (chan[ch].offset<0.0) chan[ch].offset += 1500;

								chan[ch].tr_bit_time = TOWs * 300 - 240;
								chan[ch].tow_sync = 1;
								chan[ch].TOW = TOWs * 6;
								chan[ch].sfid = sfid_s;
								chan[ch].TLM = TLMs;
							}
						}
					}
				}
			}
			if (chan[ch].subFrameSyncFlag != SUBFRAME_SYNCHRONIZED)
			{
				chan[ch].t_count = 0;
				chan[ch].n_frame = 0;
			}
		}
	}

	else
	{
		if (((GPSL1_pream^TLM) & 0x3fc00000L) == 0)
		{
			parity0 = (xors(TLM & pb1) << 5) + (xors(TLM & pb2) << 4) +
				(xors(TLM & pb3) << 3) + (xors(TLM & pb4) << 2) +
				(xors(TLM & pb5) << 1) + (xors(TLM & pb6));

			if (parity0 == (TLM & 0x3f))
			{
				parity1 = (xors(HOW & pb1) << 5) + (xors(HOW & pb2) << 4) +
					(xors(HOW & pb3) << 3) + (xors(HOW & pb4) << 2) +
					(xors(HOW & pb5) << 1) + (xors(HOW & pb6));

				if (parity1 == (HOW & 0x3f)
					&& (HOWLastTwo == 0 || HOWLastTwo == 0x3)
					&& TOWs<100800
					&& sfid_s<6 && sfid_s>0)
				{
					chan[ch].subFrameSyncFlag = SUBFRAME_SYNCHRONIZED;
					chan[ch].expectedFrameHead = (currentBitPos - 60 + 300) % 1500;
					chan[ch].expectedSubFrameID = sfid_s == 5 ? 1 : sfid_s + 1;

					TLMs = (TLM >> 2) & 0x3fff;
					if (sfid_s == 1)
					{
						d_tow = clock_tow - TOWs * 6 + 5;
#ifdef REAL_TIME
						if (labs(d_tow)<300)
#endif
						{

							writeEpoch(ch, (0x1f & readEpochCheck(ch)) | 0xa00);

							if (chan[ch].offset<0.0) chan[ch].offset += 1500;
							chan[ch].tr_bit_time = TOWs * 300 - 240;
							chan[ch].TOW = TOWs * 6;
							chan[ch].tow_sync = 1;
							thetime = thetime - d_tow;
							clock_tow = TOWs * 6 - 5;
							chan[ch].sfid = sfid_s;
							chan[ch].TLM = TLMs;
						}
					}
					else if (sfid_s>1 && sfid_s<6)
					{

						d_tow = clock_tow - TOWs * 6 + 5;
#ifdef REAL_TIME
						if (labs(d_tow)<3)
#endif
						{

							chan[ch].offset = chan[ch].t_count - 59 - (sfid_s - 1) * 300;
							writeEpoch(ch, (0x1f & readEpochCheck(ch)) | 0xa00);
							if (chan[ch].offset<0.0) chan[ch].offset += 1500;

							chan[ch].tr_bit_time = TOWs * 300 - 240;
							chan[ch].tow_sync = 1;
							chan[ch].TOW = TOWs * 6;
							chan[ch].sfid = sfid_s;
							chan[ch].TLM = TLMs;
						}
					}
				}
			}
		}
	}

	if ((chan[ch].t_count + chan[ch].n_frame * 1500 - chan[ch].offset) % 1500 == 900
		&& chan[ch].subFrameSyncFlag == SUBFRAME_SYNCHRONIZED)
	{
		chan[ch].frame_ready = 1;
	}
}
