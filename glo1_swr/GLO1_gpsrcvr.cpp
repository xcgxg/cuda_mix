// Released on July 9, 2004
#include "../stdafx.h"
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <math.h>
#include <time.h>
#include <assert.h>

#include "GLO1_gpsconst.h"
#include "GLO1_gpsstruct.h"
#include "../receiver/readData.h"
#include "GLO1_correlatorProcess.h"
#include "GLO1_acqCA.h"
#include "GLO1_cagen.h"
#include "GLO1_gp2021.h"
#include "GLO1_gpsfuncs.h"
#include "GLO1_gpsrcvr.h"
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

#define	GLO1_IRQLEVEL        0		

GLO1_CHANNEL		   GLO1_chan[GLO1_chmax+1];				// ȫ�̱��������ڴ洢ÿ��ͨ�����ٻ�·��״̬
GLO1_CORRELATOR	   GLO1_correlator[GLO1_chmax+1];		// ȫ�̱��������ڴ洢ÿ��ͨ���������״̬
GLO1_SVSTRUCT	      GLO1_svStruct[GLO1_MaxSvNumber+1];	// ��ʾ���ǿ�����
long		      GLO1_globalBufLength[GLO1_chmax+1];	// ���ڱ���ÿ��ͨ����������ݳ��ȣ����ʼ��
double  *GLO1_buffer[GLO1_chmax+1];			// ���ڱ���ÿ��ͨ���Ļ������ݣ����ʼ������ռ�
long		      GLO1_correlatorDataLength;	// ���л����������ݳ���
long		      GLO1_TIC_RT_CNTR;				// TICʵʱ��������
long		      GLO1_TIC_CNTR;					// TIC������������Ӧ��TIC�жϵķ�������
										         // ע�⣺GLO1_TIC_CNTR�ĸ�ֵֻ����GLO1_programTIC����ʵ��
double         GLO1_TIC_CNTR_RES;
double         GLO1_TIC_CNTR_DBL;
long		      GLO1_TIC_OCCUR_FLAG;			// TIC�жϷ����ı�־

int			   GLO1_display_page=0;			// ������ʾ��i��ҳ��
unsigned	GLO1_test[16]=
			{	0x0001,0x0002,0x0004,0x0008,0x0010,0x0020,0x0040,0x0080,
				0x0100,0x0200,0x0400,0x0800,0x1000,0x2000,0x4000,0x8000};
unsigned int GLO1_tr_ch[13];					   // ����0���ã�����Ϊ������Ч�۲����ݵ�GLO1_chan�ı��
int			GLO1_out_debug,GLO1_out_pos,GLO1_out_vel,GLO1_out_time;		// �Ƿ����GLO1_debug��GLO1_PVT��Ϣ�ı�־��
GLO1_DMS			GLO1_cur_lat,GLO1_cur_long;			   // ������ʾ��γ��

// ���¶����������ȱʡֵ����ͨ���ı��ļ�rcvr_par.dat���趨ʵ��
int			   GLO1_nav_tic;						// һ��GLO1_nav_up��ʱ������Ӧ��TIC�жϵĴ���
int            GLO1_ICP_CTL=0;		         // ����ۻ��ز���λ���Ǹ��ٻ�·��ֵ����־
double         GLO1_ICP_code_corr;              // CA�����ʵ����������֣�1.023MHz��Ӧ1.0
long           GLO1_time_on=0;	            // ������������Ϊ�룬����ͣ��������ֹ
// long		      GLO1_cc_scale=1540;       	// �ز���Ƶ(1575.42MHz)/CA���Ƶ(1.023MHz)
long		      GLO1_cc_scale=3136;       	// �ز���Ƶ(1602+k*0.5625MHz)/CA���Ƶ(0.511MHz)   todo  �������Ҫ�ı�,���˱�������û��ʹ��

int GLO1_alloced_k[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // ��¼ĳ�� k �Ƿ��Ѿ�����ͨ�� k = -7 --- 6

double         GLO1_nav_up=1.0;					// ������Ļ�����λ����λ����
double		   GLO1_speed,GLO1_heading;          // ���ջ��ٶȵľ���ֵ�ͷ�λ��
long		GLO1_d_tow;								// ���ջ�ʱ����λ����
int			GLO1_key=0;								// �����̵İ�����Ϣ
int			GLO1_tic_count=0;						// TIC��������Ҳ�ǲ����жϷ��������ļ�����
												// ע�⣺�����жϵ�Ƶ������ȱʡΪ��100ms
												// ������ΧΪ1s[0-9]
int			GLO1_hms_count=0;						// TIC��������������ΧΪ1����[0-599]
int			GLO1_nav_count;							// TIC��������������ΧΪ[0,GLO1_nav_tic-1]
int			GLO1_min_flag;							// ���Ӽ�������־
int			GLO1_nav_flag;							// ���㵼�����־
int			GLO1_sec_flag;							// ���־
int			GLO1_n_track;							// ����(ͨ������trackingģʽ)�����Ǹ���
unsigned int GLO1_interr_int=512;
double		GLO1_clock_offset=0.0;					// ���ջ�����Ư����λ��ppm,��ʼֵδ֪���ٶ�Ϊ0
												// ��ֵ��ʾ���ջ���Ƶ�ʵ��ڱ��ֵ
GLO1_XYZ			GLO1_rec_pos_ecef;						// ���ջ�������
long		GLO1_i_TIC_dt;							// ���μ��㵼����ļ��,��λ���������
double		GLO1_TIC_dt;								// ���μ��㵼����ļ��,��λ����
												// ����GLO1_TIC_dt=GLO1_i_TIC_dt*�������
double		GLO1_m_time[3];							// ���ջ�ʱ��,[1]�ǵ�ǰʱ�̵ģ�[0]����һ�ε�
												// [2]�ƺ�û����
												// GLO1_m_time���ڱ�ʾ���ջ���GPSʱ��
												// ��ԭ���ĳ�����GLO1_m_timeû����Ч�ĸ���,
												// ��Ӧ�ÿ����жϱ�־GLO1_sec_flag��ά��
double		GLO1_delta_m_time;						// ���ջ�ʱ���С������
double		GLO1_m_error;							// ���ջ��Ĳ���ʱ���GPS��������
long		GLO1_TIC_sum;							// ��������ж��������ۻ��ļ���ֵ
//int			astat;							// ������GP2021��������dump ready��־�ļĴ���
//int			mstat;							// ������GP2021��������miss GLO1_data��־�ļĴ���
char		**GLO1_caTable;							// ��ʵʱ����ʱ���洢37��1023��CA���

double		GLO1_DLLc1, GLO1_DLLc2;						// DLL��ϵ��
double		GLO1_PLLc0, GLO1_PLLc1, GLO1_PLLc2;				// PLL��ϵ��
double		GLO1_FLLa1, GLO1_FLLa2;						// FLL��ϵ��

// �����õ�ȫ�̱���
double		*GLO1_IQQ, *GLO1_IQI;
double		GLO1_IBUF[3000], GLO1_QBUF[3000];
FILE		*GLO1_fpIQ;
long		GLO1_IQCounter;
FILE		*GLO1_fpobs;
FILE		*GLO1_fpeph;

long		GLO1_detectNum[GLO1_chmax+1];					// ����ĳ�����ǵĳ���ʱ��
double		GLO1_threshold_sig_search;				// �źż������							

//char		GLO1_data[GLO1_SAMPLESPMS+1];							// �źŻ�����
//char		GLO1_data[6000];							// �źŻ�����
double		GLO1_minFreqErr[GLO1_chmax+1],GLO1_maxFreqErr[GLO1_chmax+1];

unsigned long GLO1_uLastPCTick,GLO1_uCurrentPCTick;

__int64 GLO1_iTotalBitCounter=0;

bool GLO1_bLocalTimeSetByTow=false;
bool GLO1_bTowDecoded=false;

// �ⲿ����
extern GLO1_XYZ			GLO1_rec_pos_xyz;
extern FILE			*GLO1_stream,*GLO1_debug,*GLO1_in,*GLO1_out,*GLO1_kalm;
extern GLO1_LLH			GLO1_current_loc,GLO1_rp_llh;
extern GLO1_LLH			GLO1_rec_pos_llh;
extern time_t		GLO1_thetime;
extern int			GLO1_status;
extern unsigned long  GLO1_clock_tow;
extern int			GLO1_alm_gps_week,GLO1_gps_week,GLO1_almanac_valid,GLO1_almanac_flag,GLO1_handle;
extern GLO1_SATVIS		GLO1_xyz[33];
extern GLO1_EPHEMERIS	GLO1_gps_eph[33];
extern double		GLO1_gdop, GLO1_pdop, GLO1_hdop, GLO1_vdop, GLO1_tdop, GLO1_alm_toa;
extern GLO1_ECEFT		GLO1_track_sat[13];
extern double		GLO1_dt[13],GLO1_cbias;
extern GLO1_XYZ			GLO1_d_sat[13];
extern double		GLO1_meas_dop[13];
extern double       GLO1_lambda[13];
extern GLO1_PVT			GLO1_rpvt;
extern GLO1_STATE		GLO1_receiver;
extern int			GLO1_m_tropo, GLO1_m_iono, GLO1_align_t;	// flags for using tropo and iono models	
extern GLO1_ALMANAC		GLO1_gps_alm[33];
extern double		GLO1_carrier_ref,GLO1_code_ref;
extern char			GLO1_tzstr[40];				
extern double		GLO1_mask_angle;
extern double		GLO1_b0,GLO1_b1,GLO1_b2,GLO1_b3,GLO1_al0,GLO1_al1,GLO1_al2,GLO1_al3;	// broadcast ionospheric delay model
extern double		GLO1_a0,GLO1_a1,GLO1_tot,GLO1_WNt,GLO1_dtls,GLO1_WNlsf,GLO1_DN,GLO1_dtlsf;//broadcast UTC GLO1_data

extern int GLO1_ADNumber;   //0624
float   GLO1_CARRIER_FREQ;   //�����Ƶ  0625 //todo:GLONASS����Ƶ��
double  GLO1_SAMPLING_FREQ;     //0626
double  GLO1_SAMPLING_INT;      //0626
long    GLO1_DETECTSIZE;        //0626
long	GLO1_TIC_ref;           //0626
double  GLO1_deltaPhaseConst;   //0626
double  GLO1_deltaCodePhaseConst;    //0626
long    GLO1_SAMPLESPMS;         //0627
long    GLO1_ACC_INT_PERIOD;     //0627
char    *GLO1_data;              //0627
long    GLO1_FFTSIZE;            //0627
long    GLO1_bufSize;            //0627

FILE   *GLO1_fpCodeRange1,*GLO1_fpCodeRange2;     //0628
FILE   *GLO1_fpCarrRange1,*GLO1_fpCarrRange2;     //0628
long   GLO1_ldcount = 0;                     //0628
FILE   *GLO1_fpdoppler;                      //0628
FILE   *GLO1_fpCNo;                          //0628

char   GLO1_if_data_file[4096];

GLO1_glonass_ephemeris GLO1_glonass_channel_ephemeris[12];
GLO1_glonass_almanac GLO1_glonass_sv_id_almanac[GLONASS_SV_TOTAL_NUM];
GLO1_glonass_almanac_str5 *GLO1_palc_str5 = new GLO1_glonass_almanac_str5;
GLO1_glonass_almanac_global *GLO1_palc_glob = new GLO1_glonass_almanac_global;

int16s	GLO1_unpack_glonass_flag[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };

int16s	GLO1_glonass_ephemeris_processing[12] = { -1, - 1,- 1, - 1, - 1, - 1, - 1, - 1, - 1, - 1, - 1, - 1 };		//��ʼ��  -1
int32u *GLO1_glonass_frame_id = new int32u;
int16s GLO1_glonass_almanac_processing[12] = { -1, -1,-1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
int16s	GLO1_glonass_almanac_global_processing[12] = { -1, -1,-1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };	//��ʼ��  -1
bool GLO1_string_1[12] = {};
//int16s *GLO1_glonass_almanac_global_processing = new int16s;
//int16s *GLO1_glonass_almanac_processing = new int16s;
/*******************************************************************************
FUNCTION main()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This is the main program to control the GPS GLO1_receiver

*******************************************************************************/

extern char GLO1_last_prn[12];
char GLO1_hot_cold;
extern char GLO1_curloc_file[];      
FILE *GLO1_daicy_file_pr;
FILE *GLO1_daicy_file_pos;
FILE   *GLO1_out_trtime;

long GLO1_sim_main_init()
{
	char  ch;
	long num,TIC_DIF;
	long numRead;
	FILE* fpData;
	FILE* fpOption;
	char fileName[200];
	int i;
	fp64 eph_effect_time = 0;

	GLO1_ADNumber = (int)pow(2.0,GLO1_ADNumber);//GLO1_ADNumber������ı�������λ��
	GLO1_carrier_ref = GLO1_CARRIER_FREQ;//��ƵƵ��

	GLO1_SAMPLING_INT = 1.0/GLO1_SAMPLING_FREQ;//GLO1_SAMPLING_FREQ:����Ƶ��
	GLO1_DETECTSIZE = (long)((long long)(GLO1_SAMPLING_FREQ*0.01+0.5) & 0xffffffffUL);//10��������ݵ���
	
	GLO1_TIC_ref = (long)((long long)(GLO1_SAMPLING_FREQ*0.1+0.5-1) & 0xffffffffUL);// 100ms��Ӧ���жϼ���ֵ 100��������ݵ���  Ԥ��GLO1_TIC_CNTR�ļ�����100����

	GLO1_deltaPhaseConst = GLO1_SAMPLING_INT*GLO1_carrierNCOMax;// GLO1_carrierNCOMax��Ӧ���ز�DCO��ʵ�ʷֱ���2^30   ���ٳ��Ա���ز��ͱ�ɱ���ز�Ƶ������
												 // GLO1_deltaPhaseConst�Ͷ�Ӧ�����ز�Ƶ���ֵ� 2^30/����Ƶ��
	GLO1_deltaCodePhaseConst = GLO1_SAMPLING_INT*0.511e6*GLO1_d_2p40;// �����Ƶ��*2^40/����Ƶ�� = ��Ƶ����
													  //GLO1_SAMPLING_INT*1.023e6*GLO1_d_2p40;

	GLO1_SAMPLESPMS = (long)((long long)(GLO1_SAMPLING_FREQ*0.001) & 0xffffffffUL);   //0627//lyq
	//jh GLO1_SAMPLESPMSÿ��������ݵ���

	GLO1_ACC_INT_PERIOD = GLO1_SAMPLESPMS/2+1;//+1֮����Ȼ���Ǹպõ�0.5���룬������ֻ�Ƕ��ĵ����������ݴ����ʱ�򣬻��ǰ�������Ķ�����ȡ�ģ�����ûӰ��
	//#define GLO1_ACC_INT_PERIOD  GLO1_SAMPLESPMS/2+1			// 0.5ms��������
	//jh GLO1_ACC_INT_PERIOD:ÿ0.5������֣�������Ȼ��1���룩��0.5������ж���
	GLO1_data = new char[GLO1_SAMPLESPMS+1];

	fopen_s(&GLO1_out_trtime, "tr_time.txt", "w");//������
	fopen_s(&GLO1_daicy_file_pr, "daicy_pr.txt", "w");
	fopen_s(&GLO1_daicy_file_pos, "daicy_pos.txt", "w");
	

	
	float fnumber=(float)GLO1_SAMPLING_FREQ/500;//2����ĵ���  ��ΪҪȡ2MS�ı���α�룬ΪʲôҪ2ms��GPS�źŵ�CA�볤1023��Ƭ������1ms �����ڸ����������£�1ms�ڲɵ��ĵ�����һ����2n
	
	int fftcount=0;
//lyq
	do
	{
		fnumber=fnumber/2;//fnumber�Ǹ�����
		fftcount++;
	}while(fnumber>2);

	if(fnumber<=1.5)
		GLO1_FFTSIZE=(int)(pow(2.0,fftcount)*1.5);//��������ܴ���1.5�� 2�Ķ��ٴη�  ��ע�ⶼ��ȡ��
	else 
		GLO1_FFTSIZE=(int)pow(2.0,fftcount+1);//������if else���϶�ȡ��������2ms�ĵ���

	GLO1_bufSize = (long)((long long)(GLO1_SAMPLING_FREQ*0.0005) & 0xffffffffUL);//0.5��������ݵ�������GLO1_ACC_INT_PERIOD��Ӧ

    fopen_s(&GLO1_fpCodeRange1, "CodeRangeSample.txt", "w");      //0628   
	fopen_s(&GLO1_fpCodeRange2, "CodeRangeFix.txt", "w");
	fopen_s(&GLO1_fpCarrRange1, "CarrRangeSample.txt", "w");
	fopen_s(&GLO1_fpCarrRange2, "CarrRangeFix.txt", "w");
	fopen_s(&GLO1_fpdoppler, "doppler.txt", "w");
	fopen_s(&GLO1_fpCNo, "CNo.txt", "w");

#ifdef DEMO_CONSOLE      
	sharemem_start();
	sharemem_write(1);
#endif

	GLO1_read_rcvr_par(); //��rcvr_par�ļ�������Ʋ�����ٵĽ��ջ�����
	GLO1_rec_pos_xyz.x=0.0;// set up user position at earth center, if last valid position is
						// stored GLO1_in a file, the position will be updated
	GLO1_rec_pos_xyz.y=0.0;
	GLO1_rec_pos_xyz.z=0.0;

//׼������ļ�
	if (GLO1_out_pos==1 || GLO1_out_vel==1 ||GLO1_out_time==1) 
		fopen_s(&GLO1_stream, "gpsrcvr.log","w");//gpsrcvr.log��Ž��ջ��ĵ�ǰ��λʱ�䣬γ���ߣ�GLO1_XYZ,�������ٶȣ���Ư��H/V/TDOP
	if (GLO1_out_debug==1) 
		fopen_s(&GLO1_debug, "debug.log","w+");


	GLO1_read_initial_data();

	GLO1_initSignalSearch();// Ϊ�ź�������̬��������

	//GLO1_current_loc=receiver_loc();
	GLO1_rec_pos_llh.lon=GLO1_current_loc.lon;// ��鱣����ļ����Ƿ���н��ջ���λ��
	GLO1_rec_pos_llh.lat=GLO1_current_loc.lat;
	GLO1_rec_pos_llh.hae=GLO1_current_loc.hae;
	
	// ����navFix�ĸ���ʱ������
	GLO1_nav_tic= (int)(GLO1_nav_up/0.1+0.5);	// ��Ӧ��GLO1_nav_up���ڵĲ����ж�����һ��10�Σ���Ϊ�����ж�100����һ�Σ�������Ϊ10����1000�������һ��
									//������Ͳ����ж�ʱ��һ����������Ƶ��
									//�����ж�һ�ξ��ܵõ�һ��α��Ӷ��õ�һ�ζ�λ���������1��10���жϼ��������1��10�ε�ˢ���ʣ�ˢ���ʿ���Ϊ1~10
	GLO1_programTIC(GLO1_TIC_ref);	// ��ЧΪ: GLO1_TIC_CNTR = GLO1_TIC_ref // TIC������������Ӧ��TIC�жϵķ�������// ע�⣺GLO1_TIC_CNTR�ĸ�ֵֻ����GLO1_programTIC����ʵ��
	
	for (ch=0;ch<=GLO1_chmax;ch++) 
    {
		GLO1_chan[ch].state = GLO1_CHN_OFF;//����ر�״̬����ͨ������
      GLO1_chan[ch].prn = 0;
    }
	for (ch=GLO1_MinSvNumber; ch<=GLO1_MaxSvNumber; ch++) 
	{
		GLO1_svStruct[ch].state = GLO1_AVAILABLE;//�����������Ǿ�����
		GLO1_svStruct[ch].undetectedCounter = 0;
		GLO1_svStruct[ch].NumBTWEachTrial = GLO1_ReAcqTime;//#define GLO1_ReAcqTime 1000000LTrialʵ����   �������ز������ޣ�����ĳ���Ǳ��ز���ʱ��
												 //NumBTWEachTrial��number between each Trial  BTW:between
		GLO1_svStruct[ch].NumToTryAcq = 1;
 		GLO1_svStruct[ch].maxf=GLO1_MAXF;
		GLO1_svStruct[ch].minf=GLO1_MINF;
		GLO1_xyz[ch].azimuth=GLO1_xyz[ch].doppler=GLO1_xyz[ch].elevation=0.0;
		GLO1_xyz[ch].x=GLO1_xyz[ch].y=GLO1_xyz[ch].z=0.0;
	}

	time(&GLO1_thetime);  
#ifndef REAL_TIME
	GLO1_initCorrelator();
#endif
	// ��ʼ���ŵ��ķ��䣬���������������ķ��䷽ʽ��ͬ
	GLO1_hot_cold = 1;
	if (GLO1_hot_cold == 1)
	{
		GLO1_ch_alloc();
	} 
	else if (GLO1_hot_cold == 2)
	{
		GLO1_hot_ch_alloc();
	}
	// double	GLO1_m_time[3]; // ���ջ�ʱ��,[1]�ǵ�ǰʱ�̵ģ�[0]����һ�ε� [2]�ƺ�û���� 
	//GLO1_m_time���ڱ�ʾ���ջ���GPSʱ�� ��ԭ���ĳ�����GLO1_m_timeû����Ч�ĸ���,��Ӧ�ÿ����жϱ�־GLO1_sec_flag��ά��
	GLO1_m_time[1]=GLO1_clock_tow;	// �����Ƶ�GPS�븳ֵ�����ջ�����ʱ��,ע���ⲻ��PC����ʱ�� 

	GLO1_read_ephemeris();//��"current.eph"�ж�����

#ifndef REAL_TIME
	GLO1_TIC_RT_CNTR = 0;	// simulator tic counter GLO1_in GLO1_correlator
	num = GLO1_ACC_INT_PERIOD;	// simulator acc_int counter GLO1_in GLO1_correlator
							//#define GLO1_ACC_INT_PERIOD  GLO1_SAMPLESPMS/2+1			// 0.5ms��������	
							//ÿ��ȡ0.5ms��������ȡ������ۼӣ�����ÿ1ms�����һ����صĽ����������һ������ۼӣ�����������1ms�ۼ�������������ȡ0.5

	GLO1_initTrackLoopPar();//��ʼ���ز���
#endif

	// fopen_s(&fpData, GLO1_if_data_file, "rb");
	// if(fpData==NULL)
	// {
	// 	perror("Cannot open GLO1_data file! Program Exited");
	// 	exit(0);
	// }

	// readFileHeader(fpData);
	fopen_s(&GLO1_fpeph, "monitor.eph", "wb");

	return num;
}

int GLO1_is_state_3 = 0;
void GLO1_sim_main(long num)
{
	fp64 eph_effect_time = 0;
	long TIC_DIF;
	long tmp_num;

	while (GLO1_key != 'x' && GLO1_key != 'X' && GLO1_receiver_buf.buf_remain_len >= num)  // ��ѭ����ʼ
	{
#ifndef REAL_TIME

		//socket���䶨λ��������������㷨����ʱ��
		if (0 == GLO1_is_state_3)
		{
			send_res_GLO1.var1 += num;

			for (int i = 0; i <= GLO1_chmax; i++)
			{
				if (3 == GLO1_chan[i].state)
				{
					GLO1_is_state_3 = 1;
					send_res_GLO1.var1 /= sameple_rate;

					break;
				}
			}
		}

		tmp_num = num; //printf("%d %d\n", num, GLO1_receiver_buf.buf_remain_len);

		/*for (int i = 0; i < tmp_num; i++)
		{
			printf("%d ", GLO1_receiver_buf.buf_current_ptr[i]);
		}
		printf("123\n");*/

		//for (int i = 0; i < tmp_num; i++)
		//{
		//	//printf("%d ", GLO1_receiver_buf.buf_current_ptr[i]);
		//	if (-6 == GLO1_receiver_buf.buf_current_ptr[i])
		//	{
		//		printf("-2\n");
		//	}
		//}
		//printf("123\n");

		// numRead = fread(GLO1_data,sizeof(char),num,fpData);//add by jh 4-2
		// //printf("%d", (int)GLO1_data[0]);
		// if (numRead<num)
		// {
		// 	GLO1_write_prn();
		// 	printf("\n reach the end of the GLO1_data file, process end!");
		// 	break;
		// }
		// compute how many samples should be read next time
		// generally, the datalength corresponds to an interval of a dump interrupt
		// but when tic interrupt occurs, the GLO1_data length can be shorter
		// GLO1_TIC_CNTR+1��GLO1_TIC_RT_CNTR���Դﵽ�����ֵ���ﵽ���������0
		// GLO1_TIC_RT_CNTR+num�ǵ����λ�������жϴ����GLO1_TIC_RT_CNTR����ﵽ��ֵ
		TIC_DIF = GLO1_TIC_CNTR+1-(GLO1_TIC_RT_CNTR+num);	
		//GLO1_TIC_RT_CNTR TICʵʱ��������
		//TIC_DIF����Ϊ�˼���num
		// �´�Ӧ�ö�ȡ�����������Ȼ�����Щ��ȥ�����
		GLO1_receiver_init_num = num = TIC_DIF < GLO1_ACC_INT_PERIOD ? TIC_DIF : GLO1_ACC_INT_PERIOD;
		//��֤��һ��ʱ������������ݸպôճ�100���룬�����100��������

		GLO1_data = GLO1_receiver_buf.buf_current_ptr;
		//memcpy(GLO1_data, GLO1_receiver_buf.buf_current_ptr, tmp_num);

		GLO1_correlatorProcess(GLO1_data, tmp_num);// simulate GLO1_correlator processing  numRead = read_share_memory(GLO1_data, num);
		
		GLO1_receiver_buf.buf_remain_len -= tmp_num;
		GLO1_receiver_buf.buf_current_ptr += tmp_num;

		GLO1_Sim_GPS_Interrupt();	// simulate interrupt processing
#endif
		// ע�⣺�ŵ��������������˷�ʱ�䣬û�б�Ҫÿ����������ж�
		// ����ʱ�ͼ��һ���ŵ�����������֮���Է������ǿ��Ǻ���ʱ
		// �ܹ����ٵر��������ŵ���ʵʱʹ��ʱ������������Ҫ��д��λ��
		// ҲҪ���°���
		for (char ch=0;ch<=GLO1_chmax;ch++)
		{
			if (GLO1_chan[ch].frame_ready==1 )
			{

				if (GLO1_glonass_sv_id_ephemeris[GLO1_chan[ch].K + 10].P1 == 0)
				{
					eph_effect_time = 30.0*60.0;
				}
				else if (GLO1_glonass_sv_id_ephemeris[GLO1_chan[ch].K + 10].P1 == 1)
				{
					eph_effect_time = 30.0*60.0;
				}
				else if (GLO1_glonass_sv_id_ephemeris[GLO1_chan[ch].K + 10].P1 == 2)
				{
					eph_effect_time = 45.0*60.0;
				}
				else if (GLO1_glonass_sv_id_ephemeris[GLO1_chan[ch].K + 10].P1 == 3)
				{
					eph_effect_time = 60.0*60.0;
				}
				/*û���ж�������Ч��*/

				int32u origin_string_data[3];
				origin_string_data[0] = GLO1_chan[ch].buf0;
				origin_string_data[1] = GLO1_chan[ch].buf1;
				origin_string_data[2] = GLO1_chan[ch].buf2;
				
				GLO1_glonass_explain_string_data(ch,origin_string_data, &GLO1_glonass_channel_ephemeris[ch] ,
					&GLO1_glonass_sv_id_almanac[ch],
					GLO1_palc_str5,
					GLO1_palc_glob,
					&GLO1_chan[ch],
					&GLO1_unpack_glonass_flag[ch],

					&GLO1_glonass_ephemeris_processing[ch],
					GLO1_glonass_frame_id,
					&GLO1_glonass_almanac_processing[ch],
					&GLO1_glonass_almanac_global_processing[ch],
					GLO1_string_1+ch
/*					GLO1_glonass_almanac_processing,
					GLO1_glonass_almanac_global_processing*/);
				//GLO1_glonass_sv_id_ephemeris[3].dot_Xn = 151.647567749;
				//GLO1_glonass_sv_id_ephemeris[3].doudot_Xn = 0.000001863;
				//GLO1_glonass_sv_id_ephemeris[3].Xn = 2008788.574218750;

				//GLO1_glonass_sv_id_ephemeris[6].dot_Xn = -374.916076660;
				//GLO1_glonass_sv_id_ephemeris[6].doudot_Xn = 0.000000931;
				//GLO1_glonass_sv_id_ephemeris[6].Xn = -18724789.062500000;

				//GLO1_glonass_sv_id_ephemeris[9].dot_Xn = 408.873558044;
				//GLO1_glonass_sv_id_ephemeris[9].doudot_Xn = 0.000000931;
				//GLO1_glonass_sv_id_ephemeris[9].Xn = 11058366.699218750;

				//GLO1_glonass_sv_id_ephemeris[10].dot_Xn = 388.826370239;
				//GLO1_glonass_sv_id_ephemeris[10].doudot_Xn = 0.000001863;
				//GLO1_glonass_sv_id_ephemeris[10].Xn = 9202125.000000000;

				//GLO1_glonass_sv_id_ephemeris[11].dot_Xn = 1554.545402527;
				//GLO1_glonass_sv_id_ephemeris[11].doudot_Xn = 0.000000931;
				//GLO1_glonass_sv_id_ephemeris[11].Xn = -20260283.203125000;

				//GLO1_glonass_sv_id_ephemeris[12].dot_Xn = -2735.612869263;
				//GLO1_glonass_sv_id_ephemeris[12].doudot_Xn = 0.000000931;
				//GLO1_glonass_sv_id_ephemeris[12].Xn = -12557306.640625000;

				//GLO1_glonass_sv_id_ephemeris[13].dot_Xn = -1261.600494385;
				//GLO1_glonass_sv_id_ephemeris[13].doudot_Xn = 0.000000000;
				//GLO1_glonass_sv_id_ephemeris[13].Xn = -23226338.867187500;

				//GLO1_glonass_sv_id_ephemeris[14].dot_Xn = -2645.416259766;
				//GLO1_glonass_sv_id_ephemeris[14].doudot_Xn = 0.000001863;
				//GLO1_glonass_sv_id_ephemeris[14].Xn = 5459494.628906250;

				//GLO1_glonass_sv_id_ephemeris[16].dot_Xn = 2514.353752136;
				//GLO1_glonass_sv_id_ephemeris[16].doudot_Xn = 0.000000000;
				//GLO1_glonass_sv_id_ephemeris[16].Xn = -9396625.488281250;


				//navmess(GLO1_chan[ch].prn,ch);  // decode the navigation message
				GLO1_chan[ch].frame_ready=0;   // for this channel
				
			}
		}
		

		if (GLO1_sec_flag==1)//����һ�����жϱ�־������ά��ϵͳ����ʱ��ͼ�黷·״̬����Sim_GPS_Interrupt��if (GLO1_tic_count==0) GLO1_sec_flag=1;		// one second has passed
		{
			GLO1_almanac_flag=0;
			
			GLO1_thetime++;// ʱ���ȥ��1��  ���⣺����ʵ�ʹ�ȥ��Ӧ�ò���������1�룬Ӧ��С��1�룿
			GLO1_clock_tow=(++GLO1_clock_tow)%604800L;
			GLO1_time_on++;// ������������Ϊ�룬����ͣ��������ֹ
			GLO1_sec_flag=0;
		} 
		if (GLO1_nav_flag==1)// ���㵼�����ʱ�䵽
		{

			if (_kbhit()) GLO1_key = _getch();
			

			GLO1_nav_fix();
			GLO1_nav_flag=0;
			GLO1_display();

			GLO1_send_sim_res_socket();
		}
		
		if (GLO1_min_flag==1)// ���������͵���ģʽ�µ��ŵ����亯��
		{
			GLO1_min_flag=0;
#ifdef BCPP
			clrscr();
#endif
		}

#ifdef DEMO_CONSOLE
			sharemem_write(1);
#endif


		
		if (GLO1_key =='p' || GLO1_key=='P')
		{
			GLO1_display_page++;
			GLO1_display_page=GLO1_display_page % 4;
#ifdef BCPP
			clrscr();
#endif
		}
	}
	
#ifdef DEMO_CONSOLE
	sharemem_write(0);
	sharemem_end();
#endif
}

void GLO1_sim_end()
{
	// Remove our interrupt and restore the old one
#ifdef REAL_TIME
	Interrupt_Remove();
#endif

#ifndef REAL_TIME
	GLO1_shutCorrelator();
#endif

#ifndef REAL_TIME
	// �ͷŶ�������
	GLO1_freeSignalSearch();
#endif
	GLO1_write_prn();
	// Update the Almanac Data file
	if (GLO1_almanac_valid == 1)  GLO1_write_almanac();
	//  Update the Ephemeris Data file
	GLO1_write_ephemeris();
	//  Update the ionospheric model and UTC parameters
	GLO1_write_ion_utc();
	{
		fopen_s(&GLO1_out, GLO1_curloc_file, "w+");
		fprintf(GLO1_out,"latitude  %f\n",GLO1_rec_pos_llh.lat*GLO1_r_to_d);
		fprintf(GLO1_out,"longitude %f\n",GLO1_rec_pos_llh.lon*GLO1_r_to_d);
		fprintf(GLO1_out,"hae       %f\n",GLO1_rec_pos_llh.hae);
		fclose(GLO1_out);
	}
	_fcloseall();
	::PostMessage(MainWnd, WM_SIMULATION_STOP, 0, 0);
}

void GLO1_send_sim_res_socket()
{
	/*���ջ���ʶ*/
	send_res_GLO1.receiver_id = RECEIVER_GLO1;

	//�������㷨����ʱ��ֻ����һ��
	//send_res_BD1.var1 = 0;

	send_res_GLO1.dim = m*n;

	//Ȩֵȡ��ǰ��������ݵ����һ��
	if (CONFIG_STAP == (ANTI_CONFIG_SEL & sel))
	{
		cudaMemcpy(send_res_GLO1.var2_real, STAP_array_matrix_R_inver[0],
			send_res_GLO1.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);

		//��ʼ��ʱ��Ϊ0
		//memset(send_res_L1.var2_image, 0, send_res_L1.dim * sizeof(SIGNAL_TYPE));
	}
	else if (CONFIG_BEAM == (ANTI_CONFIG_SEL & sel))
	{
		cudaMemcpy(send_res_GLO1.var2_real, STAP_array_matrix_R_inver[0],
			send_res_GLO1.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);

		cudaMemcpy(send_res_GLO1.var2_image, STAP_array_matrix_R_inver[0] +
			send_res_GLO1.dim, send_res_GLO1.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);
	}
	else
	{
		//��ʼ��ʱ��Ϊ0
		/*memset(send_res_L1.var2_real, 0, send_res_L1.dim * sizeof(SIGNAL_TYPE));
		memset(send_res_L1.var2_image, 0, send_res_L1.dim * sizeof(SIGNAL_TYPE));*/
	}

	{
		int tmp_count = sameple_rate *1e-3;
		send_res_GLO1.var3 = 0;
		for (int i = 0; i <tmp_count; i++)
		{
			send_res_GLO1.var3 += ((SIMRESVAR)GLO1_receiver_buf.buf_ptr[i]) *
				((SIMRESVAR)GLO1_receiver_buf.buf_ptr[i]);
		}
		send_res_GLO1.var3 /= tmp_count;
		send_res_GLO1.var3 = 10 * log(send_res_GLO1.var3);
	}

	for (int i = 0; i < SAT_NUM; i++)
	{
		send_res_GLO1.var4[i] = GLO1_chan[i].i_prompt;
		send_res_GLO1.var5[i] = GLO1_chan[i].CNo;
		send_res_GLO1.var6[i] = ((long long)(GLO1_chan[i].phaseLockDetector + 0.5)) & 0xffffffffUL;

		send_res_GLO1.var7_az[i] = GLO1_chan[i].az*GLO1_r_to_d;
		send_res_GLO1.var7_el[i] = GLO1_chan[i].el*GLO1_r_to_d;
		send_res_GLO1.var8_prn[i] = GLO1_chan[i].prn;

		send_res_GLO1.var10_x[i] = GLO1_track_sat[i + 1].x;
		send_res_GLO1.var10_y[i] = GLO1_track_sat[i + 1].y;
		send_res_GLO1.var10_z[i] = GLO1_track_sat[i + 1].z;
	}
	send_res_GLO1.var9_GDOP = GLO1_gdop;
	send_res_GLO1.var9_HDOP = GLO1_hdop;
	send_res_GLO1.var9_TDOP = GLO1_vdop;
	send_res_GLO1.var9_VDOP = GLO1_tdop;
	send_res_GLO1.var9_PDOP = GLO1_pdop;

	send_res_GLO1.var11_x = navout.X;
	send_res_GLO1.var11_y = navout.Y;
	send_res_GLO1.var11_z = navout.Z;

	send_res_GLO1.var12_he = navout.height;
	send_res_GLO1.var12_la[0] = GLO1_cur_lat.deg, send_res_GLO1.var12_la[1] = abs(GLO1_cur_lat.min), send_res_GLO1.var12_la[2] = fabs(GLO1_cur_lat.sec);
	send_res_GLO1.var12_lo[0] = GLO1_cur_long.deg, send_res_GLO1.var12_lo[1] = abs(GLO1_cur_long.min), send_res_GLO1.var12_lo[2] = fabs(GLO1_cur_long.sec);

	send_res_GLO1.var13[0] = GLO1_rpvt.xv, send_res_GLO1.var13[1] = GLO1_rpvt.yv, send_res_GLO1.var13[2] = GLO1_rpvt.zv;
	send_res_GLO1.var14 = GLO1_clock_offset;

	localtime_s(&local_time, &GLO1_thetime);
	send_res_GLO1.time_y = local_time.tm_year;
	send_res_GLO1.time_mon = local_time.tm_mon;
	send_res_GLO1.time_d = local_time.tm_mday;
	send_res_GLO1.time_h = local_time.tm_hour;
	send_res_GLO1.time_min = local_time.tm_min;
	send_res_GLO1.time_s = local_time.tm_sec;

	send_res_GLO1.var16 = GLO1_gps_week % 1024;
	send_res_GLO1.var17 = GLO1_clock_tow;


	sendto(socket_client, (char *)&send_res_GLO1, sizeof(SimRes), 0, (sockaddr *)&socket_sin, socket_len);
}

/*******************************************************************************
FUNCTION display()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function displays the current GLO1_status of the GLO1_receiver on the
	computer screen.  It is called when there is nothing else to do

*******************************************************************************/
void GLO1_display(void)
{
	
	char ch;
#ifdef BCPP
	gotoxy(1,1);
#endif

	printf("%s",ctime(&GLO1_thetime));        
	printf("TOW  %6ld\n",GLO1_clock_tow);                         //������
	printf("meas time %f  error %f \n",GLO1_m_time[1],GLO1_m_error);   //GLO1_m_time[1]:����ʱ�䡣GLO1_m_error�����ջ��Ĳ���ʱ���GPS��������
	GLO1_cur_lat.deg = int(GLO1_rec_pos_llh.lat*GLO1_r_to_d);
	GLO1_cur_lat.min = int((GLO1_rec_pos_llh.lat*GLO1_r_to_d-GLO1_cur_lat.deg)*60);
	GLO1_cur_lat.sec = float((GLO1_rec_pos_llh.lat*GLO1_r_to_d-GLO1_cur_lat.deg-GLO1_cur_lat.min/60.)*3600.);
	GLO1_cur_long.deg = int(GLO1_rec_pos_llh.lon*GLO1_r_to_d);
	GLO1_cur_long.min = int((GLO1_rec_pos_llh.lon*GLO1_r_to_d-GLO1_cur_long.deg)*60);
	GLO1_cur_long.sec = float((GLO1_rec_pos_llh.lon*GLO1_r_to_d-GLO1_cur_long.deg-GLO1_cur_long.min/60.)*3600.);
	printf("   latitude    longitude          HAE      clock error (ppm)\n");
	printf("  %4d:%2d:%5.2f  %4d:%2d:%5.2f  %10.2f  %f\n",
	GLO1_cur_lat.deg,abs(GLO1_cur_lat.min),fabs(GLO1_cur_lat.sec),GLO1_cur_long.deg,abs(GLO1_cur_long.min),
	fabs(GLO1_cur_long.sec),GLO1_rec_pos_llh.hae,GLO1_clock_offset); //GLO1_clock_offset:���ջ���Ư
	printf(" Speed      vx		vy	  vz      Heading      TIC_dt\n");
	printf(" %lf   %lf   %lf   %lf   %lf   %lf\n", GLO1_speed, GLO1_rpvt.xv, GLO1_rpvt.yv, GLO1_rpvt.zv, GLO1_heading*GLO1_r_to_d, GLO1_TIC_dt);  //GLO1_speed:���ջ��ٶȵľ���ֵ;GLO1_heading�����ջ��ķ�λ�ǣ�GLO1_TIC_dt�����μ��㵼����ļ��
	printf("   \n");

	printf("tracking %2d GLO1_status %1d almanac valid %1d gps week %4d\n",
	GLO1_n_track,GLO1_status,GLO1_almanac_valid,GLO1_gps_week%1024);

	if (GLO1_display_page==0)
	{
		//printf(" ch prn  state az  el  doppler  t_count n_frame sfid ura page CNo  PL\n");
		printf(" ch prn K  state  az  el  doppler  t_count n_frame sfid ura page CNo  PL\n");//todo:��ʾprn��Ϊ��ʾK
		for (ch=0;ch<=GLO1_chmax;ch++)
		{
			printf(" %2d %2d  %2d  %2d  %4.0f  %3.0f   %7.1f   %4d  %4d  %2d  %3d  %3d   %4.1f  %1d\n",
				//ch,GLO1_chan[ch].prn, GLO1_chan[ch].K, GLO1_chan[ch].state,
				ch, GLO1_chan[ch].prn, GLO1_chan[ch].K, GLO1_chan[ch].state, //todo:��ʾprn��Ϊ��ʾK
				GLO1_chan[ch].az*GLO1_r_to_d,GLO1_chan[ch].el*GLO1_r_to_d,                                //az:���ǵķ�λ�ǣ�el�����ǵ�����
				GLO1_chan[ch].vDoppler,GLO1_chan[ch].t_count,GLO1_chan[ch].n_frame,GLO1_chan[ch].sfid,    //vDoppler:�����˶�������doppler����;t_count:�յ��ı��ؼ���������ѭ����ʽ����0-1499
				GLO1_gps_eph[GLO1_chan[ch].prn].ura,GLO1_chan[ch].page5,GLO1_chan[ch].CNo, (long)((long long)(GLO1_chan[ch].phaseLockDetector+0.5) & 0xffffffffUL));
		       //ura���û����뾫��ָ����page5�����������е�page�ţ�CNO�����Ƶ��ŵ�����ȣ�phaseLockDetector���ز���λ�Ƿ�������ָʾ
			
			chinfo[ch].prn     = GLO1_chan[ch].prn;
			chinfo[ch].state   = GLO1_chan[ch].state;
			chinfo[ch].az      = GLO1_chan[ch].az;
			chinfo[ch].el      = GLO1_chan[ch].el;
			chinfo[ch].doppler = GLO1_chan[ch].doppler;
			chinfo[ch].t_count = GLO1_chan[ch].t_count;
			chinfo[ch].n_frames= GLO1_chan[ch].n_frame;     // �Ѿ��յ���֡����
			chinfo[ch].sfid    = GLO1_chan[ch].sfid;        // ��֡�ı��
			chinfo[ch].ura     = GLO1_gps_eph[GLO1_chan[ch].prn].ura;
			chinfo[ch].page    = GLO1_chan[ch].page5;
			chinfo[ch].CN0     = GLO1_chan[ch].CNo;

			CNR[ch] = chinfo[ch].CN0;

			corrpeak[ch].i_early   = GLO1_chan[ch].i_early; 
			corrpeak[ch].i_prompt  = GLO1_chan[ch].i_prompt;
			corrpeak[ch].i_late    = GLO1_chan[ch].i_late;
		}

		/*BEAM*/
		if (CONFIG_BEAM == (ANTI_CONFIG_SEL & sel))
		{
			bool is_EL_AZ_changed = false;

			int tmp_AZ_sj;
			int tmp_EL_si;

			for (ch = 0; ch <= GLO1_chmax; ch++)
			{
				//�޸�ȫ�ֵĸ����Ǻͷ�λ��
				tmp_AZ_sj = GLO1_chan[ch].az*GLO1_r_to_d;
				tmp_EL_si = 90 - GLO1_chan[ch].el*GLO1_r_to_d;
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

		printf(" GDOP=%6.3f  HDOP=%6.3f  VDOP=%6.3f  TDOP=%6.3f PDOP=%6.3f\n\n", GLO1_gdop, GLO1_hdop, GLO1_vdop, GLO1_tdop, GLO1_pdop);
	}
	else if (GLO1_display_page==1)
	{
		printf(  " ch prn state TLM      TOW  Health  Valid  TOW_sync offset\n");
		for (ch=0;ch<=GLO1_chmax;ch++)
		{
			printf(" %2d %2d  %2d  %6ld   %6ld   %2d     %2d     %2d   %4d\n",
				ch, GLO1_chan[ch].prn, GLO1_chan[ch].state, GLO1_chan[ch].TLM, GLO1_chan[ch].TOW,
				GLO1_gps_eph[GLO1_chan[ch].prn].health,GLO1_gps_eph[GLO1_chan[ch].prn].valid,GLO1_chan[ch].tow_sync,
				GLO1_chan[ch].offset);
		}
	}
	else if (GLO1_display_page==2)
	{
		printf(" ch prn state n_freq az  el        tropo        iono\n");
		for (ch=0;ch<=GLO1_chmax;ch++)
		{
			printf(" %2d %2d  %2d  %4.0f  %3.0f   %10.4lf   %10.4lf\n",
				ch, GLO1_chan[ch].prn, GLO1_chan[ch].state,
				GLO1_xyz[GLO1_chan[ch].prn].azimuth*GLO1_r_to_d, GLO1_xyz[GLO1_chan[ch].prn].elevation*GLO1_r_to_d,
				GLO1_chan[ch].Tropo*GLO1_SPEEDOFLIGHT, GLO1_chan[ch].Iono*GLO1_SPEEDOFLIGHT);
		}
	}
	else if (GLO1_display_page==3)
	{
		printf(" ch prn state      Pseudorange     delta Pseudorange\n");
		for (ch=0;ch<=GLO1_chmax;ch++)
		{
			printf(" %2d %2d  %2d  %20.10lf   %15.10lf\n",
				ch, GLO1_chan[ch].prn, GLO1_chan[ch].state, GLO1_chan[ch].Pr, GLO1_chan[ch].dPr);
		}
	}
	else if (GLO1_display_page==4)
	{
	}
	//add by jh ui
	//::PostMessage(ChannelWnd, WM_CHANNEL_INFORMATION, (WPARAM)chinfo, (LPARAM)12);

	sprintf(navout.lat, "%4d:%2d:%5.2f", GLO1_cur_lat.deg,abs(GLO1_cur_lat.min),fabs(GLO1_cur_lat.sec));
	sprintf(navout.lon, "%4d:%2d:%5.2f", GLO1_cur_long.deg,abs(GLO1_cur_long.min),fabs(GLO1_cur_long.sec));
	navout.height = GLO1_rec_pos_llh.hae;
	navout.gdop = GLO1_gdop;
	navout.hdop = GLO1_hdop;
	navout.vdop = GLO1_vdop;
	navout.tdop = GLO1_tdop;
	strcpy(navout.time, ctime(&GLO1_thetime));
	navout.ve = GLO1_receiver.vel.east;
	navout.vn = GLO1_receiver.vel.north;
	navout.vu = GLO1_receiver.vel.up;

	GLO1_XYZ GLO1_wgs;
	
	GLO1_wgs = llh_to_ecef(GLO1_rec_pos_llh);
	navout.X = GLO1_wgs.x;
	navout.Y = GLO1_wgs.y;
	navout.Z = GLO1_wgs.z;
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
	our own custom function. The old vector is stored GLO1_in a global variable
	and will be reinstalled at the end of program execution. IRQ0 is
	enabled by altering the interrupt mask stored by the 8259 interrupt
	handler.

*******************************************************************************/
#ifdef REAL_TIME
void Interrupt_Install()
{
  unsigned char     int_mask,i_high,i_low;
  i_high=GLO1_interr_int>>8;
  i_low=GLO1_interr_int&0xff;
  Old_Interrupt = getvect(8 + GLO1_IRQLEVEL);
  disable();
  setvect(8 + GLO1_IRQLEVEL,GPS_Interrupt);
  int_mask = inportb(0x21);   
  int_mask = int_mask & ~(1 << GLO1_IRQLEVEL);
  outportb(0x21,int_mask);  
  enable();

  outportb(0x43,0x34);
  outportb(0x40,i_low);
  outportb(0x40,i_high);
  outportb(0x20,0x20);
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

  outportb(0x20,0x20);           // clear interrupt and allow next one
  int_mask = inportb(0x21);      // get hardware interrupt mask
  int_mask = int_mask | (1 << GLO1_IRQLEVEL);
  disable();
//outportb(0x21,int_mask);       // send new mask to 8259
  setvect(8 + GLO1_IRQLEVEL,Old_Interrupt);
  enable();                      // allow hardware interrupts
  outportb(0x20,0x20);           // clear interrupt and allow next one
  outportb(0x43,0x34);           // reset clock
  outportb(0x40,0xff);
  outportb(0x40,0xff);
}
#endif

/*******************************************************************************
FUNCTION GPS_Interrupt()

RETURNS  None.

PARAMETERS None.

PURPOSE
	This function replaces the current IRQ0 Interrupt service routine with
	our GPS function which will perform the GLO1_acquisition - tracking functions
������������ǵ�ִ�в�����ٵ�GPS����������Ŀǰ��IRQ0�жϷ������
*******************************************************************************/
#ifdef REAL_TIME
void interrupt GPS_Interrupt(...)
#else
void GLO1_Sim_GPS_Interrupt()
#endif
{
	unsigned int add;
	char ch;
	double carrierPhase;



	bool bAcq=false;
	for (ch=0; ch<=GLO1_chmax; ch++)
	{
		if (GLO1_correlator[ch].ready == 1) 
		{
			GLO1_ReadAccumData(ch);//���������1����Ļ���ֵ
				/************************************************************************/
			GLO1_correlator[ch].ready = 0;
			
			switch(GLO1_chan[ch].state)
			{
			case GLO1_acquisition:	
				GLO1_ch_acq(ch);
				break;
			case GLO1_confirm    :	
				break;
			case GLO1_pull_in    :	// pull-GLO1_inģ��Ҫ���λͬ���Ĺ���
				GLO1_ch_pull_in(ch);
				break;
			case GLO1_track      :
				GLO1_ch_track(ch);
				break;
			}
		}
	}
	//��correlatorProcess�����100ms����GLO1_TIC_OCCUR_FLAG = 1;	// ���ò����жϱ�־���ñ�־���ⲿ���жϷ���������
	if (GLO1_TIC_OCCUR_FLAG == 1)                  // ������һ�������ж�,�����ŵ���ͬʱ����
	{
		GLO1_tic_count=(++GLO1_tic_count)%10;
		if (GLO1_tic_count==0) GLO1_sec_flag=1;	

		GLO1_hms_count=(++GLO1_hms_count)%600;
		if (GLO1_hms_count==0) GLO1_min_flag=1;	

		GLO1_nav_count=(++GLO1_nav_count)%GLO1_nav_tic;	// GLO1_nav_tic= (int)(GLO1_nav_up/0.1+0.5);
		if (GLO1_nav_count==0) GLO1_nav_flag=1;		// ���㵼�����ʱ�䵽

		GLO1_TIC_sum += GLO1_TIC_CNTR+1;						// �ۼ����ε�����֮��Ĳ���������
		add=1;

		// ����λ�����������ÿ�ζ����������ܼ������뱣����Ϊʲô������
		/************************************************************************/
		/*
		������Ϊ�������100����������һ�������ز�����������ҲҪ�ڲ����жϵ���ʱ��
		ȡһ�����������ز�������������һ���ʱ���������ز����ܵĻ��Ͳ���1���ۼӵģ�
		����100�����ۼӵ������ԭ�����Ҿ�������������������������100ms���治�����ˣ�
		ֱ����1�뵽��ʱ��ֱ��ȡ��������ۼӵ������ܲ��Ϳ����ˣ�
		*/
		/************************************************************************/
		// 100�����ȡһ�����ܼ���������1��Ŷ�һ������λ�������
		// �ز���λ�����ã�������С������Ϊ�˲�����������
		for (ch=0;ch<=GLO1_chmax;ch++)
		{
			if (GLO1_chan[ch].state!=GLO1_track)//Ŀǰͨ��״̬���Ǹ��پͲ���������Ķ�������С���ز���λ
			{
				continue;
			}
			//���ͨ��״̬�Ǹ��٣��Ͷ������������ز���λ�����ܺ�С��
			carrierPhase = GLO1_readCarrierPhase(ch);
			//hide by jh 4-9 carrierPhase = GLO1_readCarrierPhase(ch);//����������ۼ����ز���λ
			/************************************************************************/
			/*
			return (GLO1_correlator[ch].carrierCycleLatch//���������for (i=0;i<=GLO1_chmax;i++){//ÿ100ms����һ��GLO1_correlator[i].carrierCycleLatch = GLO1_correlator[i].carrierCycle;// ʵʱ���ز���λ������
			+(double)GLO1_correlator[ch].phaseLocalLatch*carrierNCORes);//#define carrierNCORes (double) 9.31322574615478515625e-10  //2^-30
			//���������for (i=0;i<=GLO1_chmax;i++){//ÿ100ms����һ��GLO1_correlator[i].phaseLocalLatch = GLO1_correlator[i].phaseLocal;// ʵʱ���ز���λ��С������
			//ΪʲôС������Ҫ����2^-30������2^30����Ϊ�ز�NCO��30λ�ģ�С��������30λ������Ӧ������2^30
			*/
			/************************************************************************/
			//�����ɵ�ƣ�ǰ�����һ������Ȼ�����滹����������ȥ�õ�һ��ʵ���������ַ���ȥȡ����ɶ���ְ�ʵ���ֿ����β��ɴ������︳ֵ��
			GLO1_chan[ch].cycle_sum += ((long) ((long long)carrierPhase & 0xffffffffUL));		
			GLO1_chan[ch].carr_dco_phase = fmod(carrierPhase,1.0);
		}

		if (GLO1_nav_count==0)//1���ӵ����ü��㵼������  Ϊ��Ҫȡ��������              // ��ȡ�������ݼ��㵼����
		{
			for (ch=0;ch<=GLO1_chmax;ch++)
			{
				if (GLO1_chan[ch].state!=GLO1_track)
				{
					continue;
				}
				GLO1_chan[ch].codePhase = GLO1_readCodePhase(ch);
				//���ͨ��״̬�Ǹ��٣����ȡ����λ��ʱ�䣬�����յȣ����ڼ���α�� ������ߵ�������������GLO1_nav_fix

				//hide by jh 4-9 GLO1_chan[ch].codePhase = GLO1_readCodePhase(ch);//return (double)GLO1_correlator[ch].tauLatch/(1.023e6*GLO1_d_2p40);
				//���������if (GLO1_TIC_RT_CNTR == (GLO1_TIC_CNTR+1))��100ms����GLO1_correlator[i].tauLatch = GLO1_correlator[i].tau;// ʵʱ������λ
				GLO1_chan[ch].epoch=GLO1_readEpoch(ch);//hide by jh 4-9 GLO1_chan[ch].epoch=GLO1_readEpoch(ch);//return GLO1_correlator[ch].epochCounter;
				//���������if (GLO1_TIC_RT_CNTR == (GLO1_TIC_CNTR+1))��100ms����GLO1_correlator[i].epochCounter = GLO1_correlator[i].epochCounterCheck;// ʵʱepoch������

				GLO1_chan[ch].meas_bit_time=GLO1_chan[ch].tr_bit_time;
				//��GLO1_ch_track����pChan->tr_bit_time++;
				//��GLO1_pream����GLO1_chan[ch].tr_bit_time=TOWs*300-240;
				GLO1_chan[ch].doppler=GLO1_chan[ch].fc;
				//��GLO1_ch_acq����GLO1_chan[ch].fc = doppler;
				//��GLO1_ch_pull_in����GLO1_chan[ch].fc += freqErr;//��GLO1_ch_acq�������������GLO1_chan[ch].fc = doppler;  �Ѽ�������Ƶ�������϶�����
				//��GLO1_ch_track��pChan->fc = *cLoopS1+GLO1_PLLc2*phaseErr;

				// GLO1_chan[ch].carrier_counter����������β����жϼ䣬�ز���λ��������������
				// GLO1_chan[ch].d_carr_phase����������β����жϼ䣬�ز���λ������С������
				GLO1_chan[ch].carrier_counter=GLO1_chan[ch].cycle_sum;//GLO1_chan[ch].cycle_sum += ((long)carrierPhase);
				GLO1_chan[ch].cycle_sum=0;
				GLO1_chan[ch].d_carr_phase=GLO1_chan[ch].carr_dco_phase - GLO1_chan[ch].old_carr_dco_phase;//GLO1_chan[ch].carr_dco_phase = fmod(carrierPhase,1.0);	// ��ȡ�ز���λ��С������
				GLO1_chan[ch].old_carr_dco_phase=GLO1_chan[ch].carr_dco_phase;// ΪʲôҪ��������100�����С���ز���-�ϴ�100�����С���ز���
				// ��Ϊ��εõ���С���ز��������ϴ�С���ز����ϼ����ۼӵõ��ģ�Ȼ���ۼ���������Լ�ȥ�ϴε�С���ز�����������������һ��
				// 100����������С������
			}

			// ע����������ۼ�GLO1_i_TIC_dt�ķ����ܲ�ͬ
			// ��ʵʱģʽ�£�GLO1_nav_fix������GLO1_TIC_CNTR�����޸�ʱ��
			// һ������TIC�������ڽ��У������þɵ�CNTR��ɼ�������
			// �µ�GLO1_TIC_CNTRֻ�ܴ���һ�βŻᱻװ��
			// ����ʱ��TIC_CNRT����װ��
			/* hide by jh 3-12
			#ifdef REAL_TIME
			GLO1_i_TIC_dt= GLO1_TIC_sum+old_TIC_cntr-GLO1_TIC_CNTR;//��ǰ��GLO1_TIC_sum += GLO1_TIC_CNTR+1;// �ۼ����ε�����֮��Ĳ���������
			//��main��old_TIC_cntr = GLO1_TIC_ref;// long GLO1_TIC_ref=(long)(GLO1_SAMPLING_FREQ*0.1+0.5-1); 100ms��Ӧ���жϼ���ֵ
			//��main��GLO1_programTIC(GLO1_TIC_ref);// ��ЧΪ: GLO1_TIC_CNTR = GLO1_TIC_ref
			//��GLO1_nav_fix��if (GLO1_align_t==1)//// �����ջ�ʱ���GPSʱ���׼�ı�־{old_TIC_cntr=GLO1_TIC_CNTR;
			//// GLO1_i_TIC_dt�����μ���λ�õļ��
			//��GLO1_nav_fix��GLO1_TIC_dt=GLO1_i_TIC_dt*GLO1_SAMPLING_INT;		//each clock count is 175 ns for GP2021
			#else
			*/
#ifdef REAL_TIME
#else
			GLO1_i_TIC_dt= GLO1_TIC_sum;//100����ĵ���
							  /************************************************************************/
							  /*
							  // GLO1_i_TIC_dt�����μ���λ�õļ������100����ĵ���
							  GLO1_TIC_dt=GLO1_i_TIC_dt*GLO1_SAMPLING_INT;		//each clock count is 175 ns for GP2021
							  // GLO1_SAMPLING_INT��һ��������Ҫ����ʱ�䣬����100����ĵ������õ�100����ĵ�ȼ۵�ʱ��
							  */
							  /************************************************************************/
							  //hide by jh 3-16#endif
#endif

			GLO1_TIC_sum=0;
		}
		GLO1_TIC_OCCUR_FLAG = 0;	//����жϱ�־���ȴ���һ��
		{
			GLO1_ch_alloc();//ÿ100����Ĳ����жϷ��������·���һ��ͨ������Ϊ������ǰ��Ĳ������ʲô�ģ��е�����״̬�����ı�
		} 
		}

}

void GLO1_initTrackLoopPar()
{
	double DLLK1, DLLK2;
	double BDLL, DLLkesi;
	double BPLL, PLLomega0;
	double BFLL, FLLkesi;
	
	long i, ch;
	// DLL parameters
	DLLK1=1.0;
	DLLK2 =1.0;
	BDLL = 0.5;	
	DLLkesi = pow(2.0,-0.5);

	GLO1_DLLc1 = pow((8*DLLkesi*BDLL/(1+4*DLLkesi*DLLkesi)),2.0)/(DLLK1*DLLK2);
	GLO1_DLLc2 = 16*DLLkesi*DLLkesi*BDLL/(1+4*DLLkesi*DLLkesi)/DLLK1/DLLK2;

	BPLL = 15.0;	
	PLLomega0 = BPLL/0.78445;
	GLO1_PLLc0=pow(PLLomega0,3.0);
	GLO1_PLLc1=1.1*PLLomega0*PLLomega0;
	GLO1_PLLc2=2.4*PLLomega0;
	FLLkesi = 0.707;
	BFLL = 10.0;				
	//changed by Ning Luo GLO1_in Sept/04, 5.0Hz->10Hz

	GLO1_FLLa1=pow((8*FLLkesi*BFLL/(1+4*FLLkesi*FLLkesi)),2.0);
	GLO1_FLLa2=16*FLLkesi*FLLkesi*BFLL/(1+4*FLLkesi*FLLkesi);

	for (ch=0; ch<=GLO1_chmax; ch++)
	{
		GLO1_chan[ch].BitSyncIndex = 0;	
		GLO1_chan[ch].IQBufSize = 0;		
		for (i=0;i<GLO1_BUFSIZE;i++)		
		{
			GLO1_chan[ch].signBuf[i] = 0;
		}
		for (i=0;i<20;i++)			
		{
			GLO1_chan[ch].kCell[i] = 0;
		}
	}
}

int GLO1_freq_alloc(char j) //Ϊ��ͬ���Ƿ���Ƶ���ţ�TODO��
{
	int K;
	switch (j)//����Ƶ����todo:
	{
	case 1:	  K = 1;	break;
	case 2:	  K = -4;	break;
	case 3:	  K = -5;	break;//�����ϲ�һ�£���TODO��
	case 4:	  K = 6;	break;
	case 5:	  K = 1;	break;
	case 6:	  K = -4;	break;
	case 7:	  K = 5;	break;
	case 8:	  K = 6;	break;
	case 9:	  K = -2;	break;
	case 10:  K = -7;	break;
	case 11:  K = 0;	break;
	case 12:  K = -1;	break;
	case 13:  K = -2;	break;
	case 14:  K = -7;	break;
	case 15:  K = 0;	break;
	case 16:  K = -1;	break;
	case 17:  K = 4;	break;
	case 18:  K = -3;	break;
	case 19:  K = 3;	break;
	case 20:  K = 2;	break;
	case 21:  K = 4;	break;
	case 22:  K = -3;	break;
	case 23:  K = 3;	break;
	case 24:  K = 2;	break;
	}
	return K;
}

void GLO1_hot_ch_alloc()
{	
	char i;
	short allocFlag;
	
	GLO1_read_prn();
	for (i=0; i<=GLO1_chmax; i++)
	{
		if (GLO1_chan[i].state != GLO1_CHN_OFF)
		{
			continue;
		}
		allocFlag = 0;			
		
		if (GLO1_last_prn[i]>0)
		{
			allocFlag = 1;
			GLO1_svStruct[i].state = GLO1_USING;
			GLO1_chan[i].prn = GLO1_last_prn[i];
			GLO1_chan[i].state = GLO1_acquisition;
			GLO1_svStruct[i].NumToTryAcq = 20;
			GLO1_svStruct[i].NumBTWEachTrial = 1000;
			GLO1_correlator[i].state = GLO1_CHN_OFF;
			GLO1_correlator[i].ready = 1;
			GLO1_correlator[i].sv = GLO1_last_prn[i];
		}				
		
	}

}
void GLO1_ch_alloc()//һ�������жϽ���һ��
{
	char i,j;
	short allocFlag;

	GLO1_almanac_valid=1;// ���Almanac�Ƿ���Ч
	for (i=GLO1_MinSvNumber; i<=GLO1_MaxSvNumber; i++)//��32��ͨ��
	{
		GLO1_xyz[i]=GLO1_satfind(i);//�ڴ�GLO1_ch_alloc����֮ǰ��time(&GLO1_thetime);
		if (GLO1_gps_alm[i].inc>0.0 && GLO1_gps_alm[i].week!=GLO1_gps_week%1024)//�ж�������Ч��
		{
			GLO1_almanac_valid=0;
			break;
		}
	}
	if (GLO1_al0==0.0 && GLO1_b0==0.0) GLO1_almanac_valid=0;

	// ������е����ǣ����ĳЩ����δ����⵽�źţ�����Щ���Ǳ��뱻����һ��ʱ��
	// �Ա�֤�����������л��ᱻ����������
	for (i=GLO1_MinSvNumber; i<=GLO1_MaxSvNumber; i++)
	{
		if (GLO1_svStruct[i].state==GLO1_HOLD)//GLO1_HOLD����fft����û��ֱ�Ӳ��񵽷�ֵ�����õ�
		{
			if (GLO1_svStruct[i].NumToTryAcq==0)//�����������
			{
				GLO1_svStruct[i].NumToTryAcq=1;
				GLO1_svStruct[i].NumBTWEachTrial=GLO1_ReAcqTime;
				GLO1_svStruct[i].maxf=GLO1_MAXF;
				GLO1_svStruct[i].minf=GLO1_MINF;
			}
			GLO1_svStruct[i].undetectedCounter++;
			if (GLO1_svStruct[i].undetectedCounter>GLO1_svStruct[i].NumBTWEachTrial)
			{
				GLO1_svStruct[i].state = GLO1_AVAILABLE;	
				GLO1_svStruct[i].undetectedCounter = 0;
			}
		}
	}

	for (i=0; i<=GLO1_chmax; i++)
	{
		if (GLO1_chan[i].state != GLO1_CHN_OFF)// ����ŵ��Ѿ���⵽���ź�/��ռ�ã����������ŵ��ļ��
		{
			continue;
		}
		
		// Ϊ�����ŵ�ѡ��һ������
		// check all satellites
		allocFlag = 0;
		for (j=GLO1_MinSvNumber; j<=GLO1_MaxSvNumber; j++)
		{
			if (GLO1_almanac_valid == 1)// ���Almanac�Ƿ���Ч
			{
				if (GLO1_xyz[j].elevation > GLO1_mask_angle && 
					GLO1_gps_alm[j].health == 0 &&
					GLO1_gps_alm[j].ety != 0.00 &&
					GLO1_svStruct[j].state == GLO1_AVAILABLE)
				{
					allocFlag = 1;
					GLO1_svStruct[j].state = GLO1_USING;
					GLO1_chan[i].prn = j;
					GLO1_chan[i].K = GLO1_freq_alloc(GLO1_chan[i].prn);//����Ƶ���ţ�todo��
					GLO1_chan[i].IF = GLO1_CARRIER_FREQ + GLO1_chan[i].K * 562500;//ÿ��ͨ�����ǵ���Ƶ��todo��
					GLO1_chan[i].state = GLO1_acquisition;
					GLO1_svStruct[j].NumToTryAcq = 20;//���������Ϣ�У���ÿ�����ǵ��Բ�����Ϊ20�Σ�����ֻ�Բ�һ�Σ�ֱ���´�GLO1_ch_alloc�����ڸ����ﵱ�����<30ʱ����Ϊ20��
					GLO1_svStruct[j].NumBTWEachTrial = 1000;
					GLO1_correlator[i].state = GLO1_CHN_OFF;
					GLO1_correlator[i].ready = 1;
					GLO1_ch_cntl(i,j);//iΪͨ���ţ�jΪ���Ǻ�
					break;
				}
			}
			else
			{
				if (GLO1_svStruct[j].state == GLO1_AVAILABLE)
				{
					// k = -7 --6   0--13   index = k+7
					int k = GLO1_freq_alloc(j);
					if(GLO1_alloced_k[k+7]==1){
						continue;
					}else{
						GLO1_alloced_k[k+7]=1;
					}

					allocFlag = 1;
					GLO1_svStruct[j].state = GLO1_USING;
					GLO1_chan[i].prn = j;
					GLO1_chan[i].K = GLO1_freq_alloc(GLO1_chan[i].prn);//todo
					GLO1_chan[i].IF = GLO1_CARRIER_FREQ + GLO1_chan[i].K * 562500;//ÿ��ͨ�����ǵ���Ƶ��todo��
					GLO1_chan[i].state = GLO1_acquisition;
				    if(GLO1_svStruct[j].NumToTryAcq==0)
					{
					   GLO1_svStruct[j].NumToTryAcq = 1;
					   GLO1_svStruct[j].NumBTWEachTrial = GLO1_ReAcqTime;//1000000
					   GLO1_svStruct[j].maxf=GLO1_MAXF;
					   GLO1_svStruct[j].minf=GLO1_MINF;
					}
				    GLO1_correlator[i].state = GLO1_CHN_OFF;
				    GLO1_correlator[i].ready = 1;
					GLO1_ch_cntl(i,j);
					break;
				}
			}
		}
		if (allocFlag == 0) break;
    }
}


char GLO1_first_flag = 1;
// linear FFT detector
void GLO1_ch_acq(char ch)
{
	double Rmax;
	double tau, doppler;
	long dataLength;
	int flag;
	long bufOffset;
	long numFreqBin, i;
	long nTrial;
	double freqOffset[100];
	//s-tatic char first_flag = 1;
	double *pBuf;
	char *pData;
	//sta-tic double BUFTABLE[2]={-1.0, 1.0};//add by jh ui

	pBuf = &GLO1_buffer[ch][GLO1_globalBufLength[ch]];
	pData = GLO1_data;
	for(i=0;i<GLO1_correlatorDataLength;i++)
	{
		*pBuf++ = *pData++;
	}

	GLO1_globalBufLength[ch] += GLO1_correlatorDataLength;

	#ifndef REAL_TIME 
	GLO1_correlator[ch].ready = 1;
	#endif

	if (GLO1_globalBufLength[ch] < GLO1_DETECTSIZE)
		return;
	if (GLO1_globalBufLength[ch] > (GLO1_DETECTSIZE+30000))
	{
		printf("computation abnormal, system exits");
		exit(0);
	}

	bufOffset = GLO1_globalBufLength[ch] - GLO1_DETECTSIZE;
	dataLength = GLO1_DETECTSIZE;
    numFreqBin = (long)((long long)((GLO1_svStruct[GLO1_chan[ch].prn].maxf - GLO1_svStruct[GLO1_chan[ch].prn].minf) / GLO1_DELTF + 1.5) & 0xffffffffUL);
	for(i=0;i<numFreqBin;i++) 
		freqOffset[i]=i*GLO1_DELTF+GLO1_svStruct[GLO1_chan[ch].prn].minf;

	if (GLO1_first_flag == 1)
	{
		double threshold = 0.0;
		flag=GLO1_acqCA(&GLO1_buffer[ch][bufOffset],dataLength, GLO1_caTable, 1, GLO1_SAMPLING_FREQ,freqOffset,numFreqBin,
			GLO1_CARRIER_FREQ,&tau,&doppler,&nTrial,&Rmax,&threshold);
		
		//GLO1_threshold_sig_search = Rmax*1.5f;
		GLO1_threshold_sig_search = Rmax*1.0f;
		GLO1_first_flag = 0;
	}
	//flag=GLO1_acqCA(&GLO1_buffer[ch][bufOffset],dataLength, GLO1_caTable, GLO1_chan[ch].prn, GLO1_SAMPLING_FREQ,freqOffset,numFreqBin,
	//	GLO1_CARRIER_FREQ,&tau,&doppler,&nTrial,&Rmax,&GLO1_threshold_sig_search);
	flag = GLO1_acqCA(&GLO1_buffer[ch][bufOffset], dataLength, GLO1_caTable, GLO1_chan[ch].prn, GLO1_SAMPLING_FREQ, freqOffset, numFreqBin,
		GLO1_chan[ch].IF, &tau, &doppler, &nTrial, &Rmax, &GLO1_threshold_sig_search);//todo
	GLO1_globalBufLength[ch] = 0;
	if (flag==GLO1_DETECTED)
	{
	
		GLO1_writeCodePhase(ch,tau);

		GLO1_writeCodeFreq(ch,1.0);

		//GLO1_writeCarrierFreq(ch,GLO1_carrier_ref+doppler);
		GLO1_writeCarrierFreq(ch, GLO1_chan[ch].IF + doppler);//todo

		#ifndef REAL_TIME
		GLO1_correlator[ch].state = GLO1_CHN_ON;
		GLO1_correlator[ch].ready = 0;
		#endif
		GLO1_chan[ch].state = GLO1_pull_in;
		GLO1_chan[ch].cLoopS1 = doppler;
		GLO1_chan[ch].cLoopS0 = 0;
		GLO1_chan[ch].dllS0 = 0.0;
		GLO1_chan[ch].dllS1 = 0.0;
		GLO1_detectNum[ch] = 0;
		GLO1_chan[ch].FLLIndex = 1;
		GLO1_chan[ch].FLLFlag = 1;
		GLO1_chan[ch].ch_time = 0;
		GLO1_chan[ch].freqErr = 0.0;
		GLO1_chan[ch].fc = doppler;
	}
	else if (flag==GLO1_UNDETECTED)
	{
		GLO1_svStruct[GLO1_chan[ch].prn].NumToTryAcq--;
		{
		GLO1_chan[ch].state = GLO1_CHN_OFF;
		GLO1_correlator[ch].state = GLO1_CHN_OFF;
		GLO1_correlator[ch].ready = 1;
		GLO1_svStruct[GLO1_chan[ch].prn].state = GLO1_HOLD;
		GLO1_svStruct[GLO1_chan[ch].prn].undetectedCounter = 0;
		GLO1_detectNum[ch] = 0; 
		GLO1_chan[ch].prn = 0;
		}
	}
}



void GLO1_ch_pull_in(char ch)
{
	double sER, sEI, sLR, sLI, sPR, sPI;
	double absSE, absSL;
	double codeErr, T, phaseErr, freqErr;
	double *dllS0, *dllS1;
	double *cLoopS0, *cLoopS1;
	double cross, dot, phaseDiff;
	
	GLO1_chan[ch].ch_time++;//��Ϊ��������ж���1����һ�Σ�����һ���������һ��
	// ���tau�ǳ��ӽ���1023,���һ��IQ����ֵ�ܿ���ֻ�����˺��ټ������㣬���
	// ������ܴ󣬲����á�    ��Ϊ������������ж�tau�ĸ�32�Ƿ�216888����tau�ǲ��񵽵Ĵӳ�ʼ����λ��ʼ�ۼӵģ����Կ��ܽӽ�1023����������-216888�������´ν�����ӽ�1023
	if(GLO1_chan[ch].ch_time==1) 
	{
		return;
	}

	sER = GLO1_chan[ch].i_early;
	sEI = GLO1_chan[ch].q_early;
	sLR = GLO1_chan[ch].i_late;
	sLI = GLO1_chan[ch].q_late;
	sPR = GLO1_chan[ch].i_prompt;
	sPI = GLO1_chan[ch].q_prompt;

	// below added by Ning Luo GLO1_in Sept/04
	if (GLO1_chan[ch].ch_time==2)
	{
		GLO1_chan[ch].freqErr = 0;
		GLO1_chan[ch].i_old = GLO1_chan[ch].i_prompt;
		GLO1_chan[ch].q_old = GLO1_chan[ch].q_prompt;
		GLO1_minFreqErr[ch] = GLO1_M_PI;
		GLO1_maxFreqErr[ch] = -GLO1_M_PI;
		return;
	}

    if (GLO1_chan[ch].ch_time<GLO1_FINE_FREQ_RESOLUTION+2)
	{
        cross = GLO1_chan[ch].i_old*sPI-GLO1_chan[ch].q_old*sPR;
		dot =   GLO1_chan[ch].i_old*sPR+GLO1_chan[ch].q_old*sPI;
		phaseDiff = atan2(cross,dot);

		if(phaseDiff>GLO1_maxFreqErr[ch]) GLO1_maxFreqErr[ch] = phaseDiff;
		if(phaseDiff<GLO1_minFreqErr[ch]) GLO1_minFreqErr[ch] = phaseDiff;
		GLO1_chan[ch].freqErr += phaseDiff;
		GLO1_chan[ch].i_old = GLO1_chan[ch].i_prompt;
		GLO1_chan[ch].q_old = GLO1_chan[ch].q_prompt;
	}
	else if (GLO1_chan[ch].ch_time==GLO1_FINE_FREQ_RESOLUTION+2)
	{

        T = 1e-3;
		freqErr = (GLO1_chan[ch].freqErr-GLO1_maxFreqErr[ch]-GLO1_minFreqErr[ch])/(GLO1_FINE_FREQ_RESOLUTION-3.0);
		freqErr = freqErr/(2*GLO1_M_PI*T);
		GLO1_chan[ch].fc += freqErr;
		GLO1_chan[ch].cLoopS1 = GLO1_chan[ch].fc;
		//GLO1_chan[ch].carrier_freq = GLO1_carrier_ref + GLO1_chan[ch].fc;
		GLO1_chan[ch].carrier_freq = GLO1_chan[ch].IF+ GLO1_chan[ch].fc;//todo:GLO1_carrier_ref��ΪGLO1_chan[ch].IF
		GLO1_writeCarrierFreq(ch,GLO1_chan[ch].carrier_freq);
		
	}
	// above added by Ning Luo GLO1_in Sept/04

	else if (GLO1_chan[ch].ch_time>GLO1_FINE_FREQ_RESOLUTION+2)
	{
		/////////////////////////////////////////////////////////////////////////////////
		T = 1e-3;			

		//codeErr = (sER*sPR+sEI*sPI)/(sER*sER+sPR*sPR+sEI*sEI+sPI*sPI)*(GLO1_DLLdT/(2.0*1.023e6));
		codeErr = (sER*sPR+sEI*sPI)/(sER*sER+sPR*sPR+sEI*sEI+sPI*sPI)*(GLO1_DLLdT/(2.0*0.511e6)); //todo
		/************************************************************************/
		dllS0 = &(GLO1_chan[ch].dllS0);
		dllS1 = &(GLO1_chan[ch].dllS1);
		*dllS0 += GLO1_DLLc1*T*codeErr;
        //*dllS1 = *dllS0 + GLO1_DLLc2*codeErr+GLO1_chan[ch].fc/1575.42e6;
		//*dllS1 = *dllS0 + GLO1_DLLc2*codeErr+GLO1_chan[ch].fc/(1602.0e6+1*0.5625e6);//todo
		//*dllS1 = *dllS0 + GLO1_DLLc2*codeErr+GLO1_chan[ch].fc/(1602.0e6-7*0.5625e6);
		*dllS1 = *dllS0 + GLO1_DLLc2*codeErr + GLO1_chan[ch].fc / (1602.0e6 + GLO1_chan[ch].K * 0.5625e6);//todo
		GLO1_chan[ch].code_freq = (1+(*dllS1));
		GLO1_writeCodeFreq(ch,GLO1_chan[ch].code_freq);
		/////////////////////////////////////////////////////////////////////////////////
		
		if (GLO1_chan[ch].ch_time>GLO1_PULL_IN_TIME)	// FLL������ʱ�䣬��λ��ms
		{
			freqErr = 0.0;
		}
		else
		{
			GLO1_chan[ch].FLLIndex = 1-GLO1_chan[ch].FLLIndex;
			//����GLO1_ch_acq����1��Ȼ���������0������һ��1-0�ֱ�1��Ȼ�����´��ֱ�0�����������ifÿ��һ�ν�һ��
		
			if (GLO1_chan[ch].FLLFlag ==1  && GLO1_chan[ch].FLLIndex ==1)	
			{
				cross = GLO1_chan[ch].i_old*sPI-GLO1_chan[ch].q_old*sPR;
				dot =   GLO1_chan[ch].i_old*sPR+GLO1_chan[ch].q_old*sPI;
				if((cross*cross+dot*dot)<1.0)
					phaseDiff = 0.0;
				else
					phaseDiff = dot>=0.0 ? cross/sqrt(dot*dot+cross*cross):-cross/sqrt(dot*dot+cross*cross);
				freqErr = phaseDiff/(GLO1_twoPI*T);
				GLO1_chan[ch].freqErr = freqErr;
			}
			else
			{
			
				freqErr = 0; 
			}
			GLO1_chan[ch].i_old = GLO1_chan[ch].i_prompt;
			GLO1_chan[ch].q_old = GLO1_chan[ch].q_prompt;
		}
		if (fabs(sPR)<1e-3)
		{
			phaseErr = 0.0;
		}
		else
			phaseErr = atan(sPI/sPR);

		cLoopS0 = &(GLO1_chan[ch].cLoopS0);
		cLoopS1 = &(GLO1_chan[ch].cLoopS1);

		*cLoopS0 = *cLoopS0+(phaseErr*GLO1_PLLc0+freqErr*GLO1_FLLa1)*T;
		*cLoopS1 = *cLoopS1+(GLO1_PLLc1*phaseErr+(*cLoopS0)+GLO1_FLLa2*freqErr)*T;
		GLO1_chan[ch].fc = *cLoopS1+GLO1_PLLc2*phaseErr;
		//GLO1_chan[ch].carrier_freq = GLO1_carrier_ref + GLO1_chan[ch].fc;
		GLO1_chan[ch].carrier_freq = GLO1_chan[ch].IF + GLO1_chan[ch].fc;//todo:GLO1_carrier_ref ��ΪGLO1_chan[ch].IF
		GLO1_writeCarrierFreq(ch,GLO1_chan[ch].carrier_freq);
    

		if (GLO1_chan[ch].ch_time>GLO1_PULL_IN_TIME)//1s
		{
			GLO1_bitSync(ch);
			if(GLO1_chan[ch].state==GLO1_track)
				return;
		}
	}
}


void GLO1_bitSync(char ch)
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

	index = &(GLO1_chan[ch].BitSyncIndex);
	GLO1_chan[ch].IBuf[*index] = GLO1_chan[ch].i_prompt;
	GLO1_chan[ch].QBuf[*index] = GLO1_chan[ch].q_prompt;

	oldSign = GLO1_chan[ch].signBuf[*index];

	tempIndex = (*index+GLO1_BUFSIZE-1)%GLO1_BUFSIZE;
	realP = (double)GLO1_chan[ch].i_prompt*(double)GLO1_chan[ch].IBuf[tempIndex]
		+ (double)GLO1_chan[ch].q_prompt*(double)GLO1_chan[ch].QBuf[tempIndex];
	imagP = (double)GLO1_chan[ch].i_prompt*(double)GLO1_chan[ch].QBuf[tempIndex]
		- (double)GLO1_chan[ch].q_prompt*(double)GLO1_chan[ch].IBuf[tempIndex];
	absAngle = fabs(atan2(imagP, realP))-3.1415926/2;
	newSign = absAngle>=0 ? 1 : 0;
	GLO1_chan[ch].signBuf[*index] = newSign;
	//GLO1_chan[ch].kCell[*index%20] += (newSign-oldSign);
	/*if(ch == 1){
		printf("ch : %d index : %d  ,%d  ,%d\n",ch,*index,newSign,oldSign);
	}*/
	GLO1_chan[ch].kCell[*index%10] += (newSign-oldSign); //todo
	*index = (*index+1)%GLO1_BUFSIZE;
	GLO1_chan[ch].IQBufSize++;

	if(GLO1_chan[ch].IQBufSize<GLO1_BUFSIZE) return;
	GLO1_chan[ch].IQBufSize = GLO1_BUFSIZE;

	counter1 = 0;
	counter2 = 0;

	//for (i=0; i<20; i++)
	for (i=0; i<10; i++)  //todo
	{
		//printf("%d : %d \n",i,GLO1_chan[ch].kCell[i]);

		if (GLO1_chan[ch].kCell[i]>=GLO1_NBS2)
		{
			if (GLO1_chan[ch].kCell[i]>=GLO1_NBS1)
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

	if (counter2>=2)
		return;

	if (counter1!=1)
		return;
	//GLO1_chan[ch].ms_count = (*index-1+GLO1_BUFSIZE-signIndex)%20;
	GLO1_chan[ch].ms_count = (*index-1+GLO1_BUFSIZE-signIndex)%10; //todo
	
	//if (GLO1_chan[ch].ms_count!=19) return;
	if (GLO1_chan[ch].ms_count!=9) return; //todo

	GLO1_writeEpoch(ch,GLO1_chan[ch].ms_count);
	
	counter = GLO1_BUFSIZE;
	GLO1_chan[ch].t_count=0;
	GLO1_chan[ch].subFrameSyncFlag = GLO1_SUBFRAME_SYNCHRONIZING;
	GLO1_chan[ch].WBP = GLO1_chan[ch].NP = 0.0;


	while (counter>0)
	{
		GLO1_chan[ch].i_prom_20 = 0;
		GLO1_chan[ch].q_prom_20 = 0;
		do
		{
			sPR = GLO1_chan[ch].IBuf[*index];
			sPI = GLO1_chan[ch].QBuf[*index];
			GLO1_chan[ch].i_prom_20 += sPR;
			GLO1_chan[ch].q_prom_20 += sPI;

			GLO1_chan[ch].WBP += (double)sPR*sPR+(double)sPI*sPI;

			(*index)++;
			*index %= GLO1_BUFSIZE;
			counter--;
		}// while((*index)%20 != signIndex && counter>0);
		while((*index)%10 != signIndex && counter>0); //todo

		//if (counter == 0 && (*index)%20 != signIndex)
		if (counter == 0 && (*index)%10 != signIndex)

		{
			
			break;
		}
		else
		{
			sPI20 = GLO1_chan[ch].q_prom_20;
			sPR20 = GLO1_chan[ch].i_prom_20;
			GLO1_chan[ch].NBP = sPR20*sPR20 + sPI20*sPI20;
			//GLO1_chan[ch].NP += (GLO1_chan[ch].NBP/GLO1_chan[ch].WBP/(GLO1_BUFSIZE/20));
			GLO1_chan[ch].NP += (GLO1_chan[ch].NBP/GLO1_chan[ch].WBP/(GLO1_BUFSIZE/10));
			GLO1_chan[ch].WBP = 0.0;

			absAngle = fabs(atan2(sPI20, sPR20));
			//GLO1_chan[ch].bit = absAngle> 3.1415926/2 ? 0 : 1;
			GLO1_chan[ch].bit = absAngle> GLO1_halfPi ? 0 : 1;
			//printf("λͬ������GLO1_pream\n");
			GLO1_pream(ch,GLO1_chan[ch].bit);
			GLO1_chan[ch].message[GLO1_chan[ch].t_count++] = GLO1_chan[ch].bit;

		}
	}

   if(GLO1_chan[ch].NP-1.0<0)
      GLO1_chan[ch].CNo=0.0;
   else
	  // GLO1_chan[ch].CNo = 10 * log10(1000.0*(GLO1_chan[ch].NP-1.0)/(20-GLO1_chan[ch].NP));
       GLO1_chan[ch].CNo = 10 * log10(1000.0*(GLO1_chan[ch].NP-1.0)/(10-GLO1_chan[ch].NP));

	GLO1_chan[ch].BitSyncIndex = 0;
	GLO1_chan[ch].IQBufSize = 0;	
	for (i=0;i<GLO1_BUFSIZE;i++)	
	{
		GLO1_chan[ch].signBuf[i] = 0;
	}
	//for (i=0;i<20;i++)	
	for (i=0;i<10;i++)
	{
		GLO1_chan[ch].kCell[i] = 0;
	}

	GLO1_chan[ch].i_prom_20 = 0;
	GLO1_chan[ch].q_prom_20 = 0;
	GLO1_chan[ch].i_early_20 = 0;
	GLO1_chan[ch].q_early_20 = 0;
	GLO1_chan[ch].i_late_20 = 0;
	GLO1_chan[ch].q_late_20 = 0;
	GLO1_chan[ch].WBP = GLO1_chan[ch].NBP = GLO1_chan[ch].NP = GLO1_chan[ch].NBD = 0.0;

	GLO1_chan[ch].trackTime = 0;
	GLO1_chan[ch].state = GLO1_track;
}
	

/*******************************************************************************
FUNCTION GLO1_ch_track(char ch)
RETURNS  None.

PARAMETERS
			ch  char  channel number

PURPOSE  to GLO1_track GLO1_in carrier and code the GPS satellite and partially
			decode the navigation message (to determine TOW, subframe etc.)

*******************************************************************************/
void GLO1_ch_track(char ch)
{
	double sER, sEI, sLR, sLI, sPR, sPI;
	double sPR20, sPI20, sER20, sEI20, sLR20, sLI20;
	double absSE, absSL;
	double codeErr, T, phaseErr;
	double *dllS0, *dllS1;
	double *cLoopS0, *cLoopS1;
	double absAngle;
    GLO1_CHANNEL* pChan=&GLO1_chan[ch];

	GLO1_chan[ch].trackTime++;

	sER = pChan->i_early;
	sEI = pChan->q_early;
	sLR = pChan->i_late;
	sLI = pChan->q_late;
	sPR = pChan->i_prompt;
	sPI = pChan->q_prompt;
	
	T = 1e-3;			// ������ƻ�·��������1KHz,��ÿ��һ��IQ����ֵ
						// ��·���Ʋ�����Ҫ���¼���һ�Σ��ⲻ��һ�ֱ����
						// ѡ�񣬵���pull-GLO1_in״̬�£��ߵĸ������������ڻ�·��
						// ��������		

	// PLL discriminator
    if (fabs(sPR)<1e-3)
    {
        phaseErr = 0.0;
    }
    else
        phaseErr = atan(sPI/sPR);

	////�����ز���λ���
	//char str1[10];
	////itoa(sv, str, 10);
	//sprintf(str1, "%d", GLO1_chan[ch].prn);
	//char s1[100] = "PLL_phase_error_";
	//char *pfilename = strcat(s1, str1);

	//FILE *fd1 = fopen(pfilename, "a");
	//fprintf(fd1, "%10.9f\n", phaseErr);
	//fclose(fd1);

	// update carrier loop
	cLoopS0 = &(pChan->cLoopS0);
	cLoopS1 = &(pChan->cLoopS1);

	*cLoopS0 = *cLoopS0+phaseErr*GLO1_PLLc0*T;	// 3rd order PLL
	*cLoopS1 = *cLoopS1+(GLO1_PLLc1*phaseErr+(*cLoopS0))*T;
	pChan->fc = *cLoopS1+GLO1_PLLc2*phaseErr;
	//pChan->carrier_freq = GLO1_carrier_ref + pChan->fc;
	pChan->carrier_freq = GLO1_chan[ch].IF + pChan->fc;//GLO1_carrier_ref��ΪGLO1_chan[ch].IF
	GLO1_writeCarrierFreq(ch,pChan->carrier_freq);

	// ms����(ģ20��1bit��ʱ��)  ��һ�θ��پ��Լ�һ��
	//pChan->ms_count=(++pChan->ms_count)%20;
	pChan->ms_count = (++pChan->ms_count) % 10; //todo
	// long  ms_count; // ���1bitʱ�õ�ms������(ģ20)������������IQֵ�ۼ��ٽ��
	// bit�ڵĻ���ֵ�ۼ�

	pChan->i_prom_20 += pChan->i_prompt;
	pChan->q_prom_20 += pChan->q_prompt;
	pChan->i_early_20 += pChan->i_early;
	pChan->q_early_20 += pChan->q_early;
	pChan->i_late_20 += pChan->i_late;
	pChan->q_late_20 += pChan->q_late;

	/************************************************************************/
	/*��GLO1_bitSync��ѭ���ۼ�����������ȣ�������������� long  q_prom_20,i_prom_20; // 20msIQͨ��ֵ�ĺ�
	GLO1_chan[ch].i_prom_20 += sPR;
	GLO1_chan[ch].q_prom_20 += sPI;
	*/
	/************************************************************************/
	
	// ��������ȵ��м䲽��
	pChan->WBP += sPR*sPR+sPI*sPI;

	//if(pChan->ms_count==19)
	if(pChan->ms_count==9)
	{
		
		//T = 0.02;	
		T = 0.01;	//�뻷���»�·ʱ��
		// update DLL
		sPI20 = pChan->q_prom_20;
		sPR20 = pChan->i_prom_20;
		sLI20 = pChan->q_late_20;
		sLR20 = pChan->i_late_20;
		sEI20 = pChan->q_early_20;
		sER20 = pChan->i_early_20;

		//codeErr = (sER20*sPR20+sEI20*sPI20)/(sER20*sER20+sPR20*sPR20+sEI20*sEI20+sPI20*sPI20)*(GLO1_DLLdT/(2.0*1.023e6));
		codeErr = (sER20*sPR20+sEI20*sPI20)/(sER20*sER20+sPR20*sPR20+sEI20*sEI20+sPI20*sPI20)*(GLO1_DLLdT/(2.0*0.511e6));

		////����α����λ���
		//char str[10];
		////itoa(sv, str, 10);
		//sprintf(str, "%d", GLO1_chan[ch].prn);
		//char s[100] = "code_error_";
		//char *pfilename = strcat(s, str);

		//FILE *fd = fopen(pfilename, "a");
		//fprintf(fd, "%10.9f\n", codeErr);
		//fclose(fd);



		dllS0 = &(pChan->dllS0);
		dllS1 = &(pChan->dllS1);
		*dllS0 += GLO1_DLLc1*T*codeErr;
		//*dllS1 = *dllS0 + GLO1_DLLc2*codeErr + pChan->fc/1575.42e6;
		//*dllS1 = *dllS0 + GLO1_DLLc2*codeErr + pChan->fc/(1602.0e6-7*0.5625e6);//todo:��Ҫ�޸�
		//*dllS1 = *dllS0 + GLO1_DLLc2*codeErr + pChan->fc / (1602.0e6 +1 * 0.5625e6);
		*dllS1 = *dllS0 + GLO1_DLLc2*codeErr + pChan->fc / (1602.0e6 + GLO1_chan[ch].K* 0.5625e6);//todo
        pChan->code_freq = 1+(*dllS1);
		GLO1_writeCodeFreq(ch,pChan->code_freq);


		pChan->NBP = sPI20*sPI20 + sPR20*sPR20;
		pChan->NP += (0.02 * pChan->NBP/pChan->WBP);//0.02��1/K��KΪ50

		pChan->WBP = 0.0;

		// �����ز���λ������ָʾ
		pChan->NBD = sPR20*sPR20 - sPI20*sPI20;
		pChan->phaseLockDetector = pChan->NBD/pChan->NBP;

		//if (pChan->trackTime%1000==0 && pChan->trackTime>0)
		if (pChan->trackTime%500==0 && pChan->trackTime>0)
		{
			// ���������
         if(GLO1_chan[ch].NP-1.0<0)
            GLO1_chan[ch].CNo=0.0;
         else
			 //  pChan->CNo = 10 * log10(1000.0*(pChan->NP-1.0)/(20.0-pChan->NP));
		    pChan->CNo = 10 * log10(1000.0*(pChan->NP-1.0)/(10.0-pChan->NP));
			pChan->NP = 0.0;
         if(pChan->CNo<30) 
         {
            pChan->state=GLO1_acquisition;
            pChan->phaseLockDetector=0.0;
            pChan->az=0;
            pChan->el=0;
            GLO1_svStruct[pChan->prn].NumToTryAcq = 20;//��Ϊ�Ѿ�������ǣ���ŵ����⣬ֻ������ȱȽϵͣ������Ǵ��ڣ�������ȥ��20��
            GLO1_svStruct[pChan->prn].NumBTWEachTrial = 1000;//�ز��������Ϊ1000��
            GLO1_svStruct[pChan->prn].maxf=(pChan->fc+GLO1_DELTF);//�����Ѿ���õ���Ϣ�����Բ���Χ��С���Ѿ�������Ƶ��+-һ�����
            GLO1_svStruct[pChan->prn].minf=(pChan->fc-GLO1_DELTF);
            GLO1_correlator[ch].state = GLO1_CHN_OFF;//���²��񣬲�����Ӧͨ���������
            GLO1_correlator[ch].ready = 1;//�����̫�ͣ�����Ϊ��Ҫ���²���������Ϊ1�����Խ���sim_GPS_Interrupt��case��
         }
		}
			
		// 1bitʱ�䵽�����Խ��������
		pChan->tr_bit_time++;//��ǰbit����1
		absAngle = fabs(atan2((double)pChan->q_prom_20, (double)pChan->i_prom_20));
		//pChan->bit = absAngle>3.1415926/2 ? 0 : 1;
		pChan->bit = absAngle>GLO1_halfPi ? 0 : 1;

		GLO1_pream(ch,pChan->bit);  // ÿ2�������һ��see if we can find the preamble

		 // ����������
		pChan->message[pChan->t_count++]=pChan->bit;

		pChan->i_prom_20 = 0;
		pChan->q_prom_20 = 0;
		pChan->i_early_20 = 0;
		pChan->q_early_20 = 0;
		pChan->i_late_20 = 0;
		pChan->q_late_20 = 0;
	}


	if (pChan->t_count==3000)//һ����֡����5����֡��һ����֡����1500bit// t_count�յ��ı��ؼ���������ѭ����ʽ����0-1499
	{
		pChan->n_frame++;//��֡��5����֡��
		pChan->t_count=0;
	}

}

/*******************************************************************************
FUNCTION GLO1_xors(long pattern)
RETURNS  Integer

PARAMETERS
			pattern  long 32 bit GLO1_data

PURPOSE
			count the number of bits set GLO1_in "pattern"

*******************************************************************************/

int GLO1_xors(long pattern)
{
	int count,i;
	count=0;
	pattern=pattern>>6;
	for (i=0;i<=25;i++)
	{
		count+=int(pattern & 0x1);
		pattern=pattern>>1;
	}
	count=count%2;
	return(count);
}

/*******************************************************************************
FUNCTION GLO1_sign(long GLO1_data)
RETURNS  Integer

PARAMETERS
		GLO1_data Long

PURPOSE
		This function returns
				  +1 when positive
			       0 when zero
			      -1 when negative
*******************************************************************************/
inline int GLO1_sign(long GLO1_data)
{
	int result;

	if      ( GLO1_data  > 0 ) result= 1;
	else if ( GLO1_data == 0 ) result= 0;
	else if ( GLO1_data  < 0 ) result=-1;
	return (result);
}


/*******************************************************************************
FUNCTION GLO1_bit_test()
RETURNS  None.

PARAMETERS None.

PURPOSE
	  Determine if a bit GLO1_in the GLO1_data word has been set
*******************************************************************************/
inline int  GLO1_bit_test(int GLO1_data,char bit_n)
{
	return(GLO1_data & GLO1_test[bit_n]);
}

/*******************************************************************************
FUNCTION GLO1_read_rcvr_par(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  To read GLO1_in from the rcvr_par file the GLO1_receiver parameters that control
			GLO1_acquisition, tracking etc.


*******************************************************************************/
char GLO1_rcvr_par_file[4096];
void GLO1_read_rcvr_par(void)
{
	 	char intext[40];
	 
	  	if ((GLO1_in = fopen(GLO1_rcvr_par_file, "rt")) == NULL)
	 	{
	 		printf("Cannot open rcvr_par.dat file.\n");
	 		exit(0);
	 	}
	 	else
	 	{
	 		fscanf(GLO1_in,"%s %s",intext,GLO1_tzstr);
	 		fscanf(GLO1_in,"%s %lf",intext,&GLO1_mask_angle);
	 		GLO1_mask_angle /= GLO1_r_to_d;
	 		fscanf(GLO1_in,"%s %lf",intext,&GLO1_clock_offset);
	 		fscanf(GLO1_in,"%s %d",intext,&GLO1_interr_int);
	 		fscanf(GLO1_in,"%s %d",intext,&GLO1_ICP_CTL);		
	 		fscanf(GLO1_in,"%s %lf",intext,&GLO1_nav_up);	
	 		fscanf(GLO1_in,"%s %d",intext,&GLO1_out_pos);
	 		fscanf(GLO1_in,"%s %d",intext,&GLO1_out_vel);
	 		fscanf(GLO1_in,"%s %d",intext,&GLO1_out_time);
	 		fscanf(GLO1_in,"%s %d",intext,&GLO1_out_debug);
	 		fscanf(GLO1_in,"%s %d",intext,&GLO1_m_tropo);
	 		fscanf(GLO1_in,"%s %d",intext,&GLO1_m_iono);
	 		fscanf(GLO1_in,"%s %d",intext,&GLO1_align_t);	
	 	}
	 	fclose(GLO1_in);
}

/*******************************************************************************
FUNCTION GLO1_nav_fix()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function determines the pseudorange and doppler to each
	satellite and calls GLO1_pos_vel_time to determine the position and
	GLO1_velocity of the GLO1_receiver

*******************************************************************************/
#define GLO1_DAICYCOR
double GLO1_wgs[3];
int	GLO1_pos_out_flag = 0;
int GLO1_judge_nits = 0;
double	GLO1_t_cor[13];
short	GLO1_towFlag = 0;
void  GLO1_nav_fix(void)
{
	//st-atic int judge_nits = 0;
	char			ch,n,bit;
	double			tr_time[13],tr_avg,ipart,clock_error;
	//stat-ic double	t_cor[13];
	int				i,ms;
	double			chip;
    int				meas_bit_time_rem;
    long			meas_bit_time_offset;
	double			tic_CNTR;
	GLO1_XYZ				rp_ecef;
	GLO1_ECEFT			dm_gps_sat[13],dp_gps_sat[13];
	double			clock_error_in_Hz;
	//sta-tic short	towFlag=0;
	double             const_pram=0.0;
	double          eve_K=0;


	tr_avg=0.0;
	n=1;
	for (ch=0;ch<=GLO1_chmax;ch++)
	{
		if (GLO1_chan[ch].state != GLO1_track)  
			continue;
		//if (ch == 2)
		//	continue;
		meas_bit_time_offset=0;
		ms=GLO1_chan[ch].epoch & 0x1f;//0x1f  	// the last five bits for 0 ~ 19 ms counter
		bit=GLO1_chan[ch].epoch>>8;	// counter of bit (0 - 50) һ��1000ms��20msһ���أ�һ�����һ�Σ���50����

		chip = GLO1_chan[ch].codePhase;
		//��sim_gps_interrupt�����1�뵽����GLO1_chan[ch].codePhase = GLO1_readCodePhase(ch);//return (double)GLO1_correlator[ch].tauLatch/(1.023e6*GLO1_d_2p40);
		//��Ϊ��NCO��40λ�ģ����λ���������Ƭ������2^40����Ϊ�൱��������40λ������λ��0~1023�ģ�����1023000���൱��0~1���룬1������һ��������

		GLO1_chan[ch].int_carr_phase=GLO1_chan[ch].carrier_counter + GLO1_chan[ch].d_carr_phase;

		GLO1_chan[ch].carrier_output+=(GLO1_chan[ch].int_carr_phase-GLO1_CARRIER_FREQ);

		if (GLO1_out_debug)
		{
			fprintf(GLO1_debug," ch= %d PRN =%d bit time= %ld  bit= %d  ms= %d chip= %f",
				ch,GLO1_chan[ch].prn,GLO1_chan[ch].meas_bit_time,bit,ms,chip);
			if (GLO1_ICP_CTL==0) 
				fprintf(GLO1_debug,"CTL = %lf\n",GLO1_chan[ch].doppler);
			else            
				fprintf(GLO1_debug,"ICP = %lf\n",GLO1_chan[ch].int_carr_phase);
		}
		////����α��
		//char strz[10];
		////itoa(sv, str, 10);
		//sprintf(strz, "%d", ch);
		//char sz[100] = "meas_bit_";
		//char *pfilenamez = strcat(sz, strz);

		//FILE *fsz = fopen(pfilenamez, "a");
		//fprintf(fsz, " %d %d \n", GLO1_chan[ch].meas_bit_time % 100, bit);
		//fclose(fsz);
		//meas_bit_time_rem = GLO1_chan[ch].meas_bit_time%50;	
		meas_bit_time_rem = GLO1_chan[ch].meas_bit_time % 100;//todo
		if ( meas_bit_time_rem == bit+1 ) meas_bit_time_offset = -1;
		if ( meas_bit_time_rem == bit-1 ) meas_bit_time_offset = +1;
		if ( meas_bit_time_rem == 0 && bit == 99 ) meas_bit_time_offset = -1;
		if ( meas_bit_time_rem == 99 && bit == 0 ) meas_bit_time_offset = +1;
		if ((GLO1_chan[ch].meas_bit_time+meas_bit_time_offset)%100==bit&&
			GLO1_chan[ch].state==GLO1_track && GLO1_chan[ch].CNo>30 &&
			GLO1_glonass_sv_id_ephemeris[GLO1_chan[ch].K + 10].valid==1&&//GLO1_gps_eph[GLO1_chan[ch].prn].valid==1 &&
			GLO1_glonass_sv_id_ephemeris[GLO1_chan[ch].K + 10].Bn==0&&//GLO1_gps_eph[GLO1_chan[ch].prn].health==0 &&
			GLO1_chan[ch].tow_sync==1 && chip<=1e-3&&GLO1_unpack_glonass_flag[ch]==1&&
			GLO1_string_1[ch]==true
			/*&& GLO1_glonass_sv_id_ephemeris[GLO1_chan[ch].K + 10].Xn>10*/)
		{
			tr_time[n] = (GLO1_chan[ch].meas_bit_time + meas_bit_time_offset)*0.01 +
				ms / 1000.0 + chip + GLO1_SAMPLING_INT-0.009;//-0.019-0.001
			 //ms/1000.0��С�����ؼƵ�ʱ�䲿�֣�����1����	
			 //chip��chip = GLO1_chan[ch].codePhase;  ���ڵ�����λ����ɵ�ʱ�䣬����1/1023����
			 //��sim_gps_interrupt�����1�뵽����GLO1_chan[ch].codePhase = GLO1_readCodePhase(ch);//return (double)GLO1_correlator[ch].tauLatch/(1.023e6*GLO1_d_2p40);
			 //��Ϊ��NCO��40λ�ģ����λ���������Ƭ������2^40����Ϊ�൱��������40λ������λ��0~1023�ģ�����1023000���൱��0~1���룬1������һ��������

			////����α��
			//char str[10];
			////itoa(sv, str, 10);
			//sprintf(str, "%d", 1);
			//char s[100] = "cc_";
			//char *pfilename = strcat(s, str);

			//FILE *fs = fopen(pfilename, "a");
			//fprintf(fs, " %d %d %10.9f %d\n", GLO1_chan[2].meas_bit_time, GLO1_chan[2].epoch & 0x1f, GLO1_chan[2].codePhase,(GLO1_chan[0].tr_bit_time)/100);
			//fclose(fs);

			GLO1_tr_ch[n]=ch;
			tr_avg+=tr_time[n];
			n++;
		}
	}

	GLO1_n_track=n-1;
	
	GLO1_TIC_dt=GLO1_i_TIC_dt*GLO1_SAMPLING_INT;
	// GLO1_SAMPLING_INT��һ��������Ҫ����ʱ�䣬����1000����ĵ������õ�1000����ĵ�ȼ۵�ʱ��
	if (GLO1_out_debug) fprintf(GLO1_debug,"n_track= %d\n",GLO1_n_track);

	// ��ʱ���ں���ʽ�ı���GPSʱ���ʼ��������һ�����ԵĽ��ջ��Ľ���ʱ��
	if (GLO1_towFlag == 0)
	{
      if (GLO1_n_track>0)
      {
         GLO1_m_time[1] = tr_time[1] + 0.063;//+0.075
		 GLO1_towFlag = 1;
      }
      else if (GLO1_bLocalTimeSetByTow)
      {
         GLO1_m_time[1]+=GLO1_nav_up;
      }
      else if (GLO1_bTowDecoded)//��GLO1_pream��if (sfid_s==1)GLO1_bTowDecoded=true;
      {
         GLO1_m_time[1]=GLO1_clock_tow;
         GLO1_bLocalTimeSetByTow=true;
      }
      else
         GLO1_m_time[1]+=GLO1_nav_up;//����ֻҪ����һ�Σ��Ҳ��Ǹ��٣��ͼ�һ��
	}
	else if (GLO1_towFlag == 1)
      GLO1_m_time[1]+=GLO1_nav_up;

	////����α��
	//char str[10];
	////itoa(sv, str, 10);
	//sprintf(str, "%d", 0);
	//char s[100] = "cc_";
	//char *pfilename = strcat(s, str);

	//FILE *fs= fopen(pfilename, "a");
	//fprintf(fs, " %10.9f %10.9f %10.9f\n", tr_time[2], GLO1_m_time[1], (-tr_time[2]+ GLO1_m_time[1])*GLO1_c);
	//fclose(fs);
	for (i=1;i<=GLO1_n_track;i++)
	{
		GLO1_track_sat[i] = GLO1_glonass_process_sat_pvt_L(GLO1_chan[GLO1_tr_ch[i]].K,&GLO1_glonass_sv_id_ephemeris[GLO1_chan[GLO1_tr_ch[i]].K+10],
			&GLO1_glonass_sv_id_almanac_str5[GLO1_chan[GLO1_tr_ch[i]].K+10],
			tr_time[i]);
		

		//GLO1_track_sat[i]=GLO1_satpos_ephemeris(tr_time[i],GLO1_chan[GLO1_tr_ch[i]].prn);
		GLO1_chan[GLO1_tr_ch[i]].az = GLO1_track_sat[i].az;
		GLO1_chan[GLO1_tr_ch[i]].el = GLO1_track_sat[i].el;
		
		if(GLO1_ICP_CTL==0)   //������ satellite GLO1_velocity  
		{
			dm_gps_sat[i]=GLO1_satpos_ephemeris(tr_time[i]-GLO1_TIC_dt/2.0,GLO1_chan[GLO1_tr_ch[i]].prn);
			dp_gps_sat[i]=GLO1_satpos_ephemeris(tr_time[i]+GLO1_TIC_dt/2.0,GLO1_chan[GLO1_tr_ch[i]].prn);
			/************************************************************************/
			/*
			// GLO1_i_TIC_dt�����μ���λ�õļ������1000����ĵ���
			GLO1_TIC_dt=GLO1_i_TIC_dt*GLO1_SAMPLING_INT;		//each clock count is 175 ns for GP2021
			// GLO1_SAMPLING_INT��һ��������Ҫ����ʱ�䣬����1000����ĵ������õ�1000����ĵ�ȼ۵�ʱ��
			*/
			/**********************************/
		}
		else//�����ز���λ  ���ֵ����ٶȵ�Ч����ǰһ�ֺ�  �����������Ҳ���ز�����Ч����
		{
		    dm_gps_sat[i]= GLO1_glonass_process_sat_pvt_L(GLO1_chan[GLO1_tr_ch[i]].K, &GLO1_glonass_sv_id_ephemeris[GLO1_chan[GLO1_tr_ch[i]].K + 10],
				&GLO1_glonass_sv_id_almanac_str5[GLO1_chan[GLO1_tr_ch[i]].K + 10],
				tr_time[i] - GLO1_TIC_dt);
				//GLO1_satpos_ephemeris(tr_time[i]-GLO1_TIC_dt,GLO1_chan[GLO1_tr_ch[i]].prn); 
		    dp_gps_sat[i]=GLO1_track_sat[i];

		}
		// ����ÿ����ǵ�У����
		GLO1_t_cor[i] = -GLO1_tropo_iono(GLO1_tr_ch[i], GLO1_track_sat[i].az, GLO1_track_sat[i].el, tr_time[i]);
		// GLO1_dt�ǰ��������ջ��Ӳ��α��
		// GLO1_track_sat�����ǵ�λ��
		// ������������Ϊȫ�����飬����GLO1_pos_vel_time(*)����

		
		GLO1_dt[i] = GLO1_m_time[1] - tr_time[i] + GLO1_track_sat[i].tb; //(tr_time[i]-t_cor[i])+const_pram;//α��
		GLO1_lambda[i] = GLO1_c / (1602e6 + GLO1_chan[GLO1_tr_ch[i]].K * 562500);


		// �������ǵ��ٶȣ����ǵ�λ�ò������ת��������
		GLO1_d_sat[i].x=(dp_gps_sat[i].x-dm_gps_sat[i].x)/GLO1_TIC_dt-GLO1_track_sat[i].y*GLO1_omegae;
		GLO1_d_sat[i].y=(dp_gps_sat[i].y-dm_gps_sat[i].y)/GLO1_TIC_dt+GLO1_track_sat[i].x*GLO1_omegae;
		GLO1_d_sat[i].z=(dp_gps_sat[i].z-dm_gps_sat[i].z)/GLO1_TIC_dt;



		// process Carrier Tracking Loop or Integrated Carrier Phase
		// ���������Ǽ��������Ƶ�ʣ�����������ֶ�ֻ��GP2021оƬ����Ч
		// ���磺33010105L)*42.574746268e-3 = 1.405396826e6 = GP2021��Ƶ
		// Ϊ���ó����ͨ�ã��ظ�Ϊ��
		// if (GLO1_ICP_CTL==0) GLO1_meas_dop[i]=(GLO1_chan[GLO1_tr_ch[i]].doppler-33010105L)*42.574746268e-3;  // for CTL
		// else GLO1_meas_dop[i]= GLO1_chan[GLO1_tr_ch[i]].int_carr_phase/GLO1_TIC_dt-1.405396826e6; // for ICP
		if (GLO1_ICP_CTL==0) GLO1_meas_dop[i]=GLO1_chan[GLO1_tr_ch[i]].doppler;  
		//double		GLO1_meas_dop[13];							// ������ÿ��ͨ���Ķ�����
		else GLO1_meas_dop[i] = GLO1_chan[GLO1_tr_ch[i]].int_carr_phase / GLO1_TIC_dt -(GLO1_CARRIER_FREQ + GLO1_chan[GLO1_tr_ch[i]].K * 562500);
		//GLO1_chan[ch].int_carr_phase=GLO1_chan[ch].carrier_counter + GLO1_chan[ch].d_carr_phase;//�����ز���λ+С���ز���λ
		//�������ز�+С���ز���/������ô���ز���ʱ�� - ����ز� 
	}



	if (GLO1_n_track>=4)
	{
		switch (sel & USING_POS_METHOD_SEL)
		{
		case USING_POS_VEL_TIME:
			GLO1_rpvt = GLO1_pos_vel_time(GLO1_n_track);
			break;
		case USING_KALMAN_POS_VEL:
			if (GLO1_judge_nits == 0)
			{
				GLO1_rpvt = GLO1_pos_vel_time(GLO1_n_track);
			}
			else
			{
				GLO1_rpvt = GLO1_kalman_pos_vel(GLO1_n_track);
			}
			break;
		default:
			break;
		}
	
		GLO1_cbias=GLO1_rpvt.dt;				
		clock_error=GLO1_rpvt.df;		
		//for (int i = 1; i <= GLO1_n_track; i++)
		//{
		//	eve_K +=GLO1_chan[GLO1_tr_ch[i]].K* 0.5625;
		//}
		//clock_error_in_Hz = clock_error * (1602+ GLO1_chan[GLO1_tr_ch[i]].K* 0.5625);//1575.42;// 1575,42Mhz,��һ��1575.42M���ز����ڣ�*һ������ƫ�ƶ���
		GLO1_m_time[1]=GLO1_m_time[1]-GLO1_cbias;			// �������ջ���ʱ��
		rp_ecef.x=GLO1_rpvt.x;
		rp_ecef.y=GLO1_rpvt.y;
		rp_ecef.z=GLO1_rpvt.z;
		GLO1_rp_llh=GLO1_ecef_to_llh(rp_ecef);	// ��GLO1_xyz����ת��Ϊ��γ������

		GLO1_wgs[0] = GLO1_rpvt.x;
		GLO1_wgs[1] = GLO1_rpvt.y;
		GLO1_wgs[2] = GLO1_rpvt.z;
		////����α��
		//char str[10];
		////itoa(sv, str, 10);
		//sprintf(str, "%d", 1);
		//char s[100] = "recv_";
		//char *pfilename = strcat(s, str);

		//FILE *fs = fopen(pfilename, "a");
		//fprintf(fs, " %10.9f %10.9f %10.9f %10.9f\n", GLO1_rpvt.x, GLO1_rpvt.y, GLO1_rpvt.z, GLO1_rpvt.df);
		//fclose(fs);
		::PostMessage(NavOutWnd, WM_NAVIGATION_OUT, (WPARAM)GLO1_wgs, 1);

		// vDoppler���ɽ��ջ�����������˶������doppler
		for (ch=0;ch<=GLO1_chmax;ch++)
		{
			GLO1_chan[ch].vDoppler = 0.0;//double vDoppler; // doppler�������˶�������doppler����
		}
		for (ch=1;ch<=GLO1_n_track;ch++)
		{
			//GLO1_chan[GLO1_tr_ch[ch]].vDoppler = GLO1_meas_dop[ch]-clock_error_in_Hz;
			GLO1_chan[GLO1_tr_ch[ch]].vDoppler = GLO1_meas_dop[ch] - clock_error * (1602 + GLO1_chan[GLO1_tr_ch[ch]].K* 0.5625);

		}
		if (GLO1_rp_llh.hae>-2000.0 && GLO1_rp_llh.hae< 28000 && GLO1_pos_out_flag >=0) //modified by daicy
		{
			GLO1_velocity();		// ���ٶ�ʸ����WGS84ת����ENU����ϵ

			if (GLO1_speed < 514.0)
			{
				if (fabs(clock_error)<5.0) GLO1_clock_offset=clock_error;
				if (GLO1_almanac_valid==1) GLO1_status=GLO1_navigating;

				// �������������ָҪ�����ջ��Ĳ���ʱ�̺����Ƿ���ʱ�̶�׼
				// �Ƿ��������˼����Ҫ��һ����֤
				if (GLO1_align_t==1)//// �����ջ�ʱ���GPSʱ���׼�ı�־
				{
					// PC����ʱ��(��GPS���ʾ)��С������
					GLO1_delta_m_time= modf(GLO1_m_time[1],&ipart);//GLO1_delta_m_time����GLO1_m_time[1]��С�����֣�ipartָ��GLO1_m_time[1]����������
					//The modf function breaks down the floating-point value x into fractional and integer parts,
					// each of which has the same GLO1_sign as x. The signed fractional portion of x is returned. 
					//The integer portion is stored as a floating-point value at intptr.
					// GLO1_m_error�������GLO1_nav_upʱ�̵��������
					if (GLO1_nav_up<1.0)
					{
						GLO1_delta_m_time=modf(GLO1_delta_m_time/GLO1_nav_up,&ipart);//GLO1_delta_m_time/GLO1_nav_up��ÿ����������
						//double		GLO1_m_error; // ���ջ��Ĳ���ʱ���GPS��������
						if (GLO1_delta_m_time>0.5) GLO1_m_error=(GLO1_delta_m_time-1.0)*GLO1_nav_up;//ÿ�θ���ʱ��۲�ʱ�������GLO1_nav_upʱ�̵��������
						else GLO1_m_error=GLO1_delta_m_time*GLO1_nav_up;
					}
					else
					{
	//Ϊʲô���ﲻ��modf����Ϊ����GLO1_nav_up�Ѿ����ڵ�1�룬��������С��GLO1_delta_m_time�Ѿ��������1�������ĵ�λ��
						if (GLO1_delta_m_time>0.5) GLO1_m_error=(GLO1_delta_m_time-1.0)/GLO1_nav_up;//ÿ�θ���ʱ��۲�ʱ�������GLO1_nav_upʱ�̵��������
						else GLO1_m_error=GLO1_delta_m_time/GLO1_nav_up;
					}

#ifndef REAL_TIME
					// ���¶�GLO1_TIC_CNTR�ı��ֻ�ʺ��ں���ʵʱӦ��ʱ����Ҫ����
					// ��Ϊ�µ�GLO1_TIC_CNTR���Ǽ�ʱװ��ģ����ǵ����ڵ�TIC�������̽������װ���
					// tic_CNTRȡ��ֵ��Ҫʹ�´�GLO1_nav_up����ʱ�����ջ��Ľ���ʱ�̻������Զ�׼
					// tic_CNTR = (long)(GLO1_TIC_ref*(GLO1_nav_up-GLO1_m_error)/(1.0+GLO1_clock_offset*1.0e-6)+0.5);
					//hide by jh 4-14 tic_CNTR = GLO1_TIC_ref*(1.0-GLO1_m_error/GLO1_nav_up)/(1.0+GLO1_clock_offset*1.0e-6);
					//double GLO1_clock_offset=0.0; 	// ���ջ�����Ư����λ��ppm,��ʼֵδ֪���ٶ�Ϊ0
					tic_CNTR = GLO1_TIC_ref*(1.0-GLO1_m_error/GLO1_nav_up)/(1.0+GLO1_clock_offset*1.0e-6);
#endif
					//������ʱ����һ���Ӳ�ȥ����ÿ�ζ�ȡ�����ݳ���
					//hide by jh 4-2 GLO1_programTIC(tic_CNTR);//GLO1_TIC_CNTR = GLO1_data;long GLO1_TIC_CNTR;	 // TIC������������Ӧ��TIC�жϵķ�������
					//�����˻���������������ʱ�䣬ԭ������100����
					//ÿ��GLO1_nav_fix����������ֵ
					GLO1_programTIC(tic_CNTR);	//modifid by daicy
				}

				GLO1_rec_pos_llh.lon=GLO1_rp_llh.lon;
				GLO1_rec_pos_llh.lat=GLO1_rp_llh.lat;
				GLO1_rec_pos_llh.hae=GLO1_rp_llh.hae;
				GLO1_current_loc.lon=GLO1_rp_llh.lon;
				GLO1_current_loc.lat=GLO1_rp_llh.lat;
				GLO1_current_loc.hae=GLO1_rp_llh.hae;
				GLO1_rec_pos_xyz.x=rp_ecef.x;
				GLO1_rec_pos_xyz.y=rp_ecef.y;
				GLO1_rec_pos_xyz.z=rp_ecef.z;
				
				GLO1_dops(GLO1_n_track);     // ����DOP   
				GLO1_m_time[0]=GLO1_m_time[1];
			}
		}
		GLO1_pos_out_flag++;
		
	}
	else	// ������ǵ���������4����ֻ���ܾ������ֽ��ջ�ʱ�ӵ�׼ȷ��
	{                              
		// ����֪����Ư������ʱ�䣬ע�������ʱ�䲻�ܶ�λ�����ջ���ʱ��
		// ����Ư��
		// ע�⣺����ĵ������������治ͬ��GLO1_TIC_CNTRû�иı䣬ԭ���Ĺ�ʽ�ƺ�����
		//GLO1_m_time[1]=GLO1_m_time[1]+GLO1_TIC_dt*(1.0+GLO1_clock_offset/1.e6); 
		GLO1_m_time[1] = fmod(GLO1_m_time[1],604800);
		rp_ecef.x=0.0;
		rp_ecef.y=0.0;
		rp_ecef.z=0.0;
		GLO1_rpvt.xv=0.0;
		GLO1_rpvt.yv=0.0;
		GLO1_rpvt.zv=0.0;
	}

	if (GLO1_n_track>=4)
	{
		for (i=1;i<=GLO1_n_track;i++)
		{
			
			GLO1_chan[GLO1_tr_ch[i]].Pr=(GLO1_m_time[1]-tr_time[i])*GLO1_SPEEDOFLIGHT;

			GLO1_chan[GLO1_tr_ch[i]].dPr=GLO1_chan[GLO1_tr_ch[i]].vDoppler*GLO1_lambda[GLO1_tr_ch[i]];
		
			////����α��
			//char str[10];
			////itoa(sv, str, 10);
			//sprintf(str, "%d", GLO1_chan[GLO1_tr_ch[i]].prn);
			//char s[100] = "PRN_";
			//char *pfilename = strcat(s, str);

			//FILE *fd = fopen(pfilename, "a");
			//fprintf(fd, "TOW: % 6ld, Pr: %10.9f\n", GLO1_clock_tow,GLO1_chan[GLO1_tr_ch[i]].Pr);
			//fclose(fd);
		}
	}
}


/*******************************************************************************
FUNCTION GLO1_velocity(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  To convert GLO1_velocity from ecef to local level (WGS-84) axes

*******************************************************************************/
void GLO1_velocity(void)
{

	// x,y,z -> North
	GLO1_receiver.north.x=-cos(GLO1_rec_pos_llh.lon)*sin(GLO1_rec_pos_llh.lat);
    GLO1_receiver.north.y=-sin(GLO1_rec_pos_llh.lon)*sin(GLO1_rec_pos_llh.lat);
    GLO1_receiver.north.z= cos(GLO1_rec_pos_llh.lat);

	// x,y,z -> East
    GLO1_receiver.east.x=-sin(GLO1_rec_pos_llh.lon);
    GLO1_receiver.east.y= cos(GLO1_rec_pos_llh.lon);
	//  GLO1_receiver.east.z=0.0;

	// x,y,z -> Up
    GLO1_receiver.up.x=cos(GLO1_rec_pos_llh.lon)*cos(GLO1_rec_pos_llh.lat);
    GLO1_receiver.up.y=sin(GLO1_rec_pos_llh.lon)*cos(GLO1_rec_pos_llh.lat);
    GLO1_receiver.up.z=sin(GLO1_rec_pos_llh.lat);

	GLO1_receiver.vel.north = GLO1_rpvt.xv*GLO1_receiver.north.x + GLO1_rpvt.yv*GLO1_receiver.north.y +
                         GLO1_rpvt.zv*GLO1_receiver.north.z;
	GLO1_receiver.vel.east  = GLO1_rpvt.xv*GLO1_receiver.east.x + GLO1_rpvt.yv*GLO1_receiver.east.y;
	GLO1_receiver.vel.up    = GLO1_rpvt.xv*GLO1_receiver.up.x + GLO1_rpvt.yv*GLO1_receiver.up.y +
                         GLO1_rpvt.zv*GLO1_receiver.up.z;
	GLO1_speed=sqrt(GLO1_receiver.vel.north*GLO1_receiver.vel.north+GLO1_receiver.vel.east*GLO1_receiver.vel.east);
	if (GLO1_speed==0.0) GLO1_heading=0.0;
	else GLO1_heading=atan2(GLO1_receiver.vel.east,GLO1_receiver.vel.north);

//	 gotoxy(1,24);
//	 printf("GLO1_velocity->");
}

/*������������*/
void GLO1_DeMeander(int ch)
{
	char meand[170]={0};//��fifo1~fifo6��170bit��ŵ�������������
	unsigned long  mask = 0x20000000L;
	int index=0;
	for(int i=0;i<30;i++){
		if(GLO1_chan[ch].fifo1&mask){
			if (GLO1_chan[ch].reverseflag) {  //reverseflag=1Ϊ���࣬reverseflag=0Ϊ����
				meand[index++] = 0;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 1;
			}
		}else{
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 0;
			}
		}
		mask = mask>>1;
	}

	mask = 0x20000000L;
	for(int i=0;i<30;i++){
		if(GLO1_chan[ch].fifo2&mask){
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 0;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 0;
			}
		}
		mask = mask>>1;
	}

	mask = 0x20000000L;
	for(int i=0;i<30;i++){
		if(GLO1_chan[ch].fifo3&mask){
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 0;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 0;
			}
		}
		mask = mask>>1;
	}

	mask = 0x20000000L;
	for(int i=0;i<30;i++){
		if(GLO1_chan[ch].fifo4&mask){
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 0;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 0;
			}
		}
		mask = mask>>1;
	}


	mask = 0x20000000L;
	for(int i=0;i<30;i++){
		if(GLO1_chan[ch].fifo5&mask){
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 0;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 0;
			}
		}
		mask = mask>>1;
	}

	mask = 0x80000L;
	for(int i=0;i<20;i++){
		if(GLO1_chan[ch].fifo6&mask){
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 0;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//��ͨ��Ϊ������
			}
			else {
				meand[index++] = 0;
			}
		}
		mask = mask>>1;
	}

	char relative[85]={0};//����������֮�����
	for(int i=0,j=0;i<170;i=i+2){
		relative[j++] = meand[i];//������룬01->0;10->1
	}

	char buf[85] = {0};//�����������֮�����
	buf[0] = relative[0];
	for(int i=1;i<85;i++){
		buf[i] = relative[i-1]^relative[i];//��������
	}
	index = 0;
	GLO1_chan[ch].buf0 = 0L;//����������֮�����85bit�ŵ�long int ��
	GLO1_chan[ch].buf1 = 0L;
	GLO1_chan[ch].buf2 = 0L;
	for(int i=0;i<30;i++){
		GLO1_chan[ch].buf0 = (GLO1_chan[ch].buf0<<1) + buf[index++];
	}

	for(int i=0;i<30;i++){
		GLO1_chan[ch].buf1 = (GLO1_chan[ch].buf1<<1) + buf[index++];
	}

	for(int i=0;i<25;i++){
		GLO1_chan[ch].buf2 = (GLO1_chan[ch].buf2<<1) + buf[index++];
	//for (int i = 0;i<25;i++) {
	//	GLO1_chan[ch].buf2 = (GLO1_chan[ch].buf2 << 4) + buf[index++];//��30~5
	}
}


double GLO1_ReadComplement(unsigned long para, int firstbit)
{
	double res = 0;
	if ((para >> (firstbit - 1)) == 1)
	{
		//����
		para -= 1;
		para = ~para;
		for (int i = 0;i < firstbit - 1;++i)
		{
			res += (para & 1)*pow(2.0, i);
			para >>= 1;
		}
		res = -res;
	}
	else
	{
		//����
		for (int i = 0;i < firstbit - 1;++i)
		{
			res += (para & 1)*pow(2.0, i);
			para >>= 1;
		}
	}
	return res;
}

unsigned int GLO1_GET_INT_BIT(int ch,int num,int shift){
	unsigned  long mask = 1L;
	if(num==0){
		return (GLO1_chan[ch].buf0>>shift)&(mask);
	}else if(num==1){
		return (GLO1_chan[ch].buf1>>shift)&(mask);
	}else if(num==2){
		return (GLO1_chan[ch].buf2>>shift)&(mask);
	}
}

int GLO1_glonass_verify_data(int ch)//У��
{
	unsigned int	b9,
			b10,b11,b12,b13,b14,b15,b16,b17,b18,b19,
			b20,b21,b22,b23,b24,b25,b26,b27,b28,b29,
			b30,b31,b32,b33,b34,b35,b36,b37,b38,b39,
			b40,b41,b42,b43,b44,b45,b46,b47,b48,b49,
			b50,b51,b52,b53,b54,b55,b56,b57,b58,b59;
	unsigned int	b60,b61,b62,b63,b64,b65,b66,b67,b68,b69,
			b70,b71,b72,b73,b74,b75,b76,b77,b78,b79,
			b80,b81,b82,b83,b84,b85;
	unsigned int	beta1,beta2,beta3,beta4,beta5,beta6,beta7,beta8;
	unsigned int	c1,c2,c3,c4,c5,c6,c7,c_sigma;
	unsigned int	c_sum,c_comb,i_cor,K,error_word,error_bit_in_word;
	int	flag;

	c_comb = 0;
	i_cor = 0;
	K = 0;
	error_bit_in_word = 0;
	error_word = 0;

	b85 = GLO1_GET_INT_BIT(ch,0,29);
	b84 = GLO1_GET_INT_BIT(ch,0,28);
	b83 = GLO1_GET_INT_BIT(ch,0,27);
	b82 = GLO1_GET_INT_BIT(ch,0,26);
	b81 = GLO1_GET_INT_BIT(ch,0,25);
	b80 = GLO1_GET_INT_BIT(ch,0,24);
	b79 = GLO1_GET_INT_BIT(ch,0,23);
	b78 = GLO1_GET_INT_BIT(ch,0,22);
	b77 = GLO1_GET_INT_BIT(ch,0,21);
	b76 = GLO1_GET_INT_BIT(ch,0,20);
	b75 = GLO1_GET_INT_BIT(ch,0,19);
	b74 = GLO1_GET_INT_BIT(ch,0,18);
	b73 = GLO1_GET_INT_BIT(ch,0,17);
	b72 = GLO1_GET_INT_BIT(ch,0,16);
	b71 = GLO1_GET_INT_BIT(ch,0,15);
	b70 = GLO1_GET_INT_BIT(ch,0,14);
	b69 = GLO1_GET_INT_BIT(ch,0,13);
	b68 = GLO1_GET_INT_BIT(ch,0,12);
	b67 = GLO1_GET_INT_BIT(ch,0,11);
	b66 = GLO1_GET_INT_BIT(ch,0,10);
	b65 = GLO1_GET_INT_BIT(ch,0,9);
	b64 = GLO1_GET_INT_BIT(ch,0,8);
	b63 = GLO1_GET_INT_BIT(ch,0,7);
	b62 = GLO1_GET_INT_BIT(ch,0,6);
	b61 = GLO1_GET_INT_BIT(ch,0,5);
	b60 = GLO1_GET_INT_BIT(ch,0,4);
	b59 = GLO1_GET_INT_BIT(ch,0,3);
	b58 = GLO1_GET_INT_BIT(ch,0,2);
	b57 = GLO1_GET_INT_BIT(ch,0,1);
	b56 = GLO1_GET_INT_BIT(ch,0,0);

	b55 = GLO1_GET_INT_BIT(ch,1,29);
	b54 = GLO1_GET_INT_BIT(ch,1,28);
	b53 = GLO1_GET_INT_BIT(ch,1,27);
	b52 = GLO1_GET_INT_BIT(ch,1,26);
	b51 = GLO1_GET_INT_BIT(ch,1,25);
	b50 = GLO1_GET_INT_BIT(ch,1,24);
	b49 = GLO1_GET_INT_BIT(ch,1,23);
	b48 = GLO1_GET_INT_BIT(ch,1,22);
	b47 = GLO1_GET_INT_BIT(ch,1,21);
	b46 = GLO1_GET_INT_BIT(ch,1,20);
	b45 = GLO1_GET_INT_BIT(ch,1,19);
	b44 = GLO1_GET_INT_BIT(ch,1,18);
	b43 = GLO1_GET_INT_BIT(ch,1,17);
	b42 = GLO1_GET_INT_BIT(ch,1,16);
	b41 = GLO1_GET_INT_BIT(ch,1,15);
	b40 = GLO1_GET_INT_BIT(ch,1,14);
	b39 = GLO1_GET_INT_BIT(ch,1,13);
	b38 = GLO1_GET_INT_BIT(ch,1,12);
	b37 = GLO1_GET_INT_BIT(ch,1,11);
	b36 = GLO1_GET_INT_BIT(ch,1,10);
	b35 = GLO1_GET_INT_BIT(ch,1,9);
	b34 = GLO1_GET_INT_BIT(ch,1,8);
	b33 = GLO1_GET_INT_BIT(ch,1,7);
	b32 = GLO1_GET_INT_BIT(ch,1,6);
	b31 = GLO1_GET_INT_BIT(ch,1,5);
	b30 = GLO1_GET_INT_BIT(ch,1,4);
	b29 = GLO1_GET_INT_BIT(ch,1,3);
	b28 = GLO1_GET_INT_BIT(ch,1,2);
	b27 = GLO1_GET_INT_BIT(ch,1,1);
	b26 = GLO1_GET_INT_BIT(ch,1,0);

	b25 = GLO1_GET_INT_BIT(ch,2,24);
	b24 = GLO1_GET_INT_BIT(ch,2,23);
	b23 = GLO1_GET_INT_BIT(ch,2,22);
	b22 = GLO1_GET_INT_BIT(ch,2,21);
	b21 = GLO1_GET_INT_BIT(ch,2,20);
	b20 = GLO1_GET_INT_BIT(ch,2,19);
	b19 = GLO1_GET_INT_BIT(ch,2,18);
	b18 = GLO1_GET_INT_BIT(ch,2,17);
	b17 = GLO1_GET_INT_BIT(ch,2,16);
	b16 = GLO1_GET_INT_BIT(ch,2,15);
	b15 = GLO1_GET_INT_BIT(ch,2,14);
	b14 = GLO1_GET_INT_BIT(ch,2,13);
	b13 = GLO1_GET_INT_BIT(ch,2,12);
	b12 = GLO1_GET_INT_BIT(ch,2,11);
	b11 = GLO1_GET_INT_BIT(ch,2,10);
	b10 = GLO1_GET_INT_BIT(ch,2,9);
	b9  = GLO1_GET_INT_BIT(ch,2,8);
	
	beta8 = GLO1_GET_INT_BIT(ch,2,7);
	beta7 = GLO1_GET_INT_BIT(ch,2,6);
	beta6 = GLO1_GET_INT_BIT(ch,2,5);
	beta5 = GLO1_GET_INT_BIT(ch,2,4);
	beta4 = GLO1_GET_INT_BIT(ch,2,3);
	beta3 = GLO1_GET_INT_BIT(ch,2,2);
	beta2 = GLO1_GET_INT_BIT(ch,2,1);
	beta1 = GLO1_GET_INT_BIT(ch,2,0);

	c1 = beta1^b9^b10^b12^b13^b15^b17^b19^b20^b22^b24^b26^b28^b30^b32^b34^b35^b37^b39^b41^b43^b45^b47^
		 b49^b51^b53^b55^b57^b59^b61^b63^b65^b66^b68^b70^b72^b74^b76^b78^b80^b82^b84;
	
	c2 = beta2^b9^b11^b12^b14^b15^b18^b19^b21^b22^b25^b26^b29^b30^b33^b34^b36^b37^b40^b41^b44^
		 b45^b48^b49^b52^b53^b56^b57^b60^b61^b64^b65^b67^b68^b71^b72^b75^b76^b79^b80^b83^b84;
	
	c3 = beta3^b10^b11^b12^b16^b17^b18^b19^b23^b24^b25^b26^b31^b32^b33^b34^b38^b39^b40^b41^b46^
		 b47^b48^b49^b54^b55^b56^b57^b62^b63^b64^b65^b69^b70^b71^b72^b77^b78^b79^b80^b85;
	
	c4 = beta4^b13^b14^b15^b16^b17^b18^b19^b27^b28^b29^b30^b31^b32^b33^b34^b42^b43^b44^b45^b46^
		 b47^b48^b49^b58^b59^b60^b61^b62^b63^b64^b65^b73^b74^b75^b76^b77^b78^b79^b80;

	c5 = beta5^b20^b21^b22^b23^b24^b25^b26^b27^b28^b29^b30^b31^b32^b33^b34^b50^b51^b52^b53^b54^
		 b55^b56^b57^b58^b59^b60^b61^b62^b63^b64^b65^b81^b82^b83^b84^b85;

	c6 = beta6^b35^b36^b37^b38^b39^b40^b41^b42^b43^b44^b45^b46^b47^b48^b49^b50^b51^b52^b53^b54^
		 b55^b56^b57^b58^b59^b60^b61^b62^b63^b64^b65;

	c7 = beta7^b66^b67^b68^b69^b70^b71^b72^b73^b74^b75^b76^b77^b78^b79^b80^b81^b82^b83^b84^b85;

	c_sigma = beta1^beta2^beta3^beta4^beta5^beta6^beta7^beta8^b9^b10^
			  b11^b12^b13^b14^b15^b16^b17^b18^b19^b20^b21^b22^b23^b24^b25^b26^b27^b28^b29^b30^
			  b31^b32^b33^b34^b35^b36^b37^b38^b39^b40^b41^b42^b43^b44^b45^b46^b47^b48^b49^b50^
			  b51^b52^b53^b54^b55^b56^b57^b58^b59^b60^b61^b62^b63^b64^b65^b66^b67^b68^b69^b70^
			  b71^b72^b73^b74^b75^b76^b77^b78^b79^b80^b81^b82^b83^b84^b85;

	c_sum = c1+c2+c3+c4+c5+c6+c7;

	if(c_sigma == 0)
	{
		if(c_sum ==0)
			flag = 0;		//0:��ȷ;		1:��������	2:����;		3:����
		else if(c_sum >=1)
			flag = 2;			
	}
	else if(c_sigma ==1)
	{
		if(c_sum == 6)
			flag = 0;
		else if(c_sum >=2)
		{
			c_comb = (c7<<6)|(c6<<5)|(c5<<4)|(c4<<3)|(c3<<2)|(c2<<1)|c1;	//������bit

			i_cor = c_comb;
			while(i_cor != 0)
			{
				i_cor = i_cor>>1;
				K++;				
			}
			
			i_cor = c_comb + 8 - K;
			
			if(i_cor > 85)
				flag = 2;
			else if(i_cor >=1 && i_cor <=85)
			{
				if(i_cor>=1&&i_cor<=25){
					GLO1_chan[ch].buf2 = (0x00000001L<<(i_cor-1))^GLO1_chan[ch].buf2;
				}else if(i_cor>=26&&i_cor<=55){
					GLO1_chan[ch].buf1 = (0x00000001L<<(i_cor-26))^GLO1_chan[ch].buf1;
				}else if(i_cor>=56&&i_cor<=85){
					GLO1_chan[ch].buf0 = (0x00000001L<<(i_cor-56))^GLO1_chan[ch].buf0;
				}
				flag = 1;
			}
		}
		else if(c_sum == 0)
			flag = 2;
		else 
			flag = 3;
	}

	return flag;
}


/*******************************************************************************
FUNCTION GLO1_pream(char ch, char bit)
RETURNS  None.

PARAMETERS

	ch   channel number (0-11)
	bit  the latest GLO1_data bit from the satellite

PURPOSE
	This function finds the preamble GLO1_in the navigation message and synchronizes
   to the nav message

*******************************************************************************/
unsigned long GLO1_GLO1_pream = 0x3e375096;   //11 1110 0011 0111 0101 0000 1001 0110
const unsigned GLO1_pream1 = 0x1c8af69; //00 0001 1100 1000 1010 1111 0110 1001 ������
void GLO1_pream(char ch,char bit)
{ 

	//sta-tic unsigned long GLO1_pream=0x22c00000L;//0010 0010 1100 0000 0000 0000
	//stat-ic unsigned long GLO1_pream = 0x3e375096;   //11 1110 0011 0111 0101 0000 1001 0110
	//sta-tic const unsigned pream1 = 0x1c8af69; //00 0001 1100 1000 1010 1111 0110 1001 ������
	unsigned long  parity0,parity1;
	unsigned long TimeMark;//ʱ��־
	unsigned long HOWLastTwo;
	int sfid_s;
	long currentBitPos, frameLength;

	// fifo1��fifo0��������£�
	// fifo0��������ȵ����֣�fifo1������ź󵽵���
	// fifo0�ĵ�30λΪһ����Ч�֣���2λ��ǰһ֡��У��bit��D29,D30
	// fifo1�ĵ�30λΪһ����Ч�֣���2λ��ǰһ֡��У��bit��D29,D30��fifo0�����2������ͬ

	// fifo1�����λ������fifo0�����λ

	if (GLO1_chan[ch].fifo1 & 0x20000000L) //
	{
		GLO1_chan[ch].fifo0=(GLO1_chan[ch].fifo0<<1)+ 1;//fifo0�ĵ�1λ����û�ˣ�
	}
	else
	{
		GLO1_chan[ch].fifo0=GLO1_chan[ch].fifo0<<1;
	}


	if (GLO1_chan[ch].fifo2 & 0x20000000L) //
	{
		GLO1_chan[ch].fifo1=(GLO1_chan[ch].fifo1<<1)+ 1;//fifo0�ĵ�1λ����û�ˣ�
	}
	else
	{
		GLO1_chan[ch].fifo1=GLO1_chan[ch].fifo1<<1;
	}

	if (GLO1_chan[ch].fifo3 & 0x20000000L) //todo
	{
		GLO1_chan[ch].fifo2=(GLO1_chan[ch].fifo2<<1)+ 1;//fifo0�ĵ�1λ����û�ˣ�
	}
	else
	{
		GLO1_chan[ch].fifo2=GLO1_chan[ch].fifo2<<1;
	}

	if (GLO1_chan[ch].fifo4 & 0x20000000L) //todo
	{
		GLO1_chan[ch].fifo3=(GLO1_chan[ch].fifo3<<1)+ 1;//fifo0�ĵ�1λ����û�ˣ�
	}
	else
	{
		GLO1_chan[ch].fifo3=GLO1_chan[ch].fifo3<<1;
	}

	if (GLO1_chan[ch].fifo5 & 0x20000000L) //todo
	{
		GLO1_chan[ch].fifo4=(GLO1_chan[ch].fifo4<<1)+ 1;//fifo0�ĵ�1λ����û�ˣ�
	}
	else
	{
		GLO1_chan[ch].fifo4=GLO1_chan[ch].fifo4<<1;
	}


	if (GLO1_chan[ch].fifo6 & 0x80000L) //todo
	{
		GLO1_chan[ch].fifo5=(GLO1_chan[ch].fifo5<<1)+ 1;//fifo0�ĵ�1λ����û�ˣ�
	}
	else
	{
		GLO1_chan[ch].fifo5=GLO1_chan[ch].fifo5<<1;
	}


	if (GLO1_chan[ch].fifo7 & 0x20000000L) 
	{
		GLO1_chan[ch].fifo6=(GLO1_chan[ch].fifo6<<1)+ 1;
	}
	else
	{
		GLO1_chan[ch].fifo6=GLO1_chan[ch].fifo6<<1;
	}

	GLO1_chan[ch].fifo7=(GLO1_chan[ch].fifo7<<1)+bit;// �µ�bit����fifo7�����λ

	// 30 λ��Ч�����ж�ͬ��
	GLO1_chan[ch].fifo0 = GLO1_chan[ch].fifo0 & 0x3fffffff;
	GLO1_chan[ch].fifo1 = GLO1_chan[ch].fifo1 & 0x3fffffff;
	GLO1_chan[ch].fifo2 = GLO1_chan[ch].fifo2 & 0x3fffffff;
	GLO1_chan[ch].fifo3 = GLO1_chan[ch].fifo3 & 0x3fffffff;
	GLO1_chan[ch].fifo4 = GLO1_chan[ch].fifo4 & 0x3fffffff;
	GLO1_chan[ch].fifo5 = GLO1_chan[ch].fifo5 & 0x3fffffff;
	GLO1_chan[ch].fifo6 = GLO1_chan[ch].fifo6 & 0xfffff;
	GLO1_chan[ch].fifo7 = GLO1_chan[ch].fifo7 & 0x3fffffff;

	currentBitPos = GLO1_chan[ch].t_count+1;

	// printf("fifo0: %0X \n",GLO1_chan[ch].fifo0);

	//GLO1_bitSync��GLO1_chan[ch].subFrameSyncFlag = GLO1_SUBFRAME_SYNCHRONIZING;
	if (GLO1_chan[ch].subFrameSyncFlag == GLO1_SUBFRAME_SYNCHRONIZED){
		
		frameLength = currentBitPos>=GLO1_chan[ch].expectedFrameHead ?
			currentBitPos-GLO1_chan[ch].expectedFrameHead : currentBitPos+3000-GLO1_chan[ch].expectedFrameHead;
		
		if (frameLength==200){
			GLO1_chan[ch].subFrameSyncFlag = GLO1_SUBFRAME_SYNCHRONIZING;
			
			if ((((GLO1_GLO1_pream^GLO1_chan[ch].fifo0) == 0) && ((GLO1_GLO1_pream^GLO1_chan[ch].fifo7) == 0))
				|| (((GLO1_pream1^GLO1_chan[ch].fifo0) == 0) && ((GLO1_pream1^GLO1_chan[ch].fifo7) == 0))){
				//printf("fifo0: %0X \n",GLO1_chan[ch].fifo0);
				//printf("fifo7: %0X \n",GLO1_chan[ch].fifo7);

				if ((((GLO1_pream1^GLO1_chan[ch].fifo0) == 0) && ((GLO1_pream1^GLO1_chan[ch].fifo7) == 0))) {
					GLO1_chan[ch].reverseflag = 1;//��ʾ������ͬ��
				}else {
					GLO1_chan[ch].reverseflag = 0;
				}
				
				GLO1_DeMeander(ch);

				// check and GLO1_handle 1 bit error
				int flag = GLO1_glonass_verify_data(ch);
				GLO1_chan[ch].buf2 = GLO1_chan[ch].buf2 << 5;
				if(flag == 2 || flag ==3){
					GLO1_chan[ch].subFrameSyncFlag = GLO1_SUBFRAME_SYNCHRONIZING;
			
				}else {

					sfid_s = (GLO1_chan[ch].buf0&0x1e000000L)>>25; 

					/*printf("sfid_s: %d \n",sfid_s);
					printf("sfid_s:%d  buf0: %0X \n",sfid_s,GLO1_chan[ch].buf0);
					printf("sfid_s:%d  buf1: %0X \n",sfid_s,GLO1_chan[ch].buf1);
					printf("sfid_s:%d  buf2: %0X \n",sfid_s,GLO1_chan[ch].buf2);*/

					GLO1_chan[ch].subFrameSyncFlag = GLO1_SUBFRAME_SYNCHRONIZED;
					GLO1_chan[ch].expectedFrameHead = (currentBitPos)%3000;
					GLO1_chan[ch].expectedSubFrameID= sfid_s==15 ? 1 : sfid_s+1;
					GLO1_chan[ch].frame_ready = 1;

					//if (sfid_s == 1){
					//	// reslove x.dot

					//	unsigned long xdotMask = 0x1ffL;
					//	//unsigned long xdot = 0xffffff ^ (((GLO1_chan[ch].buf0 & xdotMask) << 15) | (GLO1_chan[ch].buf1 >> 15));
					//	long xdot = ((GLO1_chan[ch].buf0 & xdotMask) << 15) | (GLO1_chan[ch].buf1 >> 15);
					//	printf("sfid_s:%d  xdot: %0X \n", sfid_s, xdot);

					//	if (GLO1_bit_test_l(xdot, 24)) xdot = xdot | 0xFF000000L;

					//	double x_dot = xdot*(9.5367431640625e-7);
					//	printf("x_dot:  %f ", x_dot);

					//	unsigned long xddMask = 0x7c00L;
					//	long xdd = (GLO1_chan[ch].buf1 &xddMask) >> 10;
					//	printf("sfid_s:%d  xdd: %0X \n", sfid_s, xdd);

					//	if (GLO1_bit_test_l(xdd, 5)) xdd = xdd | 0xFFFFFFe0L;

					//	double x_dot_dot = xdd*(9.31322574615479e-10);

					//	printf("x_dot_dot:  %f ", x_dot_dot);

					//	unsigned long xMask = 0x3ffL;
					//	//long xx = ((GLO1_chan[ch].buf1&xMask) << 17) | (GLO1_chan[ch].buf2 >> 8);
					//	long xx = ((GLO1_chan[ch].buf1&xMask) << 17) | (GLO1_chan[ch].buf2 >> 13);
					//	if (GLO1_bit_test_l(xx, 27)) xx = xx | 0xF8000000L;

					//	double xxxx = xx*(4.8828125e-4);
					//	printf("xxxx:  %f", xxxx);


					//	GLO1_chan[ch].offset = GLO1_chan[ch].t_count - 199;
					//	GLO1_writeEpoch(ch, (0x1f & GLO1_readEpochCheck(ch)) | 0x0000);

					//	if (GLO1_chan[ch].offset < 0.0){
					//		GLO1_chan[ch].offset += 3000;
					//	}

					//	GLO1_bTowDecoded = true;//��ӵ�


					//	GLO1_chan[ch].tow_sync = 1;
					//	GLO1_chan[ch].sfid = sfid_s;
					//}
				//	}else if (sfid_s == 2) {
				//	unsigned long ydotMask = 0x1ffL;
				//	
				//	long ydot = ((GLO1_chan[ch].buf0 & ydotMask) << 15) | (GLO1_chan[ch].buf1 >> 15);
				//	printf("sfid_s:%d  ydot: %0X \n",sfid_s,ydot);
				//	//ydot =0x0d09b3L;

				//	//long ydotd = (long)GLO1_ReadComplement(ydot, 24);
				//	int GLO1_sign = 1;
				//	if (GLO1_bit_test_l(ydot,24)){

				//		//ydot=ydot | 0xFF000000L;
				//		ydot = ydot&0x7fffffL;
				//		GLO1_sign = -1;
				//	}

				//	double y_dot = GLO1_sign*ydot*(9.5367431640625e-7);
				//	printf("y_dot:  %f ", y_dot);

				//	unsigned long yddMask = 0x7c00L;
				//		long ydd = (GLO1_chan[ch].buf1 &yddMask)>>10;
				//		printf("sfid_s:%d  ydd: %0X \n",sfid_s,ydd);

				//		if (GLO1_bit_test_l(ydd,5)) ydd=ydd | 0xFFFFFFe0L;

				//		double y_dot_dot = ydd*(9.31322574615479e-10);

				//		printf("y_dot_dot:  %f ",y_dot_dot);

				//		unsigned long yMask = 0x3ffL;
				//		//long yy = ((GLO1_chan[ch].buf1&yMask) << 17) | (GLO1_chan[ch].buf2 >> 8);
				//		long yy = ((GLO1_chan[ch].buf1&yMask) << 17) | (GLO1_chan[ch].buf2 >> 13);
				//		if (GLO1_bit_test_l(yy,27)) yy=yy | 0xF8000000L;

				//		double yyyy = yy*(4.8828125e-4);
				//		printf("yyyy:  %f", yyyy);
				//}else if (sfid_s == 3) {
				//	unsigned long zdotMask = 0x1ffL;
				//	
				//	long zdot = ((GLO1_chan[ch].buf0 & zdotMask) << 15) | (GLO1_chan[ch].buf1 >> 15);
				//	printf("sfid_s:%d  zdot: %0X \n",sfid_s,zdot);
				//	//ydot =0x0d09b3L;
				//	int GLO1_sign = 1;
				//	//long ydotd = (long)GLO1_ReadComplement(ydot, 24);
				//	if (GLO1_bit_test_l(zdot,24)) {
				//		//zdot=zdot | 0xFF000000L;
				//		zdot = zdot&0x7fffffL;
				//		GLO1_sign = -1;
				//	}

				//	double z_dot =GLO1_sign*zdot*(9.5367431640625e-7);
				//	printf("z_dot:  %f ", z_dot);

				//		unsigned long zddMask = 0x7c00L;
				//		long zdd = (GLO1_chan[ch].buf1 &zddMask)>>10;
				//		printf("sfid_s:%d  zdd: %0X \n",sfid_s,zdd);

				//		GLO1_sign = 1;
				//		if (GLO1_bit_test_l(zdd,5)) {
				//			//zdd=zdd | 0xFFFFFFe0L;
				//			zdd = zdd&0x0fL;
				//			GLO1_sign = -1;
				//		}
				//		double m30=9.31322574615479e-10;
				//		double z_dot_dot = GLO1_sign*zdd*(m30);

				//		printf("z_dot_dot:  %f ",z_dot_dot);

				//		unsigned long zMask = 0x3ffL;
				//		//long zz = ((GLO1_chan[ch].buf1&zMask) << 17) | (GLO1_chan[ch].buf2 >> 8);
				//		long zz = ((GLO1_chan[ch].buf1&zMask) << 17) | (GLO1_chan[ch].buf2 >> 13);
				//		GLO1_sign = 1;
				//		if (GLO1_bit_test_l(zz,27)) {
				//			//zz=zz | 0xF8000000L;
				//			zz = zz & 0xffffffL;
				//			GLO1_sign =-1;
				//		}

				//		double zzzz = GLO1_sign*zz*(4.8828125e-4);
				//		printf("zzzz:  %f", zzzz);
				//	}
					if (sfid_s>3 && sfid_s<15){
					//else if (sfid_s>1 && sfid_s<15){
						

						GLO1_chan[ch].offset=GLO1_chan[ch].t_count-199-(sfid_s-1)*200;
						GLO1_writeEpoch(ch,(0x1f&GLO1_readEpochCheck(ch))|0x000);
						if (GLO1_chan[ch].offset<0.0){
							GLO1_chan[ch].offset+=3000;
						}
						GLO1_chan[ch].tow_sync=1;
						GLO1_chan[ch].sfid=sfid_s;	


						GLO1_bTowDecoded = true;//��ӵ�
					}
				}
			}

			if (GLO1_chan[ch].subFrameSyncFlag != GLO1_SUBFRAME_SYNCHRONIZED)
			{
				GLO1_chan[ch].t_count = 0;
				GLO1_chan[ch].n_frame = 0;
			}
		}
	}
	else
	{
		if ((((GLO1_GLO1_pream^GLO1_chan[ch].fifo7) == 0)) || (((GLO1_pream1^GLO1_chan[ch].fifo7) == 0)))
		{
			
			if ((GLO1_pream1^GLO1_chan[ch].fifo7) == 0) {
				GLO1_chan[ch].reverseflag = 1;//��ʾ������ͬ��
			}else {
				GLO1_chan[ch].reverseflag = 0;
			}

			GLO1_DeMeander(ch);//������������

			// check and GLO1_handle 1 bit error
			int flag = GLO1_glonass_verify_data(ch);
			GLO1_chan[ch].buf2 = GLO1_chan[ch].buf2 << 5;
			//printf("fisrt flag:  %d\n",flag);
			if(flag == 2 || flag ==3){
				GLO1_chan[ch].subFrameSyncFlag = GLO1_SUBFRAME_SYNCHRONIZING;
			}else {
				sfid_s = (GLO1_chan[ch].buf0&0x1e000000L)>>25; //��ȡ����

				/*printf("sfid_s: %d \n",sfid_s);
				printf("sfid_s:%d  buf0: %0X \n",sfid_s,GLO1_chan[ch].buf0);
				printf("sfid_s:%d  buf1: %0X \n",sfid_s,GLO1_chan[ch].buf1);
				printf("sfid_s:%d  buf2: %0X \n",sfid_s,GLO1_chan[ch].buf2);*/

				//printf("buf0: %0X \n",GLO1_chan[ch].buf0);

				GLO1_chan[ch].subFrameSyncFlag = GLO1_SUBFRAME_SYNCHRONIZED;
				GLO1_chan[ch].expectedFrameHead = (currentBitPos)%3000;
				GLO1_chan[ch].expectedSubFrameID= sfid_s==15 ? 1 : sfid_s+1;
				GLO1_chan[ch].frame_ready = 1;
				//if (sfid_s==1){
				//		unsigned long xdotMask = 0x1ffL;
				//		//unsigned long xdot = 0xffffff ^ (((GLO1_chan[ch].buf0 & xdotMask) << 15) | (GLO1_chan[ch].buf1 >> 15));
				//		long xdot =  ((GLO1_chan[ch].buf0 & xdotMask) << 15) | (GLO1_chan[ch].buf1 >> 15);
				//		printf("sfid_s:%d  xdot: %0X \n",sfid_s,xdot);

				//		if (GLO1_bit_test_l(xdot,24)) xdot=xdot | 0xFF000000L;
				//	
				//		double x_dot = xdot*(9.5367431640625e-7);
				//		printf("x_dot:  %f ", x_dot);

				//		unsigned long xddMask = 0x7c00L;
				//		long xdd = (GLO1_chan[ch].buf1 &xddMask)>>10;
				//		printf("sfid_s:%d  xdd: %0X \n",sfid_s,xdd);

				//		if (GLO1_bit_test_l(xdd,5)) xdd=xdd | 0xFFFFFFe0L;

				//		double x_dot_dot = xdd*(9.31322574615479e-10);

				//		printf("x_dot_dot:  %f ", x_dot_dot);

				//		unsigned long xMask = 0x3ffL;
				//		//long xx = ((GLO1_chan[ch].buf1&xMask) << 17) | (GLO1_chan[ch].buf2 >> 8);
				//		long xx = ((GLO1_chan[ch].buf1&xMask) << 17) | (GLO1_chan[ch].buf2 >> 13);
				//		if (GLO1_bit_test_l(xx,27)) xx=xx | 0xF8000000L;

				//		double xxxx = xx*(4.8828125e-4);
				//		printf("xxxx:  %f", xxxx);


				//		GLO1_chan[ch].offset=GLO1_chan[ch].t_count-199;
				//		GLO1_writeEpoch(ch,(0x1f&GLO1_readEpochCheck(ch))|0x000);  
				//	
				//		if (GLO1_chan[ch].offset<0.0){ 
				//			GLO1_chan[ch].offset+=3000;
				//		}
				//		GLO1_chan[ch].tow_sync=1;
				//		GLO1_chan[ch].sfid=sfid_s;
				//	

				//		GLO1_bTowDecoded = true;//��ӵ�
				//}
				if (sfid_s>1 && sfid_s<15){
						
						GLO1_chan[ch].offset=GLO1_chan[ch].t_count-199-(sfid_s-1)*200;
						GLO1_writeEpoch(ch,(0x1f&GLO1_readEpochCheck(ch))|0x000);
						if (GLO1_chan[ch].offset<0.0){
							GLO1_chan[ch].offset+=3000;
						}
						GLO1_chan[ch].tow_sync=1;
						GLO1_chan[ch].sfid=sfid_s;	

						GLO1_bTowDecoded = true;//��ӵ�
			} 
			}
		}
	}	
}