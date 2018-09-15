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

#include "BD3_gpsconst.h"
#include "BD3_gpsstruct.h"
#include "../receiver/readData.h"
#include "BD3_correlatorProcess.h"
#include "BD3_acqCA.h"
#include "BD3_cagen.h"
#include "BD3_gp2021.h"
#include "BD3_gpsfuncs.h"
#include "BD3_gpsrcvr.h"
#include "../receiver/fft.h"

#include "../receiver/UI.h"
#include "../receiver/UpdateUIMessages.h"

#include "../core/global_var.h"
#include "../core/global_STAP.h"

#include "../receiver/anti_receiver_data.h"
#include "../core/socket.h"
/************************************************************************/

/************************************************************************/





#ifdef BD3_DEMO_CONSOLE

#endif

#include <windows.h>

//#include "TimeCounterEx.h"

#define	BD3_IRQLEVEL        0				   // IRQ Line,����ʵʱ�����������Ҫ����

BD3_CHANNEL		  BD3_chan[BD3_chmax + 1];				// ȫ�̱��������ڴ洢ÿ��ͨ�����ٻ�·��״̬
BD3_CORRELATOR	   BD3_correlator[BD3_chmax + 1];		// ȫ�̱��������ڴ洢ÿ��ͨ���������״̬
BD3_SVSTRUCT	      BD3_svStruct[BD3_MaxSvNumber + 1];	// ��ʾ���ǿ�����
long		      BD3_globalBufLength[BD3_chmax + 1];	// ���ڱ���ÿ��ͨ����������ݳ��ȣ����ʼ��
double  *BD3_buffer[BD3_chmax + 1];			// ���ڱ���ÿ��ͨ���Ļ������ݣ����ʼ������ռ�
long		      BD3_correlatorDataLength;	// ���л����������ݳ���
long		      BD3_TIC_RT_CNTR;				// TICʵʱ��������
long		      BD3_TIC_CNTR;					// TIC������������Ӧ��TIC�жϵķ�������
										         // ע�⣺TIC_CNTR�ĸ�ֵֻ����programTIC����ʵ��
double         BD3_TIC_CNTR_RES;
double         BD3_TIC_CNTR_DBL;
long		      BD3_TIC_OCCUR_FLAG;			// TIC�жϷ����ı�־

int			   BD3_display_page = 0;			// ������ʾ��i��ҳ��
unsigned	BD3_test[16] =
			{	0x0001,0x0002,0x0004,0x0008,0x0010,0x0020,0x0040,0x0080,
				0x0100,0x0200,0x0400,0x0800,0x1000,0x2000,0x4000,0x8000};
unsigned int BD3_tr_ch[13];					   // ����0���ã�����Ϊ������Ч�۲����ݵ�chan�ı��
int			BD3_out_debug, BD3_out_pos, BD3_out_vel, BD3_out_time;		// �Ƿ����debug��PVT��Ϣ�ı�־��
BD3_DMS			BD3_cur_lat, BD3_cur_long;			   // ������ʾ��γ��


// ���¶����������ȱʡֵ����ͨ���ı��ļ�rcvr_par.dat���趨ʵ��
int			   BD3_nav_tic;						// һ��nav_up��ʱ������Ӧ��TIC�жϵĴ���
int            BD3_ICP_CTL = 0;		         // ����ۻ��ز���λ���Ǹ��ٻ�·��ֵ����־
double         BD3_code_corr;              // CA�����ʵ����������֣�1.023MHz��Ӧ1.0
long           BD3_time_on = 0;	            // ������������Ϊ�룬����ͣ��������ֹ

double         BD3_nav_up = 1.0;					// ������Ļ�����λ����λ����
double		   BD3_speed, BD3_heading;          // ���ջ��ٶȵľ���ֵ�ͷ�λ��
long		BD3_d_tow;								// ���ջ�ʱ����λ����
int			BD3_key = 0;								// �����̵İ�����Ϣ
int			BD3_tic_count = 0;						// TIC��������Ҳ�ǲ����жϷ��������ļ�����
												// ע�⣺�����жϵ�Ƶ������ȱʡΪ��100ms
												// ������ΧΪ1s[0-9]
int			BD3_hms_count = 0;						// TIC��������������ΧΪ1����[0-599]
int			BD3_nav_count;							// TIC��������������ΧΪ[0,nav_tic-1]
int			BD3_min_flag;							// ���Ӽ�������־
int			BD3_nav_flag;							// ���㵼�����־
int			BD3_sec_flag;							// ���־
int			BD3_n_track;							// ����(ͨ������trackingģʽ)�����Ǹ���
unsigned int BD3_interr_int = 512;
double		BD3_clock_offset = 0.0;					// ���ջ�����Ư����λ��ppm,��ʼֵδ֪���ٶ�Ϊ0
												// ��ֵ��ʾ���ջ���Ƶ�ʵ��ڱ��ֵ
BD3_XYZ			BD3_rec_pos_ecef;						// ���ջ�������
long		BD3_i_TIC_dt;							// ���μ��㵼����ļ��,��λ���������
double		BD3_TIC_dt;								// ���μ��㵼����ļ��,��λ����
												// ����TIC_dt=i_TIC_dt*�������
double		BD3_m_time[3];							// ���ջ�ʱ��,[1]�ǵ�ǰʱ�̵ģ�[0]����һ�ε�
												// [2]�ƺ�û����
												// m_time���ڱ�ʾ���ջ���GPSʱ��
												// ��ԭ���ĳ�����m_timeû����Ч�ĸ���,
												// ��Ӧ�ÿ����жϱ�־sec_flag��ά��
double		BD3_delta_m_time;						// ���ջ�ʱ���С������
double		BD3_m_error;							// ���ջ��Ĳ���ʱ���GPS��������

long		BD3_TIC_sum;							// ��������ж��������ۻ��ļ���ֵ
//int			astat;							// ������GP2021��������dump ready��־�ļĴ���
//int			mstat;							// ������GP2021��������miss data��־�ļĴ���
char		**BD3_caTable;							// ��ʵʱ����ʱ���洢14��10230��CA���

double		BD3_DLLc1, BD3_DLLc2;						// DLL��ϵ��
double		BD3_PLLc0, BD3_PLLc1, BD3_PLLc2;				// PLL��ϵ��
double		BD3_FLLa1, BD3_FLLa2;						// FLL��ϵ��

// �����õ�ȫ�̱���
double		*BD3_IQQ, *BD3_IQI;
double		BD3_IBUF[3000], BD3_QBUF[3000];
FILE		*BD3_fpIQ;
long		BD3_IQCounter;
FILE		*BD3_fpobs;
FILE		*BD3_fpeph;


double		BD3_threshold_sig_search;				// �źż������							

//char		data[SAMPLESPMS+1];							// �źŻ�����
//char		data[6000];							// �źŻ�����
double		BD3_minFreqErr[BD3_chmax + 1], BD3_maxFreqErr[BD3_chmax + 1];

unsigned long BD3_uLastPCTick, BD3_uCurrentPCTick;

__int64 BD3_iTotalBitCounter = 0;

bool BD3_bLocalTimeSetByTow = false;
bool BD3_bTowDecoded = false;

// �ⲿ����
extern BD3_XYZ			BD3_rec_pos_xyz;
extern FILE			*BD3_stream, *BD3_debug, *BD3_in, *BD3_out, *BD3_kalm;
extern BD3_LLH			BD3_current_loc, BD3_rp_llh;
extern BD3_LLH			BD3_rec_pos_llh;
extern time_t		BD3_thetime;
extern int			BD3_status;
extern unsigned long  BD3_clock_tow;
extern int			BD3_alm_gps_week, BD3_gps_week, BD3_almanac_valid, BD3_almanac_flag, BD3_handle;
extern BD3_SATVIS		BD3_xyz[13];//
extern BD3_EPHEMERIS	BD3_gps_eph[13];//
extern double		BD3_gdop, BD3_pdop, BD3_hdop, BD3_vdop, BD3_tdop, BD3_alm_toa;
extern BD3_ECEFT		BD3_track_sat[13];
extern double		BD3_dt[13], BD3_cbias;
extern BD3_XYZ			BD3_d_sat[13];
extern double		BD3_meas_dop[13];
extern BD3_PVT			BD3_rpvt;
extern BD3_STATE		BD3_receiver;
extern int			BD3_m_tropo, BD3_m_iono, BD3_align_t;			// flags for using tropo and iono models
extern BD3_ALMANAC		BD3_gps_alm[13];//33
extern double		BD3_carrier_ref, BD3_code_ref;
extern char			BD3_tzstr[40];							// = "TZ=PST8PDT";
extern double		BD3_mask_angle;
extern double		BD3_b0, BD3_b1, BD3_b2, BD3_b3, BD3_al0, BD3_al1, BD3_al2, BD3_al3;		// broadcast ionospheric delay model
extern double		BD3_a0, BD3_a1, BD3_tot, BD3_WNt, BD3_dtls, BD3_WNlsf, BD3_DN, BD3_dtlsf;	//broadcast UTC data

extern int BD3_ADNumber;   //0624
float   BD3_CARRIER_FREQ;   //�����Ƶ  0625
double BD3_SAMPLING_FREQ;     //0626
//double	ACQ_SAMPLING_FREQ = 4.096e6;//2.048
double	BD3_ACQ_SAMPLING_FREQ = 11 * 2.048e6;//revised
double  BD3_SAMPLING_INT;      //0626
long    BD3_DETECTSIZE;        //0626
long	BD3_TIC_ref;           //0626
double  BD3_deltaPhaseConst;   //0626
double  BD3_deltaCodePhaseConst;    //0626
long    BD3_SAMPLESPMS;         //0627
long    BD3_ACC_INT_PERIOD;     //0627
char    *BD3_data;              //0627
long    BD3_FFTSIZE;            //0627
//hide by jh 3-11 long    bufSize;            //0627

FILE   *BD3_fpCodeRange1, *BD3_fpCodeRange2;     //0628
FILE   *BD3_fpCarrRange1, *BD3_fpCarrRange2;     //0628
long   BD3_ldcount = 0;                     //0628
FILE   *BD3_fpdoppler;                      //0628
FILE   *BD3_fpCNo;                          //0628
FILE   *BD3_out_trtime;

char   BD3_if_data_file[4096];
/*******************************************************************************
FUNCTION main()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This is the main program to control the GPS receiver

*******************************************************************************/

extern char BD3_last_prn[12];
char BD3_hot_cold;
extern char BD3_curloc_file[];
FILE *BD3_daicy_file_pr;
FILE *BD3_daicy_file_pos;
//extern 	char    inNavFile[100]; //yuan�����ļ�
long BD3_sim_main_init()
{
	
	char  ch;
	long num,TIC_DIF;
	long numRead;
	FILE* fpData;
	FILE* fpOption;
	char fileName[200];
	BD3_ADNumber = (int)(long long)pow(2.0, BD3_ADNumber);
	BD3_carrier_ref = BD3_CARRIER_FREQ;

	BD3_SAMPLING_INT = 1.0 / BD3_SAMPLING_FREQ;
	BD3_DETECTSIZE = (long)(long long)(BD3_SAMPLING_FREQ*0.01 + 0.5);
	//DETECTSIZE��10��������ݳ��ȣ���ΪSAMPLING_FREQ��1��ĳ���
	
	BD3_TIC_ref = (long)(long long)(BD3_SAMPLING_FREQ*0.1 + 0.5 - 1);
	// 100ms��Ӧ���жϼ���ֵ 100��������ݵ���  Ԥ��TIC_CNTR�ļ�����100����
	
	BD3_deltaPhaseConst = BD3_SAMPLING_INT*BD3_carrierNCOMax;

	//deltaCodePhaseConst = SAMPLING_INT*2.046e6*d_2p40;
	BD3_deltaCodePhaseConst = BD3_SAMPLING_INT*10.23e6*BD3_d_2p40;//done


	BD3_SAMPLESPMS = (long)(long long)(BD3_SAMPLING_FREQ*0.001);   //0627
	
	BD3_ACC_INT_PERIOD = BD3_SAMPLESPMS / 2;//+1;

	BD3_data = new char[BD3_SAMPLESPMS + 1];

	BD3_out_trtime = fopen(".\\tr_time.txt", "w");
	BD3_daicy_file_pr = fopen(".\\daicy_pr.txt", "w");
	BD3_daicy_file_pos = fopen(".\\daicy_pos.txt", "w");

	
/*
		float fnumber=(float)SAMPLING_FREQ/500;
		int fftcount=0;
		do 
		{
			fnumber=fnumber/2;
			fftcount++;
		}
		while(fnumber>2);*/
	
/*
			if(fnumber<=1.5)
				FFTSIZE=(int)(pow(2,fftcount)*1.5);
			else 
				FFTSIZE=(int)pow(2,fftcount+1);
*/
	// read configuration parameters, should be re-written
	BD3_FFTSIZE = BD3_ACQ_SAMPLING_FREQ*2e-3;
	BD3_read_rcvr_par();//2010.11.21
	BD3_rec_pos_xyz.x = 0.0;
	BD3_rec_pos_xyz.y = 0.0;
	BD3_rec_pos_xyz.z = 0.0;


	if (BD3_out_pos == 1 || BD3_out_vel == 1 || BD3_out_time == 1)
		BD3_stream = fopen("gpsrcvr.log", "w");
	BD3_read_initial_data();

	// Ϊ�ź�������̬��������
	BD3_initSignalSearch();

	// ��鱣����ļ����Ƿ���н��ջ���λ��
	//current_loc=receiver_loc();
	BD3_rec_pos_llh.lon = BD3_current_loc.lon;
	BD3_rec_pos_llh.lat = BD3_current_loc.lat;
	BD3_rec_pos_llh.hae = BD3_current_loc.hae;
///2010.11.21
	////rec_pos_llh.lon=119/57.296;
	//rec_pos_llh.lon = 116 / 57.296;
	////rec_pos_llh.lat=39/57.296;
	//rec_pos_llh.lat = 40 / 57.296;
	//rec_pos_llh.hae=100;
///////////////////////////////////////////////////
	BD3_rec_pos_xyz = BD3_llh_to_ecef(BD3_rec_pos_llh);
	//��Դ�������ԣ�ֱ�Ӷ�Դ�����ļ�����֤�����������Ƿ���ʱ���д�

	


	// ����navFix�ĸ���ʱ������
	BD3_nav_tic = (int)(long long)(BD3_nav_up / 0.1 + 0.5);
	BD3_programTIC(BD3_TIC_ref);
	for (ch = 0; ch <= BD3_chmax; ch++)
    {
		BD3_chan[ch].state = BD3_CHN_OFF;//����ر�״̬����ͨ������
		BD3_chan[ch].prn = 0;
    }
	for (ch = BD3_MinSvNumber; ch <= BD3_MaxSvNumber; ch++)
	{
		BD3_svStruct[ch].state = BD3_AVAILABLE;
		BD3_svStruct[ch].undetectedCounter = 0;
		BD3_svStruct[ch].NumBTWEachTrial = BD3_ReAcqTime;

		BD3_svStruct[ch].NumToTryAcq = 1;
		BD3_svStruct[ch].maxf = BD3_MAXF;
		BD3_svStruct[ch].minf = BD3_MINF;
		BD3_xyz[ch].azimuth = BD3_xyz[ch].doppler = BD3_xyz[ch].elevation = 0.0;
		BD3_xyz[ch].x = BD3_xyz[ch].y = BD3_xyz[ch].z = 0.0;
	}
	time(&BD3_thetime);
	
#ifndef REAL_TIME
	BD3_initCorrelator();
#endif
	
	//hot_cold = 2;//������
	BD3_hot_cold = 1;//������
	if (BD3_hot_cold == 1)
	{
		BD3_ch_alloc();
	} 
	else if (BD3_hot_cold == 2)
	{
		BD3_hot_ch_alloc();
	}

	BD3_m_time[1] = BD3_clock_tow;

	BD3_read_ephemeris();

#ifndef REAL_TIME
	BD3_TIC_RT_CNTR = 0;
	num = BD3_ACC_INT_PERIOD;
	
	BD3_initTrackLoopPar();
#endif
	
	/*fpData = fopen(BD3_if_data_file, "rb");
	if(fpData==NULL)
	{
		perror("Cannot open data file! Program Exited");
		exit(0);
	}

	readFileHeader(fpData);*/
	fopen_s(&BD3_fpeph, "monitor.eph", "wb");

	return num;
}

int BD3_is_state_3 = 0;
void BD3_sim_main(long num)
{
	long TIC_DIF;
	long tmp_num;

	while (BD3_key != 'x' && BD3_key != 'X' && BD3_receiver_buf.buf_remain_len >= num)  // ��ѭ����ʼ
	{
#ifndef REAL_TIME

		//socket���䶨λ��������������㷨����ʱ��
		if (0 == BD3_is_state_3)
		{
			send_res_BD3.var1 += num;

			for (int i = 0; i <= BD3_chmax; i++)
			{
				if (3 == BD3_chan[i].state)
				{
					BD3_is_state_3 = 1;
					send_res_BD3.var1 /= sameple_rate;

					break;
				}
			}
		}

		tmp_num = num;

		// numRead = fread(BD3_data, sizeof(char), num, fpData);
		// if (BD3_ADNumber == 16)
		// {
		// 	for (int i = 0; i < numRead; i++)
		// 	{
		// 		BD3_data[i] -= 7;//4bit����ʱ����Ҫ������ֵΪ0����15��Ϊ����ֵΪ-7����8��
		// 		if (BD3_data[i] > 8)
		// 			BD3_data[i] = 8;//����ֵ����8������������Ϊ8
		// 	}
		// }
	/*	for (int i = 0; i < numRead; i++)
		{
			if (data[i] > 15)
				data[i] = 15;
		}*/
		// if (numRead<num)
		// {
		// 	printf("\n reach the end of the data file, process end!");
		// 	break;
		// }
		
		TIC_DIF = BD3_TIC_CNTR + 1 - (BD3_TIC_RT_CNTR + num);
		BD3_receiver_init_num = num = TIC_DIF < BD3_ACC_INT_PERIOD ? TIC_DIF : BD3_ACC_INT_PERIOD;

		BD3_data = BD3_receiver_buf.buf_current_ptr;
		//memcpy(BD3_data, BD3_receiver_buf.buf_current_ptr, tmp_num);

		//if (BD3_ADNumber == 16)
		//{
		//	for (int i = 0; i < tmp_num; i++)
		//	{
		//		BD3_data[i] -= 7;//4bit����ʱ����Ҫ������ֵΪ0����15��Ϊ����ֵΪ-7����8��
		//		if (BD3_data[i] > 8)
		//			BD3_data[i] = 8;//����ֵ����8������������Ϊ8
		//	}
		//}

      //CTC1.CTimeCounterReset();
		BD3_correlatorProcess(BD3_data, tmp_num);

		BD3_receiver_buf.buf_remain_len -= tmp_num;
		BD3_receiver_buf.buf_current_ptr += tmp_num;

		BD3_Sim_BDD2B1_Interrupt();				// simulate interrupt processing
#endif
		//////////////////////////////////////////////////////////////////////////
		for (char ch = 0; ch <= BD3_chmax; ch++)
		{

			if ((BD3_chan[ch].prn >= 1) && (BD3_chan[ch].prn <= 5))//D2
		{// from ln
				if (BD3_chan[ch].frame_ready_1 == 1)
			{
				BD3_resolution(ch);   // decode the navigation message
				if (BD3_chan[ch].frame_ready == 1 && fabs(BD3_gps_eph[BD3_chan[ch].prn].toe - BD3_m_time[1])>3600.0)  //modified by daicy for eph decoding                    
				{
					BD3_navmessd2(BD3_chan[ch].prn, ch);
					BD3_chan[ch].frame_ready = 0;     // for this channel
				}
				BD3_chan[ch].frame_ready_1 = 0;
			}
		
		} 
			if (BD3_chan[ch].prn >= 6)//D1
		{//from jh
				if (BD3_chan[ch].frame_ready == 1 && fabs(BD3_gps_eph[BD3_chan[ch].prn].toe - BD3_m_time[1])>3600.0)  //modified by daicy for eph decoding   //d pream_d1 �� t_count + n_frame*1500 - offset == 900
			{
					BD3_navmessd1(BD3_chan[ch].prn, ch);   // decode the navigation message
					BD3_chan[ch].frame_ready = 0;     // for this channel
			}
		}
		
		}

		if (BD3_sec_flag == 1)
		{
			BD3_almanac_flag = 0;
			
			BD3_thetime++;

			BD3_clock_tow = (++BD3_clock_tow) % 604800L;
			BD3_time_on++;
			BD3_sec_flag = 0;

		} 
		if (BD3_nav_flag == 1)
		{

			if (_kbhit()) BD3_key = _getch();
			BD3_nav_fix();

			BD3_nav_flag = 0;
			BD3_display();

			BD3_send_sim_res_socket();
		}
		

		if (BD3_min_flag == 1)
		{
			BD3_min_flag = 0;
		}

#ifdef DEMO_CONSOLE
			sharemem_write(1);
#endif

		
			if (BD3_key == 'p' || BD3_key == 'P')
		{
				BD3_display_page++;
				BD3_display_page = BD3_display_page % 4;

		}
	}
}

void BD3_sim_end()
{

#ifndef REAL_TIME
	BD3_shutCorrelator();
#endif

#ifndef REAL_TIME
	// �ͷŶ�������
	BD3_freeSignalSearch();
#endif
	BD3_write_prn();
	// Update the Almanac Data file
	if (BD3_almanac_valid == 1)  BD3_write_almanac();
	//  Update the Ephemeris Data file
	BD3_write_ephemeris();
	//  Update the ionospheric model and UTC parameters
	BD3_write_ion_utc();
	//    Update the curloc file for the next run

	{
		BD3_out = fopen(BD3_curloc_file, "w+");//out=fopen("curloc.dat","w+");
		fprintf(BD3_out, "latitude  %f\n", BD3_rec_pos_llh.lat*BD3_r_to_d);
		fprintf(BD3_out, "longitude %f\n", BD3_rec_pos_llh.lon*BD3_r_to_d);
		fprintf(BD3_out, "hae       %f\n", BD3_rec_pos_llh.hae);
		fclose(BD3_out);
	}
	_fcloseall();//closes all open streams 


	::PostMessage(MainWnd, WM_SIMULATION_STOP, 0, 0);
}

void BD3_send_sim_res_socket()
{
	/*���ջ���ʶ*/
	send_res_BD3.receiver_id = RECEIVER_BD3;

	//�������㷨����ʱ��ֻ����һ��
	//send_res_BD1.var1 = 0;

	send_res_BD3.dim = m*n;

	//Ȩֵȡ��ǰ��������ݵ����һ��
	if (CONFIG_STAP == (ANTI_CONFIG_SEL & sel))
	{
		cudaMemcpy(send_res_BD3.var2_real, STAP_array_matrix_R_inver[0],
			send_res_BD3.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);

		//��ʼ��ʱ��Ϊ0
		//memset(send_res_L1.var2_image, 0, send_res_L1.dim * sizeof(SIGNAL_TYPE));
	}
	else if (CONFIG_BEAM == (ANTI_CONFIG_SEL & sel))
	{
		cudaMemcpy(send_res_BD3.var2_real, STAP_array_matrix_R_inver[0],
			send_res_BD3.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);

		cudaMemcpy(send_res_BD3.var2_image, STAP_array_matrix_R_inver[0] +
			send_res_BD3.dim, send_res_BD3.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);
	}
	else
	{
		//��ʼ��ʱ��Ϊ0
		/*memset(send_res_L1.var2_real, 0, send_res_L1.dim * sizeof(SIGNAL_TYPE));
		memset(send_res_L1.var2_image, 0, send_res_L1.dim * sizeof(SIGNAL_TYPE));*/
	}

	{
		int tmp_count = sameple_rate *1e-3;
		send_res_BD3.var3 = 0;
		for (int i = 0; i <tmp_count; i++)
		{
			send_res_BD3.var3 += ((SIMRESVAR)BD3_receiver_buf.buf_ptr[i]) *
				((SIMRESVAR)BD3_receiver_buf.buf_ptr[i]);
		}
		send_res_BD3.var3 /= tmp_count;
		send_res_BD3.var3 = 10 * log(send_res_BD3.var3);
	}

	for (int i = 0; i < SAT_NUM; i++)
	{
		send_res_BD3.var4[i] = BD3_chan[i].i_prompt;
		send_res_BD3.var5[i] = BD3_chan[i].CNo;
		send_res_BD3.var6[i] = ((long long)(BD3_chan[i].phaseLockDetector + 0.5)) & 0xffffffffUL;

		send_res_BD3.var7_az[i] = BD3_chan[i].az*BD3_r_to_d;
		send_res_BD3.var7_el[i] = BD3_chan[i].el*BD3_r_to_d;
		send_res_BD3.var8_prn[i] = BD3_chan[i].prn;

		send_res_BD3.var10_x[i] = BD3_track_sat[i + 1].x;
		send_res_BD3.var10_y[i] = BD3_track_sat[i + 1].y;
		send_res_BD3.var10_z[i] = BD3_track_sat[i + 1].z;
	}
	send_res_BD3.var9_GDOP = BD3_gdop;
	send_res_BD3.var9_HDOP = BD3_hdop;
	send_res_BD3.var9_TDOP = BD3_vdop;
	send_res_BD3.var9_VDOP = BD3_tdop;
	send_res_BD3.var9_PDOP = BD3_pdop;

	send_res_BD3.var11_x = navout.X;
	send_res_BD3.var11_y = navout.Y;
	send_res_BD3.var11_z = navout.Z;

	send_res_BD3.var12_he = navout.height;
	send_res_BD3.var12_la[0] = BD3_cur_lat.deg, send_res_BD3.var12_la[1] = abs(BD3_cur_lat.min), send_res_BD3.var12_la[2] = fabs(BD3_cur_lat.sec);
	send_res_BD3.var12_lo[0] = BD3_cur_long.deg, send_res_BD3.var12_lo[1] = abs(BD3_cur_long.min), send_res_BD3.var12_lo[2] = fabs(BD3_cur_long.sec);

	send_res_BD3.var13[0] = BD3_rpvt.xv, send_res_BD3.var13[1] = BD3_rpvt.yv, send_res_BD3.var13[2] = BD3_rpvt.zv;
	send_res_BD3.var14 = BD3_clock_offset;

	localtime_s(&local_time, &BD3_thetime);
	send_res_BD3.time_y = local_time.tm_year;
	send_res_BD3.time_mon = local_time.tm_mon;
	send_res_BD3.time_d = local_time.tm_mday;
	send_res_BD3.time_h = local_time.tm_hour;
	send_res_BD3.time_min = local_time.tm_min;
	send_res_BD3.time_s = local_time.tm_sec;

	send_res_BD3.var16 = BD3_gps_week % 1024;
	send_res_BD3.var17 = BD3_clock_tow;


	sendto(socket_client, (char *)&send_res_BD3, sizeof(SimRes), 0, (sockaddr *)&socket_sin, socket_len);
}

/*******************************************************************************
FUNCTION display()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function displays the current status of the receiver on the
	computer screen.  It is called when there is nothing else to do
��������ڵ�����Ļ����ʾ��ǰ���ջ���״̬����û�������ǿ�����ʱ�ű�����
*******************************************************************************/
void BD3_display(void)
{
	
	char ch;

	printf("%s", ctime(&BD3_thetime));
	printf("TOW  %6ld\n", BD3_clock_tow);
	printf("meas time %f  error %f \n", BD3_m_time[1], BD3_m_error);
	BD3_cur_lat.deg = int(BD3_rec_pos_llh.lat*BD3_r_to_d);
	BD3_cur_lat.min = int((BD3_rec_pos_llh.lat*BD3_r_to_d - BD3_cur_lat.deg) * 60);
	BD3_cur_lat.sec = float((BD3_rec_pos_llh.lat*BD3_r_to_d - BD3_cur_lat.deg - BD3_cur_lat.min / 60.)*3600.);
	BD3_cur_long.deg = int(BD3_rec_pos_llh.lon*BD3_r_to_d);
	BD3_cur_long.min = int((BD3_rec_pos_llh.lon*BD3_r_to_d - BD3_cur_long.deg) * 60);
	BD3_cur_long.sec = float((BD3_rec_pos_llh.lon*BD3_r_to_d - BD3_cur_long.deg - BD3_cur_long.min / 60.)*3600.);
	printf("   latitude    longitude          HAE      clock error (ppm)\n");
	printf("  %4d:%2d:%5.2f  %4d:%2d:%5.2f  %10.2f  %f\n",
		BD3_cur_lat.deg, abs(BD3_cur_lat.min), fabs(BD3_cur_lat.sec), BD3_cur_long.deg, abs(BD3_cur_long.min),
		fabs(BD3_cur_long.sec), BD3_rec_pos_llh.hae, BD3_clock_offset);
	printf(" Speed      vx		vy	  vz      Heading      TIC_dt\n");
	printf(" %lf   %lf   %lf   %lf   %lf   %lf\n", BD3_speed, BD3_rpvt.xv, BD3_rpvt.yv, BD3_rpvt.zv, BD3_heading*BD3_r_to_d, BD3_TIC_dt);
	printf("   \n");

	printf("tracking %2d status %1d almanac valid %1d gps week %4d\n",
		BD3_n_track, BD3_status, BD3_almanac_valid, BD3_gps_week % 1024);

	if (BD3_display_page == 0)
	{

		printf(" ch prn state az  el  doppler  t_count n_frame sfid ura page CNo  PL\n");
		for (ch = 0; ch <= BD3_chmax; ch++)
		{
			printf(" %2d %2d  %2d  %4.0f  %3.0f   %7.1f   %4d  %4d  %2d  %3d  %3d   %4.1f  %1d\n",
				ch, BD3_chan[ch].prn, BD3_chan[ch].state,
				BD3_chan[ch].az*BD3_r_to_d, BD3_chan[ch].el*BD3_r_to_d,
				BD3_chan[ch].vDoppler, BD3_chan[ch].t_count, BD3_chan[ch].n_frame, BD3_chan[ch].sfid,
				BD3_gps_eph[BD3_chan[ch].prn].ura, BD3_chan[ch].page5, BD3_chan[ch].CNo, (long)((long long)BD3_chan[ch].phaseLockDetector + 0.5));
		

			chinfo[ch].prn = BD3_chan[ch].prn;
			chinfo[ch].state = BD3_chan[ch].state;
			chinfo[ch].az = BD3_chan[ch].az;
			chinfo[ch].el = BD3_chan[ch].el;
			chinfo[ch].doppler = BD3_chan[ch].doppler;
			chinfo[ch].t_count = BD3_chan[ch].t_count;
			chinfo[ch].n_frames = BD3_chan[ch].n_frame;
			chinfo[ch].sfid = BD3_chan[ch].sfid;
			chinfo[ch].ura = BD3_gps_eph[BD3_chan[ch].prn].ura;
			chinfo[ch].page = BD3_chan[ch].page5;
			chinfo[ch].CN0 = BD3_chan[ch].CNo;

			CNR[ch] = chinfo[ch].CN0;


			corrpeak[ch].i_early = BD3_chan[ch].i_early;

			corrpeak[ch].i_prompt = BD3_chan[ch].i_prompt;//��ط�ֵ

			corrpeak[ch].i_late = BD3_chan[ch].i_late;

		}

		/*BEAM*/
		if (CONFIG_BEAM == (ANTI_CONFIG_SEL & sel))
		{
			bool is_EL_AZ_changed = false;

			int tmp_AZ_sj;
			int tmp_EL_si;

			for (ch = 0; ch <= BD3_chmax; ch++)
			{
				//�޸�ȫ�ֵĸ����Ǻͷ�λ��
				tmp_AZ_sj = BD3_chan[ch].az*BD3_r_to_d;
				tmp_EL_si = 90 - BD3_chan[ch].el*BD3_r_to_d;
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

		for (ch = 1; ch <= BD3_chmax + 1; ch++)
		{

			printf("   %2d    %lf    %lf    %lf\n",
				BD3_chan[BD3_tr_ch[ch]].prn, BD3_track_sat[ch].x, BD3_track_sat[ch].y, BD3_track_sat[ch].z);
		}

		printf(" GDOP=%6.3f  HDOP=%6.3f  VDOP=%6.3f  TDOP=%6.3f PDOP=%6.3f\n\n", BD3_gdop, BD3_hdop, BD3_vdop, BD3_tdop, BD3_pdop);
	}
	else if (BD3_display_page == 1)
	{
		printf(  " ch prn state TLM      TOW  Health  Valid  TOW_sync offset\n");
		for (ch = 0; ch <= BD3_chmax; ch++)
		{
			printf(" %2d %2d  %2d  %6ld   %6ld   %2d     %2d     %2d   %4d\n",
				ch, BD3_chan[ch].prn, BD3_chan[ch].state, BD3_chan[ch].TLM, BD3_chan[ch].TOW,
				BD3_gps_eph[BD3_chan[ch].prn].health, BD3_gps_eph[BD3_chan[ch].prn].valid, BD3_chan[ch].tow_sync,
				BD3_chan[ch].offset);
		}
	}
	else if (BD3_display_page == 2)
	{
		printf(" ch prn state n_freq az  el        tropo        iono\n");
		for (ch = 0; ch <= BD3_chmax; ch++)
		{
			printf(" %2d %2d  %2d  %4.0f  %3.0f   %10.4lf   %10.4lf\n",
				ch, BD3_chan[ch].prn, BD3_chan[ch].state,
				BD3_xyz[BD3_chan[ch].prn].azimuth*BD3_r_to_d, BD3_xyz[BD3_chan[ch].prn].elevation*BD3_r_to_d,
				BD3_chan[ch].Tropo*BD3_SPEEDOFLIGHT, BD3_chan[ch].Iono*BD3_SPEEDOFLIGHT);
		}
	}
	else if (BD3_display_page == 3)
	{
		printf(" ch prn state      Pseudorange     delta Pseudorange\n");
		for (ch = 0; ch <= BD3_chmax; ch++)
		{
			printf(" %2d %2d  %2d  %20.10lf   %15.10lf\n",
				ch, BD3_chan[ch].prn, BD3_chan[ch].state, BD3_chan[ch].Pr, BD3_chan[ch].dPr);
		}
	}
	else if (BD3_display_page == 4)// can be used for debugging purposes
	{
	}

	//::PostMessage(ChannelWnd, WM_CHANNEL_INFORMATION, (WPARAM)chinfo, (LPARAM)12);

	sprintf(navout.lat, "%4d:%2d:%5.2f", BD3_cur_lat.deg, abs(BD3_cur_lat.min), fabs(BD3_cur_lat.sec));
	sprintf(navout.lon, "%4d:%2d:%5.2f", BD3_cur_long.deg, abs(BD3_cur_long.min), fabs(BD3_cur_long.sec));
	navout.height = BD3_rec_pos_llh.hae;
	navout.gdop = BD3_gdop;
	navout.hdop = BD3_hdop;
	navout.vdop = BD3_vdop;
	navout.tdop = BD3_tdop;
	strcpy(navout.time, ctime(&BD3_thetime));
	navout.ve = BD3_receiver.vel.east;
	navout.vn = BD3_receiver.vel.north;
	navout.vu = BD3_receiver.vel.up;

	BD3_XYZ wgs;
	
	wgs = BD3_llh_to_ecef(BD3_rec_pos_llh);
	navout.X = wgs.x;
	navout.Y = wgs.y;
	navout.Z = wgs.z;

    printf("X: %lf, Y: %lf, Z: %lf.\n", navout.X, navout.Y, navout.Z);
	/*::PostMessage(NavOutWnd, WM_NAVIGATION_OUT, (WPARAM)&navout, 0);
	::PostMessage(CNRatioWnd,WM_CNRATIO, (WPARAM)CNR, 0);
	::PostMessage(CPWnd, WM_CP, (WPARAM)corrpeak, (LPARAM)3);*/

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
	����������Լ��ĺ��������Ŀǰ��IRQ0�жϷ�������ϵ��ж��������洢��һ��ȫ�ֱ�����
	���ҽ��ڳ���ִ�е�������°�װ��IRQ0ͨ���ı�洢��8259�жϴ������е��ж��������ʹ��

*******************************************************************************/
/*******************************************************************************
FUNCTION Interrupt_Remove()
RETURNS  None.

PARAMETERS None.

PURPOSE
	This function removes the custom interrupt vector from the vector
	table and restores the previous vector.

*******************************************************************************/

/*******************************************************************************
FUNCTION GPS_Interrupt()

RETURNS  None.

PARAMETERS None.

PURPOSE
	This function replaces the current IRQ0 Interrupt service routine with
	our GPS function which will perform the acquisition - tracking functions
������������ǵ�ִ�в�����ٵ�GPS����������Ŀǰ��IRQ0�жϷ������
*******************************************************************************/
void BD3_Sim_BDD2B1_Interrupt()
{

	unsigned int add;
	char ch;
	double carrierPhase;

/*	to_gps(0x80,0);             // tell 2021 to latch the correllators
	a_missed=from_gps(0x83);    // did we miss any corellation data
	astat=from_gps(0x82);       // get info on what channels have data ready
	for (ch=0;ch<=chmax;ch++)
	{
		if (astat & test[ch])
		{
			add=0x84+(ch<<2);
			chan[ch].i_dith=from_gps(add);    // inphase dither
			add++;
			chan[ch].q_dith=from_gps(add);    // quadrature dither
			add++;
			chan[ch].i_prompt=from_gps(add);  // inphase prompt
			add++;
			chan[ch].q_prompt=from_gps(add);  // quadrature prompt
			//	  ch_accum_reset(ch);
			if (a_missed & test[ch])
			{
				chan[ch].missed++;
				ch_accum_reset(ch);
			}
		}
	}
*/  // remmed by Ning, rewrite as below


	for (ch = 0; ch <= BD3_chmax; ch++)
	{
	
		if (BD3_correlator[ch].ready == 1)
		{
			BD3_ReadAccumData(ch);
			
			BD3_correlator[ch].ready = 0;
			
			switch (BD3_chan[ch].state)
			{
			case BD3_acquisition:
				BD3_ch_acq(ch);
				break;
			case BD3_confirm:
				break;
			case BD3_pull_in:
				if ((1 <= BD3_chan[ch].prn) && (BD3_chan[ch].prn <= 5))	BD3_ch_pull_in(ch);
				else										BD3_ch_pull_in_D1(ch);
				break;
			case BD3_track:
				if ((1 <= BD3_chan[ch].prn) && (BD3_chan[ch].prn <= 5))	BD3_ch_track(ch);
				else										BD3_ch_track_D1(ch);
				break;
			}
		}
	}

	if (BD3_TIC_OCCUR_FLAG == 1)//100ms			
	{
		BD3_tic_count = (++BD3_tic_count) % 10;
		if (BD3_tic_count == 0) BD3_sec_flag = 1;		// one second has passed

		BD3_hms_count = (++BD3_hms_count) % 600;
		if (BD3_hms_count == 0) BD3_min_flag = 1;		// one minute has passed

		BD3_nav_count = (++BD3_nav_count) % BD3_nav_tic;
		if (BD3_nav_count == 0)
			BD3_nav_flag = 1;		// ���㵼�����ʱ�䵽,1s

		BD3_TIC_sum += BD3_TIC_CNTR + 1;
		add=1;

		for (ch = 0; ch <= BD3_chmax; ch++)
		{
			if (BD3_chan[ch].state != BD3_track)
			{
				continue;
			}
			
			carrierPhase = BD3_readCarrierPhase(ch);
			BD3_chan[ch].cycle_sum += (long)((long long)carrierPhase);
			BD3_chan[ch].carr_dco_phase = fmod(carrierPhase, 1.0);
		}

		if (BD3_nav_count == 0)
		{
			for (ch = 0; ch <= BD3_chmax; ch++)
			{
				if (BD3_chan[ch].state != BD3_track)
				{
					continue;
				}

				BD3_chan[ch].codePhase = BD3_readCodePhase(ch);
				BD3_chan[ch].epoch = BD3_readEpoch(ch);

				BD3_chan[ch].meas_bit_time = BD3_chan[ch].tr_bit_time;
				

				BD3_chan[ch].doppler = BD3_chan[ch].fc;
				
				BD3_chan[ch].carrier_counter = BD3_chan[ch].cycle_sum;
				BD3_chan[ch].cycle_sum = 0;
				BD3_chan[ch].d_carr_phase = BD3_chan[ch].carr_dco_phase - BD3_chan[ch].old_carr_dco_phase;
				BD3_chan[ch].old_carr_dco_phase = BD3_chan[ch].carr_dco_phase;
			}
			
			BD3_i_TIC_dt = BD3_TIC_sum;

			BD3_TIC_sum = 0;
		}
		BD3_TIC_OCCUR_FLAG = 0;
		{
			BD3_ch_alloc();
		}
	}

// reset the interrupt
//	 outportb(0x20,0x20);  // remmed by Ning LUO, only used for real-time
}

void BD3_initTrackLoopPar()
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

	BD3_DLLc1 = pow((8 * DLLkesi*BDLL / (1 + 4 * DLLkesi*DLLkesi)), 2.0) / (DLLK1*DLLK2);
	BD3_DLLc2 = 16 * DLLkesi*DLLkesi*BDLL / (1 + 4 * DLLkesi*DLLkesi) / DLLK1 / DLLK2;


	BPLL = 15.0;	
	PLLomega0 = BPLL/0.78445;
	BD3_PLLc0 = pow(PLLomega0, 3.0);
	BD3_PLLc1 = 1.1*PLLomega0*PLLomega0;
	BD3_PLLc2 = 2.4*PLLomega0;
	FLLkesi = 0.707;
	BFLL = 10.0;					
	//changed by Ning Luo in Sept/04, 5.0Hz->10Hz

	BD3_FLLa1 = pow((8 * FLLkesi*BFLL / (1 + 4 * FLLkesi*FLLkesi)), 2.0);
	BD3_FLLa2 = 16 * FLLkesi*FLLkesi*BFLL / (1 + 4 * FLLkesi*FLLkesi);

	for (ch = 0; ch <= BD3_chmax; ch++)
	{
		BD3_chan[ch].BitSyncIndex = 0;		// ѭ����ַ������0  ���ڱ���ͬ��
		BD3_chan[ch].IQBufSize = 0;			// IQ��������С��0
		for (i = 0; i<BD3_BUFSIZE; i++)			// ������ű仯������ #define BUFSIZE	1000
		{
			BD3_chan[ch].signBuf[i] = 0;
		}
		for (i=0;i<20;i++)			
		{
			BD3_chan[ch].kCell[i] = 0;
		}
	}
}

void BD3_hot_ch_alloc()
{	
	char i;
	short allocFlag;
	
	BD3_read_prn();

	for (i = 0; i <= BD3_chmax; i++)
	{
		// ����ŵ��Ѿ���⵽���ź�/��ռ�ã����������ŵ��ļ��
		if (BD3_chan[i].state != BD3_CHN_OFF)//��ʼ��û����ΪOFF
		{
			continue;
		}
		
		// Ϊ�����ŵ�ѡ��һ������
		// check all satellites
		allocFlag = 0;			
		
		if (BD3_last_prn[i]>0)
		{
			allocFlag = 1;
			BD3_svStruct[i].state = BD3_USING;
			BD3_chan[i].prn = BD3_last_prn[i];
			BD3_chan[i].state = BD3_acquisition;
			BD3_svStruct[i].NumToTryAcq = 20;
			BD3_svStruct[i].NumBTWEachTrial = 1000;
		
			BD3_correlator[i].state = BD3_CHN_OFF;
			BD3_correlator[i].ready = 1;
			BD3_correlator[i].sv = BD3_last_prn[i];
		}				
		
	}

}
void BD3_ch_alloc()
{
	char i,j;
	short allocFlag;

	// ���Almanac�Ƿ���Ч
	BD3_almanac_valid = 1;
	for (i = BD3_MinSvNumber; i <= BD3_MaxSvNumber; i++)
	{
		BD3_xyz[i] = BD3_satfind(i);//�ڴ�ch_alloc����֮ǰ��time(&thetime);
		if (BD3_gps_alm[i].inc>0.0 && BD3_gps_alm[i].week != BD3_gps_week % 1024)//�ж�������Ч��
		{
			BD3_almanac_valid = 0;
			break;
		}
	}
	if (BD3_al0 == 0.0 && BD3_b0 == 0.0) BD3_almanac_valid = 0;

	for (i = BD3_MinSvNumber; i <= BD3_MaxSvNumber; i++)
	{
		if (BD3_svStruct[i].state == BD3_HOLD)

		{
			if (BD3_svStruct[i].NumToTryAcq == 0)
			{
				BD3_svStruct[i].NumToTryAcq = 1;
				BD3_svStruct[i].NumBTWEachTrial = BD3_ReAcqTime;
				BD3_svStruct[i].maxf = BD3_MAXF;
				BD3_svStruct[i].minf = BD3_MINF;
			}
			BD3_svStruct[i].undetectedCounter++;
			if (BD3_svStruct[i].undetectedCounter>BD3_svStruct[i].NumBTWEachTrial)
			{
				BD3_svStruct[i].state = BD3_AVAILABLE;
				BD3_svStruct[i].undetectedCounter = 0;
			}
		}
	}

	for (i = 0; i <= BD3_chmax; i++)
	{
		if (BD3_chan[i].state != BD3_CHN_OFF)
		{
			continue;
		}
		

		allocFlag = 0;
		for (j = BD3_MinSvNumber; j <= BD3_MaxSvNumber; j++)
		{
			if (BD3_almanac_valid == 1)
			{
				if (BD3_xyz[j].elevation > BD3_mask_angle &&
					BD3_gps_alm[j].health == 0 &&
					BD3_gps_alm[j].ety != 0.00 &&
					BD3_svStruct[j].state == BD3_AVAILABLE)
				{
					allocFlag = 1;
					BD3_svStruct[j].state = BD3_USING;
					BD3_chan[i].prn = j;
					BD3_chan[i].state = BD3_acquisition;
					BD3_svStruct[j].NumToTryAcq = 20;
					BD3_svStruct[j].NumBTWEachTrial = 1000;
					BD3_correlator[i].state = BD3_CHN_OFF;
					BD3_correlator[i].ready = 1;
					BD3_correlator[i].sv = j;
					break;
				}
			}
			else
			{
				if (BD3_svStruct[j].state == BD3_AVAILABLE)
				{
					allocFlag = 1;
					BD3_svStruct[j].state = BD3_USING;
					BD3_chan[i].prn = j;
					BD3_chan[i].state = BD3_acquisition;
					if (BD3_svStruct[j].NumToTryAcq == 0)
					{
						BD3_svStruct[j].NumToTryAcq = 1;
						BD3_svStruct[j].NumBTWEachTrial = BD3_ReAcqTime;
						BD3_svStruct[j].maxf = BD3_MAXF;
						BD3_svStruct[j].minf = BD3_MINF;
					}
					BD3_correlator[i].state = BD3_CHN_OFF;
					BD3_correlator[i].ready = 1;
					BD3_correlator[i].sv = j;
					break;
				}
			}
		}
		if (allocFlag == 0) break;
    }
}


char BD3_first_flag = 1;
// linear FFT detector
void BD3_ch_acq(char ch)
{
	double Rmax, tau, doppler;
	long dataLength;
	int flag;
	long bufOffset;
	long numFreqBin, i;
	long nTrial;
	double freqOffset[100];
	//sta-tic char first_flag = 1;
	double *pBuf;
	char *pData;

	//sta-tic double BUFTABLE[2]={1.0, -1.0};

	pBuf = &BD3_buffer[ch][BD3_globalBufLength[ch]];
	pData = BD3_data;

	for (i = 0; i<BD3_correlatorDataLength; i++)
	{
		//*pBuf++ = BUFTABLE[*pData++];
		*pBuf++ = *pData++;
	}

	BD3_globalBufLength[ch] += BD3_correlatorDataLength;

	#ifndef REAL_TIME  // ��ʾ����FFT���,������Ļ����������ǿ����1
	BD3_correlator[ch].ready = 1;
	#endif
	
	// there is no enough data for fft acquisition   һ��ֻ��0.5��������ݣ�������Ҫ10����
	if (BD3_globalBufLength[ch] < BD3_DETECTSIZE)
		return;
	// to check if data in buffer is overflow or abnormal
	if (BD3_globalBufLength[ch] > (BD3_DETECTSIZE + 30000))
	{
		printf("computation abnormal, system exits");
		exit(0);
	}

	// ���������ѹ�����������ǰ������ݶ������Ա�֤���ڼ������ݳ���������DETECTSIZE
	bufOffset = BD3_globalBufLength[ch] - BD3_DETECTSIZE;
	dataLength = BD3_DETECTSIZE;
	numFreqBin = (long)((long long)((BD3_svStruct[BD3_chan[ch].prn].maxf - BD3_svStruct[BD3_chan[ch].prn].minf) / BD3_DELTF + 1.5));// Ҫ������Ƶ������
	//����Ƶ�ʷ�Χ-10k-----+10k , ��������666Hz
	for(i=0;i<numFreqBin;i++) 
		freqOffset[i] = i*BD3_DELTF + BD3_svStruct[BD3_chan[ch].prn].minf;	// ������Χ

	if (BD3_first_flag == 1)
	{
		double threshold = 0.0;
		//Ϊʲô��14���ǣ���Ϊû��������ǣ����Բ���������ֻ�ܵõ����
		flag = BD3_acqCA(&BD3_buffer[ch][bufOffset], dataLength, BD3_caTable, 14, BD3_ACQ_SAMPLING_FREQ, freqOffset, numFreqBin,//flag=acqCA(&buffer[ch][bufOffset],dataLength, caTable, 37, SAMPLING_FREQ,freqOffset,numFreqBin,
			BD3_CARRIER_FREQ, &tau, &doppler, &nTrial, &Rmax, &threshold);
		BD3_threshold_sig_search = Rmax*1.5f; //���ֵ��1.5����������
		BD3_first_flag = 0;
	}
	flag = BD3_acqCA(&BD3_buffer[ch][bufOffset], dataLength, BD3_caTable, BD3_chan[ch].prn, BD3_ACQ_SAMPLING_FREQ, freqOffset, numFreqBin,
		BD3_CARRIER_FREQ, &tau, &doppler, &nTrial, &Rmax, &BD3_threshold_sig_search);
// 	if (chan[ch].prn == 6)
// 	{
// 		tau = 0.6838e-3;
// 		doppler = 1333;
// 	}
	// ���β�����̽�����������ݻ���
	BD3_globalBufLength[ch] = 0;
	if (flag == BD3_DETECTED)
	{
		//����⵽���ǵ�ʱ��д���ز���ncoƵ���֣�д������λ�֣�����ͨ������
		BD3_writeCodePhase(ch, tau);
		BD3_writeCodeFreq(ch, 1.0);

		BD3_writeCarrierFreq(ch, BD3_carrier_ref + doppler);
#ifndef REAL_TIME
		BD3_correlator[ch].state = BD3_CHN_ON;//�����˾Ͱ��������
		//��ch_alloc��correlator[i].state = CHN_OFF;
		//��ch_track��if(pChan->CNo<30) correlator[ch].state = CHN_OFF;
		//��initCorrelator��correlator[i].state = CHN_OFF;

		BD3_correlator[ch].ready = 0;//��������ˣ���ͨ���ͽ����٣��ڸ���������������1������ֱ�1
		//�������������pCorr->ready = 1;//�Ѿ�������һ����������ж�
		//ֻ��ch_acq����ʹcorrelator[ch].state = CHN_ON;��������򿪣����������ÿ��1�����pCorr->ready = 1�Ӷ��ܽ���Sim_GPS_Interrupt��case�
		//����ʱchan[ch].state = pull_in;�Ӷ��ܽ���ǣ��
#endif
		// �ŵ����벶���,��ʼ����
		BD3_chan[ch].state = BD3_pull_in;
		BD3_chan[ch].cLoopS1 = doppler;//��ǣ���ö�����
		BD3_chan[ch].cLoopS0 = 0;
		BD3_chan[ch].dllS0 = 0.0;
		BD3_chan[ch].dllS1 = 0.0;

		BD3_chan[ch].FLLIndex = 1;
		BD3_chan[ch].FLLFlag = 1;
		BD3_chan[ch].ch_time = 0;
		BD3_chan[ch].freqErr = 0.0;
		BD3_chan[ch].fc = doppler;
	}
	else if (flag == BD3_UNDETECTED)//����ⲻ����ʱ����������ΪHOLD
	{

		BD3_svStruct[BD3_chan[ch].prn].NumToTryAcq--;
		{
			BD3_chan[ch].state = BD3_CHN_OFF;
			BD3_correlator[ch].state = BD3_CHN_OFF;
			BD3_correlator[ch].ready = 1;
			BD3_svStruct[BD3_chan[ch].prn].state = BD3_HOLD;
			BD3_svStruct[BD3_chan[ch].prn].undetectedCounter = 0;

			BD3_chan[ch].prn = 0;
		}
	}


}



void BD3_ch_pull_in(char ch)
{
	double sER, sEI, sLR, sLI, sPR, sPI;

	double codeErr, T, phaseErr, freqErr;
	double *dllS0, *dllS1;
	double *cLoopS0, *cLoopS1;
	double cross, dot, phaseDiff;
	
	BD3_chan[ch].ch_time++;
	if (BD3_chan[ch].ch_time == 1)
	{
		return;
	}

	sER = BD3_chan[ch].i_early;
	sEI = BD3_chan[ch].q_early;
	sLR = BD3_chan[ch].i_late;
	sLI = BD3_chan[ch].q_late;
	sPR = BD3_chan[ch].i_prompt;
	sPI = BD3_chan[ch].q_prompt;

	// below added by Ning Luo in Sept/04
	if (BD3_chan[ch].ch_time == 2)
	{
		BD3_chan[ch].freqErr = 0;
		BD3_chan[ch].i_old = BD3_chan[ch].i_prompt;
		BD3_chan[ch].q_old = BD3_chan[ch].q_prompt;
		BD3_minFreqErr[ch] = BD3_M_PI;
		BD3_maxFreqErr[ch] = -BD3_M_PI;
		return;
	}

	if (BD3_chan[ch].ch_time<BD3_FINE_FREQ_RESOLUTION + 2)
	{
		cross = BD3_chan[ch].i_old*sPI - BD3_chan[ch].q_old*sPR;
		dot = BD3_chan[ch].i_old*sPR + BD3_chan[ch].q_old*sPI;
		phaseDiff = atan2(cross,dot);

		if (phaseDiff>BD3_maxFreqErr[ch]) BD3_maxFreqErr[ch] = phaseDiff;
		if (phaseDiff<BD3_minFreqErr[ch]) BD3_minFreqErr[ch] = phaseDiff;
		BD3_chan[ch].freqErr += phaseDiff;
		BD3_chan[ch].i_old = BD3_chan[ch].i_prompt;
		BD3_chan[ch].q_old = BD3_chan[ch].q_prompt;
	}
	else if (BD3_chan[ch].ch_time == BD3_FINE_FREQ_RESOLUTION + 2)
	{

        T = 1e-3;
		freqErr = (BD3_chan[ch].freqErr - BD3_maxFreqErr[ch] - BD3_minFreqErr[ch]) / (BD3_FINE_FREQ_RESOLUTION - 3.0);
		freqErr = freqErr / (2 * BD3_M_PI*T);
		BD3_chan[ch].fc += freqErr;
		BD3_chan[ch].cLoopS1 = BD3_chan[ch].fc;
		BD3_chan[ch].carrier_freq = BD3_carrier_ref + BD3_chan[ch].fc;
		BD3_writeCarrierFreq(ch, BD3_chan[ch].carrier_freq);

	}
	// above added by Ning Luo in Sept/04

	else if (BD3_chan[ch].ch_time>BD3_FINE_FREQ_RESOLUTION + 2)
	{
		/////////////////////////////////////////////////////////////////////////////////
		T = 1e-3;			
		//codeErr = (sER*sPR+sEI*sPI)/(sER*sER+sPR*sPR+sEI*sEI+sPI*sPI)*(DLLdT/(2.0*2.046e6));
		codeErr = (sER*sPR + sEI*sPI) / (sER*sER + sPR*sPR + sEI*sEI + sPI*sPI)*(BD3_DLLdT / (2.0*10.23e6));//done
		dllS0 = &(BD3_chan[ch].dllS0);
		dllS1 = &(BD3_chan[ch].dllS1);
		*dllS0 += BD3_DLLc1*T*codeErr;
        //*dllS1 = *dllS0 + DLLc2*codeErr+chan[ch].fc/1561.098e6;	
		*dllS1 = *dllS0 + BD3_DLLc2*codeErr + BD3_chan[ch].fc / 1268.52e6;	//done
		BD3_chan[ch].code_freq = (1 + (*dllS1));
		BD3_writeCodeFreq(ch, BD3_chan[ch].code_freq);
		/////////////////////////////////////////////////////////////////////////////////
		
		if (BD3_chan[ch].ch_time>BD3_PULL_IN_TIME)
		{
			freqErr = 0.0;
		}
		else
		{	
			BD3_chan[ch].FLLIndex = 1 - BD3_chan[ch].FLLIndex;

			if (BD3_chan[ch].FLLFlag == 1 && BD3_chan[ch].FLLIndex == 1)
			{

				cross = BD3_chan[ch].i_old*sPI - BD3_chan[ch].q_old*sPR;
				dot = BD3_chan[ch].i_old*sPR + BD3_chan[ch].q_old*sPI;
				if((cross*cross+dot*dot)<1.0)
					phaseDiff = 0.0;
				else
					phaseDiff = dot>=0.0 ? cross/sqrt(dot*dot+cross*cross):-cross/sqrt(dot*dot+cross*cross);
				freqErr = phaseDiff / (BD3_twoPI*T);
				BD3_chan[ch].freqErr = freqErr;
			}
			else
			{

				freqErr = 0; // changed by Ning Luo in Sept/04
			}
			BD3_chan[ch].i_old = BD3_chan[ch].i_prompt;
			BD3_chan[ch].q_old = BD3_chan[ch].q_prompt;
		}

		if (fabs(sPR)<1e-3)
		{
			phaseErr = 0.0;
		}
		else
			phaseErr = atan(sPI/sPR);


		cLoopS0 = &(BD3_chan[ch].cLoopS0);
		cLoopS1 = &(BD3_chan[ch].cLoopS1);

		*cLoopS0 = *cLoopS0 + (phaseErr*BD3_PLLc0 + freqErr*BD3_FLLa1)*T;
		*cLoopS1 = *cLoopS1 + (BD3_PLLc1*phaseErr + (*cLoopS0) + BD3_FLLa2*freqErr)*T;
		BD3_chan[ch].fc = *cLoopS1 + BD3_PLLc2*phaseErr;
		BD3_chan[ch].carrier_freq = BD3_carrier_ref + BD3_chan[ch].fc;
		BD3_writeCarrierFreq(ch, BD3_chan[ch].carrier_freq);
    

		if (BD3_chan[ch].ch_time>BD3_PULL_IN_TIME)
		{
			BD3_bitSync(ch);
			if (BD3_chan[ch].state == BD3_track)
				return;
		}

	}
}

void BD3_bitSync(char ch)
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

	index = &(BD3_chan[ch].BitSyncIndex);


	BD3_chan[ch].IBuf[*index] = BD3_chan[ch].i_prompt;
	BD3_chan[ch].QBuf[*index] = BD3_chan[ch].q_prompt;

	oldSign = BD3_chan[ch].signBuf[*index];

	tempIndex = (*index + BD3_BUFSIZE - 1) % BD3_BUFSIZE;
	realP = (double)BD3_chan[ch].i_prompt*(double)BD3_chan[ch].IBuf[tempIndex]
		+ (double)BD3_chan[ch].q_prompt*(double)BD3_chan[ch].QBuf[tempIndex];
	imagP = (double)BD3_chan[ch].i_prompt*(double)BD3_chan[ch].QBuf[tempIndex]
		- (double)BD3_chan[ch].q_prompt*(double)BD3_chan[ch].IBuf[tempIndex];
	absAngle = fabs(atan2(imagP, realP)) - BD3_HALFPI;
	newSign = absAngle>=0 ? 1 : 0;
	BD3_chan[ch].signBuf[*index] = newSign;
	BD3_chan[ch].kCell[*index % 2] += (newSign - oldSign);
	*index = (*index + 1) % BD3_BUFSIZE;
	BD3_chan[ch].IQBufSize++;


	if (BD3_chan[ch].IQBufSize<BD3_BUFSIZE) return;

	BD3_chan[ch].IQBufSize = BD3_BUFSIZE;

	counter1 = 0;
	counter2 = 0;
	for (i=0; i<2; i++)
	{

		if (BD3_chan[ch].kCell[i] >= BD3_NBS2)
		{
			if (BD3_chan[ch].kCell[i] >= BD3_NBS1)
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

	BD3_chan[ch].ms_count = (*index - 1 + BD3_BUFSIZE - signIndex) % 2;
	if (BD3_chan[ch].ms_count != 1) return;
	BD3_writeEpoch(ch, BD3_chan[ch].ms_count);//
	
	counter = BD3_BUFSIZE;

	BD3_chan[ch].t_count = 0;

	BD3_chan[ch].subFrameSyncFlag = BD3_SUBFRAME_SYNCHRONIZING;

	BD3_chan[ch].WBP = BD3_chan[ch].NP = 0.0;


	while (counter>0)
	{
		BD3_chan[ch].i_prom_20 = 0;
		BD3_chan[ch].q_prom_20 = 0;
		do
		{
			sPR = BD3_chan[ch].IBuf[*index];
			sPI = BD3_chan[ch].QBuf[*index];
			BD3_chan[ch].i_prom_20 += sPR;
			BD3_chan[ch].q_prom_20 += sPI;


			BD3_chan[ch].WBP += (double)sPR*sPR + (double)sPI*sPI;

			(*index)++;
			*index %= BD3_BUFSIZE;
			counter--;
		} while((*index)%2 != signIndex && counter>0);
		if (counter == 0 && (*index)%2 != signIndex)
		{

			break;
		}
		else
		{

			sPI20 = BD3_chan[ch].q_prom_20;
			sPR20 = BD3_chan[ch].i_prom_20;
			BD3_chan[ch].NBP = sPR20*sPR20 + sPI20*sPI20;
			BD3_chan[ch].NP += (BD3_chan[ch].NBP / BD3_chan[ch].WBP / (BD3_BUFSIZE / 2));
			BD3_chan[ch].WBP = 0.0;

			absAngle = fabs(atan2(sPI20, sPR20));
			BD3_chan[ch].bit = absAngle> BD3_HALFPI ? 0 : 1;
			BD3_bdd2b1pream(ch, BD3_chan[ch].bit);//
			BD3_chan[ch].message[BD3_chan[ch].t_count++] = BD3_chan[ch].bit;

		}
	}


	if (BD3_chan[ch].NP - 1.0<0)
		BD3_chan[ch].CNo = 0.0;
   else
	   BD3_chan[ch].CNo = 10 * log10(1000.0*(BD3_chan[ch].NP - 1.0) / (2.0 - BD3_chan[ch].NP));

	BD3_chan[ch].BitSyncIndex = 0;
	BD3_chan[ch].IQBufSize = 0;
	for (i = 0; i<BD3_BUFSIZE; i++)
	{
		BD3_chan[ch].signBuf[i] = 0;
	}
	for (i=0;i<2;i++)
	{
		BD3_chan[ch].kCell[i] = 0;
	}

	BD3_chan[ch].i_prom_20 = 0;
	BD3_chan[ch].q_prom_20 = 0;
	BD3_chan[ch].i_early_20 = 0;
	BD3_chan[ch].q_early_20 = 0;
	BD3_chan[ch].i_late_20 = 0;
	BD3_chan[ch].q_late_20 = 0;
	BD3_chan[ch].WBP = BD3_chan[ch].NBP = BD3_chan[ch].NP = BD3_chan[ch].NBD = 0.0;

	BD3_chan[ch].trackTime = 0;
	BD3_chan[ch].state = BD3_track;
}
	

/*******************************************************************************
FUNCTION ch_track(char ch)
RETURNS  None.

PARAMETERS
			ch  char  channel number

PURPOSE  to track in carrier and code the GPS satellite and partially
			decode the navigation message (to determine TOW, subframe etc.)

*******************************************************************************/
void BD3_ch_track(char ch)
{
	double sER, sEI, sLR, sLI, sPR, sPI;
	double sPR20, sPI20, sER20, sEI20, sLR20, sLI20;

	double codeErr, T, phaseErr;
	double *dllS0, *dllS1;
	double *cLoopS0, *cLoopS1;
	double absAngle;
	BD3_CHANNEL* pChan = &BD3_chan[ch];

	BD3_chan[ch].trackTime++;

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
        phaseErr = atan(sPI/sPR);

	// update carrier loop
	cLoopS0 = &(pChan->cLoopS0);
	cLoopS1 = &(pChan->cLoopS1);

	*cLoopS0 = *cLoopS0 + phaseErr*BD3_PLLc0*T;
	*cLoopS1 = *cLoopS1 + (BD3_PLLc1*phaseErr + (*cLoopS0))*T;
	pChan->fc = *cLoopS1 + BD3_PLLc2*phaseErr;
	pChan->carrier_freq = BD3_carrier_ref + pChan->fc;
	BD3_writeCarrierFreq(ch, pChan->carrier_freq);
	pChan->ms_count=(++pChan->ms_count)%2;

	pChan->i_prom_20 += pChan->i_prompt;
	pChan->q_prom_20 += pChan->q_prompt;
	pChan->i_early_20 += pChan->i_early;
	pChan->q_early_20 += pChan->q_early;
	pChan->i_late_20 += pChan->i_late;
	pChan->q_late_20 += pChan->q_late;


	pChan->WBP += sPR*sPR+sPI*sPI;

	if (BD3_chan[ch].ms_count == 1)
	{
		
		T = 0.002;
		// update DLL
		sPI20 = pChan->q_prom_20;
		sPR20 = pChan->i_prom_20;
		sLI20 = pChan->q_late_20;
		sLR20 = pChan->i_late_20;
		sEI20 = pChan->q_early_20;
		sER20 = pChan->i_early_20;

		//codeErr = (sER20*sPR20+sEI20*sPI20)/(sER20*sER20+sPR20*sPR20+sEI20*sEI20+sPI20*sPI20)*(DLLdT/(2.0*2.046e6));
		codeErr = (sER20*sPR20 + sEI20*sPI20) / (sER20*sER20 + sPR20*sPR20 + sEI20*sEI20 + sPI20*sPI20)*(BD3_DLLdT / (2.0*10.23e6));//done
		dllS0 = &(pChan->dllS0);
		dllS1 = &(pChan->dllS1);
		*dllS0 += BD3_DLLc1*T*codeErr;
		//*dllS1 = *dllS0 + DLLc2*codeErr
		//				+ pChan->fc/1561.098e6;		//Doppler Aiding
		*dllS1 = *dllS0 + BD3_DLLc2*codeErr
						+ pChan->fc/1268.52e6;	//done
        pChan->code_freq = 1+(*dllS1);
		BD3_writeCodeFreq(ch, pChan->code_freq);


		pChan->NBP = sPI20*sPI20 + sPR20*sPR20;
		pChan->NP += (0.02 * pChan->NBP/pChan->WBP);
		pChan->WBP = 0.0;


		pChan->NBD = sPR20*sPR20 - sPI20*sPI20;
		pChan->phaseLockDetector = pChan->NBD/pChan->NBP;

		if (BD3_chan[ch].trackTime % 100 == 0 && BD3_chan[ch].trackTime>0)
		{
			// ���������
			if (BD3_chan[ch].NP - 1.0<0)
				BD3_chan[ch].CNo = 0.0;
         else
			// pChan->CNo = 10 * log10(1000.0*(pChan->NP - 1.0) / (2.0 - pChan->NP));
			 pChan->CNo = 10 * log10(100.0*(pChan->NP-1.0)/(2.0-pChan->NP));//������CNo�н�����
			pChan->NP = 0.0;
         if(pChan->CNo<30) 
         {
			 pChan->state = BD3_acquisition;
            pChan->phaseLockDetector=0.0;
            pChan->az=0;
            pChan->el=0;
			BD3_svStruct[pChan->prn].NumToTryAcq = 20;
			BD3_svStruct[pChan->prn].NumBTWEachTrial = 1000;
			BD3_svStruct[pChan->prn].maxf = (pChan->fc + BD3_DELTF);
			BD3_svStruct[pChan->prn].minf = (pChan->fc - BD3_DELTF);
			BD3_correlator[ch].state = BD3_CHN_OFF;
			BD3_correlator[ch].ready = 1;
         }
		}
			

		pChan->tr_bit_time++;
		absAngle = fabs(atan2((double)pChan->q_prom_20, (double)pChan->i_prom_20));
		pChan->bit = absAngle>BD3_HALFPI ? 0 : 1;

		BD3_bdd2b1pream(ch, pChan->bit);

		pChan->message[pChan->t_count++]=pChan->bit;

		pChan->i_prom_20 = 0;
		pChan->q_prom_20 = 0;
		pChan->i_early_20 = 0;
		pChan->q_early_20 = 0;
		pChan->i_late_20 = 0;
		pChan->q_late_20 = 0;
	}


	if (pChan->t_count==1500)
	{
		pChan->n_frame++;
		pChan->t_count=0;
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

int BD3_xors(long pattern)
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
inline int BD3_sign(long data)
{
	int result;

	if      ( data  > 0 ) result= 1;
	else if ( data == 0 ) result= 0;
	else if ( data  < 0 ) result=-1;
	return (result);
}


/*******************************************************************************
FUNCTION bit_test()
RETURNS  None.

PARAMETERS None.

PURPOSE
	  Determine if a bit in the data word has been set
*******************************************************************************/
inline int  BD3_bit_test(int data, char bit_n)
{
	return(data & BD3_test[bit_n]);
}

/*******************************************************************************
FUNCTION read_rcvr_par(void)
RETURNS  None.

PARAMETERS None.

PURPOSE  To read in from the rcvr_par file the receiver parameters that control
			acquisition, tracking etc.


*******************************************************************************/
char BD3_rcvr_par_file[4096];
void BD3_read_rcvr_par(void)
{
	char intext[40];
	
	if ((BD3_in = fopen(BD3_rcvr_par_file, "rt")) == NULL)
		{
			printf("Cannot open rcvr_par.dat file.\n");
			exit(0);
		}
		else
		{
			fscanf(BD3_in, "%s %s", intext, BD3_tzstr);
			fscanf(BD3_in, "%s %lf", intext, &BD3_mask_angle);
			BD3_mask_angle /= BD3_r_to_d;
			fscanf(BD3_in, "%s %lf", intext, &BD3_clock_offset);
			fscanf(BD3_in, "%s %d", intext, &BD3_interr_int);
			fscanf(BD3_in, "%s %d", intext, &BD3_ICP_CTL);
			fscanf(BD3_in, "%s %lf", intext, &BD3_nav_up);
			fscanf(BD3_in, "%s %d", intext, &BD3_out_pos);
			fscanf(BD3_in, "%s %d", intext, &BD3_out_vel);
			fscanf(BD3_in, "%s %d", intext, &BD3_out_time);
			fscanf(BD3_in, "%s %d", intext, &BD3_out_debug);
			fscanf(BD3_in, "%s %d", intext, &BD3_m_tropo);
			fscanf(BD3_in, "%s %d", intext, &BD3_m_iono);
			fscanf(BD3_in, "%s %d", intext, &BD3_align_t);
		}
	fclose(BD3_in);
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
//#define	DAICYCOR   //TODO������������ݶ�����Ҫ��α������
int BD3_judge_nits = 0;
double	BD3_t_cor[13];
short	BD3_towFlag = 0;
double BD3_wgs[3];
void  BD3_nav_fix(void)
{
	//st-atic int judge_nits = 0;

	char			ch,n;//,bit;
	int             bit;
	double			tr_time[13],tr_avg,ipart,clock_error;
	//sta-tic double	t_cor[13];
	int				i,ms;//,ms_2;
	double			chip;
    int				meas_bit_time_rem;
    long			meas_bit_time_offset;
	double			tic_CNTR;
	BD3_XYZ				rp_ecef;
	BD3_ECEFT			dm_gps_sat[13], dp_gps_sat[13];
	double			clock_error_in_Hz;
	//s-tatic short	towFlag=0;
	//define for debug
	//int             bit_new;//2010.11.18
	double          ad_for_tro_ion=0.0;
    double          pr[13];
	int             tr_prn=0;
	double             const_pram=0.0;
    //char            inNavFile[100]; //yuan�����ļ�
    FILE *inNavFile;
	
	tr_avg=0.0;
	n=1;
	for (ch = 0; ch <= BD3_chmax; ch++)
	{
		//printf("1 ���е���λ����\n");
		if (BD3_chan[ch].state != BD3_track)  // only use satellites stably tracked
			continue;
		meas_bit_time_offset=0;

		ms = BD3_chan[ch].epoch & 0x1f;	// the last five bits for 0 ~ 19 ms counter
		bit = BD3_chan[ch].epoch >> 8;      // counter of bit (0 - 50) һ��1000ms��20msһ���أ�һ�����һ�Σ���50����
			   	   
		chip = BD3_chan[ch].codePhase;
		BD3_chan[ch].int_carr_phase = BD3_chan[ch].carrier_counter + BD3_chan[ch].d_carr_phase;

		BD3_chan[ch].carrier_output += (BD3_chan[ch].int_carr_phase - BD3_CARRIER_FREQ);

		
		////////////////////////2010.11.18////////////////////////////////////////////////////
		if ((BD3_chan[ch].prn >= 1) && (BD3_chan[ch].prn <= 5))
        	{
            //  continue; //todo:GEO����
			   //ͳһ��bit Ӧ�ÿ���
			meas_bit_time_rem = BD3_chan[ch].meas_bit_time % 500;
		       if ( meas_bit_time_rem == bit+1 ) meas_bit_time_offset = -1;
		       if ( meas_bit_time_rem == bit-1 ) meas_bit_time_offset = +1;
		       if ( meas_bit_time_rem == 0 && bit == 499 ) meas_bit_time_offset = -1;
		       if ( meas_bit_time_rem == 499 && bit == 0 ) meas_bit_time_offset = +1;
			   if ((BD3_chan[ch].meas_bit_time + meas_bit_time_offset) % 500 == bit &&
				   BD3_chan[ch].state == BD3_track && BD3_chan[ch].CNo>30 &&
				   BD3_gps_eph[BD3_chan[ch].prn].valid == 1 &&
				   BD3_gps_eph[BD3_chan[ch].prn].health == 0 &&
				   BD3_chan[ch].tow_sync == 1 && chip <= 1e-3)
			   	   {   //0.001������
				   tr_time[n] = (BD3_chan[ch].meas_bit_time + meas_bit_time_offset)*.002 +  ////1������0.02�룬20���룬�����������������Ƶ�ʱ�䲿�֣�����20����
					   ms / 1000.0 + chip + BD3_SAMPLING_INT - 0.001; //ms/1000.0��С�����ؼƵ�ʱ�䲿�֣�����1����
					//chip��chip = chan[ch].codePhase;  ���ڵ�����λ����ɵ�ʱ�䣬����1/1023����
					//��sim_gps_interrupt�����1�뵽����chan[ch].codePhase = readCodePhase(ch);//return (double)correlator[ch].tauLatch/(1.023e6*d_2p40);
					//��Ϊ��NCO��40λ�ģ����λ���������Ƭ������2^40����Ϊ�൱��������40λ������λ��0~1023�ģ�����1023000���൱��0~1���룬1������һ��������
					 // tr_time[n] = (chan[ch].meas_bit_time + meas_bit_time_offset)*.002 +
						//  ms / 1000.0 + chip + SAMPLING_INT ;//todo:ȥ��-0.001

				   BD3_tr_ch[n] = ch;
				   tr_prn = BD3_chan[ch].prn;//
			          tr_avg+=tr_time[n];
			          n++;
			   	   }
			   if (BD3_towFlag == 0)
			   	{
					if(n>1)
					{
						BD3_m_time[1] = tr_time[n - 1] - 1.0 + 0.125;
						//m_time[1] = tr_time[n - 1] - 1.0 + 0.075; //todo:
						//m_time[1] = tr_time[1] + 0.075;//���75����Ӧ���ǹ��ƵĴ���ʱ��
						BD3_towFlag = 1;
					}
					 else
						 BD3_m_time[1] += BD3_nav_up;
			   	}
			  
        	}
		if (BD3_chan[ch].prn >= 6) //MEO/IGSO
			{
			  // continue;

			 // chan[ch].meas_bit_timeΪ��TIC�жϷ���ʱ�����tr_bit_time
			 // ����GPS���ڿ�ʼ�������bit��
			 // bitΪTIC�жϷ���ʱ�������epochCounter����
			meas_bit_time_rem = BD3_chan[ch].meas_bit_time % 50; // meas_bit_time_remֻ�ֱ�1s�ڵ�ʱ�䣬�ֱ���Ϊ20ms
		       if ( meas_bit_time_rem == bit+1 ) meas_bit_time_offset = -1;
		       if ( meas_bit_time_rem == bit-1 ) meas_bit_time_offset = +1;
		       if ( meas_bit_time_rem == 0 && bit == 49 ) meas_bit_time_offset = -1;
		       if ( meas_bit_time_rem == 49 && bit == 0 ) meas_bit_time_offset = +1;
			   if ((BD3_chan[ch].meas_bit_time + meas_bit_time_offset) % 50 == bit &&
				   BD3_chan[ch].state == BD3_track && BD3_chan[ch].CNo>30 &&
				   BD3_gps_eph[BD3_chan[ch].prn].valid == 1 &&
				   BD3_gps_eph[BD3_chan[ch].prn].health == 0 &&
				   BD3_chan[ch].tow_sync == 1 && chip <= 1e-3)
		           {
				   tr_time[n] = (BD3_chan[ch].meas_bit_time + meas_bit_time_offset)*.02 +
						  ms / 1000.0 + chip + BD3_SAMPLING_INT;//-0.019;  
					  //tr_time[n] = (chan[ch].meas_bit_time + meas_bit_time_offset)*.02 +
				   // 	ms / 1000.0 + chip + SAMPLING_INT +0.28;//-0.019;   //todo20170607  -0.048����
				  
				   //1������0.02�룬20���룬�����������������Ƶ�ʱ�䲿�֣�����20����
				   //ms/1000.0��С�����ؼƵ�ʱ�䲿�֣�����1����
				   //chip��chip = chan[ch].codePhase;  ���ڵ�����λ����ɵ�ʱ�䣬����1/1023����
				   //��sim_gps_interrupt�����1�뵽����chan[ch].codePhase = readCodePhase(ch);//return (double)correlator[ch].tauLatch/(1.023e6*d_2p40);
				   //��Ϊ��NCO��40λ�ģ����λ���������Ƭ������2^40����Ϊ�൱��������40λ������λ��0~1023�ģ�����1023000���൱��0~1���룬1������һ��������

				   BD3_tr_ch[n] = ch;
				  tr_prn = BD3_chan[ch].prn;//invalid when debuging data1 
			          tr_avg+=tr_time[n];
			          n++;
		           }
			   if (BD3_towFlag == 0)
			   	{
			   		if(n>1)
					{
						BD3_m_time[1] = tr_time[n - 1] - 1.0 + 0.075;
						//m_time[1] = tr_time[n - 1] + 0.075;//todo
						//m_time[1] = tr_time[n - 1] + 0.070;
						//m_time[1] = tr_time[n - 1] + 0.075 - 0.9517; //todo
						BD3_towFlag = 1;
					}
					 else
						 BD3_m_time[1] += BD3_nav_up;
			   	}
			}	
		if (BD3_out_debug)
		{
			fprintf(BD3_debug, " ch= %d PRN =%d bit time= %ld  bit= %d  ms= %d chip= %f",
				ch, BD3_chan[ch].prn, BD3_chan[ch].meas_bit_time, bit, ms, chip);
			if (BD3_ICP_CTL == 0)
				fprintf(BD3_debug, "CTL = %ld\n", BD3_chan[ch].doppler);
			else            
				fprintf(BD3_debug, "ICP = %lf\n", BD3_chan[ch].int_carr_phase);
		}	
	}

	BD3_n_track = n - 1;
	
	BD3_TIC_dt = BD3_i_TIC_dt*BD3_SAMPLING_INT;

	if (BD3_out_debug) fprintf(BD3_debug, "n_track= %d\n", BD3_n_track);

#if 0
	if (towFlag==0)
	{
      if (n_track>0)
      {
         m_time[1] = tr_time[1]+0.075;   //����17��������
         towFlag = 1; 
       /* if ((tr_prn>=1)&&(tr_prn<=5))//����1����GEO���Ƕ�,valid when debuging data 1
		 {
             m_time[1] = tr_time[1]+0.075;
             towFlag = 1;                   //debug ����ֻ��GEO���붨λ1205,���ǵ�ǰ���ǻ�����һ�̶�0.00042s
		 }
		 else
		 {
             m_time[1]+=nav_up;      
		 }*/
      }
      else if (bLocalTimeSetByTow)
      {
         m_time[1]+=nav_up;
      }
      else if (bTowDecoded)
      {
         m_time[1]=clock_tow;
         bLocalTimeSetByTow=true;
      }
      else
         m_time[1]+=nav_up;
	}
    else if (towFlag==1)
      m_time[1]+=nav_up;
#endif
	if (BD3_towFlag == 1)
		BD3_m_time[1] += BD3_nav_up;
    
//	const_pram=0.009;         //debug for precise sat positon, no progress

	for (i = 1; i <= BD3_n_track; i++)//substract 1 when debuging data1 
	{
       /* if((inNavFile=fopen("D:\\BD0010.08N","r") )==NULL)//��֤����������ʱ��������ʱ���
		{
		    printf("Cannot open rcvr_par.dat file.\n");
			exit(0);
		}
		else
		{
			get_nav_orb(inNavFile,gps_eph);
		    fclose(inNavFile);
		}*/

		//track_sat[i]=satpos_ephemeris(tr_time[i]-const_pram,chan[tr_ch[i]].prn);//�����п��ܴ˺���д����Ӳ�����ջ���ֲ		
		//track_sat[i]=SatProcess(tr_time[i], chan[tr_ch[i]].prn, gps_eph);
	    // ECEFT Sat_Cal_accord_orign(double t,int n,EPHEMERIS ephsv[])
		BD3_track_sat[i] = BD3_Sat_Cal_accord_orign(tr_time[i], BD3_chan[BD3_tr_ch[i]].prn, BD3_gps_eph);
        //mod for data109
		fprintf(BD3_out_trtime, "prn: %d	m_time[1]: %10.9f	tr_time: %10.9f	cor: %10.9f	tr_time_cor: %10.9f	sat.x: %10.8f	sat.y: %10.8f	sat.z: %10.8f  \^..^/", BD3_chan[BD3_tr_ch[i]].prn, BD3_m_time[1], tr_time[i], BD3_track_sat[i].tb, tr_time[i] - BD3_track_sat[i].tb, BD3_track_sat[i].x, BD3_track_sat[i].y, BD3_track_sat[i].z);
		if (i==5)
		{
		//	fprintf(out_trtime,"\n");
		}
		
		BD3_chan[BD3_tr_ch[i]].az = BD3_track_sat[i].az;
		BD3_chan[BD3_tr_ch[i]].el = BD3_track_sat[i].el;
		if (BD3_ICP_CTL == 0)
		{
		//	dm_gps_sat[i]=satpos_ephemeris(tr_time[i]-TIC_dt/2.0,chan[tr_ch[i]].prn); 
		//	dp_gps_sat[i]=satpos_ephemeris(tr_time[i]+TIC_dt/2.0,chan[tr_ch[i]].prn);
		//	dm_gps_sat[i]=SatProcess(tr_time[i]-TIC_dt/2.0, chan[tr_ch[i]].prn, gps_eph); 
		//	dp_gps_sat[i]=SatProcess(tr_time[i]+TIC_dt/2.0, chan[tr_ch[i]].prn, gps_eph); 
			dm_gps_sat[i] = BD3_Sat_Cal_accord_orign(tr_time[i] - BD3_TIC_dt / 2.0, BD3_chan[BD3_tr_ch[i]].prn, BD3_gps_eph);
			dp_gps_sat[i] = BD3_Sat_Cal_accord_orign(tr_time[i] + BD3_TIC_dt / 2.0, BD3_chan[BD3_tr_ch[i]].prn, BD3_gps_eph);
		}
		else
		{
		//   dm_gps_sat[i]=satpos_ephemeris(tr_time[i]-TIC_dt,chan[tr_ch[i]].prn); //for ICP
		//    dm_gps_sat[i]=SatProcess(tr_time[i]-TIC_dt, chan[tr_ch[i]].prn, gps_eph); 
			dm_gps_sat[i] = BD3_Sat_Cal_accord_orign(tr_time[i] - BD3_TIC_dt, BD3_chan[BD3_tr_ch[i]].prn, BD3_gps_eph);
			dp_gps_sat[i] = BD3_track_sat[i];
		}
		
        //ad_for_tro_ion = tropo_iono(tr_ch[i],track_sat[i].az,track_sat[i].el,tr_time[i]); //no ion and tro
		BD3_t_cor[i] = BD3_track_sat[i].tb - ad_for_tro_ion;

		//����α��	daicy
#ifdef	DAICYCOR
		if(chan[tr_ch[i]].prn==6)	const_pram = 0.0/c;
		else if(chan[tr_ch[i]].prn==7)	const_pram = -4.0/c;
		else if(chan[tr_ch[i]].prn==11)	const_pram = 100.0/c;
		else if(chan[tr_ch[i]].prn==12)	const_pram = 588.0/c;
		else const_pram = 0.0/c;
#endif
		
		BD3_dt[i] = BD3_m_time[1] - (tr_time[i] - BD3_t_cor[i]) + const_pram;
		pr[i] = BD3_dt[i] * BD3_c;
		BD3_d_sat[i].x = (dp_gps_sat[i].x - dm_gps_sat[i].x) / BD3_TIC_dt - BD3_track_sat[i].y*BD3_omegae;
		BD3_d_sat[i].y = (dp_gps_sat[i].y - dm_gps_sat[i].y) / BD3_TIC_dt + BD3_track_sat[i].x*BD3_omegae;
		BD3_d_sat[i].z = (dp_gps_sat[i].z - dm_gps_sat[i].z) / BD3_TIC_dt;

		if (BD3_ICP_CTL == 0) BD3_meas_dop[i] = BD3_chan[BD3_tr_ch[i]].doppler;
		else BD3_meas_dop[i] = BD3_chan[BD3_tr_ch[i]].int_carr_phase / BD3_TIC_dt - BD3_CARRIER_FREQ;
	}
	
	fprintf(BD3_out_trtime, "\n");
	
	if (BD3_n_track >= 4)
	{
		switch (sel & USING_POS_METHOD_SEL)
		{
		case USING_POS_VEL_TIME:
			BD3_rpvt = BD3_pos_vel_time(BD3_n_track);
			break;
		case USING_KALMAN_POS_VEL:
			if (BD3_judge_nits == 0)
			{
				BD3_rpvt = BD3_pos_vel_time(BD3_n_track);
			}
			else
			{
				BD3_rpvt = BD3_kalman_pos_vel(BD3_n_track);
			}
			break;
		default:
			break;
		}
		BD3_cbias = BD3_rpvt.dt;		// BD�Ӳ�	
		clock_error = BD3_rpvt.df;	// BD��Ư
		//clock_error_in_Hz = clock_error*1561.098;  // 1561.098Mhz,��һ��1561.098M���ز����ڣ�*һ������ƫ�ƶ���
		clock_error_in_Hz = clock_error*1268.52;//revised
		//m_time[1]=m_time[1];//-cbias;  
		BD3_m_time[1] = BD3_m_time[1] - BD3_cbias; //todo
		rp_ecef.x = BD3_rpvt.x;
		rp_ecef.y = BD3_rpvt.y;
		rp_ecef.z = BD3_rpvt.z;
		BD3_rp_llh = BD3_ecef_to_llh(rp_ecef);
		
		

		BD3_wgs[0] = BD3_rpvt.x;
		BD3_wgs[1] = BD3_rpvt.y;
		BD3_wgs[2] = BD3_rpvt.z;
		::PostMessage(NavOutWnd, WM_NAVIGATION_OUT, (WPARAM)BD3_wgs, 1);

		// vDoppler���ɽ��ջ�����������˶������doppler
		for (ch = 0; ch <= BD3_chmax; ch++)
		{
			BD3_chan[ch].vDoppler = 0.0;
		}
		for (ch = 1; ch <= BD3_n_track; ch++)
		{
			BD3_chan[BD3_tr_ch[ch]].vDoppler = BD3_meas_dop[ch] - clock_error_in_Hz;
		}

	//	if (rp_llh.hae>-2000.0 && rp_llh.hae< 18000 ) 
		{
			BD3_velocity();
			// ���²�֪��ΪʲôҪ���ٶ�����??? ����ܺ�ʵ��Ӧ���йأ�һ����˵��
			// ����������Դﵽ�������ٶ�
			if (BD3_speed < 514.0)      //speed��ȫ�ֱ�����velocity�����м���
			{
				if (fabs(clock_error)<5.0) BD3_clock_offset = clock_error;
				if (BD3_almanac_valid == 1) BD3_status = BD3_navigating;
				if (BD3_align_t == 1)
				{
					BD3_delta_m_time = modf(BD3_m_time[1], &ipart);
					if (BD3_nav_up<1.0)
					{
						BD3_delta_m_time = modf(BD3_delta_m_time / BD3_nav_up, &ipart);
						if (BD3_delta_m_time>0.5) BD3_m_error = (BD3_delta_m_time - 1.0)*BD3_nav_up;
						else BD3_m_error = BD3_delta_m_time*BD3_nav_up;
					}
					else
					{
						if (BD3_delta_m_time>0.5) BD3_m_error = (BD3_delta_m_time - 1.0) / BD3_nav_up;
						else BD3_m_error = BD3_delta_m_time / BD3_nav_up;
					}

#ifndef REAL_TIME
					tic_CNTR = BD3_TIC_ref*(1.0 - BD3_m_error / BD3_nav_up) / (1.0 + BD3_clock_offset*1.0e-6);
#endif
				//	programTIC(tic_CNTR);
				}

				BD3_rec_pos_llh.lon = BD3_rp_llh.lon;
				BD3_rec_pos_llh.lat = BD3_rp_llh.lat;
				BD3_rec_pos_llh.hae = BD3_rp_llh.hae;
				BD3_current_loc.lon = BD3_rp_llh.lon;
				BD3_current_loc.lat = BD3_rp_llh.lat;
				BD3_current_loc.hae = BD3_rp_llh.hae;
				BD3_rec_pos_xyz.x = rp_ecef.x;
				BD3_rec_pos_xyz.y = rp_ecef.y;
				BD3_rec_pos_xyz.z = rp_ecef.z;
				
				BD3_dops(BD3_n_track);
				BD3_m_time[0] = BD3_m_time[1];
			}
		}
	}
/*	else
	{                              
		
		m_time[1] = fmod(m_time[1],604800);
		rp_ecef.x=0.0;
		rp_ecef.y=0.0;
		rp_ecef.z=0.0;
		rpvt.xv=0.0;
		rpvt.yv=0.0;
		rpvt.zv=0.0;
	}*/

	if (BD3_n_track >= 4)
	{
		for (i = 1; i <= BD3_n_track; i++)
		{
		
			BD3_chan[BD3_tr_ch[i]].Pr = (BD3_m_time[1] - tr_time[i])*BD3_SPEEDOFLIGHT;//α��ֵ
		//	chan[tr_ch[i]].Pr = (m_time[1] - tr_time[i]+0.048)*SPEEDOFLIGHT;   //todo20170607
			
			BD3_chan[BD3_tr_ch[i]].dPr = BD3_chan[BD3_tr_ch[i]].vDoppler*BD3_lambda;//α������
			
			////����α��
			//char str[10];
			////itoa(sv, str, 10);
			//sprintf(str, "%d", BD3_chan[BD3_tr_ch[i]].prn);
			//char s[100] = "PRN_";
			//char *pfilename = strcat(s, str);

			//FILE *fd = fopen(pfilename, "a");
			//fprintf(fd, "TOW % 6ld �� %10.9f\n", BD3_clock_tow, BD3_chan[BD3_tr_ch[i]].Pr);
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
void BD3_velocity(void)
{
	//printf("���е����ٳ���\n");
	BD3_receiver.north.x = -cos(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
	BD3_receiver.north.y = -sin(BD3_rec_pos_llh.lon)*sin(BD3_rec_pos_llh.lat);
	BD3_receiver.north.z = cos(BD3_rec_pos_llh.lat);


	BD3_receiver.east.x = -sin(BD3_rec_pos_llh.lon);
	BD3_receiver.east.y = cos(BD3_rec_pos_llh.lon);

	BD3_receiver.up.x = cos(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
	BD3_receiver.up.y = sin(BD3_rec_pos_llh.lon)*cos(BD3_rec_pos_llh.lat);
	BD3_receiver.up.z = sin(BD3_rec_pos_llh.lat);


	BD3_receiver.vel.north = BD3_rpvt.xv*BD3_receiver.north.x + BD3_rpvt.yv*BD3_receiver.north.y +
		BD3_rpvt.zv*BD3_receiver.north.z;
	BD3_receiver.vel.east = BD3_rpvt.xv*BD3_receiver.east.x + BD3_rpvt.yv*BD3_receiver.east.y;
	BD3_receiver.vel.up = BD3_rpvt.xv*BD3_receiver.up.x + BD3_rpvt.yv*BD3_receiver.up.y +
		BD3_rpvt.zv*BD3_receiver.up.z;
	// �����ٶȵľ���ֵ
	BD3_speed = sqrt(BD3_receiver.vel.north*BD3_receiver.vel.north + BD3_receiver.vel.east*BD3_receiver.vel.east);//������speed=sqrt(receiver.vel.north*receiver.vel.north+receiver.vel.east*receiver.vel.east+receiver.vel.up*receiver.vel.up);
	// �����ٶȵķ����
	if (BD3_speed == 0.0) BD3_heading = 0.0;
	else BD3_heading = atan2(BD3_receiver.vel.east, BD3_receiver.vel.north);

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
/************************************************************************/
/*BDÿ����֡��һ���ֵ�ǰ15���ز����о�����룬��11���ز���BCH(15,11,1)���о������ֻ��һ��BCH�룬��Ϣλ26����
����9���ֲ���BCH(15��11��1)�ӽ�֯�ķ�ʽ���о�����룬ÿ����30���غ�����BCH�룬ÿ����BCH�룬������ʽ���30�����볤
�Ľ�֯�룬��Ϣλ��22����                                                                     */
/************************************************************************/
void BD3_bdd2b1pream(char ch, char bit)
{
	//s-tatic const unsigned pream=0x38900000L;
	const unsigned pream = 0x38900000L;
	//s-tatic const unsigned pream1=0x07680000L;
	const unsigned pream1 = 0x07680000L;
	unsigned long SOW,HOW,TLM;
	int sfid_s;
	long currentBitPos, frameLength;
	int bit_left=0;
	int bit_debug=0;
	int bit_orig=0;
	int bit_after=0;

	int result,result1,result2; 
	if (BD3_chan[ch].fifo1 & 0x20000000L)
	{
		BD3_chan[ch].fifo0 = (BD3_chan[ch].fifo0 << 1) + 1;
	}
	else
	{
		BD3_chan[ch].fifo0 = BD3_chan[ch].fifo0 << 1;
	}

	BD3_chan[ch].fifo1 = (BD3_chan[ch].fifo1 << 1) + bit;
	currentBitPos = BD3_chan[ch].t_count + 1;
	if (BD3_chan[ch].subFrameSyncFlag == BD3_SUBFRAME_SYNCHRONIZED)
	{	
		bool HOWParityOK = FALSE;

		frameLength = currentBitPos >= BD3_chan[ch].expectedFrameHead ?
			currentBitPos - BD3_chan[ch].expectedFrameHead : currentBitPos + 1500 - BD3_chan[ch].expectedFrameHead;
		if (frameLength==60) 
		{
			BD3_chan[ch].subFrameSyncFlag = BD3_SUBFRAME_SYNCHRONIZING;
			
			TLM = BD3_chan[ch].fifo0; //�������ÿ֡�ĵ�һ���֣�ǰ15���ز����о�����룬��11���ز���BCH(15,11,1)���о������ֻ��һ��BCH�룬��Ϣλ26����

			if (((pream^TLM)&0x3ff80000L)==0 || ((pream1^TLM)&0x3ff80000L)==0) 
			{
				if (BD3_bch_decode(TLM & 0x00007fff))
				{
					HOW = BD3_chan[ch].fifo1; //BCH(15��11��1)�ӽ�֯�ķ�ʽ���о������
					BD3_shift_data(&HOW);   //active by jh 3-25���������°�Ľ⽻֯��Ч�ʸ���   ���11+4+11+4,��������BCH����
					result1 = BD3_bch_decode(HOW & 0x00007fff);
					result2 = BD3_bch_decode(HOW & 0x3fff8000);

					BD3_shift_data2(&HOW);
					
					SOW=((TLM & 0xff0L)<<8)|((HOW & 0x3ffc0000L)>>18);//  SOW ��8λ����12λ
					sfid_s=int((TLM & 0x7000L)>>12); 
					
					if (result1==1 && result2==1)
					{
						HOWParityOK = TRUE;

						if (((pream1^TLM) & 0x3ff80000L)==0) 
						{
							BD3_chan[ch].reverseflag = 1;
							sfid_s=(~sfid_s) & 0x0007;  
							SOW=(~SOW) & 0x000fffffL;
						}
					}

                  
					if (HOWParityOK == TRUE
						&& SOW<604800  
						&& sfid_s == BD3_chan[ch].expectedSubFrameID)
					{
						BD3_chan[ch].subFrameSyncFlag = BD3_SUBFRAME_SYNCHRONIZED;
						BD3_chan[ch].expectedFrameHead = (currentBitPos - 60 + 300) % 1500;
						if (++BD3_chan[ch].expectedSubFrameID>5)
						{
							BD3_chan[ch].expectedSubFrameID = 1;
						}
						
						if (sfid_s==1)     
						{                        
							BD3_d_tow = BD3_clock_tow - SOW - 1; //
							BD3_chan[ch].offset = BD3_chan[ch].t_count - 59;

							bit_orig = BD3_readEpochCheck(ch);
							bit_debug = (0x1f & BD3_readEpochCheck(ch)) | (60 << 8);
							
							                             
							BD3_writeEpoch(ch, (0x1f & BD3_readEpochCheck(ch)) | (60 << 8));  ///2010.11.22
							bit_after = BD3_readEpochCheck(ch);

							if (BD3_chan[ch].offset<0.0) BD3_chan[ch].offset += 1500;

							BD3_chan[ch].tr_bit_time = SOW * 500 + 60;
							BD3_chan[ch].TOW = SOW;
							BD3_chan[ch].tow_sync = 1;
							BD3_thetime = BD3_thetime - BD3_d_tow;
							BD3_clock_tow = SOW + 1;
							BD3_chan[ch].sfid = sfid_s;

						}  

						else if (sfid_s>1 && sfid_s<6)
						{
							BD3_d_tow = BD3_clock_tow - SOW - 1;
							
							BD3_chan[ch].offset = BD3_chan[ch].t_count - 59 - (sfid_s - 1) * 300;
							bit_left=(sfid_s-1)*300+60;
							while(bit_left>500)
							{
							   bit_left -=500;
							}
							BD3_writeEpoch(ch, (0x1f & BD3_readEpochCheck(ch)) | (bit_left << 8));
							

							if (BD3_chan[ch].offset<0.0) BD3_chan[ch].offset += 1500;
							
							//chan[ch].tr_bit_time=SOW*500+60;
							BD3_chan[ch].tr_bit_time = SOW * 500 + 60 + (sfid_s - 1) * 300;
							
							BD3_chan[ch].tow_sync = 1;
							BD3_chan[ch].TOW = SOW;
							BD3_chan[ch].sfid = sfid_s;
							
						} 
					} 
				} 
			}
			if (BD3_chan[ch].subFrameSyncFlag != BD3_SUBFRAME_SYNCHRONIZED)
			{

				BD3_chan[ch].t_count = 0;
				BD3_chan[ch].n_frame = 0;
			}
		}
	} 

	else
	{

		bool HOWParityOK = FALSE;

		TLM = BD3_chan[ch].fifo0;

		if (((pream^TLM)&0x3ff80000L)==0 || ((pream1^TLM)&0x3ff80000L)==0)  
		{
		
			if (BD3_bch_decode(TLM & 0x00007fff))
			{
				HOW = BD3_chan[ch].fifo1;
				BD3_shift_data(&HOW);
				result1 = BD3_bch_decode(HOW & 0x00007fff);
				result2 = BD3_bch_decode(HOW & 0x3fff8000);
				
				BD3_shift_data2(&HOW);
				
				SOW=((TLM & 0xff0L)<<8)|((HOW & 0x3ffc0000L)>>18); 
				sfid_s=int((TLM & 0x7000L)>>12); 
				
				if (result1==1 && result2==1)
				{
					HOWParityOK = TRUE;
					
					if (((pream1^TLM) & 0x3ff80000L)==0) 
					{
						BD3_chan[ch].reverseflag = 1;
						sfid_s=(~sfid_s) & 0x0007; 
						SOW=(~SOW) & 0x000fffffL;
					}
				}
				
			
				//xxpream
				if (HOWParityOK == TRUE
					&& SOW<604800  
					&& sfid_s<6 && sfid_s>0)
				{
					BD3_chan[ch].subFrameSyncFlag = BD3_SUBFRAME_SYNCHRONIZED;
					BD3_chan[ch].expectedFrameHead = (currentBitPos - 60 + 300) % 1500;
					BD3_chan[ch].expectedSubFrameID = sfid_s == 5 ? 1 : sfid_s + 1;
					
					if (sfid_s==1)     
					{
						BD3_d_tow = BD3_clock_tow - SOW - 1;
						BD3_chan[ch].offset = BD3_chan[ch].t_count - 59;
						BD3_writeEpoch(ch, (0x1f & BD3_readEpochCheck(ch)) | (60 << 8));
						
						if (BD3_chan[ch].offset<0.0) BD3_chan[ch].offset += 1500;
						
						BD3_chan[ch].tr_bit_time = SOW * 500 + 60;
						
						BD3_chan[ch].TOW = SOW;
						BD3_chan[ch].tow_sync = 1;
						BD3_thetime = BD3_thetime - BD3_d_tow;
						BD3_clock_tow = SOW + 1;
						BD3_chan[ch].sfid = sfid_s;
						
						
					}  
					else if (sfid_s>1 && sfid_s<6)
					{
						BD3_d_tow = BD3_clock_tow - SOW - 1;
						
						BD3_chan[ch].offset = BD3_chan[ch].t_count - 59 - (sfid_s - 1) * 300;
						bit_left=(sfid_s-1)*300+60;
						while(bit_left>500)
							{
							   bit_left -=500;
							}
						BD3_writeEpoch(ch, (0x1f & BD3_readEpochCheck(ch)) | (bit_left << 8));

						if (BD3_chan[ch].offset<0.0) BD3_chan[ch].offset += 1500;

						BD3_chan[ch].tr_bit_time = SOW * 500 + 60 + (sfid_s - 1) * 300;
						BD3_chan[ch].tow_sync = 1;
						BD3_chan[ch].TOW = SOW;
						BD3_chan[ch].sfid = sfid_s;
						
					}
				} 
			} 
		}
	}				
	if ((BD3_chan[ch].t_count + BD3_chan[ch].n_frame * 1500 - BD3_chan[ch].offset) % 1500 == 300
		&& BD3_chan[ch].subFrameSyncFlag == BD3_SUBFRAME_SYNCHRONIZED)
	{
		BD3_chan[ch].frame_ready_1 = 1;
	}

}


void BD3_undointerlace(unsigned long * x)
{
	unsigned long x1,x2,xx,p1,p2;
	int i;


	x1=x2=p1=p2=0;
	xx=(*x);
	xx=xx>>8;

	//--------x1--------11
	for(i=0;i<11;i++)
	{
		if( ((xx&0x00200000)>>21)==1 )   
		{
			x1++;
		}
		x1=x1<<1;
		xx=xx<<2;
	}  
	x1=x1>>1;


	xx=(*x);
	xx=xx>>8;
	for(i=0;i<11;i++)
	{
		if( ((xx&0x00100000)>>20)==1 )  
		{
			x2++;
		}//if-end
		x2=x2<<1;
		xx=xx<<2;
	} //for-end
	x2=x2>>1;

	//---p1-----4
	xx=(*x);
	xx=xx&0x000000ffL;
	for(i=0;i<4;i++)
	{
		if( ((xx&0x00000080L)>>7)==1 )
		{
			p1++;
		} //if-end
		p1=p1<<1;
		xx=xx<<2;
	}//for-end
	p1=p1>>1;


	//---p2----- 4
	xx=(*x);
	xx=xx&0x000000ffL;
	for(i=0;i<4;i++)
	{
		if( ((xx&0x00000040L)>>6)==1 )
		{
			p2++;
		} //if-end
		p2=p2<<1;
		xx=xx<<2;
	}//for-end
	p2=p2>>1;

	*x=((x1&0x7ffL)<<19)|((x2&0x7ffL)<<8)|((p1&0xfL)<<4)|(p2&0xfL);

 
}


void BD3_shift_data(unsigned long *data30bit)
{
	int temp_bit1 = 0, temp_bit2 = 0, shift_data_res1 = 0, shift_data_res2 = 0, shift_data_res3 = 0, shift_data_res4 = 0;
	int high15bit = ((*data30bit) & 0x3fff8000)>>15;
	int low15bit = (*data30bit) & 0x00007fff;
	for (int shift_itemp=0; shift_itemp<15; shift_itemp++)
	{
		temp_bit1 = ((high15bit>>shift_itemp) & 0x1 );
		temp_bit2 = ((low15bit>>shift_itemp) & 0x1 );
		
		if (shift_itemp%2 == 0)
		{
			shift_data_res1 |= (temp_bit1<<(shift_itemp/2));
			shift_data_res2 |= (temp_bit2<<(shift_itemp/2));
		}	
		else
		{
			shift_data_res3 |= (temp_bit1<<((shift_itemp-1)/2));
			shift_data_res4 |= (temp_bit2<<((shift_itemp-1)/2));
		}
	}
	*data30bit = (( (shift_data_res1<<7) | shift_data_res4 )<<15) | ( (shift_data_res3<<8) | shift_data_res2 );

}

void BD3_shift_data2(unsigned long *data30bit)
{

	int shift_data_temp1 = 0, shift_data_temp2=0, shift_data_temp3 = 0, shift_data_temp4=0;

	shift_data_temp1 =  (*data30bit) & 0x3ff80000;
	shift_data_temp2 = ((*data30bit) & 0x00078000) >> 11;
	shift_data_temp3 = ((*data30bit) & 0x00007ff0) << 4;
	shift_data_temp4 =  (*data30bit) & 0x0000000f;
    
	(*data30bit) = shift_data_temp1 | shift_data_temp3 | shift_data_temp2 | shift_data_temp4;
	
}

int BD3_checkf(unsigned long x, int flag){
	int result ,i;
	char D0,D1,D2,D3,flag1,flag2,fsbuf[30];
	D0=D1=D2=D3=0;
	for(i=0;i<30;i++){
      fsbuf[i]=(char)((x&0x20000000)>>29);
	  x=x<<1;
	} //for-end

	if(flag==1){
		for(i=0;i<15;i++){
		flag1=(D3==fsbuf[i])?0:1;  
		flag2=(D3==D0)?0:1;
		D3=D2;
		D2=D1;
		D1=flag2;
		D0=flag1;
		}//for-end
	
	} // flag==1 end
	else if(flag==2){
		for(i=0;i<15;i++){
		flag1=(D3==fsbuf[15+i])?0:1;  
		flag2=(D3==D0)?0:1;
		D3=D2;
		D2=D1;
		D1=flag2;
		D0=flag1;
		}//for-end
     
	} // flag==2 end
	else {
		  printf(" \n input error \n ");
		  D0=D1=D2=D3=1;
	} //else-end 
	
	if( (D0==0)&&(D1==0)&&(D2==0)&&(D3==0) )
		 result=0;
	else 
	{
		printf("checf BCH decode error");
		result=1;
	}
		 return result; 
}


int BD3_bch_decode(unsigned long shift_datas)
{
	int bch_itemp, bch_jtemp;
	int bch_decode_in[15] = {0};	
	int bch_decode_m = 0;
	int bch_decode_d[4] = {0};
	
	for (bch_itemp=0;bch_itemp<15;bch_itemp++)
	{
		bch_decode_in[14-bch_itemp] = ((shift_datas>>bch_itemp) & 0x1);
	}
	
	bch_decode_d[0]=0;
	bch_decode_d[1]=0;
	bch_decode_d[2]=0;
	bch_decode_d[3]=0;	
	
	for (bch_itemp=0;bch_itemp<15;bch_itemp++)
	{
		bch_decode_m   = bch_decode_d[3];
		bch_decode_d[3]= bch_decode_d[2];
		bch_decode_d[2]= bch_decode_d[1];
		bch_decode_d[1]= bch_decode_d[0]^bch_decode_m;
		bch_decode_d[0]= bch_decode_in[bch_itemp]^bch_decode_m;
	}
	
	if ((bch_decode_d[3]==0) && (bch_decode_d[2]==0) && (bch_decode_d[1]==0) && (bch_decode_d[0]==0))
	{
		return 1;
	}
	else
	{	
		printf("%c%cBCH decode error",'\a','\a');
		return 0;
	}
}


int BD3_check(unsigned long x, int flag)
{

	unsigned long xx,x1,x2,p1,p2;
	int result;
	x1=x2=p1=p2=0;
	xx=x;
	x2=(xx&0x0007ff00)>>8;
	p1=(xx&0x000000f0)>>4;

	xx=(xx&0x3ff8000f)|(x2<<4)|(p1<<15);

	result = BD3_checkf(xx, flag);
	return result;
}
//////////////////////////////////////////////////////////////////////////////	
//////////////////////////////////////////////////////////////////////////////	
//////////////////////////////////////////////////////////////////////////////	
//////////////////////////////////////////////////////////////////////////////	
//////////////////////////////////////////////////////////////////////////////	
//////////////////////////////////////////////////////////////////////////////
void BD3_ch_pull_in_D1(char ch)
{
	double sER, sEI, sLR, sLI, sPR, sPI;

	double codeErr, T, phaseErr, freqErr;
	double *dllS0, *dllS1;
	double *cLoopS0, *cLoopS1;
	double cross, dot, phaseDiff;
	
	BD3_chan[ch].ch_time++;
	if (BD3_chan[ch].ch_time == 1)
	{
		return;
	}

	sER = BD3_chan[ch].i_early;
	sEI = BD3_chan[ch].q_early;
	sLR = BD3_chan[ch].i_late;
	sLI = BD3_chan[ch].q_late;
	sPR = BD3_chan[ch].i_prompt;
	sPI = BD3_chan[ch].q_prompt;

	// below added by Ning Luo in Sept/04
	if (BD3_chan[ch].ch_time == 2)
	{
		BD3_chan[ch].freqErr = 0;
		BD3_chan[ch].i_old = BD3_chan[ch].i_prompt;
		BD3_chan[ch].q_old = BD3_chan[ch].q_prompt;
		BD3_minFreqErr[ch] = BD3_M_PI / 2;
		BD3_maxFreqErr[ch] = -BD3_M_PI / 2;
		return;
	}

	if (BD3_chan[ch].ch_time<BD3_FINE_FREQ_RESOLUTION + 2)
	{
		cross = BD3_chan[ch].i_old*sPI - BD3_chan[ch].q_old*sPR;
		dot = BD3_chan[ch].i_old*sPR + BD3_chan[ch].q_old*sPI;
		phaseDiff = atan(cross/dot);//atan2(cross,dot);

		if (phaseDiff>BD3_maxFreqErr[ch]) BD3_maxFreqErr[ch] = phaseDiff;
		if (phaseDiff<BD3_minFreqErr[ch]) BD3_minFreqErr[ch] = phaseDiff;
		BD3_chan[ch].freqErr += phaseDiff;
		BD3_chan[ch].i_old = BD3_chan[ch].i_prompt;
		BD3_chan[ch].q_old = BD3_chan[ch].q_prompt;
	}
	else if (BD3_chan[ch].ch_time == BD3_FINE_FREQ_RESOLUTION + 2)
	{

        T = 1e-3;
		freqErr = (BD3_chan[ch].freqErr - BD3_maxFreqErr[ch] - BD3_minFreqErr[ch]) / (BD3_FINE_FREQ_RESOLUTION - 3.0);
		freqErr = freqErr / (2 * BD3_M_PI*T);
		BD3_chan[ch].fc += freqErr;
		BD3_chan[ch].cLoopS1 = BD3_chan[ch].fc;
		BD3_chan[ch].carrier_freq = BD3_carrier_ref + BD3_chan[ch].fc;
		BD3_writeCarrierFreq(ch, BD3_chan[ch].carrier_freq);

	}
	// above added by Ning Luo in Sept/04

	else if (BD3_chan[ch].ch_time>BD3_FINE_FREQ_RESOLUTION + 2)
	{
		/////////////////////////////////////////////////////////////////////////////////
		T = 1e-3;			
		//codeErr = (sER*sPR+sEI*sPI)/(sER*sER+sPR*sPR+sEI*sEI+sPI*sPI)*(DLLdT/(2.0*2.046e6));
		codeErr = (sER*sPR + sEI*sPI) / (sER*sER + sPR*sPR + sEI*sEI + sPI*sPI)*(BD3_DLLdT / (2.0*10.23e6));//done
		dllS0 = &(BD3_chan[ch].dllS0);
		dllS1 = &(BD3_chan[ch].dllS1);
		*dllS0 += BD3_DLLc1*T*codeErr;
       // *dllS1 = *dllS0 + DLLc2*codeErr+chan[ch].fc/1561.098e6;	
		*dllS1 = *dllS0 + BD3_DLLc2*codeErr + BD3_chan[ch].fc / 1268.52e6;
		BD3_chan[ch].code_freq = (1 + (*dllS1));
		BD3_writeCodeFreq(ch, BD3_chan[ch].code_freq);
		/////////////////////////////////////////////////////////////////////////////////
		
		if (BD3_chan[ch].ch_time>BD3_PULL_IN_TIME)
		{
			freqErr = 0.0;
			BD3_chan[ch].i_old = BD3_chan[ch].i_prompt;
			BD3_chan[ch].q_old = BD3_chan[ch].q_prompt;
		}
		else
		{	
			BD3_chan[ch].FLLIndex = 1 - BD3_chan[ch].FLLIndex;

			if (BD3_chan[ch].FLLFlag == 1 && BD3_chan[ch].FLLIndex == 1)
			{

				cross = BD3_chan[ch].i_old*sPI - BD3_chan[ch].q_old*sPR;
				dot = BD3_chan[ch].i_old*sPR + BD3_chan[ch].q_old*sPI;
				if((cross*cross+dot*dot)<1.0)
					phaseDiff = 0.0;
				else
					phaseDiff = dot>=0.0 ? cross/sqrt(dot*dot+cross*cross):-cross/sqrt(dot*dot+cross*cross);
				freqErr = phaseDiff / (BD3_twoPI*T);
				BD3_chan[ch].freqErr = freqErr;
			}
			else
			{

				freqErr = 0; // changed by Ning Luo in Sept/04
			}
			BD3_chan[ch].i_old = BD3_chan[ch].i_prompt;
			BD3_chan[ch].q_old = BD3_chan[ch].q_prompt;
		}

		if (fabs(sPR)<1e-3)
		{
			phaseErr = 0.0;
		}
		else
			phaseErr = atan(sPI/sPR);


		cLoopS0 = &(BD3_chan[ch].cLoopS0);
		cLoopS1 = &(BD3_chan[ch].cLoopS1);

		*cLoopS0 = *cLoopS0 + (phaseErr*BD3_PLLc0 + freqErr*BD3_FLLa1)*T;
		*cLoopS1 = *cLoopS1 + (BD3_PLLc1*phaseErr + (*cLoopS0) + BD3_FLLa2*freqErr)*T;
		BD3_chan[ch].fc = *cLoopS1 + BD3_PLLc2*phaseErr;
		BD3_chan[ch].carrier_freq = BD3_carrier_ref + BD3_chan[ch].fc;
		BD3_writeCarrierFreq(ch, BD3_chan[ch].carrier_freq);
		if (BD3_chan[ch].ch_time>BD3_PULL_IN_TIME)
		{
			BD3_bitSync_D1(ch);
			BD3_chan[ch].state = BD3_track;
		}

	}
}
// λͬ����⺯��
void BD3_bitSync_D1(char ch)
{
	char bit;
	bit = (BD3_chan[ch].i_prompt >= 0.0) ? 1 : 0;
	BD3_chan[ch].count++;
	BD3_chan[ch].count = (BD3_chan[ch].count % 1000);
	BD3_NHDecoderd_D1(ch, bit);
}
//NH���뺯��
 //���� : �ŵ�n����Ӧ��NH������
 //���� : n, �ŵ���
 //        bit, ���յ��ı���
 //��� : NH��ƥ���־
 //        �Ѿ�����ı���
void BD3_NHDecoderd_D1(int n, char bit)
{
	int i;
	double spi,spq;    //2007.05.08
	//�����ݱ��� bit ���������
	for (i = BD3_FIFO_NUM - 1; i>0; i--)
	{
		if (BD3_chan[n].fifo[i - 1] & 0x80000)
			BD3_chan[n].fifo[i] = (BD3_chan[n].fifo[i] << 1) + 1;
		else
			BD3_chan[n].fifo[i] = BD3_chan[n].fifo[i] << 1;
	}
	BD3_chan[n].fifo[0] = (BD3_chan[n].fifo[0] << 1) + bit;

	if (BD3_chan[n].NHMatchFlag == BD3_NH_UN_MATCHED)
	{
		// �����������
		if (BD3_chan[n].count >= 100)
		{
			BD3_CoRelation_D1(BD3_chan[n].fifo, BD3_FIFO_NUM, BD3_chan[n].v);
			if (BD3_check_D1(BD3_chan[n].v, BD3_FIFO_NUM))
			{
				BD3_chan[n].NHMatchFlag = BD3_NH_MATCHED;
				BD3_chan[n].NH_count = 0;
				BD3_writeEpoch(n, 0);
				BD3_chan[n].t_count = 0;
				BD3_chan[n].subFrameSyncFlag = BD3_SUBFRAME_SYNCHRONIZING;
				///////��������ȵ��м䲽�� 2007.05.08
				BD3_chan[n].trackTime = 0;
				BD3_chan[n].WBP = 0.0;
				BD3_chan[n].NBP = 0.0;
				BD3_chan[n].NP = 0.0;
				BD3_chan[n].i_prom_20 = BD3_chan[n].q_prom_20 = 0;
				//////////////
				for(i=4; i>=0; i--)
				{
					// ���յ� 1 bit
					BD3_chan[n].bit = BD3_judge_D1(BD3_chan[n].v[i]);
				
					// ����֡ͬ�� pream
				
					// �����յ��ı��ط��뻺����
					BD3_chan[n].message[BD3_chan[n].t_count++] = BD3_chan[n].bit;
				}
			}
		}
	}
	else
	{
		/////////��������ȵ��м䲽��  2007.05.08
		BD3_chan[n].itemp[BD3_chan[n].NH_count] = BD3_chan[n].i_prompt;
		BD3_chan[n].qtemp[BD3_chan[n].NH_count] = BD3_chan[n].q_prompt;
		BD3_chan[n].WBP += BD3_chan[n].i_prompt*BD3_chan[n].i_prompt + BD3_chan[n].q_prompt*BD3_chan[n].q_prompt;
	    ///////////////
		BD3_chan[n].NH_count++;
		BD3_chan[n].trackTime++;
		if (BD3_chan[n].NH_count == 20)
		{
			///////////ȥ��NH����Ի���ֵ�ۼӵ�Ӱ��  2007.05.08
			BD3_chan[n].itemp[5] = -BD3_chan[n].itemp[5];
			BD3_chan[n].itemp[8] = -BD3_chan[n].itemp[8];
			BD3_chan[n].itemp[9] = -BD3_chan[n].itemp[9];
			BD3_chan[n].itemp[11] = -BD3_chan[n].itemp[11];
			BD3_chan[n].itemp[13] = -BD3_chan[n].itemp[13];
			BD3_chan[n].itemp[16] = -BD3_chan[n].itemp[16];
			BD3_chan[n].itemp[17] = -BD3_chan[n].itemp[17];
			BD3_chan[n].itemp[18] = -BD3_chan[n].itemp[18];
			BD3_chan[n].qtemp[5] = -BD3_chan[n].qtemp[5];
			BD3_chan[n].qtemp[8] = -BD3_chan[n].qtemp[8];
			BD3_chan[n].qtemp[9] = -BD3_chan[n].qtemp[9];
			BD3_chan[n].qtemp[11] = -BD3_chan[n].qtemp[11];
			BD3_chan[n].qtemp[13] = -BD3_chan[n].qtemp[13];
			BD3_chan[n].qtemp[16] = -BD3_chan[n].qtemp[16];
			BD3_chan[n].qtemp[17] = -BD3_chan[n].qtemp[17];
			BD3_chan[n].qtemp[18] = -BD3_chan[n].qtemp[18];
			//////////����20ms�Ļ���ֵ   2007.05.08
			for(int iii=0; iii<20; iii++)
			{
				BD3_chan[n].i_prom_20 += BD3_chan[n].itemp[iii];
				BD3_chan[n].q_prom_20 += BD3_chan[n].qtemp[iii];
			}
			spi = BD3_chan[n].i_prom_20;        //�˴����������������ת��������ᵼ��NBP�������
			spq = BD3_chan[n].q_prom_20;
			/////////��������ȵ��м䲽��   2007.05.08
			BD3_chan[n].NBP = spi*spi + spq*spq;
			BD3_chan[n].NP += (0.02 * BD3_chan[n].NBP / BD3_chan[n].WBP);
			BD3_chan[n].WBP = 0.0;
			////////////// ���������    2007.05.08
			//if (chan[n].trackTime%1000==0 && chan[n].trackTime>0)
			if (BD3_chan[n].trackTime % 900 == 0 && BD3_chan[n].trackTime>0)//����
			{
				if (BD3_chan[n].NP - 1.0<0)
					BD3_chan[n].CNo = 0.0;
				else
					BD3_chan[n].CNo = 10 * log10(1000.0*(BD3_chan[n].NP - 1.0) / (20 - BD3_chan[n].NP));
				BD3_chan[n].NP = 0.0;
			}
			////////////
			BD3_chan[n].tr_bit_time++;
			
			BD3_chan[n].NH_count = 0;
			BD3_CoRelation_D1(BD3_chan[n].fifo, BD3_FIFO_NUM, BD3_chan[n].v);
			// ���յ� 1 bit
			BD3_chan[n].bit = BD3_judge_D1(BD3_chan[n].v[0]);
			// ����֡ͬ�� pream
			BD3_pream_D1(n, BD3_chan[n].bit);
			// �����յ��ı��ط��뻺����
			BD3_chan[n].message[BD3_chan[n].t_count++] = BD3_chan[n].bit;

			BD3_chan[n].i_prom_20 = 0;     //2007.05.08
			BD3_chan[n].q_prom_20 = 0;     //2007.05.08

		}
	}
	if (BD3_chan[n].t_count == 1500)
	{
		BD3_chan[n].t_count = 0;
		BD3_chan[n].n_frame++;
	}
}

 //���� : fifo��NH������������

int BD3_CoRelation_D1(int fifo[], int n, int v[])
{
	int i;
	int j;
	int cr = 0;
	int xor;
	
	for(i=0; i<n; i++)
	{
		xor = BD3_NH ^ fifo[i];
		xor = xor & 0xFFFFF;
		v[i] = 0;
		
		for(j=0; j<20; j++)
		{
			if( xor & 1 )
				v[i]--;
			else
				v[i]++;
			xor = xor >> 1;
		}
		cr += v[i];
	}
	return 0;
}
/*******************************************************************************
//---------------------------15bit У��1��2-------------------------------------
********************************************************************************/
int BD3_checkf_D1(unsigned long x, int flag){
	int result ,i;
	char D0,D1,D2,D3,flag1,flag2,fsbuf[30];
	D0=D1=D2=D3=0;
	for(i=0;i<30;i++){
      fsbuf[i]=(char)((x&0x20000000)>>29);
	  x=x<<1;
	} //for-end

	if(flag==1){
		for(i=0;i<15;i++){
		flag1=(D3==fsbuf[i])?0:1;  
		flag2=(D3==D0)?0:1;
		D3=D2;
		D2=D1;
		D1=flag2;
		D0=flag1;
		}//for-end
	
	} // flag==1 end
	else if(flag==2){
		for(i=0;i<15;i++){
		flag1=(D3==fsbuf[15+i])?0:1;  
		flag2=(D3==D0)?0:1;
		D3=D2;
		D2=D1;
		D1=flag2;
		D0=flag1;
		}//for-end
     
	} // flag==2 end
	else {
		  printf(" \n input error \n ");
		  D0=D1=D2=D3=1;
	} //else-end 
	
	if( (D0==0)&&(D1==0)&&(D2==0)&&(D3==0) )
		 result=0;
	else result=1;
		 return result;    //�����Ϊ0��ʾУ����ȷ���������
}
/*******************************************************************************
//----------------------------word У��(15bit flag--1/2)------------------------
********************************************************************************/
int BD3_check_D1(unsigned long x, int flag){
unsigned long xx,x1,x2,p1,p2;
int result;
x1=x2=p1=p2=0;
xx=x;
x2=(xx&0x0007ff00)>>8;
p1=(xx&0x000000f0)>>4;

xx=(xx&0x3ff8000f)|(x2<<4)|(p1<<15);  //������xx�� : x1 p1 x2 p2

result = BD3_checkf_D1(xx, flag);
return result;
}

int BD3_check_D1(int v[], int n)
{
	int result = 0;
	for(int i=0; i<n; i++)
	{
		if (v[i] >= BD3_CR_MIN || v[i] <= (-BD3_CR_MIN))
			result++;
	}
	
	// ��ǰ����Ϊ5��������3����������
	return ( result >= 3 );
}

 //����ؽ�� v �о�

int BD3_judge_D1(int v)
{
	//int i;
	int r[2] = {0, 0};
	int p_xor = v ^ BD3_NH;
	int n_xor = v ^ (~BD3_NH);
	int retval;
	if (v > BD3_CR_MIN)
		retval = 1;
	else if (v < -BD3_CR_MIN)
		retval = 0;
	else
	{
		retval = ( v > 0 ) ? 1 : 0;
	}
	return retval;
}

unsigned long BD3_pream = 0x38900000L;  //wqnavmess
unsigned long BD3_pream1 = 0x07680000L;
void BD3_pream_D1(char ch, char bit)
{
	//s-tatic unsigned long pream=0x38900000L;  //wqnavmess
	//s-tatic unsigned long pream1=0x07680000L;
	unsigned long SOW,HOW,TLM;
	int sfid_s;
	long currentBitPos, frameLength;

	int result,result1,result2;  //xxpream

	// fifo1��fifo0��������£�
	// fifo0��������ȵ����֣�fifo1������ź󵽵���
	// fifo0�ĵ�30λΪһ����Ч�֣���2λ��ǰһ֡��У��bit��D29,D30
	// fifo1�ĵ�30λΪһ����Ч�֣���2λ��ǰһ֡��У��bit��D29,D30��fifo0�����2������ͬ
	
	// fifo1�����λ������fifo0�����λ
	if (BD3_chan[ch].fifo1 & 0x20000000L)
	{
		BD3_chan[ch].fifo0 = (BD3_chan[ch].fifo0 << 1) + 1;
	}
	else
	{
		BD3_chan[ch].fifo0 = BD3_chan[ch].fifo0 << 1;
	}

	// �µ�bit����fifo1�����λ
	BD3_chan[ch].fifo1 = (BD3_chan[ch].fifo1 << 1) + bit;

	//wqnavmess
	TLM = BD3_chan[ch].fifo0;  //wqpream
	HOW = BD3_chan[ch].fifo1;  //wqpream
	BD3_undointerlace(&HOW);  //wqpream 

	SOW=((TLM & 0xff0L)<<8)|((HOW & 0x3ffc0000L)>>18);  //xxpream  SOW ��8λ����12λ
	sfid_s=int((TLM & 0x7000)>>12); //xxpream
	currentBitPos = BD3_chan[ch].t_count + 1;

	if (BD3_chan[ch].subFrameSyncFlag == BD3_SUBFRAME_SYNCHRONIZED)
	{
		frameLength = currentBitPos >= BD3_chan[ch].expectedFrameHead ?
			currentBitPos - BD3_chan[ch].expectedFrameHead : currentBitPos + 1500 - BD3_chan[ch].expectedFrameHead;
		if (frameLength==60)  // �յ�TLM��HOW��
		{
			BD3_chan[ch].subFrameSyncFlag = BD3_SUBFRAME_SYNCHRONIZING;
			if (((BD3_pream^TLM) & 0x3ff80000L) == 0 || ((BD3_pream1^TLM) & 0x3ff80000L) == 0)   //2007.05.15  ld
			{

				result = BD3_checkf_D1(TLM, 2);  //xxpream  ---checkf?
		
				if (!result)   //xxpream
				{
					
					result1 = BD3_check_D1(HOW, 1);  //xxpream
					result2 = BD3_check_D1(HOW, 2);  //xxpream
					
					if(result1==0&&result2==0)
					{
						if (((BD3_pream1^TLM) & 0x3ff80000L) == 0)
							BD3_chan[ch].reverseflag = 1;
					}

					if (BD3_chan[ch].reverseflag)
					{
						sfid_s=(~sfid_s)&0x0007;    //2007.05.15  ld
						SOW=(~SOW)&0x000fffffL;
					}
                  
					if ((result1==0&&result2==0)// is parity of HOW ok?   //xxpream
						&& SOW<604800  
						&& sfid_s == BD3_chan[ch].expectedSubFrameID)
					{
						BD3_chan[ch].subFrameSyncFlag = BD3_SUBFRAME_SYNCHRONIZED;
						BD3_chan[ch].expectedFrameHead = (currentBitPos - 60 + 300) % 1500;
						if (++BD3_chan[ch].expectedSubFrameID>5)
						{
							BD3_chan[ch].expectedSubFrameID = 1;
						}
						
						if (sfid_s==1)     
						{                        
							// ������Ľ��Ƶ�ǰʱ��Ϊ��SOW+1
							BD3_d_tow = BD3_clock_tow - SOW - 1; //xxpream

							
								// ������ǵ�һ��֡�����˻�2���ֳ�-1��λ�þ��ǵ�һ֡��
								// ��һ���ֵĵ�һ��bit���������е�λ��
							BD3_chan[ch].offset = BD3_chan[ch].t_count - 59;

								// 0x1f&readEpochCheck(ch)�Ƕ���ÿ��bit���ms����
								// ���ֵ�ڽ���λͬ����ʱ���Ѿ���ȷ��
								// bit������Ϊ10����Ϊ��������60bit������һ��bit��Ӧ��
								// ������(0)���������������н��յ������ǵ�61��������ÿ
								// �����50bit����ǰbit������(61-1)%50
								BD3_writeEpoch(ch, (0x1f & BD3_readEpochCheck(ch)) | 0xa00);

								if (BD3_chan[ch].offset<0.0) BD3_chan[ch].offset += 1500;
								
								BD3_chan[ch].tr_bit_time = SOW * 50 + 60;  //xxpream
								BD3_chan[ch].TOW = SOW; //xxpream
								BD3_chan[ch].tow_sync = 1;
								BD3_thetime = BD3_thetime - BD3_d_tow;
								BD3_clock_tow = SOW + 1; //xxpream
								BD3_chan[ch].sfid = sfid_s;
							
						}  //if (sfid_s==1)
						// ����յ��Ĳ��ǵ�һ֡�����������ʼλ�û᲻һ��
						else if (sfid_s>1 && sfid_s<6)
						{
							BD3_d_tow = BD3_clock_tow - SOW - 1;
							
							// offset��epoch�ļ��㷽���μ��Ե�һ��֡�Ĵ���
							BD3_chan[ch].offset = BD3_chan[ch].t_count - 59 - (sfid_s - 1) * 300;
							BD3_writeEpoch(ch, (0x1f & BD3_readEpochCheck(ch)) | 0xa00);
							
							// ����յ��Ĳ��ǵ�һ֡����С��0�����������һ����
							// ����ԭ���յ��˵�һ֡�������ڴ����ԭ��û�в���λͬ��
							if (BD3_chan[ch].offset<0.0) BD3_chan[ch].offset += 1500;
							
							BD3_chan[ch].tr_bit_time = SOW * 50 + 60;  //xxpream
							BD3_chan[ch].tow_sync = 1;
							BD3_chan[ch].TOW = SOW;  //xxpream
							BD3_chan[ch].sfid = sfid_s;
							
						} // if (sfid_s>1 && sfid_s<6)
					} // validate header of a subframe
				} // validate TLM parity
			}	// validate preamble pattern
			if (BD3_chan[ch].subFrameSyncFlag != BD3_SUBFRAME_SYNCHRONIZED)
			{
				// һ��ʧȥ��֡ͬ���͸�λ������
				BD3_chan[ch].t_count = 0;
				BD3_chan[ch].n_frame = 0;
			}
		} // validate subframe length
	} // validate subframe synchronization

	else	// ��û�л����֡ͬ��
	{
		if (((BD3_pream^TLM) & 0x3ff80000L) == 0 || ((BD3_pream1^TLM) & 0x3ff80000L) == 0)   //2007.05.15 ld
		{
			result = BD3_checkf_D1(TLM, 2);  //xxpream
		
			if (!result)  //xxpream 
			{
			
				result1 = BD3_check_D1(HOW, 1); //xxpream
				result2 = BD3_check_D1(HOW, 2); //xxpream

				if(result1==0&&result2==0)
				{
					if (((BD3_pream1^TLM) & 0x3ff80000L) == 0)
						BD3_chan[ch].reverseflag = 1;
				}
			
				if (BD3_chan[ch].reverseflag)
				{
					sfid_s=(~sfid_s)&0x0007;    //2007.05.15  ld
					SOW=(~SOW)&0x000fffffL;
				}
				if ((result1==0&&result2==0) 
					&& SOW<604800  
					&& sfid_s<6 && sfid_s>0)
				{
					BD3_chan[ch].subFrameSyncFlag = BD3_SUBFRAME_SYNCHRONIZED;
					BD3_chan[ch].expectedFrameHead = (currentBitPos - 60 + 300) % 1500;
					BD3_chan[ch].expectedSubFrameID = sfid_s == 5 ? 1 : sfid_s + 1;
					
					if (sfid_s==1)     
					{                         
					
						BD3_d_tow = BD3_clock_tow - SOW - 1; //xxpream

						
						// ������ǵ�һ��֡�����˻�2���ֳ�-1��λ�þ��ǵ�һ֡��
						// ��һ���ֵĵ�һ��bit���������е�λ��
						BD3_chan[ch].offset = BD3_chan[ch].t_count - 59;
						
						// 0x1f&readEpochCheck(ch)�Ƕ���ÿ��bit���ms����
						// ���ֵ�ڽ���λͬ����ʱ���Ѿ���ȷ��
						// bit������Ϊ10����Ϊ��������60bit������һ��bit��Ӧ��
						// ������(0)���������������н��յ������ǵ�61��������ÿ
						// �����50bit����ǰbit������(61-1)%50
						BD3_writeEpoch(ch, (0x1f & BD3_readEpochCheck(ch)) | 0xa00);
						
						if (BD3_chan[ch].offset<0.0) BD3_chan[ch].offset += 1500;
						
						BD3_chan[ch].tr_bit_time = SOW * 50 + 60;  //xxpream
						
						BD3_chan[ch].TOW = SOW; //xxpream
						BD3_chan[ch].tow_sync = 1;
						BD3_thetime = BD3_thetime - BD3_d_tow;
						BD3_clock_tow = SOW + 1;  //xxpream
						BD3_chan[ch].sfid = sfid_s;
						
						
					}  //if (sfid_s==1)
					// allow resync on other subframes if TOW matches clock to 3 seconds
					// this should improve the re-acquisition time
					// ����յ��Ĳ��ǵ�һ֡�����������ʼλ�û᲻һ��
					else if (sfid_s>1 && sfid_s<6)
					{
						BD3_d_tow = BD3_clock_tow - SOW - 1; //xxpream
						
							// offset��epoch�ļ��㷽���μ��Ե�һ��֡�Ĵ���
						BD3_chan[ch].offset = BD3_chan[ch].t_count - 59 - (sfid_s - 1) * 300;
						BD3_writeEpoch(ch, (0x1f & BD3_readEpochCheck(ch)) | 0xa00);

							// ����յ��Ĳ��ǵ�һ֡����С��0�����������һ����
							// ����ԭ���յ��˵�һ֡�������ڴ����ԭ��û�в���λͬ��
						if (BD3_chan[ch].offset<0.0) BD3_chan[ch].offset += 1500;

						BD3_chan[ch].tr_bit_time = SOW * 50 + 60;  //xxpream
						BD3_chan[ch].tow_sync = 1;
						BD3_chan[ch].TOW = SOW;  //xxpream
						BD3_chan[ch].sfid = sfid_s;
						
					} // if (sfid_s>1 && sfid_s<6)
				} // validate header of a subframe
			} // validate TLM parity
		}	// validate preamble pattern
	}				

	//  a 1500 bit frame of data is ready to be read
	//  t_count��ʵ���յ���bit����offset��ѭ�������б궨�ĵ�һ��֡��λ��
	//  if ((chan[ch].t_count-chan[ch].offset)%1500==0)
	//  ֻҪ�յ���֡1��2��3�Ϳ��Զ�λ�����Խ�����������Ϊ
	if ((BD3_chan[ch].t_count + BD3_chan[ch].n_frame * 1500 - BD3_chan[ch].offset) % 1500 == 900
		&& BD3_chan[ch].subFrameSyncFlag == BD3_SUBFRAME_SYNCHRONIZED)
	{
		BD3_chan[ch].frame_ready = 1;
	}
}
void BD3_ch_track_D1(char ch)
{
	double sER, sEI, sLR, sLI, sPR, sPI;
	double codeErr, T, phaseErr, freqErr;
	double *dllS0, *dllS1;
	double *cLoopS0, *cLoopS1;
	double cross, dot, phaseDiff;
	sER = BD3_chan[ch].i_early;
	sEI = BD3_chan[ch].q_early;
	sLR = BD3_chan[ch].i_late;
	sLI = BD3_chan[ch].q_late;
	sPR = BD3_chan[ch].i_prompt;
	sPI = BD3_chan[ch].q_prompt;
	/////////////////////////////////////////////////////////////////////////////////
	T = 1e-3;			
	//codeErr = (sER*sPR+sEI*sPI)/(sER*sER+sPR*sPR+sEI*sEI+sPI*sPI)*(DLLdT/(2.0*2.046e6));
	codeErr = (sER*sPR + sEI*sPI) / (sER*sER + sPR*sPR + sEI*sEI + sPI*sPI)*(BD3_DLLdT / (2.0*10.23e6));//done
	dllS0 = &(BD3_chan[ch].dllS0);
	dllS1 = &(BD3_chan[ch].dllS1);
	*dllS0 += BD3_DLLc1*T*codeErr;
   // *dllS1 = *dllS0 + DLLc2*codeErr+chan[ch].fc/1561.098e6;	
	*dllS1 = *dllS0 + BD3_DLLc2*codeErr + BD3_chan[ch].fc / 1268.52e6;	//done
	BD3_chan[ch].code_freq = (1 + (*dllS1));
	BD3_writeCodeFreq(ch, BD3_chan[ch].code_freq);
	/////////////////////////////////////////////////////////////////////////////////
	cross = BD3_chan[ch].i_old*sPI - BD3_chan[ch].q_old*sPR;
	dot = BD3_chan[ch].i_old*sPR + BD3_chan[ch].q_old*sPI;
	phaseDiff = atan(cross/dot);//atan2(cross,dot);
	freqErr = 0;

	if (fabs(sPR)<1e-3)
	{
		phaseErr = 0.0;
	}
	else
		phaseErr = atan(sPI/sPR);

	cLoopS0 = &(BD3_chan[ch].cLoopS0);
	cLoopS1 = &(BD3_chan[ch].cLoopS1);
	*cLoopS0 = *cLoopS0 + (phaseErr*BD3_PLLc0 + freqErr*BD3_FLLa1)*T;
	*cLoopS1 = *cLoopS1 + (BD3_PLLc1*phaseErr + (*cLoopS0) + BD3_FLLa2*freqErr)*T;
	BD3_chan[ch].fc = *cLoopS1 + BD3_PLLc2*phaseErr;
	BD3_chan[ch].carrier_freq = BD3_carrier_ref + BD3_chan[ch].fc;
	BD3_writeCarrierFreq(ch, BD3_chan[ch].carrier_freq);
	BD3_bitSync_D1(ch);
}
//////////////////////////////////////////////////////////////////////////////	
//////////////////////////////////////////////////////////////////////////////	
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////	
//////////////////////////////////////////////////////////////////////////////	
void BD3_get_nav_orb(FILE *RinexEPP_file, BD3_EPHEMERIS *snv)
{	
	rewind(RinexEPP_file);
	for(int i=1;!feof(RinexEPP_file);i++)
	{
		BD3_read_RinexEPP(RinexEPP_file, &snv[i]);
	}
}

int BD3_read_RinexEPP(FILE *RinexEPP_file, BD3_EPHEMERIS *snv)//read one epoch orbit parameter
{
	char		LineOfStr[256];
	double      t1,t2,t3,t4,t5,t6;

	BD3_TIME        timetag;

	
	if(fgets(LineOfStr,sizeof(LineOfStr),RinexEPP_file))
	{
		sscanf(LineOfStr,"%d%d%d%d%d%d%lf%lf%lf%lf",
			&snv->ura,&timetag.Year,&timetag.Month,&timetag.Day,&timetag.Hour,&timetag.Min,&timetag.Second,
			&snv->af0,&snv->af1,&snv->af2);
    }
	if(fgets(LineOfStr,sizeof(LineOfStr),RinexEPP_file))
	{
		sscanf(LineOfStr,"%lf%lf%lf%lf",&snv->iode,&snv->crs,&snv->dn,&snv->ma);
	}
	if(fgets(LineOfStr,sizeof(LineOfStr),RinexEPP_file))
	{
		sscanf(LineOfStr,"%lf%lf%lf%lf",&snv->cuc,&snv->ety,&snv->cus,&snv->sqra);
	}
	if(fgets(LineOfStr,sizeof(LineOfStr),RinexEPP_file))
	{
		sscanf(LineOfStr,"%lf%lf%lf%lf",&snv->toe,&snv->cic,&snv->w0,&snv->cis);
	}
	if(fgets(LineOfStr,sizeof(LineOfStr),RinexEPP_file))
	{
		sscanf(LineOfStr,"%lf%lf%lf%lf",&snv->inc0,&snv->crc,&snv->w,&snv->omegadot);
	}
	if(fgets(LineOfStr,sizeof(LineOfStr),RinexEPP_file))
	{
		sscanf(LineOfStr,"%lf%lf%lf%lf",&snv->idot,&t1,&snv->week,&t2);
	}
	if(fgets(LineOfStr,sizeof(LineOfStr),RinexEPP_file))
	{
		sscanf(LineOfStr,"%lf%lf%lf%lf",&t3,&t4,&snv->tgd,&t5);
	}
	if(fgets(LineOfStr,sizeof(LineOfStr),RinexEPP_file))
	{
		sscanf(LineOfStr,"%lf",&t6);
	}
	// for for	
	return 1;
}

//////////////////////////////////////////////////////////////////////////////	



