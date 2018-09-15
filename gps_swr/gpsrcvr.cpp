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

CHANNEL		   chan[chmax + 1];				// 全程变量，用于存储每个通道跟踪环路的状态
CORRELATOR	   correlator[chmax + 1];		// 全程变量，用于存储每个通道相关器的状态
SVSTRUCT	      svStruct[MaxSvNumber + 1];	// 表示卫星可用性
long		      globalBufLength[chmax + 1];	// 用于保存每个通道缓存的数据长度，需初始化
double  *buffer[chmax + 1];			// 用于保存每个通道的缓存数据，需初始化分配空间
long		      correlatorDataLength;	// 共有缓存区的数据长度
long		      TIC_RT_CNTR;				// TIC实时计数器；
long		      TIC_CNTR;					// TIC的最大计数，对应了TIC中断的发生周期
// 注意：TIC_CNTR的赋值只能用programTIC函数实现
double         TIC_CNTR_RES;
double         TIC_CNTR_DBL;
long		      TIC_OCCUR_FLAG;			// TIC中断发生的标志

int			   display_page = 0;			// 控制显示第i个页面
unsigned	test[16] =
{ 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };
unsigned int tr_ch[13];					   // 索引0不用，内容为具有有效观测数据的chan的标号
int			out_debug, out_pos, out_vel, out_time;		// 是否输出debug和PVT信息的标志符
DMS			cur_lat, cur_long;			   // 用于显示经纬度

// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
int			   nav_tic;						// 一个nav_up的时间间隔对应的TIC中断的次数
int            ICP_CTL = 0;		         // 输出累积载波相位还是跟踪环路的值，标志
double         code_corr;              // CA码速率的修正控制字，1.023MHz对应1.0
long           time_on = 0;	            // 计数器，步进为秒，程序不停，计数不止
long		      cc_scale = 1540;       	// 载波标频(1575.42MHz)/CA码标频(1.023MHz)
double         nav_up = 1.0;					// 导航解的基本单位，单位：秒
double		   speed, heading;          // 接收机速度的绝对值和方位角
long		d_tow;								// 接收机时间差，单位：秒
int			key = 0;								// 检查键盘的按键信息
int			tic_count = 0;						// TIC计数器，也是测量中断发生次数的计数器
// 注意：测量中断的频率这里缺省为：100ms
// 计数范围为1s[0-9]
int			hms_count = 0;						// TIC计数器，计数范围为1分钟[0-599]
int			nav_count;							// TIC计数器，计数范围为[0,nav_tic-1]
int			min_flag;							// 分钟计数满标志
int			nav_flag;							// 计算导航解标志
int			sec_flag;							// 秒标志
int			n_track;							// 跟踪(通道处于tracking模式)的卫星个数
unsigned int interr_int = 512;
double		clock_offset = 0.0;					// 接收机的钟漂，单位是ppm,初始值未知，假定为0
// 正值表示接收机的频率低于标称值
XYZ			rec_pos_ecef;						// 接收机的坐标
long		i_TIC_dt;							// 两次计算导航解的间隔,单位是样点个数
double		TIC_dt;								// 两次计算导航解的间隔,单位是秒
// 所以TIC_dt=i_TIC_dt*采样间隔
double		m_time[3];							// 接收机时间,[1]是当前时刻的，[0]是上一次的
// [2]似乎没有用
// m_time用于表示接收机的GPS时间
// 在原来的程序中m_time没有有效的更新,
// 它应该靠秒中断标志sec_flag来维护
double		delta_m_time;						// 接收机时间的小数部分
double		m_error;							// 接收机的测量时间和GPS整秒的误差
long		TIC_sum;							// 多个测量中断中期内累积的计数值
//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
//int			mstat;							// 仅用于GP2021，包含有miss data标志的寄存器
char		**caTable;							// 非实时处理时，存储37×1023的CA码表

double		DLLc1, DLLc2;						// DLL的系数
double		PLLc0, PLLc1, PLLc2;				// PLL的系数
double		FLLa1, FLLa2;						// FLL的系数

// 调试用的全程变量
double		*IQQ, *IQI;
double		IBUF[3000], QBUF[3000];
FILE		*fpIQ;
long		IQCounter;
FILE		*fpobs;
FILE		*fpeph;

long		detectNum[chmax + 1];					// 搜索某颗卫星的持续时间
double		threshold_sig_search;				// 信号检测门限							

//char		data[SAMPLESPMS+1];							// 信号缓存区
//char		data[6000];							// 信号缓存区
double		minFreqErr[chmax + 1], maxFreqErr[chmax + 1];

unsigned long uLastPCTick, uCurrentPCTick;

__int64 iTotalBitCounter = 0;

bool bLocalTimeSetByTow = false;
bool bTowDecoded = false;

// 外部变量
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
float   CARRIER_FREQ;   //标称中频  0625
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

	ADNumber = (int)pow(2.0, ADNumber);//ADNumber：输入的比特量化位数
	carrier_ref = CARRIER_FREQ;//中频频率

	SAMPLING_INT = 1.0 / SAMPLING_FREQ;//SAMPLING_FREQ:采样频率
	DETECTSIZE = (long)((long long)(SAMPLING_FREQ*0.01 + 0.5) & 0xffffffffUL);//10毫秒的数据点数

	TIC_ref = (long)((long long)(SAMPLING_FREQ*0.1 + 0.5 - 1) & 0xffffffffUL);// 100ms对应的中断计数值 100毫秒的数据点数  预设TIC_CNTR的计数是100毫秒

	deltaPhaseConst = SAMPLING_INT*carrierNCOMax;// carrierNCOMax对应了载波DCO的实际分辨率2^30   它再乘以标称载波就变成标称载波频率字了
	// deltaPhaseConst就对应了求载波频率字的 2^30/采样频率
	deltaCodePhaseConst = SAMPLING_INT*1.023e6*d_2p40;// 标称码频率*2^40/采样频率 = 码频率字
	//SAMPLING_INT*1.023e6*d_2p40;

	SAMPLESPMS = (long)((long long)(SAMPLING_FREQ*0.001) & 0xffffffffUL);   //0627//lyq
	//jh SAMPLESPMS每毫秒的数据点数

	ACC_INT_PERIOD = SAMPLESPMS / 2 + 1;//+1之后虽然不是刚好的0.5毫秒，但是这只是读的点数，在数据处理的时候，还是按整毫秒的多少来取的，这里没影响
	//#define ACC_INT_PERIOD  SAMPLESPMS/2+1			// 0.5ms的样点数
	//jh ACC_INT_PERIOD:每0.5毫秒积分（清零仍然是1毫秒），0.5毫秒的中断数
	GPSL1_data = new char[SAMPLESPMS + 1];

	fopen_s(&out_trtime, "tr_time.txt", "w");//保存结果
	fopen_s(&daicy_file_pr, "daicy_pr.txt", "w");
	fopen_s(&daicy_file_pos, "daicy_pos.txt", "w");


	float fnumber = (float)SAMPLING_FREQ / 500;//2毫秒的点数  因为要取2MS的本地伪码，为什么要2ms：GPS信号的CA码长1023码片，周期1ms ，但在给定采样率下，1ms内采到的点数不一定是2n
	int fftcount = 0;
	//lyq
	do
	{
		fnumber = fnumber / 2;//fnumber是浮点型
		fftcount++;
	} while (fnumber > 2);

	if (fnumber <= 1.5)
		FFTSIZE = (int)(pow(2.0, fftcount)*1.5);//与下面差别很大，是1.5倍 2的多少次方  但注意都是取整
	else
		FFTSIZE = (int)pow(2.0, fftcount + 1);//不管是if else都肯定取的数大于2ms的点数

	bufSize = (long)((long long)(SAMPLING_FREQ*0.0005) & 0xffffffffUL);//0.5毫秒的数据点数，与ACC_INT_PERIOD对应

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

	read_rcvr_par(); //从rcvr_par文件读入控制捕获跟踪的接收机参数
	rec_pos_xyz.x = 0.0;// set up user position at earth center, if last valid position is
	// stored in a file, the position will be updated
	rec_pos_xyz.y = 0.0;
	//	rec_pos_xyz.z=0.0;

	//准备输出文件
	if (out_pos == 1 || out_vel == 1 || out_time == 1)
		fopen_s(&stream, "gpsrcvr.log", "w");//gpsrcvr.log存放接收机的当前定位时间，纬经高，XYZ,东北天速度，钟漂，H/V/TDOP
	if (out_debug == 1)
		fopen_s(&debug, "debug.log", "w+");


	read_initial_data();

	initSignalSearch();// 为信号搜索动态分配数组

	//current_loc=receiver_loc();
	rec_pos_llh.lon = current_loc.lon;// 检查保存的文件中是否存有接收机的位置
	rec_pos_llh.lat = current_loc.lat;
	rec_pos_llh.hae = current_loc.hae;

	// 设置navFix的更新时间间隔，
	nav_tic = (int)(nav_up / 0.1 + 0.5);	// 对应了nav_up周期的测量中断数：一秒10次，因为测量中断100毫秒一次，这里设为10，则共1000毫秒解算一次
	//这个量和测量中断时间一起决定解算的频率
	//测量中断一次就能得到一次伪距从而得到一次定位结果，这里1秒10次中断即可以最大1秒10次的刷新率，刷新率可设为1~10
	programTIC(TIC_ref);	// 等效为: TIC_CNTR = TIC_ref // TIC的最大计数，对应了TIC中断的发生周期// 注意：TIC_CNTR的赋值只能用programTIC函数实现

	for (ch = 0; ch <= chmax; ch++)
	{
		chan[ch].state = CHN_OFF;//这个关闭状态才能通道分配
		chan[ch].prn = 0;
	}
	for (ch = MinSvNumber; ch <= MaxSvNumber; ch++)
	{
		svStruct[ch].state = AVAILABLE;//设置所有卫星均可用
		svStruct[ch].undetectedCounter = 0;
		svStruct[ch].NumBTWEachTrial = ReAcqTime;//#define ReAcqTime 1000000LTrial实验期   它就是重捕的门限，控制某颗星被重捕的时间
		//NumBTWEachTrial：number between each Trial  BTW:between
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
	// 初始化信道的分配，冷启动和热启动的分配方式不同
	hot_cold = 1;
	if (hot_cold == 1)
	{
		ch_alloc();
	}
	else if (hot_cold == 2)
	{
		hot_ch_alloc();
	}
	// double	m_time[3]; // 接收机时间,[1]是当前时刻的，[0]是上一次的 [2]似乎没有用 
	//m_time用于表示接收机的GPS时间 在原来的程序中m_time没有有效的更新,它应该靠秒中断标志sec_flag来维护
	m_time[1] = clock_tow;	// 将估计的GPS秒赋值给接收机本地时间,注意这不是PC机的时间 

	read_ephemeris();//从"current.eph"中读星历

#ifndef REAL_TIME
	TIC_RT_CNTR = 0;	// simulator tic counter in correlator
	num = ACC_INT_PERIOD;	// simulator acc_int counter in correlator
	//#define ACC_INT_PERIOD  SAMPLESPMS/2+1			// 0.5ms的样点数	
	//每次取0.5ms的样点数取做相关累加，但是每1ms才输出一次相关的结果，可以来一个点就累加，但是慢，若1ms累加又慢，所以这取0.5

	initTrackLoopPar();//初始化载波环
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
		//socket传输定位结果——抗干扰算法收敛时间
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
		// TIC_CNTR+1是TIC_RT_CNTR可以达到的最大值，达到后会立即置0
		// TIC_RT_CNTR+num是到本次积分清除中断处理后，TIC_RT_CNTR将会达到的值
		TIC_DIF = TIC_CNTR + 1 - (TIC_RT_CNTR + num);
		//TIC_RT_CNTR TIC实时计数器；
		//TIC_DIF就是为了计算num
		// 下次应该读取的样点个数，然后把这些点去做相关
		GPSL1_receiver_init_num = num = TIC_DIF < ACC_INT_PERIOD ? TIC_DIF : ACC_INT_PERIOD;
		//保证它一段时间读上来的数据刚好凑成100毫秒，以完成100毫秒锁存

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
		// 注意：信道分配放在这里很浪费时间，没有必要每个积分清除中断
		// 发生时就检查一次信道分配的情况，之所以放在这是考虑后处理时
		// 能够快速地遍历所有信道，实时使用时，函数本身需要重写，位置
		// 也要重新安放
		for (char ch = 0; ch <= chmax; ch++)
		{
			if (chan[ch].frame_ready == 1)
			{
				navmess(chan[ch].prn, ch);  // decode the navigation message
				chan[ch].frame_ready = 0;   // for this channel
			}
		}


		if (sec_flag == 1)//这是一个秒中断标志，用于维护系统接收时间和检查环路状态，在Sim_GPS_Interrupt里if (tic_count==0) sec_flag=1;		// one second has passed
		{
			almanac_flag = 0;

			thetime++;// 时间过去了1秒  问题：但是实际过去的应该不是完整的1秒，应该小于1秒？
			clock_tow = (++clock_tow) % 604800L;
			time_on++;// 计数器，步进为秒，程序不停，计数不止
			sec_flag = 0;
		}
		if (nav_flag == 1)// 计算导航解的时间到
		{

			if (_kbhit()) key = _getch();

			nav_fix();
			nav_flag = 0;

			display();

			send_sim_res_socket();
		}

		if (min_flag == 1)// 在热启动和导航模式下的信道分配函数
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
	// 释放堆上数组
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
	/*接收机标识*/
	send_res_L1.receiver_id = RECEIVER_L1;

	//抗干扰算法收敛时间只计算一次
	//send_res_L1.var1 = 0;

	send_res_L1.dim = m*n;

	//权值取当前处理的数据的最后一组
	if (CONFIG_STAP == (ANTI_CONFIG_SEL & sel))
	{
		
		/*float temp[4] = {};
		cudaMemcpy(temp, STAP_array_matrix_R_inver[0], 4 * sizeof(SIGNAL_TYPE),
			cudaMemcpyDeviceToHost);*/
		cudaMemcpy(send_res_L1.var2_real, STAP_array_matrix_R_inver[0],
			send_res_L1.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);
		
		//初始化时置为0
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
		//初始化时置为0
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
	printf("TOW  %6ld\n", clock_tow);                         //周内秒
	printf("meas time %f  error %f \n", m_time[1], m_error);   //                                                                                             [1]:接收时间。m_error：接收机的测量时间和GPS整秒的误差
	cur_lat.deg = int(rec_pos_llh.lat*r_to_d);
	cur_lat.min = int((rec_pos_llh.lat*r_to_d - cur_lat.deg) * 60);
	cur_lat.sec = float((rec_pos_llh.lat*r_to_d - cur_lat.deg - cur_lat.min / 60.)*3600.);
	cur_long.deg = int(rec_pos_llh.lon*r_to_d);
	cur_long.min = int((rec_pos_llh.lon*r_to_d - cur_long.deg) * 60);
	cur_long.sec = float((rec_pos_llh.lon*r_to_d - cur_long.deg - cur_long.min / 60.)*3600.);
	printf("   latitude    longitude          HAE      clock error (ppm)\n");
	printf("  %4d:%2d:%5.2f  %4d:%2d:%5.2f  %10.2f  %f\n",
		cur_lat.deg, abs(cur_lat.min), fabs(cur_lat.sec), cur_long.deg, abs(cur_long.min),
		fabs(cur_long.sec), rec_pos_llh.hae, clock_offset); //clock_offset:接收机钟漂
	printf(" Speed      vx		vy	  vz      Heading      TIC_dt\n");
	printf(" %lf   %lf   %lf   %lf   %lf   %lf\n", speed, rpvt.xv, rpvt.yv, rpvt.zv, heading*r_to_d, TIC_dt);  //speed:接收机速度的绝对值;heading：接收机的方位角；TIC_dt：两次计算导航解的间隔
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
				chan[ch].az*r_to_d, chan[ch].el*r_to_d,                                //az:卫星的方位角，el：卫星的仰角
				chan[ch].vDoppler, chan[ch].t_count, chan[ch].n_frame, chan[ch].sfid,    //vDoppler:由于运动产生的doppler分量;t_count:收到的比特计数，采用循环方式，从0-1499
				gps_eph[chan[ch].prn].ura, chan[ch].page5, chan[ch].CNo, (long)((long long)(chan[ch].phaseLockDetector + 0.5) & 0xffffffffUL));
			//ura：用户距离精度指数；page5：导航电文中的page号；CNO：估计的信道信噪比；phaseLockDetector：载波相位是否锁定的指示

			chinfo[ch].prn = chan[ch].prn;
			chinfo[ch].state = chan[ch].state;
			chinfo[ch].az = chan[ch].az;
			chinfo[ch].el = chan[ch].el;
			chinfo[ch].doppler = chan[ch].doppler;
			chinfo[ch].t_count = chan[ch].t_count;
			chinfo[ch].n_frames = chan[ch].n_frame;     // 已经收到的帧计数
			chinfo[ch].sfid = chan[ch].sfid;        // 子帧的标号
			chinfo[ch].ura = gps_eph[chan[ch].prn].ura;
			chinfo[ch].page = chan[ch].page5;
			chinfo[ch].CN0 = chan[ch].CNo;

			CNR[ch] = chinfo[ch].CN0;

			corrpeak[ch].i_early = chan[ch].i_early;
			corrpeak[ch].i_prompt = chan[ch].i_prompt;//相关峰值
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
				//修改全局的方位角和俯仰角
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
这个函数用我们的执行捕获跟踪的GPS函数代替了目前的IRQ0中断服务程序
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
			ReadAccumData(ch);//读了锁存的1毫秒的积分值
			/************************************************************************/
			correlator[ch].ready = 0;

			switch (chan[ch].state)
			{
			case acquisition:
				ch_acq(ch);
				break;
			case confirm:
				break;
			case pull_in:	// pull-in模块要完成位同步的过程
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
void ch_alloc()//一个测量中断进来一次
{
	char i, j;
	short allocFlag;

	almanac_valid = 1;// 检查Almanac是否有效
	for (i = MinSvNumber; i <= MaxSvNumber; i++)//共32个通道
	{
		xyz[i] = satfind(i);//在此ch_alloc函数之前有time(&thetime);
		if (gps_alm[i].inc>0.0 && gps_alm[i].week != gps_week % 1024)//判断历书有效性
		{
			almanac_valid = 0;
			break;
		}
	}
	if (al0 == 0.0 && b0 == 0.0) almanac_valid = 0;

	// 检查所有的卫星，如果某些卫星未被检测到信号，则这些卫星必须被闲置一段时间
	// 以保证其他的卫星有机会被纳入检测序列
	for (i = MinSvNumber; i <= MaxSvNumber; i++)
	{
		if (svStruct[i].state == HOLD)//HOLD是在fft过后没有直接捕获到峰值而设置的
		{
			if (svStruct[i].NumToTryAcq == 0)//捕获次数到了
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
		if (chan[i].state != CHN_OFF)// 如果信道已经检测到了信号/被占用，则跳过该信道的检测
		{
			continue;
		}

		// 为空闲信道选择一颗卫星
		// check all satellites
		allocFlag = 0;
		for (j = MinSvNumber; j <= MaxSvNumber; j++)
		{
			if (almanac_valid == 1)// 检查Almanac是否有效
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
					svStruct[j].NumToTryAcq = 20;//如果历书信息有，则每颗卫星的试捕次数为20次，否则只试捕一次，直到下次ch_alloc或者在跟踪里当载噪比<30时，设为20次
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

	chan[ch].ch_time++;//因为积分清除中断是1毫秒一次，所以一毫秒进这里一次
	// 如果tau非常接近于1023,则第一次IQ积分值很可能只包含了很少几个样点，因此
	// 噪声会很大，不可用。    因为在相关器里是判断tau的高32是否216888，而tau是捕获到的从初始码相位开始累加的，所以可能接近1023，而且在那-216888，所以下次进不会接近1023
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

		if (chan[ch].ch_time>PULL_IN_TIME) //#define PULL_IN_TIME (long) 1000		// FLL的收敛时间，单位：ms
		{
			freqErr = 0.0;
		}
		else
		{
			chan[ch].FLLIndex = 1 - chan[ch].FLLIndex;
			//先在ch_acq里置1，然后在这减变0，再下一次1-0又变1，然后再下次又变0，所以下面的if每隔一次进一次

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

			////保存伪距
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
