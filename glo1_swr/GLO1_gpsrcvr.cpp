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

GLO1_CHANNEL		   GLO1_chan[GLO1_chmax+1];				// 全程变量，用于存储每个通道跟踪环路的状态
GLO1_CORRELATOR	   GLO1_correlator[GLO1_chmax+1];		// 全程变量，用于存储每个通道相关器的状态
GLO1_SVSTRUCT	      GLO1_svStruct[GLO1_MaxSvNumber+1];	// 表示卫星可用性
long		      GLO1_globalBufLength[GLO1_chmax+1];	// 用于保存每个通道缓存的数据长度，需初始化
double  *GLO1_buffer[GLO1_chmax+1];			// 用于保存每个通道的缓存数据，需初始化分配空间
long		      GLO1_correlatorDataLength;	// 共有缓存区的数据长度
long		      GLO1_TIC_RT_CNTR;				// TIC实时计数器；
long		      GLO1_TIC_CNTR;					// TIC的最大计数，对应了TIC中断的发生周期
										         // 注意：GLO1_TIC_CNTR的赋值只能用GLO1_programTIC函数实现
double         GLO1_TIC_CNTR_RES;
double         GLO1_TIC_CNTR_DBL;
long		      GLO1_TIC_OCCUR_FLAG;			// TIC中断发生的标志

int			   GLO1_display_page=0;			// 控制显示第i个页面
unsigned	GLO1_test[16]=
			{	0x0001,0x0002,0x0004,0x0008,0x0010,0x0020,0x0040,0x0080,
				0x0100,0x0200,0x0400,0x0800,0x1000,0x2000,0x4000,0x8000};
unsigned int GLO1_tr_ch[13];					   // 索引0不用，内容为具有有效观测数据的GLO1_chan的标号
int			GLO1_out_debug,GLO1_out_pos,GLO1_out_vel,GLO1_out_time;		// 是否输出GLO1_debug和GLO1_PVT信息的标志符
GLO1_DMS			GLO1_cur_lat,GLO1_cur_long;			   // 用于显示经纬度

// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
int			   GLO1_nav_tic;						// 一个GLO1_nav_up的时间间隔对应的TIC中断的次数
int            GLO1_ICP_CTL=0;		         // 输出累积载波相位还是跟踪环路的值，标志
double         GLO1_ICP_code_corr;              // CA码速率的修正控制字，1.023MHz对应1.0
long           GLO1_time_on=0;	            // 计数器，步进为秒，程序不停，计数不止
// long		      GLO1_cc_scale=1540;       	// 载波标频(1575.42MHz)/CA码标频(1.023MHz)
long		      GLO1_cc_scale=3136;       	// 载波标频(1602+k*0.5625MHz)/CA码标频(0.511MHz)   todo  多颗星需要改变,但此变量后面没有使用

int GLO1_alloced_k[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};  // 记录某个 k 是否已经分配通道 k = -7 --- 6

double         GLO1_nav_up=1.0;					// 导航解的基本单位，单位：秒
double		   GLO1_speed,GLO1_heading;          // 接收机速度的绝对值和方位角
long		GLO1_d_tow;								// 接收机时间差，单位：秒
int			GLO1_key=0;								// 检查键盘的按键信息
int			GLO1_tic_count=0;						// TIC计数器，也是测量中断发生次数的计数器
												// 注意：测量中断的频率这里缺省为：100ms
												// 计数范围为1s[0-9]
int			GLO1_hms_count=0;						// TIC计数器，计数范围为1分钟[0-599]
int			GLO1_nav_count;							// TIC计数器，计数范围为[0,GLO1_nav_tic-1]
int			GLO1_min_flag;							// 分钟计数满标志
int			GLO1_nav_flag;							// 计算导航解标志
int			GLO1_sec_flag;							// 秒标志
int			GLO1_n_track;							// 跟踪(通道处于tracking模式)的卫星个数
unsigned int GLO1_interr_int=512;
double		GLO1_clock_offset=0.0;					// 接收机的钟漂，单位是ppm,初始值未知，假定为0
												// 正值表示接收机的频率低于标称值
GLO1_XYZ			GLO1_rec_pos_ecef;						// 接收机的坐标
long		GLO1_i_TIC_dt;							// 两次计算导航解的间隔,单位是样点个数
double		GLO1_TIC_dt;								// 两次计算导航解的间隔,单位是秒
												// 所以GLO1_TIC_dt=GLO1_i_TIC_dt*采样间隔
double		GLO1_m_time[3];							// 接收机时间,[1]是当前时刻的，[0]是上一次的
												// [2]似乎没有用
												// GLO1_m_time用于表示接收机的GPS时间
												// 在原来的程序中GLO1_m_time没有有效的更新,
												// 它应该靠秒中断标志GLO1_sec_flag来维护
double		GLO1_delta_m_time;						// 接收机时间的小数部分
double		GLO1_m_error;							// 接收机的测量时间和GPS整秒的误差
long		GLO1_TIC_sum;							// 多个测量中断中期内累积的计数值
//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
//int			mstat;							// 仅用于GP2021，包含有miss GLO1_data标志的寄存器
char		**GLO1_caTable;							// 非实时处理时，存储37×1023的CA码表

double		GLO1_DLLc1, GLO1_DLLc2;						// DLL的系数
double		GLO1_PLLc0, GLO1_PLLc1, GLO1_PLLc2;				// PLL的系数
double		GLO1_FLLa1, GLO1_FLLa2;						// FLL的系数

// 调试用的全程变量
double		*GLO1_IQQ, *GLO1_IQI;
double		GLO1_IBUF[3000], GLO1_QBUF[3000];
FILE		*GLO1_fpIQ;
long		GLO1_IQCounter;
FILE		*GLO1_fpobs;
FILE		*GLO1_fpeph;

long		GLO1_detectNum[GLO1_chmax+1];					// 搜索某颗卫星的持续时间
double		GLO1_threshold_sig_search;				// 信号检测门限							

//char		GLO1_data[GLO1_SAMPLESPMS+1];							// 信号缓存区
//char		GLO1_data[6000];							// 信号缓存区
double		GLO1_minFreqErr[GLO1_chmax+1],GLO1_maxFreqErr[GLO1_chmax+1];

unsigned long GLO1_uLastPCTick,GLO1_uCurrentPCTick;

__int64 GLO1_iTotalBitCounter=0;

bool GLO1_bLocalTimeSetByTow=false;
bool GLO1_bTowDecoded=false;

// 外部变量
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
float   GLO1_CARRIER_FREQ;   //标称中频  0625 //todo:GLONASS中心频率
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

int16s	GLO1_glonass_ephemeris_processing[12] = { -1, - 1,- 1, - 1, - 1, - 1, - 1, - 1, - 1, - 1, - 1, - 1 };		//初始化  -1
int32u *GLO1_glonass_frame_id = new int32u;
int16s GLO1_glonass_almanac_processing[12] = { -1, -1,-1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
int16s	GLO1_glonass_almanac_global_processing[12] = { -1, -1,-1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };	//初始化  -1
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

	GLO1_ADNumber = (int)pow(2.0,GLO1_ADNumber);//GLO1_ADNumber：输入的比特量化位数
	GLO1_carrier_ref = GLO1_CARRIER_FREQ;//中频频率

	GLO1_SAMPLING_INT = 1.0/GLO1_SAMPLING_FREQ;//GLO1_SAMPLING_FREQ:采样频率
	GLO1_DETECTSIZE = (long)((long long)(GLO1_SAMPLING_FREQ*0.01+0.5) & 0xffffffffUL);//10毫秒的数据点数
	
	GLO1_TIC_ref = (long)((long long)(GLO1_SAMPLING_FREQ*0.1+0.5-1) & 0xffffffffUL);// 100ms对应的中断计数值 100毫秒的数据点数  预设GLO1_TIC_CNTR的计数是100毫秒

	GLO1_deltaPhaseConst = GLO1_SAMPLING_INT*GLO1_carrierNCOMax;// GLO1_carrierNCOMax对应了载波DCO的实际分辨率2^30   它再乘以标称载波就变成标称载波频率字了
												 // GLO1_deltaPhaseConst就对应了求载波频率字的 2^30/采样频率
	GLO1_deltaCodePhaseConst = GLO1_SAMPLING_INT*0.511e6*GLO1_d_2p40;// 标称码频率*2^40/采样频率 = 码频率字
													  //GLO1_SAMPLING_INT*1.023e6*GLO1_d_2p40;

	GLO1_SAMPLESPMS = (long)((long long)(GLO1_SAMPLING_FREQ*0.001) & 0xffffffffUL);   //0627//lyq
	//jh GLO1_SAMPLESPMS每毫秒的数据点数

	GLO1_ACC_INT_PERIOD = GLO1_SAMPLESPMS/2+1;//+1之后虽然不是刚好的0.5毫秒，但是这只是读的点数，在数据处理的时候，还是按整毫秒的多少来取的，这里没影响
	//#define GLO1_ACC_INT_PERIOD  GLO1_SAMPLESPMS/2+1			// 0.5ms的样点数
	//jh GLO1_ACC_INT_PERIOD:每0.5毫秒积分（清零仍然是1毫秒），0.5毫秒的中断数
	GLO1_data = new char[GLO1_SAMPLESPMS+1];

	fopen_s(&GLO1_out_trtime, "tr_time.txt", "w");//保存结果
	fopen_s(&GLO1_daicy_file_pr, "daicy_pr.txt", "w");
	fopen_s(&GLO1_daicy_file_pos, "daicy_pos.txt", "w");
	

	
	float fnumber=(float)GLO1_SAMPLING_FREQ/500;//2毫秒的点数  因为要取2MS的本地伪码，为什么要2ms：GPS信号的CA码长1023码片，周期1ms ，但在给定采样率下，1ms内采到的点数不一定是2n
	
	int fftcount=0;
//lyq
	do
	{
		fnumber=fnumber/2;//fnumber是浮点型
		fftcount++;
	}while(fnumber>2);

	if(fnumber<=1.5)
		GLO1_FFTSIZE=(int)(pow(2.0,fftcount)*1.5);//与下面差别很大，是1.5倍 2的多少次方  但注意都是取整
	else 
		GLO1_FFTSIZE=(int)pow(2.0,fftcount+1);//不管是if else都肯定取的数大于2ms的点数

	GLO1_bufSize = (long)((long long)(GLO1_SAMPLING_FREQ*0.0005) & 0xffffffffUL);//0.5毫秒的数据点数，与GLO1_ACC_INT_PERIOD对应

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

	GLO1_read_rcvr_par(); //从rcvr_par文件读入控制捕获跟踪的接收机参数
	GLO1_rec_pos_xyz.x=0.0;// set up user position at earth center, if last valid position is
						// stored GLO1_in a file, the position will be updated
	GLO1_rec_pos_xyz.y=0.0;
	GLO1_rec_pos_xyz.z=0.0;

//准备输出文件
	if (GLO1_out_pos==1 || GLO1_out_vel==1 ||GLO1_out_time==1) 
		fopen_s(&GLO1_stream, "gpsrcvr.log","w");//gpsrcvr.log存放接收机的当前定位时间，纬经高，GLO1_XYZ,东北天速度，钟漂，H/V/TDOP
	if (GLO1_out_debug==1) 
		fopen_s(&GLO1_debug, "debug.log","w+");


	GLO1_read_initial_data();

	GLO1_initSignalSearch();// 为信号搜索动态分配数组

	//GLO1_current_loc=receiver_loc();
	GLO1_rec_pos_llh.lon=GLO1_current_loc.lon;// 检查保存的文件中是否存有接收机的位置
	GLO1_rec_pos_llh.lat=GLO1_current_loc.lat;
	GLO1_rec_pos_llh.hae=GLO1_current_loc.hae;
	
	// 设置navFix的更新时间间隔，
	GLO1_nav_tic= (int)(GLO1_nav_up/0.1+0.5);	// 对应了GLO1_nav_up周期的测量中断数：一秒10次，因为测量中断100毫秒一次，这里设为10，则共1000毫秒解算一次
									//这个量和测量中断时间一起决定解算的频率
									//测量中断一次就能得到一次伪距从而得到一次定位结果，这里1秒10次中断即可以最大1秒10次的刷新率，刷新率可设为1~10
	GLO1_programTIC(GLO1_TIC_ref);	// 等效为: GLO1_TIC_CNTR = GLO1_TIC_ref // TIC的最大计数，对应了TIC中断的发生周期// 注意：GLO1_TIC_CNTR的赋值只能用GLO1_programTIC函数实现
	
	for (ch=0;ch<=GLO1_chmax;ch++) 
    {
		GLO1_chan[ch].state = GLO1_CHN_OFF;//这个关闭状态才能通道分配
      GLO1_chan[ch].prn = 0;
    }
	for (ch=GLO1_MinSvNumber; ch<=GLO1_MaxSvNumber; ch++) 
	{
		GLO1_svStruct[ch].state = GLO1_AVAILABLE;//设置所有卫星均可用
		GLO1_svStruct[ch].undetectedCounter = 0;
		GLO1_svStruct[ch].NumBTWEachTrial = GLO1_ReAcqTime;//#define GLO1_ReAcqTime 1000000LTrial实验期   它就是重捕的门限，控制某颗星被重捕的时间
												 //NumBTWEachTrial：number between each Trial  BTW:between
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
	// 初始化信道的分配，冷启动和热启动的分配方式不同
	GLO1_hot_cold = 1;
	if (GLO1_hot_cold == 1)
	{
		GLO1_ch_alloc();
	} 
	else if (GLO1_hot_cold == 2)
	{
		GLO1_hot_ch_alloc();
	}
	// double	GLO1_m_time[3]; // 接收机时间,[1]是当前时刻的，[0]是上一次的 [2]似乎没有用 
	//GLO1_m_time用于表示接收机的GPS时间 在原来的程序中GLO1_m_time没有有效的更新,它应该靠秒中断标志GLO1_sec_flag来维护
	GLO1_m_time[1]=GLO1_clock_tow;	// 将估计的GPS秒赋值给接收机本地时间,注意这不是PC机的时间 

	GLO1_read_ephemeris();//从"current.eph"中读星历

#ifndef REAL_TIME
	GLO1_TIC_RT_CNTR = 0;	// simulator tic counter GLO1_in GLO1_correlator
	num = GLO1_ACC_INT_PERIOD;	// simulator acc_int counter GLO1_in GLO1_correlator
							//#define GLO1_ACC_INT_PERIOD  GLO1_SAMPLESPMS/2+1			// 0.5ms的样点数	
							//每次取0.5ms的样点数取做相关累加，但是每1ms才输出一次相关的结果，可以来一个点就累加，但是慢，若1ms累加又慢，所以这取0.5

	GLO1_initTrackLoopPar();//初始化载波环
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

	while (GLO1_key != 'x' && GLO1_key != 'X' && GLO1_receiver_buf.buf_remain_len >= num)  // 主循环开始
	{
#ifndef REAL_TIME

		//socket传输定位结果——抗干扰算法收敛时间
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
		// GLO1_TIC_CNTR+1是GLO1_TIC_RT_CNTR可以达到的最大值，达到后会立即置0
		// GLO1_TIC_RT_CNTR+num是到本次积分清除中断处理后，GLO1_TIC_RT_CNTR将会达到的值
		TIC_DIF = GLO1_TIC_CNTR+1-(GLO1_TIC_RT_CNTR+num);	
		//GLO1_TIC_RT_CNTR TIC实时计数器；
		//TIC_DIF就是为了计算num
		// 下次应该读取的样点个数，然后把这些点去做相关
		GLO1_receiver_init_num = num = TIC_DIF < GLO1_ACC_INT_PERIOD ? TIC_DIF : GLO1_ACC_INT_PERIOD;
		//保证它一段时间读上来的数据刚好凑成100毫秒，以完成100毫秒锁存

		GLO1_data = GLO1_receiver_buf.buf_current_ptr;
		//memcpy(GLO1_data, GLO1_receiver_buf.buf_current_ptr, tmp_num);

		GLO1_correlatorProcess(GLO1_data, tmp_num);// simulate GLO1_correlator processing  numRead = read_share_memory(GLO1_data, num);
		
		GLO1_receiver_buf.buf_remain_len -= tmp_num;
		GLO1_receiver_buf.buf_current_ptr += tmp_num;

		GLO1_Sim_GPS_Interrupt();	// simulate interrupt processing
#endif
		// 注意：信道分配放在这里很浪费时间，没有必要每个积分清除中断
		// 发生时就检查一次信道分配的情况，之所以放在这是考虑后处理时
		// 能够快速地遍历所有信道，实时使用时，函数本身需要重写，位置
		// 也要重新安放
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
				/*没加判断星历有效性*/

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
		

		if (GLO1_sec_flag==1)//这是一个秒中断标志，用于维护系统接收时间和检查环路状态，在Sim_GPS_Interrupt里if (GLO1_tic_count==0) GLO1_sec_flag=1;		// one second has passed
		{
			GLO1_almanac_flag=0;
			
			GLO1_thetime++;// 时间过去了1秒  问题：但是实际过去的应该不是完整的1秒，应该小于1秒？
			GLO1_clock_tow=(++GLO1_clock_tow)%604800L;
			GLO1_time_on++;// 计数器，步进为秒，程序不停，计数不止
			GLO1_sec_flag=0;
		} 
		if (GLO1_nav_flag==1)// 计算导航解的时间到
		{

			if (_kbhit()) GLO1_key = _getch();
			

			GLO1_nav_fix();
			GLO1_nav_flag=0;
			GLO1_display();

			GLO1_send_sim_res_socket();
		}
		
		if (GLO1_min_flag==1)// 在热启动和导航模式下的信道分配函数
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
	// 释放堆上数组
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
	/*接收机标识*/
	send_res_GLO1.receiver_id = RECEIVER_GLO1;

	//抗干扰算法收敛时间只计算一次
	//send_res_BD1.var1 = 0;

	send_res_GLO1.dim = m*n;

	//权值取当前处理的数据的最后一组
	if (CONFIG_STAP == (ANTI_CONFIG_SEL & sel))
	{
		cudaMemcpy(send_res_GLO1.var2_real, STAP_array_matrix_R_inver[0],
			send_res_GLO1.dim * sizeof(SIGNAL_TYPE), cudaMemcpyDeviceToHost);

		//初始化时置为0
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
		//初始化时置为0
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
	printf("TOW  %6ld\n",GLO1_clock_tow);                         //周内秒
	printf("meas time %f  error %f \n",GLO1_m_time[1],GLO1_m_error);   //GLO1_m_time[1]:接收时间。GLO1_m_error：接收机的测量时间和GPS整秒的误差
	GLO1_cur_lat.deg = int(GLO1_rec_pos_llh.lat*GLO1_r_to_d);
	GLO1_cur_lat.min = int((GLO1_rec_pos_llh.lat*GLO1_r_to_d-GLO1_cur_lat.deg)*60);
	GLO1_cur_lat.sec = float((GLO1_rec_pos_llh.lat*GLO1_r_to_d-GLO1_cur_lat.deg-GLO1_cur_lat.min/60.)*3600.);
	GLO1_cur_long.deg = int(GLO1_rec_pos_llh.lon*GLO1_r_to_d);
	GLO1_cur_long.min = int((GLO1_rec_pos_llh.lon*GLO1_r_to_d-GLO1_cur_long.deg)*60);
	GLO1_cur_long.sec = float((GLO1_rec_pos_llh.lon*GLO1_r_to_d-GLO1_cur_long.deg-GLO1_cur_long.min/60.)*3600.);
	printf("   latitude    longitude          HAE      clock error (ppm)\n");
	printf("  %4d:%2d:%5.2f  %4d:%2d:%5.2f  %10.2f  %f\n",
	GLO1_cur_lat.deg,abs(GLO1_cur_lat.min),fabs(GLO1_cur_lat.sec),GLO1_cur_long.deg,abs(GLO1_cur_long.min),
	fabs(GLO1_cur_long.sec),GLO1_rec_pos_llh.hae,GLO1_clock_offset); //GLO1_clock_offset:接收机钟漂
	printf(" Speed      vx		vy	  vz      Heading      TIC_dt\n");
	printf(" %lf   %lf   %lf   %lf   %lf   %lf\n", GLO1_speed, GLO1_rpvt.xv, GLO1_rpvt.yv, GLO1_rpvt.zv, GLO1_heading*GLO1_r_to_d, GLO1_TIC_dt);  //GLO1_speed:接收机速度的绝对值;GLO1_heading：接收机的方位角；GLO1_TIC_dt：两次计算导航解的间隔
	printf("   \n");

	printf("tracking %2d GLO1_status %1d almanac valid %1d gps week %4d\n",
	GLO1_n_track,GLO1_status,GLO1_almanac_valid,GLO1_gps_week%1024);

	if (GLO1_display_page==0)
	{
		//printf(" ch prn  state az  el  doppler  t_count n_frame sfid ura page CNo  PL\n");
		printf(" ch prn K  state  az  el  doppler  t_count n_frame sfid ura page CNo  PL\n");//todo:显示prn改为显示K
		for (ch=0;ch<=GLO1_chmax;ch++)
		{
			printf(" %2d %2d  %2d  %2d  %4.0f  %3.0f   %7.1f   %4d  %4d  %2d  %3d  %3d   %4.1f  %1d\n",
				//ch,GLO1_chan[ch].prn, GLO1_chan[ch].K, GLO1_chan[ch].state,
				ch, GLO1_chan[ch].prn, GLO1_chan[ch].K, GLO1_chan[ch].state, //todo:显示prn改为显示K
				GLO1_chan[ch].az*GLO1_r_to_d,GLO1_chan[ch].el*GLO1_r_to_d,                                //az:卫星的方位角，el：卫星的仰角
				GLO1_chan[ch].vDoppler,GLO1_chan[ch].t_count,GLO1_chan[ch].n_frame,GLO1_chan[ch].sfid,    //vDoppler:由于运动产生的doppler分量;t_count:收到的比特计数，采用循环方式，从0-1499
				GLO1_gps_eph[GLO1_chan[ch].prn].ura,GLO1_chan[ch].page5,GLO1_chan[ch].CNo, (long)((long long)(GLO1_chan[ch].phaseLockDetector+0.5) & 0xffffffffUL));
		       //ura：用户距离精度指数；page5：导航电文中的page号；CNO：估计的信道信噪比；phaseLockDetector：载波相位是否锁定的指示
			
			chinfo[ch].prn     = GLO1_chan[ch].prn;
			chinfo[ch].state   = GLO1_chan[ch].state;
			chinfo[ch].az      = GLO1_chan[ch].az;
			chinfo[ch].el      = GLO1_chan[ch].el;
			chinfo[ch].doppler = GLO1_chan[ch].doppler;
			chinfo[ch].t_count = GLO1_chan[ch].t_count;
			chinfo[ch].n_frames= GLO1_chan[ch].n_frame;     // 已经收到的帧计数
			chinfo[ch].sfid    = GLO1_chan[ch].sfid;        // 子帧的标号
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
				//修改全局的俯仰角和方位角
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
这个函数用我们的执行捕获跟踪的GPS函数代替了目前的IRQ0中断服务程序
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
			GLO1_ReadAccumData(ch);//读了锁存的1毫秒的积分值
				/************************************************************************/
			GLO1_correlator[ch].ready = 0;
			
			switch(GLO1_chan[ch].state)
			{
			case GLO1_acquisition:	
				GLO1_ch_acq(ch);
				break;
			case GLO1_confirm    :	
				break;
			case GLO1_pull_in    :	// pull-GLO1_in模块要完成位同步的过程
				GLO1_ch_pull_in(ch);
				break;
			case GLO1_track      :
				GLO1_ch_track(ch);
				break;
			}
		}
	}
	//在correlatorProcess里如果100ms到，GLO1_TIC_OCCUR_FLAG = 1;	// 设置测量中断标志，该标志在外部被中断服务程序清除
	if (GLO1_TIC_OCCUR_FLAG == 1)                  // 发生了一个测量中断,所有信道均同时产生
	{
		GLO1_tic_count=(++GLO1_tic_count)%10;
		if (GLO1_tic_count==0) GLO1_sec_flag=1;	

		GLO1_hms_count=(++GLO1_hms_count)%600;
		if (GLO1_hms_count==0) GLO1_min_flag=1;	

		GLO1_nav_count=(++GLO1_nav_count)%GLO1_nav_tic;	// GLO1_nav_tic= (int)(GLO1_nav_up/0.1+0.5);
		if (GLO1_nav_count==0) GLO1_nav_flag=1;		// 计算导航解的时间到

		GLO1_TIC_sum += GLO1_TIC_CNTR+1;						// 累计两次导航解之间的采样点总数
		add=1;

		// 码相位和秒计数不必每次都读，但整周计数必须保留，为什么？？？
		/************************************************************************/
		/*
		答：是因为相关器里100毫秒锁存了一次整周载波计数，所以也要在测量中断到的时候，
		取一次锁存整周载波计数，否则在一秒的时候读锁存的载波整周的话就不是1秒累加的，
		而是100毫秒累加的是这个原因吗？我觉得如果是这样，不用在相关器100ms锁存不就行了？
		直接在1秒到的时候直接取相关器里累加的整数周不就可以了？
		*/
		/************************************************************************/
		// 100毫秒读取一次整周计数，但是1秒才读一次码相位和秒计数
		// 载波相位的作用（整数和小数）是为了测量多普勒用
		for (ch=0;ch<=GLO1_chmax;ch++)
		{
			if (GLO1_chan[ch].state!=GLO1_track)//目前通道状态不是跟踪就不进行下面的读整数和小数载波相位
			{
				continue;
			}
			//如果通道状态是跟踪，就读捕获相关里的载波相位，整周和小数
			carrierPhase = GLO1_readCarrierPhase(ch);
			//hide by jh 4-9 carrierPhase = GLO1_readCarrierPhase(ch);//在相关器里累加了载波相位
			/************************************************************************/
			/*
			return (GLO1_correlator[ch].carrierCycleLatch//在相关器里for (i=0;i<=GLO1_chmax;i++){//每100ms锁存一次GLO1_correlator[i].carrierCycleLatch = GLO1_correlator[i].carrierCycle;// 实时的载波相位整周数
			+(double)GLO1_correlator[ch].phaseLocalLatch*carrierNCORes);//#define carrierNCORes (double) 9.31322574615478515625e-10  //2^-30
			//在相关器里for (i=0;i<=GLO1_chmax;i++){//每100ms锁存一次GLO1_correlator[i].phaseLocalLatch = GLO1_correlator[i].phaseLocal;// 实时的载波相位的小数部分
			//为什么小数部分要乘以2^-30即除以2^30？因为载波NCO是30位的，小数部分是30位，所以应当除以2^30
			*/
			/************************************************************************/
			//这里很傻逼，前面调用一个函数然后里面还运算来运算去得到一个实数，这里又费力去取整啊啥的又把实数分开，何不干脆在这里赋值？
			GLO1_chan[ch].cycle_sum += ((long) ((long long)carrierPhase & 0xffffffffUL));		
			GLO1_chan[ch].carr_dco_phase = fmod(carrierPhase,1.0);
		}

		if (GLO1_nav_count==0)//1秒钟到，该计算导航解了  为此要取测量数据              // 获取测量数据计算导航解
		{
			for (ch=0;ch<=GLO1_chmax;ch++)
			{
				if (GLO1_chan[ch].state!=GLO1_track)
				{
					continue;
				}
				GLO1_chan[ch].codePhase = GLO1_readCodePhase(ch);
				//如果通道状态是跟踪，则读取码相位和时间，多普勒等，用于计算伪距 以下左边的量基本都用于GLO1_nav_fix

				//hide by jh 4-9 GLO1_chan[ch].codePhase = GLO1_readCodePhase(ch);//return (double)GLO1_correlator[ch].tauLatch/(1.023e6*GLO1_d_2p40);
				//在相关器里if (GLO1_TIC_RT_CNTR == (GLO1_TIC_CNTR+1))即100ms到，GLO1_correlator[i].tauLatch = GLO1_correlator[i].tau;// 实时的码相位
				GLO1_chan[ch].epoch=GLO1_readEpoch(ch);//hide by jh 4-9 GLO1_chan[ch].epoch=GLO1_readEpoch(ch);//return GLO1_correlator[ch].epochCounter;
				//在相关器里if (GLO1_TIC_RT_CNTR == (GLO1_TIC_CNTR+1))即100ms到，GLO1_correlator[i].epochCounter = GLO1_correlator[i].epochCounterCheck;// 实时epoch计数器

				GLO1_chan[ch].meas_bit_time=GLO1_chan[ch].tr_bit_time;
				//在GLO1_ch_track里有pChan->tr_bit_time++;
				//在GLO1_pream里有GLO1_chan[ch].tr_bit_time=TOWs*300-240;
				GLO1_chan[ch].doppler=GLO1_chan[ch].fc;
				//在GLO1_ch_acq里有GLO1_chan[ch].fc = doppler;
				//在GLO1_ch_pull_in里有GLO1_chan[ch].fc += freqErr;//在GLO1_ch_acq里如果捕获到了则GLO1_chan[ch].fc = doppler;  把鉴出来的频率误差加上多普勒
				//在GLO1_ch_track里pChan->fc = *cLoopS1+GLO1_PLLc2*phaseErr;

				// GLO1_chan[ch].carrier_counter保存的是两次测量中断间，载波相位增量的整数部分
				// GLO1_chan[ch].d_carr_phase保存的是两次测量中断间，载波相位增量的小数部分
				GLO1_chan[ch].carrier_counter=GLO1_chan[ch].cycle_sum;//GLO1_chan[ch].cycle_sum += ((long)carrierPhase);
				GLO1_chan[ch].cycle_sum=0;
				GLO1_chan[ch].d_carr_phase=GLO1_chan[ch].carr_dco_phase - GLO1_chan[ch].old_carr_dco_phase;//GLO1_chan[ch].carr_dco_phase = fmod(carrierPhase,1.0);	// 读取载波相位的小数部分
				GLO1_chan[ch].old_carr_dco_phase=GLO1_chan[ch].carr_dco_phase;// 为什么要相减？这次100毫秒的小数载波数-上次100毫秒的小数载波数
				// 因为这次得到的小数载波数是在上次小数载波数上继续累加得到的，然后累加溢出，所以减去上次的小数载波数才是真正的自上一个
				// 100毫秒以来的小数部分
			}

			// 注意下面关于累加GLO1_i_TIC_dt的方法很不同
			// 在实时模式下，GLO1_nav_fix函数对GLO1_TIC_CNTR进行修改时，
			// 一个测量TIC过程正在进行，它会用旧的CNTR完成计数，而
			// 新的GLO1_TIC_CNTR只能从下一次才会被装入
			// 后处理时，TIC_CNRT立即装入
			/* hide by jh 3-12
			#ifdef REAL_TIME
			GLO1_i_TIC_dt= GLO1_TIC_sum+old_TIC_cntr-GLO1_TIC_CNTR;//在前面GLO1_TIC_sum += GLO1_TIC_CNTR+1;// 累计两次导航解之间的采样点总数
			//在main里old_TIC_cntr = GLO1_TIC_ref;// long GLO1_TIC_ref=(long)(GLO1_SAMPLING_FREQ*0.1+0.5-1); 100ms对应的中断计数值
			//在main里GLO1_programTIC(GLO1_TIC_ref);// 等效为: GLO1_TIC_CNTR = GLO1_TIC_ref
			//在GLO1_nav_fix里if (GLO1_align_t==1)//// 将接收机时间和GPS时间对准的标志{old_TIC_cntr=GLO1_TIC_CNTR;
			//// GLO1_i_TIC_dt是两次计算位置的间隔
			//在GLO1_nav_fix里GLO1_TIC_dt=GLO1_i_TIC_dt*GLO1_SAMPLING_INT;		//each clock count is 175 ns for GP2021
			#else
			*/
#ifdef REAL_TIME
#else
			GLO1_i_TIC_dt= GLO1_TIC_sum;//100毫秒的点数
							  /************************************************************************/
							  /*
							  // GLO1_i_TIC_dt是两次计算位置的间隔，是100毫秒的点数
							  GLO1_TIC_dt=GLO1_i_TIC_dt*GLO1_SAMPLING_INT;		//each clock count is 175 ns for GP2021
							  // GLO1_SAMPLING_INT是一个采样点要多少时间，乘以100毫秒的点数，得到100毫秒的点等价的时间
							  */
							  /************************************************************************/
							  //hide by jh 3-16#endif
#endif

			GLO1_TIC_sum=0;
		}
		GLO1_TIC_OCCUR_FLAG = 0;	//清除中断标志，等待下一次
		{
			GLO1_ch_alloc();//每100毫秒的测量中断发生则重新分配一次通道，因为经过了前面的捕获跟踪什么的，有的卫星状态有所改变
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

int GLO1_freq_alloc(char j) //为不同的星分配频道号（TODO）
{
	int K;
	switch (j)//定义频道号todo:
	{
	case 1:	  K = 1;	break;
	case 2:	  K = -4;	break;
	case 3:	  K = -5;	break;//与书上不一致，（TODO）
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
void GLO1_ch_alloc()//一个测量中断进来一次
{
	char i,j;
	short allocFlag;

	GLO1_almanac_valid=1;// 检查Almanac是否有效
	for (i=GLO1_MinSvNumber; i<=GLO1_MaxSvNumber; i++)//共32个通道
	{
		GLO1_xyz[i]=GLO1_satfind(i);//在此GLO1_ch_alloc函数之前有time(&GLO1_thetime);
		if (GLO1_gps_alm[i].inc>0.0 && GLO1_gps_alm[i].week!=GLO1_gps_week%1024)//判断历书有效性
		{
			GLO1_almanac_valid=0;
			break;
		}
	}
	if (GLO1_al0==0.0 && GLO1_b0==0.0) GLO1_almanac_valid=0;

	// 检查所有的卫星，如果某些卫星未被检测到信号，则这些卫星必须被闲置一段时间
	// 以保证其他的卫星有机会被纳入检测序列
	for (i=GLO1_MinSvNumber; i<=GLO1_MaxSvNumber; i++)
	{
		if (GLO1_svStruct[i].state==GLO1_HOLD)//GLO1_HOLD是在fft过后没有直接捕获到峰值而设置的
		{
			if (GLO1_svStruct[i].NumToTryAcq==0)//捕获次数到了
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
		if (GLO1_chan[i].state != GLO1_CHN_OFF)// 如果信道已经检测到了信号/被占用，则跳过该信道的检测
		{
			continue;
		}
		
		// 为空闲信道选择一颗卫星
		// check all satellites
		allocFlag = 0;
		for (j=GLO1_MinSvNumber; j<=GLO1_MaxSvNumber; j++)
		{
			if (GLO1_almanac_valid == 1)// 检查Almanac是否有效
			{
				if (GLO1_xyz[j].elevation > GLO1_mask_angle && 
					GLO1_gps_alm[j].health == 0 &&
					GLO1_gps_alm[j].ety != 0.00 &&
					GLO1_svStruct[j].state == GLO1_AVAILABLE)
				{
					allocFlag = 1;
					GLO1_svStruct[j].state = GLO1_USING;
					GLO1_chan[i].prn = j;
					GLO1_chan[i].K = GLO1_freq_alloc(GLO1_chan[i].prn);//分配频道号（todo）
					GLO1_chan[i].IF = GLO1_CARRIER_FREQ + GLO1_chan[i].K * 562500;//每个通道卫星的中频（todo）
					GLO1_chan[i].state = GLO1_acquisition;
					GLO1_svStruct[j].NumToTryAcq = 20;//如果历书信息有，则每颗卫星的试捕次数为20次，否则只试捕一次，直到下次GLO1_ch_alloc或者在跟踪里当载噪比<30时，设为20次
					GLO1_svStruct[j].NumBTWEachTrial = 1000;
					GLO1_correlator[i].state = GLO1_CHN_OFF;
					GLO1_correlator[i].ready = 1;
					GLO1_ch_cntl(i,j);//i为通道号，j为卫星号
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
					GLO1_chan[i].IF = GLO1_CARRIER_FREQ + GLO1_chan[i].K * 562500;//每个通道卫星的中频（todo）
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
	
	GLO1_chan[ch].ch_time++;//因为积分清除中断是1毫秒一次，所以一毫秒进这里一次
	// 如果tau非常接近于1023,则第一次IQ积分值很可能只包含了很少几个样点，因此
	// 噪声会很大，不可用。    因为在相关器里是判断tau的高32是否216888，而tau是捕获到的从初始码相位开始累加的，所以可能接近1023，而且在那-216888，所以下次进不会接近1023
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
		GLO1_chan[ch].carrier_freq = GLO1_chan[ch].IF+ GLO1_chan[ch].fc;//todo:GLO1_carrier_ref改为GLO1_chan[ch].IF
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
		
		if (GLO1_chan[ch].ch_time>GLO1_PULL_IN_TIME)	// FLL的收敛时间，单位：ms
		{
			freqErr = 0.0;
		}
		else
		{
			GLO1_chan[ch].FLLIndex = 1-GLO1_chan[ch].FLLIndex;
			//先在GLO1_ch_acq里置1，然后在这减变0，再下一次1-0又变1，然后再下次又变0，所以下面的if每隔一次进一次
		
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
		GLO1_chan[ch].carrier_freq = GLO1_chan[ch].IF + GLO1_chan[ch].fc;//todo:GLO1_carrier_ref 改为GLO1_chan[ch].IF
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
			//printf("位同步进入GLO1_pream\n");
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
	
	T = 1e-3;			// 这里控制环路的速率是1KHz,即每来一组IQ积分值
						// 环路控制参数就要重新计算一次，这不是一种必须的
						// 选择，但在pull-GLO1_in状态下，高的更新速率有利于环路的
						// 快速收敛		

	// PLL discriminator
    if (fabs(sPR)<1e-3)
    {
        phaseErr = 0.0;
    }
    else
        phaseErr = atan(sPI/sPR);

	////保存载波相位误差
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
	pChan->carrier_freq = GLO1_chan[ch].IF + pChan->fc;//GLO1_carrier_ref改为GLO1_chan[ch].IF
	GLO1_writeCarrierFreq(ch,pChan->carrier_freq);

	// ms计数(模20，1bit的时间)  进一次跟踪就自加一次
	//pChan->ms_count=(++pChan->ms_count)%20;
	pChan->ms_count = (++pChan->ms_count) % 10; //todo
	// long  ms_count; // 解调1bit时用的ms计数器(模20)，计满后将所有IQ值累加再解调
	// bit内的积分值累加

	pChan->i_prom_20 += pChan->i_prompt;
	pChan->q_prom_20 += pChan->q_prompt;
	pChan->i_early_20 += pChan->i_early;
	pChan->q_early_20 += pChan->q_early;
	pChan->i_late_20 += pChan->i_late;
	pChan->q_late_20 += pChan->q_late;

	/************************************************************************/
	/*在GLO1_bitSync里循环累加用与算信噪比，但是最后都清零了 long  q_prom_20,i_prom_20; // 20msIQ通道值的和
	GLO1_chan[ch].i_prom_20 += sPR;
	GLO1_chan[ch].q_prom_20 += sPI;
	*/
	/************************************************************************/
	
	// 估计信噪比的中间步骤
	pChan->WBP += sPR*sPR+sPI*sPI;

	//if(pChan->ms_count==19)
	if(pChan->ms_count==9)
	{
		
		//T = 0.02;	
		T = 0.01;	//码环更新环路时间
		// update DLL
		sPI20 = pChan->q_prom_20;
		sPR20 = pChan->i_prom_20;
		sLI20 = pChan->q_late_20;
		sLR20 = pChan->i_late_20;
		sEI20 = pChan->q_early_20;
		sER20 = pChan->i_early_20;

		//codeErr = (sER20*sPR20+sEI20*sPI20)/(sER20*sER20+sPR20*sPR20+sEI20*sEI20+sPI20*sPI20)*(GLO1_DLLdT/(2.0*1.023e6));
		codeErr = (sER20*sPR20+sEI20*sPI20)/(sER20*sER20+sPR20*sPR20+sEI20*sEI20+sPI20*sPI20)*(GLO1_DLLdT/(2.0*0.511e6));

		////保存伪码相位误差
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
		//*dllS1 = *dllS0 + GLO1_DLLc2*codeErr + pChan->fc/(1602.0e6-7*0.5625e6);//todo:需要修改
		//*dllS1 = *dllS0 + GLO1_DLLc2*codeErr + pChan->fc / (1602.0e6 +1 * 0.5625e6);
		*dllS1 = *dllS0 + GLO1_DLLc2*codeErr + pChan->fc / (1602.0e6 + GLO1_chan[ch].K* 0.5625e6);//todo
        pChan->code_freq = 1+(*dllS1);
		GLO1_writeCodeFreq(ch,pChan->code_freq);


		pChan->NBP = sPI20*sPI20 + sPR20*sPR20;
		pChan->NP += (0.02 * pChan->NBP/pChan->WBP);//0.02是1/K，K为50

		pChan->WBP = 0.0;

		// 估计载波相位的锁定指示
		pChan->NBD = sPR20*sPR20 - sPI20*sPI20;
		pChan->phaseLockDetector = pChan->NBD/pChan->NBP;

		//if (pChan->trackTime%1000==0 && pChan->trackTime>0)
		if (pChan->trackTime%500==0 && pChan->trackTime>0)
		{
			// 估计信噪比
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
            GLO1_svStruct[pChan->prn].NumToTryAcq = 20;//因为已经经过了牵引才到的这，只是信噪比比较低，但卫星存在，可以再去捕20次
            GLO1_svStruct[pChan->prn].NumBTWEachTrial = 1000;//重捕间隔缩短为1000次
            GLO1_svStruct[pChan->prn].maxf=(pChan->fc+GLO1_DELTF);//利用已经获得的信息，所以捕获范围缩小到已经跟到的频率+-一个间隔
            GLO1_svStruct[pChan->prn].minf=(pChan->fc-GLO1_DELTF);
            GLO1_correlator[ch].state = GLO1_CHN_OFF;//重新捕获，不开对应通道的相关器
            GLO1_correlator[ch].ready = 1;//载噪比太低，则？因为需要重新捕获，所以设为1，可以进入sim_GPS_Interrupt的case里
         }
		}
			
		// 1bit时间到，可以解调该数据
		pChan->tr_bit_time++;//当前bit数加1
		absAngle = fabs(atan2((double)pChan->q_prom_20, (double)pChan->i_prom_20));
		//pChan->bit = absAngle>3.1415926/2 ? 0 : 1;
		pChan->bit = absAngle>GLO1_halfPi ? 0 : 1;

		GLO1_pream(ch,pChan->bit);  // 每2毫秒调用一次see if we can find the preamble

		 // 缓存解调数据
		pChan->message[pChan->t_count++]=pChan->bit;

		pChan->i_prom_20 = 0;
		pChan->q_prom_20 = 0;
		pChan->i_early_20 = 0;
		pChan->q_early_20 = 0;
		pChan->i_late_20 = 0;
		pChan->q_late_20 = 0;
	}


	if (pChan->t_count==3000)//一个主帧包含5个子帧，一个主帧包含1500bit// t_count收到的比特计数，采用循环方式，从0-1499
	{
		pChan->n_frame++;//主帧（5个子帧）
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
		bit=GLO1_chan[ch].epoch>>8;	// counter of bit (0 - 50) 一秒1000ms，20ms一比特，一秒解算一次，故50比特

		chip = GLO1_chan[ch].codePhase;
		//在sim_gps_interrupt里如果1秒到，则GLO1_chan[ch].codePhase = GLO1_readCodePhase(ch);//return (double)GLO1_correlator[ch].tauLatch/(1.023e6*GLO1_d_2p40);
		//因为码NCO是40位的，向高位溢出整数码片，除以2^40是因为相当于左移了40位，而高位是0~1023的，除以1023000，相当于0~1毫秒，1毫秒是一个码周期

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
		////保存伪距
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
			 //ms/1000.0是小数比特计的时间部分，精度1毫秒	
			 //chip是chip = GLO1_chan[ch].codePhase;  现在的码相位换算成的时间，精度1/1023毫秒
			 //在sim_gps_interrupt里如果1秒到，则GLO1_chan[ch].codePhase = GLO1_readCodePhase(ch);//return (double)GLO1_correlator[ch].tauLatch/(1.023e6*GLO1_d_2p40);
			 //因为码NCO是40位的，向高位溢出整数码片，除以2^40是因为相当于左移了40位，而高位是0~1023的，除以1023000，相当于0~1毫秒，1毫秒是一个码周期

			////保存伪距
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
	// GLO1_SAMPLING_INT是一个采样点要多少时间，乘以1000毫秒的点数，得到1000毫秒的点等价的时间
	if (GLO1_out_debug) fprintf(GLO1_debug,"n_track= %d\n",GLO1_n_track);

	// 暂时用于后处理方式的本地GPS时间初始化，估计一个粗略的接收机的接收时间
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
      else if (GLO1_bTowDecoded)//在GLO1_pream里if (sfid_s==1)GLO1_bTowDecoded=true;
      {
         GLO1_m_time[1]=GLO1_clock_tow;
         GLO1_bLocalTimeSetByTow=true;
      }
      else
         GLO1_m_time[1]+=GLO1_nav_up;//反正只要进来一次，且不是跟踪，就加一秒
	}
	else if (GLO1_towFlag == 1)
      GLO1_m_time[1]+=GLO1_nav_up;

	////保存伪距
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
		
		if(GLO1_ICP_CTL==0)   //多普勒 satellite GLO1_velocity  
		{
			dm_gps_sat[i]=GLO1_satpos_ephemeris(tr_time[i]-GLO1_TIC_dt/2.0,GLO1_chan[GLO1_tr_ch[i]].prn);
			dp_gps_sat[i]=GLO1_satpos_ephemeris(tr_time[i]+GLO1_TIC_dt/2.0,GLO1_chan[GLO1_tr_ch[i]].prn);
			/************************************************************************/
			/*
			// GLO1_i_TIC_dt是两次计算位置的间隔，是1000毫秒的点数
			GLO1_TIC_dt=GLO1_i_TIC_dt*GLO1_SAMPLING_INT;		//each clock count is 175 ns for GP2021
			// GLO1_SAMPLING_INT是一个采样点要多少时间，乘以1000毫秒的点数，得到1000毫秒的点等价的时间
			*/
			/**********************************/
		}
		else//整周载波相位  这种的算速度的效果比前一种好  而且算多普勒也比载波环的效果好
		{
		    dm_gps_sat[i]= GLO1_glonass_process_sat_pvt_L(GLO1_chan[GLO1_tr_ch[i]].K, &GLO1_glonass_sv_id_ephemeris[GLO1_chan[GLO1_tr_ch[i]].K + 10],
				&GLO1_glonass_sv_id_almanac_str5[GLO1_chan[GLO1_tr_ch[i]].K + 10],
				tr_time[i] - GLO1_TIC_dt);
				//GLO1_satpos_ephemeris(tr_time[i]-GLO1_TIC_dt,GLO1_chan[GLO1_tr_ch[i]].prn); 
		    dp_gps_sat[i]=GLO1_track_sat[i];

		}
		// 计算该颗卫星的校正项
		GLO1_t_cor[i] = -GLO1_tropo_iono(GLO1_tr_ch[i], GLO1_track_sat[i].az, GLO1_track_sat[i].el, tr_time[i]);
		// GLO1_dt是包含“接收机钟差”的伪距
		// GLO1_track_sat是卫星的位置
		// 这两个变量均为全程数组，用于GLO1_pos_vel_time(*)函数

		
		GLO1_dt[i] = GLO1_m_time[1] - tr_time[i] + GLO1_track_sat[i].tb; //(tr_time[i]-t_cor[i])+const_pram;//伪距
		GLO1_lambda[i] = GLO1_c / (1602e6 + GLO1_chan[GLO1_tr_ch[i]].K * 562500);


		// 计算卫星的速度：卫星的位置差＋地球自转的修正项
		GLO1_d_sat[i].x=(dp_gps_sat[i].x-dm_gps_sat[i].x)/GLO1_TIC_dt-GLO1_track_sat[i].y*GLO1_omegae;
		GLO1_d_sat[i].y=(dp_gps_sat[i].y-dm_gps_sat[i].y)/GLO1_TIC_dt+GLO1_track_sat[i].x*GLO1_omegae;
		GLO1_d_sat[i].z=(dp_gps_sat[i].z-dm_gps_sat[i].z)/GLO1_TIC_dt;



		// process Carrier Tracking Loop or Integrated Carrier Phase
		// 下面两行是计算多普勒频率，但具体的数字都只在GP2021芯片中有效
		// 例如：33010105L)*42.574746268e-3 = 1.405396826e6 = GP2021中频
		// 为了让程序更通用，特改为：
		// if (GLO1_ICP_CTL==0) GLO1_meas_dop[i]=(GLO1_chan[GLO1_tr_ch[i]].doppler-33010105L)*42.574746268e-3;  // for CTL
		// else GLO1_meas_dop[i]= GLO1_chan[GLO1_tr_ch[i]].int_carr_phase/GLO1_TIC_dt-1.405396826e6; // for ICP
		if (GLO1_ICP_CTL==0) GLO1_meas_dop[i]=GLO1_chan[GLO1_tr_ch[i]].doppler;  
		//double		GLO1_meas_dop[13];							// 测量的每个通道的多普勒
		else GLO1_meas_dop[i] = GLO1_chan[GLO1_tr_ch[i]].int_carr_phase / GLO1_TIC_dt -(GLO1_CARRIER_FREQ + GLO1_chan[GLO1_tr_ch[i]].K * 562500);
		//GLO1_chan[ch].int_carr_phase=GLO1_chan[ch].carrier_counter + GLO1_chan[ch].d_carr_phase;//整数载波相位+小数载波相位
		//（整周载波+小数载波）/产生这么多载波的时间 - 标称载波 
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
		//clock_error_in_Hz = clock_error * (1602+ GLO1_chan[GLO1_tr_ch[i]].K* 0.5625);//1575.42;// 1575,42Mhz,即一秒1575.42M个载波周期，*一个周期偏移多少
		GLO1_m_time[1]=GLO1_m_time[1]-GLO1_cbias;			// 修正接收机的时钟
		rp_ecef.x=GLO1_rpvt.x;
		rp_ecef.y=GLO1_rpvt.y;
		rp_ecef.z=GLO1_rpvt.z;
		GLO1_rp_llh=GLO1_ecef_to_llh(rp_ecef);	// 将GLO1_xyz坐标转换为经纬高坐标

		GLO1_wgs[0] = GLO1_rpvt.x;
		GLO1_wgs[1] = GLO1_rpvt.y;
		GLO1_wgs[2] = GLO1_rpvt.z;
		////保存伪距
		//char str[10];
		////itoa(sv, str, 10);
		//sprintf(str, "%d", 1);
		//char s[100] = "recv_";
		//char *pfilename = strcat(s, str);

		//FILE *fs = fopen(pfilename, "a");
		//fprintf(fs, " %10.9f %10.9f %10.9f %10.9f\n", GLO1_rpvt.x, GLO1_rpvt.y, GLO1_rpvt.z, GLO1_rpvt.df);
		//fclose(fs);
		::PostMessage(NavOutWnd, WM_NAVIGATION_OUT, (WPARAM)GLO1_wgs, 1);

		// vDoppler是由接收机和卫星相对运动引起的doppler
		for (ch=0;ch<=GLO1_chmax;ch++)
		{
			GLO1_chan[ch].vDoppler = 0.0;//double vDoppler; // doppler中由于运动产生的doppler分量
		}
		for (ch=1;ch<=GLO1_n_track;ch++)
		{
			//GLO1_chan[GLO1_tr_ch[ch]].vDoppler = GLO1_meas_dop[ch]-clock_error_in_Hz;
			GLO1_chan[GLO1_tr_ch[ch]].vDoppler = GLO1_meas_dop[ch] - clock_error * (1602 + GLO1_chan[GLO1_tr_ch[ch]].K* 0.5625);

		}
		if (GLO1_rp_llh.hae>-2000.0 && GLO1_rp_llh.hae< 28000 && GLO1_pos_out_flag >=0) //modified by daicy
		{
			GLO1_velocity();		// 将速度矢量从WGS84转换到ENU坐标系

			if (GLO1_speed < 514.0)
			{
				if (fabs(clock_error)<5.0) GLO1_clock_offset=clock_error;
				if (GLO1_almanac_valid==1) GLO1_status=GLO1_navigating;

				// 以下这个条件是指要将接收机的测量时刻和卫星发射时刻对准
				// 是否是这个意思还需要进一步验证
				if (GLO1_align_t==1)//// 将接收机时间和GPS时间对准的标志
				{
					// PC接收时刻(以GPS秒表示)的小数部分
					GLO1_delta_m_time= modf(GLO1_m_time[1],&ipart);//GLO1_delta_m_time返回GLO1_m_time[1]的小数部分，ipart指向GLO1_m_time[1]的整数部分
					//The modf function breaks down the floating-point value x into fractional and integer parts,
					// each of which has the same GLO1_sign as x. The signed fractional portion of x is returned. 
					//The integer portion is stored as a floating-point value at intptr.
					// GLO1_m_error是相对于GLO1_nav_up时刻的误差秒数
					if (GLO1_nav_up<1.0)
					{
						GLO1_delta_m_time=modf(GLO1_delta_m_time/GLO1_nav_up,&ipart);//GLO1_delta_m_time/GLO1_nav_up是每秒的误差秒数
						//double		GLO1_m_error; // 接收机的测量时间和GPS整秒的误差
						if (GLO1_delta_m_time>0.5) GLO1_m_error=(GLO1_delta_m_time-1.0)*GLO1_nav_up;//每次更新时候观测时间相对于GLO1_nav_up时刻的误差秒数
						else GLO1_m_error=GLO1_delta_m_time*GLO1_nav_up;
					}
					else
					{
	//为什么这里不用modf，因为本身GLO1_nav_up已经大于等1秒，到这来的小数GLO1_delta_m_time已经是相对于1秒来讲的单位了
						if (GLO1_delta_m_time>0.5) GLO1_m_error=(GLO1_delta_m_time-1.0)/GLO1_nav_up;//每次更新时候观测时间相对于GLO1_nav_up时刻的误差秒数
						else GLO1_m_error=GLO1_delta_m_time/GLO1_nav_up;
					}

#ifndef REAL_TIME
					// 以下对GLO1_TIC_CNTR的编程只适合于后处理，实时应用时，需要修正
					// 因为新的GLO1_TIC_CNTR不是即时装入的，而是等现在的TIC计数过程结束后才装入的
					// tic_CNTR取的值是要使下次GLO1_nav_up发生时，接收机的接收时刻基本可以对准
					// tic_CNTR = (long)(GLO1_TIC_ref*(GLO1_nav_up-GLO1_m_error)/(1.0+GLO1_clock_offset*1.0e-6)+0.5);
					//hide by jh 4-14 tic_CNTR = GLO1_TIC_ref*(1.0-GLO1_m_error/GLO1_nav_up)/(1.0+GLO1_clock_offset*1.0e-6);
					//double GLO1_clock_offset=0.0; 	// 接收机的钟漂，单位是ppm,初始值未知，假定为0
					tic_CNTR = GLO1_TIC_ref*(1.0-GLO1_m_error/GLO1_nav_up)/(1.0+GLO1_clock_offset*1.0e-6);
#endif
					//这里暂时按照一个钟差去调整每次读取的数据长度
					//hide by jh 4-2 GLO1_programTIC(tic_CNTR);//GLO1_TIC_CNTR = GLO1_data;long GLO1_TIC_CNTR;	 // TIC的最大计数，对应了TIC中断的发生周期
					//调整了积分清除结果的锁存时间，原本的是100毫秒
					//每次GLO1_nav_fix都会更新这个值
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
				
				GLO1_dops(GLO1_n_track);     // 计算DOP   
				GLO1_m_time[0]=GLO1_m_time[1];
			}
		}
		GLO1_pos_out_flag++;
		
	}
	else	// 如果卫星的总数少于4，则只可能尽量保持接收机时钟的准确性
	{                              
		// 用已知的钟漂来修正时间，注意如果长时间不能定位，接收机的时钟
		// 将会漂移
		// 注意：这里的调整方法和上面不同，GLO1_TIC_CNTR没有改变，原来的公式似乎有误
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
		
			////保存伪距
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

/*解曲码和相对码*/
void GLO1_DeMeander(int ch)
{
	char meand[170]={0};//将fifo1~fifo6中170bit存放到数组里，方便解码
	unsigned long  mask = 0x20000000L;
	int index=0;
	for(int i=0;i<30;i++){
		if(GLO1_chan[ch].fifo1&mask){
			if (GLO1_chan[ch].reverseflag) {  //reverseflag=1为反相，reverseflag=0为正相
				meand[index++] = 0;//该通道为反相码
			}
			else {
				meand[index++] = 1;
			}
		}else{
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//该通道为反相码
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
				meand[index++] = 0;//该通道为反相码
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//该通道为反相码
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
				meand[index++] = 0;//该通道为反相码
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//该通道为反相码
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
				meand[index++] = 0;//该通道为反相码
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//该通道为反相码
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
				meand[index++] = 0;//该通道为反相码
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//该通道为反相码
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
				meand[index++] = 0;//该通道为反相码
			}
			else {
				meand[index++] = 1;
			}
		}
		else {
			if (GLO1_chan[ch].reverseflag) {
				meand[index++] = 1;//该通道为反相码
			}
			else {
				meand[index++] = 0;
			}
		}
		mask = mask>>1;
	}

	char relative[85]={0};//存放曲码解码之后的数
	for(int i=0,j=0;i<170;i=i+2){
		relative[j++] = meand[i];//曲码解码，01->0;10->1
	}

	char buf[85] = {0};//存放相对码解码之后的数
	buf[0] = relative[0];
	for(int i=1;i<85;i++){
		buf[i] = relative[i-1]^relative[i];//相对码解码
	}
	index = 0;
	GLO1_chan[ch].buf0 = 0L;//将相对码解码之后的数85bit放到long int 里
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
	//	GLO1_chan[ch].buf2 = (GLO1_chan[ch].buf2 << 4) + buf[index++];//放30~5
	}
}


double GLO1_ReadComplement(unsigned long para, int firstbit)
{
	double res = 0;
	if ((para >> (firstbit - 1)) == 1)
	{
		//负数
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
		//正数
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

int GLO1_glonass_verify_data(int ch)//校验
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
			flag = 0;		//0:正确;		1:已修正；	2:舍弃;		3:其他
		else if(c_sum >=1)
			flag = 2;			
	}
	else if(c_sigma ==1)
	{
		if(c_sum == 6)
			flag = 0;
		else if(c_sum >=2)
		{
			c_comb = (c7<<6)|(c6<<5)|(c5<<4)|(c4<<3)|(c3<<2)|(c2<<1)|c1;	//修正误bit

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
const unsigned GLO1_pream1 = 0x1c8af69; //00 0001 1100 1000 1010 1111 0110 1001 反相码
void GLO1_pream(char ch,char bit)
{ 

	//sta-tic unsigned long GLO1_pream=0x22c00000L;//0010 0010 1100 0000 0000 0000
	//stat-ic unsigned long GLO1_pream = 0x3e375096;   //11 1110 0011 0111 0101 0000 1001 0110
	//sta-tic const unsigned pream1 = 0x1c8af69; //00 0001 1100 1000 1010 1111 0110 1001 反相码
	unsigned long  parity0,parity1;
	unsigned long TimeMark;//时间志
	unsigned long HOWLastTwo;
	int sfid_s;
	long currentBitPos, frameLength;

	// fifo1和fifo0的设计如下：
	// fifo0用来存放先到的字，fifo1用来存放后到的字
	// fifo0的低30位为一个有效字，高2位是前一帧的校验bit：D29,D30
	// fifo1的低30位为一个有效字，高2位是前一帧的校验bit：D29,D30和fifo0的最后2比特相同

	// fifo1的最高位将移至fifo0的最低位

	if (GLO1_chan[ch].fifo1 & 0x20000000L) //
	{
		GLO1_chan[ch].fifo0=(GLO1_chan[ch].fifo0<<1)+ 1;//fifo0的第1位不是没了？
	}
	else
	{
		GLO1_chan[ch].fifo0=GLO1_chan[ch].fifo0<<1;
	}


	if (GLO1_chan[ch].fifo2 & 0x20000000L) //
	{
		GLO1_chan[ch].fifo1=(GLO1_chan[ch].fifo1<<1)+ 1;//fifo0的第1位不是没了？
	}
	else
	{
		GLO1_chan[ch].fifo1=GLO1_chan[ch].fifo1<<1;
	}

	if (GLO1_chan[ch].fifo3 & 0x20000000L) //todo
	{
		GLO1_chan[ch].fifo2=(GLO1_chan[ch].fifo2<<1)+ 1;//fifo0的第1位不是没了？
	}
	else
	{
		GLO1_chan[ch].fifo2=GLO1_chan[ch].fifo2<<1;
	}

	if (GLO1_chan[ch].fifo4 & 0x20000000L) //todo
	{
		GLO1_chan[ch].fifo3=(GLO1_chan[ch].fifo3<<1)+ 1;//fifo0的第1位不是没了？
	}
	else
	{
		GLO1_chan[ch].fifo3=GLO1_chan[ch].fifo3<<1;
	}

	if (GLO1_chan[ch].fifo5 & 0x20000000L) //todo
	{
		GLO1_chan[ch].fifo4=(GLO1_chan[ch].fifo4<<1)+ 1;//fifo0的第1位不是没了？
	}
	else
	{
		GLO1_chan[ch].fifo4=GLO1_chan[ch].fifo4<<1;
	}


	if (GLO1_chan[ch].fifo6 & 0x80000L) //todo
	{
		GLO1_chan[ch].fifo5=(GLO1_chan[ch].fifo5<<1)+ 1;//fifo0的第1位不是没了？
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

	GLO1_chan[ch].fifo7=(GLO1_chan[ch].fifo7<<1)+bit;// 新的bit放在fifo7的最低位

	// 30 位有效用于判断同步
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

	//GLO1_bitSync里GLO1_chan[ch].subFrameSyncFlag = GLO1_SUBFRAME_SYNCHRONIZING;
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
					GLO1_chan[ch].reverseflag = 1;//表示反相码同步
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

					//	GLO1_bTowDecoded = true;//后加的


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


						GLO1_bTowDecoded = true;//后加的
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
				GLO1_chan[ch].reverseflag = 1;//表示反相码同步
			}else {
				GLO1_chan[ch].reverseflag = 0;
			}

			GLO1_DeMeander(ch);//解曲码和相对码

			// check and GLO1_handle 1 bit error
			int flag = GLO1_glonass_verify_data(ch);
			GLO1_chan[ch].buf2 = GLO1_chan[ch].buf2 << 5;
			//printf("fisrt flag:  %d\n",flag);
			if(flag == 2 || flag ==3){
				GLO1_chan[ch].subFrameSyncFlag = GLO1_SUBFRAME_SYNCHRONIZING;
			}else {
				sfid_s = (GLO1_chan[ch].buf0&0x1e000000L)>>25; //读取串号

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

				//		GLO1_bTowDecoded = true;//后加的
				//}
				if (sfid_s>1 && sfid_s<15){
						
						GLO1_chan[ch].offset=GLO1_chan[ch].t_count-199-(sfid_s-1)*200;
						GLO1_writeEpoch(ch,(0x1f&GLO1_readEpochCheck(ch))|0x000);
						if (GLO1_chan[ch].offset<0.0){
							GLO1_chan[ch].offset+=3000;
						}
						GLO1_chan[ch].tow_sync=1;
						GLO1_chan[ch].sfid=sfid_s;	

						GLO1_bTowDecoded = true;//后加的
			} 
			}
		}
	}	
}