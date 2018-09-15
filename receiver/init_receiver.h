#pragma once

#include "../gps_swr/gpsconst.h"
#include "../bd_swr/BD1_gpsconst.h"
#include "../bd3_swr/BD3_gpsconst.h"
#include "../glo1_swr/GLO1_gpsconst.h"
#include "../gps_swr/gpsstruct.h"
#include "../bd_swr/BD1_gpsstruct.h"
#include "../bd3_swr/BD3_gpsstruct.h"
#include "../glo1_swr/GLO1_gpsstruct.h"
#include <stdio.h>
#include <string>
//#include <windows.h>
//#include <emmintrin.h>

//correlatorprocess
extern bool bFirst;
extern char sinTable[CARRIER_TABLE_LENGTH];
extern char cosTable[CARRIER_TABLE_LENGTH];
extern int ADNumber;
extern char** CAExtendedTable;
//extern __m128i **** productTable;
extern long lgDataLength;
extern char* cgData;
//extern HANDLE chn_start_event[chmax + 1];
//extern HANDLE chn_finish_event[chmax + 1];
//extern HANDLE chn_thread[chmax + 1];
extern int chn_id[chmax + 1];
extern char directory_path[4096];

extern CHANNEL	chan[12];
extern int		out_debug;
extern FILE*	fpeph;

extern char			tzstr[40];
extern time_t			thetime;
extern FILE			*stream, *debug, *in, *out, *kalm;
extern FILE            *fprn_in, *fprn_out;
extern ALMANAC			gps_alm[33];
extern EPHEMERIS		gps_eph[33];
extern AVAILABLESV      availablesv;
extern int				SVh[33], ASV[33];
extern double			b0, b1, b2, b3, al0, al1, al2, al3;
extern double			a0, a1, tot, WNt, dtls, WNlsf, DN, dtlsf;
extern PVT				rpvt;
extern STATE			receiver;
extern double			gdop, pdop, hdop, vdop, tdop, alm_toa;
extern unsigned long	clock_tow;
extern LLH				rec_pos_llh;
extern LLH				current_loc, rp_llh;
extern ECEFT			track_sat[13];
extern XYZ				rec_pos_xyz;
extern int				alm_gps_week, gps_week, almanac_valid, almanac_flag, handle;
extern unsigned long	sf[6][11];
extern int				p_error[6];
extern int				status;
extern unsigned long	test_l[33];
extern double		mask_angle;
extern char		header[45], ttext[27], trailer;
extern double		meas_dop[13];							// 测量的每个通道的多普勒
extern XYZ			d_sat[13];								// 每个通道的卫星的速度
extern double		carrier_ref;				// 载波标称中频，单位：Hz
extern double		code_ref;					// CA码速率标频，单位：chip/s
extern double		dt[13];									// 从卫星到接收机的传播时间差
extern double		cbias;									// 接收机钟差
extern double c_2p12;
extern double c_2p4;
extern double c_2m5;
extern double c_2m11;
extern double c_2m19;
extern double c_2m20;
extern double c_2m21;
extern double c_2m23;
extern double c_2m24;
extern double c_2m27;
extern double c_2m29;
extern double c_2m30;
extern double c_2m31;
extern double c_2m33;
extern double c_2m38;
extern double c_2m43;
extern double c_2m50;
extern double c_2m55;
extern int			m_tropo, m_iono;				// flags for using tropo and iono models
extern int			align_t;						// 将接收机时间和GPS时间对准的标志
extern SATVIS		xyz[33];						// 卫星的位置、方位和多普勒
extern char last_prn[12];
extern char curloc_file[4096];
extern int		daicy_temp_cnt;
extern char ion_utc_file[4096];
extern char current_alm_file[4096];
extern char current_eph_file[4096];
extern char last_prn_file[4096];
extern CHANNEL		   chan[chmax + 1];				// 全程变量，用于存储每个通道跟踪环路的状态
extern CORRELATOR	   correlator[chmax + 1];		// 全程变量，用于存储每个通道相关器的状态
extern SVSTRUCT	      svStruct[MaxSvNumber + 1];	// 表示卫星可用性
extern long		      globalBufLength[chmax + 1];	// 用于保存每个通道缓存的数据长度，需初始化
extern double  *buffer[chmax + 1];			// 用于保存每个通道的缓存数据，需初始化分配空间
extern long		      correlatorDataLength;	// 共有缓存区的数据长度
extern long		      TIC_RT_CNTR;				// TIC实时计数器；
extern long		      TIC_CNTR;					// TIC的最大计数，对应了TIC中断的发生周期
// 注意：TIC_CNTR的赋值只能用programTIC函数实现
extern double         TIC_CNTR_RES;
extern double         TIC_CNTR_DBL;
extern long		      TIC_OCCUR_FLAG;			// TIC中断发生的标志

extern int			   display_page;		// 控制显示第i个页面
extern unsigned	test[16];
extern unsigned int tr_ch[13];					   // 索引0不用，内容为具有有效观测数据的chan的标号
extern int			out_debug, out_pos, out_vel, out_time;		// 是否输出debug和PVT信息的标志符
extern DMS			cur_lat, cur_long;			   // 用于显示经纬度

// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
extern int			   nav_tic;						// 一个nav_up的时间间隔对应的TIC中断的次数
extern int            ICP_CTL;	         // 输出累积载波相位还是跟踪环路的值，标志
extern double         code_corr;              // CA码速率的修正控制字，1.023MHz对应1.0
extern long           time_on;            // 计数器，步进为秒，程序不停，计数不止
extern long		      cc_scale;      	// 载波标频(1575.42MHz)/CA码标频(1.023MHz)
extern double         nav_up;				// 导航解的基本单位，单位：秒
extern double		   speed, heading;          // 接收机速度的绝对值和方位角
extern long		d_tow;								// 接收机时间差，单位：秒
extern int			key;						// 检查键盘的按键信息
extern int			tic_count;					// TIC计数器，也是测量中断发生次数的计数器
// 注意：测量中断的频率这里缺省为：100ms
// 计数范围为1s[0-9]
extern int			hms_count;					// TIC计数器，计数范围为1分钟[0-599]
extern int			nav_count;							// TIC计数器，计数范围为[0,nav_tic-1]
extern int			min_flag;							// 分钟计数满标志
extern int			nav_flag;							// 计算导航解标志
extern int			sec_flag;							// 秒标志
extern int			n_track;							// 跟踪(通道处于tracking模式)的卫星个数
extern unsigned int interr_int;
extern double		clock_offset;				// 接收机的钟漂，单位是ppm,初始值未知，假定为0
// 正值表示接收机的频率低于标称值
extern XYZ			rec_pos_ecef;						// 接收机的坐标
extern long		i_TIC_dt;							// 两次计算导航解的间隔,单位是样点个数
extern double		TIC_dt;								// 两次计算导航解的间隔,单位是秒
// 所以TIC_dt=i_TIC_dt*采样间隔
extern double		m_time[3];							// 接收机时间,[1]是当前时刻的，[0]是上一次的
// [2]似乎没有用
// m_time用于表示接收机的GPS时间
// 在原来的程序中m_time没有有效的更新,
// 它应该靠秒中断标志sec_flag来维护
extern double		delta_m_time;						// 接收机时间的小数部分
extern double		m_error;							// 接收机的测量时间和GPS整秒的误差
extern long		TIC_sum;							// 多个测量中断中期内累积的计数值
//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
//int			mstat;							// 仅用于GP2021，包含有miss data标志的寄存器
extern char		**caTable;							// 非实时处理时，存储37×1023的CA码表
extern double		DLLc1, DLLc2;						// DLL的系数
extern double		PLLc0, PLLc1, PLLc2;				// PLL的系数
extern double		FLLa1, FLLa2;						// FLL的系数
// 调试用的全程变量
extern double		*IQQ, *IQI;
extern double		IBUF[3000], QBUF[3000];
extern FILE		*fpIQ;
extern long		IQCounter;
extern FILE		*fpobs;
////extern FILE		*fpeph;
extern long		detectNum[chmax + 1];					// 搜索某颗卫星的持续时间
extern double		threshold_sig_search;				// 信号检测门限							
extern double		minFreqErr[chmax + 1], maxFreqErr[chmax + 1];
extern unsigned long uLastPCTick, uCurrentPCTick;
extern __int64 iTotalBitCounter;
extern bool bLocalTimeSetByTow;
extern bool bTowDecoded;
extern double DLLdT;
extern float   CARRIER_FREQ;   //标称中频  0625
extern double  SAMPLING_FREQ;     //0626
extern double  SAMPLING_INT;      //0626
extern long    DETECTSIZE;        //0626
extern long	TIC_ref;           //0626
extern double  deltaPhaseConst;   //0626
extern double  deltaCodePhaseConst;    //0626
extern long    SAMPLESPMS;         //0627
extern long    ACC_INT_PERIOD;     //0627
extern char    *GPSL1_data;              //0627
extern long    FFTSIZE;            //0627
extern long    bufSize;            //0627
extern FILE   *fpCodeRange1, *fpCodeRange2;     //0628
extern FILE   *fpCarrRange1, *fpCarrRange2;     //0628
extern long   ldcount;                 //0628
extern FILE   *fpdoppler;                      //0628
extern FILE   *fpCNo;                          //0628
extern char   if_data_file[4096];
extern char hot_cold;
extern FILE *daicy_file_pr;
extern FILE *daicy_file_pos;
extern FILE   *out_trtime;
extern char rcvr_par_file[4096];
extern int	pos_out_flag;
extern int firsttime;
extern int 	i4page, i5page;
extern int is_state_3;
extern char first_flag;
extern int judge_nits;
extern double	t_cor[13];
extern short	towFlag;
extern unsigned long GPSL1_pream;
extern unsigned long pb1, pb2, pb3;
extern unsigned long pb4, pb5, pb6;
extern double	wgs[3];

//b1
extern float	**BD1_corr;
extern float	*BD1_rCa, *BD1_iCa;
extern float	*BD1_rInData, *BD1_iInData;
extern float	*BD1_rProduct, *BD1_iProduct;
extern float	*BD1_cosPhase, *BD1_sinPhase;
extern size_t	*BD1_chipIndex;
extern char BD1_sinTable[BD1_CARRIER_TABLE_LENGTH];
extern char BD1_cosTable[BD1_CARRIER_TABLE_LENGTH];
extern int BD1_ADNumber;
extern char** BD1_CAExtendedTable;
//extern __m128i**** BD1_productTable;
extern long BD1_lgDataLength;
extern char* BD1_cgData;
//extern HANDLE BD1_chn_start_event[BD1_chmax + 1];
//extern HANDLE BD1_chn_finish_event[BD1_chmax + 1];
//extern HANDLE BD1_chn_thread[BD1_chmax + 1];
extern int BD1_chn_id[BD1_chmax + 1];
extern char BD1_directory_path[4096];
extern char			BD1_tzstr[40];
extern time_t			BD1_thetime;
extern FILE			*BD1_stream, *BD1_debug, *BD1_in, *BD1_out, *BD1_kalm;
extern FILE            *BD1_fprn_in, *BD1_fprn_out;
extern BD1_ALMANAC			BD1_gps_alm[13];
extern BD1_EPHEMERIS		BD1_gps_eph[13];

extern BD1_AVAILABLESV      BD1_availablesv;


extern int				BD1_SVh[13], BD1_ASV[13];
extern double			BD1_b0, BD1_b1, BD1_b2, BD1_b3, BD1_al0, BD1_al1, BD1_al2, BD1_al3;		// broadcast ionospheric delay model
extern double			BD1_a0, BD1_a1, BD1_tot, BD1_WNt, BD1_dtls, BD1_WNlsf, BD1_DN, BD1_dtlsf;	//broadcast UTC data

extern BD1_PVT				BD1_rpvt;
extern BD1_STATE			BD1_receiver;

extern double			BD1_gdop, BD1_pdop, BD1_hdop, BD1_vdop, BD1_tdop, BD1_alm_toa;
extern unsigned long	BD1_clock_tow;
extern BD1_LLH				BD1_rec_pos_llh;
extern BD1_LLH				BD1_current_loc, BD1_rp_llh;
extern BD1_ECEFT			BD1_track_sat[13];
extern BD1_XYZ				BD1_rec_pos_xyz;
extern int				BD1_alm_gps_week, BD1_gps_week, BD1_almanac_valid, BD1_almanac_flag, BD1_handle;

extern unsigned long	BD1_sf[13][11][6][11];							// 用于存储5个子帧10个字/页)的导航电文，13表示通道数，11表示页面数，6表示子桢数，11表示字数
////////2010.11.16//////////////////////////////////////////////////////////////////
extern unsigned long	BD1_sfd1[6][11];							// 用于存储5个子帧10个字/页)的导航电文，sf[0][]无效，sf[][0]无效

extern int				BD1_p_error[6];							// 用于存储5个子帧(10个字/页)的导航电文的校验状态
// LSB 10位有效，每个比特对应一个字的校验状态，0：正确，1：错误

extern int				BD1_status;								// 表征接收机的状态，只有三个值:
// cold_start, warm_start和navigating

extern unsigned long	BD1_test_l[33];

extern double		BD1_mask_angle;
extern char		BD1_header[45], BD1_ttext[27], BD1_trailer;
extern double		BD1_meas_dop[13];							// 测量的每个通道的多普勒
extern BD1_XYZ			BD1_d_sat[13];								// 每个通道的卫星的速度
extern double		BD1_carrier_ref;				// 载波标称中频，单位：Hz
extern double		BD1_code_ref;					// CA码速率标频，单位：chip/s
extern double		BD1_dt[13];									// 从卫星到接收机的传播时间差
extern double		BD1_cbias;									// 接收机钟差

// binary constants for nav message decoding

extern double BD1_c_2p12;
extern double BD1_c_2p4;
extern double BD1_c_2m5;
extern double BD1_c_2m6;
extern double BD1_c_2m11;
extern double BD1_c_2m19;
extern double BD1_c_2m20;
extern double BD1_c_2m21;
extern double BD1_c_2m23;
extern double BD1_c_2m24;
extern double BD1_c_2m27;
extern double BD1_c_2m29;
extern double BD1_c_2m30;
extern double BD1_c_2m31;
extern double BD1_c_2m33;
extern double BD1_c_2m38;
extern double BD1_c_2m43;
extern double BD1_c_2m50;
extern double BD1_c_2m55;
extern double BD1_c_2m66;
extern int			BD1_m_tropo, BD1_m_iono;				// flags for using tropo and iono models
extern int			BD1_align_t;						// 将接收机时间和GPS时间对准的标志
extern BD1_SATVIS		BD1_xyz[13];//33						// 卫星的位置、方位和多普勒
extern char BD1_last_prn[12];
extern char BD1_curloc_file[4096];
extern int		BD1_daicy_temp_cnt;
extern char BD1_ion_utc_file[4096];
extern char BD1_current_alm_file[4096];
extern char BD1_current_eph_file[4096];
extern char BD1_last_prn_file[4096];
extern BD1_CHANNEL		   BD1_chan[BD1_chmax + 1];				// 全程变量，用于存储每个通道跟踪环路的状态
extern BD1_CORRELATOR	   BD1_correlator[BD1_chmax + 1];		// 全程变量，用于存储每个通道相关器的状态
extern BD1_SVSTRUCT	      BD1_svStruct[BD1_MaxSvNumber + 1];	// 表示卫星可用性
extern long		      BD1_globalBufLength[BD1_chmax + 1];	// 用于保存每个通道缓存的数据长度，需初始化
extern double  *BD1_buffer[BD1_chmax + 1];			// 用于保存每个通道的缓存数据，需初始化分配空间
extern long		      BD1_correlatorDataLength;	// 共有缓存区的数据长度
extern long		      BD1_TIC_RT_CNTR;				// TIC实时计数器；
extern long		     BD1_TIC_CNTR;					// TIC的最大计数，对应了TIC中断的发生周期
extern double         BD1_TIC_CNTR_RES;
extern double         BD1_TIC_CNTR_DBL;
extern long		      BD1_TIC_OCCUR_FLAG;			// TIC中断发生的标志

extern int			   BD1_display_page;			// 控制显示第i个页面
extern unsigned	BD1_test[16];
extern unsigned int BD1_tr_ch[13];					   // 索引0不用，内容为具有有效观测数据的chan的标号
extern int			BD1_out_debug, BD1_out_pos, BD1_out_vel, BD1_out_time;		// 是否输出debug和PVT信息的标志符
extern BD1_DMS			BD1_cur_lat, BD1_cur_long;			   // 用于显示经纬度


// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
extern int			   BD1_nav_tic;						// 一个nav_up的时间间隔对应的TIC中断的次数
extern int            BD1_ICP_CTL;	         // 输出累积载波相位还是跟踪环路的值，标志
extern double         BD1_code_corr;              // CA码速率的修正控制字，1.023MHz对应1.0
extern long           BD1_time_on;	            // 计数器，步进为秒，程序不停，计数不止

extern double         BD1_nav_up;				// 导航解的基本单位，单位：秒
extern double		   BD1_speed, BD1_heading;          // 接收机速度的绝对值和方位角
extern long		BD1_d_tow;								// 接收机时间差，单位：秒
extern int			BD1_key;							// 检查键盘的按键信息
extern int			BD1_tic_count;				// TIC计数器，也是测量中断发生次数的计数器
// 注意：测量中断的频率这里缺省为：100ms
// 计数范围为1s[0-9]
extern int			BD1_hms_count;					// TIC计数器，计数范围为1分钟[0-599]
extern int			BD1_nav_count;							// TIC计数器，计数范围为[0,nav_tic-1]
extern int			BD1_min_flag;							// 分钟计数满标志
extern int			BD1_nav_flag;							// 计算导航解标志
extern int			BD1_sec_flag;							// 秒标志
extern int			BD1_n_track;							// 跟踪(通道处于tracking模式)的卫星个数
extern unsigned int BD1_interr_int;
extern double		BD1_clock_offset;				// 接收机的钟漂，单位是ppm,初始值未知，假定为0
// 正值表示接收机的频率低于标称值
extern BD1_XYZ			BD1_rec_pos_ecef;						// 接收机的坐标
extern long		BD1_i_TIC_dt;							// 两次计算导航解的间隔,单位是样点个数
extern double		BD1_TIC_dt;								// 两次计算导航解的间隔,单位是秒
// 所以TIC_dt=i_TIC_dt*采样间隔
extern double		BD1_m_time[3];							// 接收机时间,[1]是当前时刻的，[0]是上一次的
// [2]似乎没有用
// m_time用于表示接收机的GPS时间
// 在原来的程序中m_time没有有效的更新,
// 它应该靠秒中断标志sec_flag来维护
extern double		BD1_delta_m_time;						// 接收机时间的小数部分
extern double		BD1_m_error;							// 接收机的测量时间和GPS整秒的误差

extern long		BD1_TIC_sum;							// 多个测量中断中期内累积的计数值
//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
//int			mstat;							// 仅用于GP2021，包含有miss data标志的寄存器
extern char		**BD1_caTable;							// 非实时处理时，存储37×1023的CA码表

extern double		BD1_DLLc1, BD1_DLLc2;						// DLL的系数
extern double		BD1_PLLc0, BD1_PLLc1, BD1_PLLc2;				// PLL的系数
extern double		BD1_FLLa1, BD1_FLLa2;						// FLL的系数

// 调试用的全程变量
extern double		*BD1_IQQ, *BD1_IQI;
extern double		BD1_IBUF[3000], BD1_QBUF[3000];
extern FILE		*BD1_fpIQ;
extern long		BD1_IQCounter;
extern FILE		*BD1_fpobs;
extern FILE		*BD1_fpeph;


extern double		BD1_threshold_sig_search;				// 信号检测门限							

//char		data[SAMPLESPMS+1];							// 信号缓存区
//char		data[6000];							// 信号缓存区
extern double		BD1_minFreqErr[BD1_chmax + 1], BD1_maxFreqErr[BD1_chmax + 1];

extern unsigned long BD1_uLastPCTick, BD1_uCurrentPCTick;

extern __int64 BD1_iTotalBitCounter;

extern bool BD1_bLocalTimeSetByTow;
extern bool BD1_bTowDecoded;
extern float   BD1_CARRIER_FREQ;   //标称中频  0625
extern double  BD1_SAMPLING_FREQ;     //0626
extern double	BD1_ACQ_SAMPLING_FREQ;
extern double  BD1_SAMPLING_INT;      //0626
extern long    BD1_DETECTSIZE;        //0626
extern long	BD1_TIC_ref;           //0626
extern double  BD1_deltaPhaseConst;   //0626
extern double  BD1_deltaCodePhaseConst;    //0626
extern long    BD1_SAMPLESPMS;         //0627
extern long    BD1_ACC_INT_PERIOD;     //0627
extern char    *BD1_data;              //0627
extern long    BD1_FFTSIZE;            //0627
//hide by jh 3-11 long    bufSize;            //0627

extern FILE   *BD1_fpCodeRange1, *BD1_fpCodeRange2;     //0628
extern FILE   *BD1_fpCarrRange1, *BD1_fpCarrRange2;     //0628
extern long   BD1_ldcount;                //0628
extern FILE   *BD1_fpdoppler;                      //0628
extern FILE   *BD1_fpCNo;                          //0628
extern FILE   *BD1_out_trtime;

extern char   BD1_if_data_file[4096];

extern char BD1_hot_cold;
extern FILE *BD1_daicy_file_pr;
extern FILE *BD1_daicy_file_pos;

extern char BD1_rcvr_par_file[4096];

extern double BD1_wgs[3];
extern bool BD1_bFirst;
extern int BD1_is_state_3;
extern char BD1_first_flag;
extern int BD1_judge_nits;
extern double	BD1_t_cor[13];
extern short	BD1_towFlag;
extern unsigned long BD1_pream;
extern unsigned long BD1_pream1;


//b3
extern float	**BD3_corr;
extern  float	*BD3_rCa, *BD3_iCa;
extern float	*BD3_rInData, *BD3_iInData;
extern float	*BD3_rProduct, *BD3_iProduct;
extern float	*BD3_cosPhase, *BD3_sinPhase;
extern size_t	*BD3_chipIndex;
extern   char BD3_sinTable[BD3_CARRIER_TABLE_LENGTH];
extern char BD3_cosTable[BD3_CARRIER_TABLE_LENGTH];

extern int BD3_ADNumber;
extern char** BD3_CAExtendedTable;
//extern __m128i**** BD3_productTable;
extern long BD3_lgDataLength;
extern char* BD3_cgData;
//extern HANDLE BD3_chn_start_event[BD3_chmax + 1];
//extern HANDLE BD3_chn_finish_event[BD3_chmax + 1];
//extern HANDLE BD3_chn_thread[BD3_chmax + 1];
extern int BD3_chn_id[BD3_chmax + 1];
extern char BD3_directory_path[4096];
extern char			BD3_tzstr[40];
extern time_t			BD3_thetime;
extern   FILE			*BD3_stream, *BD3_debug, *BD3_in, *BD3_out, *BD3_kalm;
extern FILE            *BD3_fprn_in, *BD3_fprn_out;
extern BD3_ALMANAC			BD3_gps_alm[13];
extern BD3_EPHEMERIS		BD3_gps_eph[13];
extern BD3_AVAILABLESV      BD3_availablesv;


extern int				BD3_SVh[13], BD3_ASV[13];
extern double			BD3_b0, BD3_b1, BD3_b2, BD3_b3, BD3_al0, BD3_al1, BD3_al2, BD3_al3;		// broadcast ionospheric delay model
extern double			BD3_a0, BD3_a1, BD3_tot, BD3_WNt, BD3_dtls, BD3_WNlsf, BD3_DN, BD3_dtlsf;	//broadcast UTC data

extern BD3_PVT				BD3_rpvt;
extern BD3_STATE			BD3_receiver;

extern double			BD3_gdop, BD3_pdop, BD3_hdop, BD3_vdop, BD3_tdop, BD3_alm_toa;
extern unsigned long	BD3_clock_tow;
extern BD3_LLH				BD3_rec_pos_llh;
extern BD3_LLH				BD3_current_loc, BD3_rp_llh;
extern BD3_ECEFT			BD3_track_sat[13];
extern BD3_XYZ				BD3_rec_pos_xyz;
extern int				BD3_alm_gps_week, BD3_gps_week, BD3_almanac_valid, BD3_almanac_flag, BD3_handle;

extern unsigned long	BD3_sf[13][11][6][11];							// 用于存储5个子帧10个字/页)的导航电文，13表示通道数，11表示页面数，6表示子桢数，11表示字数
////////2010.11.16//////////////////////////////////////////////////////////////////
extern unsigned long	BD3_sfd1[6][11];							// 用于存储5个子帧10个字/页)的导航电文，sf[0][]无效，sf[][0]无效

extern int				BD3_p_error[6];							// 用于存储5个子帧(10个字/页)的导航电文的校验状态
// LSB 10位有效，每个比特对应一个字的校验状态，0：正确，1：错误

extern int				BD3_status;								// 表征接收机的状态，只有三个值:
// cold_start, warm_start和navigating

extern unsigned long	BD3_test_l[33];

extern double		BD3_mask_angle;
extern char		BD3_header[45], BD3_ttext[27], BD3_trailer;
extern double		BD3_meas_dop[13];							// 测量的每个通道的多普勒
extern BD3_XYZ			BD3_d_sat[13];								// 每个通道的卫星的速度
extern double		BD3_carrier_ref;				// 载波标称中频，单位：Hz
//double		code_ref=2046000;						// CA码速率标频，单位：chip/s
extern double		BD3_code_ref;
extern double		BD3_dt[13];									// 从卫星到接收机的传播时间差
extern double		BD3_cbias;									// 接收机钟差

// binary constants for nav message decoding

extern double BD3_c_2p12;
extern double BD3_c_2p4;
extern double BD3_c_2m5;
extern double BD3_c_2m6;
extern double BD3_c_2m11;
extern double BD3_c_2m19;
extern double BD3_c_2m20;
extern double BD3_c_2m21;
extern  double BD3_c_2m23;
extern double BD3_c_2m24;
extern double BD3_c_2m27;
extern double BD3_c_2m29;
extern  double BD3_c_2m30;
extern double BD3_c_2m31;
extern  double BD3_c_2m33;
extern double BD3_c_2m38;
extern double BD3_c_2m43;
extern  double BD3_c_2m50;
extern double BD3_c_2m55;
extern double BD3_c_2m66;

extern int			BD3_m_tropo, BD3_m_iono;				// flags for using tropo and iono models
extern int			BD3_align_t;						// 将接收机时间和GPS时间对准的标志
extern BD3_SATVIS		BD3_xyz[13];//33						// 卫星的位置、方位和多普勒

extern char BD3_last_prn[12];
extern char BD3_curloc_file[4096];
extern int		BD3_daicy_temp_cnt;
extern char BD3_ion_utc_file[4096];
extern char BD3_current_alm_file[4096];
extern char BD3_current_eph_file[4096];
extern char BD3_last_prn_file[4096];
extern BD3_CHANNEL		  BD3_chan[BD3_chmax + 1];				// 全程变量，用于存储每个通道跟踪环路的状态
extern BD3_CORRELATOR	   BD3_correlator[BD3_chmax + 1];		// 全程变量，用于存储每个通道相关器的状态
extern BD3_SVSTRUCT	      BD3_svStruct[BD3_MaxSvNumber + 1];	// 表示卫星可用性
extern long		      BD3_globalBufLength[BD3_chmax + 1];	// 用于保存每个通道缓存的数据长度，需初始化
extern double  *BD3_buffer[BD3_chmax + 1];			// 用于保存每个通道的缓存数据，需初始化分配空间
extern long		      BD3_correlatorDataLength;	// 共有缓存区的数据长度
extern long		      BD3_TIC_RT_CNTR;				// TIC实时计数器；
extern long		      BD3_TIC_CNTR;					// TIC的最大计数，对应了TIC中断的发生周期
// 注意：TIC_CNTR的赋值只能用programTIC函数实现
extern double         BD3_TIC_CNTR_RES;
extern double         BD3_TIC_CNTR_DBL;
extern long		      BD3_TIC_OCCUR_FLAG;			// TIC中断发生的标志

extern int			   BD3_display_page;			// 控制显示第i个页面
extern unsigned	BD3_test[16];
extern unsigned int BD3_tr_ch[13];					   // 索引0不用，内容为具有有效观测数据的chan的标号
extern int			BD3_out_debug, BD3_out_pos, BD3_out_vel, BD3_out_time;		// 是否输出debug和PVT信息的标志符
extern BD3_DMS			BD3_cur_lat, BD3_cur_long;			   // 用于显示经纬度


// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
extern int			   BD3_nav_tic;						// 一个nav_up的时间间隔对应的TIC中断的次数
extern int            BD3_ICP_CTL;		         // 输出累积载波相位还是跟踪环路的值，标志
extern double         BD3_code_corr;              // CA码速率的修正控制字，1.023MHz对应1.0
extern long           BD3_time_on;	            // 计数器，步进为秒，程序不停，计数不止

extern double         BD3_nav_up;					// 导航解的基本单位，单位：秒
extern double		   BD3_speed, BD3_heading;          // 接收机速度的绝对值和方位角
extern long		BD3_d_tow;								// 接收机时间差，单位：秒
extern int			BD3_key;								// 检查键盘的按键信息
extern int			BD3_tic_count;						// TIC计数器，也是测量中断发生次数的计数器
// 注意：测量中断的频率这里缺省为：100ms
// 计数范围为1s[0-9]
extern int			BD3_hms_count;						// TIC计数器，计数范围为1分钟[0-599]
extern int			BD3_nav_count;							// TIC计数器，计数范围为[0,nav_tic-1]
extern int			BD3_min_flag;							// 分钟计数满标志
extern int			BD3_nav_flag;							// 计算导航解标志
extern int			BD3_sec_flag;							// 秒标志
extern int			BD3_n_track;							// 跟踪(通道处于tracking模式)的卫星个数
extern unsigned int BD3_interr_int;
extern double		BD3_clock_offset;					// 接收机的钟漂，单位是ppm,初始值未知，假定为0
// 正值表示接收机的频率低于标称值
extern BD3_XYZ			BD3_rec_pos_ecef;						// 接收机的坐标
extern long		BD3_i_TIC_dt;							// 两次计算导航解的间隔,单位是样点个数
extern double		BD3_TIC_dt;								// 两次计算导航解的间隔,单位是秒
// 所以TIC_dt=i_TIC_dt*采样间隔
extern double		BD3_m_time[3];							// 接收机时间,[1]是当前时刻的，[0]是上一次的
// [2]似乎没有用
// m_time用于表示接收机的GPS时间
// 在原来的程序中m_time没有有效的更新,
// 它应该靠秒中断标志sec_flag来维护
extern double		BD3_delta_m_time;						// 接收机时间的小数部分
extern double		BD3_m_error;							// 接收机的测量时间和GPS整秒的误差

extern long		BD3_TIC_sum;							// 多个测量中断中期内累积的计数值
//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
//int			mstat;							// 仅用于GP2021，包含有miss data标志的寄存器
extern char		**BD3_caTable;							// 非实时处理时，存储14×10230的CA码表

extern double		BD3_DLLc1, BD3_DLLc2;						// DLL的系数
extern double		BD3_PLLc0, BD3_PLLc1, BD3_PLLc2;				// PLL的系数
extern double		BD3_FLLa1, BD3_FLLa2;						// FLL的系数

// 调试用的全程变量
extern double		*BD3_IQQ, *BD3_IQI;
extern double		BD3_IBUF[3000], BD3_QBUF[3000];
extern FILE		*BD3_fpIQ;
extern long		BD3_IQCounter;
extern FILE		*BD3_fpobs;
extern FILE		*BD3_fpeph;
extern double		BD3_threshold_sig_search;				// 信号检测门限							
extern double		BD3_minFreqErr[BD3_chmax + 1], BD3_maxFreqErr[BD3_chmax + 1];
extern unsigned long BD3_uLastPCTick, BD3_uCurrentPCTick;
extern __int64 BD3_iTotalBitCounter;
extern bool BD3_bLocalTimeSetByTow;
extern bool BD3_bTowDecoded;
extern float   BD3_CARRIER_FREQ;   //标称中频  0625
extern double BD3_SAMPLING_FREQ;     //0626
//double	ACQ_SAMPLING_FREQ = 4.096e6;//2.048
extern double	BD3_ACQ_SAMPLING_FREQ;//revised
extern double  BD3_SAMPLING_INT;      //0626
extern long    BD3_DETECTSIZE;        //0626
extern long	BD3_TIC_ref;           //0626
extern double  BD3_deltaPhaseConst;   //0626
extern double  BD3_deltaCodePhaseConst;    //0626
extern long    BD3_SAMPLESPMS;         //0627
extern long    BD3_ACC_INT_PERIOD;     //0627
extern char    *BD3_data;              //0627
extern long    BD3_FFTSIZE;            //0627
extern FILE   *BD3_fpCodeRange1, *BD3_fpCodeRange2;     //0628
extern FILE   *BD3_fpCarrRange1, *BD3_fpCarrRange2;     //0628
extern long   BD3_ldcount;                    //0628
extern FILE   *BD3_fpdoppler;                      //0628
extern FILE   *BD3_fpCNo;                          //0628
extern FILE   *BD3_out_trtime;
extern char   BD3_if_data_file[4096];
extern char BD3_hot_cold;
extern FILE *BD3_daicy_file_pr;
extern FILE *BD3_daicy_file_pos;
extern char BD3_rcvr_par_file[4096];
extern double BD3_wgs[3];
extern bool BD3_bFirst;
extern int BD3_is_state_3;
extern char BD3_first_flag;
extern int BD3_judge_nits;
extern double	BD3_t_cor[13];
extern short	BD3_towFlag;
extern unsigned long BD3_pream;  //wqnavmess
extern unsigned long BD3_pream1;

//glo1
extern float	**GLO1_corr;
extern float	*GLO1_rCa, *GLO1_iCa;
extern float	*GLO1_rInData, *GLO1_iInData;
extern float	*GLO1_rProduct, *GLO1_iProduct;
extern float	*GLO1_cosPhase, *GLO1_sinPhase;
extern char GLO1_sinTable[GLO1_CARRIER_TABLE_LENGTH];
extern char GLO1_cosTable[GLO1_CARRIER_TABLE_LENGTH];

extern int GLO1_ADNumber;

extern char** GLO1_CAExtendedTable;
//extern __m128i**** GLO1_productTable;
extern long GLO1_lgDataLength;
extern char* GLO1_cgData;
//extern HANDLE GLO1_chn_start_event[GLO1_chmax + 1];
//extern HANDLE GLO1_chn_finish_event[GLO1_chmax + 1];
//extern HANDLE GLO1_chn_thread[GLO1_chmax + 1];
extern int GLO1_chn_id[GLO1_chmax + 1];
extern char GLO1_directory_path[4096];
extern char			GLO1_tzstr[40];
extern time_t			GLO1_thetime;
extern FILE			*GLO1_stream, *GLO1_debug, *GLO1_in, *GLO1_out, *GLO1_kalm;
extern FILE            *GLO1_fprn_in, *GLO1_fprn_out;
extern GLO1_ALMANAC			GLO1_gps_alm[33];
extern GLO1_EPHEMERIS		GLO1_gps_eph[33];

extern GLO1_AVAILABLESV      GLO1_availablesv;

extern GLO1_glonass_ephemeris  GLO1_glonass_sv_id_ephemeris[GLONASS_SV_TOTAL_NUM];
extern GLO1_glonass_almanac_str5 GLO1_glonass_sv_id_almanac_str5[GLONASS_SV_TOTAL_NUM];

extern int				GLO1_SVh[33], GLO1_ASV[33];
extern double			GLO1_b0, GLO1_b1, GLO1_b2, GLO1_b3, GLO1_al0, GLO1_al1, GLO1_al2, GLO1_al3;
extern double			GLO1_a0, GLO1_a1, GLO1_tot, GLO1_WNt, GLO1_dtls, GLO1_WNlsf, GLO1_DN, GLO1_dtlsf;

extern GLO1_PVT				GLO1_rpvt;
extern GLO1_STATE			GLO1_receiver;

extern double			GLO1_gdop, GLO1_pdop, GLO1_hdop, GLO1_vdop, GLO1_tdop, GLO1_alm_toa;
extern unsigned long	GLO1_clock_tow;
extern GLO1_LLH				GLO1_rec_pos_llh;
extern GLO1_LLH				GLO1_current_loc, GLO1_rp_llh;
extern GLO1_ECEFT			GLO1_track_sat[13];
extern GLO1_XYZ				GLO1_rec_pos_xyz;
extern int				GLO1_alm_gps_week, GLO1_gps_week, GLO1_almanac_valid, GLO1_almanac_flag, GLO1_handle;
extern unsigned long	GLO1_sf[6][11];

extern int				GLO1_p_error[6];


extern int				GLO1_status;

extern unsigned long	GLO1_test_l[33];

extern double		GLO1_mask_angle;
extern char		GLO1_header[45], GLO1_ttext[27], GLO1_trailer;
extern double		GLO1_meas_dop[13];							// 测量的每个通道的多普勒
extern GLO1_XYZ			GLO1_d_sat[13];								// 每个通道的卫星的速度
extern double		GLO1_carrier_ref;				// 载波标称中频，单位：Hz
//double		code_ref=1023000;						// CA码速率标频，单位：chip/s
extern double		GLO1_code_ref;					// CA码速率标频，单位：chip/s

extern double		GLO1_dt[13];									// 从卫星到接收机的传播时间差
extern double		GLO1_cbias;									// 接收机钟差
extern double      GLO1_lambda[13];
// binary constants for nav message decoding

extern double GLO1_c_2p12;
extern double GLO1_c_2p4;
extern double GLO1_c_2m5;
extern double GLO1_c_2m11;
extern  double GLO1_c_2m19;
extern double GLO1_c_2m20;
extern double GLO1_c_2m21;
extern   double GLO1_c_2m23;
extern double GLO1_c_2m24;
extern double GLO1_c_2m27;
extern double GLO1_c_2m29;
extern double GLO1_c_2m30;
extern double GLO1_c_2m31;
extern double GLO1_c_2m33;;
extern  double GLO1_c_2m38;
extern double GLO1_c_2m43;
extern double GLO1_c_2m50;
extern double GLO1_c_2m55;

extern int			GLO1_m_tropo, GLO1_m_iono;				// flags for using tropo and iono models
extern int			GLO1_align_t;						// 将接收机时间和GPS时间对准的标志
extern GLO1_SATVIS		GLO1_xyz[33];						// 卫星的位置、方位和多普勒

extern char GLO1_last_prn[12];
extern char GLO1_curloc_file[4096];
extern unsigned char  GLO1_decode_glonass_index[5];
extern unsigned char  GLO1_decode_glonass_frame[10][5];
//定义电文格式************电文位计数:  85--1

//#define		glonass_m				0
//#define		glonass_KX				1
//#define		glonass_resv1			2
//
//#define		glonass_e_P1			3
//#define		glonass_e_tk_hour		4
//#define		glonass_e_tk_min		5
//#define		glonass_e_tk_sec		6
//#define		glonass_e_dot_Xn		7
//#define		glonass_e_doudot_Xn		8
//#define		glonass_e_Xn			9
//
///*string 2*/
//#define		glonass_e_Bn			3
//#define		glonass_e_P2			4
//#define		glonass_e_tb			5
//#define		glonass_e_dot_Yn		6
//#define		glonass_e_doudot_Yn		7
//#define		glonass_e_Yn			8
//
///*string 3*/
//#define		glonass_e_P3			3
//#define		glonass_e_gamman		4
//#define		glonass_e_P				5
//#define		glonass_e_ln			6
//#define		glonass_e_dot_Zn		7
//#define		glonass_e_doudot_Zn		8
//#define		glonass_e_Zn			9
//
///*string 4*/
//#define		glonass_resv2			3
//#define		glonass_e_taun			4
//#define		glonass_e_deltataun		5
//#define		glonass_e_En			6
//#define		glonass_e_P4			7
//#define		glonass_e_FT			8
//#define		glonass_e_NT			9
//#define		glonass_e_n				10
//#define		glonass_e_Mod			11
//
///*string 5*/
//#define		glonass_a_daynum		3
//#define		glonass_a_tauc			4
//#define		glonass_a_N4			5
//#define		glonass_a_taugps		6
//#define		glonass_a_str5_ln		7
//
///*string 6 8 10 12 14,frame 1 2 3 4 & string 6 8 10 12,frame 5*/
//#define		glonass_a_Cn			2
//#define		glonass_a_Mn			3
//#define		glonass_a_n				4
//#define		glonass_a_taun			5
//#define		glonass_a_lambdan		6
//#define		glonass_a_deltain		7
//#define		glonass_a_epsilonn		8
//
///*string 7 9 11 13 15,frame 1 2 3 4 &string 7 9 11 13,frame 5*/
//#define		glonass_a_omegan		2
//#define		glonass_a_tlambdan		3
//#define		glonass_a_deltaTn		4
//#define		glonass_a_dot_deltaTn	5
//#define		glonass_a_Hn			6
//#define		glonass_a_ln			7
//
///*string 14,frame 5 */
//#define		glonass_B1				3
//#define		glonass_B2				4
//#define		glonass_KP				5
//
///*string 15,frame 5 */
//#define		glonass_str15_ln		3
//extern GLO1_decode_glonass_info GLO1_glonass_formats[][5];
extern int		GLO1_daicy_temp_cnt;

extern char GLO1_ion_utc_file[4096];
extern char GLO1_current_alm_file[4096];
extern char GLO1_current_eph_file[4096];

extern char GLO1_last_prn_file[4096];

extern GLO1_CHANNEL		   GLO1_chan[12];				// 全程变量，用于存储每个通道跟踪环路的状态
extern GLO1_CORRELATOR	   GLO1_correlator[12];		// 全程变量，用于存储每个通道相关器的状态
extern GLO1_SVSTRUCT	      GLO1_svStruct[25];	// 表示卫星可用性
extern long		      GLO1_globalBufLength[12];	// 用于保存每个通道缓存的数据长度，需初始化
extern double  *GLO1_buffer[GLO1_chmax + 1];			// 用于保存每个通道的缓存数据，需初始化分配空间
extern long		      GLO1_correlatorDataLength;	// 共有缓存区的数据长度
extern long		      GLO1_TIC_RT_CNTR;				// TIC实时计数器；
extern long		      GLO1_TIC_CNTR;					// TIC的最大计数，对应了TIC中断的发生周期
// 注意：GLO1_TIC_CNTR的赋值只能用GLO1_programTIC函数实现
extern double         GLO1_TIC_CNTR_RES;
extern double         GLO1_TIC_CNTR_DBL;
extern long		      GLO1_TIC_OCCUR_FLAG;			// TIC中断发生的标志

extern int			   GLO1_display_page;			// 控制显示第i个页面
extern unsigned	GLO1_test[16];
extern unsigned int GLO1_tr_ch[13];					   // 索引0不用，内容为具有有效观测数据的GLO1_chan的标号
extern int			GLO1_out_debug, GLO1_out_pos, GLO1_out_vel, GLO1_out_time;		// 是否输出GLO1_debug和GLO1_PVT信息的标志符
extern GLO1_DMS			GLO1_cur_lat, GLO1_cur_long;			   // 用于显示经纬度

// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
extern int			   GLO1_nav_tic;						// 一个GLO1_nav_up的时间间隔对应的TIC中断的次数
extern int            GLO1_ICP_CTL;		         // 输出累积载波相位还是跟踪环路的值，标志
extern double         GLO1_ICP_code_corr;              // CA码速率的修正控制字，1.023MHz对应1.0
extern long           GLO1_time_on;	            // 计数器，步进为秒，程序不停，计数不止
// long		      GLO1_cc_scale=1540;       	// 载波标频(1575.42MHz)/CA码标频(1.023MHz)
extern long		      GLO1_cc_scale;       	// 载波标频(1602+k*0.5625MHz)/CA码标频(0.511MHz)   todo  多颗星需要改变,但此变量后面没有使用

extern int GLO1_alloced_k[14];  // 记录某个 k 是否已经分配通道 k = -7 --- 6

extern double         GLO1_nav_up;					// 导航解的基本单位，单位：秒
extern double		   GLO1_speed, GLO1_heading;          // 接收机速度的绝对值和方位角
extern long		GLO1_d_tow;								// 接收机时间差，单位：秒
extern int			GLO1_key;								// 检查键盘的按键信息
extern int			GLO1_tic_count;						// TIC计数器，也是测量中断发生次数的计数器
// 注意：测量中断的频率这里缺省为：100ms
// 计数范围为1s[0-9]
extern int			GLO1_hms_count;						// TIC计数器，计数范围为1分钟[0-599]
extern int			GLO1_nav_count;							// TIC计数器，计数范围为[0,GLO1_nav_tic-1]
extern int			GLO1_min_flag;							// 分钟计数满标志
extern int			GLO1_nav_flag;							// 计算导航解标志
extern int			GLO1_sec_flag;							// 秒标志
extern int			GLO1_n_track;							// 跟踪(通道处于tracking模式)的卫星个数
extern unsigned int GLO1_interr_int;
extern double		GLO1_clock_offset;					// 接收机的钟漂，单位是ppm,初始值未知，假定为0
// 正值表示接收机的频率低于标称值
extern GLO1_XYZ			GLO1_rec_pos_ecef;						// 接收机的坐标
extern long		GLO1_i_TIC_dt;							// 两次计算导航解的间隔,单位是样点个数
extern double		GLO1_TIC_dt;								// 两次计算导航解的间隔,单位是秒
// 所以GLO1_TIC_dt=GLO1_i_TIC_dt*采样间隔
extern double		GLO1_m_time[3];							// 接收机时间,[1]是当前时刻的，[0]是上一次的
// [2]似乎没有用
// GLO1_m_time用于表示接收机的GPS时间
// 在原来的程序中GLO1_m_time没有有效的更新,
// 它应该靠秒中断标志GLO1_sec_flag来维护
extern double		GLO1_delta_m_time;						// 接收机时间的小数部分
extern double		GLO1_m_error;							// 接收机的测量时间和GPS整秒的误差
extern long		GLO1_TIC_sum;							// 多个测量中断中期内累积的计数值
//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
//int			mstat;							// 仅用于GP2021，包含有miss GLO1_data标志的寄存器
extern char		**GLO1_caTable;							// 非实时处理时，存储37×1023的CA码表

extern double		GLO1_DLLc1, GLO1_DLLc2;						// DLL的系数
extern double		GLO1_PLLc0, GLO1_PLLc1, GLO1_PLLc2;				// PLL的系数
extern double		GLO1_FLLa1, GLO1_FLLa2;						// FLL的系数

// 调试用的全程变量
extern double		*GLO1_IQQ, *GLO1_IQI;
extern double		GLO1_IBUF[3000], GLO1_QBUF[3000];
extern FILE		*GLO1_fpIQ;
extern long		GLO1_IQCounter;
extern FILE		*GLO1_fpobs;
extern FILE		*GLO1_fpeph;

extern long		GLO1_detectNum[GLO1_chmax + 1];					// 搜索某颗卫星的持续时间
extern double		GLO1_threshold_sig_search;				// 信号检测门限							

//char		GLO1_data[GLO1_SAMPLESPMS+1];							// 信号缓存区
//char		GLO1_data[6000];							// 信号缓存区
extern double		GLO1_minFreqErr[12], GLO1_maxFreqErr[12];

extern unsigned long GLO1_uLastPCTick, GLO1_uCurrentPCTick;

extern __int64 GLO1_iTotalBitCounter;

extern bool GLO1_bLocalTimeSetByTow;
extern bool GLO1_bTowDecoded;

extern float   GLO1_CARRIER_FREQ;   //标称中频  0625 //todo:GLONASS中心频率
extern double  GLO1_SAMPLING_FREQ;     //0626
extern double  GLO1_SAMPLING_INT;      //0626
extern long    GLO1_DETECTSIZE;        //0626
extern long	GLO1_TIC_ref;           //0626
extern double  GLO1_deltaPhaseConst;   //0626
extern double  GLO1_deltaCodePhaseConst;    //0626
extern long    GLO1_SAMPLESPMS;         //0627
extern long    GLO1_ACC_INT_PERIOD;     //0627
extern char    *GLO1_data;              //0627
extern long    GLO1_FFTSIZE;            //0627
extern long    GLO1_bufSize;            //0627

extern FILE   *GLO1_fpCodeRange1, *GLO1_fpCodeRange2;     //0628
extern FILE   *GLO1_fpCarrRange1, *GLO1_fpCarrRange2;     //0628
extern long   GLO1_ldcount;                     //0628
extern FILE   *GLO1_fpdoppler;                      //0628
extern FILE   *GLO1_fpCNo;                          //0628

extern char   GLO1_if_data_file[4096];

extern GLO1_glonass_ephemeris GLO1_glonass_channel_ephemeris[12];
extern GLO1_glonass_almanac GLO1_glonass_sv_id_almanac[GLONASS_SV_TOTAL_NUM];
extern GLO1_glonass_almanac_str5 *GLO1_palc_str5;
extern GLO1_glonass_almanac_global *GLO1_palc_glob;

extern int16s	GLO1_unpack_glonass_flag[12];

extern int16s	GLO1_glonass_ephemeris_processing[12];		//初始化  -1
extern int32u *GLO1_glonass_frame_id;
extern int16s GLO1_glonass_almanac_processing[12];
extern int16s	GLO1_glonass_almanac_global_processing[12];	//初始化  -1
extern bool GLO1_string_1[12];
extern char GLO1_hot_cold;
extern FILE *GLO1_daicy_file_pr;
extern FILE *GLO1_daicy_file_pos;
extern  FILE   *GLO1_out_trtime;
extern char GLO1_rcvr_par_file[4096];
extern double GLO1_wgs[3];
extern int	GLO1_pos_out_flag;

extern int GLO1_judge_nits;
extern double	GLO1_t_cor[13];
extern short	GLO1_towFlag;
extern unsigned long GLO1_GLO1_pream;
extern const unsigned GLO1_pream1;
extern int GLO1_is_state_3;
extern char GLO1_first_flag;
extern bool GLO1_bFirst;
extern int GLO1_firsttime;

extern fp64 GLO1_cx[5], GLO1_cy[5], GLO1_cz[5];
extern fp64 GLO1_dx[5], GLO1_dy[5], GLO1_dz[5];
extern fp64 GLO1_ux[5], GLO1_uy[5], GLO1_uz[5];
extern fp64 GLO1_px[5], GLO1_py[5], GLO1_pz[5];

extern int 	GLO1_i4page, GLO1_i5page;

void init_receiver_var()
{
	bFirst = true;
	memset(sinTable, 0, sizeof(sinTable));
	memset(cosTable, 0, sizeof(cosTable));
	CAExtendedTable = 0;
	//productTable = 0;
	lgDataLength = 0;
	cgData = 0;
	/*extern HANDLE chn_start_event[chmax + 1];
	extern HANDLE chn_finish_event[chmax + 1];
	extern HANDLE chn_thread[chmax + 1];*/
	memset(chn_id, 0, sizeof(chn_id));
	memset(directory_path, 0, sizeof(directory_path));
	memset(chan, 0, sizeof(chan));
	out_debug = 0;
	/*extern FILE*	fpeph;*/
	memset(tzstr, 0, sizeof(tzstr));
	/*extern FILE			*stream, *debug, *in, *out, *kalm;
	extern FILE            *fprn_in, *fprn_out;*/
	memset(gps_alm, 0, sizeof(gps_alm));
	memset(gps_eph, 0, sizeof(gps_eph));
	memset(&availablesv, 0, sizeof(availablesv));
	memset(SVh, 0, sizeof(SVh));
	memset(ASV, 0, sizeof(ASV));
	b0 = b1 = b2 = b3 = al0 = al1 = al2 = al3 = 0;
	a0 = a1 = tot = WNt = dtls = WNlsf = DN = dtlsf = 0;
	memset(&rpvt, 0, sizeof(rpvt));
	memset(&receiver, 0, sizeof(receiver));
	gdop = pdop = hdop = vdop = tdop = alm_toa = 0;
	clock_tow = 0;
	memset(&rec_pos_llh, 0, sizeof(rec_pos_llh));
	memset(&current_loc, 0, sizeof(current_loc));
	memset(&rp_llh, 0, sizeof(rp_llh));
	memset(track_sat, 0, sizeof(track_sat));
	memset(&rec_pos_xyz, 0, sizeof(rec_pos_xyz));
	alm_gps_week = gps_week = almanac_valid = almanac_flag = handle = 0;
	memset(sf, 0, sizeof(sf));
	memset(p_error, 0, sizeof(p_error));
	status = 0;
	unsigned long tmp1[] = { 0x00000000L,            // single bit set numbers
		0x00000001L, 0x00000002L, 0x00000004L, 0x00000008L,  // for testing bit positions
		0x00000010L, 0x00000020L, 0x00000040L, 0x00000080L,	// 0000 0001 0010 0100 1000
		0x00000100L, 0x00000200L, 0x00000400L, 0x00000800L,	// 32位的第1位到第32位
		0x00001000L, 0x00002000L, 0x00004000L, 0x00008000L,
		0x00010000L, 0x00020000L, 0x00040000L, 0x00080000L,
		0x00100000L, 0x00200000L, 0x00400000L, 0x00800000L,
		0x01000000L, 0x02000000L, 0x04000000L, 0x08000000L,
		0x10000000L, 0x20000000L, 0x40000000L, 0x80000000L };
	memcpy(test_l, tmp1, sizeof(test_l));

	mask_angle = 0;
	memset(header, 0, sizeof(header));
	memset(ttext, 0, sizeof(ttext));
	trailer = 0;
	memset(meas_dop, 0, sizeof(meas_dop));
	memset(d_sat, 0, sizeof(d_sat));
	carrier_ref = 0;				// 载波标称中频，单位：Hz
	code_ref = 1023000;						// CA码速率标频，单位：chip/s
	memset(dt, 0, sizeof(dt));								// 从卫星到接收机的传播时间差
	cbias = 0;									// 接收机钟差
	c_2p12 = 4096;
	c_2p4 = 16;
	c_2m5 = 0.03125;
	c_2m11 = 4.8828125e-4;
	c_2m19 = 1.9073486328125e-6;
	c_2m20 = 9.5367431640625e-7;
	c_2m21 = 4.76837158203125e-7;
	c_2m23 = 1.19209289550781e-7;
	c_2m24 = 5.96046447753906e-8;
	c_2m27 = 7.45058059692383e-9;
	c_2m29 = 1.86264514923096e-9;
	c_2m30 = 9.31322574615479e-10;
	c_2m31 = 4.65661287307739e-10;
	c_2m33 = 1.16415321826935E-10;
	c_2m38 = 3.63797880709171e-12;
	c_2m43 = 1.13686837721616e-13;
	c_2m50 = 8.881784197e-16;
	c_2m55 = 2.77555756156289e-17;
	m_tropo = m_iono = align_t = 0;
	memset(xyz, 0, sizeof(xyz));
	memset(last_prn, 0, sizeof(last_prn));
	memset(curloc_file, 0, sizeof(curloc_file));
	daicy_temp_cnt = 0;
	memset(ion_utc_file, 0, sizeof(ion_utc_file));
	memset(current_alm_file, 0, sizeof(current_alm_file));
	memset(current_eph_file, 0, sizeof(current_eph_file));
	memset(last_prn_file, 0, sizeof(last_prn_file));
	memset(chan, 0, sizeof(chan));			// 全程变量，用于存储每个通道跟踪环路的状态
	memset(correlator, 0, sizeof(correlator));	// 全程变量，用于存储每个通道相关器的状态
	memset(svStruct, 0, sizeof(svStruct));	// 表示卫星可用性
	memset(globalBufLength, 0, sizeof(globalBufLength));	// 用于保存每个通道缓存的数据长度，需初始化
	memset(buffer, 0, sizeof(buffer));		// 用于保存每个通道的缓存数据，需初始化分配空间

	correlatorDataLength = TIC_RT_CNTR = TIC_CNTR = 0;					// TIC的最大计数，对应了TIC中断的发生周期
	// 注意：TIC_CNTR的赋值只能用programTIC函数实现
	TIC_CNTR_RES = TIC_CNTR_DBL = 0;
	TIC_OCCUR_FLAG = 0;			// TIC中断发生的标志

	display_page = 0;			// 控制显示第i个页面
	unsigned tmp_2[] = { 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
		0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };
	memcpy(test, tmp_2, sizeof(test));
	memset(tr_ch, 0, sizeof(tr_ch));			   // 索引0不用，内容为具有有效观测数据的chan的标号
	out_debug = out_pos = out_vel = out_time = 0;		// 是否输出debug和PVT信息的标志符
	memset(&cur_lat, 0, sizeof(cur_lat));
	memset(&cur_long, 0, sizeof(cur_long));		   // 用于显示经纬度

	// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
	nav_tic = ICP_CTL = 0;		         // 输出累积载波相位还是跟踪环路的值，标志
	code_corr = 0;              // CA码速率的修正控制字，1.023MHz对应1.0
	time_on = 0;	            // 计数器，步进为秒，程序不停，计数不止
	cc_scale = 1540;       	// 载波标频(1575.42MHz)/CA码标频(1.023MHz)
	nav_up = 1.0;					// 导航解的基本单位，单位：秒
	speed = heading = 0;          // 接收机速度的绝对值和方位角
	d_tow = 0;								// 接收机时间差，单位：秒
	key = 0;								// 检查键盘的按键信息
	tic_count = 0;						// TIC计数器，也是测量中断发生次数的计数器
	// 注意：测量中断的频率这里缺省为：100ms
	// 计数范围为1s[0-9]
	hms_count = nav_count = min_flag = nav_flag = sec_flag = n_track = 0;							// 跟踪(通道处于tracking模式)的卫星个数
	interr_int = 512;
	clock_offset = 0.0;					// 接收机的钟漂，单位是ppm,初始值未知，假定为0
	// 正值表示接收机的频率低于标称值
	memset(&rec_pos_ecef, 0, sizeof(rec_pos_ecef));// 接收机的坐标
	i_TIC_dt = 0;							// 两次计算导航解的间隔,单位是样点个数
	TIC_dt = 0;								// 两次计算导航解的间隔,单位是秒
	// 所以TIC_dt=i_TIC_dt*采样间隔
	//memset(m_time, 0, sizeof(m_time));				// 接收机时间,[1]是当前时刻的，[0]是上一次的
	// [2]似乎没有用
	// m_time用于表示接收机的GPS时间
	// 在原来的程序中m_time没有有效的更新,
	// 它应该靠秒中断标志sec_flag来维护
	delta_m_time = m_error = 0;							// 接收机的测量时间和GPS整秒的误差
	TIC_sum = 0;							// 多个测量中断中期内累积的计数值
	//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
	//int			mstat;							// 仅用于GP2021，包含有miss data标志的寄存器
	caTable = 0;							// 非实时处理时，存储37×1023的CA码表
	DLLc1 = DLLc2 = PLLc0 = PLLc1 = PLLc2 = FLLa1 = FLLa2 = 0;						// FLL的系数
	// 调试用的全程变量
	IQQ = IQI = 0;
	memset(IBUF, 0, sizeof(IBUF));
	memset(QBUF, 0, sizeof(QBUF));
	//FILE		*fpIQ;
	IQCounter = 0;
	//FILE		*fpobs;
	//FILE		*fpeph;
	memset(detectNum, 0, sizeof(detectNum));					// 搜索某颗卫星的持续时间
	threshold_sig_search = 0;				// 信号检测门限		
	memset(minFreqErr, 0, sizeof(minFreqErr));
	memset(maxFreqErr, 0, sizeof(maxFreqErr));
	uLastPCTick = uCurrentPCTick = 0;
	iTotalBitCounter = 0;
	bLocalTimeSetByTow = false;
	bTowDecoded = false;
	DLLdT = 0.5;
	CARRIER_FREQ = 0;   //标称中频  0625
	SAMPLING_FREQ = SAMPLING_INT = 0;      //0626
	DETECTSIZE = TIC_ref = 0;           //0626
	deltaPhaseConst = deltaCodePhaseConst = 0;    //0626
	SAMPLESPMS = ACC_INT_PERIOD = 0;     //0627
	GPSL1_data = 0;              //0627
	FFTSIZE = bufSize = 0;            //0627
	//FILE   *fpCodeRange1, *fpCodeRange2;     //0628
	//FILE   *fpCarrRange1, *fpCarrRange2;     //0628
	ldcount = 0;                     //0628
	//FILE   *fpdoppler;                      //0628
	//FILE   *fpCNo;                          //0628
	memset(if_data_file, 0, sizeof(if_data_file));
	hot_cold = 0;
	//FILE *daicy_file_pr;
	//FILE *daicy_file_pos;
	//FILE   *out_trtime;
	memset(rcvr_par_file, 0, sizeof(rcvr_par_file));
	memset(wgs, 0, sizeof(wgs));
	pos_out_flag = 0;
	firsttime = 0;
	i4page = i5page = 0;
	is_state_3 = 0;
	first_flag = 1;
	judge_nits = 0;
	memset(t_cor, 0, sizeof(t_cor));
	towFlag = 0;
	GPSL1_pream = 0x22c00000L;
	pb1 = 0xbb1f3480L, pb2 = 0x5d8f9a40L, pb3 = 0xaec7cd00L;
	pb4 = 0x5763e680L, pb5 = 0x6bb1f340L, pb6 = 0x8b7a89c0L;


	//b1
	BD1_corr = 0;
	BD1_rCa = BD1_iCa = BD1_rInData = BD1_iInData = BD1_rProduct = BD1_iProduct = BD1_cosPhase = BD1_sinPhase = 0;
	BD1_chipIndex = 0;
	memset(BD1_sinTable, 0, sizeof(BD1_sinTable));
	memset(BD1_cosTable, 0, sizeof(BD1_cosTable));
	//int BD1_ADNumber;
	BD1_CAExtendedTable = 0;
	//BD1_productTable = 0;
	BD1_lgDataLength = 0;
	BD1_cgData = 0;
	/*HANDLE BD1_chn_start_event[BD1_chmax + 1];
	HANDLE BD1_chn_finish_event[BD1_chmax + 1];
	HANDLE BD1_chn_thread[BD1_chmax + 1];*/
	memset(BD1_chn_id, 0, sizeof(BD1_chn_id));
	memset(BD1_directory_path, 0, sizeof(BD1_directory_path));
	memset(BD1_tzstr, 0, sizeof(BD1_tzstr));
	memset(BD1_tzstr, 0, sizeof(BD1_tzstr));
	//time_t			BD1_thetime;
	//FILE			*BD1_stream, *BD1_debug, *BD1_in, *BD1_out, *BD1_kalm;
	//            *BD1_fprn_in, *BD1_fprn_out;
	memset(BD1_gps_alm, 0, sizeof(BD1_gps_alm));
	memset(BD1_gps_eph, 0, sizeof(BD1_gps_eph));
	memset(&BD1_availablesv, 0, sizeof(BD1_availablesv));

	memset(BD1_SVh, 0, sizeof(BD1_SVh));
	memset(BD1_ASV, 0, sizeof(BD1_ASV));
	BD1_b0 = BD1_b1 = BD1_b2 = BD1_b3 = BD1_al0 = BD1_al1 = BD1_al2 = BD1_al3 = BD1_a0 = BD1_a1 = BD1_tot = BD1_WNt = BD1_dtls = BD1_WNlsf = BD1_DN = BD1_dtlsf = 0;	//broadcast UTC data
	memset(&BD1_rpvt, 0, sizeof(BD1_rpvt));
	memset(&BD1_receiver, 0, sizeof(BD1_receiver));

	BD1_gdop = BD1_pdop = BD1_hdop = BD1_vdop = BD1_tdop = BD1_alm_toa = 0;
	BD1_clock_tow = 0;
	memset(&BD1_rec_pos_llh, 0, sizeof(BD1_rec_pos_llh));
	memset(&BD1_current_loc, 0, sizeof(BD1_current_loc));
	memset(&BD1_rp_llh, 0, sizeof(BD1_rp_llh));
	memset(BD1_track_sat, 0, sizeof(BD1_track_sat));
	memset(&BD1_rec_pos_xyz, 0, sizeof(BD1_rec_pos_xyz));
	BD1_alm_gps_week = BD1_gps_week = BD1_almanac_valid = BD1_almanac_flag = BD1_handle = 0;
	memset(BD1_sf, 0, sizeof(BD1_sf));						// 用于存储5个子帧10个字/页)的导航电文，13表示通道数，11表示页面数，6表示子桢数，11表示字数
	////////2010.11.16//////////////////////////////////////////////////////////////////
	memset(BD1_sfd1, 0, sizeof(BD1_sfd1));						// 用于存储5个子帧10个字/页)的导航电文，sf[0][]无效，sf[][0]无效
	memset(BD1_p_error, 0, sizeof(BD1_p_error));						// 用于存储5个子帧(10个字/页)的导航电文的校验状态
	// LSB 10位有效，每个比特对应一个字的校验状态，0：正确，1：错误

	BD1_status = 0;								// 表征接收机的状态，只有三个值
	// cold_start, warm_start和navigating

	unsigned long	TMP_3[] = { 0x00000000L,            // single bit set numbers
		0x00000001L, 0x00000002L, 0x00000004L, 0x00000008L,  // for testing bit positions
		0x00000010L, 0x00000020L, 0x00000040L, 0x00000080L,	// 0000 0001 0010 0100 1000
		0x00000100L, 0x00000200L, 0x00000400L, 0x00000800L,	// 32位的第1位到第32位
		0x00001000L, 0x00002000L, 0x00004000L, 0x00008000L,
		0x00010000L, 0x00020000L, 0x00040000L, 0x00080000L,
		0x00100000L, 0x00200000L, 0x00400000L, 0x00800000L,
		0x01000000L, 0x02000000L, 0x04000000L, 0x08000000L,
		0x10000000L, 0x20000000L, 0x40000000L, 0x80000000L };
	memcpy(BD1_test_l, TMP_3, sizeof(BD1_test_l));

	BD1_mask_angle = 0;
	memset(BD1_header, 0, sizeof(BD1_header));
	memset(BD1_ttext, 0, sizeof(BD1_ttext));
	BD1_trailer = 0;
	memset(BD1_meas_dop, 0, sizeof(BD1_meas_dop));
	memset(BD1_d_sat, 0, sizeof(BD1_d_sat));// 测量的每个通道的多普勒							// 每个通道的卫星的速度
	BD1_carrier_ref = 0;				// 载波标称中频，单位：Hz
	BD1_code_ref = 2046000;						// CA码速率标频，单位：chip/s
	memset(BD1_dt, 0, sizeof(BD1_dt));							// 从卫星到接收机的传播时间差
	BD1_cbias = 0;									// 接收机钟差

	// binary constants for nav message decoding

	BD1_c_2p12 = 4096;
	BD1_c_2p4 = 16;
	BD1_c_2m5 = 0.03125;
	BD1_c_2m6 = 0.015625;//add by jh 3-16
	BD1_c_2m11 = 4.8828125e-4;
	BD1_c_2m19 = 1.9073486328125e-6;
	BD1_c_2m20 = 9.5367431640625e-7;
	BD1_c_2m21 = 4.76837158203125e-7;
	BD1_c_2m23 = 1.19209289550781e-7;
	BD1_c_2m24 = 5.96046447753906e-8;
	BD1_c_2m27 = 7.45058059692383e-9;
	BD1_c_2m29 = 1.86264514923096e-9;
	BD1_c_2m30 = 9.31322574615479e-10;
	BD1_c_2m31 = 4.65661287307739e-10;
	BD1_c_2m33 = 1.16415321826935E-10;
	BD1_c_2m38 = 3.63797880709171e-12;
	BD1_c_2m43 = 1.13686837721616e-13;
	BD1_c_2m50 = 8.881784197e-16;
	BD1_c_2m55 = 2.77555756156289e-17;
	BD1_c_2m66 = 1.35525271560688e-20;
	BD1_m_tropo = BD1_m_iono = BD1_align_t = 0;						// 将接收机时间和GPS时间对准的标志
	memset(BD1_xyz, 0, sizeof(BD1_xyz));				// 卫星的位置、方位和多普勒
	memset(BD1_last_prn, 0, sizeof(BD1_last_prn));
	memset(BD1_curloc_file, 0, sizeof(BD1_curloc_file));
	BD1_daicy_temp_cnt = 0;
	memset(BD1_ion_utc_file, 0, sizeof(BD1_ion_utc_file));
	memset(BD1_current_alm_file, 0, sizeof(BD1_current_alm_file));
	memset(BD1_current_eph_file, 0, sizeof(BD1_current_eph_file));
	memset(BD1_last_prn_file, 0, sizeof(BD1_last_prn_file));
	memset(BD1_chan, 0, sizeof(BD1_chan));			// 全程变量，用于存储每个通道跟踪环路的状态
	memset(BD1_correlator, 0, sizeof(BD1_correlator));		// 全程变量，用于存储每个通道相关器的状态
	memset(BD1_svStruct, 0, sizeof(BD1_svStruct));	// 表示卫星可用性
	memset(BD1_globalBufLength, 0, sizeof(BD1_globalBufLength));	// 用于保存每个通道缓存的数据长度，需初始化
	memset(BD1_buffer, 0, sizeof(BD1_buffer));		// 用于保存每个通道的缓存数据，需初始化分配空间
	BD1_correlatorDataLength = BD1_TIC_RT_CNTR = BD1_TIC_CNTR = 0;					// TIC的最大计数，对应了TIC中断的发生周期
	// 注意：TIC_CNTR的赋值只能用programTIC函数实现
	BD1_TIC_CNTR_RES = BD1_TIC_CNTR_DBL = 0;
	BD1_TIC_OCCUR_FLAG = 0;			// TIC中断发生的标志

	BD1_display_page = 0;			// 控制显示第i个页面
	unsigned tmp_4[] = { 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
		0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };
	memcpy(BD1_test, tmp_4, sizeof(BD1_test));
	memset(BD1_tr_ch, 0, sizeof(BD1_tr_ch));			   // 索引0不用，内容为具有有效观测数据的chan的标号
	BD1_out_debug = BD1_out_pos = BD1_out_vel = BD1_out_time = 0;		// 是否输出debug和PVT信息的标志符
	memset(&BD1_cur_lat, 0, sizeof(BD1_cur_lat));
	memset(&BD1_cur_long, 0, sizeof(BD1_cur_long));		   // 用于显示经纬度


	// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
	BD1_nav_tic = BD1_ICP_CTL = 0;		         // 输出累积载波相位还是跟踪环路的值，标志
	BD1_code_corr = 0;              // CA码速率的修正控制字，1.023MHz对应1.0
	BD1_time_on = 0;	            // 计数器，步进为秒，程序不停，计数不止

	BD1_nav_up = 1.0;					// 导航解的基本单位，单位：秒
	BD1_speed = BD1_heading = 0;          // 接收机速度的绝对值和方位角
	BD1_d_tow = 0;								// 接收机时间差，单位：秒
	BD1_key = BD1_tic_count = 0;						// TIC计数器，也是测量中断发生次数的计数器
	// 注意：测量中断的频率这里缺省为：100ms
	// 计数范围为1s[0-9]
	BD1_hms_count = BD1_nav_count = BD1_min_flag = BD1_nav_flag = BD1_sec_flag = BD1_n_track = 0;							// 跟踪(通道处于tracking模式)的卫星个数
	BD1_interr_int = 512;
	BD1_clock_offset = 0.0;					// 接收机的钟漂，单位是ppm,初始值未知，假定为0
	// 正值表示接收机的频率低于标称值
	memset(&BD1_rec_pos_ecef, 0, sizeof(BD1_rec_pos_ecef));					// 接收机的坐标
	BD1_i_TIC_dt = 0;							// 两次计算导航解的间隔,单位是样点个数
	BD1_TIC_dt = 0;								// 两次计算导航解的间隔,单位是秒
	// 所以TIC_dt=i_TIC_dt*采样间隔
	memset(BD1_m_time, 0, sizeof(BD1_m_time));						// 接收机时间,[1]是当前时刻的，[0]是上一次的
	// [2]似乎没有用
	// m_time用于表示接收机的GPS时间
	// 在原来的程序中m_time没有有效的更新,
	// 它应该靠秒中断标志sec_flag来维护
	BD1_delta_m_time = BD1_m_error = 0;							// 接收机的测量时间和GPS整秒的误差

	BD1_TIC_sum = 0;							// 多个测量中断中期内累积的计数值
	//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
	//int			mstat;							// 仅用于GP2021，包含有miss data标志的寄存器
	BD1_caTable = 0;							// 非实时处理时，存储37×1023的CA码表

	BD1_DLLc1 = BD1_DLLc2 = BD1_PLLc0 = BD1_PLLc1 = BD1_PLLc2 = BD1_FLLa1 = BD1_FLLa2 = 0;						// FLL的系数

	// 调试用的全程变量
	BD1_IQQ = BD1_IQI = 0;
	memset(BD1_IBUF, 0, sizeof(BD1_IBUF));
	memset(BD1_QBUF, 0, sizeof(BD1_QBUF));
	//FILE		*BD1_fpIQ;
	BD1_IQCounter = 0;
	//FILE		*BD1_fpobs;
	//FILE		*BD1_fpeph;


	BD1_threshold_sig_search = 0;				// 信号检测门限							

	//char		data[SAMPLESPMS+1];							// 信号缓存区
	//char		data[6000];							// 信号缓存区
	memset(BD1_minFreqErr, 0, sizeof(BD1_minFreqErr));
	memset(BD1_maxFreqErr, 0, sizeof(BD1_maxFreqErr));

	BD1_uLastPCTick = BD1_uCurrentPCTick = 0;

	BD1_iTotalBitCounter = 0;

	BD1_bLocalTimeSetByTow = false;
	BD1_bTowDecoded = false;
	BD1_CARRIER_FREQ = 0;   //标称中频  0625
	BD1_SAMPLING_FREQ = 0;     //0626
	BD1_ACQ_SAMPLING_FREQ = 4.096e6;//2.048
	BD1_SAMPLING_INT = 0;      //0626
	BD1_DETECTSIZE = BD1_TIC_ref = 0;           //0626
	BD1_deltaPhaseConst = BD1_deltaCodePhaseConst = 0;    //0626
	BD1_SAMPLESPMS = BD1_ACC_INT_PERIOD = 0;     //0627
	BD1_data = 0;              //0627
	BD1_FFTSIZE = 0;            //0627
	//hide by jh 3-11 long    bufSize;            //0627

	//FILE   *BD1_fpCodeRange1, *BD1_fpCodeRange2;     //0628
	//FILE   *BD1_fpCarrRange1, *BD1_fpCarrRange2;     //0628
	BD1_ldcount = 0;                     //0628
	//FILE   *BD1_fpdoppler;                      //0628
	//FILE   *BD1_fpCNo;                          //0628
	//FILE   *BD1_out_trtime;

	memset(BD1_if_data_file, 0, sizeof(BD1_if_data_file));

	BD1_hot_cold = 0;
	//FILE *BD1_daicy_file_pr;
	//FILE *BD1_daicy_file_pos;
	memset(BD1_rcvr_par_file, 0, sizeof(BD1_rcvr_par_file));
	memset(BD1_wgs, 0, sizeof(BD1_wgs));

	BD1_bFirst = true;
	BD1_is_state_3 = 0;
	BD1_first_flag = 1;
	BD1_judge_nits = 0;
	memset(BD1_t_cor, 0, sizeof(BD1_t_cor));
	BD1_towFlag = 0;
	BD1_pream = 0x38900000L;  //wqnavmess
	BD1_pream1 = 0x07680000L;

	//b3
	BD3_corr = 0;
	BD3_rCa = BD3_iCa = 0;
	BD3_rInData = BD3_iInData = 0;
	BD3_rProduct = BD3_iProduct = 0;
	BD3_cosPhase = BD3_sinPhase = 0;
	BD3_chipIndex = 0;
	memset(BD3_sinTable, 0, sizeof(BD3_sinTable));
	memset(BD3_cosTable, 0, sizeof(BD3_cosTable));

	//int BD3_ADNumber;
	BD3_CAExtendedTable = 0;
	//BD3_productTable = 0;
	BD3_lgDataLength = 0;
	BD3_cgData = 0;
	///HANDLE BD3_chn_start_event[BD3_chmax + 1];
	//HANDLE BD3_chn_finish_event[BD3_chmax + 1];
	//HANDLE BD3_chn_thread[BD3_chmax + 1];
	memset(BD3_chn_id, 0, sizeof(BD3_chn_id));
	memset(BD3_directory_path, 0, sizeof(BD3_directory_path));
	memset(BD3_tzstr, 0, sizeof(BD3_tzstr));
	//time_t			BD3_thetime;
	//FILE			*BD3_stream, *BD3_debug, *BD3_in, *BD3_out, *BD3_kalm;
	//FILE            *BD3_fprn_in, *BD3_fprn_out;
	memset(BD3_gps_alm, 0, sizeof(BD3_gps_alm));
	memset(BD3_gps_eph, 0, sizeof(BD3_gps_eph));
	memset(&BD3_availablesv, 0, sizeof(BD3_availablesv));

	memset(BD3_SVh, 0, sizeof(BD3_SVh));
	memset(BD3_ASV, 0, sizeof(BD3_ASV));
	BD3_b0 = BD3_b1 = BD3_b2 = BD3_b3 = BD3_al0 = BD3_al1 = BD3_al2 = BD3_al3 =
		BD3_a0 = BD3_a1 = BD3_tot = BD3_WNt = BD3_dtls = BD3_WNlsf = BD3_DN = BD3_dtlsf = 0;	//broadcast UTC data

	memset(&BD3_rpvt, 0, sizeof(BD3_rpvt));
	memset(&BD3_receiver, 0, sizeof(BD3_receiver));

	BD3_gdop = BD3_pdop = BD3_hdop = BD3_vdop = BD3_tdop = BD3_alm_toa = 0;
	BD3_clock_tow = 0;
	memset(&BD3_rec_pos_llh, 0, sizeof(BD3_rec_pos_llh));
	memset(&BD3_current_loc, 0, sizeof(BD3_current_loc));
	memset(&BD3_current_loc, 0, sizeof(BD3_current_loc));
	memset(BD3_track_sat, 0, sizeof(BD3_track_sat));
	memset(&BD3_rec_pos_xyz, 0, sizeof(BD3_rec_pos_xyz));
	BD3_alm_gps_week = BD3_gps_week = BD3_almanac_valid = BD3_almanac_flag = BD3_handle = 0;
	memset(BD3_sf, 0, sizeof(BD3_sf));							// 用于存储5个子帧10个字/页)的导航电文，13表示通道数，11表示页面数，6表示子桢数，11表示字数
	////////2010.11.16//////////////////////////////////////////////////////////////////
	memset(BD3_sfd1, 0, sizeof(BD3_sfd1));						// 用于存储5个子帧10个字/页)的导航电文，sf[0][]无效，sf[][0]无效
	memset(BD3_p_error, 0, sizeof(BD3_p_error));					// 用于存储5个子帧(10个字/页)的导航电文的校验状态
	// LSB 10位有效，每个比特对应一个字的校验状态，0：正确，1：错误

	BD3_status = 0;								// 表征接收机的状态，只有三个值:
	// cold_start, warm_start和navigating

	unsigned long tmp_5[] = { 0x00000000L,            // single bit set numbers
		0x00000001L, 0x00000002L, 0x00000004L, 0x00000008L,  // for testing bit positions
		0x00000010L, 0x00000020L, 0x00000040L, 0x00000080L,	// 0000 0001 0010 0100 1000
		0x00000100L, 0x00000200L, 0x00000400L, 0x00000800L,	// 32位的第1位到第32位
		0x00001000L, 0x00002000L, 0x00004000L, 0x00008000L,
		0x00010000L, 0x00020000L, 0x00040000L, 0x00080000L,
		0x00100000L, 0x00200000L, 0x00400000L, 0x00800000L,
		0x01000000L, 0x02000000L, 0x04000000L, 0x08000000L,
		0x10000000L, 0x20000000L, 0x40000000L, 0x80000000L };
	memcpy(BD3_test_l, tmp_5, sizeof(BD3_test_l));

	BD3_mask_angle = 0;
	memset(BD3_header, 0, sizeof(BD3_header));
	memset(BD3_ttext, 0, sizeof(BD3_ttext));
	BD3_trailer = 0;
	memset(BD3_meas_dop, 0, sizeof(BD3_meas_dop));						// 测量的每个通道的多普勒
	memset(BD3_d_sat, 0, sizeof(BD3_d_sat));						// 每个通道的卫星的速度
	BD3_carrier_ref = 0;				// 载波标称中频，单位：Hz
	//double		code_ref=2046000;						// CA码速率标频，单位：chip/s
	BD3_code_ref = 10230000;
	memset(BD3_dt, 0, sizeof(BD3_dt));							// 从卫星到接收机的传播时间差
	BD3_cbias = 0;									// 接收机钟差

	// binary constants for nav message decoding

	BD3_c_2p12 = 4096;
	BD3_c_2p4 = 16;
	BD3_c_2m5 = 0.03125;
	BD3_c_2m6 = 0.015625;//add by jh 3-16
	BD3_c_2m11 = 4.8828125e-4;
	BD3_c_2m19 = 1.9073486328125e-6;
	BD3_c_2m20 = 9.5367431640625e-7;
	BD3_c_2m21 = 4.76837158203125e-7;
	BD3_c_2m23 = 1.19209289550781e-7;
	BD3_c_2m24 = 5.96046447753906e-8;
	BD3_c_2m27 = 7.45058059692383e-9;
	BD3_c_2m29 = 1.86264514923096e-9;
	BD3_c_2m30 = 9.31322574615479e-10;
	BD3_c_2m31 = 4.65661287307739e-10;
	BD3_c_2m33 = 1.16415321826935E-10;
	BD3_c_2m38 = 3.63797880709171e-12;
	BD3_c_2m43 = 1.13686837721616e-13;
	BD3_c_2m50 = 8.881784197e-16;
	BD3_c_2m55 = 2.77555756156289e-17;
	BD3_c_2m66 = 1.35525271560688e-20;

	BD3_m_tropo = BD3_m_iono = BD3_align_t = 0;						// 将接收机时间和GPS时间对准的标志
	memset(BD3_xyz, 0, sizeof(BD3_xyz));						// 卫星的位置、方位和多普勒

	memset(BD3_last_prn, 0, sizeof(BD3_last_prn));
	memset(BD3_curloc_file, 0, sizeof(BD3_curloc_file));
	BD3_daicy_temp_cnt = 0;
	memset(BD3_ion_utc_file, 0, sizeof(BD3_ion_utc_file));
	memset(BD3_current_alm_file, 0, sizeof(BD3_current_alm_file));
	memset(BD3_current_eph_file, 0, sizeof(BD3_current_eph_file));

	memset(BD3_chan, 0, sizeof(BD3_chan));				// 全程变量，用于存储每个通道跟踪环路的状态
	memset(BD3_correlator, 0, sizeof(BD3_correlator));		// 全程变量，用于存储每个通道相关器的状态
	memset(BD3_svStruct, 0, sizeof(BD3_svStruct));	// 表示卫星可用性
	memset(BD3_globalBufLength, 0, sizeof(BD3_globalBufLength));	// 用于保存每个通道缓存的数据长度，需初始化
	memset(BD3_buffer, 0, sizeof(BD3_buffer));			// 用于保存每个通道的缓存数据，需初始化分配空间
	BD3_correlatorDataLength = BD3_TIC_RT_CNTR = BD3_TIC_CNTR = 0;					// TIC的最大计数，对应了TIC中断的发生周期
	// 注意：TIC_CNTR的赋值只能用programTIC函数实现
	BD3_TIC_CNTR_RES = BD3_TIC_CNTR_DBL = 0;
	BD3_TIC_OCCUR_FLAG = 0;			// TIC中断发生的标志

	BD3_display_page = 0;			// 控制显示第i个页面
	unsigned tmp_6[] = { 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
		0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };
	memcpy(BD3_test, tmp_6, sizeof(BD3_test));
	memset(BD3_tr_ch, 0, sizeof(BD3_tr_ch));			   // 索引0不用，内容为具有有效观测数据的chan的标号
	BD3_out_debug = BD3_out_pos = BD3_out_vel = BD3_out_time = 0;		// 是否输出debug和PVT信息的标志符
	memset(&BD3_cur_lat, 0, sizeof(BD3_cur_lat));
	memset(&BD3_cur_long, 0, sizeof(BD3_cur_long));		   // 用于显示经纬度


	// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
	BD3_nav_tic = BD3_ICP_CTL = 0;		         // 输出累积载波相位还是跟踪环路的值，标志
	BD3_code_corr = 0;              // CA码速率的修正控制字，1.023MHz对应1.0
	BD3_time_on = 0;	            // 计数器，步进为秒，程序不停，计数不止

	BD3_nav_up = 1.0;					// 导航解的基本单位，单位：秒
	BD3_speed = BD3_heading = 0;          // 接收机速度的绝对值和方位角
	BD3_d_tow = 0;								// 接收机时间差，单位：秒
	BD3_key = 0;								// 检查键盘的按键信息
	BD3_tic_count = 0;						// TIC计数器，也是测量中断发生次数的计数器
	// 注意：测量中断的频率这里缺省为：100ms
	// 计数范围为1s[0-9]
	BD3_hms_count = 0;						// TIC计数器，计数范围为1分钟[0-599]
	BD3_nav_count = BD3_min_flag = BD3_nav_flag = BD3_sec_flag = BD3_n_track = 0;							// 跟踪(通道处于tracking模式)的卫星个数
	BD3_interr_int = 512;
	BD3_clock_offset = 0.0;					// 接收机的钟漂，单位是ppm,初始值未知，假定为0
	// 正值表示接收机的频率低于标称值
	memset(&BD3_rec_pos_ecef, 0, sizeof(BD3_rec_pos_ecef));				// 接收机的坐标
	BD3_i_TIC_dt = 0;							// 两次计算导航解的间隔,单位是样点个数
	BD3_TIC_dt = 0;								// 两次计算导航解的间隔,单位是秒
	// 所以TIC_dt=i_TIC_dt*采样间隔
	memset(BD3_m_time, 0, sizeof(BD3_m_time));			// 接收机时间,[1]是当前时刻的，[0]是上一次的
	// [2]似乎没有用
	// m_time用于表示接收机的GPS时间
	// 在原来的程序中m_time没有有效的更新,
	// 它应该靠秒中断标志sec_flag来维护
	BD3_delta_m_time = BD3_m_error = 0;							// 接收机的测量时间和GPS整秒的误差

	BD3_TIC_sum = 0;							// 多个测量中断中期内累积的计数值
	//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
	//int			mstat;							// 仅用于GP2021，包含有miss data标志的寄存器
	BD3_caTable = 0;							// 非实时处理时，存储14×10230的CA码表

	BD3_DLLc1 = BD3_DLLc2 = BD3_PLLc0 = BD3_PLLc1 = BD3_PLLc2 = BD3_FLLa1 = BD3_FLLa2 = 0;						// FLL的系数

	// 调试用的全程变量
	BD3_IQQ = BD3_IQI = 0;
	memset(BD3_IBUF, 0, sizeof(BD3_IBUF));
	memset(BD3_QBUF, 0, sizeof(BD3_QBUF));
	//FILE		*BD3_fpIQ;
	BD3_IQCounter = 0;
	//FILE		*BD3_fpobs;
	//FILE		*BD3_fpeph;
	BD3_threshold_sig_search = 0;				// 信号检测门限	
	memset(BD3_minFreqErr, 0, sizeof(BD3_minFreqErr));
	memset(BD3_maxFreqErr, 0, sizeof(BD3_maxFreqErr));
	BD3_uLastPCTick = BD3_uCurrentPCTick = 0;
	BD3_iTotalBitCounter = 0;
	BD3_bLocalTimeSetByTow = false;
	BD3_bTowDecoded = false;
	BD3_CARRIER_FREQ = 0;   //标称中频  0625
	BD3_SAMPLING_FREQ = 0;     //0626
	//double	ACQ_SAMPLING_FREQ = 4.096e6;//2.048
	BD3_ACQ_SAMPLING_FREQ = 11 * 2.048e6;//revised
	BD3_SAMPLING_INT = 0;      //0626
	BD3_DETECTSIZE = BD3_TIC_ref = 0;           //0626
	BD3_deltaPhaseConst = BD3_deltaCodePhaseConst = 0;    //0626
	BD3_SAMPLESPMS = BD3_ACC_INT_PERIOD = 0;     //0627
	BD3_data = 0;              //0627
	BD3_FFTSIZE = 0;            //0627
	//FILE   *BD3_fpCodeRange1, *BD3_fpCodeRange2;     //0628
	//FILE   *BD3_fpCarrRange1, *BD3_fpCarrRange2;     //0628
	BD3_ldcount = 0;                     //0628
	//FILE   *BD3_fpdoppler;                      //0628
	//FILE   *BD3_fpCNo;                          //0628
	//FILE   *BD3_out_trtime;
	memset(BD3_if_data_file, 0, sizeof(BD3_if_data_file));
	BD3_hot_cold = 0;
	//FILE *BD3_daicy_file_pr;
	//FILE *BD3_daicy_file_pos;
	memset(BD3_rcvr_par_file, 0, sizeof(BD3_rcvr_par_file));
	memset(BD3_wgs, 0, sizeof(BD3_wgs));

	BD3_bFirst = true;
	BD3_is_state_3 = 0;
	BD3_first_flag = 1;
	BD3_judge_nits = 0;
	memset(BD3_t_cor, 0, sizeof(BD3_t_cor));
	BD3_towFlag = 0;
	BD3_pream = 0x38900000L;  //wqnavmess
	BD3_pream1 = 0x07680000L;


	//glo1
	GLO1_corr = 0;
	GLO1_rCa = GLO1_iCa = GLO1_rInData = GLO1_iInData = GLO1_rProduct = GLO1_iProduct = GLO1_cosPhase = GLO1_sinPhase = 0;
	memset(GLO1_sinTable, 0, sizeof(GLO1_sinTable));
	memset(GLO1_cosTable, 0, sizeof(GLO1_cosTable));

	//int GLO1_ADNumber;

	GLO1_CAExtendedTable = 0;
	//GLO1_productTable = 0;
	GLO1_lgDataLength = 0;
	GLO1_cgData = 0;
	//HANDLE GLO1_chn_start_event[GLO1_chmax + 1];
	// HANDLE GLO1_chn_finish_event[GLO1_chmax + 1];
	//HANDLE GLO1_chn_thread[GLO1_chmax + 1];
	memset(GLO1_chn_id, 0, sizeof(GLO1_chn_id));
	memset(GLO1_directory_path, 0, sizeof(GLO1_directory_path));
	memset(GLO1_tzstr, 0, sizeof(GLO1_tzstr));
	//time_t			GLO1_thetime;
	// FILE			*GLO1_stream, *GLO1_debug, *GLO1_in, *GLO1_out, *GLO1_kalm;
	// FILE            *GLO1_fprn_in, *GLO1_fprn_out;
	memset(GLO1_gps_alm, 0, sizeof(GLO1_gps_alm));
	memset(GLO1_gps_eph, 0, sizeof(GLO1_gps_eph));

	memset(&GLO1_availablesv, 0, sizeof(GLO1_availablesv));
	memset(GLO1_glonass_sv_id_ephemeris, 0, sizeof(GLO1_glonass_sv_id_ephemeris));
	memset(GLO1_glonass_sv_id_almanac_str5, 0, sizeof(GLO1_glonass_sv_id_almanac_str5));
	memset(GLO1_SVh, 0, sizeof(GLO1_SVh));
	memset(GLO1_ASV, 0, sizeof(GLO1_ASV));
	GLO1_b0 = GLO1_b1 = GLO1_b2 = GLO1_b3 = GLO1_al0 = GLO1_al1 = GLO1_al2 = GLO1_al3 =
		GLO1_a0, GLO1_a1 = GLO1_tot = GLO1_WNt = GLO1_dtls = GLO1_WNlsf = GLO1_DN = GLO1_dtlsf = 0;
	memset(&GLO1_rpvt, 0, sizeof(GLO1_rpvt));
	memset(&GLO1_receiver, 0, sizeof(GLO1_receiver));

	GLO1_gdop = GLO1_pdop = GLO1_hdop = GLO1_vdop = GLO1_tdop = GLO1_alm_toa = 0;
	GLO1_clock_tow = 0;
	memset(&GLO1_rec_pos_llh, 0, sizeof(GLO1_rec_pos_llh));
	memset(&GLO1_current_loc, 0, sizeof(GLO1_current_loc));
	memset(&GLO1_rp_llh, 0, sizeof(GLO1_rp_llh));
	memset(GLO1_track_sat, 0, sizeof(GLO1_track_sat));
	memset(&GLO1_rec_pos_xyz, 0, sizeof(GLO1_rec_pos_xyz));
	GLO1_alm_gps_week = GLO1_gps_week = GLO1_almanac_valid = GLO1_almanac_flag = GLO1_handle = 0;
	memset(GLO1_sf, 0, sizeof(GLO1_sf));
	memset(GLO1_p_error, 0, sizeof(GLO1_p_error));


	GLO1_status = 0;

	unsigned long tmp_7[] = { 0x00000000L,            // single bit set numbers
		0x00000001L, 0x00000002L, 0x00000004L, 0x00000008L,  // for testing bit positions
		0x00000010L, 0x00000020L, 0x00000040L, 0x00000080L,	// 0000 0001 0010 0100 1000
		0x00000100L, 0x00000200L, 0x00000400L, 0x00000800L,	// 32位的第1位到第32位
		0x00001000L, 0x00002000L, 0x00004000L, 0x00008000L,
		0x00010000L, 0x00020000L, 0x00040000L, 0x00080000L,
		0x00100000L, 0x00200000L, 0x00400000L, 0x00800000L,
		0x01000000L, 0x02000000L, 0x04000000L, 0x08000000L,
		0x10000000L, 0x20000000L, 0x40000000L, 0x80000000L };
	memcpy(GLO1_test_l, tmp_7, sizeof(GLO1_test_l));

	GLO1_mask_angle = 0;
	memset(GLO1_header, 0, sizeof(GLO1_header));
	memset(GLO1_ttext, 0, sizeof(GLO1_ttext));
	GLO1_trailer = 0;
	memset(GLO1_meas_dop, 0, sizeof(GLO1_meas_dop));					// 测量的每个通道的多普勒
	memset(GLO1_d_sat, 0, sizeof(GLO1_d_sat));							// 每个通道的卫星的速度
	GLO1_carrier_ref = 0;				// 载波标称中频，单位：Hz
	//double		code_ref=1023000;						// CA码速率标频，单位：chip/s
	GLO1_code_ref = 511000;						// CA码速率标频，单位：chip/s
	memset(GLO1_dt, 0, sizeof(GLO1_dt));								// 从卫星到接收机的传播时间差
	GLO1_cbias = 0;									// 接收机钟差
	memset(GLO1_lambda, 0, sizeof(GLO1_lambda));
	// binary constants for nav message decoding

	GLO1_c_2p12 = 4096;
	GLO1_c_2p4 = 16;
	GLO1_c_2m5 = 0.03125;
	GLO1_c_2m11 = 4.8828125e-4;
	GLO1_c_2m19 = 1.9073486328125e-6;
	GLO1_c_2m20 = 9.5367431640625e-7;
	GLO1_c_2m21 = 4.76837158203125e-7;
	GLO1_c_2m23 = 1.19209289550781e-7;
	GLO1_c_2m24 = 5.96046447753906e-8;
	GLO1_c_2m27 = 7.45058059692383e-9;
	GLO1_c_2m29 = 1.86264514923096e-9;
	GLO1_c_2m30 = 9.31322574615479e-10;
	GLO1_c_2m31 = 4.65661287307739e-10;
	GLO1_c_2m33 = 1.16415321826935E-10;
	GLO1_c_2m38 = 3.63797880709171e-12;
	GLO1_c_2m43 = 1.13686837721616e-13;
	GLO1_c_2m50 = 8.881784197e-16;
	GLO1_c_2m55 = 2.77555756156289e-17;

	GLO1_m_tropo = GLO1_m_iono = 0;				// flags for using tropo and iono models
	GLO1_align_t = 0;						// 将接收机时间和GPS时间对准的标志
	memset(GLO1_xyz, 0, sizeof(GLO1_xyz));				// 卫星的位置、方位和多普勒
	memset(GLO1_last_prn, 0, sizeof(GLO1_last_prn));
	memset(GLO1_curloc_file, 0, sizeof(GLO1_curloc_file));

	unsigned char tmp_8[] = { 0, 10, 19, 29, 41 };
	memcpy(GLO1_decode_glonass_index, tmp_8, sizeof(GLO1_decode_glonass_index));
	unsigned char tmp_9[][5] = {
			{ 49, 49, 49, 49, 49 },		//string 6
			{ 58, 58, 58, 58, 58 },		//string 7
			{ 49, 49, 49, 49, 49 },		//string 8
			{ 58, 58, 58, 58, 58 },		//string 9
			{ 49, 49, 49, 49, 49 },		//string 10
			{ 58, 58, 58, 58, 58 },		//string 11
			{ 49, 49, 49, 49, 49 },		//string 12
			{ 58, 58, 58, 58, 58 },		//string 13
			{ 49, 49, 49, 49, 66 },		//string 14
			{ 58, 58, 58, 58, 72 },		//string 15
	};
	memcpy(GLO1_decode_glonass_frame, tmp_9, sizeof(GLO1_decode_glonass_frame));

	//定义电文格式************电文位计数:  85--1

	//#define		glonass_m				0
	//#define		glonass_KX				1
	//#define		glonass_resv1			2
	//
	//#define		glonass_e_P1			3
	//#define		glonass_e_tk_hour		4
	//#define		glonass_e_tk_min		5
	//#define		glonass_e_tk_sec		6
	//#define		glonass_e_dot_Xn		7
	//#define		glonass_e_doudot_Xn		8
	//#define		glonass_e_Xn			9
	//
	//	  /*string 2*/
	//#define		glonass_e_Bn			3
	//#define		glonass_e_P2			4
	//#define		glonass_e_tb			5
	//#define		glonass_e_dot_Yn		6
	//#define		glonass_e_doudot_Yn		7
	//#define		glonass_e_Yn			8
	//
	//	  /*string 3*/
	//#define		glonass_e_P3			3
	//#define		glonass_e_gamman		4
	//#define		glonass_e_P				5
	//#define		glonass_e_ln			6
	//#define		glonass_e_dot_Zn		7
	//#define		glonass_e_doudot_Zn		8
	//#define		glonass_e_Zn			9
	//
	//	  /*string 4*/
	//#define		glonass_resv2			3
	//#define		glonass_e_taun			4
	//#define		glonass_e_deltataun		5
	//#define		glonass_e_En			6
	//#define		glonass_e_P4			7
	//#define		glonass_e_FT			8
	//#define		glonass_e_NT			9
	//#define		glonass_e_n				10
	//#define		glonass_e_Mod			11
	//
	//	  /*string 5*/
	//#define		glonass_a_daynum		3
	//#define		glonass_a_tauc			4
	//#define		glonass_a_N4			5
	//#define		glonass_a_taugps		6
	//#define		glonass_a_str5_ln		7
	//
	//	  /*string 6 8 10 12 14,frame 1 2 3 4 & string 6 8 10 12,frame 5*/
	//#define		glonass_a_Cn			2
	//#define		glonass_a_Mn			3
	//#define		glonass_a_n				4
	//#define		glonass_a_taun			5
	//#define		glonass_a_lambdan		6
	//#define		glonass_a_deltain		7
	//#define		glonass_a_epsilonn		8
	//
	//	  /*string 7 9 11 13 15,frame 1 2 3 4 &string 7 9 11 13,frame 5*/
	//#define		glonass_a_omegan		2
	//#define		glonass_a_tlambdan		3
	//#define		glonass_a_deltaTn		4
	//#define		glonass_a_dot_deltaTn	5
	//#define		glonass_a_Hn			6
	//#define		glonass_a_ln			7
	//
	//	  /*string 14,frame 5 */
	//#define		glonass_B1				3
	//#define		glonass_B2				4
	//#define		glonass_KP				5
	//
	//	  /*string 15,frame 5 */
	//#define		glonass_str15_ln		3
	//	  GLO1_decode_glonass_info GLO1_glonass_formats[] = {
	//		  /*string 1*/
	//		  { glonass_m, 0, 0, 81, 4 },
	//		  { glonass_KX, 0, 0, 1, 8 },
	//		  { glonass_resv1, 0, 0, 79, 2 },
	//		  { glonass_e_P1, 0, 0, 77, 2 },
	//		  { glonass_e_tk_hour, 0, 0, 72, 5 },
	//		  { glonass_e_tk_min, 0, 0, 66, 6 },
	//		  { glonass_e_tk_sec, 30, 0, 65, 1 },
	//		  { glonass_e_dot_Xn, -20, 1, 41, 24 },
	//		  { glonass_e_doudot_Xn, -30, 1, 36, 5 },
	//		  { glonass_e_Xn, -11, 1, 9, 27 },
	//
	//		  /*string 2*/
	//		  { glonass_m, 0, 0, 81, 4 },
	//		  { glonass_KX, 0, 0, 1, 8 },
	//		  { glonass_resv1, 0, 0, 65, 5 },
	//		  { glonass_e_Bn, 0, 0, 78, 3 },
	//		  { glonass_e_P2, 0, 0, 77, 1 },
	//		  { glonass_e_tb, 15, 0, 70, 7 },
	//		  { glonass_e_dot_Yn, -20, 1, 41, 24 },
	//		  { glonass_e_doudot_Yn, -30, 1, 36, 5 },
	//		  { glonass_e_Yn, -11, 1, 9, 27 },
	//
	//		  /*string 3*/
	//		  { glonass_m, 0, 0, 81, 4 },
	//		  { glonass_KX, 0, 0, 1, 8 },
	//		  { glonass_resv1, 0, 0, 68, 1 },
	//		  { glonass_e_P3, 0, 0, 80, 1 },
	//		  { glonass_e_gamman, -40, 1, 69, 11 },
	//		  { glonass_e_P, 0, 0, 66, 2 },
	//		  { glonass_e_ln, 0, 0, 65, 1 },
	//		  { glonass_e_dot_Zn, -20, 1, 41, 24 },
	//		  { glonass_e_doudot_Zn, -30, 1, 36, 5 },
	//		  { glonass_e_Zn, -11, 1, 9, 27 },
	//
	//		  /*string 4*/
	//		  { glonass_m, 0, 0, 81, 4 },
	//		  { glonass_KX, 0, 0, 1, 8 },
	//		  { glonass_resv1, 0, 0, 35, 14 },
	//		  { glonass_resv2, 0, 0, 27, 3 },
	//		  { glonass_e_taun, -30, 1, 59, 22 },
	//		  { glonass_e_deltataun, -30, 1, 54, 5 },
	//		  { glonass_e_En, 0, 0, 49, 5 },
	//		  { glonass_e_P4, 0, 0, 34, 1 },
	//		  { glonass_e_FT, 0, 0, 30, 4 },
	//		  { glonass_e_NT, 0, 0, 16, 11 },
	//		  { glonass_e_n, 0, 0, 11, 5 },
	//		  { glonass_e_Mod, 0, 0, 9, 2 },
	//
	//		  /*string 5*/
	//		  { glonass_m, 0, 0, 81, 4 },
	//		  { glonass_KX, 0, 0, 1, 8 },
	//		  { glonass_resv1, 0, 0, 37, 1 },
	//		  { glonass_a_daynum, 0, 0, 70, 11 },
	//		  { glonass_a_tauc, -31, 1, 38, 32 },
	//		  { glonass_a_N4, 0, 0, 32, 5 },
	//		  { glonass_a_taugps, -30, 1, 10, 22 },
	//		  { glonass_a_str5_ln, 0, 0, 9, 1 },
	//
	//		  /*string 6 8 10 12 14,frame 1 2 3 4 & string 6 8 10 12,frame 5*/
	//		  { glonass_m, 0, 0, 81, 4 },
	//		  { glonass_KX, 0, 0, 1, 8 },
	//		  { glonass_a_Cn, 0, 0, 80, 1 },
	//		  { glonass_a_Mn, 0, 0, 78, 2 },
	//		  { glonass_a_n, 0, 0, 73, 5 },
	//		  { glonass_a_taun, -18, 1, 63, 10 },
	//		  { glonass_a_lambdan, -20, 1, 42, 21 },
	//		  { glonass_a_deltain, -20, 1, 24, 18 },
	//		  { glonass_a_epsilonn, -20, 0, 9, 15 },
	//
	//		  /*string 7 9 11 13 15,frame 1 2 3 4 &string 7 9 11 13,frame 5*/
	//		  { glonass_m, 0, 0, 81, 4 },
	//		  { glonass_KX, 0, 0, 1, 8 },
	//		  { glonass_a_omegan, -15, 1, 65, 16 },
	//		  { glonass_a_tlambdan, -5, 0, 44, 21 },
	//		  { glonass_a_deltaTn, -9, 1, 22, 22 },
	//		  { glonass_a_dot_deltaTn, -14, 1, 15, 7 },
	//		  { glonass_a_Hn, 0, 0, 10, 5 },
	//		  { glonass_a_ln, 0, 0, 9, 1 },
	//
	//		  /*string 14,frame 5 */
	//		  { glonass_m, 0, 0, 81, 4 },
	//		  { glonass_KX, 0, 0, 1, 8 },
	//		  { glonass_resv1, 0, 0, 9, 49 },
	//		  { glonass_B1, -10, 1, 70, 11 },
	//		  { glonass_B2, -16, 1, 60, 10 },
	//		  { glonass_KP, 0, 0, 58, 2 },
	//
	//		  /*string 15,frame 5 */
	//		  { glonass_m, 0, 0, 81, 4 },
	//		  { glonass_KX, 0, 0, 1, 8 },
	//		  { glonass_resv1, 0, 0, 10, 79 },
	//		  { glonass_str15_ln, 0, 0, 9, 1 }
	//	  };
	GLO1_daicy_temp_cnt = 0;

	memset(GLO1_ion_utc_file, 0, sizeof(GLO1_ion_utc_file));
	memset(GLO1_current_alm_file, 0, sizeof(GLO1_current_alm_file));
	memset(GLO1_current_eph_file, 0, sizeof(GLO1_current_eph_file));
	memset(GLO1_last_prn_file, 0, sizeof(GLO1_last_prn_file));
	memset(GLO1_chan, 0, sizeof(GLO1_chan));				// 全程变量，用于存储每个通道跟踪环路的状态
	memset(GLO1_correlator, 0, sizeof(GLO1_correlator));	// 全程变量，用于存储每个通道相关器的状态
	memset(GLO1_svStruct, 0, sizeof(GLO1_svStruct));	// 表示卫星可用性
	memset(GLO1_globalBufLength, 0, sizeof(GLO1_globalBufLength));	// 用于保存每个通道缓存的数据长度，需初始化
	memset(GLO1_buffer, 0, sizeof(GLO1_buffer));		// 用于保存每个通道的缓存数据，需初始化分配空间
	GLO1_correlatorDataLength = GLO1_TIC_RT_CNTR = GLO1_TIC_CNTR = 0;					// TIC的最大计数，对应了TIC中断的发生周期
	// 注意：GLO1_TIC_CNTR的赋值只能用GLO1_programTIC函数实现
	GLO1_TIC_CNTR_RES = GLO1_TIC_CNTR_DBL = 0;
	GLO1_TIC_OCCUR_FLAG = 0;			// TIC中断发生的标志

	GLO1_display_page = 0;			// 控制显示第i个页面
	unsigned tmp_10[] = { 0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040, 0x0080,
		0x0100, 0x0200, 0x0400, 0x0800, 0x1000, 0x2000, 0x4000, 0x8000 };
	memcpy(GLO1_test, tmp_10, sizeof(GLO1_test));
	memset(GLO1_tr_ch, 0, sizeof(GLO1_tr_ch));			   // 索引0不用，内容为具有有效观测数据的GLO1_chan的标号
	GLO1_out_debug = GLO1_out_pos = GLO1_out_vel = GLO1_out_time = 0;		// 是否输出GLO1_debug和GLO1_PVT信息的标志符
	memset(&GLO1_cur_lat, 0, sizeof(GLO1_cur_lat));
	memset(&GLO1_cur_long, 0, sizeof(GLO1_cur_long));			   // 用于显示经纬度

	// 以下定义变量，其缺省值可以通过改变文件rcvr_par.dat的设定实现
	GLO1_nav_tic = GLO1_ICP_CTL = 0;		         // 输出累积载波相位还是跟踪环路的值，标志
	GLO1_ICP_code_corr = 0;              // CA码速率的修正控制字，1.023MHz对应1.0
	GLO1_time_on = 0;	            // 计数器，步进为秒，程序不停，计数不止
	// long		      GLO1_cc_scale=1540;       	// 载波标频(1575.42MHz)/CA码标频(1.023MHz)
	GLO1_cc_scale = 3136;       	// 载波标频(1602+k*0.5625MHz)/CA码标频(0.511MHz)   todo  多颗星需要改变,但此变量后面没有使用
	memset(GLO1_alloced_k, 0, sizeof(GLO1_alloced_k));  // 记录某个 k 是否已经分配通道 k = -7 --- 6

	GLO1_nav_up = 1.0;					// 导航解的基本单位，单位：秒
	GLO1_speed = GLO1_heading = 0;          // 接收机速度的绝对值和方位角
	GLO1_d_tow = 0;								// 接收机时间差，单位：秒
	GLO1_key = 0;								// 检查键盘的按键信息
	GLO1_tic_count = 0;						// TIC计数器，也是测量中断发生次数的计数器
	// 注意：测量中断的频率这里缺省为：100ms
	// 计数范围为1s[0-9]
	GLO1_hms_count = GLO1_nav_count = GLO1_min_flag = GLO1_nav_flag = GLO1_sec_flag = GLO1_n_track = 0;							// 跟踪(通道处于tracking模式)的卫星个数
	GLO1_interr_int = 512;
	GLO1_clock_offset = 0.0;					// 接收机的钟漂，单位是ppm,初始值未知，假定为0
	// 正值表示接收机的频率低于标称值
	memset(&GLO1_rec_pos_ecef, 0, sizeof(GLO1_rec_pos_ecef));					// 接收机的坐标
	GLO1_i_TIC_dt = 0;							// 两次计算导航解的间隔,单位是样点个数
	GLO1_TIC_dt = 0;								// 两次计算导航解的间隔,单位是秒
	// 所以GLO1_TIC_dt=GLO1_i_TIC_dt*采样间隔
	memset(GLO1_m_time, 0, sizeof(GLO1_m_time));						// 接收机时间,[1]是当前时刻的，[0]是上一次的
	// [2]似乎没有用
	// GLO1_m_time用于表示接收机的GPS时间
	// 在原来的程序中GLO1_m_time没有有效的更新,
	// 它应该靠秒中断标志GLO1_sec_flag来维护
	GLO1_delta_m_time = GLO1_m_error = 0;							// 接收机的测量时间和GPS整秒的误差
	GLO1_TIC_sum = 0;							// 多个测量中断中期内累积的计数值
	//int			astat;							// 仅用于GP2021，包含有dump ready标志的寄存器
	//int			mstat;							// 仅用于GP2021，包含有miss GLO1_data标志的寄存器
	GLO1_caTable = 0;							// 非实时处理时，存储37×1023的CA码表

	GLO1_DLLc1 = GLO1_DLLc2 = GLO1_PLLc0 = GLO1_PLLc1 = GLO1_PLLc2 = GLO1_FLLa1 = GLO1_FLLa2 = 0;						// FLL的系数

	// 调试用的全程变量
	GLO1_IQQ = GLO1_IQI = 0;
	memset(GLO1_IBUF, 0, sizeof(GLO1_IBUF));
	memset(GLO1_QBUF, 0, sizeof(GLO1_QBUF));
	// FILE		*GLO1_fpIQ;
	GLO1_IQCounter = 0;
	//FILE		*GLO1_fpobs;
	//FILE		*GLO1_fpeph;
	memset(GLO1_detectNum, 0, sizeof(GLO1_detectNum));				// 搜索某颗卫星的持续时间
	GLO1_threshold_sig_search = 0;				// 信号检测门限							

	//char		GLO1_data[GLO1_SAMPLESPMS+1];							// 信号缓存区
	//char		GLO1_data[6000];							// 信号缓存区
	memset(GLO1_minFreqErr, 0, sizeof(GLO1_minFreqErr));
	memset(GLO1_maxFreqErr, 0, sizeof(GLO1_maxFreqErr));

	GLO1_uLastPCTick = GLO1_uCurrentPCTick = 0;

	GLO1_iTotalBitCounter = 0;

	GLO1_bLocalTimeSetByTow = false;
	GLO1_bTowDecoded = false;

	GLO1_CARRIER_FREQ = 0;   //标称中频  0625 //todo:GLONASS中心频率
	GLO1_SAMPLING_FREQ = GLO1_SAMPLING_INT = 0;      //0626
	GLO1_DETECTSIZE = GLO1_TIC_ref = 0;           //0626
	GLO1_deltaPhaseConst = GLO1_deltaCodePhaseConst = 0;    //0626
	GLO1_SAMPLESPMS = GLO1_ACC_INT_PERIOD = 0;     //0627
	GLO1_data = 0;              //0627
	GLO1_FFTSIZE = GLO1_bufSize = 0;            //0627

	//FILE   *GLO1_fpCodeRange1, *GLO1_fpCodeRange2;     //0628
	// FILE   *GLO1_fpCarrRange1, *GLO1_fpCarrRange2;     //0628
	GLO1_ldcount = 0;                     //0628
	//FILE   *GLO1_fpdoppler;                      //0628
	//FILE   *GLO1_fpCNo;                          //0628
	memset(GLO1_if_data_file, 0, sizeof(GLO1_if_data_file));
	memset(GLO1_glonass_channel_ephemeris, 0, sizeof(GLO1_glonass_channel_ephemeris));
	memset(GLO1_glonass_sv_id_almanac, 0, sizeof(GLO1_glonass_sv_id_almanac));
	GLO1_palc_str5 = new GLO1_glonass_almanac_str5;
	GLO1_palc_glob = new GLO1_glonass_almanac_global;
	memset(GLO1_unpack_glonass_flag, 0, sizeof(GLO1_unpack_glonass_flag));

	int16s tmp_12[] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	memcpy(GLO1_glonass_ephemeris_processing, tmp_12, sizeof(tmp_12));
	GLO1_glonass_frame_id = new int32u;
	memcpy(GLO1_glonass_almanac_processing, tmp_12, sizeof(tmp_12));
	memcpy(GLO1_glonass_almanac_global_processing, tmp_12, sizeof(tmp_12));
	memset(GLO1_string_1, 0, sizeof(GLO1_string_1));
	GLO1_hot_cold = 0;
	//FILE *GLO1_daicy_file_pr;
	// FILE *GLO1_daicy_file_pos;
	//FILE   *GLO1_out_trtime;
	memset(GLO1_rcvr_par_file, 0, sizeof(GLO1_rcvr_par_file));
	memset(GLO1_wgs, 0, sizeof(GLO1_wgs));
	GLO1_pos_out_flag = 0;

	GLO1_judge_nits = 0;
	memset(GLO1_t_cor, 0, sizeof(GLO1_t_cor));
	GLO1_towFlag = 0;
	GLO1_GLO1_pream = 0x3e375096;   //11 1110 0011 0111 0101 0000 1001 0110
	// GLO1_pream1 = 0x1c8af69; //00 0001 1100 1000 1010 1111 0110 1001 反相码
	GLO1_is_state_3 = 0;
	GLO1_first_flag = 1;
	GLO1_bFirst = true;
	GLO1_firsttime = 0;
	memset(GLO1_cx, 0, sizeof(GLO1_cx));
	memset(GLO1_cy, 0, sizeof(GLO1_cy));
	memset(GLO1_cz, 0, sizeof(GLO1_cz));

	memset(GLO1_dx, 0, sizeof(GLO1_dx));
	memset(GLO1_dy, 0, sizeof(GLO1_dy));
	memset(GLO1_dz, 0, sizeof(GLO1_dz));

	memset(GLO1_ux, 0, sizeof(GLO1_ux));
	memset(GLO1_uy, 0, sizeof(GLO1_uy));
	memset(GLO1_uz, 0, sizeof(GLO1_uz));

	memset(GLO1_px, 0, sizeof(GLO1_px));
	memset(GLO1_py, 0, sizeof(GLO1_py));
	memset(GLO1_pz, 0, sizeof(GLO1_pz));

	GLO1_i4page = GLO1_i5page = 0;
}