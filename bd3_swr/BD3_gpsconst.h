// Released on July 9, 2004

#ifndef BD3_GPS_CONST_H
#define BD3_GPS_CONST_H

#ifndef BD3_M_PI
#define BD3_M_PI (double) 3.1415926535897932384626
#endif

#define BD3_twoPI 2*BD3_M_PI

#define BD3_DtoR (double)1.74532925239284e-2//2010.11.17 add
#define BD3_pi (double)3.1415926535898E0
#define	BD3_HALFPI (double) BD3_pi/2
#define BD3_c (double)2.99792458e8
#define BD3_omegae (double)7.2921151467E-5//地球的自转角速度

#define BD3_a (double)6378137.0
#define BD3_b (double)6356752.314 //  WGS-84 ellipsoid parameters
//#define lambda (double) 0.1902936728 // L1 wavelength in meters
//#define lambda (double)299792458.0/1561.098e6  // B1 wavelength in meters
#define BD3_lambda (double)299792458.0/1268.52e6  //B3 wavelength in meters

#define BD3_SPEEDOFLIGHT (double)299792458.0    

//#define CARRIER_RF (double)   1561.098e6				// B1 RF前端的本振频率，signalTAp中是1560MHz
#define BD3_CARRIER_RF (double)   1268.52e6   // B3 RF前端的本振频率
#define BD3_chmax (long)11

#define BD3_CHN_ON 1
#define BD3_CHN_OFF 0

#define BD3_off 0
#define BD3_acquisition 1
#define BD3_confirm 2
#define BD3_pull_in 3
#define BD3_track 4
#define BD3_cold_start 0
#define BD3_warm_start 1
#define BD3_hot_start 2
#define BD3_tracking 3
#define BD3_navigating 4

#define BD3_DETECTED 1
#define BD3_UNDETECTED 0
#define BD3_DETECTING -1


#define BD3_DLLdT 0.5 //相关间距					

#define BD3_r_to_d (double)57.29577951308232


#define BD3_CARRIER_TABLE_LENGTH 16

#define BD3_NBS1	25
#define BD3_NBS2	15
//#define BUFSIZE	100
#define BD3_BUFSIZE	1000

#define BD3_MinSvNumber 1
#define BD3_MaxSvNumber 12

// 以下参数定义为卫星的使用状态
#define BD3_USING 1			// 正被使用中
#define BD3_HOLD 2			// 暂时不可使用
#define BD3_AVAILABLE 3		// 可以使用

#define BD3_DELTF	(double) 500			// 信号搜索的频率间隔
#define BD3_MAXF	(double) 10000			// 信号搜索的最高频率
#define BD3_MINF	(double) -10000			// 信号搜索的最低频率

#define BD3_PULL_IN_TIME (long) 1000		// FLL的收敛时间，单位：ms
#define BD3_FINE_FREQ_RESOLUTION (long) 20	// 信号捕获后，在pull-in状态下用于频率
										// 精细分辨的时间，单位：ms
#define BD3_SUBFRAME_SYNCHRONIZED	1		// 用于子帧同步的标志
#define BD3_SUBFRAME_SYNCHRONIZING	0

#define BD3_ReAcqTime 1000000L
#define BD3_ReAcqTime_hotstart 8000L
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
// NH码对齐用的FIFO数
#define BD3_FIFO_NUM 5         //2007.05.06

// NH码
#define BD3_NH 0xFB2B1         //2007.05.06

// NH码匹配标志
#define BD3_NH_MATCHED    1       //2007.05.06
#define BD3_NH_UN_MATCHED 0       //2007.05.06

//相关器门限
#define BD3_CR_MIN        10      //2007.05.06
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
#endif