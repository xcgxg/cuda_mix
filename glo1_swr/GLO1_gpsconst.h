// Released on July 9, 2004

#ifndef GLO1_GPS_CONST_H
#define GLO1_GPS_CONST_H

#ifndef GLO1_M_PI
#define GLO1_M_PI (double) 3.1415926535897932384626
#endif

#define GLO1_twoPI 2*GLO1_M_PI

#define GLO1_pi (double)3.1415926535898E0
#define	GLO1_halfPi (double) GLO1_pi/2
#define GLO1_c (double)2.99792458e8
#define GLO1_omegae (double)7.2921151467E-5//地球的自转角速度
#define GLO1_a (double)6378137.0
#define GLO1_b (double)6356752.314 //  WGS-84 ellipsoid parameters
//#define lambda (double) 0.1872659176029963//0.1902936728 // L1 wavelength in meters  


#define GLO1_SPEEDOFLIGHT (double)299792458.0    

#define GLO1_CARRIER_RF (double)   1560e6		
#define GLO1_chmax (long)11

#define GLO1_CHN_ON 1
#define GLO1_CHN_OFF 0

#define GLO1_off 0
#define GLO1_acquisition 1
#define GLO1_confirm 2
#define GLO1_pull_in 3
#define GLO1_track 4
#define GLO1_cold_start 0
#define GLO1_warm_start 1
#define GLO1_hot_start 2
#define GLO1_tracking 3
#define GLO1_navigating 4

#define GLO1_DETECTED 1
#define GLO1_UNDETECTED 0
#define GLO1_DETECTING -1

#define GLO1_DLLdT 0.5			
#define GLO1_r_to_d (double)57.29577951308232

#define GLO1_CARRIER_TABLE_LENGTH 16

//#define GLO1_NBS1	25
//#define GLO1_NBS2	15
#define GLO1_NBS1	50  //todo
#define GLO1_NBS2	35

#define GLO1_BUFSIZE	1000

#define GLO1_MinSvNumber 1
//#define MaxSvNumber 32
#define GLO1_MaxSvNumber 24//todo

#define GLO1_USING 1			// 正被使用中
#define GLO1_HOLD 2			// 暂时不可使用
#define GLO1_AVAILABLE 3		// 可以使用

//#define DELTF	(double) 666.67			// 信号搜索的频率间隔
#define GLO1_DELTF	(double) 500
#define GLO1_MAXF	(double) 10000			// 信号搜索的最高频率
#define GLO1_MINF	(double) -10000			// 信号搜索的最低频率

//#define GLO1_PULL_IN_TIME(long) 1000		// FLL的收敛时间，单位：ms
#define GLO1_PULL_IN_TIME (long) 1000
#define GLO1_FINE_FREQ_RESOLUTION (long) 20	// 信号捕获后，在pull-in状态下用于频率

// 精细分辨的时间，单位：ms
#define GLO1_SUBFRAME_SYNCHRONIZED	1		// 用于子帧同步的标志
#define GLO1_SUBFRAME_SYNCHRONIZING	0

#define GLO1_ReAcqTime 1000000L
#define GLO1_ReAcqTime_hotstart 8000L

#endif