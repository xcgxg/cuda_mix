// Released on July 9, 2004

#ifndef GPS_CONST_H
#define GPS_CONST_H

#ifndef M_PI
#define M_PI (double) 3.1415926535897932384626
#endif

#define twoPI 2*M_PI

#define pi (double)3.1415926535898E0
#define	halfPi (double) pi/2
#define c (double)2.99792458e8
#define omegae (double)7.2921151467E-5//地球的自转角速度
#define a (double)6378137.0
#define b (double)6356752.314 //  WGS-84 ellipsoid parameters
#define lambda (double) 0.1902936728 // L1 wavelength in meters


#define SPEEDOFLIGHT (double)299792458.0    

#define CARRIER_RF (double)   1560e6		
#define chmax (long)11

#define CHN_ON 1
#define CHN_OFF 0

#define off 0
#define acquisition 1
#define confirm 2
#define pull_in 3
#define track 4
#define cold_start 0
#define warm_start 1
#define hot_start 2
#define tracking 3
#define navigating 4

#define DETECTED 1
#define UNDETECTED 0
#define DETECTING -1

//#define DLLdT 0.5			
#define r_to_d (double)57.29577951308232

#define CARRIER_TABLE_LENGTH 16

#define NBS1	25
#define NBS2	15
#define BUFSIZE	1000

#define MinSvNumber 1
#define MaxSvNumber 32

#define USING 1			// 正被使用中
#define HOLD 2			// 暂时不可使用
#define AVAILABLE 3		// 可以使用

#define DELTF	(double) 666.67			// 信号搜索的频率间隔
#define MAXF	(double) 10000			// 信号搜索的最高频率
#define MINF	(double) -10000			// 信号搜索的最低频率

#define PULL_IN_TIME (long) 1000		// FLL的收敛时间，单位：ms
#define FINE_FREQ_RESOLUTION (long) 20	// 信号捕获后，在pull-in状态下用于频率
// 精细分辨的时间，单位：ms
#define SUBFRAME_SYNCHRONIZED	1		// 用于子帧同步的标志
#define SUBFRAME_SYNCHRONIZING	0

#define ReAcqTime 1000000L
#define ReAcqTime_hotstart 8000L

#endif