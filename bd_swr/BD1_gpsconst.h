// Released on July 9, 2004

#ifndef BD1_GPS_CONST_H
#define BD1_GPS_CONST_H

#ifndef BD1_M_PI
#define BD1_M_PI (double) 3.1415926535897932384626
#endif

#define BD1_twoPI 2*BD1_M_PI

#define BD1_DtoR (double)1.74532925239284e-2//2010.11.17 add
#define BD1_pi (double)3.1415926535898E0
#define	BD1_HALFPI (double) BD1_pi/2
#define BD1_c (double)2.99792458e8
#define BD1_omegae (double)7.2921151467E-5//�������ת���ٶ�

#define BD1_a (double)6378137.0
#define BD1_b (double)6356752.314 //  WGS-84 ellipsoid parameters
//#define lambda (double) 0.1902936728 // L1 wavelength in meters
#define BD1_lambda (double)299792458.0/1561.098e6  // B1 wavelength in meters


#define BD1_SPEEDOFLIGHT (double)299792458.0    

#define BD1_CARRIER_RF (double)   1561.098e6				// RFǰ�˵ı���Ƶ�ʣ�signalTAp����1560MHz

#define BD1_chmax (long)11

#define BD1_CHN_ON 1
#define BD1_CHN_OFF 0

#define BD1_off 0
#define BD1_acquisition 1
#define BD1_confirm 2
#define BD1_pull_in 3
#define BD1_track 4
#define BD1_cold_start 0
#define BD1_warm_start 1
#define BD1_hot_start 2
#define BD1_tracking 3
#define BD1_navigating 4

#define BD1_DETECTED 1
#define BD1_UNDETECTED 0
#define BD1_DETECTING -1


#define BD1_DLLdT 0.5 //��ؼ��					

#define BD1_r_to_d (double)57.29577951308232


#define BD1_CARRIER_TABLE_LENGTH 16

#define BD1_NBS1	25
#define BD1_NBS2	15
#define BD1_BUFSIZE	100

#define BD1_MinSvNumber 1
#define BD1_MaxSvNumber 12

// ���²�������Ϊ���ǵ�ʹ��״̬
#define BD1_USING 1			// ����ʹ����
#define BD1_HOLD 2			// ��ʱ����ʹ��
#define BD1_AVAILABLE 3		// ����ʹ��

#define BD1_DELTF	(double) 500			// �ź�������Ƶ�ʼ��
#define BD1_MAXF	(double) 10000			// �ź����������Ƶ��
#define BD1_MINF	(double) -10000			// �ź����������Ƶ��

#define BD1_PULL_IN_TIME (long) 1000		// FLL������ʱ�䣬��λ��ms
#define BD1_FINE_FREQ_RESOLUTION (long) 20	// �źŲ������pull-in״̬������Ƶ��
// ��ϸ�ֱ��ʱ�䣬��λ��ms
#define BD1_SUBFRAME_SYNCHRONIZED	1		// ������֡ͬ���ı�־
#define BD1_SUBFRAME_SYNCHRONIZING	0

#define BD1_ReAcqTime 1000000L
#define BD1_ReAcqTime_hotstart 8000L
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
// NH������õ�FIFO��
#define BD1_FIFO_NUM 5         //2007.05.06

// NH��
#define BD1_NH 0xFB2B1         //2007.05.06

// NH��ƥ���־
#define BD1_NH_MATCHED    1       //2007.05.06
#define BD1_NH_UN_MATCHED 0       //2007.05.06

//���������
#define BD1_CR_MIN        10      //2007.05.06
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
#endif