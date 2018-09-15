#pragma once

#define TEST_CHUNK 100

/*�ö���������Ҫ�޸�Ϊ��������ź��ļ�ʱ������λ���룩*/
//#define DATA_TIME_LENGTH 60

//���ݴ���λʱ��s
#define EACH_DATA_PROCESS_TIME 0.1

//���δ���ĵ���
#define STEPLENGTH_10MHZ 10000//32768
#define STEPLENGTH_20MHZ 20000
#define STEPLENGTH_46MHZ 46000

//�������ε���ʸ������
#define BEAM_SIG_VECTOR 12

//�Ƿ�д�ļ�
//#define WRITE_FILE 0

//ִ��������
#define NUM_STREAM 1

#define NUM_SIGNAL_FILE_HANDLE 10

/*�������ͼ�������*/
#define IS_SIGNAL_TYPE_DOUBLE 0

//��������ź�����
#define SIGNAL_TYPE_QUAN char

//���ջ��е����ݻ�������С
#define ANTI_RECEIVER_BUF_SIZE 62000000

#if 0 == IS_SIGNAL_TYPE_DOUBLE
	#define SIGNAL_TYPE float
	#define COMPLEX_TYPE cufftComplex
#elif 1 == IS_SIGNAL_TYPE_DOUBLE
	#define SIGNAL_TYPE double
	#define COMPLEX_TYPE cufftDoubleComplex
#endif

#if 0 == IS_SIGNAL_TYPE_DOUBLE
	#define cublasXscal cublasSscal
	#define cublasXgemm cublasSgemm
	#define cublasXasum cublasSasum
	#define cublasIXamax cublasIsamax
	#define cublasIXamin cublasIsamin
	#define cublasXdot cublasSdot
	#define cublasXgetrfBatched cublasSgetrfBatched
	#define cublasXgetriBatched cublasSgetriBatched
	#define cufftExecRX2CX cufftExecR2C
	#define cufftExecCX2RX cufftExecC2R
	#define roundX roundf
	#define fabsX fabsf
	#define cosX cosf
	#define sqrtX sqrtf
	#define floorX floorf
#elif 1 == IS_SIGNAL_TYPE_DOUBLE
	#define cublasXscal cublasDscal
	#define cublasXgemm cublasDgemm
	#define cublasXasum cublasDasum
	#define cublasIXamax cublasIdamax
	#define cublasIXamin cublasIdamin
	#define cublasXdot cublasDdot
	#define cublasXgetrfBatched cublasDgetrfBatched
	#define cublasXgetriBatched cublasDgetriBatched
	#define cufftExecRX2CX cufftExecD2Z
	#define cufftExecCX2RX cufftExecZ2D
	#define roundX round
	#define fabsX fabs
	#define cosX cos
	#define sqrtX sqrt
	#define floorX floor
#endif

#define SIGNAL_DATA_TYPE SIGNAL_TYPE
/*�������ͼ�������*/


#define CONFIG_ANTI_NONE 0X0
#define CONFIG_STAP 0X1
#define CONFIG_BEAM 0X2
#define CONFIG_FDAJ 0X4
#define CONFIG_LMS 0X8
#define ANTI_CONFIG_SEL 0XF
/*�����š����ջ��Ϳ�����д�ļ�����ʸ��
��λʮ���������������λ�����λ����һλ���������㷨��ѡ�񣬵ڶ�λ����ͬ���͵Ľ��ջ�������λ�����Ƿ񽫿����Ž��д���ļ�����Ϊд���ı��ļ����������ļ���
����
�������㷨ΪFDAJ��BD1���ջ����������Ž��д���ļ���������ʸ��Ϊ0X421
*/
#define WITH_GPSL1_RECEIVER 0X10
#define WITH_BD1_RECEIVER 0X20
#define WITH_BD3_RECEIVER 0X40
#define WITH_GLO1_RECEIVER 0X80
#define RECEIVER_CONFIG_SEL 0XF0

#define WRITE_ANTI_RESULT_TXT 0X100
#define WRITE_ANTI_RESULT_BINARY_CHAR 0X200
#define WRITE_ANTI_RESULT_CONFIG_SEL 0XF00

/*����,��λ����ѡ��*/
#define USING_POS_VEL_TIME 0X0000
#define USING_KALMAN_POS_VEL 0X1000
#define USING_POS_METHOD_SEL 0XF000

/*�������ŵĽ��ջ��Ƿ�ʹ�����紫�䶨λ����*/
#define USING_LOCAL_DATA 0X00000
#define USING_SOCKET_DATA 0X10000
#define USING_DATA_CONFIG 0XF0000



#define PI 3.1415926535897932384626433832795028841971
#define lv 2.99792458e8 //����


//�����㷨�Ĳ�������

#define QUAN_1_BIT_0 1
#define QUAN_1_BIT_1 0
#define QUAN_1_BIT_2 1

#define QUAN_2_BIT_0 2
#define QUAN_2_BIT_1 1
#define QUAN_2_BIT_2 -1

#define QUAN_4_BIT_0 1
#define QUAN_4_BIT_1 0
#define QUAN_4_BIT_2 -1

/*���ջ���ʶ*/
#define RECEIVER_L1 0
#define RECEIVER_BD1 1
#define RECEIVER_BD3 2
#define RECEIVER_GLO1 3

#define CUDA_KERNEL_GRID_NUM_10MHZ 40
#define CUDA_KERNEL_BLOCK_NUM_10MHZ 256
#define CUDA_KERNEL_GRID_NUM_20MHZ 80
#define CUDA_KERNEL_BLOCK_NUM_20MHZ 256
#define CUDA_KERNEL_GRID_NUM_46MHZ 180
#define CUDA_KERNEL_BLOCK_NUM_46MHZ 256
