#include "../stdafx.h"
#include "def.h"
#include "queue.h"
#include "socket.h"

//选项配置矢量
extern int sel;

extern const SIGNAL_TYPE one;
extern const SIGNAL_TYPE zero;

//执行流
extern cudaStream_t cuda_stream[];

//循环队列
extern Queue q;

//socket通信相关变量
extern SOCKET socket_client;
extern sockaddr_in socket_sin;
extern int socket_port;
extern int self_port;
extern string socket_addr;
extern int socket_len;
extern struct tm local_time;

//定位返回结果
extern SimRes send_res_L1;
extern SimRes send_res_BD1;
extern SimRes send_res_BD3;
extern SimRes send_res_GLO1;

//信号文件
extern FILE *fp[];

extern cublasHandle_t blas_handle;

//Time Stamp
extern char nowtime[50];


//extern string filename;
//extern string filename;
extern string anti_filename;
//extern string filename;
extern string receiver_filepath;

/*输入的信号文件时长（单位：秒）*/
extern SIGNAL_TYPE data_time_length;

extern int sameple_rate;     //采样率Hz
extern int bit;    //量化位数
extern int3 QUAN_config;

//STAP抗干扰参数配置
extern int m;     //阵元数
extern int n;     //延迟单元数
//beam_form参数配置
extern SIGNAL_TYPE TF;	//射频Hz
extern SIGNAL_TYPE IF;	//中频Hz
extern SIGNAL_TYPE signal_lamda;//信号波长
extern SIGNAL_TYPE EL_si[];
extern SIGNAL_TYPE AZ_sj[];
extern char array_antenna;      //阵型选择
extern SIGNAL_TYPE array_posx[];//阵元坐标
extern SIGNAL_TYPE array_posy[];//阵元坐标
extern SIGNAL_TYPE array_posz[];//阵元坐标


//FDAJ抗干扰参数配置：
extern int antenna;     //阵元个数
extern int threshold;   //门限系数
extern int loop;        //迭代次数

extern int cuda_kernel_grid;
extern int cuda_kernel_block;

//与接收机接口
extern void GPSL1_sim_main(long num);
extern void GPSL1_sim_end();
extern void BD1_sim_main(long);
extern void BD1_sim_end();
extern void BD3_sim_main(long num);
extern void BD3_sim_end();
extern void GLO1_sim_main(long);
extern void GLO1_sim_end();

void set_file_pos(int start_sec, int length_sec);