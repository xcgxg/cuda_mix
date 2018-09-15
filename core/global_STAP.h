
#include "global_var.h"
//#include "../gps_swr/Exports.h"
//#include "../bd_swr/BD1_Exports.h"
//#include "../gps_swr/gpsrcvr.h"
//#include "../bd_swr/BD1_gpsrcvr.h"
#include "../receiver/anti_receiver_data.h"

extern int STAP_row_sig_STAP;
extern int STAP_col_sig_STAP;
extern SIGNAL_TYPE *STAP_signal[];
extern SIGNAL_TYPE *STAP_dev_signal[];
extern SIGNAL_TYPE_QUAN *STAP_anti_out_quan;
extern SIGNAL_TYPE *STAP_dev_anti_out;
extern SIGNAL_TYPE_QUAN *STAP_dev_anti_out_quan;
extern SIGNAL_TYPE **STAP_dev_array_anti_out;
extern SIGNAL_TYPE_QUAN **STAP_dev_array_anti_out_quan;
extern SIGNAL_TYPE *STAP_dev_anti_out_max;
extern SIGNAL_TYPE *STAP_signal_STAP[];
//extern SIGNAL_TYPE *STAP_array_signal_STAP[][TEST_CHUNK];
extern SIGNAL_TYPE **STAP_dev_array_signal_STAP;

extern int STAP_lead_dimension_matrix_R;
extern int STAP_lead_dimension_matrix_A;
extern int STAP_size_matrix_R;
extern SIGNAL_TYPE *STAP_array_matrix_R[];
extern SIGNAL_TYPE **STAP_dev_array_matrix_R;
extern SIGNAL_TYPE *STAP_array_matrix_R_inver[];
extern SIGNAL_TYPE **STAP_dev_array_matrix_R_inver;
extern SIGNAL_TYPE **STAP_dev_array_matrix_R_inver_1col;
extern int *STAP_PivotArray;
extern int *STAP_infoArray;

extern FILE *STAP_out_file;

//量化参数
extern SIGNAL_TYPE STAP_pow_quantization;

//波束成形
extern SIGNAL_TYPE *beam_vector_real;
extern SIGNAL_TYPE *dev_beam_vector_real;
extern SIGNAL_TYPE *beam_vector_image;
extern SIGNAL_TYPE *dev_beam_vector_image;
extern SIGNAL_TYPE *dev_R_inv_v_real[];
extern SIGNAL_TYPE **dev_array_R_inv_v_real;
extern SIGNAL_TYPE *dev_R_inv_v_image[];
extern SIGNAL_TYPE **dev_array_R_inv_v_image;
extern SIGNAL_TYPE *dev_v_real_R_inv[];
extern SIGNAL_TYPE **dev_array_v_real_R_inv;
extern SIGNAL_TYPE *dev_v_image_R_inv[];
extern SIGNAL_TYPE **dev_array_v_image_R_inv;
extern SIGNAL_TYPE *dev_beam_scal_real;
extern SIGNAL_TYPE *dev_beam_scal_image;


//初始化全局变量， 一次读入0.01s的所有数据， fp:文件指针数组， m:阵元数, sn:单次处理数据的点数, s:需要循环处理的次数
//n是延迟单元数，bit是量化位数,sel==0为STAP，sel==1为波束成形
void init_STAP(int m, int sn, int s, int n, int bit, int sel);

//析构全局变量
void destroy_STAP(int m, int sel);

//读信号文件
void STAP_read_file(int m, int sn, int s, int n, int times_read);

//写文件
void STAP_write_file(int total_point, int sel);

/* 输入信号转换成空时矩阵
输入参数：
signal:输入的M信号；
m:输入信号的行数
n:输入信号的列数
s:循环次数
CN:抽头数=延迟单元数+1；
delay:各个抽头之间的延迟
*/
void Mat_chg(int m, int n, int s, int CN, int delay, int next_signal, int times_chg);

void STAP_exec(int m, int n, int sn, int bit, int s, int sameple_rate, SIGNAL_TYPE data_process_time, int sel);

void cuda_STAP(int m, int sn, int s, int n, int sel);

//波束形成导向矢量计算
//EL_si:信号的俯仰角            
//AZ_sj:信号的方位角
void beam_s_vector(SIGNAL_TYPE *EL_si, SIGNAL_TYPE *AZ_sj, SIGNAL_TYPE d, int m, int n, char array_s,
	SIGNAL_TYPE sameple, SIGNAL_TYPE IF);