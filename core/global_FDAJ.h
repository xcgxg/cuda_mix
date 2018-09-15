
#include "global_var.h"
//#include "../gps_swr/Exports.h"
//#include "../bd_swr/BD1_Exports.h"
//#include "../gps_swr/gpsrcvr.h"
//#include "../bd_swr/BD1_gpsrcvr.h"
#include "../receiver/anti_receiver_data.h"

//定义变量、数组
extern SIGNAL_TYPE *FDAJ_signal[];	//定义数组长度，限制为2的N次方
extern SIGNAL_TYPE *FDAJ_dev_signal[];
extern SIGNAL_TYPE **FDAJ_dev_array_signal;
extern SIGNAL_TYPE *FDAJ_dev_s_amp;
extern SIGNAL_TYPE **FDAJ_dev_array_s_amp;
extern SIGNAL_TYPE *FDAJ_dev_TH_aver;
extern SIGNAL_TYPE_QUAN *FDAJ_s_out;
extern SIGNAL_TYPE *FDAJ_dev_s_out;
extern SIGNAL_TYPE_QUAN *FDAJ_dev_s_out_quan;
extern SIGNAL_TYPE **FDAJ_dev_array_s_out;
extern SIGNAL_TYPE_QUAN **FDAJ_dev_array_s_out_quan;
extern SIGNAL_TYPE *FDAJ_dev_s_out_max;
extern SIGNAL_TYPE *FDAJ_window;
extern SIGNAL_TYPE *FDAJ_dev_window;
extern COMPLEX_TYPE *FDAJ_dev_s_in;
extern COMPLEX_TYPE **FDAJ_dev_array_s_in;
extern cufftHandle FDAJ_plan_r2c;
extern cufftHandle FDAJ_plan_c2r;
extern cufftHandle FDAJ_plan_c2c;
extern FILE *FDAJ_out_file;
extern SIGNAL_TYPE FDAJ_pow_quantization;

void FDAJ_exec(int antenna, int threshold, int loop, int bit, int steplength, int each_number,
	SIGNAL_TYPE data_process_time, int sel);
void cuda_FDAJ(int m, int sn, int T, int K, int bit, int cycle_number);
void FDAJ_read_file(int sn, int times, int times_read);
void FDAJ_write_file(int total_point, int sel);

//sn:单个样本处理的个数
void init_FDAJ(int sn, int bit, int cycle_number, int sel);

void destroy_FDAJ(int sel);
//void FDAJ_hanning(SIGNAL_TYPE a[], int N);