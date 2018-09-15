#include "global_var.h"
#include "../receiver/anti_receiver_data.h"

extern SIGNAL_TYPE LMS_pow_quantization;
extern SIGNAL_TYPE LMS_quan_max;
extern SIGNAL_TYPE LMS_quan_min;
extern SIGNAL_TYPE LMS_wi[NUM_SIGNAL_FILE_HANDLE];
extern SIGNAL_TYPE LMS_u;

//处理的数据
extern SIGNAL_TYPE *LMS_signal[QUEUE_MAX_SIZE];
extern SIGNAL_TYPE *LMS_anti_out;
extern SIGNAL_TYPE_QUAN *LMS_anti_out_quan;
extern FILE *LMS_out_file;

void init_LMS(int sn, int sel);
void LMS_exec(int m, int sn, int bit, SIGNAL_TYPE sameple_rate, int sel);
void LMS(int m, int sn, int bit);
void LMS_quantization(SIGNAL_TYPE *LMS_anti_out, int sn, int bit);
void des_LMS(int sel);
void LMS_read_file(int m, int sn, int times_read);
void LMS_write_file(int total_point, int sel);