#include "global_var.h"
#include "../receiver/anti_receiver_data.h"

//#define ANTI_RECEIVER_DATA_QUEUE_MAX_SIZE 100

//处理的数据
extern SIGNAL_TYPE_QUAN *anti_none_signal[QUEUE_MAX_SIZE];
extern SIGNAL_TYPE_QUAN *anti_none_anti_out_quan;
extern int socket_request;

void init_anti_none(int sn, int sel);
void destroy_anti_none();
void anti_none_read_file(int sn, int times_read);
void anti_none_exec(int sn, SIGNAL_TYPE data_process_time, int sel);