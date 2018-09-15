#pragma once

#include "../core/def.h"

//struct Receiver_Buf_t
//{
//	int source_remain_len;
//	SIGNAL_TYPE_QUAN* source_ptr;
//	SIGNAL_TYPE_QUAN* source_current_ptr;
//
//	int source_len;
//	int buf_size;
//	
//	int buf_len;
//	SIGNAL_TYPE_QUAN* buf_ptr;
//};

struct Receiver_Buf_t
{
	long buf_size;
	long buf_remain_len;

	SIGNAL_TYPE_QUAN* buf_ptr;
	SIGNAL_TYPE_QUAN* buf_current_ptr;
	
};

typedef struct Receiver_Buf_t Receiver_Buf;

extern Receiver_Buf GPSL1_receiver_buf;
extern Receiver_Buf BD1_receiver_buf;
extern Receiver_Buf BD3_receiver_buf;
extern Receiver_Buf GLO1_receiver_buf;

extern long GPSL1_receiver_init_num;
extern long BD1_receiver_init_num;
extern long BD3_receiver_init_num;
extern long GLO1_receiver_init_num;

void init_receiver_buf(Receiver_Buf &receiver_buf, long source_len, SIGNAL_TYPE_QUAN* source_ptr);
void set_receiver_buf(Receiver_Buf &receiver_buf);