#include "anti_receiver_data.h"

Receiver_Buf GPSL1_receiver_buf;
Receiver_Buf BD1_receiver_buf;
Receiver_Buf BD3_receiver_buf;
Receiver_Buf GLO1_receiver_buf;

long GPSL1_receiver_init_num;
long BD1_receiver_init_num;
long BD3_receiver_init_num;
long GLO1_receiver_init_num;


//struct Receiver_Buf_t
//{
//	int buf_size;
//	int buf_remain_len;
//
//	SIGNAL_TYPE_QUAN* buf_ptr;
//	SIGNAL_TYPE_QUAN* buf_current_ptr;
//
//};

void init_receiver_buf(Receiver_Buf &receiver_buf, long source_len, SIGNAL_TYPE_QUAN* source_ptr)
{
	receiver_buf.buf_size = source_len;
	receiver_buf.buf_remain_len = 0;

	receiver_buf.buf_ptr = receiver_buf.buf_current_ptr = source_ptr;
		
}

void set_receiver_buf(Receiver_Buf &receiver_buf)
{
	receiver_buf.buf_remain_len = receiver_buf.buf_size;
	receiver_buf.buf_current_ptr = receiver_buf.buf_ptr;

}