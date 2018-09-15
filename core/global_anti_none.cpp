#include "global_anti_none.h"

//处理的数据
SIGNAL_TYPE_QUAN *anti_none_signal[QUEUE_MAX_SIZE] = {};
SIGNAL_TYPE_QUAN *anti_none_anti_out_quan = 0;
int socket_request = 1000;

void init_anti_none(int sn, int sel)
{
	int open_file_info;
	switch (sel & USING_DATA_CONFIG)
	{
	case USING_LOCAL_DATA:
		//打开文件
		open_file_info = fopen_s(&fp[0], anti_filename.c_str(), "rb");
		if (0 != open_file_info)
		{
			printf("error info %d\n", open_file_info);
			exit(-1);
		}
		break;

	case USING_SOCKET_DATA:
		//nothing to do
		break;

	}

	anti_none_signal[0] = new SIGNAL_TYPE_QUAN[sn * QUEUE_MAX_SIZE];
	for (int i = 0; i < QUEUE_MAX_SIZE; i++)
	{
		anti_none_signal[i] = anti_none_signal[0] + i * sn;
	}

	switch (sel & RECEIVER_CONFIG_SEL)
	{
	case WITH_GPSL1_RECEIVER:
		init_receiver_buf(GPSL1_receiver_buf, sn, anti_none_anti_out_quan);
		break;
	case WITH_BD1_RECEIVER:
		init_receiver_buf(BD1_receiver_buf, sn, anti_none_anti_out_quan);
		break;
	case WITH_BD3_RECEIVER:
		init_receiver_buf(BD3_receiver_buf, sn, anti_none_anti_out_quan);
		break;
	case WITH_GLO1_RECEIVER:
		init_receiver_buf(GLO1_receiver_buf, sn, anti_none_anti_out_quan);
		break;
	default:
		break;
	}
}

void destroy_anti_none()
{
	delete[] anti_none_signal[0];

	switch (sel & USING_DATA_CONFIG)
	{
	case USING_LOCAL_DATA:
		fclose(fp[0]);
		break;

	case USING_SOCKET_DATA:
		//nothing to do
		break;

	}
}

void anti_none_read_file(int sn, int times_read)
{
	//队列满了，就一直循环
	while (isQueueFull(q));

	int next_signal = nextQueueRear(q);;
	SIGNAL_TYPE_QUAN *current_signal = anti_none_signal[next_signal];
	int socket_ret;

	switch (sel & USING_DATA_CONFIG)
	{
	case USING_LOCAL_DATA:
		fread_s(current_signal, sn * sizeof(SIGNAL_TYPE_QUAN), sizeof(SIGNAL_TYPE_QUAN), sn, fp[0]);
		//printf("123\n");

		break;
	case USING_SOCKET_DATA:
		for (int i = 0; i < 1000; i++)
		{
			sendto(socket_client, (char *)&socket_request, sizeof(socket_request), 0, (sockaddr *)&socket_sin, socket_len);
			socket_ret = recvfrom(socket_client, current_signal + i * 1000, 1000, 0, (sockaddr *)&socket_sin, &socket_len);

			/*if (socket_ret <= 0)
			{
			printf("123\n");
			}*/
		}

		break;
	}

	in_queue(q);
}

void anti_none_exec(int sn, SIGNAL_TYPE data_process_time, int sel)
{
	int count_for = data_time_length / data_process_time;
	int with_receiver = RECEIVER_CONFIG_SEL & sel;

	printf("waiting...\n");

#pragma omp parallel num_threads(2)
	{
#pragma omp sections
		{
#pragma omp section
			{
				for (int i = 0; i < count_for; i++)
				{
					anti_none_read_file(sn, i);
				}
			}
#pragma omp section
			{
			for (int i = 0; i < count_for; i++)
			{
				while (isQueueEmpty(q));

				int tmp = currentQueueFront(q);
				anti_none_anti_out_quan = anti_none_signal[tmp];

				switch (with_receiver)
				{
				case WITH_GPSL1_RECEIVER:
					GPSL1_receiver_buf.buf_ptr = anti_none_anti_out_quan;
					set_receiver_buf(GPSL1_receiver_buf);
					GPSL1_sim_main(GPSL1_receiver_init_num);
					break;
				case WITH_BD1_RECEIVER:
					BD1_receiver_buf.buf_ptr = anti_none_anti_out_quan;
					set_receiver_buf(BD1_receiver_buf);
					BD1_sim_main(BD1_receiver_init_num);
					break;
				case WITH_BD3_RECEIVER:
					BD3_receiver_buf.buf_ptr = anti_none_anti_out_quan;
					set_receiver_buf(BD3_receiver_buf);
					BD3_sim_main(BD3_receiver_init_num);
					break;
				case WITH_GLO1_RECEIVER:
					GLO1_receiver_buf.buf_ptr = anti_none_anti_out_quan;
					set_receiver_buf(GLO1_receiver_buf);
					GLO1_sim_main(GLO1_receiver_init_num);
					break;
				default:
					break;
				}

				/*socket_feedback_end = 1;
				sendto(socket_client, (char *)&socket_feedback_end, sizeof(socket_feedback), 0, (sockaddr *)&socket_sin, socket_len);*/

				out_queue(q);
			}
		}
		}
	}

	//接收机结束
	switch (with_receiver)
	{
	case WITH_GPSL1_RECEIVER:
		GPSL1_sim_end();
		break;
	case WITH_BD1_RECEIVER:
		BD1_sim_end();
		break;
	case WITH_BD3_RECEIVER:
		BD3_sim_end();
		break;
	case WITH_GLO1_RECEIVER:
		GLO1_sim_end();
		break;
	default:
		break;
	}
}
