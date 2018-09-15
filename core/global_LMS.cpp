#include "global_LMS.h"

SIGNAL_TYPE LMS_pow_quantization;
SIGNAL_TYPE LMS_quan_max;
SIGNAL_TYPE LMS_quan_min;
SIGNAL_TYPE LMS_wi[NUM_SIGNAL_FILE_HANDLE] = {};
SIGNAL_TYPE LMS_u = 1e-12;

//处理的数据
SIGNAL_TYPE *LMS_signal[QUEUE_MAX_SIZE] = {};
SIGNAL_TYPE *LMS_anti_out = 0;
SIGNAL_TYPE_QUAN *LMS_anti_out_quan = 0;
FILE *LMS_out_file;


void init_LMS(int sn, int sel)
{
	char fn[1000];
	const char *filename = anti_filename.c_str();

	//打开文件
	int open_file_info;
	for (int i = 0; i<m; i++)
	{
		sprintf_s(fn, filename, (i + 1));	//字符串赋给filename
		open_file_info = fopen_s(&fp[i], fn, "rb");
		if (0 != open_file_info)
		{
			printf("error%d info\n", i, open_file_info);
			exit(-1);
		}
	}

	LMS_pow_quantization = pow(2.0, bit - 1) - 1;

	LMS_signal[0] = new SIGNAL_TYPE[QUEUE_MAX_SIZE * m * sn];
	for (int i = 0; i < QUEUE_MAX_SIZE; i++)
	{
		LMS_signal[i] = LMS_signal[0] + i * m * sn;
	}
	LMS_anti_out = new SIGNAL_TYPE[sn];
	LMS_anti_out_quan = new SIGNAL_TYPE_QUAN[sn];

	//打开输出文件
	int tmp = sel & WRITE_ANTI_RESULT_CONFIG_SEL;
	if (tmp)
	{
		switch (tmp)
		{
		case WRITE_ANTI_RESULT_TXT:
			fopen_s(&LMS_out_file, nowtime, "a+");
			break;
		case WRITE_ANTI_RESULT_BINARY_CHAR:
			fopen_s(&LMS_out_file, nowtime, "ab");
			break;
		default:
			break;
		}

		if (NULL == LMS_out_file)
		{
			printf("can't open LMS out file.\n");

			exit(-1);
		}
	}

	switch (sel & RECEIVER_CONFIG_SEL)
	{
	case WITH_GPSL1_RECEIVER:
		init_receiver_buf(GPSL1_receiver_buf, sn, LMS_anti_out_quan);
		break;
	case WITH_BD1_RECEIVER:
		init_receiver_buf(BD1_receiver_buf, sn, LMS_anti_out_quan);
		break;
	case WITH_BD3_RECEIVER:
		init_receiver_buf(BD3_receiver_buf, sn, LMS_anti_out_quan);
		break;
	case WITH_GLO1_RECEIVER:
		init_receiver_buf(GLO1_receiver_buf, sn, LMS_anti_out_quan);
		break;
	default:
		break;
	}

}

void LMS_exec(int m, int sn, int bit, SIGNAL_TYPE sameple_rate, int sel)
{
	int count_for = sameple_rate * data_time_length / sn;
	int write_file = WRITE_ANTI_RESULT_CONFIG_SEL & sel;
	int with_receiver = RECEIVER_CONFIG_SEL & sel;
	int total_point = sn;

	printf("start LMS\n");

	/*pipeline*/
	{

#pragma omp parallel num_threads(2)
		{
#pragma omp sections
			{
#pragma omp section
				{
					for (int i = 0; i < count_for; i++)
					{
						LMS_read_file(m, sn, i);
					}
				}
#pragma omp section
				{
					for (int i = 0; i < count_for; i++)
					{
						LMS(m, sn, bit);

						if (write_file)
						{
							LMS_write_file(total_point, write_file); //printf("123\n");
						}

						switch (with_receiver)
						{
							case WITH_GPSL1_RECEIVER:
								set_receiver_buf(GPSL1_receiver_buf);
								GPSL1_sim_main(GPSL1_receiver_init_num);
								break;
							case WITH_BD1_RECEIVER:
								set_receiver_buf(BD1_receiver_buf);
								BD1_sim_main(BD1_receiver_init_num);
								break;
							case WITH_BD3_RECEIVER:
								set_receiver_buf(BD3_receiver_buf);
								BD3_sim_main(BD3_receiver_init_num);
								break;
							case WITH_GLO1_RECEIVER:
								set_receiver_buf(GLO1_receiver_buf);
								GLO1_sim_main(GLO1_receiver_init_num);
								break;
							default:
								break;
						}


						//printf("%d\n", i);

						out_queue(q);

						//printf("%d error=%d\n", i, cudaGetLastError());
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
			}
		}
	}
	/*pipeline*/

	printf("STAP end\n");

}

void LMS(int m, int sn, int bit)
{
	while (isQueueEmpty(q));

	int current_LMS_signal_index = currentQueueFront(q);
	SIGNAL_TYPE *current_LMS_signal = LMS_signal[current_LMS_signal_index];

	int tmp0;
	SIGNAL_TYPE tmp1, tmp2, tmp3;

	LMS_quan_max = -FLT_MAX;
	LMS_quan_min = FLT_MAX;

	for (int i = 0; i < sn; i++)
	{
		tmp0 = i;
		tmp1 = 0;
		tmp3 = current_LMS_signal[i];

		for (int o = 0; o < m - 1; o++)
		{
			tmp0 += sn;
			tmp1 += (LMS_wi[o] * current_LMS_signal[tmp0]);
		}

		LMS_anti_out[i] = tmp2 = tmp3 - tmp1;

		LMS_quan_max = (tmp2 > LMS_quan_max) ? tmp2 : LMS_quan_max;
		LMS_quan_min = (tmp2 < LMS_quan_min) ? tmp2 : LMS_quan_min;

		tmp0 = i;

		for (int o = 0; o < m - 1; o++)
		{
			tmp0 += sn;
			LMS_wi[o] += (LMS_u * 2 * tmp2 * current_LMS_signal[tmp0]);
		}
	}

	LMS_quantization(LMS_anti_out, sn, bit);
}

void LMS_quantization(SIGNAL_TYPE *LMS_anti_out, int sn, int bit)
{
	LMS_quan_max = fabs(LMS_quan_max);
	LMS_quan_min = fabs(LMS_quan_min);
	LMS_quan_max = (LMS_quan_max > LMS_quan_min) ? LMS_quan_max : LMS_quan_min;

	SIGNAL_TYPE some_input;
	SIGNAL_TYPE_QUAN tmp;

#pragma omp parallel for private(some_input, tmp) num_threads(15)
	for (int i = 0; i < sn; i++)
	{
		some_input = LMS_anti_out[i];
		tmp = (SIGNAL_TYPE_QUAN)(QUAN_config.x * round(some_input * LMS_pow_quantization / LMS_quan_max));

		LMS_anti_out_quan[i] = (some_input >= 0) ? (tmp + QUAN_config.y) : (tmp + QUAN_config.z);
	}
}

void des_LMS(int sel)
{
	if (sel & WRITE_ANTI_RESULT_CONFIG_SEL)
	{
		fclose(LMS_out_file);
	}

	delete[] LMS_anti_out_quan;
	delete[] LMS_anti_out;
	delete[] LMS_signal[0];

	//关闭读取文件
	for (int i = 0; i < m; i++)
	{
		fclose(fp[i]);
	}

}

void LMS_read_file(int m, int sn, int times_read)
{
	//队列满了，就一直循环
	while (isQueueFull(q));

	int next_signal = nextQueueRear(q);

	SIGNAL_TYPE *current_signal = LMS_signal[next_signal];

	for (int i = 0; i < m; i++)
	{
		fread_s(current_signal, sn * sizeof(SIGNAL_TYPE), sizeof(SIGNAL_TYPE), sn, fp[i]);

		current_signal += sn;
	}

	in_queue(q);

}

void LMS_write_file(int total_point, int sel)
{
	switch (sel)
	{
	case WRITE_ANTI_RESULT_TXT:
		for (int i = 0; i < total_point; i++)
		{
			fprintf(LMS_out_file, "%d\n", LMS_anti_out_quan[i]);
		}
		break;

	case WRITE_ANTI_RESULT_BINARY_CHAR:
		fwrite(LMS_anti_out_quan, sizeof(SIGNAL_TYPE_QUAN), total_point, LMS_out_file);
		break;

	default:
		break;
	}
}