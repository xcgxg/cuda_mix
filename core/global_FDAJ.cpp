#include "global_FDAJ.h"

extern void cuda_vector_mul(SIGNAL_TYPE **dev_input1, SIGNAL_TYPE *dev_input2, SIGNAL_TYPE **dev_result,
	int n, int times);

extern void cuda_TH_filter(int K, COMPLEX_TYPE **dev_array_s_in, SIGNAL_TYPE **dev_array_s_amp,
	SIGNAL_TYPE *dev_TH_aver, int T, int sn, int cycle_number);

extern void cuda_vector_quantization(SIGNAL_TYPE **dev_input, SIGNAL_TYPE_QUAN **dev_output, 
	SIGNAL_TYPE pow_quantization, int n, int times);

extern void cuda_hanning(SIGNAL_TYPE *dev_window, int sn);


//定义变量、数组
SIGNAL_TYPE *FDAJ_signal[QUEUE_MAX_SIZE];	//定义数组长度，限制为2的N次方
SIGNAL_TYPE *FDAJ_dev_signal[QUEUE_MAX_SIZE];
SIGNAL_TYPE **FDAJ_dev_array_signal = 0;
SIGNAL_TYPE *FDAJ_dev_s_amp = 0;
SIGNAL_TYPE **FDAJ_dev_array_s_amp = 0;
SIGNAL_TYPE *FDAJ_dev_TH_aver = 0;	//最终的TH
SIGNAL_TYPE_QUAN *FDAJ_s_out = 0;
SIGNAL_TYPE *FDAJ_dev_s_out = 0;
SIGNAL_TYPE_QUAN *FDAJ_dev_s_out_quan;
SIGNAL_TYPE **FDAJ_dev_array_s_out = 0;
SIGNAL_TYPE_QUAN **FDAJ_dev_array_s_out_quan;
SIGNAL_TYPE *FDAJ_dev_s_out_max = 0;
SIGNAL_TYPE *FDAJ_window = 0;
SIGNAL_TYPE *FDAJ_dev_window = 0;
COMPLEX_TYPE *FDAJ_dev_s_in = 0;
COMPLEX_TYPE **FDAJ_dev_array_s_in = 0;
cufftHandle FDAJ_plan_r2c;
cufftHandle FDAJ_plan_c2r;
cufftHandle FDAJ_plan_c2c;
FILE *FDAJ_out_file;
SIGNAL_TYPE FDAJ_pow_quantization = 0;

void FDAJ_read_file(int sn, int times, int times_read)
{
	while (isQueueFull(q));
	
	int index_next_signal = nextQueueRear(q);

	int count = times * sn;
	//读取文件
	fread_s(FDAJ_signal[index_next_signal], count * sizeof(SIGNAL_TYPE), sizeof(SIGNAL_TYPE), count, fp[0]);

	cudaMemcpyAsync(FDAJ_dev_signal[index_next_signal], FDAJ_signal[index_next_signal], count * sizeof(SIGNAL_TYPE),
		cudaMemcpyHostToDevice, cuda_stream[0]);

	cudaStreamSynchronize(cuda_stream[0]);

	in_queue(q);

	/*SIGNAL_TYPE temp[10];
	cudaMemcpy(temp, dev_signal + sn-10, sizeof(SIGNAL_TYPE)* 10, cudaMemcpyDeviceToHost);
	for (int i = 0; i < 10; i++)
	{
	printf("%f ", temp[i]);
	}printf("\n");*/
}

void FDAJ_exec(int antenna, int threshold, int loop, int bit, int steplength, int each_number, 
	SIGNAL_TYPE data_process_time, int sel)
{
	int count_for = data_time_length / data_process_time;
	int with_receiver = RECEIVER_CONFIG_SEL & sel;
	int write_file = WRITE_ANTI_RESULT_CONFIG_SEL & sel;
	int total_point = each_number*steplength;

	printf("start FDAJ\n");
	//printf("start FDAJ %d\n", count_for);

	{
		
#pragma omp parallel num_threads(2)
		{
#pragma omp sections
			{
#pragma omp section
				{
					for (int i = 0; i < count_for; i++)
					{
						FDAJ_read_file(steplength, each_number, i);
					}
				}
#pragma omp section
				{
					for (int i = 0; i < count_for; i++)
					{
						cuda_FDAJ(antenna, steplength, threshold, loop, bit, each_number);

						if (write_file)
						{
							FDAJ_write_file(total_point, write_file);
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
						
						//printf("%d error=%d\n", i, cudaGetLastError());
						//printf("%d\n", i);

						out_queue(q);
						
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

		/*no mp*/
		//for (int i = 0; i < count_for; i++)
		//{
		//	read_FDAJ_file(steplength, each_number, i);
		//	FDAJ(antenna, steplength, threshold, loop, bit, each_number);
		//	/*cudaDeviceSynchronize();
		//	write_file(each_number, steplength);*/
		//}
		/*no mp*/
	}

	cudaDeviceSynchronize();

	printf("FDAJ end\n");

	/*printf("cudaGetLastError = %d\n", cudaGetLastError());
	printf("GetLastError = %d\n", GetLastError());*/

}

void cuda_FDAJ(int m, int sn, int T, int K, int bit, int cycle_number)
{
	while (isQueueEmpty(q));

	int index_current_signal = currentQueueFront(q);	

	//乘上窗函数
	cuda_vector_mul(FDAJ_dev_array_signal + index_current_signal * cycle_number, FDAJ_dev_window, FDAJ_dev_array_signal +
		index_current_signal * cycle_number, sn, cycle_number);

	//阈值滤波算法
	cufftExecRX2CX(FDAJ_plan_r2c, FDAJ_dev_signal[index_current_signal], FDAJ_dev_s_in);

	/*new TH filter for*/
	cuda_TH_filter(K, FDAJ_dev_array_s_in, FDAJ_dev_array_s_amp, FDAJ_dev_TH_aver, T, sn, cycle_number);
	/*new TH filter for*/

	//cuda逆FFT
	cufftExecCX2RX(FDAJ_plan_c2r, FDAJ_dev_s_in, FDAJ_dev_s_out);

	//cuda量化
	cuda_vector_quantization(FDAJ_dev_array_s_out, FDAJ_dev_array_s_out_quan, FDAJ_pow_quantization, sn, cycle_number);

	cudaMemcpy(FDAJ_s_out, FDAJ_dev_s_out_quan, cycle_number * sn * sizeof(SIGNAL_TYPE_QUAN), cudaMemcpyDeviceToHost);

	/*SIGNAL_TYPE temp[4];
	cudaMemcpy(temp, dev_s_out, sizeof(SIGNAL_TYPE)* 4, cudaMemcpyDeviceToHost);
	for (int i = 0; i < 4; i++)
	{
	printf("%f ", temp[i]);
	}printf("\n");*/

}

void init_FDAJ(int sn, int bit, int cycle_number, int sel)
{
	fp[0] = fopen(anti_filename.c_str(), "rb");

	cudaHostAlloc((void**)&FDAJ_signal[0], QUEUE_MAX_SIZE * cycle_number * sn * sizeof(SIGNAL_TYPE), cudaHostAllocDefault);
	cudaMalloc((void**)&FDAJ_dev_signal[0], QUEUE_MAX_SIZE * cycle_number * sn * sizeof(SIGNAL_TYPE));
	cudaMalloc((void**)&FDAJ_dev_array_signal, QUEUE_MAX_SIZE * cycle_number * sizeof(SIGNAL_TYPE *));
	{
		SIGNAL_TYPE **tmp_dev_array_signal = new SIGNAL_TYPE*[QUEUE_MAX_SIZE * cycle_number];
		for (int i = 0; i < QUEUE_MAX_SIZE; i++)
		{
			FDAJ_signal[i] = FDAJ_signal[0] + i * cycle_number * sn;
			FDAJ_dev_signal[i] = FDAJ_dev_signal[0] + i * cycle_number * sn;

			for (int o = 0; o < cycle_number; o++)
			{
				tmp_dev_array_signal[i * cycle_number + o] = FDAJ_dev_signal[i] + o * sn;
			}
		}

		cudaMemcpy(FDAJ_dev_array_signal, tmp_dev_array_signal, QUEUE_MAX_SIZE * cycle_number * sizeof(SIGNAL_TYPE *),
			cudaMemcpyHostToDevice);
		delete[] tmp_dev_array_signal;
	}

	//加窗
	cudaMalloc((void**)&FDAJ_dev_window, sn * sizeof(SIGNAL_TYPE));
	cuda_hanning(FDAJ_dev_window, sn);

	//FFT CUFFT_R2C 结果只保留非冗余的复数系数，即n/2+1个复数
	cudaMalloc((void**)&FDAJ_dev_s_in, cycle_number * (sn / 2 + 1)* sizeof(COMPLEX_TYPE));
	cudaMalloc((void**)&FDAJ_dev_array_s_in, cycle_number * sizeof(COMPLEX_TYPE *));
	{
		COMPLEX_TYPE **tmp_dev_array_s_in = new COMPLEX_TYPE*[cycle_number];
		for (int i = 0; i < cycle_number; i++)
		{
			tmp_dev_array_s_in[i] = FDAJ_dev_s_in + i * (sn / 2 + 1);
		}
		cudaMemcpy(FDAJ_dev_array_s_in, tmp_dev_array_s_in, cycle_number * sizeof(COMPLEX_TYPE *),
			cudaMemcpyHostToDevice);

		delete[] tmp_dev_array_s_in;
	}

	//抗干扰结果为量化后
	cudaHostAlloc((void**)&FDAJ_s_out, cycle_number * sn * sizeof(SIGNAL_TYPE_QUAN), cudaHostAllocDefault);

	//抗干扰过程按照浮点类型计算
	cudaMalloc((void**)&FDAJ_dev_s_out, cycle_number * sn * sizeof(SIGNAL_TYPE));
	cudaMalloc((void**)&FDAJ_dev_array_s_out, cycle_number * sizeof(SIGNAL_TYPE *));

	//抗干扰结果按8字节输出
	cudaMalloc((void**)&(FDAJ_dev_s_out_quan), cycle_number * sn * sizeof(SIGNAL_TYPE_QUAN));
	cudaMalloc((void**)&(FDAJ_dev_array_s_out_quan), cycle_number * sizeof(SIGNAL_TYPE_QUAN *));
	{
		SIGNAL_TYPE **tmp_dev_array_s_out = new SIGNAL_TYPE*[cycle_number];
		SIGNAL_TYPE_QUAN **tmp_dev_array_s_out_quan = new SIGNAL_TYPE_QUAN*[cycle_number];
		for (int i = 0; i < cycle_number; i++)
		{
			tmp_dev_array_s_out[i] = FDAJ_dev_s_out + i * sn;
			tmp_dev_array_s_out_quan[i] = FDAJ_dev_s_out_quan + i * sn;
		}
		cudaMemcpy(FDAJ_dev_array_s_out, tmp_dev_array_s_out, cycle_number * sizeof(SIGNAL_TYPE *),
			cudaMemcpyHostToDevice);
		cudaMemcpy(FDAJ_dev_array_s_out_quan, tmp_dev_array_s_out_quan, cycle_number * sizeof(SIGNAL_TYPE_QUAN *),
			cudaMemcpyHostToDevice);

		delete[] tmp_dev_array_s_out_quan;
		delete[] tmp_dev_array_s_out;
	}
	cudaMalloc((void**)&FDAJ_dev_s_out_max, cycle_number * sizeof(SIGNAL_TYPE));

	cufftPlan1d(&FDAJ_plan_r2c, sn, CUFFT_R2C, cycle_number);
	cufftPlan1d(&FDAJ_plan_c2r, sn, CUFFT_C2R, cycle_number);
	cufftPlan1d(&FDAJ_plan_c2c, sn, CUFFT_C2C, cycle_number);

	FDAJ_pow_quantization = pow(2.0, bit-1) - 1;

	//复数的绝对值
	cudaMalloc((void**)&FDAJ_dev_s_amp, cycle_number * (sn / 2 + 1) * sizeof(SIGNAL_TYPE));
	cudaMalloc((void**)&FDAJ_dev_array_s_amp, cycle_number * sizeof(SIGNAL_TYPE *));
	{
		SIGNAL_TYPE **tmp_dev_array_s_amp = new SIGNAL_TYPE*[cycle_number];
		for (int i = 0; i < cycle_number; i++)
		{
			tmp_dev_array_s_amp[i] = FDAJ_dev_s_amp + i * (sn / 2 + 1);
		}
		cudaMemcpy(FDAJ_dev_array_s_amp, tmp_dev_array_s_amp, cycle_number * sizeof(SIGNAL_TYPE *),
			cudaMemcpyHostToDevice);
		delete[] tmp_dev_array_s_amp;
	}

	cudaMalloc((void**)&FDAJ_dev_TH_aver, cycle_number * sizeof(SIGNAL_TYPE));

	//打开输出文件
	if (sel & WRITE_ANTI_RESULT_CONFIG_SEL)
	{
		int tmp = sel & WRITE_ANTI_RESULT_CONFIG_SEL;

		switch (tmp)
		{
		case WRITE_ANTI_RESULT_TXT:
			FDAJ_out_file = fopen(nowtime, "a+");
			break;
		case WRITE_ANTI_RESULT_BINARY_CHAR:
			FDAJ_out_file = fopen(nowtime, "ab");
			break;
		default:
			break;
		}

		if (NULL == FDAJ_out_file)
		{
			printf("open FDAJ out file error\n");

			exit(-1);
		}
	}
	
	switch (sel & RECEIVER_CONFIG_SEL)
	{
	case WITH_GPSL1_RECEIVER:
		init_receiver_buf(GPSL1_receiver_buf, cycle_number * sn, FDAJ_s_out);
		//init_receiver_buf(GPSL1_receiver_buf, 1000000, FDAJ_s_out);
		break;
	case WITH_BD1_RECEIVER:
		init_receiver_buf(BD1_receiver_buf, cycle_number * sn, FDAJ_s_out);
		break;
	case WITH_BD3_RECEIVER:
		init_receiver_buf(BD3_receiver_buf, cycle_number * sn, FDAJ_s_out);
		break;
	case WITH_GLO1_RECEIVER:
		init_receiver_buf(GLO1_receiver_buf, cycle_number * sn, FDAJ_s_out);
		break;
	default:
		break;
	}

}

void destroy_FDAJ(int sel)
{
	if (sel & WRITE_ANTI_RESULT_CONFIG_SEL)
	{
		fclose(FDAJ_out_file);
	}
	
	cudaFreeHost(FDAJ_s_out);
	cudaFree(FDAJ_dev_TH_aver);
	cudaFree(FDAJ_dev_s_amp);
	cufftDestroy(FDAJ_plan_c2r);
	cufftDestroy(FDAJ_plan_r2c);
	cudaFree(FDAJ_dev_array_s_out);
	cudaFree(FDAJ_dev_s_out);
	cudaFree(FDAJ_dev_s_out_quan);
	cudaFree(FDAJ_dev_array_s_out_quan);
	cudaFree(FDAJ_dev_s_out_max);
	cudaFree(FDAJ_dev_s_in);
	cudaFree(FDAJ_dev_window);
	cudaFree(FDAJ_dev_array_signal);
	cudaFree(FDAJ_dev_signal[0]);
	cudaFreeHost(FDAJ_signal[0]);

	fclose(fp[0]);
}

void FDAJ_write_file(int total_point, int sel)
{
	switch (sel)
	{
	case WRITE_ANTI_RESULT_TXT:
		for (int i = 0; i < total_point; i++)
		{
			fprintf(FDAJ_out_file, "%d\n", FDAJ_s_out[i]);
		}
		break;

	case WRITE_ANTI_RESULT_BINARY_CHAR:
		fwrite(FDAJ_s_out, sizeof(SIGNAL_TYPE_QUAN), total_point, FDAJ_out_file);
		break;

	default:
		break;
	}
}
