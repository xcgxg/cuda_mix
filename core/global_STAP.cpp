
#include "global_STAP.h"

#include "Eigen/Eigen"

using namespace Eigen;

extern void cuda_LCMV(int q_front, int s, int times_LCVM);

//信号输入
int STAP_row_sig_STAP;
int STAP_col_sig_STAP;

SIGNAL_TYPE *STAP_signal[QUEUE_MAX_SIZE] = {};
SIGNAL_TYPE *STAP_dev_signal[QUEUE_MAX_SIZE] = {};

//输出结果
SIGNAL_TYPE_QUAN *STAP_anti_out_quan = 0;
SIGNAL_TYPE *STAP_dev_anti_out = 0;
SIGNAL_TYPE_QUAN *STAP_dev_anti_out_quan;
SIGNAL_TYPE *STAP_dev_anti_out_max = 0;
SIGNAL_TYPE **STAP_dev_array_anti_out = 0;	//device中输出结果数组的首地址
SIGNAL_TYPE_QUAN **STAP_dev_array_anti_out_quan;

//信号输入后的空时处理后的信号
SIGNAL_TYPE *STAP_signal_STAP[QUEUE_MAX_SIZE] = {};
//SIGNAL_TYPE *STAP_array_signal_STAP[QUEUE_MAX_SIZE][TEST_CHUNK]; //将device中信号数组的首地址存在host数组中
SIGNAL_TYPE **STAP_dev_array_signal_STAP = 0;	//device中信号数组的首地址

/*矩阵运算*/
int STAP_lead_dimension_matrix_R;
int STAP_lead_dimension_matrix_A;
int STAP_size_matrix_R;
SIGNAL_TYPE *STAP_array_matrix_R[TEST_CHUNK];
SIGNAL_TYPE **STAP_dev_array_matrix_R = 0;	//device中矩阵R数组的首地址
SIGNAL_TYPE *STAP_array_matrix_R_inver[TEST_CHUNK];
SIGNAL_TYPE **STAP_dev_array_matrix_R_inver = 0;
SIGNAL_TYPE **STAP_dev_array_matrix_R_inver_1col = 0;	//device中矩阵R'的第一列的数组的首地址

//CUDA矩阵运算相关量
int *STAP_PivotArray;
int *STAP_infoArray;
/*矩阵运算*/

//量化参数
SIGNAL_TYPE STAP_pow_quantization = 0;

FILE *STAP_out_file;

//波束成形
SIGNAL_TYPE *beam_vector_real;
SIGNAL_TYPE *dev_beam_vector_real;
SIGNAL_TYPE *beam_vector_image;
SIGNAL_TYPE *dev_beam_vector_image;
SIGNAL_TYPE *dev_R_inv_v_real[TEST_CHUNK];
SIGNAL_TYPE **dev_array_R_inv_v_real;
SIGNAL_TYPE *dev_R_inv_v_image[TEST_CHUNK];
SIGNAL_TYPE **dev_array_R_inv_v_image;
SIGNAL_TYPE *dev_v_real_R_inv[TEST_CHUNK];
SIGNAL_TYPE **dev_array_v_real_R_inv;
SIGNAL_TYPE *dev_v_image_R_inv[TEST_CHUNK];
SIGNAL_TYPE **dev_array_v_image_R_inv;
SIGNAL_TYPE *dev_beam_scal_real;
SIGNAL_TYPE *dev_beam_scal_image;


/* 输入信号转换成空时矩阵
输入参数：
signal:输入的M信号；
m:输入信号的行数
n:输入信号的列数
s:循环次数
CN:抽头数=延迟单元数+1；
delay:各个抽头之间的延迟
*/
void Mat_chg(int m, int n, int s, int CN, int delay, int next_signal, int times_chg)
{
	SIGNAL_TYPE *current_dev_signal = 0, *current_signal_STAP = 0;
	int real_col_sig_STAP = n - (CN - 1) * delay;

	current_dev_signal = STAP_dev_signal[next_signal];
	current_signal_STAP = STAP_signal_STAP[next_signal];

	for (int i = 0; i < s; i++)
	{
		for (int o = 0; o < m; o++)
		{
			current_dev_signal = STAP_dev_signal[next_signal] + o * s * n + i * n;

			for (int j = CN - 1; j >= 0; j--)
			{
				cudaMemcpyAsync(current_signal_STAP, current_dev_signal + j, real_col_sig_STAP * sizeof(SIGNAL_TYPE),
					cudaMemcpyDeviceToDevice, cuda_stream[0]);
				/*cudaMemcpy(current_signal_STAP, current_dev_signal + j, col_sig_STAP * sizeof(SIGNAL_TYPE),
				cudaMemcpyDeviceToDevice);*/

				current_signal_STAP += STAP_col_sig_STAP;
			}
		}
	}
}

void STAP_exec(int m, int n, int sn, int bit, int s, int sameple_rate, SIGNAL_TYPE data_process_time, int sel)
{
	int count_for = data_time_length / data_process_time; //printf("%d\n", count_for);
	int anti_sel = ANTI_CONFIG_SEL & sel;
	int with_receiver = RECEIVER_CONFIG_SEL & sel;
	int write_file = WRITE_ANTI_RESULT_CONFIG_SEL & sel;
	int total_point = s * STAP_col_sig_STAP;

	switch (anti_sel)
	{
	case CONFIG_STAP:
		printf("start STAP\n");
		break;
	case CONFIG_BEAM:
		printf("start BEAM\n");
	default:
		break;
	}

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
						STAP_read_file(m, sn, s, n, i);
					}
				}
#pragma omp section
				{
					for (int i = 0; i < count_for; i++)
					{
						cuda_STAP(m, sn, s, n, anti_sel);
						cudaMemcpy(STAP_anti_out_quan, STAP_dev_anti_out_quan, s * STAP_col_sig_STAP * sizeof(SIGNAL_TYPE_QUAN),
							cudaMemcpyDeviceToHost); 

						//cudaDeviceSynchronize();

						/*float wop[12] = {};
						cudaMemcpy(wop, dev_beam_scal_image, 12 * sizeof(SIGNAL_TYPE),
							cudaMemcpyDeviceToHost);
						for (int j = 0; j < 12; j++)
						{
							printf("%f ", wop[j]);
						}
						printf("\n");*/
						/*float temp_inver[4 * 4] = {};

						cudaMemcpy(temp_inver, STAP_array_matrix_R_inver[0], 4 * 4 * sizeof(SIGNAL_TYPE),
							cudaMemcpyDeviceToHost);
						for (int j = 0; j < 4; j++)
						{
							printf("%f ", temp_inver[j]);
						}
						printf("\t");
						for (int j = 0; j < 4; j++)
						{
							printf("%f ", temp_inver[4 + j]);
						}
						printf("\n");
						for (int o = 0; o < 4; o++)
						{
							for (int j = 0; j < 4; j++)
							{
								printf("%f ", temp_inver[o * 4 + j]);
							}
						}*/
						
						//printf("\n");
						//float temp[48] = {};
						//cudaMemcpy(temp, dev_R_inv_v_real[0], 48 * sizeof(SIGNAL_TYPE),
						//	cudaMemcpyDeviceToHost);
						//for (int o = 0; o < 12; o++)
						//{
						//	//float ttt = 0;
						//	for (int j = 0; j < 4; j++)
						//	{
						//		printf("%f ", temp[o * 4 + j]);
						//	}
						//	printf("\n");							
						//}
						//printf("\n");

						if (write_file)
						{
							STAP_write_file(total_point, write_file);

							printf("%d\n", i);
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
	cudaDeviceSynchronize();
	/*pipeline*/

	switch (anti_sel)
	{
	case CONFIG_STAP:
		printf("STAP end\n");
		break;
	case CONFIG_BEAM:
		printf("BEAM end\n");
	default:
		break;
	}
	/*新增代码*/

	// printf("cudaGetLastError = %d\n", cudaGetLastError());
	// printf("GetLastError = %d\n", GetLastError());
}

void cuda_STAP(int m, int sn, int s, int n, int sel)
{
	while (isQueueEmpty(q));	

	int tmp = currentQueueFront(q); 
	
	//printf("cudaGetLastError = %d\n", cudaGetLastError());
	cuda_LCMV(tmp, s, sel); 
}

void init_STAP(int m, int sn, int s, int n, int bit, int sel)
{
	char fn[1000];
	const char *filename = anti_filename.c_str();

	/*printf("%s\n", filename);
	printf("%s\n", filename);*/

	//打开文件
	int open_file_info;
	for (int i = 0; i<m; i++)
	{
		sprintf(fn, filename, (i + 1));    //字符串赋给filename
		open_file_info = fopen_s(&fp[i], fn, "rb"); //printf("%d %d\n", i, fp[i]);
		if (0 != open_file_info)
		{
			printf("error%d info %d\n", i, open_file_info);
			exit(-1);
		}
	}//printf("123\n");

	//信号输入初始化
	cudaHostAlloc((void **)&STAP_signal[0], QUEUE_MAX_SIZE * m * sn * s * sizeof(SIGNAL_TYPE), cudaHostAllocDefault);
	//printf("cudaGetLastError = %d\n", cudaGetLastError());
	cudaMalloc((void**)&(STAP_dev_signal[0]), QUEUE_MAX_SIZE * m * sn * s * sizeof(SIGNAL_TYPE));
	{
		SIGNAL_TYPE *temp_signal = STAP_signal[0];
		SIGNAL_TYPE *temp_dev_signal = STAP_dev_signal[0];

		for (int i = 0; i < QUEUE_MAX_SIZE; i++)
		{
			STAP_signal[i] = temp_signal + i * m * sn * s;
			STAP_dev_signal[i] = temp_dev_signal + i * m * sn * s;
		}
	}

	/*结果输出初始化*/
	STAP_row_sig_STAP = m * n; 
	STAP_col_sig_STAP = sn; //printf("%d\n", STAP_col_sig_STAP);
	/*col_sig_STAP = sn - (n - 1) * 1;*/

	//抗干扰结果为量化后
	cudaHostAlloc((void **)&STAP_anti_out_quan, s * sn * sizeof(SIGNAL_TYPE_QUAN), cudaHostAllocDefault);

	//抗干扰过程按照浮点类型计算
	cudaMalloc((void**)&(STAP_dev_anti_out), s * sn * sizeof(SIGNAL_TYPE));
	cudaMalloc((void**)&(STAP_dev_anti_out_max), s * sizeof(SIGNAL_TYPE));
	cudaMalloc((void**)&(STAP_dev_array_anti_out), s * sizeof(SIGNAL_TYPE *));

	//抗干扰结果按8字节输出
	cudaMalloc((void**)&(STAP_dev_anti_out_quan), s * sn * sizeof(SIGNAL_TYPE_QUAN));
	cudaMalloc((void**)&(STAP_dev_array_anti_out_quan), s * sizeof(SIGNAL_TYPE_QUAN *));

	//将anti_out数组的首地址拷贝到dev_array_anti_out中
	{
		SIGNAL_TYPE **temp_array_anti_out=new SIGNAL_TYPE*[s];
		SIGNAL_TYPE_QUAN **temp_array_anti_out_quan = new SIGNAL_TYPE_QUAN *[s];

		for (int i = 0; i < s; i++)
		{
			temp_array_anti_out[i] = STAP_dev_anti_out + i * sn;
			temp_array_anti_out_quan[i] = STAP_dev_anti_out_quan + i * sn;
		}
		cudaMemcpy(STAP_dev_array_anti_out, temp_array_anti_out, s * sizeof(SIGNAL_TYPE *),
			cudaMemcpyHostToDevice);
		cudaMemcpy(STAP_dev_array_anti_out_quan, temp_array_anti_out_quan, s * sizeof(SIGNAL_TYPE_QUAN *),
			cudaMemcpyHostToDevice);

		delete[] temp_array_anti_out_quan;
		delete[] temp_array_anti_out;
	}
	
	/*结果输出初始化*/

	/*信号输入后的空时处理后的信号*/
	cudaMalloc((void**)&(STAP_signal_STAP[0]), QUEUE_MAX_SIZE * STAP_row_sig_STAP * sn * s * sizeof(SIGNAL_TYPE));
	//初始化signal_STAP为0
	cudaMemset(STAP_signal_STAP[0], 0, QUEUE_MAX_SIZE * STAP_row_sig_STAP * sn * s * sizeof(SIGNAL_TYPE));
	cudaMalloc((void**)&(STAP_dev_array_signal_STAP), QUEUE_MAX_SIZE * s * sizeof(SIGNAL_TYPE *));
	//将signal_STAP数组的首地址拷贝到dev_array_signal_STAP中
	{
		SIGNAL_TYPE **tmp_array_signal_STAP = new SIGNAL_TYPE*[QUEUE_MAX_SIZE * s];

		for (int i = 0; i < QUEUE_MAX_SIZE; i++)
		{
			STAP_signal_STAP[i] = STAP_signal_STAP[0] + i * STAP_row_sig_STAP * sn * s;

			for (int o = 0; o < s; o++)
			{
				tmp_array_signal_STAP[i * s + o] = STAP_signal_STAP[i] + o * STAP_row_sig_STAP * sn;
			}
		}
		cudaMemcpy(STAP_dev_array_signal_STAP, tmp_array_signal_STAP, QUEUE_MAX_SIZE * s * sizeof(SIGNAL_TYPE *),
			cudaMemcpyHostToDevice);

		delete[] tmp_array_signal_STAP;
	}
	/*信号输入后的空时处理后的信号*/

	/*矩阵运算*/
	STAP_lead_dimension_matrix_R = STAP_row_sig_STAP;
	STAP_lead_dimension_matrix_A = STAP_row_sig_STAP;
	STAP_size_matrix_R = STAP_lead_dimension_matrix_R * STAP_lead_dimension_matrix_R;

	//将matrix_R数组的首地址拷贝到dev_array_matrix_R中
	cudaMalloc((void**)&(STAP_dev_array_matrix_R), s * sizeof(SIGNAL_TYPE *));
	{
		cudaMalloc((void**)&(STAP_array_matrix_R[0]), s * STAP_size_matrix_R * sizeof(SIGNAL_TYPE));
		for (int i = 0; i < s; i++)
		{
			STAP_array_matrix_R[i] = STAP_array_matrix_R[0] + i * STAP_size_matrix_R;
		}
		cudaMemcpy(STAP_dev_array_matrix_R, STAP_array_matrix_R, s * sizeof(SIGNAL_TYPE *),
			cudaMemcpyHostToDevice);
	}

	//初始化array_matrix_R_inver
	cudaMalloc((void**)&(STAP_dev_array_matrix_R_inver), s * sizeof(SIGNAL_TYPE *));
	cudaMalloc((void**)&(STAP_array_matrix_R_inver[0]), s * STAP_size_matrix_R * sizeof(SIGNAL_TYPE));
	{
		for (int i = 0; i < s; i++)
		{
			STAP_array_matrix_R_inver[i] = STAP_array_matrix_R_inver[0] + i * STAP_size_matrix_R;
		}
		cudaMemcpy(STAP_dev_array_matrix_R_inver, STAP_array_matrix_R_inver, s * sizeof(SIGNAL_TYPE *),
			cudaMemcpyHostToDevice);
	}

	//将matrix_R_inver_1col数组的首地址拷贝到dev_array_matrix_R_inver_1col中
	{
		cudaMalloc((void**)&(STAP_dev_array_matrix_R_inver_1col), s * sizeof(SIGNAL_TYPE *));
		cudaMemcpy(STAP_dev_array_matrix_R_inver_1col, STAP_array_matrix_R_inver, s * sizeof(SIGNAL_TYPE *),
			cudaMemcpyHostToDevice);
	}

	//初始化矩阵运算相关变量
	cudaMalloc((void**)&(STAP_PivotArray), sizeof(int)* STAP_lead_dimension_matrix_R * s);
	cudaMalloc((void**)&(STAP_infoArray), sizeof(int)* s);
	/*矩阵运算*/

	//打开输出文件
	if (sel & WRITE_ANTI_RESULT_CONFIG_SEL)
	{
		int tmp = sel & WRITE_ANTI_RESULT_CONFIG_SEL;
		switch (tmp)
		{
		case WRITE_ANTI_RESULT_TXT:
			STAP_out_file = fopen(nowtime, "a+");
			break;
		case WRITE_ANTI_RESULT_BINARY_CHAR:
			STAP_out_file = fopen(nowtime, "ab");
			break;
		default:
			break;
		}
		
		if (NULL == STAP_out_file)
		{
			printf("can't open STAP out file.\n");

			exit(-1);
		}
	}
	

	STAP_pow_quantization = pow(2.0, bit-1) - 1; //printf("%d\n", STAP_col_sig_STAP);

	//初始化波束成形
	if (CONFIG_BEAM == (sel & ANTI_CONFIG_SEL))
	{
		beam_vector_real = new SIGNAL_TYPE[2 * STAP_row_sig_STAP * BEAM_SIG_VECTOR];
		//beam_vector的实部虚部合为一个数组
		beam_vector_image = beam_vector_real + STAP_row_sig_STAP * BEAM_SIG_VECTOR;

		cudaMalloc((void**)&(dev_beam_vector_real), 2 * STAP_row_sig_STAP * BEAM_SIG_VECTOR * sizeof(SIGNAL_TYPE));
		//dev_beam_vector的实部虚部合为一个数组
		dev_beam_vector_image = dev_beam_vector_real + STAP_row_sig_STAP * BEAM_SIG_VECTOR;

		//STAP_row_sig_STAP与STAP_lead_dimension_matrix_R相等，均为阵源数*延时单元数加一(m*n)

		cudaMalloc((void**)&(dev_R_inv_v_real[0]), s * STAP_lead_dimension_matrix_R * BEAM_SIG_VECTOR * sizeof(SIGNAL_TYPE));
		cudaMalloc((void**)&(dev_array_R_inv_v_real), s * sizeof(SIGNAL_TYPE *));

		cudaMalloc((void**)&(dev_R_inv_v_image[0]), s * STAP_lead_dimension_matrix_R * BEAM_SIG_VECTOR * sizeof(SIGNAL_TYPE));
		cudaMalloc((void**)&(dev_array_R_inv_v_image), s * sizeof(SIGNAL_TYPE *));

		cudaMalloc((void**)&(dev_v_real_R_inv[0]), s * STAP_lead_dimension_matrix_R * BEAM_SIG_VECTOR * sizeof(SIGNAL_TYPE));
		cudaMalloc((void**)&(dev_array_v_real_R_inv), s * sizeof(SIGNAL_TYPE *));

		cudaMalloc((void**)&(dev_v_image_R_inv[0]), s * STAP_lead_dimension_matrix_R * BEAM_SIG_VECTOR * sizeof(SIGNAL_TYPE));
		cudaMalloc((void**)&(dev_array_v_image_R_inv), s * sizeof(SIGNAL_TYPE *));

		{
			for (int i = 0; i < s; i++)
			{
				dev_R_inv_v_real[i] = dev_R_inv_v_real[0] + i * STAP_lead_dimension_matrix_R * BEAM_SIG_VECTOR;
				dev_R_inv_v_image[i] = dev_R_inv_v_image[0] + i * STAP_lead_dimension_matrix_R * BEAM_SIG_VECTOR;
				dev_v_real_R_inv[i] = dev_v_real_R_inv[0] + i * STAP_lead_dimension_matrix_R * BEAM_SIG_VECTOR;
				dev_v_image_R_inv[i] = dev_v_image_R_inv[0] + i * STAP_lead_dimension_matrix_R * BEAM_SIG_VECTOR;
			}
			cudaMemcpy(dev_array_R_inv_v_real, dev_R_inv_v_real, s * sizeof(SIGNAL_TYPE *),
				cudaMemcpyHostToDevice);
			cudaMemcpy(dev_array_R_inv_v_image, dev_R_inv_v_image, s * sizeof(SIGNAL_TYPE *),
				cudaMemcpyHostToDevice);
			cudaMemcpy(dev_array_v_real_R_inv, dev_v_real_R_inv, s * sizeof(SIGNAL_TYPE *),
				cudaMemcpyHostToDevice);
			cudaMemcpy(dev_array_v_image_R_inv, dev_v_image_R_inv, s * sizeof(SIGNAL_TYPE *),
				cudaMemcpyHostToDevice);
		}

		cudaMalloc((void**)&(dev_beam_scal_real), s * BEAM_SIG_VECTOR * sizeof(SIGNAL_TYPE));
		cudaMalloc((void**)&(dev_beam_scal_image), s * BEAM_SIG_VECTOR * sizeof(SIGNAL_TYPE));
		
	}

	switch (sel & RECEIVER_CONFIG_SEL)
	{
	case WITH_GPSL1_RECEIVER:
		init_receiver_buf(GPSL1_receiver_buf, s * sn, STAP_anti_out_quan);
		break;
	case WITH_BD1_RECEIVER:
		init_receiver_buf(BD1_receiver_buf, s * sn, STAP_anti_out_quan);
		break;
	case WITH_BD3_RECEIVER:
		init_receiver_buf(BD3_receiver_buf, s * sn, STAP_anti_out_quan);
		break;
	case WITH_GLO1_RECEIVER:
		init_receiver_buf(GLO1_receiver_buf, s * sn, STAP_anti_out_quan);
		break;
	default:
		break;
	}
	
}

void destroy_STAP(int m,int sel)
{
	if (CONFIG_BEAM == (sel & ANTI_CONFIG_SEL))
	{
		cudaFree(dev_beam_scal_image);
		cudaFree(dev_beam_scal_real);

		cudaFree(dev_array_v_image_R_inv);
		cudaFree(dev_v_image_R_inv[0]);

		cudaFree(dev_array_v_real_R_inv);
		cudaFree(dev_v_real_R_inv[0]);

		cudaFree(dev_array_R_inv_v_image);
		cudaFree(dev_R_inv_v_image[0]);

		cudaFree(dev_array_R_inv_v_real);
		cudaFree(dev_R_inv_v_real[0]);

		cudaFree(dev_beam_vector_real);
		
		delete[] beam_vector_real;
	}

	//关闭读取文件
	for (int i = 0; i < m; i++)
	{
		/*if (fp[i])
		{
			fclose(fp[i]);
		}*/
		//int tmp = fclose(fp[i]); printf("%d %d %d\n", i, fp[i], tmp);

		/*if (fclose(fp[i]))
		{ 
			perror("fclose");
		}*/
		//printf("%d %d\n", i, fp[i]);

		fclose(fp[i]);
	}

	if (sel & WRITE_ANTI_RESULT_CONFIG_SEL)
	{
		fclose(STAP_out_file);
	}
	
	cudaFree(STAP_PivotArray);
	cudaFree(STAP_infoArray);
	cudaFree(STAP_dev_array_matrix_R_inver_1col);

	cudaFree(STAP_array_matrix_R_inver[0]);

	cudaFree(STAP_array_matrix_R[0]);
	cudaFree(STAP_dev_array_matrix_R);

	cudaFree(STAP_signal_STAP[0]);
	cudaFree(STAP_dev_array_signal_STAP);

	cudaFree(STAP_dev_anti_out_quan);
	cudaFree(STAP_dev_array_anti_out_quan);

	cudaFree(STAP_dev_array_anti_out);
	cudaFree(STAP_dev_anti_out_max);
	cudaFree(STAP_dev_anti_out);

	cudaFreeHost(STAP_anti_out_quan);
	cudaFree(STAP_dev_signal[0]);

	cudaFreeHost(STAP_signal[0]);

}

void STAP_read_file(int m, int sn, int s, int n, int times_read)
{
	//队列满了，就一直循环
	while (isQueueFull(q));

	//printf("begin read part of file...  %d\n", times_read);

	int next_signal = nextQueueRear(q);
	
	SIGNAL_TYPE *current_signal = STAP_signal[next_signal];

	for (int i = 0; i < m; i++)
	{
		fread_s(current_signal, s * sn * sizeof(SIGNAL_TYPE), sizeof(SIGNAL_TYPE), s * sn, fp[i]);
		//int num = fread_s(current_signal, s * sn * sizeof(SIGNAL_TYPE), sizeof(SIGNAL_TYPE), s * sn, fp[i]); printf("%d %d\n", times_read, num);

		current_signal += s * sn;
	}

	/*float temp[16] = {};
	for (int q = 0; q < 4; q++)
	{
		for (int p = 0; p < 4; p++)
		{
			printf("%f ", STAP_signal[next_signal][q * 4 + p]);
		}
		printf("\n");
	}*/
	

	cudaMemcpyAsync(STAP_dev_signal[next_signal], STAP_signal[next_signal], s * m * sn * sizeof(SIGNAL_TYPE),
		cudaMemcpyHostToDevice, cuda_stream[0]);

	//调用矩阵变换函数Mat_chg
	Mat_chg(m, sn, s, n, 1, next_signal, times_read);  //输入信号转换成空时矩阵

	cudaStreamSynchronize(cuda_stream[0]);

	in_queue(q);
	

	//printf("%d\n",times_read);

	//printf("read part of file end\n");
}

void STAP_write_file(int total_point, int sel)
{
	switch (sel)
	{
	case WRITE_ANTI_RESULT_TXT:
		for (int i = 0; i < total_point; i++)
		{
			fprintf(STAP_out_file, "%d\n", STAP_anti_out_quan[i]);
		}
		break;

	case WRITE_ANTI_RESULT_BINARY_CHAR:
		fwrite(STAP_anti_out_quan, sizeof(SIGNAL_TYPE_QUAN), total_point, STAP_out_file);
		break;

	default:
		break;
	}
		
}

//延迟单元对应的相位延迟计算
//n:权系数个数
//sameple:采样率
//IF:中频
VectorXcf delay_wt(SIGNAL_TYPE sameple, SIGNAL_TYPE IF, int n)
{
	int l;
	SIGNAL_TYPE wt;
	wt = -2 * PI * IF / sameple;
	VectorXf theta(n);
	VectorXcf b(n);
	for (l = 0; l<n; l++)
	{
		theta(l) = l*wt;
		b(l).real(cos(theta(l)));// set b(l)的实部
		b(l).imag(sin(theta(l)));// set b(l)的虚部
	}
	return b;
}

//波束形成导向矢量计算
//EL_si:信号的俯仰角            
//AZ_sj:信号的方位角
void beam_s_vector(SIGNAL_TYPE *EL_si, SIGNAL_TYPE *AZ_sj, SIGNAL_TYPE d, int m, int n, char array_s, SIGNAL_TYPE sameple, SIGNAL_TYPE IF)
{
	SIGNAL_TYPE sita;
	SIGNAL_TYPE ws[BEAM_SIG_VECTOR];
	VectorXf x(m);  //天线坐标
	VectorXf y(m);  //天线坐标
	VectorXf z(m);  //天线坐标
	VectorXf theta(m * BEAM_SIG_VECTOR);
	VectorXcf a(m * BEAM_SIG_VECTOR);
	VectorXcf b(n);
	VectorXcf c(m*n * BEAM_SIG_VECTOR);
	int l, k;
	switch (array_s)
	{
	case'Y':
		sita = 2 * PI / (m - 1);
		x(0) = 0;
		y(0) = 0;
		z(0) = 0;
		for (l = 1; l<m; l++)
		{
			x(l) = d*cos((l - 1)*sita);
			y(l) = d*sin((l - 1)*sita);
			z(l) = 0;
		}
		break;
	case'L':
		for (l = 0; l<m; l++)
		{
			x(l) = l*d;
			y(l) = 0;
			z(l) = 0;
		}
		break;
	case'P':
		for (l = 0; l<m; l++)
		{
			x(l) = array_posx[l];
			y(l) = array_posy[l];
			z(l) = array_posz[l];
		}
		break;

	default:printf("error\n");
	}

	int tmp_base_a;
	for (int tmp_i = 0; tmp_i < BEAM_SIG_VECTOR; tmp_i++)
	{
		tmp_base_a = tmp_i*m;

		ws[tmp_i] = (sin(EL_si[tmp_i] * PI / 180)) ;
		for (k = 0; k<m; k++)
		{
			/*theta(tmp_base_a + k) = ws[tmp_i] * (x(k)*cos(AZ_sj[tmp_i] * PI / 180) + y(k)*sin(AZ_sj[tmp_i] * PI / 180));*/
			theta(tmp_base_a+k) = -(2 * PI / signal_lamda)*(ws[tmp_i] * (x(k)*cos(AZ_sj[tmp_i] * PI / 180) + y(k)*sin(AZ_sj[tmp_i] * PI / 180)) + z(k)*cos(EL_si[tmp_i] * PI / 180));
			a(tmp_base_a + k).real(cos(theta(tmp_base_a + k)));// set a(k)的实部
			a(tmp_base_a + k).imag(sin(theta(tmp_base_a + k)));// set a(k)的虚部
		}
	}
	b = delay_wt(sameple, IF, n);
	c = kroneckerProduct(a, b);
	

	{
		int count = c.size();

		for (int i = 0; i < count; i++)
		{
			beam_vector_real[i] = c(i).real();
			beam_vector_image[i] = c(i).imag();

			//printf("%f %f\n", c(i).real(), c(i).imag());
		}

		//beam_vector的实部虚部合为一个数组
		cudaMemcpy(dev_beam_vector_real, beam_vector_real, 2 * count * sizeof(SIGNAL_TYPE),
			cudaMemcpyHostToDevice);
		// cudaMemcpyAsync(dev_beam_vector_image, beam_vector_image, count * sizeof(SIGNAL_TYPE),
		// 	cudaMemcpyHostToDevice, stream[0]);

		
	}
	
}
