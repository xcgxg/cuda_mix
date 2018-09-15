#pragma once

#include "global_main.h"
#include "global_STAP.h"
#include "global_FDAJ.h"
#include "global_LMS.h"
#include "global_anti_none.h"
#include "../receiver/init_receiver.h"


void global_exec(int sel)
{
	SIGNAL_TYPE each_data_process_time = EACH_DATA_PROCESS_TIME;         //Êý¾Ý´¦Àíµ¥Î»Ê±¼äs

	int sn = sameple_rate / 1E3;   //µ¥´Î´¦ÀíÊý¾ÝµÄµãÊý
	//¶¨Òå·ÂÕæÐÅºÅ²ÎÊý£¬·ÂÕæÊ±³¤£º
	int s;   //ÐèÒªÑ­»·´¦ÀíµÄ´ÎÊý 
	s = floor(sameple_rate*each_data_process_time / sn);

	SIGNAL_TYPE d = signal_lamda / 2;          //ÕóÔª¼ä¾à lamda/2

	//FDAJ¿¹¸ÉÈÅ²ÎÊýÅäÖÃ£º
	int fdaj_steplength = sn;     //µ¥´Î´¦ÀíÊý¾Ý³¤¶È
	//µ¥´Î´¦ÀíµÄ´ÎÊý
	int each_number = (int)(sameple_rate*each_data_process_time / fdaj_steplength);

	/*printf("%d\n", each_number);
	printf("%d\n", each_number);*/


	int anti_sel = sel & ANTI_CONFIG_SEL;
	switch (anti_sel)
	{
	case CONFIG_STAP:
	case CONFIG_BEAM:
		init_STAP(m, sn, s, n, bit, sel);
		//printf("123\n");
		//if (!((CONFIG_BEAM & sel) ^ CONFIG_BEAM))
		if (CONFIG_BEAM == anti_sel)
		{
			beam_s_vector(EL_si, AZ_sj, d, m, n, array_antenna, sameple_rate, IF);
		}

		STAP_exec(m, n, sn, bit, s, sameple_rate, each_data_process_time, sel);

		destroy_STAP(m, sel);

		break;
	case CONFIG_FDAJ:
		init_FDAJ(fdaj_steplength, bit, each_number, sel); 

		FDAJ_exec(antenna, threshold, loop, bit, fdaj_steplength, each_number, each_data_process_time,sel);

		destroy_FDAJ(sel);

		break;
	case CONFIG_LMS:
		//¸Ã·½·¨Ê¹ÓÃCPU²¢ÐÐ´¦Àí·½°¸
		sn = sameple_rate / 1e1;

		init_LMS(sn, sel);

		LMS_exec(m, sn, bit, sameple_rate, sel);

		des_LMS(sel);

		break;

	case CONFIG_ANTI_NONE:
		sn = sameple_rate / 1e1;
		
		init_anti_none(sn, sel);

		anti_none_exec(sn, each_data_process_time, sel);

		destroy_anti_none(); 

		break;
		
	default:
		printf("%d select error!\n", anti_sel);
		break;
	}

	global_destory();
}

void global_init(int sel)
{
	//Time Stamp
	getTimeStamp();

	cublasCreate(&blas_handle);

	//³õÊ¼»¯¶ÓÁÐ
	init_queue(q);
	
	//³õÊ¼»¯socketÍ¨ÐÅ
	init_socket(socket_client, socket_sin, socket_len, socket_port, socket_addr,self_port);
	memset(&send_res_L1, 0, sizeof(SimRes));
	memset(&send_res_BD1, 0, sizeof(SimRes));
	memset(&send_res_BD3, 0, sizeof(SimRes));
	memset(&send_res_GLO1, 0, sizeof(SimRes));

	//³õÊ¼»¯Ö´ÐÐÁ÷
	for (int i = 0; i < NUM_STREAM; i++)
	{
		cudaStreamCreate(&cuda_stream[i]);
	}

	//Ñ¡ÔñÁ¿»¯Ëã·¨²ÎÊý
	switch (bit)
	{
	case 1:
		QUAN_config.x = QUAN_1_BIT_0;
		QUAN_config.y = QUAN_1_BIT_1;
		QUAN_config.z = QUAN_1_BIT_2;

		break;
	case 2:
		QUAN_config.x = QUAN_2_BIT_0;
		QUAN_config.y = QUAN_2_BIT_1;
		QUAN_config.z = QUAN_2_BIT_2;

		break;
	case 4:
		QUAN_config.x = QUAN_4_BIT_0;
		QUAN_config.y = QUAN_4_BIT_1;
		QUAN_config.z = QUAN_4_BIT_2;

		break;
	default:
		printf("bit sel error!\n");
		break;
	}

	//Ñ¡Ôñcuda kernelµÄÏß³Ì¹æ¸ñ
	int sn = sameple_rate / 1E3;
	switch (sn)
	{
	case STEPLENGTH_10MHZ:
		cuda_kernel_grid = CUDA_KERNEL_GRID_NUM_10MHZ;
		cuda_kernel_block = CUDA_KERNEL_BLOCK_NUM_10MHZ; 
		break;
	case STEPLENGTH_20MHZ:
		cuda_kernel_grid = CUDA_KERNEL_GRID_NUM_20MHZ;
		cuda_kernel_block = CUDA_KERNEL_BLOCK_NUM_20MHZ;
		break;
	case STEPLENGTH_46MHZ:
		cuda_kernel_grid = CUDA_KERNEL_GRID_NUM_46MHZ;
		cuda_kernel_block = CUDA_KERNEL_BLOCK_NUM_46MHZ;
		break;
	default:
		break;
	}

	init_receiver_var();

	switch (sel & RECEIVER_CONFIG_SEL)
	{
	case WITH_GPSL1_RECEIVER:
		SetIF(1.42e6);
		SetFS(sameple_rate);
		SetQB(bit);

		//SetFilePath(IF_DATA_FILE, filename);
		SetFilePath(FILE_PATH, receiver_filepath.c_str());

		SetBD(1575.42e6);

		GPSL1_receiver_init_num = GPSL1_sim_main_init();
		break;
	case WITH_BD1_RECEIVER:
		BD1_SetIF(2.098e6);
		BD1_SetFS(sameple_rate);
		BD1_SetQB(bit);

		//SetFilePath(IF_DATA_FILE, filename);
		BD1_SetFilePath(FILE_PATH, receiver_filepath.c_str());

		BD1_SetBD(1561.098e6);

		BD1_receiver_init_num = BD1_sim_main_init();
		break;
	case WITH_BD3_RECEIVER:
		BD3_SetIF(12.52e6);
		BD3_SetFS(sameple_rate);
		BD3_SetQB(bit);

		//SetFilePath(IF_DATA_FILE, filename);
		BD3_SetFilePath(FILE_PATH, receiver_filepath.c_str());

		BD3_SetBD(1268.52e6);

		BD3_receiver_init_num = BD3_sim_main_init();
		break;
	case WITH_GLO1_RECEIVER:
		GLO1_SetIF(5.0e6);
		GLO1_SetFS(sameple_rate);
		GLO1_SetQB(bit);

		//SetFilePath(IF_DATA_FILE, filename);
		GLO1_SetFilePath(FILE_PATH, receiver_filepath.c_str());

		GLO1_SetBD(1602.0e6);

		GLO1_receiver_init_num = GLO1_sim_main_init();
		break;
	default:
		break;
	}

}

void global_destory()
{
	for (int i = 0; i < NUM_STREAM; i++)
	{
		cudaStreamDestroy(cuda_stream[i]);
	}

	des_socket(socket_client);

	cublasDestroy(blas_handle);

}

void getTimeStamp()
{
	time_t t;
	struct tm lt = {};

	t = time(NULL);
	localtime_s(&lt, &t);
	memset(nowtime, 0, sizeof(nowtime));
	strftime(nowtime, 50, "last_signal_2_%Y_%m_%d_%H_%M_%S.txt", &lt);

}