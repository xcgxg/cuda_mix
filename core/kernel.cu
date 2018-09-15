#pragma once


#include "global_STAP.h"
#include "global_FDAJ.h"

extern int cuda_kernel_grid;
extern int cuda_kernel_block;

///*STAP*/
//#define STAP_CUDA_KERNEL_GRID_NUM 40
//#define STAP_CUDA_KERNEL_BLOCK_NUM 256
///*STAP*/
//
///*FDAJ*/
//#define FDAJ_CUDA_KERNEL_GRID_DIM 40
//#define FDAJ_CUDA_KERNEL_BLOCK_DIM 256//512
//#define FDAJ_CUDA_KERNEL_HANNING_GRID_DIM FDAJ_CUDA_KERNEL_GRID_DIM
//#define FDAJ_CUDA_KERNEL_HANNING_BLOCK_DIM FDAJ_CUDA_KERNEL_BLOCK_DIM
//#define FDAJ_CUDA_KERNEL_MUL_GRID_DIM FDAJ_CUDA_KERNEL_GRID_DIM
//#define FDAJ_CUDA_KERNEL_MUL_BLOCK_DIM FDAJ_CUDA_KERNEL_BLOCK_DIM
//#define FDAJ_CUDA_KERNEL_ABS_GRID_DIM FDAJ_CUDA_KERNEL_GRID_DIM
//#define FDAJ_CUDA_KERNEL_ABS_BLOCK_DIM FDAJ_CUDA_KERNEL_BLOCK_DIM
//#define FDAJ_CUDA_KERNEL_TH_GRID_DIM FDAJ_CUDA_KERNEL_GRID_DIM
//#define FDAJ_CUDA_KERNEL_TH_BLOCK_DIM FDAJ_CUDA_KERNEL_BLOCK_DIM
//#define FDAJ_CUDA_KERNEL_QUANTIZATION_GRID_DIM FDAJ_CUDA_KERNEL_GRID_DIM
//#define FDAJ_CUDA_KERNEL_QUANTIZATION_BLOCK_DIM FDAJ_CUDA_KERNEL_BLOCK_DIM
///*FDAJ*/

__global__ void cuda_kernel_DgemmBatched(SIGNAL_TYPE **dev_array_signal_STAP, SIGNAL_TYPE **dev_array_matrix_R, int col_sig_STAP,
	int lead_dimension_matrix_R)
{
	int i = blockIdx.x;
	const SIGNAL_TYPE one = 1;
	const SIGNAL_TYPE zero = 0;
	cublasHandle_t dev_blas_handle;
	cublasCreate(&dev_blas_handle);

	//printf("%d\n", lead_dimension_matrix_R);
	cublasXgemm(dev_blas_handle, CUBLAS_OP_T, CUBLAS_OP_N,
		lead_dimension_matrix_R, lead_dimension_matrix_R, col_sig_STAP, &one,
		dev_array_signal_STAP[i], col_sig_STAP,
		dev_array_signal_STAP[i], col_sig_STAP,
		&zero, dev_array_matrix_R[i], lead_dimension_matrix_R);

	cublasDestroy(dev_blas_handle);
}

__global__ void cuda_kernel_DgemmBatched_scal(SIGNAL_TYPE **dev_array_matrix_R, int col_sig_STAP, int lead_dimension_matrix_R)
{
	int i = blockIdx.x;

	SIGNAL_TYPE scal = 1.0 / col_sig_STAP; //printf("%f\n", scal);
	cublasHandle_t dev_blas_handle;

	cublasCreate(&dev_blas_handle);

	cublasXscal(dev_blas_handle, lead_dimension_matrix_R * lead_dimension_matrix_R, &scal, dev_array_matrix_R[i], 1);

	cublasDestroy(dev_blas_handle);
}

__global__ void cuda_kernel_scal_matrix_R_inver_1col(SIGNAL_TYPE **dev_array_matrix_R_inver_1col, int lead_dimension_matrix_R)
{
	int i = blockIdx.x;
	SIGNAL_TYPE scal = 1.0 / dev_array_matrix_R_inver_1col[i][0];
	//SIGNAL_TYPE scal = __fdiv_rn(1.0f, dev_array_matrix_R_inver_1col[i][0]);
	
	//printf("%f ", scal);

	cublasHandle_t dev_blas_handle;
	cublasCreate(&dev_blas_handle);

	cublasXscal(dev_blas_handle, lead_dimension_matrix_R, &scal, dev_array_matrix_R_inver_1col[i], 1);

	cublasDestroy(dev_blas_handle);

}

__global__ void cuda_kernel_inver_1col_mul(SIGNAL_TYPE **dev_array_matrix_R_inver_1col, SIGNAL_TYPE **dev_array_signal_STAP,
	SIGNAL_TYPE **dev_array_anti_out, int col_sig_STAP, int lead_dimension_matrix_R)
{
	int i = blockIdx.y;
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int count = col_sig_STAP;
	SIGNAL_TYPE res = 0;
	SIGNAL_TYPE *current_signal_STAP = dev_array_signal_STAP[i] + tid;

	if (tid < count)
	{
		for (int o = 0; o < lead_dimension_matrix_R; o++)
		{
			res += dev_array_matrix_R_inver_1col[i][o] * (*(current_signal_STAP + o * count));
		}

		dev_array_anti_out[i][tid] = res;
	}
}

__global__ void cuda_kernel_cublasIsamax(SIGNAL_TYPE **dev_input, SIGNAL_TYPE *dev_max, int n)
{
	int i = blockIdx.x;
	int max_index;
	cublasHandle_t blas_handle;

	cublasCreate(&blas_handle);

	cublasIXamax(blas_handle, n, (SIGNAL_TYPE *)(dev_input[i]), 1, &max_index);

	dev_max[i] = fabsX(dev_input[i][max_index - 1]);

	cublasDestroy(blas_handle);


	/*int i = blockIdx.x;
	SIGNAL_TYPE max;
	cublasHandle_t blas_handle;

	cublasCreate(&blas_handle);

	cublasSdot(blas_handle, n, (SIGNAL_TYPE *)(dev_input[i]), 1, (SIGNAL_TYPE *)(dev_input[i]), 1, &max);

	dev_max[i] = sqrtX(max / n) * 2.2910;

	cublasDestroy(blas_handle);*/
}

__global__ void cuda_kernel_vector_quantization_mul(SIGNAL_TYPE_QUAN **dev_output, SIGNAL_TYPE **dev_input, 
	SIGNAL_TYPE *dev_max, SIGNAL_TYPE pow_quantization, int n, int3 QUAN_config)
{
	int i = blockIdx.y;
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int num_all_thread = blockDim.x * gridDim.x;
	SIGNAL_TYPE max = dev_max[i];
	
	if (tid < n)
	{
		SIGNAL_TYPE some_input = dev_input[i][tid];

		SIGNAL_TYPE_QUAN tmp = (SIGNAL_TYPE_QUAN)(QUAN_config.x * roundX(some_input * pow_quantization / max));

		dev_output[i][tid] = (some_input >= 0) ? (tmp + QUAN_config.y) : (tmp + QUAN_config.z);

		/*4bit--1*/
		/*SIGNAL_TYPE_QUAN tmp = (SIGNAL_TYPE_QUAN)(roundX(some_input * pow_quantization / max));

		dev_output[i][tid] = (some_input >= 0) ? (tmp) : (tmp - 1);*/

		/*4bit--2*/
		/*SIGNAL_TYPE_QUAN tmp = (SIGNAL_TYPE_QUAN)(floorX((some_input >= 0 ? some_input : -some_input)* pow_quantization / max));
		dev_output[i][tid] = (some_input >= 0) ? (tmp + 1) : (-tmp - 1);*/

		/*2bit*/
		/*SIGNAL_TYPE_QUAN tmp = (SIGNAL_TYPE_QUAN)(2 * roundX(some_input * pow_quantization / max));

		dev_output[i][tid] = (some_input >= 0) ? (tmp + 1) : (tmp - 1);*/

		/*1bit--1*/
		/*SIGNAL_TYPE_QUAN *tmp = ((SIGNAL_TYPE_QUAN *)(&dev_input[i][tid])) + (sizeof(SIGNAL_TYPE) - 1);

		dev_output[i][tid] = -((*tmp) >> 7);*/

		/*1bit--2*/
		/*SIGNAL_TYPE_QUAN tmp = (SIGNAL_TYPE_QUAN)(roundX(some_input * pow_quantization / max));

		dev_output[i][tid] = (some_input >= 0) ? (tmp) : (tmp + 1);*/
	}

}

//__global__ void cuda_kernel_LCMV_ex(int col_sig_STAP, SIGNAL_TYPE **dev_array_anti_out, SIGNAL_TYPE *dev_anti_out_max,
//	int pow_quantization, SIGNAL_TYPE **__dev_array_signal_STAP, int lead_dimension_matrix_R,
//	int lead_dimension_matrix_A, int size_matrix_R, SIGNAL_TYPE **dev_array_matrix_R,
//	SIGNAL_TYPE **dev_array_matrix_R_inver, SIGNAL_TYPE **dev_array_matrix_R_inver_1col, int *PivotArray,
//	int *infoArray, int q_front, int s, SIGNAL_TYPE_QUAN **STAP_dev_array_anti_out_quan, int3  QUAN_config, 
//	int cuda_kernel_grid, int cuda_kernel_block)
//{
//	cublasHandle_t matrix_multi_blas_handle;
//	SIGNAL_TYPE ** dev_array_signal_STAP = __dev_array_signal_STAP + q_front * s;
//	cublasCreate(&matrix_multi_blas_handle);
//
//	//1.
//	//1.1 R=AxA'
//	cuda_kernel_DgemmBatched << <s, 1 >> >(dev_array_signal_STAP, dev_array_matrix_R, col_sig_STAP, lead_dimension_matrix_R);
//
//	//1.2
//	cuda_kernel_DgemmBatched_scal << <s, 1 >> >(dev_array_matrix_R, col_sig_STAP, lead_dimension_matrix_R);
//
//	//2. LU分解求矩阵
//	//2.1
//	cublasXgetrfBatched(matrix_multi_blas_handle, lead_dimension_matrix_R, dev_array_matrix_R,
//		lead_dimension_matrix_R, PivotArray, infoArray, s);
//
//	//2.2
//	const SIGNAL_TYPE **temp_dev_array_matrix_R = (const SIGNAL_TYPE **)dev_array_matrix_R;
//	cublasXgetriBatched(matrix_multi_blas_handle, lead_dimension_matrix_R, temp_dev_array_matrix_R,
//		lead_dimension_matrix_R, (const int *)PivotArray, dev_array_matrix_R_inver, lead_dimension_matrix_R,
//		infoArray, s);
//
//	//3.
//	//将matrix_R_inver_1col缩放小若干倍
//	cuda_kernel_scal_matrix_R_inver_1col << <s, 1 >> >(dev_array_matrix_R_inver_1col, lead_dimension_matrix_R);
//
//	///*for (int i = 0; i < 10; i++)
//	//{
//	//	printf("%lf ", dev_array_matrix_R_inver_1col[0][i]);
//	//}*/
//
//	//4.
//	cuda_kernel_inver_1col_mul << <dim3(cuda_kernel_grid, s), cuda_kernel_block >> >
//		(dev_array_matrix_R_inver_1col, dev_array_signal_STAP, dev_array_anti_out, col_sig_STAP,
//		lead_dimension_matrix_R);
//
//	/*for (int i = 0; i < 10000; i++)
//	{
//		printf("%f ", dev_array_anti_out[0][i]);
//	}
//	printf("\n");*/
//
//	//5.量化
//	//5.1
//	cuda_kernel_cublasIsamax << <s, 1 >> >(dev_array_anti_out, dev_anti_out_max, col_sig_STAP);
//
//	/*for (int i = 0; i < s; i++)
//	{
//		printf("%f\n", dev_anti_out_max[i]);
//	}*/
//
//	//5.2
//	cuda_kernel_vector_quantization_mul << <dim3(cuda_kernel_grid, s), cuda_kernel_block >> >
//		(STAP_dev_array_anti_out_quan, dev_array_anti_out, dev_anti_out_max, pow_quantization, col_sig_STAP, QUAN_config);
//
//
//	cublasDestroy(matrix_multi_blas_handle);
//
//	/*for (int i = 0; i < 100; i++)
//	{
//		for (int o = 0; o < 4; o++)
//		{
//			for (int j = 0; j < 4; j++)
//			{
//				printf("%f ", dev_array_matrix_R_inver[i][o * 4 + j]);
//			}
//			printf("\n");
//		}
//	}*/
//}

void cuda_kernel_LCMV(int q_front, int s)
{
	cublasHandle_t matrix_multi_blas_handle;
	SIGNAL_TYPE ** tmp_dev_array_signal_STAP = STAP_dev_array_signal_STAP + q_front * s;
	cublasCreate(&matrix_multi_blas_handle);

	//1.
	//1.1 R=AxA'
	cuda_kernel_DgemmBatched << <s, 1 >> >(tmp_dev_array_signal_STAP, STAP_dev_array_matrix_R, STAP_col_sig_STAP, STAP_lead_dimension_matrix_R);

	//1.2
	cuda_kernel_DgemmBatched_scal << <s, 1 >> >(STAP_dev_array_matrix_R, STAP_col_sig_STAP, STAP_lead_dimension_matrix_R);

	//2. LU分解求矩阵
	//2.1
	cublasXgetrfBatched(matrix_multi_blas_handle, STAP_lead_dimension_matrix_R, STAP_dev_array_matrix_R,
		STAP_lead_dimension_matrix_R, STAP_PivotArray, STAP_infoArray, s);

	//2.2
	const SIGNAL_TYPE **temp_dev_array_matrix_R = (const SIGNAL_TYPE **)STAP_dev_array_matrix_R;
	cublasXgetriBatched(matrix_multi_blas_handle, STAP_lead_dimension_matrix_R, temp_dev_array_matrix_R,
		STAP_lead_dimension_matrix_R, (const int *)STAP_PivotArray, STAP_dev_array_matrix_R_inver, STAP_lead_dimension_matrix_R,
		STAP_infoArray, s);

	//3.
	//将matrix_R_inver_1col缩放小若干倍
	cuda_kernel_scal_matrix_R_inver_1col << <s, 1 >> >(STAP_dev_array_matrix_R_inver_1col, STAP_lead_dimension_matrix_R);

	///*for (int i = 0; i < 10; i++)
	//{
	//	printf("%lf ", dev_array_matrix_R_inver_1col[0][i]);
	//}*/

	//4.
	cuda_kernel_inver_1col_mul << <dim3(cuda_kernel_grid, s), cuda_kernel_block >> >
		(STAP_dev_array_matrix_R_inver_1col, tmp_dev_array_signal_STAP, STAP_dev_array_anti_out, STAP_col_sig_STAP,
		STAP_lead_dimension_matrix_R);

	/*for (int i = 0; i < 10000; i++)
	{
	printf("%f ", dev_array_anti_out[0][i]);
	}
	printf("\n");*/

	//5.量化
	//5.1
	cuda_kernel_cublasIsamax << <s, 1 >> >(STAP_dev_array_anti_out, STAP_dev_anti_out_max, STAP_col_sig_STAP);

	/*for (int i = 0; i < s; i++)
	{
	printf("%f\n", dev_anti_out_max[i]);
	}*/

	//5.2
	cuda_kernel_vector_quantization_mul << <dim3(cuda_kernel_grid, s), cuda_kernel_block >> >
		(STAP_dev_array_anti_out_quan, STAP_dev_array_anti_out, STAP_dev_anti_out_max, STAP_pow_quantization, STAP_col_sig_STAP, QUAN_config);


	cublasDestroy(matrix_multi_blas_handle);

	/*for (int i = 0; i < 100; i++)
	{
	for (int o = 0; o < 4; o++)
	{
	for (int j = 0; j < 4; j++)
	{
	printf("%f ", dev_array_matrix_R_inver[i][o * 4 + j]);
	}
	printf("\n");
	}
	}*/
}

__global__ void cuda_kernel_beam_R_inv_v(SIGNAL_TYPE **dev_array_matrix_R_inver, SIGNAL_TYPE
	*dev_beam_vector_real, SIGNAL_TYPE *dev_beam_vector_image, SIGNAL_TYPE **dev_array_R_inv_v_real, 
	SIGNAL_TYPE **dev_array_R_inv_v_image, SIGNAL_TYPE ** dev_array_v_real_R_inv, SIGNAL_TYPE ** 
	dev_array_v_image_R_inv, int lead_dimension_matrix_R)
{
	//第几组数据
	int i = blockIdx.x;
	
	//一组数据12个卫星通道
	int which_vector_i = lead_dimension_matrix_R * threadIdx.x;
	cublasHandle_t dev_blas_handle;
	SIGNAL_TYPE one = 1;
	SIGNAL_TYPE minus_one = -1;
	SIGNAL_TYPE zero = 0;
	cublasCreate(&dev_blas_handle);

	cublasXgemm(dev_blas_handle, CUBLAS_OP_N, CUBLAS_OP_N, 
		lead_dimension_matrix_R, 1, lead_dimension_matrix_R, &one, 
		dev_array_matrix_R_inver[i], lead_dimension_matrix_R, 
		dev_beam_vector_real + which_vector_i, lead_dimension_matrix_R,
		&zero, dev_array_R_inv_v_real[i] + which_vector_i, lead_dimension_matrix_R);

	//if (i == 0 && threadIdx.x == 0)
	//{
	//	for (int o = 0; o < 4; o++)
	//	{
	//		//float ttt = 0;
	//		for (int j = 0; j < 4; j++)
	//		{
	//			printf("%f ", dev_array_matrix_R_inver[0][j * 4 + o]);
	//		}
	//		printf("\n");
	//	}
	//}
	//if (i == 0 && threadIdx.x == 0)
	//{
	//	for (int o = 0; o < 12; o++)
	//	{
	//		//float ttt = 0;
	//		for (int j = 0; j < 4; j++)
	//		{
	//			printf("%f ", dev_beam_vector_real[o * 4 + j]);
	//		}
	//		printf("\n");							
	//	}
	//}
	
	cublasXgemm(dev_blas_handle, CUBLAS_OP_N, CUBLAS_OP_N, 
		1, lead_dimension_matrix_R, lead_dimension_matrix_R, &one, 
		dev_beam_vector_real + which_vector_i, 1,
		dev_array_matrix_R_inver[i], lead_dimension_matrix_R,
		&zero, dev_array_v_real_R_inv[i] + which_vector_i, 1);

	cublasXgemm(dev_blas_handle, CUBLAS_OP_N, CUBLAS_OP_N, 
		lead_dimension_matrix_R, 1, lead_dimension_matrix_R, &one,
		dev_array_matrix_R_inver[i], lead_dimension_matrix_R, 
		dev_beam_vector_image + which_vector_i, lead_dimension_matrix_R,
		&zero, dev_array_R_inv_v_image[i] + which_vector_i, lead_dimension_matrix_R);

	cublasXgemm(dev_blas_handle, CUBLAS_OP_N, CUBLAS_OP_N, 
		1, lead_dimension_matrix_R, lead_dimension_matrix_R, &minus_one,
		dev_beam_vector_image + which_vector_i, 1,
		dev_array_matrix_R_inver[i], lead_dimension_matrix_R,
		&zero, dev_array_v_image_R_inv[i] + which_vector_i, lead_dimension_matrix_R);

	cublasDestroy(dev_blas_handle);

}

__global__ void cuda_kernel_beam_scal_para(SIGNAL_TYPE **dev_array_v_real_R_inv, SIGNAL_TYPE *dev_beam_vector_real, SIGNAL_TYPE **dev_array_v_image_R_inv, SIGNAL_TYPE *dev_beam_vector_image, SIGNAL_TYPE *scal_real, SIGNAL_TYPE *scal_image, int lead_dimension_matrix_R)
{
	int i = blockIdx.x;

	//一组数据12个卫星通道
	int which_vector_i = lead_dimension_matrix_R * threadIdx.x;
	cublasHandle_t dev_blas_handle;
	SIGNAL_TYPE scal_real1, scal_real2, scal_image1, scal_image2;
	SIGNAL_TYPE one = 1;

	cublasCreate(&dev_blas_handle);

	cublasXdot(dev_blas_handle, lead_dimension_matrix_R, dev_array_v_real_R_inv[i] + which_vector_i, one,
		dev_beam_vector_real + which_vector_i, one, &scal_real1);

	cublasXdot(dev_blas_handle, lead_dimension_matrix_R, dev_array_v_image_R_inv[i] + which_vector_i, one,
		dev_beam_vector_image + which_vector_i, one, &scal_real2);

	cublasXdot(dev_blas_handle, lead_dimension_matrix_R, dev_array_v_real_R_inv[i] + which_vector_i, one,
		dev_beam_vector_image + which_vector_i, one, &scal_image1);

	cublasXdot(dev_blas_handle, lead_dimension_matrix_R, dev_array_v_image_R_inv[i] + which_vector_i, one,
		dev_beam_vector_real + which_vector_i, one, &scal_image2);

	scal_real[i * BEAM_SIG_VECTOR + threadIdx.x] = scal_real1 - scal_real2;
	scal_image[i * BEAM_SIG_VECTOR + threadIdx.x] = scal_image1 + scal_image2;

	//printf("%d %f %f\n", i, scal_real[i], scal_image[i]);

	cublasDestroy(dev_blas_handle);

}

__global__ void cuda_kernel_beam_scal(SIGNAL_TYPE **dev_array_R_inv_v_real, SIGNAL_TYPE **
	dev_array_R_inv_v_image, SIGNAL_TYPE *scal_real,SIGNAL_TYPE *scal_image, SIGNAL_TYPE **
	dev_array_matrix_R_inver_1col)
{
	int i = blockIdx.x;
	int o = threadIdx.x;
	int j = threadIdx.y;

	//节约空间，在R的逆矩阵的第二列存放权值矢量的虚部
	SIGNAL_TYPE *matrix_R_inver_1col_Sig_d_image = dev_array_matrix_R_inver_1col[i] + blockDim.x;
	
	dev_array_matrix_R_inver_1col[i][o] = 0;
	matrix_R_inver_1col_Sig_d_image[o] = 0;

	__syncthreads();

	SIGNAL_TYPE a = dev_array_R_inv_v_real[i][j * blockDim.x + o];
	SIGNAL_TYPE b = dev_array_R_inv_v_image[i][j * blockDim.x + o];
	SIGNAL_TYPE c = scal_real[i * BEAM_SIG_VECTOR + j];
	SIGNAL_TYPE d = scal_image[i * BEAM_SIG_VECTOR + j];

	atomicAdd(&dev_array_matrix_R_inver_1col[i][o], (a * c + b * d) / (c * c + d * d));
	atomicAdd(&matrix_R_inver_1col_Sig_d_image[o], (b * c - a * d) / (c * c + d * d));

	//printf("%d %d %f\n", i, o, dev_array_matrix_R_inver_1col[i][o]);
	/*printf("%d %f %f\n", i, c, d);*/

}

//__global__ void cuda_kernel_beam_ex(int col_sig_STAP, SIGNAL_TYPE **dev_array_anti_out, SIGNAL_TYPE *dev_anti_out_max,
//	int pow_quantization, SIGNAL_TYPE **dev_array_signal_STAP, int lead_dimension_matrix_R,
//	int lead_dimension_matrix_A, int size_matrix_R, SIGNAL_TYPE **dev_array_matrix_R,
//	SIGNAL_TYPE **dev_array_matrix_R_inver, SIGNAL_TYPE **dev_array_matrix_R_inver_1col, int *PivotArray,
//	int *infoArray, int q_front, int s, SIGNAL_TYPE *dev_beam_vector_real, SIGNAL_TYPE *dev_beam_vector_image,
//	SIGNAL_TYPE **dev_array_R_inv_v_real, SIGNAL_TYPE **dev_array_R_inv_v_image, SIGNAL_TYPE **dev_array_v_real_R_inv,
//	SIGNAL_TYPE **dev_array_v_image_R_inv, SIGNAL_TYPE *dev_beam_scal_real, SIGNAL_TYPE *dev_beam_scal_image,
//	SIGNAL_TYPE_QUAN **STAP_dev_array_anti_out_quan, int3  QUAN_config, int cuda_kernel_grid, int cuda_kernel_block)
//{
//	cublasHandle_t matrix_multi_blas_handle;
//	dev_array_signal_STAP += q_front * s;
//	cublasCreate(&matrix_multi_blas_handle);
//
//	const SIGNAL_TYPE one = 1;
//	const SIGNAL_TYPE zero = 0;
//
//	//1.
//	//1.1 R=AxA'
//	cuda_kernel_DgemmBatched << <s, 1 >> >(dev_array_signal_STAP, dev_array_matrix_R, col_sig_STAP, lead_dimension_matrix_R);
//
//	//1.2
//	cuda_kernel_DgemmBatched_scal << <s, 1 >> >(dev_array_matrix_R, col_sig_STAP, lead_dimension_matrix_R);
//
//	//2. LU分解求矩阵
//	//2.1
//	cublasXgetrfBatched(matrix_multi_blas_handle, lead_dimension_matrix_R, dev_array_matrix_R,
//		lead_dimension_matrix_R, PivotArray, infoArray, s);
//
//	//2.2
//	const SIGNAL_TYPE **temp_dev_array_matrix_R = (const SIGNAL_TYPE **)dev_array_matrix_R;
//	cublasXgetriBatched(matrix_multi_blas_handle, lead_dimension_matrix_R, temp_dev_array_matrix_R,
//		lead_dimension_matrix_R, (const int *)PivotArray, dev_array_matrix_R_inver, lead_dimension_matrix_R,
//		infoArray, s);
//
//	//3.
//	cuda_kernel_beam_R_inv_v << <s, BEAM_SIG_VECTOR >> >(dev_array_matrix_R_inver, dev_beam_vector_real,
//		dev_beam_vector_image, dev_array_R_inv_v_real, dev_array_R_inv_v_image, dev_array_v_real_R_inv,
//		dev_array_v_image_R_inv, lead_dimension_matrix_R);
//
//	/*SIGNAL_TYPE scal_real[TEST_CHUNK], scal_image[TEST_CHUNK];*/
//	cuda_kernel_beam_scal_para << <s, BEAM_SIG_VECTOR >> >(dev_array_v_real_R_inv, dev_beam_vector_real,
//		dev_array_v_image_R_inv, dev_beam_vector_image, dev_beam_scal_real, dev_beam_scal_image, lead_dimension_matrix_R);
//
//	cuda_kernel_beam_scal << <s, dim3(lead_dimension_matrix_R, BEAM_SIG_VECTOR) >> >(dev_array_R_inv_v_real,
//		dev_array_R_inv_v_image, dev_beam_scal_real, dev_beam_scal_image, dev_array_matrix_R_inver_1col);
//
//	//4.
//	cuda_kernel_inver_1col_mul << <dim3(cuda_kernel_grid, s), cuda_kernel_block >> >
//		(dev_array_matrix_R_inver_1col, dev_array_signal_STAP, dev_array_anti_out, col_sig_STAP,
//		lead_dimension_matrix_R);
//
//	/*for (int o = 0; o < lead_dimension_matrix_R; o++)
//	{
//	printf("%f ", dev_array_matrix_R_inver[0][o]);
//	}
//	printf("\n");*/
//
//	//5.量化
//	//5.1
//	cuda_kernel_cublasIsamax << <s, 1 >> >(dev_array_anti_out, dev_anti_out_max, col_sig_STAP);
//
//	//5.2
//	cuda_kernel_vector_quantization_mul << <dim3(cuda_kernel_grid, s), cuda_kernel_block >> >
//		(STAP_dev_array_anti_out_quan, dev_array_anti_out, dev_anti_out_max, pow_quantization, col_sig_STAP, QUAN_config);
//
//	cublasDestroy(matrix_multi_blas_handle);
//}

void cuda_kernel_beam(int q_front, int s)
{
	cublasHandle_t matrix_multi_blas_handle;
	SIGNAL_TYPE ** tmp_dev_array_signal_STAP = STAP_dev_array_signal_STAP + q_front * s;
	cublasCreate(&matrix_multi_blas_handle);

	const SIGNAL_TYPE one = 1;
	const SIGNAL_TYPE zero = 0;

	//1.
	//1.1 R=AxA'
	cuda_kernel_DgemmBatched << <s, 1 >> >(tmp_dev_array_signal_STAP, STAP_dev_array_matrix_R, STAP_col_sig_STAP, STAP_lead_dimension_matrix_R);

	//1.2
	cuda_kernel_DgemmBatched_scal << <s, 1 >> >(STAP_dev_array_matrix_R, STAP_col_sig_STAP, STAP_lead_dimension_matrix_R);

	//2. LU分解求矩阵
	//2.1
	cublasXgetrfBatched(matrix_multi_blas_handle, STAP_lead_dimension_matrix_R, STAP_dev_array_matrix_R,
		STAP_lead_dimension_matrix_R, STAP_PivotArray, STAP_infoArray, s);

	//2.2
	const SIGNAL_TYPE **temp_dev_array_matrix_R = (const SIGNAL_TYPE **)STAP_dev_array_matrix_R;
	cublasXgetriBatched(matrix_multi_blas_handle, STAP_lead_dimension_matrix_R, temp_dev_array_matrix_R,
		STAP_lead_dimension_matrix_R, (const int *)STAP_PivotArray, STAP_dev_array_matrix_R_inver, STAP_lead_dimension_matrix_R,
		STAP_infoArray, s);

	//3.
	cuda_kernel_beam_R_inv_v << <s, BEAM_SIG_VECTOR >> >(STAP_dev_array_matrix_R_inver, dev_beam_vector_real,
		dev_beam_vector_image, dev_array_R_inv_v_real, dev_array_R_inv_v_image, dev_array_v_real_R_inv,
		dev_array_v_image_R_inv, STAP_lead_dimension_matrix_R);

	/*SIGNAL_TYPE scal_real[TEST_CHUNK], scal_image[TEST_CHUNK];*/
	cuda_kernel_beam_scal_para << <s, BEAM_SIG_VECTOR >> >(dev_array_v_real_R_inv, dev_beam_vector_real,
		dev_array_v_image_R_inv, dev_beam_vector_image, dev_beam_scal_real, dev_beam_scal_image, STAP_lead_dimension_matrix_R);

	cuda_kernel_beam_scal << <s, dim3(STAP_lead_dimension_matrix_R, BEAM_SIG_VECTOR) >> >(dev_array_R_inv_v_real,
		dev_array_R_inv_v_image, dev_beam_scal_real, dev_beam_scal_image, STAP_dev_array_matrix_R_inver_1col);

	//4.
	cuda_kernel_inver_1col_mul << <dim3(cuda_kernel_grid, s), cuda_kernel_block >> >
		(STAP_dev_array_matrix_R_inver_1col, tmp_dev_array_signal_STAP, STAP_dev_array_anti_out, STAP_col_sig_STAP,
		STAP_lead_dimension_matrix_R);

	/*for (int o = 0; o < lead_dimension_matrix_R; o++)
	{
	printf("%f ", dev_array_matrix_R_inver[0][o]);
	}
	printf("\n");*/

	//5.量化
	//5.1
	cuda_kernel_cublasIsamax << <s, 1 >> >(STAP_dev_array_anti_out, STAP_dev_anti_out_max, STAP_col_sig_STAP);

	//5.2
	cuda_kernel_vector_quantization_mul << <dim3(cuda_kernel_grid, s), cuda_kernel_block >> >
		(STAP_dev_array_anti_out_quan, STAP_dev_array_anti_out, STAP_dev_anti_out_max, STAP_pow_quantization, STAP_col_sig_STAP, QUAN_config);

	cublasDestroy(matrix_multi_blas_handle);
}

__global__ void cuda_kernel_vector_mul(SIGNAL_TYPE **dev_input1, SIGNAL_TYPE *dev_input2,
	SIGNAL_TYPE **dev_result, int n)
{
	int i = blockIdx.y;
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int num_all_thread = blockDim.x * gridDim.x;

	if (tid < n)
	{
		dev_result[i][tid] = dev_input1[i][tid] * dev_input2[tid];
	}

}

void cuda_vector_mul(SIGNAL_TYPE **dev_input1, SIGNAL_TYPE *dev_input2, SIGNAL_TYPE **dev_result, int n, int times)
{
	cuda_kernel_vector_mul << <dim3(cuda_kernel_grid, times), cuda_kernel_block >> >
		(dev_input1, dev_input2, dev_result, n);
}

__global__ void cuda_kernel_vector_abs(COMPLEX_TYPE **dev_input, SIGNAL_TYPE **dev_result, int n)
{
	int i = blockIdx.y;
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int num_all_thread = blockDim.x * gridDim.x;

	while (tid < n)
	{
		dev_result[i][tid] = sqrtX(dev_input[i][tid].x * dev_input[i][tid].x +
			dev_input[i][tid].y * dev_input[i][tid].y);

		tid += num_all_thread;
	}

}

__global__ void cuda_kernel_cublasSasum(SIGNAL_TYPE **dev_array_s_amp, SIGNAL_TYPE *dev_TH_aver, int n, int T)
{
	int i = blockIdx.x;
	cublasHandle_t blas_handle;

	cublasCreate(&blas_handle);

	cublasXasum(blas_handle, n, dev_array_s_amp[i], 1, &dev_TH_aver[i]);

	//CUFFT_R2C 结果只保留非冗余的复数系数，即n/2+1个复数，第0个和第n/2+1个复数不重复
	dev_TH_aver[i] = (dev_TH_aver[i] * 2 - dev_array_s_amp[i][0] - dev_array_s_amp[i][n - 1]) / (2 * n - 2) * T;

	cublasDestroy(blas_handle);

}

__global__ void cuda_kernel_vector_TH(COMPLEX_TYPE **dev_input1, SIGNAL_TYPE **dev_input2, SIGNAL_TYPE* TH, int n)
{
	int i = blockIdx.y;
	int tid = threadIdx.x + blockIdx.x * blockDim.x;
	int num_all_thread = blockDim.x * gridDim.x;
	COMPLEX_TYPE some_input1;
	SIGNAL_TYPE some_input2;
	SIGNAL_TYPE some_TH = TH[i];

	while (tid < n)
	{
		some_input1 = dev_input1[i][tid];
		some_input2 = dev_input2[i][tid];

		dev_input1[i][tid].x = (some_input2 >= some_TH) ? (some_input1.x * some_TH / some_input2) : (some_input1.x);
		dev_input1[i][tid].y = (some_input2 >= some_TH) ? (some_input1.y * some_TH / some_input2) : (some_input1.y);

		tid += num_all_thread;
	}

}

__global__ void cuda_kernel_TH_filter(int K, COMPLEX_TYPE **dev_array_s_in, SIGNAL_TYPE **dev_array_s_amp,
	SIGNAL_TYPE *dev_TH_aver, int T, int sn, int cycle_number, int cuda_kernel_grid, int cuda_kernel_block)
{
	int n = (sn >> 1) + 1;

	while (K--)
	{
		cuda_kernel_vector_abs << <dim3(cuda_kernel_grid, cycle_number), cuda_kernel_block >> >
			(dev_array_s_in, dev_array_s_amp, n);

		cuda_kernel_cublasSasum << <cycle_number, 1 >> >(dev_array_s_amp, dev_TH_aver, n, T);

		cuda_kernel_vector_TH << <dim3(cuda_kernel_grid, cycle_number), cuda_kernel_block >> >
			(dev_array_s_in, dev_array_s_amp, dev_TH_aver, n);
	}
}

void cuda_TH_filter(int K, COMPLEX_TYPE **dev_array_s_in, SIGNAL_TYPE **dev_array_s_amp,
	SIGNAL_TYPE *dev_TH_aver, int T, int sn, int cycle_number)
{
	cuda_kernel_TH_filter << <1, 1, 0, 0 >> >(K, dev_array_s_in, dev_array_s_amp, dev_TH_aver,
		T, sn, cycle_number, cuda_kernel_grid, cuda_kernel_block);
}

void cuda_vector_quantization(SIGNAL_TYPE **dev_input, SIGNAL_TYPE_QUAN **dev_output, SIGNAL_TYPE pow_quantization, 
	int n, int times)
{
	cuda_kernel_cublasIsamax << <times, 1 >> >(dev_input, FDAJ_dev_s_out_max, n);

	cuda_kernel_vector_quantization_mul << <dim3(cuda_kernel_grid, times),
		cuda_kernel_block >> >(dev_output, dev_input, FDAJ_dev_s_out_max, pow_quantization, n, QUAN_config);

}

__global__ void cuda_kernel_hanning(SIGNAL_TYPE *dev_window, int n)
{
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	if (tid < n)
	{
		dev_window[tid] = 0.5 * (1 - cosX(2 * PI * tid / (n - 1)));
	}

}

void cuda_hanning(SIGNAL_TYPE *dev_window, int sn)
{
	cuda_kernel_hanning << <cuda_kernel_grid, cuda_kernel_block >> >(dev_window, sn);
}

void cuda_LCMV(int q_front, int s, int sel)
{
	switch (sel)
	{
	case CONFIG_STAP:
		/*cuda_kernel_LCMV_ex << <1, 1 >> >(STAP_col_sig_STAP, STAP_dev_array_anti_out,
			STAP_dev_anti_out_max, STAP_pow_quantization, STAP_dev_array_signal_STAP, STAP_lead_dimension_matrix_R,
			STAP_lead_dimension_matrix_A, STAP_size_matrix_R, STAP_dev_array_matrix_R, STAP_dev_array_matrix_R_inver,
			STAP_dev_array_matrix_R_inver_1col, STAP_PivotArray, STAP_infoArray, q_front, s, STAP_dev_array_anti_out_quan, 
			QUAN_config, cuda_kernel_grid, cuda_kernel_block);*/
		cuda_kernel_LCMV(q_front, s);

		break;
	case CONFIG_BEAM:
		/*cuda_kernel_beam_ex <<<1, 1 >> >(STAP_col_sig_STAP, STAP_dev_array_anti_out,
			STAP_dev_anti_out_max, STAP_pow_quantization, STAP_dev_array_signal_STAP, STAP_lead_dimension_matrix_R,
			STAP_lead_dimension_matrix_A, STAP_size_matrix_R, STAP_dev_array_matrix_R, STAP_dev_array_matrix_R_inver,
			STAP_dev_array_matrix_R_inver_1col, STAP_PivotArray, STAP_infoArray, q_front, s,
			dev_beam_vector_real, dev_beam_vector_image, dev_array_R_inv_v_real, dev_array_R_inv_v_image,
			dev_array_v_real_R_inv, dev_array_v_image_R_inv, dev_beam_scal_real, dev_beam_scal_image, 
			STAP_dev_array_anti_out_quan, QUAN_config, cuda_kernel_grid, cuda_kernel_block);*/
		cuda_kernel_beam(q_front, s);

		break;
	}
	
}