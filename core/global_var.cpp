#include "global_var.h"

int sel = 0;

const SIGNAL_TYPE one = 1;
const SIGNAL_TYPE zero = 0;

//执行流
cudaStream_t cuda_stream[NUM_STREAM];

//循环队列
Queue q;

//socket通信相关变量
SOCKET socket_client;
sockaddr_in socket_sin;
int socket_len;
int socket_port = 8888;
int self_port = 9999;
//string socket_addr = "10.129.41.180";
string socket_addr = "127.0.0.1";
struct tm local_time = {};

//定位返回结果
SimRes send_res_L1;
SimRes send_res_BD1;
SimRes send_res_BD3;
SimRes send_res_GLO1;


cublasHandle_t blas_handle;

//信号文件
FILE *fp[NUM_SIGNAL_FILE_HANDLE];

char nowtime[50];

//string filename = "D:\\documents\\signal\\STAP\\0.01s_62MHz_beam_b_float\\signal_%d.txt";
//string filename = "D:\\documents\\signal\\STAP\\180s__62MHz_beam_b_float\\signal_%d.txt";
//string anti_filename = "D:\\documents\\signal\\FDAJ\\1s_62MHz_b_float\\signal_1.txt";
//string anti_filename = "D:\\documents\\signal\\matrix\\BD1_4阵源_宽带_gauss_10MHz_60s_b_float\\signal_%d.txt";   //输入文件名
//string anti_filename = "D:\\documents\\signal\\matrix\\BD1_4阵源_点频_10MHz_60s_b_float\\signal_%d.txt";
//string anti_filename = "D:\\documents\\signal\\matrix\\BD3_danzaibo_46MHz_60s_b_float\\signal_%d.txt";
//string anti_filename = "D:\\documents\\signal\\matrix\\GLO1_20MHz_60s_b_float\\signal_%d.txt";
//string anti_filename = "D:\\documents\\signal\\matrix\\glodanzaibo10_23\\R1Ant%d.dat";
//string anti_filename = "D:\\documents\\signal\\matrix\\glodanzaibo_120s1023\\R1Ant%d.dat";
//string anti_filename = "D:\\documents\\signal\\matrix\\无干扰\\glo1\\R1Ant1_4bit_120s.dat";
//string anti_filename = "E:\\抗干扰数据0629\\4zhenyuan_glo_Y_danzaibo(diuliu)\\R1Ant2.dat";
//string anti_filename = "E:\\抗干扰数据0629\\4zhenyuan_Y_b1_5min_float(duiliudianli)\\B1Ant%d.dat";
//string anti_filename = "E:\\软件接收机\\数据\\L1_点频_10MHz_60s_b_float\\signal_%d.txt";
//string anti_filename = "E:\\软件接收机\\数据\\无干扰导航信号\\glo1\\R1Ant1_1bit.dat";
string anti_filename = "E:\\软件接收机\\数据\\无干扰导航信号\\gps\\test_4bit.txt";
//string anti_filename = "E:\\软件接收机\\数据\\无干扰导航信号\\gps\\test_1bit.txt";
//string anti_filename = "E:\\软件接收机\\数据\\无干扰导航信号\\bd\\B1Ant1_1bit.dat";
//string anti_filename = "D:\\documents\\signal\\matrix\\无干扰+动态b1\\B1Ant1.dat";
//string anti_filename = ".\\signal_1.txt";
//string anti_filename = "D:\\documents\\signal\\matrix\\4阵源点频干扰0921_10MHz_60s_b_float\\signal_%d.txt";
//string anti_filename = "D:\\documents\\signal\\matrix\\无干扰\\gps\\L1Ant_4bit.dat";
//string anti_filename = "D:\\documents\\signal\\matrix\\4阵元1窄带干扰0629_10MHz_60s_b_float\\signal_%d.txt";
//string anti_filename = "D:\\documents\\signal\\matrix\\4阵源_宽带_gauss_10MHz_60s_b_float\\signal_%d.txt";
//string filename = "D:\\documents\\signal\\receiver\\new_receiver\\I_gps_173000_23_10_doub.dat";
string receiver_filepath = "E:\\软件接收机\\数据\\receiver\\new_receiver";
//string receiver_filepath = ".\\new_receiver";

/*输入的信号文件时长（单位：秒）*/
SIGNAL_TYPE data_time_length = 60;

int sameple_rate = 10e6;     //采样率Hz
int bit = 4;        //量化位数
int3 QUAN_config;	//量化参数

//STAP抗干扰参数配置
int m = 4;     //阵元数
int n = 1;     //延迟单元数+1

//beamform参数配置
SIGNAL_TYPE TF = 1575.42e6;	//射频Hz
SIGNAL_TYPE IF = 1.42e6;    //中频Hz
SIGNAL_TYPE signal_lamda =  lv/TF; //波长m
SIGNAL_TYPE EL_si[BEAM_SIG_VECTOR] = { 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 };
SIGNAL_TYPE AZ_sj[BEAM_SIG_VECTOR] = { 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20 };
//SIGNAL_TYPE EL_si[BEAM_SIG_VECTOR] = {};
//SIGNAL_TYPE AZ_sj[BEAM_SIG_VECTOR] = {};
char array_antenna = 'Y';   //阵型选择
SIGNAL_TYPE array_posx[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
SIGNAL_TYPE array_posy[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
SIGNAL_TYPE array_posz[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//FDAJ抗干扰参数配置：
int antenna = 1;             //阵元个数
//2、单天线频域滤波参数
int threshold = 4;   //门限系数
int loop = 3;        //迭代次数

int cuda_kernel_grid;
int cuda_kernel_block;

void set_file_pos(int start_sec, int length_sec)
{
	data_time_length = length_sec;

	int offset;
	
	switch (sel & ANTI_CONFIG_SEL)
	{
	case CONFIG_ANTI_NONE:
		offset = start_sec * sameple_rate * sizeof(SIGNAL_TYPE_QUAN);

		break;
	default:
		offset = start_sec * sameple_rate * sizeof(SIGNAL_TYPE);

		break;
	}

	int res=0;
	for (int  i = 0; i < m; i++)
	{
		res = fseek(fp[i], offset, SEEK_SET);
		if (-1 == res)
		{
			perror("set file position error");
		}
	}
}