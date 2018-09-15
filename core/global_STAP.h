
#include "global_var.h"
//#include "../gps_swr/Exports.h"
//#include "../bd_swr/BD1_Exports.h"
//#include "../gps_swr/gpsrcvr.h"
//#include "../bd_swr/BD1_gpsrcvr.h"
#include "../receiver/anti_receiver_data.h"

extern int STAP_row_sig_STAP;
extern int STAP_col_sig_STAP;
extern SIGNAL_TYPE *STAP_signal[];
extern SIGNAL_TYPE *STAP_dev_signal[];
extern SIGNAL_TYPE_QUAN *STAP_anti_out_quan;
extern SIGNAL_TYPE *STAP_dev_anti_out;
extern SIGNAL_TYPE_QUAN *STAP_dev_anti_out_quan;
extern SIGNAL_TYPE **STAP_dev_array_anti_out;
extern SIGNAL_TYPE_QUAN **STAP_dev_array_anti_out_quan;
extern SIGNAL_TYPE *STAP_dev_anti_out_max;
extern SIGNAL_TYPE *STAP_signal_STAP[];
//extern SIGNAL_TYPE *STAP_array_signal_STAP[][TEST_CHUNK];
extern SIGNAL_TYPE **STAP_dev_array_signal_STAP;

extern int STAP_lead_dimension_matrix_R;
extern int STAP_lead_dimension_matrix_A;
extern int STAP_size_matrix_R;
extern SIGNAL_TYPE *STAP_array_matrix_R[];
extern SIGNAL_TYPE **STAP_dev_array_matrix_R;
extern SIGNAL_TYPE *STAP_array_matrix_R_inver[];
extern SIGNAL_TYPE **STAP_dev_array_matrix_R_inver;
extern SIGNAL_TYPE **STAP_dev_array_matrix_R_inver_1col;
extern int *STAP_PivotArray;
extern int *STAP_infoArray;

extern FILE *STAP_out_file;

//��������
extern SIGNAL_TYPE STAP_pow_quantization;

//��������
extern SIGNAL_TYPE *beam_vector_real;
extern SIGNAL_TYPE *dev_beam_vector_real;
extern SIGNAL_TYPE *beam_vector_image;
extern SIGNAL_TYPE *dev_beam_vector_image;
extern SIGNAL_TYPE *dev_R_inv_v_real[];
extern SIGNAL_TYPE **dev_array_R_inv_v_real;
extern SIGNAL_TYPE *dev_R_inv_v_image[];
extern SIGNAL_TYPE **dev_array_R_inv_v_image;
extern SIGNAL_TYPE *dev_v_real_R_inv[];
extern SIGNAL_TYPE **dev_array_v_real_R_inv;
extern SIGNAL_TYPE *dev_v_image_R_inv[];
extern SIGNAL_TYPE **dev_array_v_image_R_inv;
extern SIGNAL_TYPE *dev_beam_scal_real;
extern SIGNAL_TYPE *dev_beam_scal_image;


//��ʼ��ȫ�ֱ����� һ�ζ���0.01s���������ݣ� fp:�ļ�ָ�����飬 m:��Ԫ��, sn:���δ������ݵĵ���, s:��Ҫѭ������Ĵ���
//n���ӳٵ�Ԫ����bit������λ��,sel==0ΪSTAP��sel==1Ϊ��������
void init_STAP(int m, int sn, int s, int n, int bit, int sel);

//����ȫ�ֱ���
void destroy_STAP(int m, int sel);

//���ź��ļ�
void STAP_read_file(int m, int sn, int s, int n, int times_read);

//д�ļ�
void STAP_write_file(int total_point, int sel);

/* �����ź�ת���ɿ�ʱ����
���������
signal:�����M�źţ�
m:�����źŵ�����
n:�����źŵ�����
s:ѭ������
CN:��ͷ��=�ӳٵ�Ԫ��+1��
delay:������ͷ֮����ӳ�
*/
void Mat_chg(int m, int n, int s, int CN, int delay, int next_signal, int times_chg);

void STAP_exec(int m, int n, int sn, int bit, int s, int sameple_rate, SIGNAL_TYPE data_process_time, int sel);

void cuda_STAP(int m, int sn, int s, int n, int sel);

//�����γɵ���ʸ������
//EL_si:�źŵĸ�����            
//AZ_sj:�źŵķ�λ��
void beam_s_vector(SIGNAL_TYPE *EL_si, SIGNAL_TYPE *AZ_sj, SIGNAL_TYPE d, int m, int n, char array_s,
	SIGNAL_TYPE sameple, SIGNAL_TYPE IF);