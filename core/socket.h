#pragma once

#include <stdio.h>
#include <stdlib.h>

#include <windows.h>

#include <iostream>

using namespace std;

#define SIMRESVAR float
#define SIMRESVAR_LF double
#define SIMRES_VAR2_SIZE (7 * 10)
#define SAT_NUM (12)

struct SimRes_t
{
	/*���ջ���ʶ*/
	int receiver_id;	//L1:0;BD1:1;BD3:2;GLO:3

	/*anti result*/
	SIMRESVAR var1;	//�������㷨����ʱ��
	int dim;	//�������㷨Ȩֵ��������ά��
	SIMRESVAR var2_real[SIMRES_VAR2_SIZE];	//�������㷨Ȩֵʵ��
	SIMRESVAR var2_image[SIMRES_VAR2_SIZE];	//�������㷨Ȩֵ�鲿
	SIMRESVAR var3;	//�ź�ͳ�ƹ���

	/*receiver*/
	SIMRESVAR var4[SAT_NUM];	//����ͨ����ط�ֵ
	SIMRESVAR var5[SAT_NUM];	//����ͨ�������
	int var6[SAT_NUM];	//����ͨ����������ָ��
	SIMRESVAR var7_az[SAT_NUM];  //��λ��
	SIMRESVAR var7_el[SAT_NUM];  //������
	int var8_prn[SAT_NUM]; //�Ǻţ�prn��

	//DOPֵ
	SIMRESVAR var9_GDOP;
	SIMRESVAR var9_HDOP;
	SIMRESVAR var9_VDOP;
	SIMRESVAR var9_TDOP;
	SIMRESVAR var9_PDOP;

	//��λͼ
	SIMRESVAR var10_x[SAT_NUM];
	SIMRESVAR var10_y[SAT_NUM];
	SIMRESVAR var10_z[SAT_NUM];

	//��λ���
	SIMRESVAR_LF var11_x, var11_y, var11_z;	    //��λ���
	SIMRESVAR var12_lo[3], var12_la[3], var12_he;	//����γ�ȸ߶�
	SIMRESVAR var13[3];	                  //���ٽ��
	SIMRESVAR var14;                      //��Ư

	//ʱ��
	int time_y, time_mon, time_d, time_h, time_min, time_s;                   //ʱ��������ʱ����
	int var16;                   //����
	int var17;                   //������

};

typedef struct SimRes_t SimRes;

void init_socket(SOCKET &sclient, sockaddr_in &sin, int &len, int port, string &addr, int self_port);
void des_socket(SOCKET sclient);