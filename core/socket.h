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
	/*接收机标识*/
	int receiver_id;	//L1:0;BD1:1;BD3:2;GLO:3

	/*anti result*/
	SIMRESVAR var1;	//抗干扰算法收敛时间
	int dim;	//抗干扰算法权值复数数组维数
	SIMRESVAR var2_real[SIMRES_VAR2_SIZE];	//抗干扰算法权值实部
	SIMRESVAR var2_image[SIMRES_VAR2_SIZE];	//抗干扰算法权值虚部
	SIMRESVAR var3;	//信号统计功率

	/*receiver*/
	SIMRESVAR var4[SAT_NUM];	//跟踪通道相关峰值
	SIMRESVAR var5[SAT_NUM];	//跟踪通道载噪比
	int var6[SAT_NUM];	//跟踪通道跟踪锁定指标
	SIMRESVAR var7_az[SAT_NUM];  //方位角
	SIMRESVAR var7_el[SAT_NUM];  //俯仰角
	int var8_prn[SAT_NUM]; //星号（prn）

	//DOP值
	SIMRESVAR var9_GDOP;
	SIMRESVAR var9_HDOP;
	SIMRESVAR var9_VDOP;
	SIMRESVAR var9_TDOP;
	SIMRESVAR var9_PDOP;

	//星位图
	SIMRESVAR var10_x[SAT_NUM];
	SIMRESVAR var10_y[SAT_NUM];
	SIMRESVAR var10_z[SAT_NUM];

	//定位结果
	SIMRESVAR_LF var11_x, var11_y, var11_z;	    //定位结果
	SIMRESVAR var12_lo[3], var12_la[3], var12_he;	//经度纬度高度
	SIMRESVAR var13[3];	                  //测速结果
	SIMRESVAR var14;                      //钟漂

	//时间
	int time_y, time_mon, time_d, time_h, time_min, time_s;                   //时间年月日时分秒
	int var16;                   //周数
	int var17;                   //周内秒

};

typedef struct SimRes_t SimRes;

void init_socket(SOCKET &sclient, sockaddr_in &sin, int &len, int port, string &addr, int self_port);
void des_socket(SOCKET sclient);