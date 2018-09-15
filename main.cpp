// classdesign2015.cpp : 定义控制台应用程序的入口点。
//


#include "stdafx.h"
#include "core/global_main.h"

#include <iostream>

using namespace std;

extern bool bFirst;

int main()
{
	//19s
	// sel = CONFIG_STAP | WITH_GPSL1_RECEIVER;// | USING_KALMAN_POS_VEL;
	// sel = CONFIG_STAP | WITH_BD1_RECEIVER;
	// sel = CONFIG_STAP | WITH_BD3_RECEIVER ;
	// sel = CONFIG_STAP | WITH_GLO1_RECEIVER;
	// sel = CONFIG_STAP | WRITE_ANTI_RESULT_TXT;
	// sel = CONFIG_STAP | WRITE_ANTI_RESULT_BINARY_CHAR;

	// sel = CONFIG_BEAM | WITH_GPSL1_RECEIVER;
	// sel = CONFIG_BEAM | WITH_BD1_RECEIVER;
	// sel = CONFIG_BEAM | WITH_BD3_RECEIVER;
	// sel = CONFIG_BEAM | WITH_GLO1_RECEIVER;
	// sel = CONFIG_BEAM | WRITE_ANTI_RESULT_TXT;
	// sel = CONFIG_BEAM | WRITE_ANTI_RESULT_BINARY_CHAR;

	//21s
	// sel = CONFIG_FDAJ | WITH_GPSL1_RECEIVER;
	// sel = CONFIG_FDAJ | WITH_BD1_RECEIVER;
	// sel = CONFIG_FDAJ | WITH_BD3_RECEIVER | USING_KALMAN_POS_VEL;
	// sel = CONFIG_FDAJ | WITH_GLO1_RECEIVER;
	// sel = CONFIG_FDAJ | WRITE_ANTI_RESULT_TXT;
	// sel = CONFIG_FDAJ | WRITE_ANTI_RESULT_BINARY_CHAR;

	//27s
	// sel = CONFIG_LMS | WITH_GPSL1_RECEIVER;
	// sel = CONFIG_LMS | WITH_BD1_RECEIVER;
	// sel = CONFIG_LMS | WITH_BD3_RECEIVER;
	// sel = CONFIG_LMS | WITH_GLO1_RECEIVER;
	// sel = CONFIG_LMS | WRITE_ANTI_RESULT_TXT;
	// sel = CONFIG_LMS | WRITE_ANTI_RESULT_BINARY_CHAR;

	sel = WITH_GPSL1_RECEIVER;// | USING_KALMAN_POS_VEL | USING_SOCKET_DATA;// USING_LOCAL_DATA;
	// sel = WITH_BD1_RECEIVER;
	// sel = WITH_BD3_RECEIVER;
	// sel = WITH_GLO1_RECEIVER;


	global_init(sel);

	global_exec(sel);

	printf("cudaGetLastError = %d\n", cudaGetLastError());
	printf("GetLastError = %d\n", GetLastError());

	bFirst = true;
	global_init(sel);

	global_exec(sel);

	printf("cudaGetLastError = %d\n", cudaGetLastError());
	printf("GetLastError = %d\n", GetLastError());




	//cin >> sel;

	return 0;
}