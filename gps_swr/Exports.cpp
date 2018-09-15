#include "../stdafx.h"
#include "Exports.h"
#include "../receiver/UI.h"
#include "correlatorProcess.h"
#include <string.h>
extern int ADNumber;
extern float CARRIER_FREQ;
extern double SAMPLING_FREQ;

GPSSWR_API void SetIF(float c)
{
	CARRIER_FREQ = c;
}

GPSSWR_API void SetFS(double f)
{
	SAMPLING_FREQ = f;
}

GPSSWR_API void SetQB(int q)
{
	ADNumber = q;
}

GPSSWR_API void SetBD(double f)
{
	// nothing to do for GPS
}

extern char if_data_file[];      
extern char current_alm_file[]; 
extern char current_eph_file[];   
extern char ion_utc_file[];       
extern char rcvr_par_file[];      
extern char curloc_file[];      
extern char last_prn_file[];     

char directory_path[4096];
int firsttime = 0;
GPSSWR_API void SetFilePath(int type, const char* path)
{
	//st-atic int firsttime = 0;
	switch( type )
	{
	case IF_DATA_FILE:
		strcpy(if_data_file, path);
		break;
	case FILE_PATH:
		{

			strcpy(directory_path, path);
			strcat(directory_path, "\\gpsconfig");

			strcpy(current_alm_file, directory_path);
			strcat(current_alm_file, "\\currentgps.alm");

			strcpy(current_eph_file, directory_path);
			strcat(current_eph_file, "\\currentgps.eph");

			strcpy(ion_utc_file, directory_path);
			strcat(ion_utc_file, "\\ion_utcgps.dat");

			strcpy(rcvr_par_file, directory_path);
			strcat(rcvr_par_file, "\\rcvr_pargps.dat");

			strcpy(curloc_file, directory_path);
			strcat(curloc_file, "\\curlocgps.dat");

			strcpy(last_prn_file, directory_path);
			strcat(last_prn_file, "\\lasttimegps.prn");


		}
		break;
	}
}

GPSSWR_API void SetWindow(int type, HWND hwnd)
{
	switch(type)
	{
	case MAIN_WINDOW:
		MainWnd = hwnd;
		break;
	case CHANNEL_WINDOW:
		ChannelWnd = hwnd;
		break;
	case CNRATIO_WINDOW:
		CNRatioWnd = hwnd;
		break;
	case NAVIGATIONOUT_WINDOW:
		NavOutWnd = hwnd;
		break;
	case CP_WINDOW:
		CPWnd = hwnd;
		break;
	case FLL_WINDOW:
		FLLWnd = hwnd;
		break;
	}
}
