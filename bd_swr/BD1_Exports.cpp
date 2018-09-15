#include "../StdAfx.h"
#include "BD1_Exports.h"
#include "../receiver/UI.h"
#include "BD1_correlatorProcess.h"
#include <string.h>
extern int BD1_ADNumber;
extern float BD1_CARRIER_FREQ;
extern double BD1_SAMPLING_FREQ;

BD1_GPSSWR_API void BD1_SetIF(float c)
{
	BD1_CARRIER_FREQ = c;
}

BD1_GPSSWR_API void BD1_SetFS(double f)
{
	BD1_SAMPLING_FREQ = f;
}

BD1_GPSSWR_API void BD1_SetQB(int q)
{
	BD1_ADNumber = q;
}

BD1_GPSSWR_API void BD1_SetBD(double f)
{

}

extern char BD1_if_data_file[];
extern char BD1_current_alm_file[];
extern char BD1_current_eph_file[];
extern char BD1_ion_utc_file[];
extern char BD1_rcvr_par_file[];
extern char BD1_curloc_file[];
extern char BD1_last_prn_file[];
char BD1_directory_path[4096];

BD1_GPSSWR_API void BD1_SetFilePath(int type, const char* path)
{
	switch( type )
	{
	case IF_DATA_FILE:
		strcpy(BD1_if_data_file, path);
		break;
	case FILE_PATH:
		{
			strcpy(BD1_directory_path, path);
			strcat(BD1_directory_path, "\\bdconfig");

			strcpy(BD1_current_alm_file, BD1_directory_path);
			strcat(BD1_current_alm_file, "\\currentbd.alm");

			strcpy(BD1_current_eph_file, BD1_directory_path);
			strcat(BD1_current_eph_file, "\\currentbd.eph");

			strcpy(BD1_ion_utc_file, BD1_directory_path);
			strcat(BD1_ion_utc_file, "\\ion_utcbd.dat");

			strcpy(BD1_rcvr_par_file, BD1_directory_path);
			strcat(BD1_rcvr_par_file, "\\rcvr_parbd.dat");

			strcpy(BD1_curloc_file, BD1_directory_path);
			strcat(BD1_curloc_file, "\\curlocbd.dat");

			strcpy(BD1_last_prn_file, BD1_directory_path);
			strcat(BD1_last_prn_file, "\\lasttimebd.prn");
		}
		break;
	}
}

BD1_GPSSWR_API void BD1_SetWindow(int type, HWND hwnd)
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
