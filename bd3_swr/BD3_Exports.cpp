#include "../stdafx.h"
#include "BD3_Exports.h"
#include "../receiver/UI.h"
#include "BD3_correlatorProcess.h"
#include <string.h>
extern int BD3_ADNumber;
extern float BD3_CARRIER_FREQ;
extern double BD3_SAMPLING_FREQ;

BD3_GPSSWR_API void BD3_SetIF(float c)
{
	BD3_CARRIER_FREQ = c;
}

BD3_GPSSWR_API void BD3_SetFS(double f)
{
	BD3_SAMPLING_FREQ = f;
}

BD3_GPSSWR_API void BD3_SetQB(int q)
{
	BD3_ADNumber = q;
}

BD3_GPSSWR_API void BD3_SetBD(double f)
{

}

extern char BD3_if_data_file[];
extern char BD3_current_alm_file[];
extern char BD3_current_eph_file[];
extern char BD3_ion_utc_file[];
extern char BD3_rcvr_par_file[];
extern char BD3_curloc_file[];
extern char BD3_last_prn_file[];
char BD3_directory_path[4096];

BD3_GPSSWR_API void BD3_SetFilePath(int type, const char* path)
{
	switch( type )
	{
	case BD3_IF_DATA_FILE:
		strcpy(BD3_if_data_file, path);
		break;
	case BD3_FILE_PATH:
		{
		strcpy(BD3_directory_path, path);
		strcat(BD3_directory_path, "\\bd3config");

		strcpy(BD3_current_alm_file, BD3_directory_path);
		strcat(BD3_current_alm_file, "\\currentbd.alm");

		strcpy(BD3_current_eph_file, BD3_directory_path);
		strcat(BD3_current_eph_file, "\\currentbd.eph");

		strcpy(BD3_ion_utc_file, BD3_directory_path);
		strcat(BD3_ion_utc_file, "\\ion_utcbd.dat");

		strcpy(BD3_rcvr_par_file, BD3_directory_path);
		strcat(BD3_rcvr_par_file, "\\rcvr_parbd.dat");

		strcpy(BD3_curloc_file, BD3_directory_path);
		strcat(BD3_curloc_file, "\\curlocbd.dat");

		strcpy(BD3_last_prn_file, BD3_directory_path);
		strcat(BD3_last_prn_file, "\\lasttimebd.prn");
		}
		break;
	}
}

BD3_GPSSWR_API void BD3_SetWindow(int type, HWND hwnd)
{
	switch(type)
	{
	case BD3_MAIN_WINDOW:
		MainWnd = hwnd;
		break;
	case BD3_CHANNEL_WINDOW:
		ChannelWnd = hwnd;
		break;
	case BD3_CNRATIO_WINDOW:
		CNRatioWnd = hwnd;
		break;
	case BD3_NAVIGATIONOUT_WINDOW:
		NavOutWnd = hwnd;
		break;
	case BD3_CP_WINDOW:
		CPWnd = hwnd;
		break;
	case BD3_FLL_WINDOW:
		FLLWnd = hwnd;
		break;
	}
}
