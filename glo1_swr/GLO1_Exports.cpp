#include "../stdafx.h"
#include "GLO1_Exports.h"
#include "../receiver/UI.h"
#include "GLO1_correlatorProcess.h"
#include <string.h>
extern int GLO1_ADNumber;
extern float GLO1_CARRIER_FREQ;
extern double GLO1_SAMPLING_FREQ;

GLO1_GPSSWR_API void GLO1_SetIF(float c)
{
	GLO1_CARRIER_FREQ = c;
}

GLO1_GPSSWR_API void GLO1_SetFS(double f)
{
	GLO1_SAMPLING_FREQ = f;
}

GLO1_GPSSWR_API void GLO1_SetQB(int q)
{
	GLO1_ADNumber = q;
}

GLO1_GPSSWR_API void GLO1_SetBD(double f)
{
	// nothing to do for GPS
}

extern char GLO1_if_data_file[];
extern char GLO1_current_alm_file[];
extern char GLO1_current_eph_file[];
extern char GLO1_ion_utc_file[];
extern char GLO1_rcvr_par_file[];
extern char GLO1_curloc_file[];
extern char GLO1_last_prn_file[];

char GLO1_directory_path[4096];
int GLO1_firsttime = 0;
GLO1_GPSSWR_API void GLO1_SetFilePath(int type, const char* path)
{
	//s-tatic int firsttime = 0;
	switch( type )
	{
	case GLO1_IF_DATA_FILE:
		strcpy(GLO1_if_data_file, path);
		break;
	case GLO1_FILE_PATH:
		{

		strcpy(GLO1_directory_path, path);
		strcat(GLO1_directory_path, "\\glo1config");

		strcpy(GLO1_current_alm_file, GLO1_directory_path);
		strcat(GLO1_current_alm_file, "\\currentgps.alm");

		strcpy(GLO1_current_eph_file, GLO1_directory_path);
		strcat(GLO1_current_eph_file, "\\currentgps.eph");

		strcpy(GLO1_ion_utc_file, GLO1_directory_path);
		strcat(GLO1_ion_utc_file, "\\ion_utcgps.dat");

		strcpy(GLO1_rcvr_par_file, GLO1_directory_path);
		strcat(GLO1_rcvr_par_file, "\\rcvr_pargps.dat");

		strcpy(GLO1_curloc_file, GLO1_directory_path);
		strcat(GLO1_curloc_file, "\\curlocgps.dat");

		strcpy(GLO1_last_prn_file, GLO1_directory_path);
		strcat(GLO1_last_prn_file, "\\lasttimegps.prn");


		}
		break;
	}
}

GLO1_GPSSWR_API void GLO1_SetWindow(int type, HWND hwnd)
{
	switch(type)
	{
	case GLO1_MAIN_WINDOW:
		MainWnd = hwnd;
		break;
	case GLO1_CHANNEL_WINDOW:
		ChannelWnd = hwnd;
		break;
	case GLO1_CNRATIO_WINDOW:
		CNRatioWnd = hwnd;
		break;
	case GLO1_NAVIGATIONOUT_WINDOW:
		NavOutWnd = hwnd;
		break;
	case GLO1_CP_WINDOW:
		CPWnd = hwnd;
		break;
	case GLO1_FLL_WINDOW:
		FLLWnd = hwnd;
		break;
	}
}
