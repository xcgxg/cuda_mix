#ifndef EXPORTS_H
#define EXPORTS_H

#include "GPSSwrCorrWrap.h"
#include <windows.h>

#define IF_DATA_FILE 0
#define FILE_PATH    1

GPSSWR_API void SetFS(double f);
GPSSWR_API void SetIF(float);
GPSSWR_API void SetQB(int q);
GPSSWR_API void SetBD(double f);
GPSSWR_API void SetFilePath(int type, const char* path);

#define CHANNEL_WINDOW       0
#define NAVIGATIONOUT_WINDOW 1
#define MAIN_WINDOW          2
#define CNRATIO_WINDOW       3
#define CP_WINDOW            4
#define FLL_WINDOW           5

extern "C" GPSSWR_API void SetWindow(int type, HWND hwnd);

#endif