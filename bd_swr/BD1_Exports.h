#ifndef BD1_EXPORTS_H
#define BD1_EXPORTS_H

#include "BD1_bdd2jxqno.h"
#include <windows.h>

#define IF_DATA_FILE 0
#define FILE_PATH    1

BD1_GPSSWR_API void BD1_SetFS(double f);
BD1_GPSSWR_API void BD1_SetIF(float);
BD1_GPSSWR_API void BD1_SetQB(int q);
BD1_GPSSWR_API void BD1_SetBD(double f);
BD1_GPSSWR_API void BD1_SetFilePath(int type, const char* path);

#define CHANNEL_WINDOW       0
#define NAVIGATIONOUT_WINDOW 1
#define MAIN_WINDOW          2
#define CNRATIO_WINDOW       3
#define CP_WINDOW            4
#define FLL_WINDOW           5

extern "C" BD1_GPSSWR_API void BD1_SetWindow(int type, HWND hwnd);

#endif