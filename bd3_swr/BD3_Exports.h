#ifndef BD3_EXPORTS_H
#define BD3_EXPORTS_H

#include "BD3_bdd2jxqno.h"
#include <windows.h>

#define BD3_IF_DATA_FILE 0
#define BD3_FILE_PATH    1

BD3_GPSSWR_API void BD3_SetIF(float c);
BD3_GPSSWR_API void BD3_SetFS(double f);
BD3_GPSSWR_API void BD3_SetQB(int q);
BD3_GPSSWR_API void BD3_SetBD(double f);
BD3_GPSSWR_API void BD3_SetFilePath(int type, const char* path);

#define BD3_CHANNEL_WINDOW       0
#define BD3_NAVIGATIONOUT_WINDOW 1
#define BD3_MAIN_WINDOW          2
#define BD3_CNRATIO_WINDOW       3
#define BD3_CP_WINDOW            4
#define BD3_FLL_WINDOW           5

extern "C" BD3_GPSSWR_API void BD3_SetWindow(int type, HWND hwnd);

#endif