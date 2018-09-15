#ifndef GLO1_EXPORTS_H
#define GLO1_EXPORTS_H

#include "GLO1_GPSSwrCorrWrap.h"
#include <windows.h>

#define GLO1_IF_DATA_FILE 0
#define GLO1_FILE_PATH    1

GLO1_GPSSWR_API void GLO1_SetIF(float c);
GLO1_GPSSWR_API void GLO1_SetFS(double f);
GLO1_GPSSWR_API void GLO1_SetQB(int q);
GLO1_GPSSWR_API void GLO1_SetBD(double f);
GLO1_GPSSWR_API void GLO1_SetFilePath(int type, const char* path);

#define GLO1_CHANNEL_WINDOW       0
#define GLO1_NAVIGATIONOUT_WINDOW 1
#define GLO1_MAIN_WINDOW          2
#define GLO1_CNRATIO_WINDOW       3
#define GLO1_CP_WINDOW            4
#define GLO1_FLL_WINDOW           5

extern "C" GLO1_GPSSWR_API void GLO1_SetWindow(int type, HWND hwnd);

#endif