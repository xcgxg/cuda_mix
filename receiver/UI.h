#ifndef UI_H
#define UI_H

#include <windows.h>

typedef struct tagNavOutData
{
	char lat[32]; //纬度
	char lon[32]; //精度
	double height;//高度
	double X;     //ECEF-X
	double Y;     //ECEF-Y
	double Z;     //ECEF-Z
	double gdop;
	double hdop;
	double vdop;
	double tdop;
	char time[64];
	// 07.8.17
	double ve;
	double vn;
	double vu;
	double ae;
	double an;
	double au;
} NavOutData;

typedef struct tagChannelInfo
{
	int prn;
	int state;
	double az;
	double el;
	double doppler;
	int t_count;
	int n_frames;
	int sfid;
	double ura;
	int page;
	double CN0;
} ChannelInfo;

typedef struct tagCorrPeak
{
	double i_early34;
	double i_early;	
	double i_early14;
	double i_prompt;
	double i_late14;
	double i_late;
	double i_late34;
} CorrPeak;

extern HWND ChannelWnd;
extern HWND NavOutWnd;
extern HWND CNRatioWnd;
extern HWND MainWnd;
extern HWND CPWnd;
extern HWND FLLWnd;

extern NavOutData navout;
extern ChannelInfo chinfo[];
extern CorrPeak corrpeak[];
extern double CNR[];
extern double CP[][15];
extern double FLLOut[];

#endif