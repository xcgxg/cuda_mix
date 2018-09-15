#include "../stdafx.h"
#include "UI.h"

// UI Components
HWND ChannelWnd;
HWND NavOutWnd;
HWND CNRatioWnd;
HWND MainWnd;
HWND CPWnd;
HWND FLLWnd;

NavOutData navout;
double CNR[12];     
double CP[12][15];     
double FLLOut[12];    
ChannelInfo chinfo[12]; 
CorrPeak corrpeak[15];
