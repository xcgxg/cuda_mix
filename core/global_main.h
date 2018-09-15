#pragma once

#include "global_var.h"
#include "../gps_swr/Exports.h"
#include "../bd_swr/BD1_Exports.h"
#include "../bd3_swr/BD3_Exports.h"
#include "../glo1_swr/GLO1_Exports.h"

#include "../gps_swr/gpsrcvr.h"
#include "../bd_swr/BD1_gpsrcvr.h"
#include "../bd3_swr/BD3_gpsrcvr.h"
#include "../glo1_swr/GLO1_gpsrcvr.h"

void getTimeStamp();

/*sel ²é¿´def.h*/
void global_exec(int sel);

void global_init(int sel);
void global_destory();