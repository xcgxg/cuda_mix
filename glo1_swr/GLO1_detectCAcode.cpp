#include "../stdafx.h"
#include <math.h>
#include <stdlib.h>
#include "GLO1_caCode.h"
#include "GLO1_CAGen.h"
#include "GLO1_acqca.h"
#include "GLO1_gpsstruct.h"
#include "GLO1_gpsconst.h"
#include "GLO1_detectCAcode.h"

extern GLO1_CHANNEL	GLO1_chan[GLO1_chmax + 1];
extern GLO1_SVSTRUCT	GLO1_svStruct[GLO1_MaxSvNumber + 1];