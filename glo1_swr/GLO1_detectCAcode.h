#ifndef GLO1_detectCAcode_H
#define GLO1_detectCAcode_H

#include "GLO1_gpsstruct.h"

void GLO1_detectCACode(GLO1_CHANNEL* channel, GLO1_SVSTRUCT* svStruct, double* data,
				  long dataLength,char** caTable);


short GLO1_SelectOneSv(GLO1_SVSTRUCT* svStruct);

void GLO1_acquire_ca(GLO1_CHANNEL* channel, GLO1_SVSTRUCT* svStruct, double* data,
				long dataLength,char** caTable);

#endif