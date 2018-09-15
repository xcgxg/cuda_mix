#ifndef BD3_detectCAcode_H
#define BD3_detectCAcode_H
void BD3_detectCACode(BD3_CHANNEL* channel, BD3_SVSTRUCT* svStruct, double* data,
				  long dataLength,char** caTable);


short BD3_SelectOneSv(BD3_SVSTRUCT* svStruct);

void BD3_acquire_ca(BD3_CHANNEL* channel, BD3_SVSTRUCT* svStruct, double* data,
				long dataLength,char** caTable);

#endif