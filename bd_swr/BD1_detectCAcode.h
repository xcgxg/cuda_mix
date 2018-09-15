#ifndef BD1_detectCAcode_H
#define BD1_detectCAcode_H
void BD1_detectCACode(BD1_CHANNEL* channel, BD1_SVSTRUCT* svStruct, double* data,
				  long dataLength,char** caTable);


short BD1_SelectOneSv(BD1_SVSTRUCT* svStruct);

void BD1_acquire_ca(BD1_CHANNEL* channel, BD1_SVSTRUCT* svStruct, double* data,
				long dataLength,char** caTable);

#endif