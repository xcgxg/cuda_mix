#ifndef detectCAcode_H
#define detectCAcode_H
void detectCACode(CHANNEL* channel, SVSTRUCT* svStruct, double* data, 
				  long dataLength,char** caTable);


short SelectOneSv(SVSTRUCT* svStruct);

void acquire_ca(CHANNEL* channel, SVSTRUCT* svStruct, double* data,
				long dataLength,char** caTable);

#endif