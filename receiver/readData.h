// Released on July 9, 2004

#ifndef READ_DATA_FILE_H
#define READ_DATA_FILE_H

void readFileHeader(FILE* fp);

long readData(char* data, long num, FILE* fp);
#endif