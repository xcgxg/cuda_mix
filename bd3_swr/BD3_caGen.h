// Released on July 9, 2004

#ifndef BD3_CAGEN_H
#define BD3_CAGEN_H

void BD3_g1Gen(char* g1);

void BD3_g2Gen(char* g2);

char** BD3_bdd2_caGen(void);

void BD3_CreateCACodeTable(char svcode[], int prn);

void BD3_freeCaTable(char** caTable);
#endif