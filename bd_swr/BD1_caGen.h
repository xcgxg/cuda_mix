// Released on July 9, 2004

#ifndef BD1_CAGEN_H
#define BD1_CAGEN_H

void BD1_g1Gen(char* g1);

void BD1_g2Gen(char* g2);

char** BD1_bdd2_caGen(void);

void BD1_CreateCACodeTable(char svcode[], int prn);

void BD1_freeCaTable(char** caTable);
#endif