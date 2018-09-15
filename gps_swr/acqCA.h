#ifndef ACQ_CA_H
#define ACQ_CA_H

void initSignalSearch();
void freeSignalSearch();

int acqCA(double *data,				/* input data sequence */
			long num,				/* data length */
			char** caTable,			/* CA CODE TABLE */
			short sv,				/* sv number */
			double fs,				/* sampling frequency */
			double *freqOffset,		/* frequency offset array in Hz */
			long numFreqBin,		/* array length */
			double freqCarrier,		/* nomial carrier frequence in Hz */
			double *tau,			/* detected code phase in code chip*/
			double *doppler,		/* detected doppler in Hz */
			long *nTrial,			/* number of trials*/
			double *Rmax,			/* returned correlation peak */
			double *threshold);



#endif