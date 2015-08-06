#ifndef _RANSAC_USAC_H
#define _RANSAC_USAC_H
#include "usac/ConfigParams.h"

int usac (double *u, unsigned int len, ConfigParams cfg, double *M, unsigned char * inl, unsigned int * stats);
#endif

