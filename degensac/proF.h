#ifndef _RANSAC_PROF_H
#define _RANSAC_PROF_H

int inFranio (double *u, int len, int *inliers, int psz, int ninl, double th,
              int **iinls, double *buffer, double *F, int rep, int ** max_inl);

#endif //_RANSAC_PROF_H
