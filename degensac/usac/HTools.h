#ifndef HTOOLS_H
#define HTOOLS_H

#include "MathFunctions.h"

namespace HTools
{
void computeDataMatrix(double* data_matrix, unsigned int num_points, double* points);
//dupicity with MathTools void crossprod(double *out, const double *a, const double *b, unsigned int st);
}
#endif

