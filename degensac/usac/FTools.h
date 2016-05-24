#ifndef FTOOLS_H
#define FTOOLS_H
#include <vector>
#include "MathFunctions.h"

namespace FTools
{
void computeDataMatrix(double* data_matrix, unsigned int num_points, double* points);
int nullspaceQR7x9(const double* A, double* N);
int nullspace(double* matrix, double* nullspace, int n, int* buffer);
void makePolynomial(double* A, double* B, double* p);
unsigned int rroots3 (double* po, double* r);
void formCovMat(double* Cv, const double* A, unsigned int len, unsigned int siz);
void singulF(double* F);
void computeEpipole(double* e, const double* F);
double getOriSign(double* F, double* e, double* pt);
void computeHFromF(const std::vector<unsigned int>& sample, double* u, double* ep, double* F, double* H);
unsigned int getHError(const std::vector<unsigned int>& test, unsigned int numPoints, std::vector<double>& errs,
                       double* u, double* H, double threshold);
unsigned int computeHFromCorrs(const std::vector<unsigned int>& sample, unsigned int numPoints,
                               unsigned int numDataPoints, double* u, double* H);
unsigned int computeHFromMinCorrs(const std::vector<unsigned int>& sample, unsigned int numPoints,
                                  unsigned int numDataPoints, double* u, double* H);
}
#endif
