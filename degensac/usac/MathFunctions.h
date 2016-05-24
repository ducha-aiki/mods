#ifndef MATHFUNCTIONS_H
#define MATHFUNCTIONS_H

#include <stdlib.h>
#include <math.h>

namespace MathTools
{
typedef int integer;
typedef double doublereal;

// extern "C" {
// int dgeqp3_(integer const *m,
// 			integer const *n,
// 			doublereal *a,
// 			integer const *lda,
// 			integer *jpvt,
// 			doublereal *tau,
// 			doublereal *work,
// 			integer *lwork,
// 			integer *info);
// }

// the below functions are from ccmath
int svdu1v(double *d, double *a, int m, double *v, int n);
int svduv(double *d, double *a, double *u, int m, double *v, int n);
void ldumat(double *x, double *y, int i, int k);
void ldvmat(double *x, double *y, int k);
void atou1(double *r, int i, int j);
int qrbdu1(double *w, double *x, double *y, int k, double *z, int l);
int qrbdv(double *x, double *y, double *z, int i, double *w, int j);
int minv(double *a,int n);
void vmul(double *vp,double *mat,double *v,int n);
void mattr(double *a,double *b,int m,int n) ;
void skew_sym(double *A, double *a);
void mmul(double *c,double *a,double *b,int n);
void mt_crossprod(double *out, const double *a, const double *b, unsigned int st);
void trnm(double *a,int n);
}
#endif
