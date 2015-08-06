/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/
#ifndef UTLS3X3_HPP
#define UTLS3X3_HPP

double det3x3(double a11,double a12,double a13,double a21,double a22,double a23,double a31,double a32,double a33);
double det3x3(double* A);

int inv_3x3(double a11,double a12,double a13,double a21,double a22,double a23,double a31,double a32,double a33,
            double& b11,double &b12,double &b13,double &b21,double &b22,double &b23,double &b31,double &b32,double &b33);
int inv_3x3(double* A, double *res);

void multiply3x3 (double* L, double* R, double* res);


#endif // UTLS3X3_HPP
