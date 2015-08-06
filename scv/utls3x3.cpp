/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/

#include "utls3x3.hpp"

double det3x3(double a11,double a12,double a13,double a21,double a22,double a23,double a31,double a32,double a33)
{
    double d = a11 *(a22*a33-a23*a32);
    d -= a12*(a21*a33-a23*a31);
    d += a13*(a21*a32-a22*a31);
    return d;
}
double det3x3(double *A)
{
    double d = A[0] *(A[4]*A[8]-A[5]*A[7]);
    d -= A[1]*(A[3]*A[8]-A[5]*A[6]);
    d += A[2]*(A[3]*A[7]-A[4]*A[6]);
    return d;
}

int inv_3x3(double a11,double a12,double a13,double a21,double a22,double a23,double a31,double a32,double a33,
            double& b11,double &b12,double &b13,double &b21,double &b22,double &b23,double &b31,double &b32,double &b33)
{
    double det = det3x3(a11,a12,a13,a21,a22,a23,a31,a32,a33);
    if (det == 0) return 0; //inverse matrix does not exist
    b11 = (a22*a33-a32*a23)/det;
    b21 = (a23*a31-a21*a33)/det;
    b31 = (a21*a32-a22*a31)/det;

    b12 = (a13*a32-a12*a33)/det;
    b22 = (a11*a33-a13*a31)/det;
    b32 = (a12*a31-a11*a32)/det;

    b13 = (a12*a23-a13*a22)/det;
    b23 = (a13*a21-a11*a23)/det;
    b33 = (a11*a22-a12*a21)/det;
    return 1;
}
int inv_3x3(double* A, double *res)
{
    double det = det3x3(A);
    if (det == 0) return 0; //inverse matrix does not exist
    res[0] = (A[4]*A[8]-A[7]*A[5])/det;
    res[3] = (A[5]*A[6]-A[3]*A[8])/det;
    res[6] = (A[3]*A[7]-A[4]*A[6])/det;

    res[1] = (A[2]*A[7]-A[1]*A[8])/det;
    res[4] = (A[0]*A[8]-A[2]*A[6])/det;
    res[7] = (A[1]*A[6]-A[0]*A[7])/det;

    res[2] = (A[1]*A[5]-A[2]*A[4])/det;
    res[5] = (A[2]*A[3]-A[0]*A[5])/det;
    res[8] = (A[0]*A[4]-A[1]*A[3])/det;

    return 1;
}
void multiply3x3 (double* L, double* R, double* res)
{
    res[0] = L[0]*R[0]+L[1]*R[3]+L[2]*R[6];
    res[1] = L[0]*R[1]+L[1]*R[4]+L[2]*R[7];
    res[2] = L[0]*R[2]+L[1]*R[5]+L[2]*R[8];

    res[3] = L[3]*R[0]+L[4]*R[3]+L[5]*R[6];
    res[4] = L[3]*R[1]+L[4]*R[4]+L[5]*R[7];
    res[5] = L[3]*R[2]+L[4]*R[5]+L[5]*R[8];

    res[6] = L[6]*R[0]+L[7]*R[3]+L[8]*R[6];
    res[7] = L[6]*R[1]+L[7]*R[4]+L[8]*R[7];
    res[8] = L[6]*R[2]+L[7]*R[5]+L[8]*R[8];
}
