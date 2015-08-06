#include "FTools.h"
#include <iostream>
#include <cstring>

using std::ptrdiff_t; //HACK - support of older lapack libs in newer compilers
#include <lapack.h>
#include <lapack_compat.h>

namespace FTools
{

void computeDataMatrix(double* data_matrix, unsigned int num_points, double* points)
{
    // linearizes corresp. with respect to entries of fundamental matrix
    // so that x' F x -> A f
    const double *data_ptr;
    double *matrix_ptr = data_matrix;
    unsigned int offset;

    for (unsigned int i = 0; i < num_points; ++i)
    {
        data_ptr = points + 6*i;
        offset = 0;
        for (unsigned int j = 0; j < 3; ++j)
        {
            for (unsigned int k = 0; k < 3; ++k)
            {
                *(matrix_ptr+offset) = *(data_ptr+j+3) * (*(data_ptr+k));
                offset += num_points;
            }
        }
        ++matrix_ptr;
    }
} // end computeDataMatrix

int nullspace(double* matrix, double* nullspace, int n, int* buffer)
{
    int *pnopivot = buffer, nonpivot = 0;
    int *ppivot = buffer + n;
    int i, j, k, l, ptr, max;
    double pivot, t;
    double tol = 1e-12;

    ptr = 0;
    i = 0;
    for (j = 0; j < n; ++j)
    {
        /* find pivot, start with diagonal element */
        pivot = matrix[n*i+j];
        max = i;
        for (k=i+1; k<n; k++)
        {
            t = fabs(matrix[n*k+j]);
            if (pivot<t)
            {
                pivot=t;
                max=k;
            }
        }
        if (pivot<tol)
        {
            *(pnopivot++) = j;
            nonpivot++;
            /* negligible column, zero out */
            for (k=i; k<n; k++) matrix[n*k+j]=0;
        }
        else
        {
            *(ppivot++) = j;
            /* swap rows i <-> max */
            for (k=j; k<n; k++)
            {
                t = matrix[i*n+k];
                matrix[i*n+k] = matrix[max*n+k];
                matrix[max*n+k]=t;
            }
            pivot = matrix[i*n+j];
            /* divide the pivot row by the pivot element. */
            for (k=j; k<n; k++)
                matrix[i*n+k] /= pivot;

            /* Subtract multiples of the pivot row from all the other rows. */
            for (k=0; k<i; k++)
            {
                pivot = -matrix[k*n+j];
                for (l=j; l<n; l++)
                    matrix[k*n+l] += pivot*matrix[i*n+l];
            }

            for (k=i+1; k<n; k++)
            {
                pivot = matrix[k*n+j];
                for (l=j; l<n; l++)
                    matrix[k*n+l] -= pivot*matrix[i*n+l];
            }
            i++;
        }
    }

    /* initialize null space vectors */
    for (k=0; k<nonpivot; k++)
    {
        j=buffer[k];
        /* copy nonpivot -column above diagonal */
        for (l=0; l<n-nonpivot; l++)
            nullspace[k*n+buffer[n+l]]=-matrix[l*n+j];

        for (l=0; l<nonpivot; l++)
            nullspace[k*n+buffer[l]]=(j==buffer[l])?1:0;
    }
    /* number of nullspace vectors */
    return nonpivot;
} // end nullspace


int nullspaceQR7x9(const double *A, double *N)
{
    const lapack_int rows=7;
    const lapack_int cols=9;
    int i,j;
    // allocate workspaces
    // change row->column organization for Fortran
#ifndef _MSC_VER
    double T[rows*cols];
    double tau[cols];
    double work[3*cols+1];
    lapack_int p[cols];
#else
    double T[7*9];
    double tau[9];
    double work[3*9+1];
    lapack_int p[9];
#endif

    lapack_int work_size = 3*cols+1;
    lapack_int info;
    // assume underdetermined system with full possible rank...
    int null_size = cols - rows;
    lapack_int k,r,c;
    double *sol = N;
    double a;

    for (i=0; i<rows; i++)
        for (j=0; j<cols; j++)
            T[i + rows*j] = A[cols*i + j];

    // prepare permutation vector
    for (j=0; j<cols; j++) p[j] = 0;

    r = rows;
    c = cols;
    // call Fortran LAPACK function
    dgeqp3(&r, &c, T, &r, p, tau, work, &work_size, &info);
    if (info!=0)
        return -1;

    // correct permutation offset
    for (j=0; j<cols; j++)
        p[j]--;

    // do backsubstitution, resulting T is column organized rows x cols
    // matrix, only elements on and above diagonal are valid and permuted
    // with permutation in p
    for (k=1; k<=null_size; k++)
    {
        // setup arbitrary part of solution vector
        for (c=rows; c<cols; c++) sol[p[c]]=0;
        sol[p[cols-k]]=1;

        // do backsubstitution
        for (r=rows-1; r>=0; r--)
        {
            a=0;
            if (T[r*rows+r]==0.0)
                return -1;
            for (c=r+1; c<cols; c++)
                a += T[c*rows+r]*sol[p[c]];
            // newvalue = -a/diagonal element
            sol[p[r]]=-a/T[r*rows+r];
        }
        sol+=cols;
    }
    return 0;
}

void makePolynomial(double* A, double* B, double* p)
{
    // calculates polynomial p in x, so that det(xA + (1-x)B) = 0
    // where A,B are [3][3] and p is [4] arrays
    // ** CHANGES B to A-B ***
    // so finally det(A + (x-1) B) = 0

    *p = -((*(B+2))*(*(B+4))*(*(B+6))) + (*(B+1))*(*(B+5))*(*(B+6)) + (*(B+2))*(*(B+3))*(*(B+7)) -
         (*B)*(*(B+5))*(*(B+7)) - (*(B+1))*(*(B+3))*(*(B+8)) + (*B)*(*(B+4))*(*(B+8));

    *(p+1) = -((*(A+8))*(*(B+1))*(*(B+3))) + (*(A+7))*(*(B+2))*(*(B+3)) + (*(A+8))*(*B)*(*(B+4)) -
             (*(A+6))*(*(B+2))*(*(B+4)) - (*(A+7))*(*B)*(*(B+5)) + (*(A+6))*(*(B+1))*(*(B+5)) +
             (*(A+5))*(*(B+1))*(*(B+6)) - (*(A+4))*(*(B+2))*(*(B+6)) - (*(A+2))*(*(B+4))*(*(B+6)) +
             3*(*(B+2))*(*(B+4))*(*(B+6)) + (*(A+1))*(*(B+5))*(*(B+6)) - 3*(*(B+1))*(*(B+5))*(*(B+6)) -
             (*(A+5))*(*B)*(*(B+7)) + (*(A+3))*(*(B+2))*(*(B+7)) + (*(A+2))*(*(B+3))*(*(B+7)) -
             3*(*(B+2))*(*(B+3))*(*(B+7)) - (*A)*(*(B+5))*(*(B+7)) + 3*(*B)*(*(B+5))*(*(B+7)) +
             ((*(A+4))*(*B) - (*(A+3))*(*(B+1)) - (*(A+1))*(*(B+3)) + 3*(*(B+1))*(*(B+3)) + (*A)*(*(B+4)) -
              3*(*B)*(*(B+4)))*(*(B+8));

    *(p+2) = -((*(A+3))*(*(A+8))*(*(B+1))) + (*(A+3))*(*(A+7))*(*(B+2)) +
             (*(A+2))*(*(A+7))*(*(B+3)) - (*(A+1))*(*(A+8))*(*(B+3)) + 2*(*(A+8))*(*(B+1))*(*(B+3)) -
             2*(*(A+7))*(*(B+2))*(*(B+3)) - (*(A+2))*(*(A+6))*(*(B+4)) + (*A)*(*(A+8))*(*(B+4)) -
             2*(*(A+8))*(*B)*(*(B+4)) + 2*(*(A+6))*(*(B+2))*(*(B+4)) + (*(A+1))*(*(A+6))*(*(B+5)) -
             (*A)*(*(A+7))*(*(B+5)) + 2*(*(A+7))*(*B)*(*(B+5)) - 2*(*(A+6))*(*(B+1))*(*(B+5)) +
             2*(*(A+2))*(*(B+4))*(*(B+6)) - 3*(*(B+2))*(*(B+4))*(*(B+6)) - 2*(*(A+1))*(*(B+5))*(*(B+6)) +
             3*(*(B+1))*(*(B+5))*(*(B+6)) + (*(A+2))*(*(A+3))*(*(B+7)) - 2*(*(A+3))*(*(B+2))*(*(B+7)) -
             2*(*(A+2))*(*(B+3))*(*(B+7)) + 3*(*(B+2))*(*(B+3))*(*(B+7)) + 2*(*A)*(*(B+5))*(*(B+7)) -
             3*(*B)*(*(B+5))*(*(B+7)) + (*(A+5))*
             (-((*(A+7))*(*B)) + (*(A+6))*(*(B+1)) + (*(A+1))*(*(B+6)) - 2*(*(B+1))*(*(B+6)) -
              (*A)*(*(B+7)) + 2*(*B)*(*(B+7))) +
             (-((*(A+1))*(*(A+3))) + 2*(*(A+3))*(*(B+1)) + 2*(*(A+1))*(*(B+3)) - 3*(*(B+1))*(*(B+3)) -
              2*(*A)*(*(B+4)) + 3*(*B)*(*(B+4)))*(*(B+8)) +
             (*(A+4))*((*(A+8))*(*B) - (*(A+6))*(*(B+2)) - (*(A+2))*(*(B+6)) + 2*(*(B+2))*(*(B+6)) +
                       (*A)*(*(B+8)) - 2*(*B)*(*(B+8)));

    for (unsigned int i=0; i < 9; ++i)
    {
        B[i] = A[i] - B[i];
    }

    *(p+3) =-((*(B+2))*(*(B+4))*(*(B+6))) + (*(B+1))*(*(B+5))*(*(B+6)) + (*(B+2))*(*(B+3))*(*(B+7)) -
            (*B)*(*(B+5))*(*(B+7)) - (*(B+1))*(*(B+3))*(*(B+8)) + (*B)*(*(B+4))*(*(B+8));
} // end makePolynomial

unsigned int rroots3(double* po, double* r)
{
    // real roots of the polynomial of degree 3
    double b,c, b2, bt, v, pit, e;
    double p, q, D, A, cosphi, phit, R, _2R;
    b = *(po + 1) / (*po);
    c = *(po + 2) / (*po);
    b2 = b*b;
    bt = b/3;

    p = (3*c - b2)/ 9;
    q = ((2 * b2 * b)/27 - b*c/3 + (*(po + 3))/(*po)) / 2;

    D = q*q + p*p*p;

    if (D > 0)
    {
        A = sqrt(D) - q;
        if (A > 0)
        {
            v = pow(A,1.0/3);
            *r = v - p/v - bt;
        }
        else
        {
            v = pow(-A,1.0/3);
            *r = p/v - v - bt;
        }

        return 1;
    }
    else
    {
        if (q > 0) e = 1;
        else e = -1;
        R = e * sqrt(-p);
        _2R = R *2;
        cosphi = q / (R*R*R);
        if (cosphi > 1) cosphi = 1;
        else if (cosphi < -1) cosphi = -1;
        phit = acos(cosphi) /3;
        pit = 3.14159265358979/3;

        r[0] = -_2R * cos(phit) -bt;
        r[1] =  _2R * cos(pit - phit) -bt;
        r[2] =  _2R * cos(pit + phit) -bt;

        return 3;
    }
} // end rroots3

void formCovMat(double* Cv, const double* Z, unsigned int len, unsigned int siz)
{
    unsigned int lenM = len*siz;
    double val;

    for (unsigned int i = 0; i < siz; ++i)
    {
        for (unsigned int j = 0; j <= i; ++j)
        {
            val = 0;
            for (unsigned int k = 0; k < lenM; k += siz)
            {
                val += Z[k+i] * Z[k+j];
            }
            Cv[siz*i + j] = val;
            Cv[i + siz*j] = val;
        }
    }
} // end formCovMat

void singulF(double *F)
{
    double U[9], D[3], V[9];
    int i, j, k=0, l=0;
    MathTools::svduv(D, F, V, 3, U, 3);

    j = 0;
    for (i = 1; i < 3; i ++)
        if (fabs(D[j]) > fabs(D[i])) j = i;

    switch (j)
    {
    case 0:
        k = 1;
        l = 2;
        break;
    case 1:
        k = 0;
        l = 2;
        break;
    case 2:
        k = 0;
        l = 1;
        break;
    }

    for (i = 0; i < 9; i+=3)
    {
        V[i+k] *= D[k];
        V[i+l] *= D[l];
    }

    for (j = 0; j < 9; j+=3)
        for (i = 0; i < 9; i+=3, F++)
            *F = U[i+k] * V[j+k] + U[i+l] * V[j+l];
} // end singulF

void computeEpipole(double* epi, const double* F)
{
    double xeps = 1.9984e-15;
    MathTools::mt_crossprod(epi, F, F+6, 1);
    for(unsigned int i = 0; i < 3; ++i)
    {
        if ( (epi[i] > xeps) || (epi[i] < -xeps) )
            return;
    }
    MathTools::mt_crossprod(epi, F+3, F+6, 1);
}

double getOriSign(double* F, double* e, double* pt)
{
    double s1, s2;
    s1 = F[0]*pt[3] + F[3]*pt[4] + F[6]*pt[5];
    s2 = e[1]*pt[2] - e[2]*pt[1];
    return (s1 * s2);
}

void computeHFromF(const std::vector<unsigned int>& sample, double* u, double* ep, double* F, double* H)
{
    double A[9], Ex[9], M[9];
    double p1[3], p2[3], b[3];
    double h[3], norm;
    double *pt1, *pt2, *m;

    // A = [e']_x * F
    MathTools::skew_sym(Ex, ep);
    MathTools::mmul(A, Ex, F, 3);

    m = M;
    for(unsigned int i = 0; i < 3; ++i)
    {
        pt1 = u + 6*sample[i];
        pt2 = pt1 + 3;

        // rows of M are the points x_i
        memcpy(m, pt1, sizeof(double) * 3);
        m += 3;

        // compute b_i
        MathTools::vmul(h, A, pt1, 3);
        MathTools::mt_crossprod(p1, pt2, h, 1);
        MathTools::mt_crossprod(p2, pt2, ep, 1);
        norm = p2[0] * p2[0] + p2[1] * p2[1] + p2[2] * p2[2];
        b[i] =  (p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2]) / norm;
    }

    // compute H = A - epipole*(inv(M)*b)^T
    MathTools::minv(M, 3);
    MathTools::vmul(h, M, b, 3);
    unsigned int k = 0;
    for (unsigned int i = 0; i < 3; ++i)
    {
        for(unsigned int j = 0; j < 3; ++j)
        {
            H[k] = A[k] - ep[i]*h[j];
            k++;
        }
    }
}

unsigned int getHError(const std::vector<unsigned int>& test, unsigned int numPoints, std::vector<double>& errs,
                       double* u, double* H, double threshold)
{
    double* model = H;
    double* inv_model = new double[9];
    double h_x[3], h_inv_xp[3], temp_err;
    double* pt;
    unsigned int num_inliers = 0;
    errs.clear();
    errs.resize(numPoints);

    for (unsigned int i = 0; i < 9; ++i)
    {
        inv_model[i] = model[i];
    }
    MathTools::minv(inv_model, 3);

    // check each point for symmetric transfer error
    for (unsigned int i = 0; i < numPoints; ++i)
    {
        // compute symmetric transfer error
        pt = u + 6*test[i];
        MathTools::vmul(h_x, model, pt, 3);
        MathTools::vmul(h_inv_xp, inv_model, pt+3, 3);

        double err1 = 0.0, err2 = 0.0;
        for (unsigned int j = 0; j < 2; ++j)
        {
            err1 += (h_x[j]/h_x[2] - pt[3+j]) * (h_x[j]/h_x[2] - pt[3+j]);
            err2 += (h_inv_xp[j]/h_inv_xp[2] - pt[j]) * (h_inv_xp[j]/h_inv_xp[2] - pt[j]);
        }
        temp_err = err1 + err2;
        errs[i] = temp_err;

        if (temp_err < threshold)
        {
            ++num_inliers;
        }
    }
    delete[] inv_model;
    return num_inliers;
}

unsigned int computeHFromCorrs(const std::vector<unsigned int>& sample, unsigned int numPoints,
                               unsigned int numDataPoints, double* u, double* H)
{
    // form the matrix of equations for this non-minimal sample
    double *A = new double[numPoints*2*9];
    double *src_ptr;
    double *dst_ptr = A;
    for (unsigned int i = 0; i < numPoints; ++i)
    {
        for (unsigned int j = 0; j < 2; ++j)
        {
            src_ptr = u + 2*sample[i] + j;
            for (unsigned int k = 0; k < 9; ++k)
            {
                *dst_ptr = *src_ptr;
                ++dst_ptr;
                src_ptr += 2*numDataPoints;
            }
        }
    }

    // decompose
    double V[9*9], D[9], *p;
    MathTools::svdu1v(D, A, 2*numPoints, V, 9);

    unsigned int j = 0;
    for (unsigned int i = 1; i < 9; ++i)
    {
        if (D[i] < D[j])
            j = i;
    }
    p = V + j;

    for (unsigned int i = 0; i < 9; ++i)
    {
        H[i] = *p;
        p += 9;
    }

    delete A;

    return 1;
}

unsigned int computeHFromMinCorrs(const std::vector<unsigned int>& sample, unsigned int numPoints,
                                  unsigned int numDataPoints, double* u, double* H)
{
    double A[8*9];
    double At[9*8];

    // form the matrix of equations for this minimal sample
    double *src_ptr;
    double *dst_ptr = A;
    for (unsigned int i = 0; i < numPoints; ++i)
    {
        for (unsigned int j = 0; j < 2; ++j)
        {
            src_ptr = u + 2*sample[i] + j;
            for (unsigned int k = 0; k < 9; ++k)
            {
                *dst_ptr = *src_ptr;
                ++dst_ptr;
                src_ptr += 2*numDataPoints;
            }
        }
    }

    MathTools::mattr(At, A, 8, 9);

    double D[9], U[9*9], V[8*8], *p;
    MathTools::svduv(D, At, U, 9, V, 8);
    p = U + 8;

    for (unsigned int i = 0; i < 9; ++i)
    {
        H[i] = *p;
        p += 9;
    }
    return 1;
}

}
