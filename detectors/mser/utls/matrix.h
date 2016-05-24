#ifndef __UTLS_MATRIX_H__
#define __UTLS_MATRIX_H__

#include <math.h>

namespace utls

{
  /* simple Matrix2 class */
  class Matrix2
  {
  public:
    Matrix2(const double a11=0,const double a12=0,
            const double a21=0,const double a22=0);
    Matrix2(const double *A);
    Matrix2(const Matrix2 &other);
    ~Matrix2();
    void dump(void) const;

  public:
    /* scalar operations */
    Matrix2 operator * (const double s) const;
    Matrix2 operator / (const double s) const;
    Matrix2 operator + (const double s) const;
    Matrix2 operator - (const double s) const;

  public:
    /* matrix operations */
    void operator = (const Matrix2 &other);
    Matrix2 operator * (const Matrix2 &m) const;
    Matrix2 operator + (const Matrix2 &m) const;
    Matrix2 operator - (const Matrix2 &m) const;

  public:
    /* special operations */

    /* returns inverse matrix */
    Matrix2 inv(void) const;
    /* returns transposed matrix */
    Matrix2 transpose(void) const;
    /* returns determinant */
    double det(void) const;
    /* returns trace */
    double trace(void) const;
    /* memberwise square root */
    Matrix2 sqrt(void) const;
    /* frobenius norm */
    double fnorm(void) const;

    double *operator[](int row_idx) const;

  public:
    /* inplace operations */
    void eye();
    void inv_i(void);
    void transpose_i(void);
    void sqrt_i(void);
    void fnormalize(void);

  public:
    /* symetric matrix operations */
    /* calculates Schur decomposition of symetric matrix.
       decomposion of matrix to rotation Q (orthonormal matrix) and
       scale T (matrix with eigenvalues on main diagonal) such that
       matrix = Q * T * Q' */
    void schur_sym(Matrix2 &Q, Matrix2 &T) const;

    /* calculate Cholesky decomposition (works for symetric positive
       semidefinite matrixes only
       when G = chol2(); then Matrix = G*G' */
    Matrix2 chol() const;

    /* Givens QR transformation step for 2x2 matrix */
    void QR(Matrix2 &Q, Matrix2 &R) const;

    /* calculates Singular Value Decomposition of matrix
       matrix = U * S * V'  */
    void svd(Matrix2 &U, Matrix2 &S, Matrix2 &V) const;

  public:
    /* public members */
    /* data first index is row !!! second index is column */
    double a[2][2];

  private:
    /* algorithms */
    /* SVD of a 2x2 double upper triangular matrix
     *
     *   [f g] = [ cu -su] * [smax 0] * [ cv  sv]
     *   [0 h]   [ su  cu] * [0 smin] * [-sv  cv]
     *
     * smax and smin are singular values
     * cv, sv - entries of right singular vector
     * cu, su - entries of left singular vector
     * f, g, h - input matrix entries
     *
     * This code is translated from FORTRAN code SLASV2 listed in
     * Z.Bai and J.Demmel,
     * "Computing the Generalized Singular Value Decomposition",
     * SIAM J. Sci. Comput., Vol. 14, No. 6, pp. 1464-1486, November 1993
     *
     */
    void slasv2(double f, double g, double h,
                double &cu, double &su,
                double &smax, double &smin,
                double &cv, double &sv) const;

  };

}
#endif // __UTLS_MATRIX_H__
