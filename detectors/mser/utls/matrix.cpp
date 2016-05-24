#undef __STRICT_ANSI__
#include "matrix.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

namespace utls
{

  Matrix2::Matrix2(const double a11, const double a12,
                   const double a21, const double a22)
  {
    a[0][0]=a11;
    a[0][1]=a12;
    a[1][0]=a21;
    a[1][1]=a22;
  }

  Matrix2::Matrix2(const double *A)
  {
    memcpy(a, A, 4*sizeof(double));
  }

  Matrix2::Matrix2(const Matrix2 &other)
  {
    memcpy(a, const_cast<Matrix2 &>(other).a, sizeof(a));
  }

  Matrix2::~Matrix2()
  {
  }

  /* scalar operations */
  Matrix2 Matrix2::operator * (const double s) const
  {
    return Matrix2(a[0][0]*s, a[0][1]*s,
                   a[1][0]*s, a[1][1]*s);
  }

  Matrix2 Matrix2::operator / (const double s) const
  {
    return Matrix2(a[0][0]/s, a[0][1]/s,
                   a[1][0]/s, a[1][1]/s);
  }

  Matrix2 Matrix2::operator + (const double s) const
  {
    return Matrix2(a[0][0]+s, a[0][1]+s,
                   a[1][0]+s, a[1][1]+s);
  }

  Matrix2 Matrix2::operator - (const double s) const
  {
    return Matrix2(a[0][0]-s, a[0][1]-s,
                   a[1][0]-s, a[1][1]-s);
  }


  /* matrix operations */
  void Matrix2::operator = (const Matrix2 &other)
  {
    memcpy(a, const_cast<Matrix2 &>(other).a, sizeof(a));
  }

  Matrix2 Matrix2::operator * (const Matrix2 &m) const
  {
    return Matrix2(
          a[0][0]*m.a[0][0]+a[0][1]*m.a[1][0],
          a[0][0]*m.a[0][1]+a[0][1]*m.a[1][1],
          a[1][0]*m.a[0][0]+a[1][1]*m.a[1][0],
          a[1][0]*m.a[0][1]+a[1][1]*m.a[1][1]
          );
  }

  Matrix2 Matrix2::operator + (const Matrix2 &m) const
  {
    return Matrix2(a[0][0]+m.a[0][0], a[0][1]+m.a[0][1],
                   a[1][0]+m.a[1][0], a[1][1]+m.a[1][1]);
  }

  Matrix2 Matrix2::operator - (const Matrix2 &m) const
  {
    return Matrix2(a[0][0]-m.a[0][0], a[0][1]-m.a[0][1],
                   a[1][0]-m.a[1][0], a[1][1]-m.a[1][1]);
  }


  /* special operations */
  Matrix2 Matrix2::inv(void) const
  {
    double det = a[0][0]*a[1][1] - a[0][1]*a[1][0];
    if (det==0)
      return Matrix2(0,0,0,0);
    return Matrix2( a[1][1]/det, -a[0][1]/det,
                    -a[1][0]/det,  a[0][0]/det);
  }

  Matrix2 Matrix2::transpose(void) const
  {
    return Matrix2(a[0][0], a[1][0],
                   a[0][1], a[1][1]);
  }

  double Matrix2::det(void) const
  {
    return a[0][0]*a[1][1] - a[0][1]*a[1][0];
  }

  double Matrix2::trace(void) const
  {
    return a[0][0]+a[1][1];
  }

  Matrix2 Matrix2::sqrt(void) const
  {
    return Matrix2(::sqrt(a[0][0]), ::sqrt(a[0][1]),
                   ::sqrt(a[1][0]), ::sqrt(a[1][1]));
  }

  double Matrix2::fnorm(void) const
  {
    double sum;
    sum = a[0][0]*a[0][0];
    sum += a[0][1]*a[0][1];
    sum += a[1][0]*a[1][0];
    sum += a[1][1]*a[1][1];
    return ::sqrt(sum);
  }

  /* inplace operations */
  void Matrix2::eye()
  {
    a[0][0]=a[1][1]=1;
    a[1][0]=a[0][1]=0;
  }

  void Matrix2::inv_i(void)
  {
    double det;
    /* swap diagonal */
    det = a[0][0];
    a[0][0] = a[1][1];
    a[1][1] = det;
    /* calc determinant */
    det = double(a[0][0])*a[1][1] - a[0][1]*a[1][0];

    a[0][0] /= det;
    a[0][1] /=-det;
    a[1][0] /=-det;
    a[1][1] /= det;
  }

  void Matrix2::transpose_i(void)
  {
    double tmp;
    tmp = a[1][0];
    a[1][0] = a[0][1];
    a[0][1] = tmp;
  }

  void Matrix2::sqrt_i(void)
  {
    a[0][0] = ::sqrt(a[0][0]);
    a[0][1] = ::sqrt(a[0][1]);
    a[1][0] = ::sqrt(a[1][0]);
    a[1][1] = ::sqrt(a[1][1]);
  }

  void Matrix2::fnormalize(void)
  {
    double n = fnorm();
    a[0][0] /= n;
    a[0][1] /= n;
    a[1][0] /= n;
    a[1][1] /= n;
  }

  double *Matrix2::operator[](int row_idx) const
  {
    return const_cast<double *>(a[row_idx]);
  }

  /* symetric matrix operations */
  void Matrix2::schur_sym(Matrix2 &Q, Matrix2 &T) const
  {
    double t, r;
    if (!(a[0][1]==a[1][0]))
      {
        printf("Matric must be symmetric.\n");
        dump();
        exit(-1);
      }
    if (a[0][1]!=0)
      {
        r = double(a[1][1]-a[0][0])/(2*a[0][1]);
        if (r>=0)
          t = 1.0/(r+::sqrt(1+r*r));
        else
          t = -1.0/(-r+::sqrt(1+r*r));

        r = 1.0/::sqrt(1+t*t); /* c */
        t = t*r;            /* s */
      }
    else
      {
        r = 1;
        t = 0;
      }
    Q[0][0]=r;
    Q[0][1]=t;
    Q[1][0]=-t;
    Q[1][1]=r;
    T = Q.transpose() * (*this) * Q;
    T[0][1]=0;
    T[1][0]=0;
  }

  Matrix2 Matrix2::chol() const
  {
    Matrix2 R;
    R[0][0] = ::sqrt(a[0][0]);
    R[0][1] = a[0][0] / R[0][0];
    R[1][0] = 0;
    R[1][1] = ::sqrt(a[1][1]- R[0][1] * R[0][1]);
    return R;
  }

  void Matrix2::QR(Matrix2 &Q, Matrix2 &R) const
  {
    if (a[1][0]!=0)
      {
        double x=a[0][0], y=a[1][0], r, c, s;
        /* apply Givens transformation which makes *this upper triangular */
        if (fabs(x)<fabs(y))
          {
            r = -x/y;
            s = 1/::sqrt(1+r*r);
            c = s*r;
          }
        else
          {
            r = -y/x;
            c = 1/::sqrt(1+r*r);
            s = c*r;
          }
        Q = Matrix2(c, s, -s, c);
        R = Q.transpose() * (*this);
      }
    else
      {
        /* this is already triangular */
        Q = Matrix2(1,0,0,1);
        R = (*this);
      }
  }

  void Matrix2::dump() const
  {
    printf("%20g %20g\n %20g %20g\n", a[0][0], a[0][1], a[1][0], a[1][1]);
  }

  void Matrix2::svd(Matrix2 &U, Matrix2 &S, Matrix2 &V) const
  {
    Matrix2 Q(1,0,0,1);
    if (a[1][0]!=0)
      {
        double x=a[0][0], y=a[1][0], r, c, s;
        /* apply Givens transformation which makes *this upper triangular */
        if (fabs(x)<fabs(y))
          {
            r = -x/y;
            s = 1/::sqrt(1+r*r);
            c = s*r;
          }
        else
          {
            r = -y/x;
            c = 1/::sqrt(1+r*r);
            s = c*r;
          }
        Q =  Matrix2(c,s,-s,c);
      }
    Matrix2 A = Q.transpose() * (*this);
    /* okej A is upper triangular, use slasv2 algorithm */
    double cu, su, smax, smin, cv, sv;
    slasv2(A[0][0], A[0][1], A[1][1], cu, su, smax, smin, cv, sv);

    U = Q * Matrix2(cu, -su, su, cu);
    if (smax<0)
      {
        smax = -smax;
        /* change signs in first row of V */
        V[0][0] = -cv;
        V[1][0] = -sv;
      }
    else
      {
        V[0][0] = cv;
        V[1][0] = sv;
      }

    if (smin<0)
      {
        smin = -smin;
        /* change signs in second row of V */
        V[0][1] = sv;
        V[1][1] = -cv;
      }
    else
      {
        V[0][1] = -sv;
        V[1][1] = cv;
      }
    S = Matrix2(smax, 0, 0, smin);
  }

#define M_DOUBLE_EPS 2.1e-16
#define isign(i) ((i)<0?(-1):(+1))
#define sign(x) ((x)<0.0?(-1):(+1))

  void Matrix2::slasv2(double f, double g, double h,
                       double &cu, double &su,
                       double &smax, double &smin,
                       double &cv, double &sv) const
  {
    double fa, ga, ha;
    fa = fabs(f);
    ga = fabs(g);
    ha = fabs(h);

    double ft, gt, ht;
    ft = f;
    gt =g;
    ht = h;

    double fmh, d, q, qq, s, ss, spq, dpq, a;
    double tmp, tt, cut=0, sut=0, cvt=0, svt=0;

    int pmax = 1, swap = 0, glarge = 0, tsign = 0;

    if (fa<ha)
      {
        pmax = 3;
        tmp = ft;
        ft = ht;
        ht = tmp;
        tmp = fa;
        fa = ha;
        ha = tmp;
        swap = 1;
      }

    if (ga==0.0)
      {
        smin = ha;
        smax = fa;
        cut = 1;
        sut = 0;
        cvt = 1;
        svt = 0;
      }
    else
      {
        if (ga>fa)
          {
            pmax = 2;
            if ((fa/ga)<M_DOUBLE_EPS)
              {
                glarge = 1;
                smax = ga;
                if (ha>1)
                  smin = fa/(ga/ha);
                else
                  smin = (fa/ga)*ha;
                cut = 1;
                sut = ht/gt;
                cvt = 1;
                svt = ft/gt;
              }
          }
        if (glarge==0)
          {
            fmh = fa - ha;
            if (fmh==fa)
              d = 1;
            else
              d = fmh/fa;
            q = gt/ft;
            s = 2 - d;
            qq = q*q;
            ss = s*s;
            spq = ::sqrt(ss + qq);
            if (d==0)
              dpq = fabs(q);
            else
              dpq = ::sqrt(d*d + qq);

            a = 0.5 * (spq + dpq);
            smin = ha / a;
            smax = fa * a;

            if (qq == 0.0)
              {
                if (d==0.0)
                  tmp = sign(ft)*2*sign(gt);
                else
                  tmp = gt/(sign(ft)*fmh) + q/s;
              }
            else
              tmp = (q/(spq+s) + q/(dpq + d))*(1 + a);
            tt = ::sqrt(tmp*tmp + 4);
            cvt = 2/tt;
            svt = tmp/tt;
            cut = (cvt + svt * q)/a;
            sut = (ht/ft)*svt/a;
          }
        if (swap==1)
          {
            cu=svt;
            su=cvt;
            cv=sut;
            sv=cut;
          }
        else
          {
            cu=cut;
            su=sut;
            cv=cvt;
            sv=svt;
          }
        switch (pmax)
          {
          case 1:
            tsign = sign(cv) * sign(cu) * sign(f);
            break;
          case 2:
            tsign = sign(sv) * sign(cu) * sign(g);
            break;
          case 3:
            tsign = sign(sv) * sign(su) * sign(h);
            break;
          }
        smax = isign(tsign)*smax;
        smin = isign(tsign*sign(f)*sign(h))*smin;
      }
  }

}
