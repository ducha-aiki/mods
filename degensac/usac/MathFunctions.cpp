#include <stdlib.h>
#include <math.h>
#include "MathFunctions.h"

namespace MathTools
{
/*  svdu1v.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
int svdu1v(double *d,double *a,int m,double *v,int n)
{
    double *p,*p1,*q,*pp,*w,*e;
    double s,h,r,t,sv;
    int i,j,k,mm,nm,ms;
    if(m<n) return -1;
    w=(double *)calloc(m+n,sizeof(double));
    e=w+m;
    for(i=0,mm=m,nm=n-1,p=a; i<n ; ++i,--mm,--nm,p+=n+1)
    {
        if(mm>1)
        {
            sv=h=0.;
            for(j=0,q=p,s=0.; j<mm ; ++j,q+=n)
            {
                w[j]= *q;
                s+= *q* *q;
            }
            if(s>0.)
            {
                h=sqrt(s);
                if(*p<0.) h= -h;
                s+= *p*h;
                s=1./s;
                t=1./(w[0]+=h);
                sv=1.+fabs(*p/h);
                for(k=1,ms=n-i; k<ms ; ++k)
                {
                    for(j=0,q=p+k,r=0.; j<mm ; q+=n) r+=w[j++]* *q;
                    r*=s;
                    for(j=0,q=p+k; j<mm ; q+=n) *q-=r*w[j++];
                }
                for(j=1,q=p; j<mm ;) *(q+=n)=t*w[j++];
            }
            *p=sv;
            d[i]= -h;
        }
        if(mm==1) d[i]= *p;
        p1=p+1;
        sv=h=0.;
        if(nm>1)
        {
            for(j=0,q=p1,s=0.; j<nm ; ++j,++q) s+= *q* *q;
            if(s>0.)
            {
                h=sqrt(s);
                if(*p1<0.) h= -h;
                sv=1.+fabs(*p1/h);
                s+= *p1*h;
                s=1./s;
                t=1./(*p1+=h);
                for(k=n,ms=n*(m-i); k<ms ; k+=n)
                {
                    for(j=0,q=p1,pp=p1+k,r=0.; j<nm ; ++j) r+= *q++ * *pp++;
                    r*=s;
                    for(j=0,q=p1,pp=p1+k; j<nm ; ++j) *pp++ -=r* *q++;
                }
                for(j=1,q=p1+1; j<nm ; ++j) *q++ *=t;
            }
            *p1=sv;
            e[i]= -h;
        }
        if(nm==1) e[i]= *p1;
    }
    ldvmat(a,v,n);
    atou1(a,m,n);
    qrbdu1(d,e,a,m,v,n);
    for(i=0; i<n ; ++i)
    {
        if(d[i]<0.)
        {
            d[i]= -d[i];
            for(j=0,p=v+i; j<n ; ++j,p+=n) *p= - *p;
        }
    }
    free(w);
    return 0;
}

/*  ldvmat.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
void ldvmat(double *a,double *v,int n)
{
    double *p0,*q0,*p,*q,*qq;
    double h,s;
    int i,j,k,mm;
    for(i=0,mm=n*n,q=v; i<mm ; ++i) *q++ =0.;
    *v=1.;
    q0=v+n*n-1;
    *q0=1.;
    q0-=n+1;
    p0=a+n*n-n-n-1;
    for(i=n-2,mm=1; i>0 ; --i,p0-=n+1,q0-=n+1,++mm)
    {
        if(*(p0-1)!=0.)
        {
            for(j=0,p=p0,h=1.; j<mm ; ++j,++p) h+= *p* *p;
            h= *(p0-1);
            *q0=1.-h;
            for(j=0,q=q0+n,p=p0; j<mm ; ++j,q+=n) *q= -h* *p++;
            for(k=i+1,q=q0+1; k<n ; ++k)
            {
                for(j=0,qq=q+n,p=p0,s=0.; j<mm ; ++j,qq+=n) s+= *qq* *p++;
                s*=h;
                for(j=0,qq=q+n,p=p0; j<mm ; ++j,qq+=n) *qq-=s* *p++;
                *q++ = -s;
            }
        }
        else
        {
            *q0=1.;
            for(j=0,p=q0+1,q=q0+n; j<mm ; ++j,q+=n) *q= *p++ =0.;
        }
    }
}

/*  atou1.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
void atou1(double *a,int m,int n)
{
    double *p0,*p,*q,*w;
    int i,j,k,mm;
    double s,h;
    w=(double *)calloc(m,sizeof(double));
    p0=a+n*n-1;
    i=n-1;
    mm=m-n;
    if(mm==0)
    {
        *p0=1.;
        p0-=n+1;
        --i;
        ++mm;
    }
    for(; i>=0 ; --i,++mm,p0-=n+1)
    {
        if(*p0!=0.)
        {
            for(j=0,p=p0+n; j<mm ; p+=n) w[j++]= *p;
            h= *p0;
            *p0=1.-h;
            for(j=0,p=p0+n; j<mm ; p+=n) *p= -h*w[j++];
            for(k=i+1,q=p0+1; k<n ; ++k)
            {
                for(j=0,p=q+n,s=0.; j<mm ; p+=n) s+=w[j++]* *p;
                s*=h;
                for(j=0,p=q+n; j<mm ; p+=n) *p-=s*w[j++];
                *q++ = -s;
            }
        }
        else
        {
            *p0=1.;
            for(j=0,p=p0+n,q=p0+1; j<mm ; ++j,p+=n) *p= *q++ =0.;
        }
    }
    free(w);
}

/*  qrbdu1.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
int qrbdu1(double *dm,double *em,double *um,int mm,double *vm,int m)
{
    int i,j,k,n,jj,nm;
    double u,x,y,a,b,c,s,t,w,*p,*q;
    for(j=1,t=fabs(dm[0]); j<m ; ++j)
        if((s=fabs(dm[j])+fabs(em[j-1]))>t) t=s;
    t*=1.e-15;
    n=100*m;
    nm=m;
    for(j=0; m>1 && j<n ; ++j)
    {
        for(k=m-1; k>0 ; --k)
        {
            if(fabs(em[k-1])<t) break;
            if(fabs(dm[k-1])<t)
            {
                for(i=k,s=1.,c=0.; i<m ; ++i)
                {
                    a=s*em[i-1];
                    b=dm[i];
                    em[i-1]*=c;
                    dm[i]=u=sqrt(a*a+b*b);
                    s= -a/u;
                    c=b/u;
                    for(jj=0,p=um+k-1; jj<mm ; ++jj,p+=nm)
                    {
                        q=p+i-k+1;
                        w=c* *p+s* *q;
                        *q=c* *q-s* *p;
                        *p=w;
                    }
                }
                break;
            }
        }
        y=dm[k];
        x=dm[m-1];
        u=em[m-2];
        a=(y+x)*(y-x)-u*u;
        s=y*em[k];
        b=s+s;
        u=sqrt(a*a+b*b);
        if(u>0.)
        {
            c=sqrt((u+a)/(u+u));
            if(c!=0.) s/=(c*u);
            else s=1.;
            for(i=k; i<m-1 ; ++i)
            {
                b=em[i];
                if(i>k)
                {
                    a=s*em[i];
                    b*=c;
                    em[i-1]=u=sqrt(x*x+a*a);
                    c=x/u;
                    s=a/u;
                }
                a=c*y+s*b;
                b=c*b-s*y;
                for(jj=0,p=vm+i; jj<nm ; ++jj,p+=nm)
                {
                    w=c* *p+s* *(p+1);
                    *(p+1)=c* *(p+1)-s* *p;
                    *p=w;
                }
                s*=dm[i+1];
                dm[i]=u=sqrt(a*a+s*s);
                y=c*dm[i+1];
                c=a/u;
                s/=u;
                x=c*b+s*y;
                y=c*y-s*b;
                for(jj=0,p=um+i; jj<mm ; ++jj,p+=nm)
                {
                    w=c* *p+s* *(p+1);
                    *(p+1)=c* *(p+1)-s* *p;
                    *p=w;
                }
            }
        }
        em[m-2]=x;
        dm[m-1]=y;
        if(fabs(x)<t) --m;
        if(m==k+1) --m;
    }
    return j;
}

/*  svduv.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
int svduv(double *d,double *a,double *u,int m,double *v,int n)
{
    double *p,*p1,*q,*pp,*w,*e;
    double s,h,r,t,sv;
    int i,j,k,mm,nm,ms;
    if(m<n) return -1;
    w=(double *)calloc(m+n,sizeof(double));
    e=w+m;
    for(i=0,mm=m,nm=n-1,p=a; i<n ; ++i,--mm,--nm,p+=n+1)
    {
        if(mm>1)
        {
            sv=h=0.;
            for(j=0,q=p,s=0.; j<mm ; ++j,q+=n)
            {
                w[j]= *q;
                s+= *q* *q;
            }
            if(s>0.)
            {
                h=sqrt(s);
                if(*p<0.) h= -h;
                s+= *p*h;
                s=1./s;
                t=1./(w[0]+=h);
                sv=1.+fabs(*p/h);
                for(k=1,ms=n-i; k<ms ; ++k)
                {
                    for(j=0,q=p+k,r=0.; j<mm ; q+=n) r+=w[j++]* *q;
                    r*=s;
                    for(j=0,q=p+k; j<mm ; q+=n) *q-=r*w[j++];
                }
                for(j=1,q=p; j<mm ;) *(q+=n)=t*w[j++];
            }
            *p=sv;
            d[i]= -h;
        }
        if(mm==1) d[i]= *p;
        p1=p+1;
        sv=h=0.;
        if(nm>1)
        {
            for(j=0,q=p1,s=0.; j<nm ; ++j,++q) s+= *q* *q;
            if(s>0.)
            {
                h=sqrt(s);
                if(*p1<0.) h= -h;
                sv=1.+fabs(*p1/h);
                s+= *p1*h;
                s=1./s;
                t=1./(*p1+=h);
                for(k=n,ms=n*(m-i); k<ms ; k+=n)
                {
                    for(j=0,q=p1,pp=p1+k,r=0.; j<nm ; ++j) r+= *q++ * *pp++;
                    r*=s;
                    for(j=0,q=p1,pp=p1+k; j<nm ; ++j) *pp++ -=r* *q++;
                }
                for(j=1,q=p1+1; j<nm ; ++j) *q++ *=t;
            }
            *p1=sv;
            e[i]= -h;
        }
        if(nm==1) e[i]= *p1;
    }
    ldvmat(a,v,n);
    ldumat(a,u,m,n);
    qrbdv(d,e,u,m,v,n);
    for(i=0; i<n ; ++i)
    {
        if(d[i]<0.)
        {
            d[i]= -d[i];
            for(j=0,p=v+i; j<n ; ++j,p+=n) *p= - *p;
        }
    }
    free(w);
    return 0;
}

/*  ldumat.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
void ldumat(double *a,double *u,int m,int n)
{
    double *p0,*q0,*p,*q,*w;
    int i,j,k,mm;
    double s,h;
    w=(double *)calloc(m,sizeof(double));
    for(i=0,mm=m*m,q=u; i<mm ; ++i) *q++ =0.;
    p0=a+n*n-1;
    q0=u+m*m-1;
    mm=m-n;
    i=n-1;
    for(j=0; j<mm ; ++j,q0-=m+1) *q0=1.;
    if(mm==0)
    {
        p0-=n+1;
        *q0=1.;
        q0-=m+1;
        --i;
        ++mm;
    }
    for(; i>=0 ; --i,++mm,p0-=n+1,q0-=m+1)
    {
        if(*p0!=0.)
        {
            for(j=0,p=p0+n,h=1.; j<mm ; p+=n) w[j++]= *p;
            h= *p0;
            *q0=1.-h;
            for(j=0,q=q0+m; j<mm ; q+=m) *q= -h*w[j++];
            for(k=i+1,q=q0+1; k<m ; ++k)
            {
                for(j=0,p=q+m,s=0.; j<mm ; p+=m) s+=w[j++]* *p;
                s*=h;
                for(j=0,p=q+m; j<mm ; p+=m) *p-=s*w[j++];
                *q++ = -s;
            }
        }
        else
        {
            *q0=1.;
            for(j=0,p=q0+1,q=q0+m; j<mm ; ++j,q+=m) *q= *p++ =0.;
        }
    }
    free(w);
}

/*  qrbdv.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
int qrbdv(double *dm,double *em,double *um,int mm,double *vm,int m)
{
    int i,j,k,n,jj,nm;
    double u,x,y,a,b,c,s,t,w,*p,*q;
    for (j=1,t=fabs(dm[0]); j<m ; ++j)
        if((s=fabs(dm[j])+fabs(em[j-1]))>t) t=s;
    t*=1.e-15;
    n=100*m;
    nm=m;
    for(j=0; m>1 && j<n ; ++j)
    {
        for(k=m-1; k>0 ; --k)
        {
            if(fabs(em[k-1])<t) break;
            if(fabs(dm[k-1])<t)
            {
                for(i=k,s=1.,c=0.; i<m ; ++i)
                {
                    a=s*em[i-1];
                    b=dm[i];
                    em[i-1]*=c;
                    dm[i]=u=sqrt(a*a+b*b);
                    s= -a/u;
                    c=b/u;
                    for(jj=0,p=um+k-1; jj<mm ; ++jj,p+=mm)
                    {
                        q=p+i-k+1;
                        w=c* *p+s* *q;
                        *q=c* *q-s* *p;
                        *p=w;
                    }
                }
                break;
            }
        }
        y=dm[k];
        x=dm[m-1];
        u=em[m-2];
        a=(y+x)*(y-x)-u*u;
        s=y*em[k];
        b=s+s;
        u=sqrt(a*a+b*b);
        if(u!=0.)
        {
            c=sqrt((u+a)/(u+u));
            if(c!=0.) s/=(c*u);
            else s=1.;
            for(i=k; i<m-1 ; ++i)
            {
                b=em[i];
                if(i>k)
                {
                    a=s*em[i];
                    b*=c;
                    em[i-1]=u=sqrt(x*x+a*a);
                    c=x/u;
                    s=a/u;
                }
                a=c*y+s*b;
                b=c*b-s*y;
                for(jj=0,p=vm+i; jj<nm ; ++jj,p+=nm)
                {
                    w=c* *p+s* *(p+1);
                    *(p+1)=c* *(p+1)-s* *p;
                    *p=w;
                }
                s*=dm[i+1];
                dm[i]=u=sqrt(a*a+s*s);
                y=c*dm[i+1];
                c=a/u;
                s/=u;
                x=c*b+s*y;
                y=c*y-s*b;
                for(jj=0,p=um+i; jj<mm ; ++jj,p+=mm)
                {
                    w=c* *p+s* *(p+1);
                    *(p+1)=c* *(p+1)-s* *p;
                    *p=w;
                }
            }
        }
        em[m-2]=x;
        dm[m-1]=y;
        if(fabs(x)<t) --m;
        if(m==k+1) --m;
    }
    return j;
}

/*  minv.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
int minv(double *a,int n)
{
    int lc,*le;
    double s,t,tq=0.,zr=1.e-15;
    double *pa,*pd,*ps,*p,*q,*q0;
    int i,j,k,m;
    le=(int *)malloc(n*sizeof(int));
    q0=(double *)malloc(n*sizeof(double));
    for(j=0,pa=pd=a; j<n ; ++j,++pa,pd+=n+1)
    {
        if(j>0)
        {
            for(i=0,q=q0,p=pa; i<n ; ++i,p+=n) *q++ = *p;
            for(i=1; i<n ; ++i)
            {
                lc=i<j?i:j;
                for(k=0,p=pa+i*n-j,q=q0,t=0.; k<lc ; ++k) t+= *p++ * *q++;
                q0[i]-=t;
            }
            for(i=0,q=q0,p=pa; i<n ; ++i,p+=n) *p= *q++;
        }
        s=fabs(*pd);
        lc=j;
        for(k=j+1,ps=pd; k<n ; ++k)
        {
            if((t=fabs(*(ps+=n)))>s)
            {
                s=t;
                lc=k;
            }
        }
        tq=tq>s?tq:s;
        if(s<zr*tq)
        {
            free(le-j);
            free(q0);
            return -1;
        }
        *le++ =lc;
        if(lc!=j)
        {
            for(k=0,p=a+n*j,q=a+n*lc; k<n ; ++k)
            {
                t= *p;
                *p++ = *q;
                *q++ =t;
            }
        }
        for(k=j+1,ps=pd,t=1./ *pd; k<n ; ++k) *(ps+=n)*=t;
        *pd=t;
    }
    for(j=1,pd=ps=a; j<n ; ++j)
    {
        for(k=0,pd+=n+1,q= ++ps; k<j ; ++k,q+=n) *q*= *pd;
    }
    for(j=1,pa=a; j<n ; ++j)
    {
        ++pa;
        for(i=0,q=q0,p=pa; i<j ; ++i,p+=n) *q++ = *p;
        for(k=0; k<j ; ++k)
        {
            t=0.;
            for(i=k,p=pa+k*n+k-j,q=q0+k; i<j ; ++i) t-= *p++ * *q++;
            q0[k]=t;
        }
        for(i=0,q=q0,p=pa; i<j ; ++i,p+=n) *p= *q++;
    }
    for(j=n-2,pd=pa=a+n*n-1; j>=0 ; --j)
    {
        --pa;
        pd-=n+1;
        for(i=0,m=n-j-1,q=q0,p=pd+n; i<m ; ++i,p+=n) *q++ = *p;
        for(k=n-1,ps=pa; k>j ; --k,ps-=n)
        {
            t= -(*ps);
            for(i=j+1,p=ps,q=q0; i<k ; ++i) t-= *++p * *q++;
            q0[--m]=t;
        }
        for(i=0,m=n-j-1,q=q0,p=pd+n; i<m ; ++i,p+=n) *p= *q++;
    }
    for(k=0,pa=a; k<n-1 ; ++k,++pa)
    {
        for(i=0,q=q0,p=pa; i<n ; ++i,p+=n) *q++ = *p;
        for(j=0,ps=a; j<n ; ++j,ps+=n)
        {
            if(j>k)
            {
                t=0.;
                p=ps+j;
                i=j;
            }
            else
            {
                t=q0[j];
                p=ps+k+1;
                i=k+1;
            }
            for(; i<n ;) t+= *p++ *q0[i++];
            q0[j]=t;
        }
        for(i=0,q=q0,p=pa; i<n ; ++i,p+=n) *p= *q++;
    }
    for(j=n-2,le--; j>=0 ; --j)
    {
        for(k=0,p=a+j,q=a+ *(--le); k<n ; ++k,p+=n,q+=n)
        {
            t=*p;
            *p=*q;
            *q=t;
        }
    }
    free(le);
    free(q0);
    return 0;
}

/*  vmul.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
void vmul(double *vp,double *mat,double *v,int n)
{
    double s,*q;
    int k,i;
    for(k=0; k<n ; ++k)
    {
        for(i=0,q=v,s=0.; i<n ; ++i) s+= *mat++ * *q++;
        *vp++ =s;
    }
}

/*  mattr.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
void mattr(double *a,double *b,int m,int n)
{
    double *p;
    int i,j;
    for(i=0; i<n ; ++i,++b)
        for(j=0,p=b; j<m ; ++j,p+=n) *a++ = *p;
}

void skew_sym(double *A, double *a)
{
    A[0] = 0;
    A[1] = -a[2];
    A[2] = a[1];
    A[3] = a[2];
    A[4] = 0;
    A[5] = -a[0];
    A[6] = -a[1];
    A[7] = a[0];
    A[8] = 0;
}

/*  mmul.c    CCMATH mathematics library source code.
*
*  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
*  This code may be redistributed under the terms of the GNU library
*  public license (LGPL). ( See the lgpl.license file for details.)
* ------------------------------------------------------------------------
*/
void mmul(double *c,double *a,double *b,int n)
{
    double *p,*q,s;
    int i,j,k;
    trnm(b,n);
    for(i=0; i<n ; ++i,a+=n)
    {
        for(j=0,q=b; j<n ; ++j)
        {
            for(k=0,p=a,s=0.; k<n ; ++k) s+= *p++ * *q++;
            *c++ =s;
        }
    }
    trnm(b,n);
}

void mt_crossprod(double *out, const double *a, const double *b, unsigned int st)
{
    unsigned int st2 = 2 * st;
    out[0] = a[st]*b[st2] - a[st2]*b[st];
    out[1] = a[st2]*b[0]  - a[0]*b[st2];
    out[2] = a[0]*b[st]   - a[st]*b[0];
}

/*  trnm.c    CCMATH mathematics library source code.
 *
 *  Copyright (C)  2000   Daniel A. Atkinson    All rights reserved.
 *  This code may be redistributed under the terms of the GNU library
 *  public license (LGPL). ( See the lgpl.license file for details.)
 * ------------------------------------------------------------------------
 */
void trnm(double *a,int n)
{
    double s,*p,*q;
    int i,j,e;
    for(i=0,e=n-1; i<n-1 ; ++i,--e,a+=n+1)
    {
        for(p=a+1,q=a+n,j=0; j<e ; ++j)
        {
            s= *p;
            *p++ = *q;
            *q=s;
            q+=n;
        }
    }
}

}
