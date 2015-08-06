/*
This is free library for a non-commercial purpose. Commercial use
requires a licence. The software is distributed AS IS, WITHOUT ANY
IMPLIED OR EXPRESSED WARRANTY and WITHOUT ANY FURTHER SUPPORT for
the desired and/or other purpose.
(c) Jan Cech (cechj@cmp.felk.cvut.cz) FEE CTU Prague, Apr 30, 2009
*/

#include <stack>
#include <math.h>
#include <scv.hpp>

using namespace std;  // std c++ libs implemented in std

#define max(a,b) (a>b)?a:b;
#define round(a) int(a+0.5)

//const double NaN = mxGetNaN();
//const double eps = mxGetEps();

//const double NaN = mxGetNaN();
const double eps = 0.000001;

//
#include <iostream>
//

bool operator < (const seed& a, const seed&b)
{
    return a.c<b.c;
}

// MAIN methods implementation ---------------------------------------------

MAIN::MAIN(ARRAY_2D<double>& iL_, ARRAY_2D<double> &iR_,
           ARRAY_2D<double>& SEEDs_, ARRAY_2D<double> IN_,
           ARRAY_2D<double>& D_, ARRAY_2D<double>& Dv_, ARRAY_2D<double>& W_,
           ARRAY_2D<double>& K_, ARRAY_2D<double>& I_,
           ARRAY_2D<double>& Q_, ARRAY_2D<double>& OUT_,
           int w_, ARRAY_2D<double>& local_search_, double c_thr_,
           double dmin_, double dmax_, double dVmin_, double dVmax_,
           double epsilon_, bool i_seed_accept_, int maxK_)
{

    //seed s;
    int i;
    //double tmp;
    iL.assign(iL_);
    iR.assign(iR_);
    SEEDs.assign(SEEDs_);
    IN.assign(IN_);
    D.assign(D_);
    Dv.assign(Dv_);
    W.assign(W_);
    K.assign(K_);
    I.assign(I_);
    Q.assign(Q_);
    OUT.assign(OUT_);

    ARRAY_2D<int> loc_search(local_search_.M,local_search_.N);
    for (i=1; i<=local_search_.M*local_search_.N; i++)
    {
        loc_search[i] = int(local_search_[i]);
    }
    local_search.assign(loc_search);
    loc_search.need_to_release = 0;
    local_search.need_to_release = 1;

    w = w_;
    c_thr = c_thr_;
    dmin = dmin_;
    dmax = dmax_;
    dVmin = dVmin_;
    dVmax = dVmax_;
    epsilon = epsilon_;
    i_seed_accept = i_seed_accept_;
    maxK = maxK_;

    //output intialilzation
    for(i=1; i<=(iL.M*iL.N); i++)
    {
        /*   D[i] = NaN;
           Dv[i] = NaN;
           W[i] = NaN;
           I[i] = NaN; */
        D[i] = 0;
        Dv[i] = 0;
        W[i] = 0;
        I[i] = 0;
    }

    for(i=1; i<=(iR.M*iR.N); i++)
    {
        K[i] = 0;
    }

//   s.xl = 1; s.xr = 1; s.row = 2; s.c = 0.1;
//   SEEDS.push(s);
//   s.xl = 1; s.xr = 1; s.row = 2; s.c = 0.1;
//   SEEDS.push(s);
//   //Print out the content
//   while (!SEEDS.empty()) {
//     s = SEEDS.top();
//     mexPrintf("(%i,%i,%i|%f) \n",s.xl,s.xr,s.row,s.c);
//     SEEDS.pop();
//   }


// iLcv = cv::Mat(iL.M,iL.N,CV_64F,iL.data).clone();
// iLcv.convertTo(iLcv,CV_32F);
// iRcv = cv::Mat(iR.M,iR.N,CV_64F,iR.data).clone();
// iRcv.convertTo(iRcv,CV_32F);


}

// window correlation -----------------------------------------------------
//
// double MAIN::wcorr(int xl, int xr, int yl, int yr) {
//   int i,j;
//   double sLR;
//   double c;
//
//   sLR = 0;
//   for (i=-w; i<=w; i++) {
//       for (j=-w; j<=w; j++) {
//         sLR = sLR + iL.s(yl+i,xl+j) * iR.s(yr+i,xr+j);
//       }
//   }
//
//   c = 2*(sLR - sL.s(yl,xl)*sR.s(yr,xr)) / (vL.s(yl,xl) + vR.s(yr,xr));
//   if (c>1) c=0; //hack
//   return c;
// }

// window correlation (affine frame) --------------------------------------

double MAIN::wcorr_a(seed& s)
{
    double L[441]; //max window size = 10 (2*10+1)^2
    double R[441];
    //double L[41*41]; //max window size = 10 (2*10+1)^2
    //double R[41*41];

    double sL,sR,vL,vR,sLR;
    double c;
    double a;

    int i,j,k;
    int xl,yl,xr,yr;
    k=0;

    //
    /*
       double mrScale = 3.0*sqrt(3.0); // half patch size in pixels of image
       int patchSize = 41;
       int patchImageSize = 2*int(mrScale)+1; // odd size
       double imageToPatchScale = double(patchImageSize) / (double)patchSize;  // patch size in the image / patch size -> amount of down/up sampling

       double xrr = s.a1*s.a3_0 + s.a2*yl + s.a3;
       double yrr = s.a4*s.a3_0 + s.a5*yl + s.a6;

       double Xahom[3*3] = {xrr, s.a1*(s.a3_0+s.a2) + s.a2*s.a6_0 + s.a3, s.a1*(s.a3_0) + s.a2*(s.a6_0+s.a3) + s.a3,
                        yrr, s.a4*(s.a3_0 +s.a2)+ s.a5*s.a6_0 + s.a6, s.a4*s.a3_0 + s.a5*(s.a6_0+s.a3) + s.a6,
                        1.0, 1.0, 1.0};
       double Xaprhom[3*3] =  {0, 1.0, 0,
                               0, 0, 1.0,
                               1.0, 1.0, 1.0};
       double Xaprhom_inv[3*3];
       assert(inv_3x3(Xaprhom,Xaprhom_inv));
       double Ar[3*3];
       multiply3x3(Xahom,Xaprhom_inv,Ar);

       cv::Mat patchL(patchSize,patchSize,CV_32F);
       cv::Mat patchR(patchSize,patchSize,CV_32F);
       interpolate(iLcv, (float)s.xl,
                       (float)s.yl,
                       (float)s.a1*imageToPatchScale,
                       (float)s.a2*imageToPatchScale,
                       (float)s.a4*imageToPatchScale,
                       (float)s.a5*imageToPatchScale,
                       patchL);

       interpolate(iRcv, (float)xrr,
                       (float)yrr,
                       (float)Ar[0]*imageToPatchScale,
                       (float)Ar[1]*imageToPatchScale,
                       (float)Ar[3]*imageToPatchScale,
                       (float)Ar[4]*imageToPatchScale,
                       patchR);

       for (i=0; i<patchSize; i++) {
           for (j=0; j<patchSize; j++) {
               xl = i;
               yl = j;
               xr = xl;
               yr = yl; //Mishkin

               //if ((xr<1) || (xr>iR.N) || (yr<1) || (yr>iR.M)) return 0;
               //mexPrintf("(%i,%i) (%i %i %i %i) ... ",i,j,xl,yl,xr,yr);
               L[k] = patchL.at<float>(yl,xl);
               R[k] = patchR.at<float>(yr,xr);
               //mexPrintf("*");
               //mexPrintf("%f %f \n",L[k],R[k]);
               //mexPrintf("%i %i | %i %i |  \n",i,j,xr,yr);
               k++;
           }
       }
    */
    ///
    for (i=-w; i<=w; i++)
    {
        for (j=-w; j<=w; j++)
        {
            xl = s.xl+i;
            yl = s.yl+j;
            xr = int(round(s.a1*xl + s.a2*yl + s.a3));
            yr = int(round(s.a4*xl + s.a5*yl + s.a6));

            if ((xr<1) || (xr>iR.N) || (yr<1) || (yr>iR.M)) return 0;
            //mexPrintf("(%i,%i) (%i %i %i %i) ... ",i,j,xl,yl,xr,yr);
            L[k] = iL.s(yl,xl);
            R[k] = iR.s(yr,xr);
            //mexPrintf("*");
            //mexPrintf("%f %f \n",L[k],R[k]);
            //mexPrintf("%i %i | %i %i |  \n",i,j,xr,yr);
            k++;
        }
    }

    sL = 0;
    sR = 0;
    vL = 0;
    vR = 0;
    sLR = 0;
    for (i=0; i<k; i++)
    {
        sL = sL + L[i];
        sR = sR + R[i];
        vL = vL + L[i]*L[i];
        vR = vR + R[i]*R[i];
        sLR = sLR + L[i]*R[i];
    }
    a = 2*w+1;
    sL = sL/a;
    sR = sR/a;
    vL = vL - sL*sL;
    vR = vR - sR*sR;

    c = 2*(sLR-sL*sR)/(vL+vR);
    //mexPrintf("* %f %f %f ## %f \n",sL,vL,sLR,c);
    //printf("* %f %f %f ## %f \n",sL,vL,sLR,c);

    if (c>1) c=0; //hack
    num_correlations++;
    return c;
}

// window correlation (affine frame + bilinear interpolation) ------------

double MAIN::wcorr_a_interp(seed& s)
{
    double L[100];
    double R[100];
    double sL,sR,vL,vR,sLR;
    double c;
    double a;
    double f00,f01,f10,f11;

    int i,j,k;
    int xl,yl;
    double xr,yr;

    k=0;
    for (i=-w; i<=w; i++)
    {
        for (j=-w; j<=w; j++)
        {
            xl = s.xl+i;
            yl = s.yl+j;
            xr = s.a1*xl + s.a2*yl + s.a3;
            yr = s.a4*xl + s.a5*yl + s.a6;
            if ((xr<1) || (xr>iR.N-1) || (yr<1) || (yr>iR.M-1)) return 0;
            L[k] = iL.s(yl,xl);
            f00 = iR.s(int(floor(yr)),int(floor(xr)));
            f10 = iR.s(int(floor(yr)),int(ceil(xr)));
            f01 = iR.s(int(ceil(yr)),int(floor(xr)));
            f11 = iR.s(int(ceil(yr)),int(floor(xr)));
            xr = xr - floor(xr);
            yr = yr - floor(yr);
            R[k] = f00*(1-xr)*(1-yr)+f10*xr*(1-yr)+f01*(1-xr)*yr+f11*xr*yr;

            // mexPrintf("%i %i | %i %i |  \n",i,j,xr,yr);
            // mexPrintf("%f %f \n",L[k],R[k]);
            k++;
        }
    }

    sL = 0;
    sR = 0;
    vL = 0;
    vR = 0;
    sLR = 0;
    for (i=0; i<k; i++)
    {
        sL = sL + L[i];
        sR = sR + R[i];
        vL = vL + L[i]*L[i];
        vR = vR + R[i]*R[i];
        sLR = sLR + L[i]*R[i];
    }
    a = 2*w+1;
    sL = sL/a;
    sR = sR/a;
    vL = vL - sL*sL;
    vR = vR - sR*sR;

    c = 2*(sLR-sL*sR)/(vL+vR);
    //mexPrintf("* %f %f %f ## %f \n",sL,vL,sLR,c);
    if (c>1) c=0; //hack
    return c;
}

// local optimization ------------------------------------------------------

void MAIN::local_optim(seed& s, double& c, int& dp, int& dpv, double& cd)
{
    //c,dp,dpv,cd is output (cd is correlation difference)

    int i,j,k;//,xrr,yrr;
    double cc; //current corelation
    double c2 =0; //2nd best correlation
    seed st;
    c = -1;
    dp = 0;
    dpv = 0;

    st.xl = s.xl;
    st.yl = s.yl;
    st.a1 = s.a1;
    st.a2 = s.a2;
    st.a3 = s.a3;
    st.a4 = s.a4;
    st.a5 = s.a5;
    st.a6 = s.a6;
    st.c =  s.c;

    for (k=1; k<=local_search.M; k++)
    {
        //  for (i=-range; i<=range; i++) {
        i = local_search.s(k,1);
        j = local_search.s(k,2);
        st.a3 = s.a3 + i;
        st.a6 = s.a6 + j;
        cc = wcorr_a(st);
        //      mexPrintf("(%i,%i, %i,%i |%f) ",xl,xrr,yl,yrr,cc);
        if (cc > c)
        {
            c2 = c;
            c = cc;
            dp = i;
            dpv = j;
        };
    }
    cd = c-c2;
    if ( c-c2 < epsilon )   //do not grow when ambiguity
    {
        c = -1;
    }
    //  mexPrintf("\n");
}

// growing -----------------------------------------------------------------

void MAIN::grow()
{

    unsigned vis=0;
    int xr,yr; //xl
    int xr0,yr0; //expected according to local affinity
    int dp,dpv;
    int d,dv;
    int dx,dy; //deviation from expected local affinity
    seed s,st,sn;
    double c,cd;
    int ind,indr;
    //mxArray *M; //only for visualization
    int i;
    int q;
    double sum_c, sum_cd, sum_distLAF;
    int sum_K;
    int l;
    int p,maxP; //progress bar
    double sift_ratio;
    int stage; //decision stage
    int max_stage = IN.N;

    double bar_g = 0;
    double bar_c = 0;
    double bar_u = 0; //growing statistics

    double SVM_output; //output of the SVM_classifier
    int decision_status; //Wald SPRT decision status (+1 pos, -1 neg, 0 not conclusive)
    bool queue_exhausted; //queue is exhausted (further growth impossible)

    stack<int> IXr; //stack of indeces into right image (of accepted pixels)
    //(to quickly erase K after each LAF)

    p=0;
    maxP = int(SEEDs.M/10+1); //progress bar

    for (i=1; i<=SEEDs.M; i++) //for all LAFs (3 points with same affine transform)
    {

        stage = 1;
        queue_exhausted = true;
        q=0; //number of accepted pixels
        sum_c=0;
        sum_cd=0;
        sum_K=0;
        sum_distLAF=0;

        num_correlations = 0;
        sift_ratio = SEEDs.s(i,13);

        //1st stage (decision according to SIFT ratio only, no growing yet) --------
        SVM_output = IN.s(7,stage)*sift_ratio;
        // std::cout << IN.s(7,stage) << " " << sift_ratio << " " << SVM_output << " " << IN.s(1,stage) << " " << IN.s(2,stage) << std::endl;

        if ( (SVM_output>IN.s(1,stage)) || (SVM_output<IN.s(2,stage)) )   //decided
        {
            //i1                         i2
            decision_status = SVM_output > IN.s(1,stage) ? 1 : -1;
        }
        else   //end of 1st stage decision ------------------------------------------------
        {

            decision_status = 0;
            stage = 2;

            s.a1 = SEEDs.s(i,7);
            s.a2 = SEEDs.s(i,8);
            s.a3 = SEEDs.s(i,9);
            s.a4 = SEEDs.s(i,10);
            s.a5 = SEEDs.s(i,11);
            s.a6 = SEEDs.s(i,12);
            s.a3_0 = SEEDs.s(i,9);
            s.a6_0 = SEEDs.s(i,12);

            //mexPrintf("%i %i %f %f %f %f %f %f | %f \n ",s.xl,s.yl,s.a1,s.a2,s.a3,s.a4,s.a5,s.a6,s.c);
            //mexPrintf("%i %i %i",s.xl,s.yl,w);

            //  printf("%i %i %f %f %f %f %f %f | %f \n ",s.xl,s.yl,s.a1,s.a2,s.a3,s.a4,s.a5,s.a6,s.c);
            //printf("%i %i %i \n",s.xl,s.yl,w);

            //push all 3 points of LAF into the priority queue
            s.xl = int(SEEDs.s(i,1));
            s.yl = int(SEEDs.s(i,2));
            if ((s.xl>w) && (s.xl<D.N-w) && (s.yl>w) && (s.yl<D.M-w))
            {
                s.c = wcorr_a(s);
                SEEDS.push(s);
            }
            s.xl = int(SEEDs.s(i,3));
            s.yl = int(SEEDs.s(i,4));
            if ((s.xl>w) && (s.xl<D.N-w) && (s.yl>w) && (s.yl<D.M-w))
            {
                s.c = wcorr_a(s);
                SEEDS.push(s);
            }
            s.xl = int(SEEDs.s(i,5));
            s.yl = int(SEEDs.s(i,6));
            if ((s.xl>w) && (s.xl<D.N-w) && (s.yl>w) && (s.yl<D.M-w))
            {
                s.c = wcorr_a(s);
                SEEDS.push(s);
            }

            //main loop
            while (!SEEDS.empty())
            {

                s = SEEDS.top();
                SEEDS.pop();   //remove the processed seed

                // printf("%i %i %f %f %f %f %f %f | %f \n ",s.xl,s.yl,s.a1,s.a2,s.a3,s.a4,s.a5,s.a6,s.c);
                // printf("%f ",s.c);

                if ((s.xl>w+1) && (s.xl<D.N-w-1) && (s.yl>w+1) && (s.yl<D.M-w-1))
                {

                    //input shifted seed
                    sn.a1=s.a1;
                    sn.a2=s.a2;
                    sn.a3=s.a3;
                    sn.a4=s.a4;
                    sn.a5=s.a5;
                    sn.a6=s.a6;
                    //sn.c=s.c;

                    //output shifted seed
                    st.a3_0 = s.a3_0;
                    st.a6_0 = s.a6_0;
                    st.a1=s.a1;
                    st.a2=s.a2;
                    st.a4=s.a4;
                    st.a5=s.a5;

                    //left
                    sn.xl=s.xl-1;
                    sn.yl=s.yl;
                    //    mexPrintf("%i \n",sn.xl);
                    ind = D.sub2ind(sn.yl,sn.xl);
                    if ( i != I[ind] )
                    {
                        local_optim(sn,c,dp,dpv,cd);
                        if (c>c_thr)
                        {
                            xr = int(round(sn.a1*sn.xl+sn.a2*sn.yl+sn.a3+dp));
                            xr0 = int(round(sn.a1*sn.xl+sn.a2*sn.yl+st.a3_0));
                            dx = xr-xr0;
                            d = sn.xl-xr;
                            if ((dx>=dmin) && (dx<=dmax))   //horizontl searchrange check
                            {
                                yr = int(round(sn.a4*sn.xl+sn.a5*sn.yl+sn.a6+dpv));
                                yr0 = int(round(sn.a4*sn.xl+sn.a5*sn.yl+st.a6_0));
                                dy = yr-yr0;
                                dv = sn.yl-yr;
                                if ((dy>=dVmin) && (dy<=dVmax))   //vertical searchrange check
                                {
                                    indr = K.sub2ind(yr,xr);
                                    if ( K[indr] < maxK)
                                    {
                                        if (K[indr]==1) sum_K = sum_K + 2; //sum all non-bijective pixels (the first one too)
                                        if (K[indr]>1) sum_K++;
                                        K[indr]++;
                                        IXr.push(indr);
                                        I[ind] = i;
                                        D[ind] = d;//xr-xr0;//d;
                                        Dv[ind] = dv;//yr-yr0;//dv;
                                        W[ind] = c;
                                        st.xl = sn.xl;
                                        st.yl = sn.yl;
                                        st.a3 = sn.a3+dp;
                                        st.a6 = sn.a6+dpv;
                                        st.c = c;
                                        SEEDS.push(st);
                                        //mexPrintf("%i %i %f \n",st.xl,st.yl,st.c);
                                        q++;
                                        sum_c = sum_c+c;
                                        sum_cd = sum_cd+cd;
                                        sum_distLAF = sum_distLAF + sqrt(double((xr0-xr)*(xr0-xr)+(yr0-yr)*(yr0-yr)))/vis_step;
                                    }
                                }
                            }
                        }
                    };


                    //right
                    sn.xl=s.xl+1;
                    sn.yl=s.yl;
                    //mexPrintf("%i \n",sn.xl);
                    ind = D.sub2ind(sn.yl,sn.xl);
                    if ( i != I[ind] )
                    {
                        local_optim(sn,c,dp,dpv,cd);
                        if (c>c_thr)
                        {
                            xr = int(round(sn.a1*sn.xl+sn.a2*sn.yl+sn.a3+dp));
                            xr0 = int(round(sn.a1*sn.xl+sn.a2*sn.yl+st.a3_0));
                            dx = xr-xr0;
                            d = sn.xl-xr;
                            if ((dx>=dmin) && (dx<=dmax))   //horizontl searchrange check
                            {
                                yr = int(round(sn.a4*sn.xl+sn.a5*sn.yl+sn.a6+dpv));
                                yr0 = int(round(sn.a4*sn.xl+sn.a5*sn.yl+st.a6_0));
                                dy = yr-yr0;
                                dv = sn.yl-yr;
                                if ((dy>=dVmin) && (dy<=dVmax))   //vertical searchrange check
                                {
                                    indr = K.sub2ind(yr,xr);
                                    if ( K[indr] < maxK)
                                    {
                                        if (K[indr]==1) sum_K = sum_K + 2; //sum all non-bijective pixels (the first one too)
                                        if (K[indr]>1) sum_K++;
                                        K[indr]++;
                                        IXr.push(indr);
                                        I[ind] = i;
                                        D[ind] = d;//xr-xr0;//d;
                                        Dv[ind] = dv;//yr-yr0;//dv;
                                        W[ind] = c;
                                        st.xl = sn.xl;
                                        st.yl = sn.yl;
                                        st.a3 = sn.a3+dp;
                                        st.a6 = sn.a6+dpv;
                                        st.c = c;
                                        SEEDS.push(st);
                                        //mexPrintf("%i %i %f \n",st.xl,st.yl,st.c);
                                        q++;
                                        sum_c = sum_c+c;
                                        sum_cd = sum_cd+cd;
                                        sum_distLAF = sum_distLAF + sqrt(double((xr0-xr)*(xr0-xr)+(yr0-yr)*(yr0-yr)))/vis_step;
                                    }
                                }
                            }
                        }
                    };

                    //up
                    sn.xl=s.xl;
                    sn.yl=s.yl-1;
                    ind = D.sub2ind(sn.yl,sn.xl);
                    if ( i != I[ind] )
                    {
                        local_optim(sn,c,dp,dpv,cd);
                        if (c>c_thr)
                        {
                            xr = int(round(sn.a1*sn.xl+sn.a2*sn.yl+sn.a3+dp));
                            xr0 = int(round(sn.a1*sn.xl+sn.a2*sn.yl+st.a3_0));
                            dx = xr-xr0;
                            d = sn.xl-xr;
                            if ((dx>=dmin) && (dx<=dmax))   //horizontl searchrange check
                            {
                                yr = int(round(sn.a4*sn.xl+sn.a5*sn.yl+sn.a6+dpv));
                                yr0 = int(round(sn.a4*sn.xl+sn.a5*sn.yl+st.a6_0));
                                dy = yr-yr0;
                                dv = sn.yl-yr;
                                if ((dy>=dVmin) && (dy<=dVmax))   //vertical searchrange check
                                {
                                    indr = K.sub2ind(yr,xr);
                                    if ( K[indr] < maxK)
                                    {
                                        if (K[indr]==1) sum_K = sum_K + 2; //sum all non-bijective pixels (the first one too)
                                        if (K[indr]>1) sum_K++;
                                        K[indr]++;
                                        IXr.push(indr);
                                        I[ind] = i;
                                        D[ind] = d;//xr-xr0;//d;
                                        Dv[ind] = dv;//yr-yr0;//dv;
                                        W[ind] = c;
                                        st.xl = sn.xl;
                                        st.yl = sn.yl;
                                        st.a3 = sn.a3+dp;
                                        st.a6 = sn.a6+dpv;
                                        st.c = c;
                                        SEEDS.push(st);
                                        //mexPrintf("%i %i %f \n",st.xl,st.yl,st.c);
                                        q++;
                                        sum_c = sum_c+c;
                                        sum_cd = sum_cd+cd;
                                        sum_distLAF = sum_distLAF + sqrt(double((xr0-xr)*(xr0-xr)+(yr0-yr)*(yr0-yr)))/vis_step;
                                    }
                                }
                            }
                        }
                    };

                    //down
                    sn.xl=s.xl;
                    sn.yl=s.yl+1;
                    ind = D.sub2ind(sn.yl,sn.xl);
                    if ( i != I[ind] )
                    {
                        local_optim(sn,c,dp,dpv,cd);
                        if (c>c_thr)
                        {
                            xr = int(round(sn.a1*sn.xl+sn.a2*sn.yl+sn.a3+dp));
                            xr0 = int(round(sn.a1*sn.xl+sn.a2*sn.yl+st.a3_0));
                            dx = xr-xr0;
                            d = sn.xl-xr;
                            if ((dx>=dmin) && (dx<=dmax))   //horizontl searchrange check
                            {
                                yr = int(round(sn.a4*sn.xl+sn.a5*sn.yl+sn.a6+dpv));
                                yr0 = int(round(sn.a4*sn.xl+sn.a5*sn.yl+st.a6_0));
                                dy = yr-yr0;
                                dv = sn.yl-yr;
                                if ((dy>=dVmin) && (dy<=dVmax))   //vertical searchrange check
                                {
                                    indr = K.sub2ind(yr,xr);
                                    if ( K[indr] < maxK)
                                    {
                                        if (K[indr]==1) sum_K = sum_K + 2; //sum all non-bijective pixels (the first one too)
                                        if (K[indr]>1) sum_K++;
                                        K[indr]++;
                                        IXr.push(indr);
                                        I[ind] = i;
                                        D[ind] = d;//xr-xr0;//d;
                                        Dv[ind] = dv;//yr-yr0;//dv;
                                        W[ind] = c;
                                        st.xl = sn.xl;
                                        st.yl = sn.yl;
                                        st.a3 = sn.a3+dp;
                                        st.a6 = sn.a6+dpv;
                                        st.c = c;
                                        SEEDS.push(st);
                                        //mexPrintf("%i %i %f \n",st.xl,st.yl,st.c);
                                        q++;
                                        sum_c = sum_c+c;
                                        sum_cd = sum_cd+cd;
                                        sum_distLAF = sum_distLAF + sqrt(double((xr0-xr)*(xr0-xr)+(yr0-yr)*(yr0-yr)))/vis_step;
                                    }
                                }
                            }
                        }
                    };

                    vis++;
                } //end of range check if

                //---
                if ( vis >= IN.s(3,stage) )
                {
                    //SVM
                    //bar_g = double(q)/double(vis);
                    bar_g = IN.s(3,stage)==0 ? 0 : double(q)/IN.s(3,stage);  //growth_rate is normalized with maximum number of growing_steps (not steps achieved)!
                    bar_c = q==0 ? 0 : sum_c/double(q);
                    bar_u = q==0 ? 0 : double(sum_K)/double(q);
                    SVM_output = IN.s(4,stage)*bar_g + IN.s(5,stage)*bar_c + IN.s(6,stage)*bar_u + IN.s(7,stage)*sift_ratio + IN.s(8,stage);
                    //  w1                     w2                   w3                      w4                         b
                    //Wald's SPRT
                    if ( (SVM_output>IN.s(1,stage)) || (SVM_output<IN.s(2,stage)) || (stage==max_stage))   //decided
                    {
                        //i1                         i2

                        if (stage==max_stage) decision_status = 0;
                        else decision_status = SVM_output > IN.s(1,stage) ? 1 : -1;
                        while (!SEEDS.empty()) SEEDS.pop(); //empty the priority queue of current LAF
                        queue_exhausted = false;
                        break;
                    }
                    else   //undecided
                    {
                        stage++;
                        decision_status = 0;
                    }
                }


            } //end of while

            if (queue_exhausted)   //queu is exhausted (no further growth, compute statistics from the growth achieved)
            {
                //SVM
                bar_g = IN.s(3,stage)==0 ? 0 : double(q)/IN.s(3,stage);  //growth_rate is normalized with maximum number of growing_steps (not steps achieved)!
                bar_c = q==0 ? 0 : sum_c/double(q);
                bar_u = q==0 ? 0 : double(sum_K)/double(q);
                SVM_output = IN.s(4,stage)*bar_g + IN.s(5,stage)*bar_c + IN.s(6,stage)*bar_u + IN.s(7,stage)*sift_ratio + IN.s(8,stage);
                //  w1                     w2                   w3                      w4                         b
                if ( (SVM_output>IN.s(1,stage)) || (SVM_output<IN.s(2,stage)) )   //decided
                {
                    //i1                         i2
                    decision_status = SVM_output > IN.s(1,stage) ? 1 : -1;
                }
                else
                {
                    decision_status = 0;
                }
            }

            while (!IXr.empty())   //reinitial the K-array
            {
                l=IXr.top();
                IXr.pop();
                //if (K[l]>1) sum_K = sum_K+int(K[l]);
                K[l]=0;
            };

            vis = 0;
        } // end of else (stage>1), i.e. growing -------------------------------------------------------------------------

        //for(l=1;l<=(iR.M*iR.N);l++) {K[l] = 0;};
        Q.s(i,1) = q;
        Q.s(i,2) = sum_c;
        Q.s(i,3) = sum_cd;
        Q.s(i,4) = sum_K;
        Q.s(i,5) = sum_distLAF;
        Q.s(i,6) = num_correlations;

        //new output_matrix
        /*     [1) \bar g,
                2) \bar c,
                3) \bar u,
                4) SIFT-ratio
                5) #correlations,
                6) stage when decided,
                7) SVM-output (q),
                8) Wald SPRT test decision (1/-1),
                9) queue exhausted (1/0)]
        */
        OUT.s(i,1) = bar_g;
        OUT.s(i,2) = bar_c;
        OUT.s(i,3) = bar_u;
        OUT.s(i,4) = sift_ratio;
        OUT.s(i,5) = num_correlations;
        OUT.s(i,6) = stage;
        OUT.s(i,7) = SVM_output;
        OUT.s(i,8) = decision_status; //Wald's SPRT is not conclusive
        OUT.s(i,9) = queue_exhausted;

        //progress bar
        p++;
        //mexPrintf("[%i, %i]",p, maxP);
        if (p > maxP)
        {
            p = 0;
            //          mexPrintf("%i%%  ",int((round(i/maxP))*10));
            //        mexEvalString("drawnow;");
            //mexCallMATLAB(0,NULL,0,NULL,"drawnow");
        }

    } //end of for
//    mexPrintf("%i%%  \n",100);
}


// ========================================================================
/*
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  //input

  ARRAY_2D<double> iL(prhs[0]);
  ARRAY_2D<double> iR(prhs[1]);

//  ARRAY_2D<double> sL(prhs[2]);
//  ARRAY_2D<double> sR(prhs[3]);

//  ARRAY_2D<double> vL(prhs[4]);
//  ARRAY_2D<double> vR(prhs[5]);

  ARRAY_2D<double> SEEDs(prhs[2]);
  ARRAY_2D<double> local_search(prhs[4]);

  ARRAY_2D<double> IN(prhs[11]);  //Input matrix


  double* tmp;
  int w;
  double c_thr;
  double epsilon;
  double dmin, dmax;
  double dVmin, dVmax;
  bool i_seed_accept;
  int maxK;

  tmp = mxGetPr(prhs[3]); w = int(tmp[0]);
  tmp = mxGetPr(prhs[5]); c_thr = tmp[0];
  tmp = mxGetPr(prhs[6]); dmin = tmp[0]; dmax = tmp[1];
  tmp = mxGetPr(prhs[7]); dVmin = tmp[0]; dVmax = tmp[1];
  tmp = mxGetPr(prhs[8]); epsilon = tmp[0];
  tmp = mxGetPr(prhs[9]); i_seed_accept = bool(tmp[0]);
  tmp = mxGetPr(prhs[10]); maxK = int(tmp[0]);
  if (maxK<0) maxK = INT_MAX;


  //output
  plhs[0] = mxCreateDoubleMatrix(iL.M, iL.N, mxREAL);
  plhs[1] = mxCreateDoubleMatrix(iL.M, iL.N, mxREAL);
  plhs[2] = mxCreateDoubleMatrix(iL.M, iL.N, mxREAL);
  plhs[3] = mxCreateDoubleMatrix(iR.M, iR.N, mxREAL);
  plhs[4] = mxCreateDoubleMatrix(iL.M, iL.N, mxREAL);
  plhs[5] = mxCreateDoubleMatrix(SEEDs.M, 6, mxREAL);
  plhs[6] = mxCreateDoubleMatrix(SEEDs.M, 9, mxREAL);
  ARRAY_2D<double> D(plhs[0]);
  ARRAY_2D<double> Dv(plhs[1]);
  ARRAY_2D<double> W(plhs[2]);
  ARRAY_2D<double> K(plhs[3]);
  ARRAY_2D<double> I(plhs[4]);
  ARRAY_2D<double> Q(plhs[5]);
  ARRAY_2D<double> OUT(plhs[6]);

  //end of interface

  MAIN p(iL,iR,SEEDs,IN,D,Dv,W,K,I,Q,OUT,
         w,local_search,c_thr,dmin,dmax,dVmin,dVmax,epsilon,i_seed_accept,maxK);

  p.grow();

}
*/

/*
  sequential growing (SVM, Wald's test embedded)

  [Dh,Dv,W,K,Q] = dgrow4e_mex(Il,Ir,SEEDs,
                 w,local_search,c_thr,searchrange,searchrangeV,vis_step,
                 epsilon, IN)

   - Il,Ir...input images (rectified)
   - SEEDs...initial correspondences [xl,row,a1,...,a6,sift_ratio] (nx9 vector)
   - w...half window size (2),
   - local_search...local search matrix ([-1,-1; -1,0; -1,1; 0,-1; 0,0; 0,1; 1,-1; 1,0; 1,1]);
   - c_thr...threshold for correlation (0.5)
   - searchrange...([-5,5])
   - searchrangeV...vertical searchrange ([-5,5])
   - epsilon...first to second maxima correlation threshold (0.01)
   - i_seed_accept...accept initial seeds as correct (0)
   - maxK...maximum non-unique matches per pixel (2)
   - IN.....input matrix (see build_seq_input_matrix)  (8xnum_stages)

   ->Dh,Dv...disparity maps
   ->W.......correlation map
   ->K.......number of matches per pixel (size Ir)
   ->I.......map of LAF's id             (size Il)
   ->Q.......quality (nx6) (number of grown pixels,
                              sum of correlations,
                              sum of corr.maxima distances,
                              sum of number of matches per pixel in right image,
                              sum of distances from original LAF
                              number of correlation computations
                           )
   ->OUT.....output matrix (num_of_seeds x 10)
       [1) \bar g,
        2) \bar c,
        3) \bar u,
        4) SIFT-ratio
        5) #correlations,
        6) stage when decided,
        7) SVM-output (q),
        8) Wald SPRT test decision (1/-1),
        9) queue exhausted (1/0)]

 (same as dgrow_mex, but SEEDs are processed in best-first strategy)
 AFFINE CORRECTION

 + diagnostic information Q(6,:) = number of correlation computations

*/


