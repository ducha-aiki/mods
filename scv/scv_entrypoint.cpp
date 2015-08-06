/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/

#include "scv_entrypoint.hpp"
#include "scv.hpp"
#include "utls3x3.hpp"
#include <math.h>
#include <assert.h>

#include <iostream>
#include <fstream>

int scv(double* img1, const int w1, const int h1,
        double* img2, const int w2, const int h2,
        doubleMatrix &AffRegs1, doubleMatrix &AffRegs2,
        std::vector<double> &lratio,std::vector<int> &good_pts,
        doubleMatrix M, doubleMatrix DS,
        std::vector<double> SIFTratio,
        const int w, const double c_thr, const double epsilon,const double dmin,const double dmax,
        const double dVmin,const double dVmax,const bool i_seed_accept,const int maxK)
{
    if ((AffRegs1.size() != AffRegs2.size()) || (AffRegs1.size() != SIFTratio.size())) return -1; //Regions have to correspond.
    if (M.size() != DS.size()) return -1; //Regions have to correspond.

    int num_of_seeds = AffRegs1.size();
    int num_of_steps = M.size();

    double lcl_search[2*9] = {-1,-1, -1,0, 0,0,1,1,1,-1,0,1,-1,0,1,-1,0,1};
    //  double lcl_search[2*25] = {-2,-2,-2,-2,-2, -1,-1, -1,-1,-1,0,0,0, 0,0,1,1,1,1,1,2,2,2,2,2,
    //                            -2,-1,0,1,2,-2,-1,0,1,2,-2,-1,0,1,2,-2,-1,0,1,2,-2,-1,0,1,2};

    ARRAY_2D<double> iL(h1,w1, img1);
    ARRAY_2D<double> iR(h2,w2, img2);
    ARRAY_2D<double> local_search(9,2,lcl_search);
    // ARRAY_2D<double> local_search(25,2,lcl_search);

    ARRAY_2D<double> D(h1,w1);
    ARRAY_2D<double> Dv(h1,w1);
    ARRAY_2D<double> W(h1,w1);
    ARRAY_2D<double> K(h2,w2);
    ARRAY_2D<double> I(h1,w1);
    ARRAY_2D<double> Q(num_of_seeds,6);
    ARRAY_2D<double> OUT(num_of_seeds,9);

    ARRAY_2D<double> IN(8,num_of_steps);

    for (int i=0; i<num_of_steps; i++)
        for (int j=0; j<8; j++)
            IN.s(j+1,i+1) = M[i][j];


    ARRAY_2D<double> SEEDs1(num_of_seeds,13);
    for (int i=0; i<num_of_seeds; i++)
    {
        double A1[3*3];
        double A2[3*3];
        for (int j=0; j<6; j++)
        {
            A1[j] = AffRegs1[i][j];
            A2[j] = AffRegs2[i][j];
        }
        A1[2]++; //because of one-based arrays
        A1[5]++;

        A2[2]++;
        A2[5]++;

        A1[6] = 0;
        A1[7] = 0;
        A1[8] = 1.0;

        A2[6] = 0;
        A2[7] = 0;
        A2[8] = 1.0;

        double A1inv[3*3];
        inv_3x3(A1,A1inv);
        double AH[3*3];
        multiply3x3(A2,A1inv,AH);
        SEEDs1.s(i+1,1) = round(A1[2]);
        SEEDs1.s(i+1,2) = round(A1[5]);

        SEEDs1.s(i+1,3) = round(A1[2]+A1[1]);
        SEEDs1.s(i+1,4) = round(A1[5]+A1[4]);

        SEEDs1.s(i+1,5) = round(A1[2]+A1[0]);
        SEEDs1.s(i+1,6) = round(A1[5]+A1[3]);
        for (int j=0; j<6; j++)
            SEEDs1.s(i+1,7+j) = AH[j];

        SEEDs1.s(i+1,13) = SIFTratio[i];
    };
    IN.need_to_release =  0;
    SEEDs1.need_to_release = 0;

    MAIN p(iL,iR,SEEDs1,IN,D,Dv,W,K,I,Q,OUT,
           w,local_search,c_thr,dmin,dmax,dVmin,dVmax,epsilon,i_seed_accept,maxK);
    p.grow();
    lratio.resize(num_of_seeds);

    for (int i=0; i<num_of_seeds; i++)
    {
        double P,N, s_p, s_n, m_p,m_n;
        int step = OUT.s(i+1,6)-1;
        double x = OUT.s(i+1,7);

        s_p = DS[step][0];
        s_n = DS[step][1];
        m_p = DS[step][2];
        m_n = DS[step][3];
        P = (1/sqrt(2*M_PI)*s_p)*exp(-(x-m_p)*(x-m_p)/(2*s_p*s_p));
        N = (1/sqrt(2*M_PI)*s_n)*exp(-(x-m_n)*(x-m_n)/(2*s_n*s_n));
        lratio[i] = P/N;
    }

    int good_pts_numb =0;
    good_pts.resize(num_of_seeds);

    for (int i=0; i<num_of_seeds; i++)
        if (lratio[i] > 1)
        {
            good_pts[i] = 1;
            good_pts_numb++;
        }
        else good_pts[i] = 0;


    /*
    ofstream qual_file("Q.txt");
    if (qual_file.is_open())
    {
            for (int j=1; j<=num_of_seeds;j++)
              {for (int i=1; i<=6; i++)
                {qual_file << p.Q.s(j,i) << " ";
                }
              qual_file << std::endl;
              }
      }
    else {
        cerr << "Cannot SVM-parameters file DS_svm.dat" << endl;
        return 0;
    }
    qual_file.close();
    */
    return good_pts_numb;
}
/*
  sequential growing (SVM, Wald's test embedded)

  [Dh,Dv,W,K,Q] = dgrow4e_mex(Il,Ir,SEEDs,
                 w,local_search,c_thr,searchrange,searchrangeV,vis_step,
                 epsilon, IN)

//      - Il,Ir...input images (rectified)
   - SEEDs...initial correspondences [xl,row,a1,...,a6,sift_ratio] (nx9 vector)
//       - w...half window size (2),
//       - local_search...local search matrix ([-1,-1; -1,0; -1,1; 0,-1; 0,0; 0,1; 1,-1; 1,0; 1,1]);
//       - c_thr...threshold for correlation (0.5)
//      - searchrange...([-5,5])
//    - searchrangeV...vertical searchrange ([-5,5])
//       - epsilon...first to second maxima correlation threshold (0.01)
//       - i_seed_accept...accept initial seeds as correct (0)
//      - maxK...maximum non-unique matches per pixel (2)
   - IN.....input matrix (see build_seq_input_matrix)  (8xnum_stages)

//      ->Dh,Dv...disparity maps
//      ->W.......correlation map
//      ->K.......number of matches per pixel (size Ir)
//      ->I.......map of LAF's id             (size Il)
//      ->Q.......quality (nx6) (number of grown pixels,
//                                sum of correlations,
//                                sum of corr.maxima distances,
//                                sum of number of matches per pixel in right image,
//                                 sum of distances from original LAF
//                                 number of correlation computations
                           )
//    ->OUT.....output matrix (num_of_seeds x 9)
       [1) \bar g,
        2) \bar c,
        3) \bar u,
        4) SIFT-ratio
        5) #correlations,
        6) stage when decided,
        7) SVM-output (q),
        8) Wald SPRT test decision (1/-1),
        9) queue exhausted (1/0)]

*/
