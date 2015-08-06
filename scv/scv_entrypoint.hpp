/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/

#ifndef SCV_ENTRYPOINT_HPP
#define SCV_ENTRYPOINT_HPP
#include <vector>

typedef std::vector< std::vector <double> > doubleMatrix;

int scv(double* img1, const int w1, const int h1,
        double* img2, const int w2, const int h2,
        doubleMatrix &AffRegs1, doubleMatrix &AffRegs2,
        std::vector<double> &lratio,std::vector<int> &good_pts,
        doubleMatrix M, doubleMatrix DS,
        std::vector<double> SIFTratio,
        const int w = 2,
        const double c_thr = 0.5,
        const double epsilon = 0.001,
        const double dmin = -5.0,const double dmax = 5.0,
        const double dVmin = -5.0,const double dVmax = 5.0,
        const bool i_seed_accept = 0,
        const int maxK = 2 );
//AffRegs = vector of [a11*s a12*s x a21*s a22*s y] (N_regs x 6)
//dim M = (N_stages x 8)
//dim DS = (N_stages x 4)

#endif // SCV_ENTRYPOINT_HPP
