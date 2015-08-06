/*
This is free library for a non-commercial purpose. Commercial use
requires a licence. The software is distributed AS IS, WITHOUT ANY
IMPLIED OR EXPRESSED WARRANTY and WITHOUT ANY FURTHER SUPPORT for
the desired and/or other purpose.
(c) Jan Cech (cechj@cmp.felk.cvut.cz) FEE CTU Prague, Apr 30, 2009
*/

#ifndef SCV_HPP
#define SCV_HPP

#include <queue>
#include "arrays.h"

using namespace std;  // std c++ libs implemented in std


struct seed
{
    double a3_0;
    double a6_0;
    int xl;
    int yl;
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;
    double a6;
    double c;
};


class MAIN
{
public:
    ARRAY_2D<double> iL;
    ARRAY_2D<double> iR;
    ARRAY_2D<double> SEEDs;
    ARRAY_2D<double> IN; //input matrix (see build_seq_input_matrix.m)
    ARRAY_2D<double> D;  //output (horizontal) disparity map
    ARRAY_2D<double> Dv; //output vertical disparity map
    ARRAY_2D<double> W;  //output correlation map
    ARRAY_2D<double> K;  //output number of matches per pixel
    ARRAY_2D<double> I;  //id map (LAF's id)
    ARRAY_2D<double> Q;  //output quality per LAF (number of LAF-seedsx6)
    ARRAY_2D<double> OUT;  //OUTPUT quality matrix per LAF (number of LAF-seedsx10)



    int num_correlations; //number of evaluated correlations (diagnostic info only)

    int w;
    int maxK;
    ARRAY_2D<int> local_search; //local search-matrix [x,y] (nx2)
    double c_thr;
    double epsilon;
    unsigned vis_step;
    double dmin, dmax; //(horizontal) disparity searchrange
    double dVmin, dVmax; //(veritcal) disparity searchrange
    bool i_seed_accept; //accept all initial seeds

    priority_queue<seed> SEEDS;

    //double wcorr(int xl, int xr, int yl, int yr);
    double wcorr_a(seed& s);
    double wcorr_a_interp(seed& s);
    void local_optim(seed& s, double& c, int& dp, int& dpv, double &dc);
    void grow();

    MAIN(ARRAY_2D<double>& iL_, ARRAY_2D<double> &iR_,
         ARRAY_2D<double>& SEEDs_, ARRAY_2D<double> IN_,
         ARRAY_2D<double>& D_, ARRAY_2D<double>& Dv_, ARRAY_2D<double>& W_,
         ARRAY_2D<double>& K_, ARRAY_2D<double>& I_,
         ARRAY_2D<double>& Q_, ARRAY_2D<double>& OUT_,
         int w_, ARRAY_2D<double>& local_search_, double c_thr_,
         double dmin_, double dmax_, double dVmin_, double dVmax_,
         double epsilon_, bool i_seed_accept_, int maxK_);

    ~MAIN() {};
};

#endif // SCV_HPP
