#include <stdlib.h>
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <matutl.h>

#ifdef _WIN32
#include <windows.h>

#else
#include <sys/time.h>

#endif
/*
#include "utools.h"
#include "Htools.h"
#include "Ftools.h"
#include "rtools.h"
*/
#include "usac.h"

#include "usac/HomogEstimator.hh"
#include "usac/FundMatrixEstimator.hh"

int usac (double *u, unsigned int len, ConfigParams cfg, double *M, unsigned char * inl, unsigned int * stats)
{

#ifdef _WIN32
    srand((unsigned int)GetTickCount());
#else
    struct timeval tv;
    gettimeofday(&tv, NULL);
    srand(tv.tv_sec * tv.tv_usec);
#endif

    if(cfg.common.estimationProblem == USACConfig::EP_FUNDMATRIX)
    {
        FundMatrixEstimator * Fest = new FundMatrixEstimator;
        // set up the homography estimation problem
        Fest->init(cfg);

        if (!Fest->solveMaster())
        {
            std::cerr << "Error while using USAC.";
            return(1);
        }

        memcpy(M, Fest->m_solution, 9*sizeof(double));

        for(unsigned int i = 0; i < len; ++i)
        {
            inl[i] = (unsigned char)Fest->m_inliers[i];
        }
        stats[0] = (unsigned int)Fest->getHyp_count();
        stats[1] = (unsigned int)Fest->getLo_count();
        stats[2] = (unsigned int)Fest->getSample_rejected_count();

        // clean up
        Fest->cleanup();
        delete Fest;
    }
    else if (cfg.common.estimationProblem == USACConfig::EP_HOMOGRAPHY)
    {
        HomogEstimator * Hest = new HomogEstimator;
        // set up the homography estimation problem
        Hest->init(cfg);

        if (!Hest->solveMaster())
        {
            std::cerr << "Error while using USAC.";
            return(1);
        }

        memcpy(M, Hest->m_solution, 9*sizeof(double));


        for(unsigned int i = 0; i < len; ++i)
        {
            inl[i] = (unsigned char)Hest->m_inliers[i];
        }
        stats[0] = (unsigned int)Hest->getHyp_count();
        stats[1] = (unsigned int)Hest->getLo_count();
        stats[2] = (unsigned int)Hest->getSample_rejected_count();


        // clean up
        Hest->cleanup();
        delete Hest;

    }
    else
    {
        std::cerr << "Unknown model!\n";
        return 1;
    }

    //oriented constraint not used yet... //TODO

    //cfg.homog.maxHypotheses = max_sam; //TODO



    return 0;


}
