#ifndef CONFIGPARAMS_H
#define CONFIGPARAMS_H

#include <string>
#include <cstdlib>
#include <mex.h>

namespace USACConfig
{
enum EstimationProblem		  {EP_NULL, EP_FUNDMATRIX, EP_HOMOGRAPHY, EP_ESSENTIALMATRIX, EP_LINEFITTING};
enum RandomSamplingMethod	  {SAMP_UNIFORM, SAMP_PROSAC};
enum VerifMethod			  {VERIF_STANDARD, VERIF_SPRT};
enum LocalOptimizationMethod  {LO_NONE, LO_LOSAC};
enum MatrixDecomposition      {DECOMP_QR, DECOMP_LU};

// common RANSAC parameters
struct Common
{
    // change these parameters according to the problem of choice
    Common(): confThreshold		     (0.95),
        randomSamplingMethod   (SAMP_UNIFORM),
        verifMethod			 (VERIF_STANDARD),
        localOptMethod         (LO_NONE),
        prevalidateSample 	 (false),
        prevalidateModel	     (false),
        testDegeneracy	 	 (false),
        estimationProblem	     (EP_FUNDMATRIX)
    {}

    // common USAC parameters
    double				    confThreshold;
    RandomSamplingMethod    randomSamplingMethod;
    VerifMethod			    verifMethod;
    LocalOptimizationMethod localOptMethod;
    bool					prevalidateSample;
    bool					prevalidateModel;
    bool					testDegeneracy;
    EstimationProblem		estimationProblem;
};

// PROSAC parameters
struct Prosac
{
    Prosac(): maxSamplesPROSAC		(200000),
        //sortedPointsFile		(""),		// leave blank if not reading from file
        sortedPointIndices    (NULL)		// this should point to an array of point indices
        // sorted in decreasing order of quality scores
    {}
    unsigned int  maxSamplesPROSAC;
    //std::string   sortedPointsFile; not used while getting the data from Matlab
    unsigned int* sortedPointIndices;
};

// SPRT parameters
struct Sprt
{
    Sprt(): tM      (200.0),
        mS	    (2.38),
        delta   (0.05),
        epsilon (0.2)
    {}
    double tM;
    double mS;
    double delta;
    double epsilon;
};

// LOSAC parameters
struct Losac
{
    Losac(): innerSampleSize		  (14),
        innerRansacRepetitions   (10),
        thresholdMultiplier	  (2.0),
        numStepsIterative	      (4)
    {}
    unsigned int innerSampleSize;
    unsigned int innerRansacRepetitions;
    double		 thresholdMultiplier;
    unsigned int numStepsIterative;
};

// problem specific parameters: fundamental matrix
struct Fund
{
    Fund(): minSampleSize		  (7),
        inlierThreshold		  (0.001),
        maxHypotheses		  (100000),
        maxSolutionsPerSample (3),
        decompositionAlg	  (DECOMP_QR),
        inputFilePath	      (""),			// leave blank if not using config file
        numDataPoints		  (0),			// set if not using config file
        inputPoints			  (NULL)		// should point to input data
    {}
    unsigned int		minSampleSize;
    double				inlierThreshold;
    unsigned int		maxHypotheses;
    unsigned int		maxSolutionsPerSample;
    MatrixDecomposition decompositionAlg;
    std::string			inputFilePath;
    unsigned int		numDataPoints;
    double*				inputPoints;
};

// problem specific parameters: homography
struct Homog
{
    Homog(): minSampleSize		   (4),
        inlierThreshold	   (0.001),
        maxHypotheses		   (100000),
        maxSolutionsPerSample (3),
        inputFilePath         (""),		// leave blank if not using config file
        numDataPoints		   (0),			// set if not using config file
        inputPoints		   (NULL)		// should point to input data
    {}
    unsigned int		minSampleSize;
    double				inlierThreshold;
    unsigned int		maxHypotheses;
    unsigned int		maxSolutionsPerSample;
    std::string			inputFilePath;
    unsigned int		numDataPoints;
    double*				inputPoints;
};

// problem specific parameters: essential matrix
struct Essential
{
    Essential(): minSampleSize		   (5),
        inlierThreshold	   (0.001),
        maxHypotheses		   (100000),
        maxSolutionsPerSample (10),
        inputFilePath         (""),		// leave blank if not using config file
        calibMatricesPath     (""),		// leave blank if not using config file
        numDataPoints		   (0),			// set if not using config file
        inputPoints		   (NULL),		// should point to input data
        calibMatrices		   (NULL)		// should point to the two calibration matrices
    {}
    unsigned int		minSampleSize;
    double				inlierThreshold;
    unsigned int		maxHypotheses;
    unsigned int		maxSolutionsPerSample;
    std::string			inputFilePath;
    std::string			calibMatricesPath;
    unsigned int		numDataPoints;
    double*				inputPoints;
    double*				calibMatrices;
};

// problem specific parameters: line fitting
struct Line
{
    Line(): minSampleSize		   (2),
        inlierThreshold	   (0.5),
        maxHypotheses		   (100000),
        maxSolutionsPerSample (1),
        inputFilePath         (""),		// leave blank if not using config file
        numDataPoints		   (0),			// set if not using config file
        inputPoints		   (NULL)		// should point to input data
    {}
    unsigned int		minSampleSize;
    double				inlierThreshold;
    unsigned int		maxHypotheses;
    unsigned int		maxSolutionsPerSample;
    std::string			inputFilePath;
    unsigned int		numDataPoints;
    double*				inputPoints;
};
}

struct ConfigParams
{
    USACConfig::Common    common;
    USACConfig::Prosac    prosac;
    USACConfig::Sprt      sprt;
    USACConfig::Losac     losac;
    USACConfig::Fund	  fund;
    USACConfig::Homog	  homog;
    USACConfig::Essential essential;
    USACConfig::Line	  line;

    bool initParamsFromMatlab(const mxArray * src, double * u, unsigned int len);
};

#endif
