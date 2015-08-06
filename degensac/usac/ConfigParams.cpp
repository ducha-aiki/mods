#include <string.h>
#include "ConfigParams.h"

bool ConfigParams::initParamsFromMatlab(const mxArray * src, double * u, unsigned int len)
{

    mxArray * tmp = 0, * tmp2 = 0;
    char * tmpchar;
    int tmpSize;

    try
    {

        // read in problem specific data /////////////////////////////////////
        //only threshold now - it is common for everything, maybe later move?
        tmp = mxGetField(src, 0, "threshold");
        if (tmp)
        {
            double thr = mxGetScalar(tmp);
            if(thr < 0)
            {
                throw "Threshold (or sigma) must be non-negative.";
            }
            switch (common.estimationProblem)
            {
            case USACConfig::EP_FUNDMATRIX:
                fund.inputPoints = u;
                fund.numDataPoints = len;
                fund.inlierThreshold = thr;
                sprt.tM = 200.0;
                sprt.mS = 2.38; //would be better get from auto-calibration?
                sprt.delta = 0.05;
                sprt.epsilon = 0.2; //default values differs depending on model (could be set manualy in cfg)
                break;

            case USACConfig::EP_HOMOGRAPHY:
                homog.inputPoints = u;
                homog.numDataPoints = len;
                homog.inlierThreshold = thr;
                sprt.tM = 200.0;
                sprt.mS = 1.0;
                sprt.delta = 0.01;
                sprt.epsilon = 0.1;
                break;

            case USACConfig::EP_ESSENTIALMATRIX:
                //cannot happen
                break;

            case USACConfig::EP_LINEFITTING:
                //cannot happen
                break;

            case USACConfig::EP_NULL:
                //cannot happen
                //std::cerr << "Set estimation problem, currently NULL" << std::endl;
                return false;
            }
        }


        // get common parameters ////////////////////////////////////////////

        //confidence = confThreshold
        tmp = mxGetField(src, 0, "confidence");
        if (tmp)
        {
            common.confThreshold = mxGetScalar(tmp);
            if(common.confThreshold < 0 || common.confThreshold >= 1)
            {
                throw "Confidence must be between 0 and 1.";
            }
        }

        //sampling = randomSamplingMethod
        tmp = mxGetField(src, 0, "sampling");
        if (tmp)
        {
            tmpSize = mxGetN(tmp);
            tmpchar = (char *)malloc((tmpSize+1)*sizeof(char));
            mxGetString(tmp, tmpchar, tmpSize+1);
            if ( !strcmp(tmpchar, "UNIFORM") )
            {
                common.randomSamplingMethod = USACConfig::SAMP_UNIFORM;
            }
            else if ( !strcmp(tmpchar, "PROSAC") )
            {
                common.randomSamplingMethod = USACConfig::SAMP_PROSAC;
            }
            else
            {
                free(tmpchar);
                throw "Unknown sampling method!";
            }
            free(tmpchar);
        }

        //verification = verifMethod
        tmp = mxGetField(src, 0, "verification");
        if (tmp)
        {
            int tmpSize = mxGetN(tmp);
            tmpchar = (char *)malloc((tmpSize+1)*sizeof(char));
            mxGetString(tmp, tmpchar, tmpSize+1);
            if ( !strcmp(tmpchar, "STD") )
            {
                common.verifMethod = USACConfig::VERIF_STANDARD;
            }
            else if ( !strcmp(tmpchar, "SPRT") )
            {
                common.verifMethod = USACConfig::VERIF_SPRT;
            }
            else
            {
                free(tmpchar);
                throw "Unknown verification method!";
            }
            free(tmpchar);
        }

        //lo_on = localOptMethod
        tmp = mxGetField(src, 0, "lo_on");
        if (tmp)
        {
            if(!mxGetScalar(tmp))
            {
                common.localOptMethod = USACConfig::LO_NONE;
            }
            else
            {
                common.localOptMethod = USACConfig::LO_LOSAC;
            }
        }

        //prevalidateSample
        tmp = mxGetField(src, 0, "prevalidateSample");
        if (tmp)
        {
            common.prevalidateSample = mxGetScalar(tmp) > 0;
        }

        //prevalidateModel
        tmp = mxGetField(src, 0, "prevalidateModel");
        if (tmp)
        {
            common.prevalidateModel = mxGetScalar(tmp) > 0;
        }

        //testDegeneracy
        tmp = mxGetField(src, 0, "testDegeneracy");
        if (tmp)
        {
            common.testDegeneracy = mxGetScalar(tmp) > 0;
        }

        //model = EstimationProblem
        tmp = mxGetField(src, 0, "model");
        if (tmp)
        {
            int tmpSize = mxGetN(tmp);
            tmpchar = (char *)malloc((tmpSize+1)*sizeof(char));
            mxGetString(tmp, tmpchar, tmpSize+1);
            if ( !strcmp(tmpchar, "EG") )
            {
                common.estimationProblem = USACConfig::EP_FUNDMATRIX;
            }
            else if ( !strcmp(tmpchar, "HG") )
            {
                common.estimationProblem = USACConfig::EP_HOMOGRAPHY;
            }
            else
            {
                free(tmpchar);
                throw "Unknown model!";
            }
            free(tmpchar);
        }

        // read in PROSAC parameters if required //////////////////////////////
        if (common.randomSamplingMethod == USACConfig::SAMP_PROSAC)
        {
            tmp = mxGetField(src, 0, "prosac");
            if (tmp)
            {
                //prosac.maxSamples = maxSamplesPROSAC
                tmp2 = mxGetField(tmp, 0, "maxSamples");
                if (tmp2)
                {
                    prosac.maxSamplesPROSAC = (unsigned int)mxGetScalar(tmp2);
                }
            }
        }

        // read in SPRT parameters if required ////////////////////////////////
        //common RANSAC parameters, but used only in SPRT (maybe? :-)
        if (common.verifMethod == USACConfig::VERIF_SPRT)
        {
            //delta = sprt.delta
            tmp = mxGetField(src, 0, "delta");
            if (tmp)
            {
                sprt.delta = mxGetScalar(tmp);
                if(sprt.delta < 0 || sprt.delta >= 1)
                {
                    throw "Delta must be between 0 and 1.";
                }
            }

            //epsilon = sprt.epsilon
            tmp = mxGetField(src, 0, "epsilon");
            if (tmp)
            {
                sprt.epsilon = mxGetScalar(tmp);
                if(sprt.epsilon < 0 || sprt.epsilon > 1)
                {
                    throw "Epsilon must be between 0 and 1.";
                }
            }
            //tM & mS not set, will be constants given by model, or by autocalibration (//TODO later)
        }

        // read in LO parameters if required //////////////////////////////////
        if (common.localOptMethod == USACConfig::LO_LOSAC)
        {
            tmp = mxGetField(src, 0, "lo");
            if (tmp)
            {
                //lo.innerSampleSize
                tmp2 = mxGetField(tmp, 0, "innerSampleSize");
                if (tmp2)
                {
                    losac.innerSampleSize = (unsigned int)mxGetScalar(tmp2);
                }

                //lo.innerRansacRepetitions
                tmp2 = mxGetField(tmp, 0, "innerRansacRepetitions");
                if (tmp2)
                {
                    losac.innerRansacRepetitions = (unsigned int)mxGetScalar(tmp2);
                }

                //lo.thresholdMultiplier
                tmp2 = mxGetField(tmp, 0, "thresholdMultiplier");
                if (tmp2)
                {
                    losac.thresholdMultiplier = mxGetScalar(tmp2);
                    if(losac.thresholdMultiplier < 1)
                    {
                        throw "LO.thresholdMultiplier must be greater than 1.";
                    }
                }

                //lo.numStepsIterative
                tmp2 = mxGetField(tmp, 0, "numStepsIterative");
                if (tmp2)
                {
                    losac.numStepsIterative = (unsigned int)mxGetScalar(tmp2);
                    if(losac.numStepsIterative == 0)
                    {
                        throw "LO.numStepsIterative must be at least 1.";
                    }
                }
            }
        }


    }
    catch(const char * c)
    {
        mexErrMsgTxt(c);
        mexErrMsgTxt("\n\n");
        return false;
    }
    catch(...)
    {
        return false;
    }

    return true;
}

