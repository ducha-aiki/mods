#ifndef HOMOGESTIMATOR_H
#define HOMOGESTIMATOR_H

#include <iostream>
#include <fstream>
#include <string>
#include "USAC.hh"
#include "ConfigParams.h"
#include "MathFunctions.h"
#include "HTools.h"

class HomogEstimator: public USAC<HomogEstimator>
{
	public:
		HomogEstimator() {};
		~HomogEstimator() {};
		bool initProblem(const ConfigParams& cfg);
		void cleanupProblem();	
		unsigned int generateMinimalSampleModels();
		bool generateRefinedModel(const std::vector<unsigned int>& sample, const unsigned int numPoints, 
								  bool weighted = false, double* weights = NULL);
		bool validateSample();
		bool validateModel(const unsigned int modelIndex);
		bool evaluateModel(const unsigned int modelIndex, unsigned int& numInliers, 
						   unsigned int& numPointsTested);
		void testSolutionDegeneracy(bool& degenerateModel, bool& upgradeModel);
		unsigned int upgradeDegenerateModel();
		void findWeights(unsigned int modelIndex, const std::vector<unsigned int>& inliers, 
						 unsigned int numInliers, double* weights);
		void storeSolution(const unsigned int modelIndex, unsigned int numInliers);

	public:
		// storage for the final results
		double* m_solution;

	private:
		double* m_inputPoints;								// input data
		double* m_dataMatrix;								// linearized input data
		std::vector<double*> m_models;						// stores vector of models
};

bool HomogEstimator::initProblem(const ConfigParams& cfg)
{
	// read in the homography specific parameters from the config file
	m_minSampleSize			= cfg.homog.minSampleSize;
	m_inlierThreshold		= cfg.homog.inlierThreshold;
	m_maxHypotheses			= cfg.homog.maxHypotheses;
	m_maxSolutionsPerSample = cfg.homog.maxSolutionsPerSample;
	m_numDataPoints			= cfg.homog.numDataPoints;

	// read in pointer to input data
	m_inputPoints = cfg.homog.inputPoints;

	// allocate other storage 
	m_solution = new double[9];

	m_models.resize(m_maxSolutionsPerSample);
	for (unsigned int i = 0; i < m_maxSolutionsPerSample; ++i)
	{
		m_models[i] = new double[9];
	}

	// precompute the data matrix
	m_dataMatrix = new double[18*m_numDataPoints];	// 2 equations per correspondence
	HTools::computeDataMatrix(m_dataMatrix, m_numDataPoints, m_inputPoints);

	return true;
}

void HomogEstimator::cleanupProblem()
{
	for (unsigned int i = 0; i < m_models.size(); ++i)
	{
		if (m_models[i]) delete[] m_models[i];
	}
	if (m_dataMatrix) delete[] m_dataMatrix;
	if (m_solution) delete[] m_solution;
}

unsigned int HomogEstimator::generateMinimalSampleModels()
{
   double A[8*9];
   double At[9*8];

	// form the matrix of equations for this minimal sample
	double *src_ptr;
	double *dst_ptr = A;
	for (unsigned int i = 0; i < m_minSampleSize; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			src_ptr = m_dataMatrix + 2*m_sample[i] + j;
			for (unsigned int k = 0; k < 9; ++k)
			{
				*dst_ptr = *src_ptr; 
				++dst_ptr;
				src_ptr += 2*m_numDataPoints;
			}
		}
	}

	MathTools::mattr(At, A, 8, 9);

	double D[9], U[9*9], V[8*8], *p;
	MathTools::svduv(D, At, U, 9, V, 8);
	p = U + 8;

	for (unsigned int i = 0; i < 9; ++i)
	{
		*(m_models[0]+i) = *p;
		p += 9;
	}
	return 1;
}

bool HomogEstimator::generateRefinedModel(const std::vector<unsigned int>& sample,
										  const unsigned int numPoints,
										  bool weighted,
										  double* weights)
{
	// form the matrix of equations for this non-minimal sample
	double *A = new double[numPoints*2*9];	
	double *src_ptr;
	double *dst_ptr = A;
	for (unsigned int i = 0; i < numPoints; ++i)
	{
		for (unsigned int j = 0; j < 2; ++j)
		{
			src_ptr = m_dataMatrix + 2*sample[i] + j;
			for (unsigned int k = 0; k < 9; ++k)
			{
				if (!weighted)
				{
					*dst_ptr = *src_ptr;
				}
				else
				{
					*dst_ptr = (*src_ptr)*weights[i];
				}
				++dst_ptr;
				src_ptr += 2*m_numDataPoints;
			}
		}
	}

	// decompose
	double V[9*9], D[9], *p;
	MathTools::svdu1v(D, A, 2*numPoints, V, 9);

	unsigned int j = 0;
	for (unsigned int i = 1; i < 9; ++i)
	{
		if (D[i] < D[j]) 
			j = i;
	}
	p = V + j;

	for (unsigned int i = 0; i < 9; ++i)
	{
		*(m_models[0]+i) = *p;
		p += 9;
	}

	delete A;

	return true;
}

bool HomogEstimator::validateSample()
{
	// check oriented constraints
   double p[3], q[3];
   double *a, *b, *c, *d;

   a = m_inputPoints + 6*m_sample[0];
   b = m_inputPoints + 6*m_sample[1];
   c = m_inputPoints + 6*m_sample[2];
   d = m_inputPoints + 6*m_sample[3];

   MathTools::mt_crossprod(p, a, b, 1);
   MathTools::mt_crossprod(q, a+3, b+3, 1);

   if ((p[0]*c[0]+p[1]*c[1]+p[2]*c[2])*(q[0]*c[3]+q[1]*c[4]+q[2]*c[5])<0)
      return false;
   if ((p[0]*d[0]+p[1]*d[1]+p[2]*d[2])*(q[0]*d[3]+q[1]*d[4]+q[2]*d[5])<0)
      return false;

   MathTools::mt_crossprod(p, c, d, 1);
   MathTools::mt_crossprod(q, c+3, d+3, 1);

   if ((p[0]*a[0]+p[1]*a[1]+p[2]*a[2])*(q[0]*a[3]+q[1]*a[4]+q[2]*a[5])<0)
      return false;
   if ((p[0]*b[0]+p[1]*b[1]+p[2]*b[2])*(q[0]*b[3]+q[1]*b[4]+q[2]*b[5])<0)
      return false;

   return true;	
}

bool HomogEstimator::validateModel(const unsigned int modelIndex)
{
	return true;
}

bool HomogEstimator::evaluateModel(const unsigned int modelIndex,
								   unsigned int& numInliers,
								   unsigned int& numPointsTested)
{
	double* model = m_models[modelIndex];
	double* inv_model = new double[9];
	double h_x[3], h_inv_xp[3], temp_err;
	double* pt;
	double *current_err_array = m_errPtr[0];
	bool good_flag = true;
	double lambdaj, lambdaj_1 = 1.0;
	numInliers = 0;
	numPointsTested = 0;
	unsigned int pt_index;

	for (unsigned int i = 0; i < 9; ++i)
	{
		inv_model[i] = model[i];
	}
	MathTools::minv(inv_model, 3);
	for (unsigned int i = 0; i < m_numDataPoints; ++i)
	{
		// get index of point to be verified
		if (m_evalPoolIndex > m_numDataPoints-1)
		{
			m_evalPoolIndex = 0;
		}
		pt_index = m_evaluationPool[m_evalPoolIndex];
		++m_evalPoolIndex;

		// compute symmetric transfer error
		pt = m_inputPoints + 6*pt_index;
		MathTools::vmul(h_x, model, pt, 3);
		MathTools::vmul(h_inv_xp, inv_model, pt+3, 3);

		double err1 = 0.0, err2 = 0.0;
		for (unsigned int j = 0; j < 2; ++j)
		{
			err1 += (h_x[j]/h_x[2] - pt[3+j]) * (h_x[j]/h_x[2] - pt[3+j]);
			err2 += (h_inv_xp[j]/h_inv_xp[2] - pt[j]) * (h_inv_xp[j]/h_inv_xp[2] - pt[j]);
		}
		temp_err = err1 + err2;
		current_err_array[pt_index] = temp_err;

		if (temp_err < m_inlierThreshold)
		{
			++numInliers;
		}

		if (m_verifMethod == USACConfig::VERIF_SPRT)
		{
			if (temp_err < m_inlierThreshold)
			{			
				lambdaj = lambdaj_1 * (m_SPRT_delta/m_SPRT_epsilon);
			}
			else
			{
				lambdaj = lambdaj_1 * ( (1 - m_SPRT_delta)/(1 - m_SPRT_epsilon) );
			}

			if (lambdaj > m_SPRTDecisionThreshold)
			{
				good_flag = false;
				numPointsTested = i+1;
				return good_flag;
			}
			else
			{
				lambdaj_1 = lambdaj;
			}
		}
	}
	numPointsTested = m_numDataPoints;
	
	delete[] inv_model;

	return good_flag;
}

void HomogEstimator::testSolutionDegeneracy(bool& degenerateModel, bool& upgradeModel)
{
	return;
}

unsigned int HomogEstimator::upgradeDegenerateModel()
{
	return 0;
}

void HomogEstimator::findWeights(unsigned int modelIndex, const std::vector<unsigned int>& inliers, 
								 unsigned int numInliers, double* weights)
{
	for (unsigned int i = 0; i < numInliers; ++i)
	{
		weights[i] = 1.0;
	}
}

void HomogEstimator::storeSolution(const unsigned int modelIndex, unsigned int numInliers)
{
	// save the current model as the best solution so far
	for (unsigned int i = 0; i < 9; ++i)
	{
		*(m_solution+i) = *(m_models[modelIndex]+i);
	}

	// save the current best set of inliers
	m_num_inliers = numInliers;
	double *current_err_array = m_errPtr[0];
	for (unsigned int i = 0; i < m_numDataPoints; ++i)
	{
		if (*(current_err_array+i) < m_inlierThreshold)
			m_inliers[i] = 1;
		else
			m_inliers[i] = 0;
	}

	// save the current best sample indices
	for (unsigned int i = 0; i < m_minSampleSize; ++i)
	{
		m_best_sample[i] = m_sample[i];
	}

	// and switch the error pointers
	double* temp = m_errPtr[0];
	m_errPtr[0] = m_errPtr[1];
	m_errPtr[1] = temp;
}

#endif

