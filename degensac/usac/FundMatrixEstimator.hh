#ifndef FUNDMATRIXESTIMATOR_H
#define FUNDMATRIXESTIMATOR_H

#include <iostream>
#include <fstream>
#include <string>
#include "USAC.hh"
#include "ConfigParams.h"
#include "MathFunctions.h"
#include "FTools.h"
#include "HTools.h"

class FundMatrixEstimator: public USAC<FundMatrixEstimator>
{
	public:
		FundMatrixEstimator() {};
		~FundMatrixEstimator() {};
		bool initProblem(const ConfigParams& cfg);
		void cleanupProblem();
		unsigned int generateMinimalSampleModels();
		bool generateRefinedModel(std::vector<unsigned int>& sample, const unsigned int numPoints, 
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
		// storage for the final result
		double* m_solution;

		double* m_degen_solution;
		
	private:
		double* m_inputPoints;								// input data
		double* m_dataMatrix;								// linearized input data
		double* m_degenDataMatrix;							// only for degeneracy testing
		USACConfig::MatrixDecomposition m_decompositionAlg;	// QR/LU decomposition
		std::vector<double*> m_models;						// stores vector of models
};

bool FundMatrixEstimator::initProblem(const ConfigParams& cfg)
{
	// read in the f-matrix specific parameters from the config struct
	m_minSampleSize			= cfg.fund.minSampleSize;
	m_inlierThreshold		= cfg.fund.inlierThreshold;
	m_maxHypotheses			= cfg.fund.maxHypotheses;
	m_maxSolutionsPerSample = cfg.fund.maxSolutionsPerSample;
	m_numDataPoints			= cfg.fund.numDataPoints;
	m_decompositionAlg		= cfg.fund.decompositionAlg;

	// read in pointer to input data
	m_inputPoints = cfg.fund.inputPoints;

	// allocate other storage 
	m_solution = new double[9];

	m_models.resize(m_maxSolutionsPerSample);
	for (unsigned int i = 0; i < m_maxSolutionsPerSample; ++i)
	{
		m_models[i] = new double[9];
	}

	// precompute the data matrix
	m_dataMatrix = new double[9*m_numDataPoints];	// 9 values per correspondence
	FTools::computeDataMatrix(m_dataMatrix, m_numDataPoints, m_inputPoints);

	// if degeneracy testing option is set, also compute the data matrix for H
	if (m_testDegeneracy)
	{
		m_degenDataMatrix = new double[2*9*m_numDataPoints];	// 2 equations per correspondence
		HTools::computeDataMatrix(m_degenDataMatrix, m_numDataPoints, m_inputPoints);
	}
	else
	{
		m_degenDataMatrix = NULL;
	}

	if (m_testDegeneracy)
	{
		m_degen_solution = new double[9];
	}

	return true;
}

void FundMatrixEstimator::cleanupProblem()
{
	for (unsigned int i = 0; i < m_models.size(); ++i)
	{
		if (m_models[i]) delete[] m_models[i];
	}
	if (m_dataMatrix) delete[] m_dataMatrix;
	if (m_degenDataMatrix) delete[] m_degenDataMatrix;
	if (m_solution) delete[] m_solution;

	if (m_testDegeneracy)
	{
		delete[] m_degen_solution;
	}
}

unsigned int FundMatrixEstimator::generateMinimalSampleModels()
{
	double A[9*9];
	unsigned int nsols = 0;

	// form the matrix of equations for this minimal sample
	double *src_ptr;
	double *dst_ptr = A;
	for (unsigned int i = 0; i < m_minSampleSize; ++i)
	{
		src_ptr = m_dataMatrix + m_sample[i];
		for (unsigned int j = 0; j < 9; ++j)
		{
			*dst_ptr = *src_ptr;
			++dst_ptr;
			src_ptr += m_numDataPoints;
		}
	}

	// LU/QR factorization
	double sol[9*9];
	double poly[4], roots[3];
	double *f1, *f2;
	int nullbuff [18];
	f1 = sol;
	f2 = sol+9;
	if (m_decompositionAlg == USACConfig::DECOMP_QR)
	{
		FTools::nullspaceQR7x9(A, sol);
	}
	else if (m_decompositionAlg == USACConfig::DECOMP_LU)
	{
		for (unsigned int i = 7*9; i < 9*9; ++i)
		{
			A[i] = 0.0;
		}
		int nullsize = FTools::nullspace(A, f1, 9, nullbuff);
		if (nullsize != 2)
		{
			return 0;
		}
	}

	// solve polynomial
	FTools::makePolynomial(f1, f2, poly);  
	nsols = FTools::rroots3(poly, roots);

	// form up to three fundamental matrices
	for (unsigned int i = 0; i < nsols; ++i)
	{
		for (unsigned int j = 0; j < 9; ++j)
		{
			*(m_models[i]+j) = f1[j] * roots[i] + f2[j] * (1 -roots[i]);
		}
	}

	return nsols;
}

bool FundMatrixEstimator::generateRefinedModel(std::vector<unsigned int>& sample,
											   const unsigned int numPoints,
											   bool weighted,
											   double* weights)
{
	// form the matrix of equations for this non-minimal sample
	double *A = new double[numPoints*9];	
	double *src_ptr;
	double *dst_ptr = A;
	for (unsigned int i = 0; i < numPoints; ++i)
	{
		src_ptr = m_dataMatrix + sample[i];
		for (unsigned int j = 0; j < 9; ++j)
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
			src_ptr += m_numDataPoints;
		}
	}

	double Cv[9*9];
	FTools::formCovMat(Cv, A, numPoints, 9);

	double V[9*9], D[9], *p;
	MathTools::svdu1v(D, Cv, 9, V, 9);

	unsigned int j = 0;
	for (unsigned int i = 1; i < 9; ++i)
	{
		if (D[i] < D[j]) 
		{
			j = i;
		}
	}
	p = V + j;

	for (unsigned int i = 0; i < 9; ++i)
	{
		*(m_models[0]+i) = *p;
		p += 9;
	}
	FTools::singulF(m_models[0]);

	delete[] A;

	return true;
}

bool FundMatrixEstimator::validateSample()
{
	return true;
}

bool FundMatrixEstimator::validateModel(const unsigned int modelIndex)
{
	// check oriented constraints
	double e[3], sig1, sig2;
	FTools::computeEpipole(e, m_models[modelIndex]);

	sig1 = FTools::getOriSign(m_models[modelIndex], e, m_inputPoints + 6*m_sample[0]);
	for(unsigned int i = 1; i < m_sample.size(); ++i)
	{
		sig2 = FTools::getOriSign(m_models[modelIndex], e, m_inputPoints + 6*m_sample[i]);
		if (sig1 * sig2 < 0) 
		{
			return false;
		}
	}
	return true;	
}

bool FundMatrixEstimator::evaluateModel(const unsigned int modelIndex,
										unsigned int& numInliers,
										unsigned int& numPointsTested)
{
    double rx, ry, rwc, ryc, rxc, r, temp_err;
	double* model = m_models[modelIndex];
	double* pt;
	double *current_err_array = m_errPtr[0];
	bool good_flag = true;
	double lambdaj, lambdaj_1 = 1.0;
	numInliers = 0;
	numPointsTested = 0;
	unsigned int pt_index;

	for (unsigned int i = 0; i < m_numDataPoints; ++i)
	{
		// get index of point to be verified
		if (m_evalPoolIndex > m_numDataPoints-1)
		{
			m_evalPoolIndex = 0;
		}
		pt_index = m_evaluationPool[m_evalPoolIndex];
		++m_evalPoolIndex;

		// compute sampson error
		pt = m_inputPoints + 6*pt_index;
		rxc = (*model) * (*(pt+3)) + (*(model+3)) * (*(pt+4)) + (*(model+6));
		ryc = (*(model+1)) * (*(pt+3)) + (*(model+4)) * (*(pt+4)) + (*(model+7));
		rwc = (*(model+2)) * (*(pt+3)) + (*(model+5)) * (*(pt+4)) + (*(model+8));
		r =((*(pt)) * rxc + (*(pt+1)) * ryc + rwc);
		rx = (*model) * (*(pt)) + (*(model+1)) * (*(pt+1)) + (*(model+2));
		ry = (*(model+3)) * (*(pt)) + (*(model+4)) * (*(pt+1)) + (*(model+5)); 
		temp_err = r*r / (rxc*rxc + ryc*ryc + rx*rx + ry*ry);
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
	return good_flag;
}

void FundMatrixEstimator::testSolutionDegeneracy(bool& degenerateModel, bool& upgradeModel)
{
	double h_inlier_threshold = 2*m_inlierThreshold;
	unsigned int num_inner_trials = 20;
	unsigned int inner_sample_size = 20;
	degenerateModel = false;
	upgradeModel = false;

	// make up the tuples to be used to check for degeneracy
	unsigned int degen_sample_indices[] = {0, 1, 2, 3,
										   3, 4, 5, 6,
										   0, 1, 5, 6,
										   0, 2, 4, 5,
										   1, 2, 4, 6,
										   0, 3, 4, 6,
										   1, 3, 4, 5,
										   2, 3, 5, 6};

	// the above tuples need to be tested on the remaining points for each case
	unsigned int test_point_indices[] = {4, 5, 6,
									     0, 1, 2,
									     2, 3, 4,
									     1, 3, 6,
									     0, 3, 5,
									     1, 2, 5,
									     0, 2, 6,
									     0, 1, 4};

	unsigned int *sample_pos = degen_sample_indices;
	unsigned int *test_pos = test_point_indices;
	double h[9];

	std::vector<unsigned int> sample(4), test(3);
	std::vector<double> errs;
	for(unsigned int i = 0; i < 8; ++i)
	{
		// compute H from the current set of 4 points
		sample.resize(4);
		for (unsigned int j = 0; j < 4; ++j)
		{
			sample[j] = m_sample[sample_pos[j]];
		}
		FTools::computeHFromMinCorrs(sample, 4, m_numDataPoints, m_degenDataMatrix, h);

		// check test points to see how many are consistent
		for (unsigned int j = 0; j < 3; ++j)
		{
			test[j] = m_sample[test_pos[j]];
		}
		unsigned int num_inliers = FTools::getHError(test, 3, errs, m_inputPoints, h, h_inlier_threshold);

		// if at least 1 inlier in the test points, then h-degenerate sample found
		if (num_inliers > 0)
		{
			// set flag
			degenerateModel = true;

			// recompute H using all inliers
			for (unsigned int j = 0; j < errs.size(); ++j)
			{
				if (errs[j] < h_inlier_threshold)
				{
					sample.push_back(test[j]);
				}
			}

			FTools::computeHFromCorrs(sample, sample.size(), m_numDataPoints, m_degenDataMatrix, h);

			// find support of homography
			num_inliers = FTools::getHError(m_evaluationPool, m_numDataPoints, errs, m_inputPoints, h, h_inlier_threshold);
			std::cout << "Degenerate sample found with " << num_inliers << " inliers" << std::endl;

			// locally refine
			std::vector<unsigned int> inner_inliers;
			unsigned int best_inner_inlier_count = num_inliers, inner_inlier_count = 0;
			double best_h[9], inner_h[9];
			if (num_inliers < inner_sample_size)
			{
				num_inner_trials = 1;
				inner_sample_size = num_inliers;
			}
			for (unsigned int j = 0; j < num_inner_trials; ++j)
			{
				sample.resize(inner_sample_size);
				inner_inliers.clear();
				for (unsigned int k = 0; k < errs.size(); ++k)
				{
					if (errs[k] < h_inlier_threshold)
					{
						inner_inliers.push_back(m_evaluationPool[k]);
					}
				}
				if (inner_inliers.size() < m_minSampleSize)
					continue;

				generateUniformRandomSample(sample, inner_inliers.size(), inner_sample_size);
				for (unsigned int k = 0; k < sample.size(); ++k)
				{
					sample[k] = inner_inliers[sample[k]];
				}
				FTools::computeHFromCorrs(sample, sample.size(), m_numDataPoints, m_degenDataMatrix, inner_h);
				inner_inlier_count = FTools::getHError(m_evaluationPool, m_numDataPoints, errs, m_inputPoints, inner_h, h_inlier_threshold);

				if (inner_inlier_count > best_inner_inlier_count)
				{
					for (unsigned int k = 0; k < 9; ++k)
					{
						best_h[k] = inner_h[k];
					}
					best_inner_inlier_count = inner_inlier_count;
				}
			}

			// find support of best homography
			if (best_inner_inlier_count > num_inliers)
			{
				for (unsigned int k = 0; k < 9; ++k)
				{
					h[k] = best_h[k];
				}
			}
			num_inliers = FTools::getHError(m_evaluationPool, m_numDataPoints, errs, m_inputPoints, h, h_inlier_threshold);
			
			std::cout << "Degenerate inliers after refinement " << num_inliers << " inliers" << std::endl;	

			// if largest degenerate model found so far, store results
			if (num_inliers > m_num_degen_inliers)
			{
				// set flag
				upgradeModel = true;

				m_num_degen_inliers = num_inliers;
				// store homography
				for (unsigned int j = 0; j < 9; ++j)
				{
					m_degen_solution[j] = h[j];
				}
				// store inliers and outliers - for use in model completion
				for (unsigned int j = 0; j < m_numDataPoints; ++j)
				{
					if (errs[j] < h_inlier_threshold)
					{
						m_degen_inliers[m_evaluationPool[j]] = 1;
						m_degen_outliers[m_evaluationPool[j]] = 0;
					}
					else
					{
						m_degen_outliers[m_evaluationPool[j]] = 1;
						m_degen_inliers[m_evaluationPool[j]] = 0;
					}
				}
				// store the inliers to the degenerate model from the minimal sample
				num_inliers = FTools::getHError(m_sample, m_sample.size(), errs, m_inputPoints, h, h_inlier_threshold);
				unsigned int count = 0;
				for (unsigned int j = 0; j < m_sample.size(); ++j)
				{
					if (errs[j] < h_inlier_threshold)
					{
						m_degen_sample[count++] = m_sample[j];
					}
				}
			}
			else
			{
				continue;
			}
		}
		sample_pos += 4;
		test_pos += 3;
	}
	return;
}

unsigned int FundMatrixEstimator::upgradeDegenerateModel()
{
	unsigned int max_tries = 400;
	unsigned int best_upgrade_inliers = 0;
	unsigned int num_outliers = m_numDataPoints - m_num_degen_inliers;
	std::vector<unsigned int> outlier_sample(2);

	std::vector<unsigned int> outlier_indices;
	for (unsigned int i = 0; i < m_numDataPoints; ++i)
	{
		if (m_degen_outliers[i])
		{
			outlier_indices.push_back(i);
		}
	}

	double* pt1_index, *pt2_index;
	double x1[3], x1p[3], x2[3], x2p[3];
	double temp[3], l1[3], l2[3], ep[3];
	double skew_sym_ep[9];
	for (unsigned int i = 0; i < max_tries; ++i)
	{
		generateUniformRandomSample(outlier_sample, num_outliers, 2);
	
		pt1_index = m_inputPoints + 6*outlier_indices[outlier_sample[0]];
		pt2_index = m_inputPoints + 6*outlier_indices[outlier_sample[1]];

		x1[0]  = pt1_index[0]; x1[1]  = pt1_index[1]; x1[2]  = 1.0;
		x1p[0] = pt1_index[3]; x1p[1] = pt1_index[4]; x1p[2] = 1.0;
		x2[0]  = pt2_index[0]; x2[1]  = pt2_index[1]; x2[2]  = 1.0;
		x2p[0] = pt2_index[3]; x2p[1] = pt2_index[4]; x2p[2] = 1.0;

		MathTools::vmul(temp, m_degen_solution, x1, 3);
		MathTools::mt_crossprod(l1, temp, x1p, 1);

		MathTools::vmul(temp, m_degen_solution, x2, 3);
		MathTools::mt_crossprod(l2, temp, x2p, 1);

		MathTools::mt_crossprod(ep, l1, l2, 1);

		MathTools::skew_sym(skew_sym_ep, ep);
		MathTools::mmul(m_models[0], skew_sym_ep, m_degen_solution, 3);

		unsigned int num_inliers, num_pts_tested;
		evaluateModel(0, num_inliers, num_pts_tested);

		if (num_inliers > best_upgrade_inliers)
		{
			m_degen_sample[5] = outlier_indices[outlier_sample[0]];
			m_degen_sample[6] = outlier_indices[outlier_sample[1]];
			for (unsigned int j = 0; j < m_minSampleSize; ++j)
			{
				m_sample[j] = m_degen_sample[j];
			}
			storeSolution(0, num_inliers);
			best_upgrade_inliers = num_inliers;

			//// try to refine model
			//std::cout << "Performing LO for degenerate data" << std::endl;
			//unsigned int lo_inlier_count = locallyOptimizeSolution(best_upgrade_inliers);

			//if (lo_inlier_count > best_upgrade_inliers)
			//{
			//	best_upgrade_inliers = lo_inlier_count;
			//}
		}
	}

	std::cout << "Upgraded model has " << best_upgrade_inliers << " inliers" << std::endl;
	return best_upgrade_inliers;
}

void FundMatrixEstimator::findWeights(unsigned int modelIndex, const std::vector<unsigned int>& inliers, 
									  unsigned int numInliers, double* weights)
{
    double rx, ry, ryc, rxc;
	double* model = m_models[modelIndex];
	double* pt;
	unsigned int pt_index;

	for (unsigned int i = 0; i < numInliers; ++i)
	{
		// get index of point to be verified
		pt_index = inliers[i];

		// compute weight (ref: torr dissertation, eqn. 2.25)
		pt = m_inputPoints + 6*pt_index;
		rxc = (*model) * (*(pt+3)) + (*(model+3)) * (*(pt+4)) + (*(model+6));
		ryc = (*(model+1)) * (*(pt+3)) + (*(model+4)) * (*(pt+4)) + (*(model+7));
		rx = (*model) * (*(pt)) + (*(model+1)) * (*(pt+1)) + (*(model+2));
		ry = (*(model+3)) * (*(pt)) + (*(model+4)) * (*(pt+1)) + (*(model+5)); 

		weights[i] = 1/sqrt(rxc*rxc + ryc*ryc + rx*rx + ry*ry);
	}
}

void FundMatrixEstimator::storeSolution(const unsigned int modelIndex, unsigned int numInliers)
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

