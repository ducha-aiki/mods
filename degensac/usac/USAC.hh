#ifndef USAC_HH
#define USAC_HH
#define NOMINMAX

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

#include "ConfigParams.h" 

template <class ProblemType>
class USAC
{
	public:
		// final results
		unsigned int m_num_inliers;
		unsigned int* m_inliers;
		unsigned int* m_best_sample;
		
	protected:
		//stats
		unsigned int hyp_count;				// counter for number of hypotheses generated so far
		unsigned int sample_rejected_count;	// counter for number of samples rejected by pre-validation
		unsigned int lo_count;				// counter for number of local optimizations
	public:
		unsigned int getHyp_count(){return hyp_count;}
		unsigned int getSample_rejected_count(){return sample_rejected_count;}
		unsigned int getLo_count(){return lo_count;}
		
		// for degenerate solution
		unsigned int m_num_degen_inliers;
		unsigned int* m_degen_inliers;
		unsigned int* m_degen_outliers;
		unsigned int* m_degen_sample;

	public:
		USAC() 
		{
			m_inliers					= NULL;
			m_best_sample				= NULL;
			m_degen_inliers				= NULL;
			m_degen_outliers			= NULL;
			m_degen_sample				= NULL;
			m_sortedPointIndicesPROSAC	= NULL;
			m_growthFunctionPROSAC		= NULL;
			m_nonRandomSamplesPROSAC	= NULL;
			m_maxInliersPROSAC			= NULL;
			m_errs						= NULL;
			hyp_count				= 0;
			sample_rejected_count	= 0;
			lo_count				= 0;
		};
		virtual ~USAC() {};
		bool init(const ConfigParams& cfg);
		bool solveMaster();
		void cleanup();

	protected:
		// common RANSAC input parameters
		double m_confThreshold;
		USACConfig::RandomSamplingMethod m_randomSamplingMethod;
		USACConfig::VerifMethod m_verifMethod;
		USACConfig::LocalOptimizationMethod m_localOptMethod;
		bool m_prevalidateSample;
		bool m_prevalidateModel;
		bool m_testDegeneracy;

		// PROSAC parameters
		unsigned int m_maxSamplesPROSAC;
		unsigned int* m_sortedPointIndicesPROSAC;
		unsigned int* m_growthFunctionPROSAC;
		unsigned int* m_nonRandomSamplesPROSAC;
		unsigned int* m_maxInliersPROSAC;
		unsigned int m_subsetSizePROSAC;
		unsigned int m_largestSetPROSAC;
		unsigned int m_stopLenPROSAC;
		unsigned int m_minStopLenPROSAC;

		// SPRT parameters
		double m_SPRT_tM;
		double m_SPRT_mS;
		double m_SPRT_delta;
		double m_SPRT_epsilon;
		double m_SPRTDecisionThreshold;

		// LOSAC parameters
		unsigned int m_LoInnerSampleSize;
		unsigned int m_LoInnerRansacRepetitions;
		double m_LoThresholdMultiplier;
		unsigned int m_LoNumStepsIterative;

		// problem specific input parameters
		unsigned int m_minSampleSize;  
		double m_inlierThreshold;   
		unsigned int m_maxHypotheses;	
		unsigned int m_maxSolutionsPerSample; 
		unsigned int m_numDataPoints;  

		// scratch space
		double* m_errs;
		double* m_errPtr[2];	// m_errPtr[0] points to the current scratch space
								// m_errPtr[1] points to the error values for the best solution

		// for randomized evaluation
		std::vector<unsigned int> m_evaluationPool;	 // holds ordering of points for evaluation
		unsigned int m_evalPoolIndex;				 // index to the first point to be verified

		// sample indices
		std::vector<unsigned int> m_sample;		

		// random sampling functions
		inline void generateUniformRandomSample(std::vector<unsigned int>& sample, unsigned int dataSize, 
												unsigned int sampleSize);
		inline void generatePROSACSample(std::vector<unsigned int>& sample, unsigned int hypCount);

	public:
		struct testHistorySPRT
		{
			double epsilon, delta, A;
			unsigned int k;
			struct testHistorySPRT *prev;
		};

	protected:
		inline void initPROSAC();
		inline void designSPRTTest();
		testHistorySPRT* addTestHistorySPRT(double epsilon, double delta, unsigned int numHyp, 
											testHistorySPRT* testHistory, unsigned int& lastUpdate);
		double computeExpSPRT(double new_epsilon, double epsilon, double delta);
		unsigned int locallyOptimizeSolution(const unsigned int bestInliers);
		unsigned int findInliers(const double* const errs, std::vector<unsigned int>& inliers, 
								 const double threshold);
		unsigned int updateStandardStopping(unsigned int numInliers, unsigned int totPoints);
		unsigned int updatePROSACStopping(unsigned int hypCount);
		unsigned int updateSPRTStopping(unsigned int numInliers, unsigned int totPoints, testHistorySPRT* testHistory);
};

// initializes the estimation problem by reading values from the config file
// also calls the initialization function of the specific problem 
template <class ProblemType>
bool USAC<ProblemType>::init(const ConfigParams& cfg)
{
	// store common parameters
	m_confThreshold			= cfg.common.confThreshold;
	m_randomSamplingMethod  = cfg.common.randomSamplingMethod;
	m_verifMethod			= cfg.common.verifMethod;
	m_localOptMethod        = cfg.common.localOptMethod;
	m_prevalidateSample		= cfg.common.prevalidateSample;
	m_prevalidateModel		= cfg.common.prevalidateSample;
	m_testDegeneracy		= cfg.common.testDegeneracy;

	// read in PROSAC parameters if required
	if (m_randomSamplingMethod == USACConfig::SAMP_PROSAC)
	{
		m_maxSamplesPROSAC   = cfg.prosac.maxSamplesPROSAC;
		m_sortedPointIndicesPROSAC = cfg.prosac.sortedPointIndices;
	}

	// read in SPRT parameters if required
	if (m_verifMethod == USACConfig::VERIF_SPRT)
	{
		m_SPRT_tM	   = cfg.sprt.tM;
		m_SPRT_mS      = cfg.sprt.mS;
		m_SPRT_delta   = cfg.sprt.delta;
		m_SPRT_epsilon = cfg.sprt.epsilon;
	}

	// read in LO parameters if required
	if (m_localOptMethod == USACConfig::LO_LOSAC)
	{
		m_LoInnerSampleSize		   = cfg.losac.innerSampleSize;
		m_LoInnerRansacRepetitions = cfg.losac.innerRansacRepetitions;
		m_LoThresholdMultiplier    = cfg.losac.thresholdMultiplier;
		m_LoNumStepsIterative	   = cfg.losac.numStepsIterative;
	}

	// initialize the problem specific stuff
	if (!static_cast<ProblemType *>(this)->initProblem(cfg))
	{
		return false;
	}

	// init the PROSAC data structures
	if (m_randomSamplingMethod == USACConfig::SAMP_PROSAC)
	{
		initPROSAC();
	}

	m_sample.resize(m_minSampleSize);

	m_errs = new double[2*m_numDataPoints];			
	for (unsigned int i = 0; i < 2; ++i)
	{
		m_errPtr[i] = m_errs + i*m_numDataPoints;
	}

	// inititalize evaluation ordering to a random permutation of 0...m_numDataPoints-1
	m_evalPoolIndex = 0;
	m_evaluationPool.resize(m_numDataPoints);
	for (unsigned int i = 0; i < m_numDataPoints; ++i)
	{
		m_evaluationPool[i] = i;
	}
	std::random_shuffle(m_evaluationPool.begin(), m_evaluationPool.end());

	// storage for results
	m_inliers = new unsigned int[m_numDataPoints];
	for (unsigned int i = 0; i < m_numDataPoints; ++i)
	{
		m_inliers[i] = 0;
	}

	m_best_sample = new unsigned int[m_minSampleSize];

	// if degeneracy testing option is selected
	if (m_testDegeneracy)
	{
		m_num_degen_inliers = 0;
		m_degen_inliers = new unsigned int[m_numDataPoints];
		for (unsigned int i = 0; i < m_numDataPoints; ++i)
		{
			m_degen_inliers[i] = 0;
		}
		m_degen_outliers = new unsigned int[m_numDataPoints];
		for (unsigned int i = 0; i < m_numDataPoints; ++i)
		{
			m_degen_outliers[i] = 0;
		}
		m_degen_sample = new unsigned int[m_minSampleSize];
	}

	return true;
}

template <class ProblemType>
void USAC<ProblemType>::cleanup()
{
	// TODO: clean this up - these must all be initialized to null!

	if (m_errs) { delete[] m_errs; m_errs = NULL; }

	if (m_randomSamplingMethod == USACConfig::SAMP_PROSAC) 
	{
		{ delete[] m_nonRandomSamplesPROSAC; m_nonRandomSamplesPROSAC = NULL; }
		{ delete[] m_maxInliersPROSAC; m_maxInliersPROSAC = NULL; }
		{ delete[] m_growthFunctionPROSAC; m_growthFunctionPROSAC = NULL; }
	}

	if (m_inliers) { delete[] m_inliers; m_inliers = NULL; }
	if (m_best_sample) { delete[] m_best_sample; m_best_sample = NULL; }

	if (m_testDegeneracy)
	{
		{ delete[] m_degen_inliers; m_degen_inliers = NULL; }
		{ delete[] m_degen_outliers; m_degen_outliers = NULL; }
		{ delete[] m_degen_sample; m_degen_sample = NULL; }
	}

	// clean up the problem specific stuff
	static_cast<ProblemType *>(this)->cleanupProblem();
}

template <class ProblemType>
bool USAC<ProblemType>::solveMaster()
{
	// common parameters and counts
	// moved up to protected fields unsigned int hyp_count = 0;		// counter for number of hypotheses generated so far
	unsigned int model_count = 0;									// counter for number of models generated so far 
	// moved up to protected fields unsigned int sample_rejected_count = 0;// counter for number of samples rejected by pre-validation
	unsigned int model_rejected_count = 0;							// counter for number of models rejected by pre-validation
	unsigned int best_inlier_count = 0;								// inliers for best sample so far
	unsigned int tot_points_verified = 0;							// total number of data point verifications
	unsigned int adaptive_stopping_count = m_maxHypotheses;			// initialize with worst case	
	
	// sprt parameters
	unsigned int last_wald_history_update;							
	testHistorySPRT *wald_test_history;
	bool update_sprt_stopping;
	if (m_verifMethod == USACConfig::VERIF_SPRT)
	{
		last_wald_history_update = 0;
		wald_test_history = NULL;
		update_sprt_stopping = true;
		designSPRTTest();
	}

	// main USAC loop
	while (hyp_count < adaptive_stopping_count && hyp_count < m_maxHypotheses)
	{
		++hyp_count;

		// generate sample
		switch (m_randomSamplingMethod)
		{
			case USACConfig::SAMP_UNIFORM:
			{
				generateUniformRandomSample(m_sample, m_numDataPoints, m_minSampleSize);
				break;
			}

			case USACConfig::SAMP_PROSAC:
			{
				generatePROSACSample(m_sample, hyp_count);
				break;
			} 
		}
		
		// validate sample
		if (m_prevalidateSample)
		{
			// pre-validate sample before testing generating model
			bool valid_sample = static_cast<ProblemType *>(this)->validateSample();
			if (!valid_sample)
			{
				++sample_rejected_count;
				continue;
			}
		}

		// generate model(s)
		unsigned int num_solns = static_cast<ProblemType *>
								 (this)->generateMinimalSampleModels();
		model_count += num_solns;

		// evaluate model(s)
		bool update_best = false;
		for (unsigned int i = 0; i < num_solns; ++i)
		{
			if (m_prevalidateModel)
			{
				// pre-validate model before testing against data points
				bool valid_model = static_cast<ProblemType *>(this)->validateModel(i);
				if (!valid_model)
				{
					++model_rejected_count;
					continue;
				}
			}

			// evaluate model
			// note: the type of evaluation (all points/sprt) is handled inside this function
			unsigned int inlier_count, num_points_tested;
			bool good = static_cast<ProblemType *>
				        (this)->evaluateModel(i, inlier_count, num_points_tested);

			// update based on verification results
			switch (m_verifMethod)
			{
				case USACConfig::VERIF_STANDARD:
				{
					tot_points_verified += m_numDataPoints;
					// check if best so far
					if (inlier_count > best_inlier_count)
					{
						update_best = true;
						best_inlier_count = inlier_count;
						// store model
						static_cast<ProblemType *>(this)->storeSolution(i, best_inlier_count);
					}
					break;
				} // end case standard verification
				
				case USACConfig::VERIF_SPRT:
				{
					if (!good)
					{
						tot_points_verified += num_points_tested;
						double delta_new = (double)inlier_count/num_points_tested;
						if (delta_new > 0 && fabs(m_SPRT_delta - delta_new)/m_SPRT_delta > 0.1)
						{
							// update parameters
							// TODO: only update the history once in a while for changes in delta?
							wald_test_history = addTestHistorySPRT(m_SPRT_epsilon, m_SPRT_delta, hyp_count, wald_test_history, last_wald_history_update);
							m_SPRT_delta = delta_new;
							designSPRTTest();
						}
					}
					else
					{
						tot_points_verified += m_numDataPoints;
						if (inlier_count > best_inlier_count)
						{
							update_best = true;
							best_inlier_count = inlier_count;
							wald_test_history = addTestHistorySPRT(m_SPRT_epsilon, m_SPRT_delta, hyp_count, wald_test_history, last_wald_history_update);
							m_SPRT_epsilon = (double)best_inlier_count/m_numDataPoints;
							designSPRTTest();
							update_sprt_stopping = true;
							// store model
							static_cast<ProblemType *>(this)->storeSolution(i, best_inlier_count);
						}
					}
					break;
				} // end case sprt

			} // end switch verification method
		} // end evaluating all models for one minimal sample

		// check for degeneracy in the model
		bool degenerate_model = false, upgrade_model = false, upgrade_successful = false;
		if (update_best && m_testDegeneracy)
		{
			std::cout << "Testing for degeneracy (" << best_inlier_count << ")" << std::endl;
			static_cast<ProblemType *>(this)->testSolutionDegeneracy(degenerate_model, upgrade_model);
			if (degenerate_model && upgrade_model)
			{
				// complete model
				unsigned int upgrade_inliers = static_cast<ProblemType *>(this)->upgradeDegenerateModel();
				if (upgrade_inliers >= best_inlier_count)
				{
					best_inlier_count = upgrade_inliers;
					upgrade_successful = true;
				}
			}
		}

		// perform local optimization if specified
		if (m_localOptMethod == USACConfig::LO_LOSAC && update_best == true)
		{
			++lo_count;
			std::cout << "(" << hyp_count << ") Performing LO. Inlier count before: " << best_inlier_count;
			unsigned int lo_inlier_count = locallyOptimizeSolution(best_inlier_count);
			if (lo_inlier_count > best_inlier_count)
			{
				best_inlier_count = lo_inlier_count;
			}
			std::cout << ", inlier count after: " << best_inlier_count << '\n';
			if (!lo_inlier_count) {
				std::cout << "Prematurely escaped LO, not enough inliers. Maybe LO.innerSampleSize was set too high?\n";
			}
			//std::cout << ", inlier count after: " << lo_inlier_count << std::endl; //returned 0 often
		}

		if (update_best)
		{
			// update the number of samples required
			if ( m_randomSamplingMethod == USACConfig::SAMP_PROSAC && hyp_count <= m_maxSamplesPROSAC && 
			     (!degenerate_model || (degenerate_model && upgrade_successful)) )
			{
				adaptive_stopping_count = updatePROSACStopping(hyp_count); 
			}
			else
			{
				adaptive_stopping_count = updateStandardStopping(best_inlier_count, m_numDataPoints); 
			}
		}

		// update adaptive stopping count to take SPRT test into account
		if (m_verifMethod == USACConfig::VERIF_SPRT && m_randomSamplingMethod != USACConfig::SAMP_PROSAC)
		{
			if (hyp_count >= adaptive_stopping_count && update_sprt_stopping)
			{
				adaptive_stopping_count = updateSPRTStopping(best_inlier_count, m_numDataPoints, wald_test_history);
				update_sprt_stopping = false;
			}
		}

	} // end the main USAC loop

	// output statistics

	//std::cout << "Number of hypotheses/models: " << hyp_count << "/" << model_count << std::endl;
	if (m_prevalidateSample) {
		std::cout << "Number of samples rejected by pre-validation: " << sample_rejected_count << std::endl;
	}
	if (m_prevalidateModel) {
		std::cout << "Number of models rejected by pre-validation: " << model_rejected_count << std::endl;
	}
	//std::cout << "Number of verifications per model: " << (double)tot_points_verified/(model_count-model_rejected_count) << std::endl;
	//std::cout << "Max inliers/total points: " << best_inlier_count << "/" << m_numDataPoints << std::endl;

	// clean up
	if (m_verifMethod == USACConfig::VERIF_SPRT)
	{
		while (wald_test_history)
		{
			testHistorySPRT *temp = wald_test_history->prev;
			delete wald_test_history;
			wald_test_history = temp;
		}
	}

	return true;
}


template <class ProblemType>
void USAC<ProblemType>::generateUniformRandomSample(std::vector<unsigned int>& sample, 
													unsigned int dataSize, unsigned int sampleSize)
{
	unsigned int count=0;
	unsigned int index;
	std::vector<unsigned int>::iterator pos;
	pos = sample.begin();
	do {
		index = rand() % dataSize;
		if (find(sample.begin(), pos, index) == pos)
		{
				sample[count] = index;
				++count;
				++pos;
		}
	} while (count < sampleSize);
}


template <class ProblemType>
void USAC<ProblemType>::generatePROSACSample(std::vector<unsigned int>& sample, unsigned int hypCount)
{
	// revert to RANSAC-style sampling if maximum number of PROSAC samples have been tested
	if (hypCount > m_maxSamplesPROSAC)
	{
		generateUniformRandomSample(sample, m_numDataPoints, m_minSampleSize);	
		return;
	}

	// if current stopping length is less than size of current pool, use only points up to the stopping length
	if (m_subsetSizePROSAC > m_stopLenPROSAC)
	{
		generateUniformRandomSample(sample, m_stopLenPROSAC, m_minSampleSize);	
	}

	// increment the size of the sampling pool if required
	if (hypCount > m_growthFunctionPROSAC[m_subsetSizePROSAC-1])
	{
		++m_subsetSizePROSAC;
		if (m_subsetSizePROSAC > m_numDataPoints)
		{
			m_subsetSizePROSAC = m_numDataPoints;
		}
		if (m_largestSetPROSAC < m_subsetSizePROSAC)
		{
			m_largestSetPROSAC = m_subsetSizePROSAC;
		}
	}

	// generate PROSAC sample
	generateUniformRandomSample(sample, m_subsetSizePROSAC-1, m_minSampleSize-1);
	sample[m_minSampleSize-1] = m_subsetSizePROSAC-1;
	for (unsigned int i = 0; i < sample.size(); ++i)
	{
		sample[i] = m_sortedPointIndicesPROSAC[sample[i]];
	}
	return;

}

template <class ProblemType> inline
void USAC<ProblemType>::initPROSAC()
{
	// this array allows us to set up the non-randomness part of the stopping criterion
	// each value represents the point at which the minimum number of non-random inliers is incremented
	unsigned int min_samples[] = {  8,  10,  13,  17,  22,  28,  34,  42,  51,  61,
								   72,  84,  97, 110, 125, 141, 157, 175, 193, 213,
								  233, 255, 277, 300, 324, 349, 375, 402, 430, 459,
								  489, 519, 551, 583, 616, 651, 686, 722, 759, 797,
								  835, 875, 915, 957, 999, 1000000};

	// initialize the arrays that determine stopping
	m_nonRandomSamplesPROSAC = new unsigned int[m_numDataPoints];	// i-th entry - number of samples for pool [0...i] (pool length = i+1)
	m_maxInliersPROSAC = new unsigned int[m_numDataPoints];			// i-th entry - inlier counts for termination up to i-th point (term length = i+1)
	unsigned int j = 0;
	for (unsigned int i = 0; i < m_numDataPoints; ++i)
	{
		if (min_samples[j] <= i)
		{
			++j;
		}
		m_nonRandomSamplesPROSAC[i] = m_maxHypotheses;
		m_maxInliersPROSAC[i] = m_minSampleSize + j;
	}

	// growth function
	m_growthFunctionPROSAC = new unsigned int[m_numDataPoints];
	double T_n;
	unsigned int T_n_p = 1; 

	// compute initial value for T_n
	T_n = m_maxSamplesPROSAC;
	for (unsigned int i = 0; i < m_minSampleSize; ++i)
	{
		T_n *= (double)(m_minSampleSize-i)/(m_numDataPoints-i);
	}

	for (unsigned int i = 0; i < m_numDataPoints; ++i)
	{
		if (i+1 <= m_minSampleSize)
		{
			m_growthFunctionPROSAC[i] = T_n_p;
			continue;
		}

		double temp = (double)(i+1)*T_n/(i+1-m_minSampleSize);
		m_growthFunctionPROSAC[i] = T_n_p + (unsigned int)ceil(temp - T_n);
		T_n = temp;
		T_n_p = m_growthFunctionPROSAC[i];
	}

	// other initializations
	m_minStopLenPROSAC = 20;					// check at least this many points
	m_largestSetPROSAC = m_minStopLenPROSAC;	// holds the largest size of the sampling pool
	m_subsetSizePROSAC = m_minSampleSize;		// size of the current sampling pool
	m_stopLenPROSAC = m_numDataPoints;			// current stopping length
}

template <class ProblemType> inline
void USAC<ProblemType>::designSPRTTest()
{
	double An_1, An, C, K;

	C = (1 - m_SPRT_delta)*log( (1 - m_SPRT_delta)/(1-m_SPRT_epsilon) ) 
		+ m_SPRT_delta*(log( m_SPRT_delta/m_SPRT_epsilon ));
	K = (m_SPRT_tM*C)/m_SPRT_mS + 1;
	An_1 = K;

	// compute A using a recursive relation
	// A* = lim(n->inf)(An), the series typically converges within 4 iterations
	for (unsigned int i = 0; i < 10; ++i)
    {
		An = K + log(An_1);
		if (An - An_1 < 1.5e-8) 
		{
			break;
		}
	    An_1 = An;
    }

	m_SPRTDecisionThreshold = An;
}

template <class ProblemType> inline
unsigned int USAC<ProblemType>::locallyOptimizeSolution(const unsigned int bestInliers)
{
	// return if insufficient number of points
	// TODO: check to see if this value is always correct
	if (bestInliers < 2*m_LoInnerSampleSize) 
	{
		return 0;
	}

	unsigned int lo_sample_size = std::min(m_LoInnerSampleSize, bestInliers/2);
	std::vector<unsigned int> sample(lo_sample_size);
	std::vector<unsigned int> orig_inliers(m_numDataPoints);
	std::vector<unsigned int> iter_inliers(m_numDataPoints);
	unsigned int num_points_tested;
	double *weights = new double[m_numDataPoints];	
	double threshold_step_size = (m_LoThresholdMultiplier*m_inlierThreshold - m_inlierThreshold)
								  /m_LoNumStepsIterative;

	// find all inliers less than threshold 
	unsigned int lo_inliers = bestInliers;
	unsigned int temp_inliers = 0;
	findInliers(m_errPtr[1], orig_inliers, m_inlierThreshold);	

	// perform number of inner RANSAC repetitions
	for (unsigned int i = 0; i < m_LoInnerRansacRepetitions; ++i)
	{
		// generate non-minimal sample model and find inliers 
		generateUniformRandomSample(sample, bestInliers, lo_sample_size);
		for (unsigned int j = 0; j < lo_sample_size; ++j)
		{
			sample[j] = orig_inliers[sample[j]];    // we want points only from the current inlier set
		}
		if ( !static_cast<ProblemType *>(this)->generateRefinedModel(sample, lo_sample_size) )
		{
			continue;
		}
		if (! static_cast<ProblemType *>(this)->evaluateModel(0, temp_inliers, num_points_tested) )
		{
			continue;
		}
		temp_inliers = findInliers(m_errPtr[0], iter_inliers, m_LoThresholdMultiplier*m_inlierThreshold);
		// TODO: if this set of inliers is almost the same as the best one, not much chance of improving

		// generate least squares model from all inliers
		if (! static_cast<ProblemType *>(this)->generateRefinedModel(iter_inliers, temp_inliers) )
		{
			continue;
		}

		// iterative (reweighted) refinement - reduce threshold in steps, find new inliers and refit fundamental matrix
		// using weighted least-squares
		for (unsigned int j = 0; j < m_LoNumStepsIterative; ++j)
		{
			if (! static_cast<ProblemType *>(this)->evaluateModel(0, temp_inliers, num_points_tested) )
			{
				continue;
			}
			findInliers(m_errPtr[0], iter_inliers, (m_LoThresholdMultiplier*m_inlierThreshold) - (j+1)*threshold_step_size);		
			static_cast<ProblemType *>(this)->findWeights(0, iter_inliers, temp_inliers, weights);
			if (! static_cast<ProblemType *>(this)->generateRefinedModel(iter_inliers, temp_inliers, true, weights) )
			{
				continue;
			}
		}

		// find final set of inliers for this round
		if (! static_cast<ProblemType *>(this)->evaluateModel(0, temp_inliers, num_points_tested) )
		{
			continue;
		}
		findInliers(m_errPtr[0], iter_inliers, m_inlierThreshold);	

		if (temp_inliers > lo_inliers)
		{
			// store model
			lo_inliers = temp_inliers;
			static_cast<ProblemType *>(this)->storeSolution(0, lo_inliers);
		}
	}

	delete[] weights;
	return lo_inliers;
}	

template <class ProblemType> inline
unsigned int USAC<ProblemType>::findInliers(const double* const errs, std::vector<unsigned int> &inliers, 
											const double threshold)
{
	unsigned int inlier_count = 0;
	for (unsigned int i = 0; i < m_numDataPoints; ++i)
	{
		if (*(errs+i) < threshold)
		{
			inliers[inlier_count] = i;
			++inlier_count;
		}
	}
	return inlier_count;
}

template <class ProblemType> inline
unsigned int USAC<ProblemType>::updateStandardStopping(unsigned int numInliers, unsigned int totPoints)
{
	double n_inliers = 1.0;
	double n_pts = 1.0;

	for (unsigned int i = 0; i < m_minSampleSize; ++i)
	{
		n_inliers *= numInliers - i;
		n_pts *= totPoints - i;
	}
	double prob_good_model = n_inliers/n_pts;

	if ( prob_good_model < std::numeric_limits<double>::epsilon() )
	{
		return m_maxHypotheses;
	}
	else if ( 1 - prob_good_model < std::numeric_limits<double>::epsilon() )
	{
		return 1;
	}
	else 
	{
		double num_samples = log(1-m_confThreshold)/log(1-prob_good_model);
		return (unsigned int) ceil(num_samples);
	}
}

template <class ProblemType> inline
unsigned int USAC<ProblemType>::updatePROSACStopping(unsigned int hypCount)
{
	unsigned int max_samples = m_nonRandomSamplesPROSAC[m_stopLenPROSAC-1];
	
	// go through sorted points and track inlier counts
	unsigned int inlier_count = 0;

	// just accumulate the count for the first m_minStopLenPROSAC points
	for (unsigned int i = 0; i < m_minStopLenPROSAC; ++i)
	{
		inlier_count += m_inliers[m_sortedPointIndicesPROSAC[i]];
	}	

	// after this initial subset, try to update the stopping length if possible
	for (unsigned int i = m_minStopLenPROSAC; i < m_numDataPoints; ++i)
	{
		inlier_count += m_inliers[m_sortedPointIndicesPROSAC[i]];

		if (m_maxInliersPROSAC[i] < inlier_count)
		{
			m_maxInliersPROSAC[i] = inlier_count;	// update the best inliers for the the subset [0...i]

			// update the number of samples based on this inlier count
			if ( (i == m_numDataPoints-1) || (m_inliers[m_sortedPointIndicesPROSAC[i]] && !m_inliers[m_sortedPointIndicesPROSAC[i+1]]) )
			{
				unsigned int new_samples = updateStandardStopping(inlier_count, i+1);
				if (i+1 < m_largestSetPROSAC)
				{
					// correct for number of samples that have points in [i+1, m_largestSetPROSAC-1]
					// TODO: check this
					new_samples += hypCount - m_growthFunctionPROSAC[i];
				}

				if (new_samples < m_nonRandomSamplesPROSAC[i])
				{
					m_nonRandomSamplesPROSAC[i] = new_samples;
					if ( (new_samples < max_samples) || ( (new_samples == max_samples) && (i+1 >= m_stopLenPROSAC) ) )
					{
						m_stopLenPROSAC = i+1;
						max_samples = new_samples;
					}
				}
			}
		}
	}
	return max_samples;
}

template <class ProblemType> inline
unsigned int USAC<ProblemType>::updateSPRTStopping(unsigned int numInliers, unsigned int totPoints, testHistorySPRT* testHistory)
{
	double n_inliers = 1.0;
	double n_pts = 1.0;
	double h = 0.0, k = 0.0, prob_reject_good_model = 0.0, log_eta = 0.0;
	double new_eps = (double)numInliers/totPoints;
	testHistorySPRT* current_test = testHistory;

	for (unsigned int i = 0; i < m_minSampleSize; ++i)
	{
		n_inliers *= numInliers - i;
		n_pts *= totPoints - i;
	}
	double prob_good_model = n_inliers/n_pts;

	if ( prob_good_model < std::numeric_limits<double>::epsilon() )
	{
		return m_maxHypotheses;
	}
	else if ( 1 - prob_good_model < std::numeric_limits<double>::epsilon() )
	{
		return 1;
	}

	while (current_test != NULL)
	{
		k += current_test->k;
		h = computeExpSPRT(new_eps, current_test->epsilon, current_test->delta);
		prob_reject_good_model = 1/(exp( h*log(current_test->A) ));
		log_eta += (double) current_test->k * log( 1 - prob_good_model*(1-prob_reject_good_model) );
		current_test = current_test->prev;
	}

	double num_samples = k + ( log(1-m_confThreshold) - log_eta ) / log( 1-prob_good_model * (1-(1/m_SPRTDecisionThreshold)) );
	return (unsigned int) ceil(num_samples);	
}

template <class ProblemType> inline
double USAC<ProblemType>::computeExpSPRT(double newEpsilon, double epsilon, double delta)
{
	double al, be, x0, x1, v0, v1, h;

	al = log(delta/epsilon);
	be = log( (1-delta)/(1-epsilon) );

	x0 = log( 1/(1-newEpsilon) )/be;
	v0 = newEpsilon * exp(x0 *al);
	x1 = log( (1-2*v0) / (1-newEpsilon) )/be;
	v1 = newEpsilon * exp(x1 * al) + (1-newEpsilon) * exp(x1 * be);
	h = x0 - (x0 - x1)/(1+v0 - v1)*v0;
	return h;
}

template <class ProblemType> inline
typename USAC<ProblemType>::testHistorySPRT* USAC<ProblemType>::addTestHistorySPRT(double epsilon, double delta, unsigned int numHyp, 
																				   testHistorySPRT *testHistory, unsigned int &lastUpdate)
{
	testHistorySPRT *new_test_history = new testHistorySPRT;
	new_test_history->epsilon = epsilon;
	new_test_history->delta = delta;
	new_test_history->A = m_SPRTDecisionThreshold;
	new_test_history->k = numHyp - lastUpdate;
	new_test_history->prev = testHistory;
	lastUpdate = numHyp;

	return new_test_history;
}

#endif   

