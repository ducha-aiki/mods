/*
 *	SELF-SIMILARITY DESCRIPTOR
 *	C++ IMPLEMENTATION
 *	v 1.1
 *
 *	Ken Chatfield
 *	Engineering Department
 *	University of Oxford
 *
 *	Date: March 2009
 *	Updated March 2010
 *
 *	C++ Routine for calculating self-similarity descriptors densely across an image
 *
 *	The self-similarity descriptor was first described in:
 *	Shectman E., Irani M., "Matching Local Self-Similarities across Images and Videos" (CVPR '07)
 *
 *	In addition, the modifications described in the following paper are applied:
 *	Chatfield K., Philbin J., Zisserman A., "Efficient Retrieval of Deformable Shape Classes
 *		using Local Self-Similarities" ICCV Workshop on Non-rigid Shape Analysis and Deformable
 *		Image Alignment (NORDIA '09)
 *	In particular the second nearest neighbor ratio used to sparsify descriptors
 *
 *	In the following code, ssd is sum-of-square diffs and ssdesc is self-similarity
 *	descriptor
 *
 *	NOTE: row-major arrays are used for all internal calculations, with
 *	input/output arrays being in column-major style to ease interfacing with MATLAB
 *	using the mex function defined in mexCalcSsdescs.cc
 *
 *	For usage instructions see readme.txt
 *
 */

#ifndef SSDESC_H_
#define SSDESC_H_

/* toggles use of approximate/exact nearest neighbor
 * N.B. approximate NN requires use of external library 'jp_nn_kdtree.hpp'
 */
//#define USE_APPROXNN

#define _USE_MATH_DEFINES

#include <algorithm>
#include <limits>
#include <vector>
#include <list>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#ifdef USE_APPROXNN
#include "jp_nn_kdtree.hpp"	//approximate snn function
#endif

using namespace std;

namespace ssdesc
{
	//self-similarity descriptor parameters
	template<class fnumtype> struct ssdesc_parms
	{
		int window_size;
		int cor_size;
		int nrad;
		int nang;
		fnumtype var_noise;
		fnumtype saliency_thresh;
		fnumtype homogeneity_thresh;
		fnumtype snn_thresh;
	};

	//used as basic list element for returning points
	struct coordElem
	{
		int x;
		int y;
		coordElem(int xval, int yval): x(xval), y(yval) {}
	};

	//used to specify the region over the image in which to calculate descriptors
	struct rectRegion
	{
		int xfrom, xto, yfrom, yto;
		rectRegion(): xfrom(-1), xto(-1), yfrom(-1), yto(-1) {}
		rectRegion(int xfromval, int xtoval, int yfromval, int ytoval): xfrom(xfromval), xto(xtoval), yfrom(yfromval), yto(ytoval) {}
	};

	//class descSimElem
	//{
	//public:
	//	double ssd;
	//	int x;
	//	int y;
	//	bool operator<(const descSimElem& b) const { return ssd < b.ssd; }
	//	descSimElem(double ssdval, int xval, int yval)
	//	{
	//		ssd = ssdval;
	//		x = xval;
	//		y = yval;
	//	}
	//};

	/* -- Self-similarity descriptor Functions ------------------------------------------------------*/

	template<class fnumtype> void calc_ssdescs(const fnumtype* image, const int image_width,
		const int image_height, const int image_channels, const ssdesc_parms<fnumtype> &parms,
		vector<fnumtype>* ssdescs, const rectRegion calc_rect = rectRegion());
	template<class fnumtype> void calc_ssdescs_alt(const fnumtype* image, const int image_width,
		const int image_height, const int image_channels, const ssdesc_parms<fnumtype> &parms,
		vector<fnumtype>* ssdescs, const rectRegion calc_rect = rectRegion());

	template<class fnumtype> void prune_normalise_ssdescs(vector<fnumtype>& ssdescs,
		const int image_width, const int image_height, const ssdesc_parms<fnumtype> &parms,
		vector<fnumtype>* resp, vector<coordElem>* draw_coords, vector<coordElem>* salient_coords,
		vector<coordElem>* homogeneous_coords, vector<coordElem>* snn_coords, const rectRegion calc_rect = rectRegion());

	namespace details
	{
		void ssdesc_imask(const int ssd_sz, const int nrad, const int nang, vector<int>* imask);
		template<class fnumtype> inline fnumtype fast_exp(const fnumtype x);
		template<> inline float fast_exp(const float x);
		template<class fnumtype> void ssdesc_descriptor(const vector<fnumtype>& ssd, const vector<int>& imask,
			const int ssd_sz, const int nrad, const int nang,const fnumtype var_noise, vector<fnumtype>* ssdescs, const int offset);
		template<class fnumtype> void ssd_compute(const fnumtype* img, const int width, const int height,
			const int nchan, const int xl, const int xr, const int yl, const int yr, const int xp,
			const int yp, const int ssd_sz, const int sz, vector<fnumtype>* ssd_surf);
		template<class fnumtype> inline void ssd_compute_irow(const fnumtype* img, const int width,
			const int height, const int nchan, const int xl, const int xr, const int yl, const int yr,
			const int xp, const int yp, const int sz, const vector<fnumtype>& ssd_surf_in, vector<fnumtype>* ssd_surf_out);
		//functions for exact nearest neighbor - used if USE_APPROXNN is not defined
		template<class fnumtype> fnumtype calc_ssd_ssdesc(const fnumtype* ssdesc1,
			const fnumtype* ssdesc2, const int ssdesc_size);
		template<class fnumtype> double calc_ssd_ssdesc_min2(const fnumtype* ssdesc1,
			const fnumtype* ssdesc2, const int ssdesc_size, fnumtype &ssd_min_bound1,
			fnumtype &ssd_min_bound2, const fnumtype boundMultiplier = -1);
	}
}

#endif /* SSDESC_H_ */
