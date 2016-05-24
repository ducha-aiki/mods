// libTILDE.hpp --- 
// 
// Filename: libTILDE.hpp
// Description: 
// Author: Yannick Verdie, Kwang Moo Yi, Alberto Crivella
// Maintainer: Yannick Verdie, Kwang Moo Yi
// Created: Tue Mar  3 17:54:26 2015 (+0100)
// Version: 0.5a
// Package-Requires: ()
// Last-Updated: Thu May 28 13:17:42 2015 (+0200)
//           By: Kwang
//     Update #: 36
// URL: 
// Doc URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Change Log:
// 
// 
// 
// 
// Copyright (C), EPFL Computer Vision Lab.
// 
// 

// Code:

#pragma once
#ifndef _LIBTILDE_HPP_
#define _LIBTILDE_HPP_

#include <algorithm>
#include <functional>
#include <array>
#include <fstream>		// std::ifstream
#include <stdio.h>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>  	//for parallel_opencv

#include "NonMaxSup.hpp"
#include "../3rdParties/rgbConvertMex.hpp"

using namespace std;
using namespace cv;

// Class structure for individual filters (used for non-approx filters only)
// NOTE: This class was used in early development. Will be removed in the future.
class lfilter {
public:
	lfilter() {
	};
	lfilter(Mat _m, float _f) {
		w.clear();
		Mat r;
		_m.copyTo(r);	//do we need that ?
		w.push_back(r);
		b = _f;
	};

	void push_back(const Mat & _m) {
		w.push_back(_m);
	}

	int size() const {
		return w.size();
	}
	lfilter & operator=(const lfilter & anotherStruct) {
		this->w = anotherStruct.w;
		this->b = anotherStruct.b;
		return *this;
	};

	vector < Mat > w;	//for each channel
	float b;
};

// Class structure for TILDE objects
// NOTE: this class also contains lfilter class for backward compatibility
class TILDEobjects {
public:
	vector < float >parameters;	//for both

	//for approx
	vector < vector < float >>coeffs;
	vector < Mat > filters;
	vector < float >bias;
	//---------
	//for non approx
	vector < vector < lfilter > >nonApprox_filters;
	bool isApprox;
	bool useDescriptorField;
	string name;
	
};



//--------------------------------------------------------------------------------------
// TILDE Keypoint extraction function
vector < KeyPoint > getTILDEKeyPoints(
	const Mat & indatav,
	const string & nameFilter,
	const bool useApprox,
	const bool sortMe = false,
	const bool keepPositiveScoreOnly = false,
	Mat * score = NULL);

vector < KeyPoint > getTILDEKeyPoints_fast(
	const Mat & indatav,
	const string & nameFilter,
	const bool sortMe = false,
	const bool keepPositiveScoreOnly = false,
	Mat * score = NULL);

Mat normalizeScore(const Mat& score);
//--------------------------------------------------------------------------------------
// For approximated TILDE

// Read TILDE filters from the txt file
TILDEobjects getTILDEApproxObjects(
	const string & name,
	void *_p);

// Apply and get TILDE keypoints in < x, y, score > format
vector < Point3f > applyApproxFilters(
	const Mat & p,
	const TILDEobjects & why,
	const vector < float >&param,
	const bool useDescriptorField,
	const bool sortMe,
	const bool keep_only_positive,
	Mat * score);

// Apply and get TILDE keypoints in < x, y, score > format
vector < KeyPoint > applyApproxFilters_fast(
	const Mat & p,
	const TILDEobjects & why,
	const vector < float >&param,
	const bool sortMe,
	const bool keep_only_positive,
	Mat * score);

// Apply and get TILDE score map
vector < vector < Mat > >getScoresForApprox(
	const TILDEobjects & cas,
	const vector < Mat > &convt_image);

void getScoresandCombine_Approx(const TILDEobjects & cas,
						       const vector < Mat > &convt_image,
						       const bool keep_only_positive,
						       	Mat *output);

//--------------------------------------------------------------------------------------
// For Non approximated TILDE

// Read TILDE filters from txt file
vector < vector < lfilter > >getTILDENonApproxFilters(
	const string & name,
	void *param = NULL);

// Apply and get TILDE keypoints in < x, y, score > format
vector < Point3f > applyNonApproxFilters(
	const Mat & p,
	const vector < vector < lfilter > >&dual_cascade_filters,
	const vector < float >&param,
	const bool useDescriptorField,
	const bool sortMe,
	const bool keep_only_positive,
	Mat * score);



//--------------------------------------------------------------------------------------
// Filter Reading

// Function for reading the txt file. Calls appropriate function for approx and non approx
TILDEobjects getTILDEObject(
	const string & name,
	void *_p,
	bool useApprox,
	bool useDescriptorField);

// Additional functions for parsing
template < class T > T sToT(std::string text)
{
	std::stringstream temp_ss(std::stringstream::in | std::stringstream::out);
	temp_ss.setf(std::ios::fixed, std::ios::floatfield);
	T temp;
	temp_ss << text;
	temp_ss >> temp;

	return temp;
}
void Tokenize(const std::string & mystring, std::vector < std::string > &tok, const std::string & sep = " ", int lp = 0, int p = 0);
std::string delSpaces(std::string & str);

//--------------------------------------------------------------------------------------
// Image Processing and Misc
vector < Mat > getGradImage(const Mat & p);
vector < Mat > getLuvImage(const Mat & p);
Mat convBGR2PlaneWiseRGB(const Mat &);
Mat convPlaneWiseRGB2RGB(const Mat &);
Mat sumMatArray(const vector < Mat > &MatArray);

// DescriptorField extraction. This function is for future use.
vector < Mat > getNormalizedDescriptorField(const Mat & im);

// TILDE Keypoint extraction function
Mat getTILDEResponce(
	const Mat & indatav,
	const string & nameFilter,
	const bool useApprox,
	const bool keepPositiveScoreOnly);

#endif

// 
// libParts.hpp ends here
