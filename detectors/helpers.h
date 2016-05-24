/*
 * Copyright (C) 2008-12 Michal Perdoch
 * All rights reserved.
 *
 * This file is part of the HessianAffine detector and is made available under
 * the terms of the BSD license (see the COPYING file).
 *
 */

#ifndef __HELPERS_H__
#define __HELPERS_H__

#include <opencv2/core/core.hpp>
#include "structures.hpp"

void solveLinear3x3(float *A, float *b);
bool getEigenvalues(float a, float b, float c, float d, float &l1, float &l2);
void invSqrt(float &a, float &b, float &c, float &l1, float &l2);
void computeGaussMask(cv::Mat &mask);
void computeCircularGaussMask(cv::Mat &mask, float sigma = 0);
//void rectifyAffineTransformationUpIsUp(float *U);
//void rectifyAffineTransformationUpIsUpF(float &a11, float &a12, float &a21, float &a22);
void rectifyAffineTransformationUpIsUp(float &a11, float &a12, float &a21, float &a22);
void rectifyAffineTransformationUpIsUp(double *U);
void rectifyAffineTransformationUpIsUp(double &a11, double &a12, double &a21, double &a22);

bool interpolate(const cv::Mat &im,const float ofsx,const float ofsy,
                 const float a11,const float a12,const float a21,const float a22, cv::Mat &res);

bool interpolateCheckBorders(const cv::Mat &im, const float ofsx, const float ofsy,
                             const float a11,const float a12,const float a21,const float a22, const cv::Mat &res);

bool interpolateCheckBorders(const int orig_img_w, const int orig_img_h, const float ofsx, const float ofsy,
                             const float a11, const float a12,const float a21,const float a22, const int res_w, const int res_h);

void photometricallyNormalize(cv::Mat &image, const cv::Mat &weight_mask, float &sum, float &var);

cv::Mat gaussianBlur(const cv::Mat input, float sigma);
void gaussianBlurInplace(cv::Mat &inplace, float sigma);
cv::Mat doubleImage(const cv::Mat &input);
cv::Mat halfImage(const cv::Mat &input);
//double atan2approx(double y,double x);
//double atan2approx2(double y,double x);
//double atan2LUT(double y,double x);
float atan2LUTff(float y,float x);

void computeGradient(const cv::Mat &img, cv::Mat &gradx, cv::Mat &grady);
void computeGradientMagnitudeAndOrientation(const cv::Mat &img, cv::Mat &mag, cv::Mat &ori);
//void computeGradientMagnitudeAndOrientationWLD(const cv::Mat &img, cv::Mat &mag, cv::Mat &ori);

/*
void calculateWLDfast(const cv::Mat &inImg, cv::Mat &outImg, const WLDParams pars = WLDParams(),
                      const double sigmaNext = 1.5,
                      const double sigmaStart = 0.5,
                      const double sigmaEnd = -1);
*/
double getTime();

#endif // __HELPERS_H__
