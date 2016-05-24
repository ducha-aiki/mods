/*
 * Copyright (C) 2008-12 Michal Perdoch
 * All rights reserved.
 *
 * This file is part of the HessianAffine detector and is made available under
 * the terms of the BSD license (see the COPYING file).
 *
 */

#ifndef __PYRAMID_H__
#define __PYRAMID_H__


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../helpers.h"
#include "../structures.hpp"
using cv::Mat;

bool responseCompare(AffineKeypoint k1,AffineKeypoint k2);
bool responseCompareInvOrder(AffineKeypoint k1,AffineKeypoint k2);

class KeypointCallback
{
public:
  virtual void onKeypointDetected(const Mat &blur, float x, float y, float s, float pixelDistance, int type, float response) = 0;
};

struct ScaleSpaceDetector
{
  enum
  {
    HESSIAN_DARK   = 0,
    HESSIAN_BRIGHT = 1,
    HESSIAN_SADDLE = 2,
    DOG_DARK   = 10,
    DOG_BRIGHT = 11,
    HARRIS_DARK   = 30,
    HARRIS_BRIGHT = 31,
    CAFFE_GRAD = 40,
    TILDE = 51
  };
public:
  KeypointCallback *keypointCallback;
  PyramidParams Pyrpar;
  ScalePyramid scale_pyramid;
  ScaleSpaceDetector(const PyramidParams &Pyrpar) :
    edgeScoreThreshold((Pyrpar.edgeEigenValueRatio + 1.0f)*(Pyrpar.edgeEigenValueRatio + 1.0f)/Pyrpar.edgeEigenValueRatio),
    finalThreshold(Pyrpar.threshold),
    positiveThreshold(0.8 * finalThreshold),
    negativeThreshold(-positiveThreshold)
  {
    extrema_points = 0;
    localized_points = 0;
    this->Pyrpar = Pyrpar;
    if (Pyrpar.DetectorType == DET_HESSIAN)
      finalThreshold = Pyrpar.threshold*Pyrpar.threshold;

    if (Pyrpar.DetectorMode !=FIXED_TH)
      finalThreshold = positiveThreshold = negativeThreshold = effectiveThreshold = 0.0;
    else effectiveThreshold = Pyrpar.threshold;

    if (Pyrpar.DetectorType == DET_HESSIAN)
      effectiveThreshold = effectiveThreshold*effectiveThreshold;

    keypointCallback = 0;
  }
  void setKeypointCallback(KeypointCallback *callback)
  {
    keypointCallback = callback;
  }
  void detectPyramidKeypoints(const Mat &image);
  int extrema_points;
  int localized_points;
  float effectiveThreshold;
  std::string filters;
  float scale_coef_tilde;
  bool tilde_only_positive;

protected:
  void detectOctaveKeypoints(const Mat &firstLevel, float pixelDistance, Mat &nextOctaveFirstLevel);
  void localizeKeypoint(int r, int c, float curScale, float pixelDistance);
  void findLevelKeypoints(float curScale, float pixelDistance);
  Mat Response(const Mat &inputImage, float norm);
  Mat iidogResponse(const Mat &inputImage, float norm);
  Mat dogResponse(const Mat &inputImage, float norm);
  Mat HessianResponse(const Mat &inputImage, float norm);
  Mat TILDEResponse(const Mat &inputImage, float norm);
  Mat HarrisResponse(const Mat &inputImage, float norm);
  Mat CaffeGradResponse(const Mat &inputImage, float norm);
  const Mat* originalImg;

private:
  // some constants derived from parameters
  const double edgeScoreThreshold;
  float finalThreshold;
  float positiveThreshold;
  float negativeThreshold;

  // temporary arrays used by protected functions
  Mat octaveMap;
  Mat prevBlur, blur;
  Mat low, cur, high;
};


\
#endif // __PYRAMID_H__
