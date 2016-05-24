/*
 * Copyright (C) 2008-12 Michal Perdoch
 * All rights reserved.
 *
 * This file is part of the HessianAffine detector and is made available under
 * the terms of the BSD license (see the COPYING file).
 *
 */

// The SIFT descriptor is subject to US Patent 6,711,293

#ifndef __SIFTDESC_H__
#define __SIFTDESC_H__

#include <vector>

#include <opencv2/core/core.hpp>
#include "../detectors/helpers.h"
#include "../detectors/structures.hpp"

struct DomainSizePolingParams
{
  int numScales;
  double startCoef;
  double endCoef;
  DomainSizePolingParams() {
    numScales = 3;
    startCoef = 0.5;
    endCoef = 1.5;
  }
};
struct SIFTDescriptorParams
{
  int spatialBins;
  int orientationBins;
  double maxBinValue;
  int patchSize;
  char useRootSIFT;
//  double mrSize;
//  double mrSizeOri;
  bool FastPatchExtraction;
  int doHalfSIFT;
  int dims;
  int maxOrientations;
  bool estimateOrientation;
  double orientTh;
  bool doNorm;
  bool magnLess;
  DomainSizePolingParams DSPParam;
  PatchExtractionParams PEParam;
  SIFTDescriptorParams()
  {
    spatialBins = 4;
    orientationBins = 8;
    maxBinValue = 0.2f;
    patchSize = 41;
    useRootSIFT=0;
    //  mrSize = 3.0*sqrt(3.0);
    //  mrSizeOri = mrSize;
    doHalfSIFT = 0;
    dims = spatialBins*spatialBins*orientationBins;
    maxOrientations = 0;
    estimateOrientation= true;
    doNorm=true;
    orientTh = 0.8;
    magnLess = false;
    //  FastPatchExtraction = false;
  }
};


struct SIFTDescriptor
{

public:
  // top level interface
  SIFTDescriptor(const SIFTDescriptorParams &par) :
    mask(par.PEParam.patchSize, par.PEParam.patchSize, CV_32FC1),
    grad(par.PEParam.patchSize, par.PEParam.patchSize, CV_32FC1),
    ori(par.PEParam.patchSize, par.PEParam.patchSize, CV_32FC1)
  {
    this->par = par;
    if (par.useRootSIFT) type = DESC_ROOT_SIFT;
    else
      type = DESC_SIFT;
    vec.resize(par.spatialBins * par.spatialBins * par.orientationBins);
    computeCircularGaussMask(mask);
    precomputeBinsAndWeights();
  }

  void computeSiftDescriptor(cv::Mat &patch);
  void computeRootSiftDescriptor(cv::Mat &patch);
  void operator()(cv::Mat &patch, std::vector<float>& desc);

public:
  std::vector<double> vec;
  void SIFTnorm(std::vector<float> &in_vect);
  void RootSIFTnorm(std::vector<float> &in_vect);
  void SIFTnorm(std::vector<double> &in_vect);
  void RootSIFTnorm(std::vector<double> &in_vect);
  descriptor_type type;

private:
  // helper functions

  double normalize(std::vector<double>& vec1);
  float normalize(std::vector<float>& vec1);

  void sample(bool do_norm);
  void rootsample(bool do_norm);
  void samplePatch();
  void precomputeBinsAndWeights();

private:
  SIFTDescriptorParams par;
  cv::Mat mask, grad, ori;
  std::vector<int> precomp_bins;
  std::vector<double> precomp_weights;
  int *bin0, *bin1;
  double *w0, *w1;
};

#endif //__SIFTDESC_H__
