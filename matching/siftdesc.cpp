/*
 * Copyright (C) 2008-12 Michal Perdoch
 * All rights reserved.
 *
 * This file is part of the HessianAffine detector and is made available under
 * the terms of the BSD license (see the COPYING file).
 *
 */
#undef __STRICT_ANSI__
#include <vector>
#include "siftdesc.h"
#include <iostream>
using namespace std;
using cv::Mat;

//#include <iostream>

#define M_PI_DOUBLED 6.28318530718
const double M_PI_255 = 255.0/M_PI;
// The SIFT descriptor is subject to US Patent 6,711,293

void SIFTDescriptor::precomputeBinsAndWeights()
{
  int halfSize = par.PEParam.patchSize>>1;
  float step = float(par.spatialBins+1)/(2*halfSize);

  // allocate maps at the same location
  precomp_bins.resize(2*par.PEParam.patchSize);
  precomp_weights.resize(2*par.PEParam.patchSize);
  bin1 = bin0 = &precomp_bins.front();
  bin1 += par.PEParam.patchSize;
  w1   =   w0 = &precomp_weights.front();
  w1 += par.PEParam.patchSize;

  // maps every pixel in the patch 0..patch_size-1 to appropriate spatial bin and weight
  for (int i = 0; i < par.PEParam.patchSize; i++)
  {
    float x = step*i;      // x goes from <-1 ... spatial_bins> + 1
    int  xi = (int)(x);
    // bin indices
    bin0[i] = xi-1; // get real xi
    bin1[i] = xi;
    // weights
    w1[i]   = x - xi;
    w0[i]   = 1.0f - w1[i];
    // truncate weights and bins in case they reach outside of valid range
    if (bin0[i] <          0)
    {
      bin0[i] = 0;
      w0[i] = 0;
    }
    if (bin0[i] >= par.spatialBins)
    {
      bin0[i] = par.spatialBins-1;
      w0[i] = 0;
    }
    if (bin1[i] <          0)
    {
      bin1[i] = 0;
      w1[i] = 0;
    }
    if (bin1[i] >= par.spatialBins)
    {
      bin1[i] = par.spatialBins-1;
      w1[i] = 0;
    }
    // adjust for orientation bin skip
    bin0[i] *= par.orientationBins;
    bin1[i] *= par.orientationBins;
  }
}

void SIFTDescriptor::samplePatch()
{

  for (int r = 0; r < par.PEParam.patchSize; ++r)
  {
    const int br0 = par.spatialBins * bin0[r];
    const float wr0 = w0[r];
    const int br1 = par.spatialBins * bin1[r];
    const float wr1 = w1[r];
    const float* maskRow = mask.ptr<float>(r);
    const float* gradRow = grad.ptr<float>(r);
    const float* oriRow = ori.ptr<float>(r);


    for (int c = 0; c < par.PEParam.patchSize; ++c)
    {
      float val = float(par.magnLess) * 1.0 + (1.0 - float(par.magnLess))*maskRow[c]*gradRow[c];

      const int bc0 = bin0[c];
      const float wc0 = w0[c]*val;
      const int bc1 = bin1[c];
      const float wc1 = w1[c]*val;

      // ori from atan2 is in range <-pi,pi> so add 2*pi to be surely above zero
      const float o = float(par.orientationBins)*(oriRow[c] + M_PI_DOUBLED)/M_PI_DOUBLED;

      int   bo0 = (int)o;
      const float wo1 =  o - bo0;
      bo0 %= par.orientationBins;

      int   bo1 = (bo0+1) % par.orientationBins;
      const float wo0 = 1.0f - wo1;
      val = wr0*wc0;
      if (val>0)
      {
        vec[br0+bc0+bo0] += val * wo0;
        vec[br0+bc0+bo1] += val * wo1;
      }
      val = wr0*wc1;
      if (val>0)
      {
        vec[br0+bc1+bo0] += val * wo0;
        vec[br0+bc1+bo1] += val * wo1;
      }
      val = wr1*wc0;
      if (val>0)
      {
        vec[br1+bc0+bo0] += val * wo0;
        vec[br1+bc0+bo1] += val * wo1;
      }
      val = wr1*wc1;
      if (val>0)
      {
        vec[br1+bc1+bo0] += val * wo0;
        vec[br1+bc1+bo1] += val * wo1;
      }
    }
  }
}

double SIFTDescriptor::normalize(std::vector<double>& vec1)
{
  double vec1tlen = 0.0;
  if (vec1.size() % 4 == 0) //loop unrolling
    for (size_t i = 0; i < vec1.size(); i+=4)
    {
      const double sq0 = vec1[i]*vec1[i];
      const double sq1 = vec1[i+1]*vec1[i+1];
      const double sq2 = vec1[i+2]*vec1[i+2];
      const double sq3 = vec1[i+3]*vec1[i+3];
      vec1tlen += sq0+sq1+sq2+sq3;
    }
  else
    //plain version
    for (size_t i = 0; i < vec1.size(); i++)
    {
      const double val0 = (double) vec1[i];
      vec1tlen += (val0 * val0);
    }
  vec1tlen = sqrt(vec1tlen);
  const double fac = 1.0 / vec1tlen;
  for (size_t i = 0; i < vec1.size(); i++) {
    vec1[i] *= fac;
  }
  return vec1tlen;
}

float SIFTDescriptor::normalize(std::vector<float>& vec1)
{
  float vec1tlen = 0.0f;
  if (vec1.size() == 128) //loop unrolling
    for (size_t i = 0; i < vec1.size(); i+=4)
    {
      const float sq0 = vec1[i]*vec1[i];
      const float sq1 = vec1[i+1]*vec1[i+1];
      const float sq2 = vec1[i+2]*vec1[i+2];
      const float sq3 = vec1[i+3]*vec1[i+3];
      vec1tlen += sq0+sq1+sq2+sq3;
    }
  else
    //plain version
    for (size_t i = 0; i < vec1.size(); i++)
    {
      const float val0 = (float) vec1[i];
      vec1tlen += (val0 * val0);
    }
  vec1tlen = sqrt(vec1tlen);
  const double fac = 1.0 / vec1tlen;
  for (size_t i = 0; i < vec1.size(); i++) {
    vec1[i] *= (float)fac;
  }
  return vec1tlen;
}
void SIFTDescriptor::sample(bool do_norm)
{
  type = DESC_SIFT;
  for (size_t i = 0; i < vec.size(); i++) {
    vec[i]=0;
  }
  // accumulate histograms
  samplePatch();

  if (do_norm) {
    SIFTnorm(vec);
  }
}
void SIFTDescriptor::RootSIFTnorm(std::vector<double> &in_vect)
{
  normalize(in_vect);
  // check if there are some values above threshold
  bool changed = false;
  for (size_t i = 0; i < in_vect.size(); i++)
    if (in_vect[i] > par.maxBinValue) {
      in_vect[i] = par.maxBinValue;
      changed = true;
    }
  if (changed) normalize(in_vect);

  double sum = 0.;
  for (size_t i = 0; i < in_vect.size(); i++)
    sum += fabs(in_vect[i]);
  for (size_t i = 0; i < in_vect.size(); i++)
    in_vect[i] = sqrt(in_vect[i] / sum);

  for (size_t i = 0; i < in_vect.size(); i++) {
    int b = MAX(0, MIN((int) (512.0 * in_vect[i] + 0.5), 255)); //0.5 - for appropriate rounding.
    //It is important, that all SIFT lengths are normalized to 512.
    in_vect[i] = double(b);
  }
}
void SIFTDescriptor::RootSIFTnorm(std::vector<float> &in_vect)
{
  normalize(in_vect);
  // check if there are some values above threshold
  bool changed = false;
  for (size_t i = 0; i < in_vect.size(); i++)
    if (in_vect[i] > par.maxBinValue) {
      in_vect[i] = (float) par.maxBinValue;
      changed = true;
    }
  if (changed) normalize(in_vect);

  double sum = 0.;
  for (size_t i = 0; i < in_vect.size(); i++)
    sum += fabs(in_vect[i]);
  for (size_t i = 0; i < in_vect.size(); i++)
    in_vect[i] = (float) sqrt(in_vect[i] / sum);

  for (size_t i = 0; i < in_vect.size(); i++) {
    int b = MAX(0, MIN((int) (512.0 * in_vect[i] + 0.5), 255)); //0.5 - for appropriate rounding.
    //It is important, that all SIFT lengths are normalized to 512.
    in_vect[i] = float(b);
  }
}
void SIFTDescriptor::SIFTnorm(std::vector<double> &in_vect) {
  normalize(in_vect);
// check if there are some values above threshold
  bool changed = false;
  for (size_t i = 0; i < vec.size(); i++)
    if (in_vect[i] > par.maxBinValue) {
      in_vect[i] = par.maxBinValue;
      changed = true;
    }
  if (changed) normalize(in_vect);
  for (size_t i = 0; i < in_vect.size(); i++) {
    int b = MAX(0, MIN((int) (512.0f * in_vect[i] + 0.5), 255)); //0.5 - for appropriate rounding.
//It is important, that all SIFT lengths are normalized to 512.
    in_vect[i] = double(b);
  }
}
void SIFTDescriptor::SIFTnorm(std::vector<float> &in_vect) {
  normalize(in_vect);
// check if there are some values above threshold
  bool changed = false;
  for (size_t i = 0; i < vec.size(); i++)
    if (in_vect[i] > par.maxBinValue) {
      in_vect[i] = (float) par.maxBinValue;
      changed = true;
    }
  if (changed) normalize(in_vect);
  for (size_t i = 0; i < in_vect.size(); i++) {
    int b = MAX(0, MIN((int) (512.0f * in_vect[i] + 0.5), 255)); //0.5 - for appropriate rounding.
//It is important, that all SIFT lengths are normalized to 512.
    in_vect[i] = float(b);
  }
}
void SIFTDescriptor::rootsample(bool do_norm) {
  for (size_t i = 0; i < vec.size(); i++) {
    vec[i] = 0;
  }
// accumulate histograms
  samplePatch();
  if (do_norm) {
    RootSIFTnorm(vec);
  }
}

void SIFTDescriptor::computeSiftDescriptor(Mat &patch)
{
  const int width = patch.cols;
  const int height = patch.rows;
  //   float *pPtr = (float*)patch.data;

  // photometrically normalize with weights as in SIFT gradient magnitude falloff
  float mean, var;
 // photometricallyNormalize(patch, mask, mean, var);

  //prepare gradients
  for (int r = 0; r < height; ++r)
  {
    float* Row0 = patch.ptr<float>(r);  //Mishkin. Got rig of at<>`s
    float* Row1 = patch.ptr<float>(r+1);
    float* Rowi1 = patch.ptr<float>(r-1);
    float* gradPtr = grad.ptr<float>(r);
    float* oriPtr = ori.ptr<float>(r);
    for (int c = 0; c < width; ++c)
    {
      float xgrad, ygrad;
      if (c == 0) xgrad = Row0[c+1]- Row0[c];
      else if (c == width-1) xgrad = Row0[c] - Row0[c-1];
      else
        xgrad = Row0[c+1] - Row0[c-1];

      if (r == 0) ygrad = Row1[c] - Row0[c];
      else if (r == height-1) ygrad = Row0[c] - Rowi1[c];
      else
        ygrad = Row1[c]-Rowi1[c];

      gradPtr[c] = ::sqrt(xgrad * xgrad + ygrad * ygrad);
      oriPtr[c] = atan2LUTff(ygrad, xgrad);
      //oriPtr[c] = atan2(ygrad, xgrad);
    }
  }
  sample(par.doNorm);
  //original function
  //  for (int r = 0; r < height; ++r)
  //    for (int c = 0; c < width; ++c)
  //      {
  //        float xgrad, ygrad;
  //        if (c == 0) xgrad = patch.at<float>(r,c+1) - patch.at<float>(r,c);
  //        else if (c == width-1) xgrad = patch.at<float>(r,c) - patch.at<float>(r,c-1);
  //        else
  //          xgrad = patch.at<float>(r,c+1) - patch.at<float>(r,c-1);

  //        if (r == 0) ygrad = patch.at<float>(r+1,c) - patch.at<float>(r,c);
  //        else if (r == height-1) ygrad = patch.at<float>(r,c) - patch.at<float>(r-1,c);
  //        else
  //          ygrad = patch.at<float>(r+1,c) - patch.at<float>(r-1,c);
  //        grad.at<float>(r,c) = ::sqrt(xgrad * xgrad + ygrad * ygrad);
  //        ori.at<float>(r,c) = atan2(ygrad, xgrad);
  //      }
  // compute SIFT vector
}
void SIFTDescriptor::computeRootSiftDescriptor(Mat &patch)
{
  const int width = patch.cols;
  const int height = patch.rows;
  // photometrically normalize with weights as in SIFT gradient magnitude falloff
  float mean, var;
//  photometricallyNormalize(patch, mask, mean, var);
  // prepare gradients
  for (int r = 0; r < height; ++r)
  {
    float* Row0 = patch.ptr<float>(r);  //Mishkin. Got rig of at<>`s
    float* Row1 = patch.ptr<float>(r+1);
    float* Rowi1 = patch.ptr<float>(r-1);
    float* gradPtr = grad.ptr<float>(r);
    float* oriPtr = ori.ptr<float>(r);
    for (int c = 0; c < width; ++c)
    {
      float xgrad, ygrad;
      if (c == 0) xgrad = Row0[c+1]- Row0[c];
      else if (c == width-1) xgrad = Row0[c] - Row0[c-1];
      else
        xgrad = Row0[c+1] - Row0[c-1];

      if (r == 0) ygrad = Row1[c] - Row0[c];
      else if (r == height-1) ygrad = Row0[c] - Rowi1[c];
      else
        ygrad = Row1[c]-Rowi1[c];

      gradPtr[c] = ::sqrt(xgrad * xgrad + ygrad * ygrad);
      oriPtr[c] = atan2LUTff(ygrad, xgrad);
      //      oriPtr[c] = atan2(ygrad, xgrad);
    }
  }
  rootsample(par.doNorm);
  //original
  //  for (int r = 0; r < height; ++r)
  //  {
  //    for (int c = 0; c < width; ++c)
  //    {
  //        float xgrad, ygrad;
  //        if (c == 0) xgrad = patch.at<float>(r,c+1) - patch.at<float>(r,c);
  //        else if (c == width-1) xgrad = patch.at<float>(r,c) - patch.at<float>(r,c-1);
  //        else
  //          xgrad = patch.at<float>(r,c+1) - patch.at<float>(r,c-1);

  //        if (r == 0) ygrad = patch.at<float>(r+1,c) - patch.at<float>(r,c);
  //        else if (r == height-1) ygrad = patch.at<float>(r,c) - patch.at<float>(r-1,c);
  //        else
  //          ygrad = patch.at<float>(r+1,c) - patch.at<float>(r-1,c);

  //        grad.at<float>(r,c) = ::sqrt(xgrad * xgrad + ygrad * ygrad);
  //        ori.at<float>(r,c) = atan2(ygrad, xgrad);
  // }
  // compute RootSIFT vector
}
void SIFTDescriptor::operator()(cv::Mat &patch, std::vector<float>& desc)
{
  if (par.doHalfSIFT) {
    par.doNorm = false;
  }
  if (par.useRootSIFT)
    computeRootSiftDescriptor(patch);
  else
    computeSiftDescriptor(patch);

  if (par.doHalfSIFT) // HalfSIFT
  {
    par.doNorm = true;
    int spBins = par.spatialBins * par.spatialBins;
    int oriHalf = par.orientationBins / 2;
    int half_size = spBins * oriHalf;
    std::vector<double> half_vec(half_size);

    int bin1=0;
    for (int i=0; i < spBins; i++) {
      for (int j=0; j < oriHalf; j++) {
        half_vec[bin1] = vec[i*par.orientationBins + j] + vec[i*par.orientationBins + j+oriHalf];
        bin1++;
      }
    }
    if (par.useRootSIFT){
      RootSIFTnorm(half_vec);
    } else {
      SIFTnorm(half_vec);
    }

    desc.resize(half_vec.size());
    for (unsigned int i = 0; i < half_vec.size(); i++) {
      desc[i] = (float) half_vec[i];
    }
  }
  else { //Normal SIFT
    desc.resize(vec.size());
    for (unsigned int i = 0; i < vec.size(); i++) {
      desc[i]=(float) vec[i];
    }
  }
}
