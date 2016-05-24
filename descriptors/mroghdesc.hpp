#ifndef MROGH_HPP
#define MROGH_HPP



#include <vector>
#include <opencv2/core/core.hpp>
#include "../detectors/structures.hpp"
#include "../mrogh/mrogh.h"
#include "detectors/structures.hpp"
#include <iostream>

struct MROGHParams
{
  int nDir;
  int nOrder;
  int nMultiRegion;
    PatchExtractionParams PEParam;
//  int patchSize;
//  double mrSize;
//  bool FastPatchExtraction;
  MROGHParams()
  {
    nDir = 8;
    nOrder = 6;
    nMultiRegion = 4;
//    mrSize = 3.0*sqrt(3.0);
//    patchSize = 41;
//    FastPatchExtraction = false;
  }
};

struct MROGHDescriptor
{
public:
  MROGHDescriptor(const MROGHParams &par)
  {
    this->par = par;
    type = DESC_MROGH;
    int desc_size = par.nDir * par.nOrder * par.nMultiRegion;
  }
  void operator()(cv::Mat& img,const AffineRegionVector& temp_kp1,AffineRegionVector& temp_kp1_desc)
  {
    /// Data preparation
    int desc_size = par.nDir *
                    par.nOrder *
                    par.nMultiRegion;

    img.convertTo(CharImage, CV_8U);
    m_pImg = CharImage;
    unsigned int kp_size = temp_kp1.size();
    temp_kp1_desc.resize(kp_size);
    OxKey *pKeys = new OxKey[kp_size];
    ///Description
    ///
    cv::Mat mask(par.PEParam.patchSize,par.PEParam.patchSize,CV_32F);
    computeCircularGaussMask(mask);
    for (unsigned int kp_num = 0; kp_num < kp_size; kp_num++) {
      pKeys[kp_num].x = temp_kp1[kp_num].det_kp.x;
      pKeys[kp_num].y = temp_kp1[kp_num].det_kp.y;
      pKeys[kp_num].trans[0] = temp_kp1[kp_num].det_kp.a11 * temp_kp1[kp_num].det_kp.s;
      pKeys[kp_num].trans[1] = temp_kp1[kp_num].det_kp.a12 * temp_kp1[kp_num].det_kp.s;
      pKeys[kp_num].trans[2] = temp_kp1[kp_num].det_kp.a21 * temp_kp1[kp_num].det_kp.s;
      pKeys[kp_num].trans[3] = temp_kp1[kp_num].det_kp.a22 * temp_kp1[kp_num].det_kp.s;

      temp_kp1_desc[kp_num] = temp_kp1[kp_num];
      temp_kp1_desc[kp_num].desc.type = DESC_MROGH;
      temp_kp1_desc[kp_num].desc.vec.resize(desc_size);

      int *desc = 0;
      desc = Extract_MROGH(pKeys[kp_num], &m_pImg,
                           par.nDir,
                           par.nOrder,
                           par.nMultiRegion, par.PEParam.patchSize,
                           par.PEParam.photoNorm, mask);
      for (int jj = 0; jj < desc_size; jj++, desc++)
        temp_kp1_desc[kp_num].desc.vec[jj] = (float) *desc;
    }
    delete[] pKeys;
  }
public:
  descriptor_type type;

private:
  MROGHParams par;
  int desc_size;
  cv::Mat CharImage;
  IplImage m_pImg;
  OxKey pKey;
};


#endif // MROGH_HPP
