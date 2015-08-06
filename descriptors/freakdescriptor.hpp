#ifndef FREAKDESCRIPTOR_HPP
#define FREAKDESCRIPTOR_HPP
//
// Created by old-ufo on 5/2/15.
//
#include "detectors/structures.hpp"

struct FREAKParams
{
  bool orientationNormalized;
  bool scaleNormalized;
  float patternScale;
  int nOctaves;
  PatchExtractionParams PEParam;
  //  int patchSize;
//  double mrSize;
//  bool FastPatchExtraction;
  FREAKParams()
  {
//    patchSize = 41;
//    mrSize =  3.0*sqrt(3.0);
//    FastPatchExtraction = false;
    orientationNormalized=false;//true;
    scaleNormalized=false;//true;
    patternScale = 22.0;
    nOctaves=4;
  }
};

struct FREAKDescriptor
{
public:
  FREAKDescriptor(const FREAKParams &par)
  {
    CurrentDescriptor = new cv::FREAK(par.orientationNormalized,
                       par.scaleNormalized,
                       par.patternScale,
                       par.nOctaves);
    this->par = par;
    type = DESC_FREAK;
    cv::KeyPoint temp_pt;
    temp_pt.pt.x = par.PEParam.patchSize / 2;
    temp_pt.pt.y = par.PEParam.patchSize / 2;
    temp_pt.angle = 0;
    temp_pt.size = float (par.PEParam.patchSize) / par.PEParam.mrSize;
    keypoints_1.push_back(temp_pt);
  }
  void operator()(cv::Mat &patch, std::vector<float>& desc)
  {
    patch.convertTo(CharImage, CV_8U);
    //   std::cerr << descriptors_1.cols << " " << descriptors_1.rows << std::endl;
    CurrentDescriptor->compute(CharImage, keypoints_1, descriptors_1);
    //    std::cerr << descriptors_1.cols << " " << descriptors_1.rows << std::endl;
    desc_size = descriptors_1.cols;
    desc.resize(desc_size);
    unsigned char *descPtr = descriptors_1.ptr<unsigned char>(0);
    for (int jj = 0; jj < desc_size; jj++, descPtr++)
      desc[jj] = (float) *descPtr;
  }
public:
  descriptor_type type;
  int desc_size;
  cv::FREAK* CurrentDescriptor;

private:
  FREAKParams par;
  std::vector<cv::KeyPoint> keypoints_1; //for binary-dets
  cv::Mat descriptors_1; //for binary-dets
  cv::Mat CharImage;

};

#endif // FREAKDESCRIPTOR_HPP
