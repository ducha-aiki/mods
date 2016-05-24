#ifndef KAZEDESCRIPTOR_HPP
#define KAZEDESCRIPTOR_HPP
//
// Created by old-ufo on 5/2/15.
//
#include "detectors/structures.hpp"
#include "../akaze/src/lib/AKAZE.h"
struct KAZEParams
{
    PatchExtractionParams PEParam;
//  int patchSize;
//  double mrSize;
//  bool FastPatchExtraction;

  KAZEParams()
  {
//    patchSize = 41;
//    mrSize =  3.0*sqrt(3.0);
//    FastPatchExtraction = false;
  }
};

struct KAZEDescriptor
{
public:
  KAZEDescriptor(const KAZEParams &par)
  {
    this->par = par;
    options.descriptor = aka::MLDB_UPRIGHT;
    options.img_width = par.PEParam.patchSize;
    options.img_height = par.PEParam.patchSize;
    options.descriptor_size = 0;

    img_32 = cv::Mat::zeros(par.PEParam.patchSize,par.PEParam.patchSize,CV_32F);

    evolution1 = new aka::AKAZE(options);
    type = DESC_KAZE;
    cv::KeyPoint temp_pt;
    temp_pt.pt.x = par.PEParam.patchSize / 2;
    temp_pt.pt.y = par.PEParam.patchSize / 2;
    temp_pt.angle = 0;
    temp_pt.size = float (par.PEParam.patchSize) / par.PEParam.mrSize;
    temp_pt.octave = 1;
    temp_pt.class_id = 1;
    keypoints_1.push_back(temp_pt);
  }
  void operator()(cv::Mat &patch, std::vector<float>& desc)
  {
    patch.convertTo(img_32, CV_32F, 1.0/255.0, 0);
    evolution1->Create_Nonlinear_Scale_Space(img_32);
    evolution1->Compute_Descriptors(keypoints_1, descriptors_1);
    desc_size = descriptors_1.cols;
    desc.resize(desc_size);
    unsigned char *descPtr = descriptors_1.ptr<unsigned char>(0);
    for (int jj = 0; jj < desc_size; jj++, descPtr++)
      desc[jj] = (float) *descPtr;
  }
public:
  descriptor_type type;
  int desc_size;

private:
  KAZEParams par;
  std::vector<cv::KeyPoint> keypoints_1; //for binary-dets
  cv::Mat descriptors_1; //for binary-dets
  cv::Mat CharImage;
  aka::AKAZEOptions options; //For KAZE
  aka::AKAZE* evolution1;
  cv::Mat img_32;

  };


#endif // KAZEDESCRIPTOR_HPP
