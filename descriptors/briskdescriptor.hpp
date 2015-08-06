#ifndef BRISKDESCRIPTOR_HPP
#define BRISKDESCRIPTOR_HPP

#include "../detectors/structures.hpp"
#include "../detectors/detectors_parameters.hpp"


struct BRISKDescriptor
{
public:
  BRISKDescriptor(const BRISKParams &par)
  {
    this->par = par;
    CurrentDescriptor = new cv::BRISK(par.thresh, par.octaves,par.patternScale);
    type = DESC_BRISK;
    cv::KeyPoint temp_pt;
    temp_pt.pt.x = par.PEParam.patchSize / 2;
    temp_pt.pt.y = par.PEParam.patchSize / 2;
    temp_pt.angle = 0;
    temp_pt.size = float (par.PEParam.patchSize) / par.PEParam.mrSize;
    temp_pt.octave = 1;
    temp_pt.response = 1.5f;
    keypoints_1.push_back(temp_pt);
    CharImage=cv::Mat::zeros(par.PEParam.patchSize,par.PEParam.patchSize,CV_8U);
  }
  void operator()(cv::Mat &patch, std::vector<float>& desc)
  {
    patch.convertTo(CharImage, CV_8U);
    CurrentDescriptor->compute(CharImage,keypoints_1, descriptors_1);
    desc_size = descriptors_1.cols;
    desc.resize(desc_size);
    unsigned char *descPtr = descriptors_1.ptr<unsigned char>(0);
    for (int jj = 0; jj < desc_size; jj++, descPtr++)
      desc[jj] = (float) *descPtr;
  }
public:
  descriptor_type type;
  int desc_size;
  cv::BRISK* CurrentDescriptor;

private:
  BRISKParams par;
  std::vector<cv::KeyPoint> keypoints_1; //for binary-dets
  cv::Mat descriptors_1; //for binary-dets
  cv::Mat CharImage;

};

#endif // BRISKDESCRIPTOR_HPP
