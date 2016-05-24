#ifndef ORBDESCRIPTOR_HPP
#define ORBDESCRIPTOR_HPP
#include "detectors/structures.hpp"
#include "../detectors/detectors_parameters.hpp"


struct ORBDescriptor
{
public:
  ORBDescriptor(const ORBParams &par)
  {
    CurrentDescriptor = new cv::OrbFeatureDetector(par.nfeatures,
                                                   1,
                                                   1,
                                                   par.edgeThreshold,
                                                   par.firstLevel,
                                                   par.WTA_K,
                                                   ORB::HARRIS_SCORE,
                                                   par.PEParam.patchSize);

    this->par = par;
    type = DESC_ORB;
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
    //   std::cerr << descriptors_1.cols << " " << descriptors_1.rows << std::endl;
    //   CurrentDescriptor->compute(CharImage, keypoints_1, descriptors_1);
    //    std::cerr << descriptors_1.cols << " " << descriptors_1.rows << std::endl;
    //    std::cerr << keypoints_1.size() << std::endl;
    CurrentDescriptor->compute(CharImage,keypoints_1, descriptors_1);
    //        std::cerr << descriptors_1.cols << " " << descriptors_1.rows << std::endl;
    //        std::cerr << keypoints_1.size() << std::endl;
    desc_size = descriptors_1.cols;
    desc.resize(desc_size);
    unsigned char *descPtr = descriptors_1.ptr<unsigned char>(0);
    for (int jj = 0; jj < desc_size; jj++, descPtr++)
      desc[jj] = (float) *descPtr;
  }
public:
  descriptor_type type;
  int desc_size;
  cv::ORB* CurrentDescriptor;

private:
  ORBParams par;
  std::vector<cv::KeyPoint> keypoints_1; //for binary-dets
  cv::Mat descriptors_1; //for binary-dets
  cv::Mat CharImage;

};

#endif // ORBDESCRIPTOR_HPP
