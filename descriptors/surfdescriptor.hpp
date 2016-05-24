#ifndef SURFDESCRIPTOR_HPP
#define SURFDESCRIPTOR_HPP
//
// Created by old-ufo on 5/2/15.
//
#include "../opensurf/surflib.h"
#include "detectors/structures.hpp"
#include "../detectors/detectors_parameters.hpp"

struct SURFDescriptor
{
public:
  SURFDescriptor(const SURFParams &par)
  {
    desc_size = 64;
    this->par = par;
    type = DESC_SURF;
    Ipoint temp_pt;
    temp_pt.x = (float) par.PEParam.patchSize / 2.0f;
    temp_pt.y = (float) par.PEParam.patchSize / 2.0f;
    temp_pt.orientation = 0.0f;
    temp_pt.scale = float (par.PEParam.patchSize) / par.PEParam.mrSize;
    ipts1.push_back(temp_pt);
  }
  void operator()(cv::Mat &patch, std::vector<float>& desc)
  {
    Iplimg1 = patch;
    int_img = Integral(&Iplimg1);
    // Create Surf Descriptor Object
    Surf des(int_img, ipts1);

    des.getDescriptors(true);
    desc.resize(desc_size);
    for (int jj = 0; jj < desc_size; jj++) {
        desc[jj] = ipts1[0].descriptor[jj];
      }
  }
public:
  descriptor_type type;
  int desc_size;
private:
  SURFParams par;
  IplImage Iplimg1;
  IplImage *int_img;
  IpVec ipts1;

};

#endif // SURFDESCRIPTOR_HPP
