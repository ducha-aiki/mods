#ifndef LIOPDESC_HPP
#define LIOPDESC_HPP


#ifdef __cplusplus
extern "C"
{
#endif
#include "vlfeat/vl/generic.h"
#include "vlfeat/vl/liop.h"
#ifdef __cplusplus
}
#endif

#include <vector>
#include <opencv2/core/core.hpp>
#include "../detectors/structures.hpp"
#include "detectors/structures.hpp"
#include <iostream>
struct LIOPDescriptorParams
{
  int neighbours;
  int bins;
  float radius;
  float threshold;
//  int patchSize;
//  double mrSize;
//  bool FastPatchExtraction;
  PatchExtractionParams PEParam;
  LIOPDescriptorParams()
  {
    neighbours = 4;
    bins = 6;
    radius = 6;
    threshold = 5.0f;
//    mrSize = 3.0*sqrt(3.0);
//    patchSize = 41;
//    FastPatchExtraction = false;
  }
};

struct LIOPDescriptor
{
public:
  LIOPDescriptor(const LIOPDescriptorParams &par)
  {
    this->par = par;
    type = DESC_LIOP;

    liop = vl_liopdesc_new_basic (par.PEParam.patchSize);
    // allocate the descriptor array
    dimension = vl_liopdesc_get_dimension(liop) ;
    desc1 = (float*) vl_malloc(sizeof(float) * dimension) ;
  }
  void operator()(cv::Mat &patch, std::vector<float>& desc)
  {
    float *patchPtr = (float*)(patch.data);
    vl_liopdesc_process(liop, desc1, patchPtr);
    desc.resize(dimension);
    for (unsigned int i=0; i<dimension; i++)
      desc[i]=desc1[i];

  }
public:
  std::vector<double> vec;
  descriptor_type type;

private:
  LIOPDescriptorParams par;
  VlLiopDesc* liop;
  vl_size dimension;
  float* desc1;
};


#endif // LIOPDESC_HPP
