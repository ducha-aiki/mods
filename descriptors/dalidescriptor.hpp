#ifndef DALIDESCRIPTOR_HPP
#define DALIDESCRIPTOR_HPP

#include "detectors/structures.hpp"

#include "../dali/dali.h"

struct DALIParams
{
    PatchExtractionParams PEParam;
//  int patchSize;
//  double mrSize;
//  bool FastPatchExtraction;
  DALIParams()
  {
//    patchSize = 41;
//    mrSize =  3.0*sqrt(3.0);
//    FastPatchExtraction = false;
  }
};

struct DALIDescriptor
{
public:
  DALIDescriptor(const DALIParams &par)
  {
    this->par = par;
    type = DESC_DALI;

    temp_pt.pt.x = par.PEParam.patchSize / 2;
    temp_pt.pt.y = par.PEParam.patchSize / 2;
    temp_pt.angle = 0;
    temp_pt.size = float (par.PEParam.patchSize) / par.PEParam.mrSize;
    temp_pt.octave = 1;
    temp_pt.class_id = 1;
    //   keypoints_1.push_back(temp_pt);
    dali_optsDefault( &params );
    params.Sz_coarse = 10;
    params.mtype = DALI_MESH_TYPE_CIRCLE_VARIABLE;
    params.mesh_K = 1000.;
    params.mesh_sigma = 0.5;
    params.Sz    = 20;
    params.wmax  = 20;
    params.ncomp = 100;
    params.ntime = 100;
    params.verbose = 1;
  }
  void operator()(cv::Mat &patch, std::vector<float>& desc)
  {
    cv::Mat patch64;
    patch.convertTo(patch64,CV_64F);
    const double* patchPtr = patch64.ptr<double>(0);
    int n = 1;
    int uc[1];
    int vc[1];
    uc[0] = (int)temp_pt.pt.x;
    vc[0] = (int)temp_pt.pt.y;

    desc_dali1 = dali_compute( patchPtr, par.PEParam.patchSize, par.PEParam.patchSize, uc,
                               vc, n, &params, &info );
    dali_fprintInfo( stdout, &info );
    desc.resize(desc_dali1->len*desc_dali1[0].wlen);
    int jj=0;
    for (int u=0; u<desc_dali1->len; u++) {
       for (int k=0; k<desc_dali1[0].wlen; k++) {
           desc[jj] = (float) desc_dali1[0].desc[u*desc_dali1[0].wlen + k];
         jj++;
         }
      }
  }
public:
  descriptor_type type;
  int desc_size;

private:
  DALIParams par;
  //  std::vector<cv::KeyPoint> keypoints_1; //for binary-dets
  cv::KeyPoint temp_pt;
  dali_params_t params;
  dali_info_t info;
  dali_t *desc_dali1;
};


#endif // DALIDESCRIPTOR_HPP
