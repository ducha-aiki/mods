#ifndef DAISYDESCRIPTOR_HPP
#define DAISYDESCRIPTOR_HPP
#include "detectors/structures.hpp"
#include "../detectors/detectors_parameters.hpp"
#include "libdaisy/include/daisy/daisy.h"
struct DAISYParams
{

  int rad;
  int radq;
  int thq;
  int histq ;
    PatchExtractionParams PEParam;
//  int patchSize;
//  double mrSize;
//  bool FastPatchExtraction;
  int nrm_type;
//  bool photoNorm;
  DAISYParams()
  {
//    patchSize = 41;
//    mrSize =  3.0*sqrt(3.0);
//    FastPatchExtraction = false;
    rad   = 15;
    radq  =  3;
    thq   =  8;
    histq =  8;
    nrm_type = NRM_PARTIAL;
//    photoNorm=true;
  }
};

struct DAISYDescriptor
{
public:
  DAISYDescriptor(const DAISYParams &par)
  {
    this->par = par;
    CurrentDescriptor = new daisy();
    im = NULL;
    type = DESC_DAISY;

    temp_pt.pt.x = par.PEParam.patchSize / 2;
    temp_pt.pt.y = par.PEParam.patchSize / 2;
    temp_pt.angle = 0;
    temp_pt.size = float (par.PEParam.patchSize) / par.PEParam.mrSize;
    temp_pt.octave = 1;
    temp_pt.response = 1.5f;
    CharImage=cv::Mat::zeros(par.PEParam.patchSize,par.PEParam.patchSize,CV_8U);
  }
  void operator()(cv::Mat &patch, std::vector<float>& desc)
  {
    patch.convertTo(CharImage, CV_8U);
    im = CharImage.ptr<uchar>(0);

    CurrentDescriptor->set_image(im,par.PEParam.patchSize,par.PEParam.patchSize);
    CurrentDescriptor->verbose(0);

    //    int orientation_resolution = 18;
    //    bool rotation_inv = false;

    CurrentDescriptor->set_parameters(par.rad, par.radq, par.thq, par.histq);
    // !! this part is optional. You don't need to set the workspace memory
    int ws = CurrentDescriptor->compute_workspace_memory();
    float* workspace = new float[ ws ];

    CurrentDescriptor->set_workspace_memory( workspace, ws);
    CurrentDescriptor->initialize_single_descriptor_mode();

    if( par.nrm_type == 0 ) CurrentDescriptor->set_normalization( NRM_PARTIAL );
    if( par.nrm_type == 1 ) CurrentDescriptor->set_normalization( NRM_FULL );
    if( par.nrm_type == 2 ) CurrentDescriptor->set_normalization( NRM_SIFT );

    desc_size =  CurrentDescriptor->descriptor_size();
    desc.resize(desc_size);
    CurrentDescriptor->get_descriptor(temp_pt.pt.y,temp_pt.pt.x,temp_pt.angle,&desc[0]);
    CurrentDescriptor->release_auxilary();
    im=NULL;
    delete []workspace;
  }
public:
  descriptor_type type;
  int desc_size;
  daisy* CurrentDescriptor;

private:
  DAISYParams par;
  cv::KeyPoint temp_pt;
  cv::Mat CharImage;
  uchar* im;

};

#endif // DAISYDESCRIPTOR_HPP
