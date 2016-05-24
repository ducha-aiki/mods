#ifndef SSIMDESCRIPTOR_HPP
#define SSIMDESCRIPTOR_HPP

#include <descriptors_parameters.hpp>
#include "detectors/structures.hpp"
#include "../ssdesc-cpp-1.1.1/ssdesc.h"

struct SSIMParams
{
  PatchExtractionParams PEParam;

  //  int patchSize;
  //  double mrSize;
  //  bool FastPatchExtraction;
  int window_size;
  int cor_size;
  int nrad;
  int nang;
  float var_noise;
  float saliency_thresh;
  float homogeneity_thresh;
  float snn_thresh;
  int desc_rad;
  bool doBaumberg;
  SSIMParams()
  {
    //    patchSize = 41;
    //    mrSize =  3.0*sqrt(3.0);
    //    FastPatchExtraction = false;
    window_size = 5;
    desc_rad = 40;
    nrad = 3;
    nang=12;
    cor_size = 20;
    var_noise=300000;
    doBaumberg = false;
    saliency_thresh  =   0.7;/*      Used for salient descriptor detection. If all bins in the
                                  non-normalised descriptor have a ssd value of this threshold
                                  or more when compared to the central patch, then the
                                  descriptor is marked as salient. A value of 1.0 disables
                                  salient descriptor detection. */
    homogeneity_thresh = 0.7; /*      Used for homogeneous descriptor detection.
                                  If all bins in the non-normalised descriptor have a similarity
                                  (1-ssd) of this threshold or more when compared to the central
                                  patch, then the descriptor is marked as homogeneous. A value
                                  of 1.0 disables homogeneous descriptor detection. */
    snn_thresh    =      0.85; /*     Used for elimination of descriptors based upon a second-nearest
                                  neighbour constraint. For each descriptor, takes the
                                  two most similar matching descriptors from across the image,
                                  then eliminates those descriptors whose euclidean distance
                                  ratio to these two descriptors d1/d2 > snn_thresh (where d2 >
                                  d1). A value of 1.0 disables the second-nearest neighbour test.*/

  }
};

struct SSIMDescriptor
{
public:
  SSIMDescriptor(const SSIMParams &par)
  {
    this->par = par;
    sspar.cor_size = par.cor_size;
    sspar.homogeneity_thresh = par.homogeneity_thresh;
    sspar.nang = par.nang;
    sspar.nrad = par.nrad;
    sspar.window_size = par.window_size;
    sspar.saliency_thresh = par.saliency_thresh;
    sspar.snn_thresh = par.snn_thresh;
    sspar.var_noise = par.var_noise;
    type = DESC_SSIM;
    temp_pt.pt.x = par.PEParam.patchSize / 2;
    temp_pt.pt.y = par.PEParam.patchSize / 2;
    temp_pt.angle = 0;
    temp_pt.size = float (par.PEParam.patchSize) / par.PEParam.mrSize;
    temp_pt.octave = 1;
    temp_pt.response = 1.5f;
    doubleImg=cv::Mat::zeros(par.PEParam.patchSize,par.PEParam.patchSize,CV_64F);
  }
  void operator()(cv::Mat &patch, std::vector<float>& desc)
  {
    patch.convertTo(doubleImg,CV_64F);
    const double* patchPtr = doubleImg.ptr<double>(0);
    std::vector<double> descs;
    //    std::cerr << "before calc_descs" << std::endl;
    //    std::cerr << patch.cols << " " << patch.rows << " " << patch.channels() << " " << sspar.cor_size << " " << sspar.nang << "  " << sspar.nrad << std::endl;
    ssdesc::calc_ssdescs_alt<double>(patchPtr, patch.cols, patch.rows, patch.channels(), sspar, &descs);
    // std::cerr << "after calc_descs" << std::endl;

    std::vector<double> resp;
    std::vector<ssdesc::coordElem> draw_coords;
    std::vector<ssdesc::coordElem> salient_coords;
    std::vector<ssdesc::coordElem> homogeneous_coords;
    std::vector<ssdesc::coordElem> snn_coords;
    //    std::cerr << "before prune_descs" << std::endl;

    ssdesc::prune_normalise_ssdescs<double>(descs, patch.cols, patch.rows, sspar,
                                            & resp,
                                            & draw_coords,
                                            & salient_coords,
                                            & homogeneous_coords,
                                            & snn_coords);
    //  std::cerr << "after prune_descs" << std::endl;

    desc_size = resp.size();
    desc.resize(desc_size);
    //  std::cerr << "desc size= " << desc_size << std::endl;
    for (unsigned int i = 0; i<desc_size; ++i){
        desc[i]=(float) resp[i];
      }
  }
public:
  descriptor_type type;
  int desc_size;

private:
  SSIMParams par;
  cv::KeyPoint temp_pt;
  ssdesc::ssdesc_parms<double> sspar;
  cv::Mat doubleImg;
};

#endif // SSIMDESCRIPTOR_HPP
