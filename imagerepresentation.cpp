/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/
#include "imagerepresentation.h"
#include "synth-detection.hpp"
#include "detectors/mser/extrema/extrema.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "opensurf/surflib.h"
#include "matching/liopdesc.hpp"
#include "akaze/src/lib/AKAZE.h"
#include "TILDE/c++/src/libTILDE.hpp"
//#include "detectors/new-saddle/sorb.h"

//#include "synthviewdet_old.hpp"
//#include "synthviewdet_old.hpp"


#ifdef _OPENMP
#include <omp.h>
#endif

//inline long getMilliSecs()
//{
//  timeval t;
//  gettimeofday(&t, NULL);
//  return t.tv_sec*1000 + t.tv_usec/1000;
//}

#define VERBOSE 1
//


void saveKP(AffineKeypoint &ak, std::ostream &s) {
  s << ak.x << " " << ak.y << " " << ak.a11 << " " << ak.a12 << " " << ak.a21 << " " << ak.a22 << " ";
  s << ak.pyramid_scale << " " << ak.octave_number << " " << ak.s << " " << ak.sub_type << " ";
}
void saveKPBench(AffineKeypoint &ak, std::ostream &s) {
  s << ak.x << " " << ak.y << " "  << ak.s << " " << ak.a11 << " " << ak.a12 << " " << ak.a21 << " " << ak.a22;
}
//  det([a11,a12;a21,a22}) = 1
void saveKPMichal(AffineKeypoint &ak, std::ostream &s) {
  //float x, y, s, a11, a12, a21, a22, int type, float response, unsigned char desc[128]
  ak.s *= sqrt(fabs(ak.a11*ak.a22 - ak.a12*ak.a21))*3.0*sqrt(3.0);

  rectifyAffineTransformationUpIsUp(ak.a11,ak.a12,ak.a21,ak.a22);

  s << ak.x << " " << ak.y << " " << ak.s << " " << ak.a11 << " " << ak.a12 << " " << ak.a21 << " " << ak.a22 << " ";
  s << ak.sub_type << " " << ak.response << " ";
}
void saveKPMichalBin(AffineKeypoint &ak, std::ostream &s) {
  //float x, y, s, a11, a12, a21, a22, int type, float response, unsigned char desc[128]
  ak.s *= sqrt(fabs(ak.a11*ak.a22 - ak.a12*ak.a21))*3.0*sqrt(3.0);
  rectifyAffineTransformationUpIsUp(ak.a11,ak.a12,ak.a21,ak.a22);

  float x = (float)ak.x;
  s.write((char *)&x, sizeof(float));

  float y = (float)ak.y;
  s.write((char *)&y, sizeof(float));
  //float scale = ak.s*mrSize;
  float scale = (float)ak.s;
  s.write((char *)&scale, sizeof(float));

  float a11 = (float)ak.a11;
  s.write((char *)&a11, sizeof(float));

  float a12 = (float)ak.a12;
  s.write((char *)&a12, sizeof(float));

  float a21 = (float)ak.a21;
  s.write((char *)&a21, sizeof(float));

  float a22 = (float)ak.a22;
  s.write((char *)&a22, sizeof(float));

  s.write((char *)&ak.sub_type, sizeof(int));

  float resp = (float)ak.response;
  s.write((char *)&resp, sizeof(float));

  //  std::cerr << x << " " << y << " " << scale  << std::endl;

  //  s << ak.x << " " << ak.y << " " << ak.s << " " << ak.a11 << " " << ak.a12 << " " << ak.a21 << " " << ak.a22 << " ";
  //  s << ak.sub_type << " " << ak.response << " ";
}

void saveAR(AffineRegion &ar, std::ostream &s) {
 // s << ar.id << " " << ar.img_id << " " <<  ar.img_reproj_id << " ";
 // s << ar.parent_id <<  " ";
 // saveKP(ar.det_kp,s);
  saveKPBench(ar.reproj_kp,s);
  // s << ar.desc.type <<
  s << " " << ar.desc.vec.size() << " ";
  for (unsigned int i = 0; i < ar.desc.vec.size(); ++i) {
      s << ar.desc.vec[i] << " ";
    }
}
void saveARBench(AffineRegion &ar, std::ostream &s, std::ostream &s2) {
  saveKPBench(ar.det_kp,s2);
  saveKPBench(ar.reproj_kp,s);
}
void saveARMichal(AffineRegion &ar, std::ostream &s) {
  // s << ar.id << " " << ar.img_id << " " <<  ar.img_reproj_id << " ";
  // s << ar.parent_id <<  " ";
  // saveKP(ar.det_kp,s);
  saveKPMichal(ar.reproj_kp,s);
  // s << ar.desc.type <<
  // s << " " << ar.desc.vec.size() << " ";
  for (unsigned int i = 0; i < ar.desc.vec.size(); ++i) {
      s << ar.desc.vec[i] << " ";
    }
}
void saveARMichalBinary(AffineRegion &ar, std::ostream &s) {
  // s << ar.id << " " << ar.img_id << " " <<  ar.img_reproj_id << " ";
  // s << ar.parent_id <<  " ";
  // saveKP(ar.det_kp,s);
  saveKPMichalBin(ar.reproj_kp,s);
  // s << ar.desc.type <<
  // s << " " << ar.desc.vec.size() << " ";
  // std::cerr << ar.desc.vec.size() << std::endl;
  for (unsigned int i = 0; i < ar.desc.vec.size(); ++i) {
      int desc = (int)MAX(0,MIN(ar.desc.vec[i], 255));
      unsigned char desc1 = (unsigned char) (desc);
      s.write((char *)&desc1, sizeof(unsigned char));
      //    s << ar.desc.vec[i] << " ";
    }
}
void loadKP(AffineKeypoint &ak, std::istream &s) {
  s >> ak.x >> ak.y >> ak.a11 >> ak.a12 >>ak.a21 >> ak.a22 >> ak.pyramid_scale >> ak.octave_number >> ak.s >> ak.sub_type;
}

void loadAR(AffineRegion &ar, std::istream &s) {
  s >> ar.id >> ar.img_id >> ar.img_reproj_id;
  s >> ar.parent_id;
  loadKP(ar.det_kp,s);
  loadKP(ar.reproj_kp,s);
  //  s >> ar.desc.type;
  int size1;
  s >> size1;
  ar.desc.vec.resize(size1);
  for (unsigned int i = 0; i < ar.desc.vec.size(); ++i) {
      s >> ar.desc.vec[i];
    }
}

#ifdef WITH_CAFFE
using namespace caffe;  // NOLINT(build/namespaces)
bool cvMatToDatum(const cv::Mat & cv_img, const int label, Datum* datum) {
  // accept only char type matrices
  CV_Assert(cv_img.depth() != sizeof(uchar));
  if (!cv_img.data) {
      return false;
    }

  const unsigned int num_channels = cv_img.channels();
  const unsigned int height = cv_img.rows;
  const unsigned int width = cv_img.cols;

  datum->set_channels(num_channels);
  datum->set_height(height);
  datum->set_width(width);
  datum->set_label(label);
  datum->clear_data();
  datum->clear_float_data();
  string* datum_string = datum->mutable_data();

  for (unsigned int c=0; c < num_channels; ++c) {
      for (unsigned int h = 0; h < height; ++h){
          const cv::Vec3b *cv_img_data = cv_img.ptr<cv::Vec3b>(h);
          for (unsigned int w = 0; w < width; ++w){
              datum_string->push_back(static_cast<char>(cv_img_data[w][c]));
            }
        }
    }

  return true;
}
#endif
void L2normalize(const float* input_arr, int size, std::vector<float> &output_vect)
{
  double norm = 0.0;
  for (int i = 0; i < size; ++i) {
      norm+=input_arr[i] * input_arr[i];
    }
  const double norm_coef = 1.0/sqrt(norm);
  for (int i = 0; i < size; ++i) {
      const float v1 = floor(512.0*norm_coef*input_arr[i]);
      output_vect[i] = v1;
    }
}
void L1normalize(const float* input_arr, int size, std::vector<float> &output_vect)
{
  double norm=0.0;
  for (int i = 0; i < size; ++i) {
      norm+=input_arr[i];
    }
  const double norm_coef = 1.0/norm;
  for (int i = 0; i < size; ++i) {
      const float v1 = floor(512.0*norm_coef*input_arr[i]);
      output_vect[i] = v1;
    }
}
void RootNormalize(const float* input_arr, int size, std::vector<float> &output_vect)
{
  L2normalize(input_arr,size,output_vect);
  double norm=0.0;
  for (int i = 0; i < size; ++i) {
      norm+=input_arr[i];
    }
  const double norm_coef = 1.0/norm;
  for (int i = 0; i < size; ++i) {
      const float v1 = sqrt(512.0*norm_coef*input_arr[i]);
      output_vect[i] = v1;
    }
}

ImageRepresentation::ImageRepresentation(cv::Mat _in_img, std::string _name)
{
  if (_in_img.channels() ==3) {
      _in_img.convertTo(OriginalImg,CV_32FC3);

    } else {
      _in_img.convertTo(OriginalImg,CV_32F);
    }
  Name = _name;
}
ImageRepresentation::ImageRepresentation()
{

}
#ifdef WITH_CAFFE
void ImageRepresentation::InitCaffe(caffe::Net<float>* net_ptr)
{
  caffe_net_ptr = net_ptr;
}
#endif
ImageRepresentation::~ImageRepresentation()
{
  RegionVectorMap.clear();
}
descriptor_type ImageRepresentation::GetDescriptorType(std::string desc_name)
{
  for (unsigned int i=0; i< DescriptorNames.size(); i++)
    if (DescriptorNames[i].compare(desc_name)==0)
      return static_cast<descriptor_type>(i);
  return DESC_UNKNOWN;
}

detector_type ImageRepresentation::GetDetectorType(std::string det_name)
{
  for (unsigned int i=0; i< DetectorNames.size(); i++)
    if (DetectorNames[i].compare(det_name)==0)
      return static_cast<detector_type>(i);
  return DET_UNKNOWN;
}

TimeLog ImageRepresentation::GetTimeSpent()
{
  return TimeSpent;
}

int ImageRepresentation::GetRegionsNumber(std::string det_name)
{
  int reg_number = 0;
  std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
  if (det_name.compare("All") == 0)
    {
      for (regions_it = RegionVectorMap.begin();
           regions_it != RegionVectorMap.end(); regions_it++)
        {
          AffineRegionVectorMap::iterator desc_it;
          if ( (desc_it = regions_it->second.find("None")) != regions_it->second.end() )
            reg_number +=  desc_it->second.size();
        }
    }
  else
    {
      regions_it = RegionVectorMap.find(det_name);
      if ( regions_it != RegionVectorMap.end())
        {
          AffineRegionVectorMap::iterator desc_it;
          if ( (desc_it = regions_it->second.find("None")) != regions_it->second.end() )
            reg_number +=  desc_it->second.size();
        }
    }
  return reg_number;
}
int ImageRepresentation::GetDescriptorsNumber(std::string desc_name, std::string det_name)
{
  int reg_number = 0;
  std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
  AffineRegionVectorMap::iterator desc_it;

  if (det_name.compare("All") == 0)
    {
      for (regions_it = RegionVectorMap.begin();
           regions_it != RegionVectorMap.end(); regions_it++)
        if (desc_name.compare("All") == 0)
          {
            for (desc_it = regions_it->second.begin();
                 desc_it != regions_it->second.end(); desc_it++)
              reg_number +=  desc_it->second.size();
          }
        else
          {
            desc_it = regions_it->second.find(desc_name);
            if (desc_it != regions_it->second.end() )
              reg_number +=  desc_it->second.size();

          }
    }
  else
    {
      regions_it = RegionVectorMap.find(det_name);
      if ( regions_it != RegionVectorMap.end())
        {
          if (desc_name.compare("All") == 0)
            {
              for (desc_it = regions_it->second.begin();
                   desc_it != regions_it->second.end(); desc_it++)
                reg_number +=  desc_it->second.size();
            }
          else
            {
              desc_it = regions_it->second.find(desc_name);
              if (desc_it != regions_it->second.end() )
                reg_number +=  desc_it->second.size();

            }
        }
    }
  return reg_number;
}
int ImageRepresentation::GetDescriptorDimension(std::string desc_name)
{
  int dim = 0;
  std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
  AffineRegionVectorMap::iterator desc_it;

  for (regions_it = RegionVectorMap.begin();regions_it != RegionVectorMap.end(); regions_it++)
    {
      desc_it = regions_it->second.find(desc_name);
      if (desc_it != regions_it->second.end() )
        if (desc_it->second.size() > 0)
          {
            dim = desc_it->second[0].desc.vec.size();
            break;
          }
    }
  return dim;
}
cv::Mat ImageRepresentation::GetDescriptorsMatByDetDesc(const std::string desc_name,const std::string det_name)
{
  unsigned int dim = GetDescriptorDimension(desc_name);
  unsigned int n_descs = GetDescriptorsNumber(desc_name,det_name);

  cv::Mat descriptors(dim, n_descs, CV_32F);
  int reg_number = 0;

  std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
  AffineRegionVectorMap::iterator desc_it;

  if (det_name.compare("All") == 0)
    {
      for (regions_it = RegionVectorMap.begin();
           regions_it != RegionVectorMap.end(); regions_it++)
        {
          desc_it = regions_it->second.find(desc_name);
          if (desc_it != regions_it->second.end() )
            {
              AffineRegionVector *currentDescVector = &(desc_it->second);
              unsigned int curr_size = currentDescVector->size();
              for (unsigned int i = 0; i<curr_size; i++, reg_number++)
                {
                  float* Row = descriptors.ptr<float>(reg_number);
                  AffineRegion curr_region = (*currentDescVector)[i];
                  for (unsigned int j = 0; j<dim; j++)
                    Row[j] = curr_region.desc.vec[j];
                }
            }
        }
    }
  else
    {
      regions_it = RegionVectorMap.find(det_name);
      if ( regions_it != RegionVectorMap.end())
        {
          desc_it = regions_it->second.find(desc_name);
          if (desc_it != regions_it->second.end() )
            {
              AffineRegionVector *currentDescVector = &(desc_it->second);
              unsigned int curr_size = currentDescVector->size();
              for (unsigned int i = 0; i<curr_size; i++, reg_number++)
                {
                  float* Row = descriptors.ptr<float>(reg_number);
                  AffineRegion curr_region = (*currentDescVector)[i];
                  for (unsigned int j = 0; j<dim; j++)
                    Row[j] = curr_region.desc.vec[j];
                }
            }
        }
    }
  return descriptors;
}

cv::Mat ImageRepresentation::GetDescriptorsMatByDetDesc(std::vector<Point2f> &coordinates, const std::string desc_name,const std::string det_name)
{
  unsigned int dim = GetDescriptorDimension(desc_name);
  unsigned int n_descs = GetDescriptorsNumber(desc_name,det_name);

  cv::Mat descriptors(dim, n_descs, CV_32F);
  coordinates.clear();
  coordinates.reserve(n_descs);
  int reg_number = 0;

  std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
  AffineRegionVectorMap::iterator desc_it;

  if (det_name.compare("All") == 0)
    {
      for (regions_it = RegionVectorMap.begin();
           regions_it != RegionVectorMap.end(); regions_it++)
        {
          desc_it = regions_it->second.find(desc_name);
          if (desc_it != regions_it->second.end() )
            {
              AffineRegionVector *currentDescVector = &(desc_it->second);
              unsigned int curr_size = currentDescVector->size();
              for (unsigned int i = 0; i<curr_size; i++, reg_number++)
                {
                  float* Row = descriptors.ptr<float>(reg_number);
                  AffineRegion curr_region = (*currentDescVector)[i];
                  Point2f curr_point;
                  curr_point.x = curr_region.reproj_kp.x;
                  curr_point.y = curr_region.reproj_kp.y;
                  coordinates.push_back(curr_point);
                  for (unsigned int j = 0; j<dim; j++)
                    Row[j] = curr_region.desc.vec[j];
                }
            }
        }
    }
  else
    {
      regions_it = RegionVectorMap.find(det_name);
      if ( regions_it != RegionVectorMap.end())
        {
          desc_it = regions_it->second.find(desc_name);
          if (desc_it != regions_it->second.end() )
            {
              AffineRegionVector *currentDescVector = &(desc_it->second);
              unsigned int curr_size = currentDescVector->size();
              for (unsigned int i = 0; i<curr_size; i++, reg_number++)
                {
                  float* Row = descriptors.ptr<float>(reg_number);
                  AffineRegion curr_region = (*currentDescVector)[i];
                  Point2f curr_point;
                  curr_point.x = curr_region.reproj_kp.x;
                  curr_point.y = curr_region.reproj_kp.y;
                  coordinates.push_back(curr_point);

                  for (unsigned int j = 0; j<dim; j++)
                    Row[j] = curr_region.desc.vec[j];
                }
            }
        }
    }
  return descriptors;
}

AffineRegion ImageRepresentation::GetAffineRegion(std::string desc_name, std::string det_name, int idx)
{
  AffineRegion curr_region;
  std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
  AffineRegionVectorMap::iterator desc_it;

  regions_it = RegionVectorMap.find(det_name);
  if ( regions_it != RegionVectorMap.end())
    {
      desc_it = regions_it->second.find(desc_name);
      if (desc_it != regions_it->second.end() )
        {
          AffineRegionVector *currentDescVector = &(desc_it->second);
          curr_region = (*currentDescVector)[idx];
          return curr_region;
        }
    }
  return curr_region;
}
AffineRegionVector ImageRepresentation::GetAffineRegionVector(std::string desc_name, std::string det_name, std::vector<int> idxs)
{
  unsigned int n_regs = idxs.size();
  AffineRegionVector regions;
  regions.reserve(n_regs);

  std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
  AffineRegionVectorMap::iterator desc_it;


  regions_it = RegionVectorMap.find(det_name);
  if ( regions_it != RegionVectorMap.end())
    {
      desc_it = regions_it->second.find(desc_name);
      if (desc_it != regions_it->second.end() )
        {
          AffineRegionVector *currentDescVector = &(desc_it->second);
          for (unsigned int i = 0; i < n_regs; i++)
            regions.push_back((*currentDescVector)[idxs[i]]);
        }
    }

  return regions;
}
AffineRegionVector ImageRepresentation::GetAffineRegionVector(std::string desc_name, std::string det_name)
{
  unsigned int n_regs = GetDescriptorsNumber(desc_name,det_name);
  AffineRegionVector regions;
  regions.reserve(n_regs);

  std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
  AffineRegionVectorMap::iterator desc_it;
  if (det_name.compare("All") == 0)  {
      for (regions_it = RegionVectorMap.begin();
           regions_it != RegionVectorMap.end(); regions_it++)
        {
          desc_it = regions_it->second.find(desc_name);
          if (desc_it != regions_it->second.end() )
            {
              AffineRegionVector *currentDescVector = &(desc_it->second);
              for (unsigned int i = 0; i < n_regs; i++)
                regions.push_back((*currentDescVector)[i]);
            }
        }
    }
  else {
      regions_it = RegionVectorMap.find(det_name);
      if ( regions_it != RegionVectorMap.end())
        {
          desc_it = regions_it->second.find(desc_name);
          if (desc_it != regions_it->second.end() )
            {
              AffineRegionVector *currentDescVector = &(desc_it->second);
              for (unsigned int i = 0; i < n_regs; i++)
                regions.push_back((*currentDescVector)[i]);
            }
        }
    }
  return regions;
}

void ImageRepresentation::AddRegions(AffineRegionVector &RegionsToAdd, std::string det_name, std::string desc_name)
{
  std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
  AffineRegionVectorMap::iterator desc_it;

  regions_it = RegionVectorMap.find(det_name);
  if ( regions_it != RegionVectorMap.end())
    {
      desc_it = regions_it->second.find(desc_name);
      if (desc_it != regions_it->second.end() )
        {
          AffineRegionVector *currentDescVector = &(desc_it->second);
          ImageRepresentation::AddRegionsToList(*currentDescVector,RegionsToAdd);
        }
      else
        {
          regions_it->second[desc_name] = RegionsToAdd;
        }
    }
  else
    {
      std::map<std::string, AffineRegionVector> new_desc;
      new_desc[desc_name] = RegionsToAdd;
      RegionVectorMap[det_name] = new_desc;
    }
}
void ImageRepresentation::AddRegions(AffineRegionVectorMap &RegionsMapToAdd, std::string det_name)
{
  AffineRegionVectorMap::iterator desc_it;

  for (desc_it = RegionsMapToAdd.begin();
       desc_it != RegionsMapToAdd.end(); desc_it++)
    AddRegions(desc_it->second,det_name,desc_it->first);
}

void ImageRepresentation::AddRegionsToList(AffineRegionList &kp_list, AffineRegionList &new_kps)
{
  int size = (int)kp_list.size();
  unsigned int new_size = size + new_kps.size();
  AffineRegionList::iterator ptr = new_kps.begin();
  for (unsigned int i=size; i< new_size; i++, ptr++)
    {
      AffineRegion temp_reg = *ptr;
      temp_reg.id += size;
      temp_reg.parent_id +=size;
      kp_list.push_back(temp_reg);
    }
}


void ImageRepresentation::SynthDetectDescribeKeypoints (IterationViewsynthesisParam &synth_par,
                                                        DetectorsParameters &det_par,
                                                        DescriptorsParameters &desc_par,
                                                        DominantOrientationParams &dom_ori_par)
{
  double time1 = 0;
#ifdef _OPENMP
  omp_set_nested(1);
#endif
#pragma omp parallel for schedule (dynamic,1)
  for (unsigned int det=0; det < DetectorNames.size(); det++)
    {
      std::string curr_det = DetectorNames[det];
      unsigned int n_synths = synth_par[curr_det].size();

      std::vector<AffineRegionVectorMap> OneDetectorKeypointsMapVector;
      OneDetectorKeypointsMapVector.resize(n_synths);

#pragma omp parallel for schedule (dynamic,1)
      for (unsigned int synth=0; synth<n_synths; synth++)
        {
          ///Synthesis
          long s_time = getMilliSecs1();
          AffineRegionVector temp_kp1;
          AffineRegionVectorMap temp_kp_map;
          SynthImage temp_img1;
          if ((curr_det != "TILDE") && (curr_det != "TILDE-plugin")) {
              GenerateSynthImageCorr(OriginalImg, temp_img1, Name.c_str(),
                                     synth_par[curr_det][synth].tilt,
                                     synth_par[curr_det][synth].phi,
                                     synth_par[curr_det][synth].zoom,
                                     synth_par[curr_det][synth].InitSigma,
                                     synth_par[curr_det][synth].doBlur, synth);
            } else {
              cv::Mat rgbimg;

              if (OriginalImg.channels() == 3)
                {
                  const bool convert_to_gray = false;
                  GenerateSynthImageCorr(OriginalImg, temp_img1, Name.c_str(),
                                         synth_par[curr_det][synth].tilt,
                                         synth_par[curr_det][synth].phi,
                                         synth_par[curr_det][synth].zoom,
                                         synth_par[curr_det][synth].InitSigma,
                                         synth_par[curr_det][synth].doBlur, synth,convert_to_gray);
                  temp_img1.rgb_pixels = temp_img1.pixels.clone();
                  std::vector<cv::Mat> RGB_planes(3);
                  cv::Mat in_32f;
                  temp_img1.rgb_pixels.convertTo(in_32f,CV_32FC3);
                  cv::split(in_32f, RGB_planes);
                  temp_img1.pixels = (RGB_planes[0] + RGB_planes[1] + RGB_planes[2]) / 3.0 ;

                } else
                {
                  std::cerr << "Grayscale input to TILDE!" << std::endl;
                  GenerateSynthImageCorr(OriginalImg, temp_img1, Name.c_str(),
                                         synth_par[curr_det][synth].tilt,
                                         synth_par[curr_det][synth].phi,
                                         synth_par[curr_det][synth].zoom,
                                         synth_par[curr_det][synth].InitSigma,
                                         synth_par[curr_det][synth].doBlur, synth);

                  cv::cvtColor(temp_img1.pixels, rgbimg, CV_GRAY2BGR);
                  temp_img1.rgb_pixels = rgbimg;
                }
            }
          bool doExternalAffineAdaptation = false;

          time1 = ((double)(getMilliSecs1() - s_time))/1000;
          TimeSpent.SynthTime += time1;

          ///Structures initialization
          IplImage *int_img; //for SURF
          IpVec ipts1;//for SURF
          cv::Mat CharImage; //for OpenCV detectors
          aka::AKAZEOptions options; //For KAZE
          options.img_width = temp_img1.pixels.cols;
          options.img_height = temp_img1.pixels.rows;
          aka::AKAZE evolution1(options);

          std::vector<cv::KeyPoint> keypoints_1; //for binary-dets
          cv::Mat descriptors_1; //for binary-dets

          bool OpenCV_det = ((curr_det.compare("ORB") == 0) ||
                             (curr_det.compare("Saddle") == 0) ||
                             (curr_det.compare("FAST") == 0) ||
                             (curr_det.compare("STAR") == 0) ||
                             (curr_det.compare("KAZE") == 0) ||
                             (curr_det.compare("BRISK") == 0));
          bool SIFT_like_desc = false;
          bool HalfSIFT_like_desc = false;

          for (unsigned int i_desc=0; i_desc < synth_par[curr_det][synth].descriptors.size();i_desc++) {
              std::string curr_desc = synth_par[curr_det][synth].descriptors[i_desc];
              //        if  (curr_desc.find("LIOP") != std::string::npos) {
              //          SIFT_like_desc = true;
              //        }
              //        if  (curr_desc.find("Pixels") != std::string::npos) {
              //          SIFT_like_desc = true;
              //        }
              //        if  (curr_desc.find("SIFT") != std::string::npos) {
              if (curr_desc.find("Half") != std::string::npos) {
                  HalfSIFT_like_desc = true;
                }
              if (curr_desc.find("SIFT") != std::string::npos) {
//              if ((curr_desc.find("ORB") != std::string::npos) || (curr_desc.find("FREAK") != std::string::npos)
//                  || (curr_desc.find("KAZE") != std::string::npos)) {
                  SIFT_like_desc = true;
                } else {
                  SIFT_like_desc = false;
                }
            }
          /// Detection
          s_time = getMilliSecs1();
          if (curr_det.compare("HessianAffine")==0)
            {
              DetectAffineRegions(temp_img1, temp_kp1,det_par.HessParam,DET_HESSIAN,DetectAffineKeypoints);
            }
          else if (curr_det.compare("ReadAffs") == 0) {
              std::ifstream focikp(det_par.ReadAffsFromFileParam.fname);
              if (focikp.is_open()) {
                  int kp_size;
                  focikp >> kp_size;
                  std::cerr << kp_size << std::endl;
                  temp_kp1.reserve(kp_size);
                  for (int kp_num = 0; kp_num < kp_size; kp_num++) {
                      AffineRegion temp_region;
                      temp_region.det_kp.pyramid_scale = -1;
                      temp_region.det_kp.octave_number = -1;
                      temp_region.det_kp.sub_type = 101;
                      focikp >> temp_region.det_kp.x;
                      focikp >> temp_region.det_kp.y;
                      focikp >> temp_region.det_kp.s;
                      focikp >> temp_region.det_kp.a11;
                      focikp >> temp_region.det_kp.a12;
                      focikp >> temp_region.det_kp.a21;
                      focikp >> temp_region.det_kp.a22;
                      temp_region.det_kp.response = 100;
                      temp_region.type = DET_FOCI;
                      temp_kp1.push_back(temp_region);
                    }
                }
              focikp.close();
            }
          else if (curr_det.compare("FOCI")==0)
            {
              doExternalAffineAdaptation = det_par.FOCIParam.doBaumberg;
              //  DetectAffineRegions(temp_img1, temp_kp1,det_par.DoGParam,DET_DOG,DetectAffineKeypoints);
              int rnd1 = (int) getMilliSecs() + (std::rand() % (int)(1001));
              std::string img_fname = "FOCI"+std::to_string(synth+rnd1)+".png";
              cv::imwrite(img_fname,temp_img1.pixels);
              //srand();
              std::string command = "wine EdgeFociAndBice.exe -mi -i " + img_fname;
              //   command += " -mi";
              if (det_par.FOCIParam.numberKPs > 0) {
                  command += " -n "+ std::to_string(det_par.FOCIParam.numberKPs);
                }
              if (det_par.FOCIParam.computeOrientation) {
                  command += " -co";
                  if (det_par.FOCIParam.secondOrientation) {
                      command += " -mo ";
                    }
                }
              std::string fname1 = "FOCI" + std::to_string(synth+rnd1) + ".txt";
              command += " -o " + fname1;
              std::cerr << command <<std::endl;
              system(command.c_str());
              std::ifstream focikp(fname1);
              if (focikp.is_open()) {

                  int kp_size;
                  focikp >> kp_size;

                  temp_kp1.reserve(kp_size);
                  for (int kp_num=0; kp_num < kp_size; kp_num++)
                    {
                      AffineRegion temp_region;
                      temp_region.det_kp.pyramid_scale = -1;
                      temp_region.det_kp.octave_number = -1;
                      temp_region.det_kp.sub_type = 55;
                      focikp >> temp_region.det_kp.x;
                      focikp >> temp_region.det_kp.y;
                      focikp >> temp_region.det_kp.a11;
                      temp_region.det_kp.a11 = sqrt(temp_region.det_kp.a11);

                      focikp >> temp_region.det_kp.a12;
                      temp_region.det_kp.a12 = sqrt(temp_region.det_kp.a12);
                      temp_region.det_kp.a21 = 0;
                      focikp >> temp_region.det_kp.a22;
                      temp_region.det_kp.a22 = sqrt(temp_region.det_kp.a22);
                      temp_region.det_kp.s = 1.0;  //?
                      focikp >> temp_region.det_kp.response;
                      temp_region.type = DET_FOCI;
                      float angle;
                      focikp >> angle; //Not good yet

                      temp_region.det_kp.s *= sqrt(fabs(temp_region.det_kp.a11*temp_region.det_kp.a22
                                                        - temp_region.det_kp.a12*temp_region.det_kp.a21));

                      rectifyAffineTransformationUpIsUp(temp_region.det_kp.a11,
                                                        temp_region.det_kp.a12,
                                                        temp_region.det_kp.a21,
                                                        temp_region.det_kp.a22);

                      temp_kp1.push_back(temp_region);

                    }
                }
              focikp.close();
              std::string rm_command = "rm " + fname1;
              system(rm_command.c_str());
              rm_command = "rm " + img_fname;
              system(rm_command.c_str());

            }
          else if (curr_det.compare("SFOP")==0)
            {
              doExternalAffineAdaptation = det_par.SFOPParam.doBaumberg;
              int rnd1 = (int) getMilliSecs() + (std::rand() % (int)(1001));
              std::string img_fname = "SFOP"+std::to_string(synth+rnd1)+".png";
              cv::imwrite(img_fname,temp_img1.pixels);
              std::string command = "./sfop -i " + img_fname;
              command += " --display false";
              command += " --noise "+ std::to_string(det_par.SFOPParam.noise);
              command += " --pTresh "+ std::to_string(det_par.SFOPParam.pThresh);
              command += " --lWeight "+ std::to_string(det_par.SFOPParam.lWeight);
              command += " --numOctaves  "+ std::to_string(det_par.SFOPParam.nLayers);
              command += " --numLayers "+ std::to_string(det_par.SFOPParam.nOctaves);
              std::string fname1 = "SFOP" + std::to_string(synth+rnd1) + ".txt";
              command += " -o " + fname1;
              std::cerr << command <<std::endl;
              system(command.c_str());
              std::ifstream focikp(fname1);
              if (focikp.is_open()) {
                  ReadKPsMik(temp_kp1, focikp, det_par.ToSMSERParam.scale);
                }

              focikp.close();
              std::string rm_command = "rm " + fname1;
              system(rm_command.c_str());
              rm_command = "rm " + img_fname;
              system(rm_command.c_str());
            }
          else if (curr_det.compare("WAVE")==0)
            {
              doExternalAffineAdaptation = det_par.WAVEParam.doBaumberg;
              int rnd1 = (int) getMilliSecs() + (std::rand() % (int)(1001));
              std::string img_fname = "WAVE"+std::to_string(synth+rnd1)+".png";
              cv::imwrite(img_fname,temp_img1.pixels);
              std::string command = "./WaveDetector -i " + img_fname;
              command += " -b "+ std::to_string(det_par.WAVEParam.b_wave);
              command += " --non_maxima_suppression "+ std::to_string(det_par.WAVEParam.nms);
              command += " -s "+ std::to_string(det_par.WAVEParam.s);
              command += " -t "+ std::to_string(det_par.WAVEParam.t);
              command += " -r "+ std::to_string(det_par.WAVEParam.r);
              command += " -k "+ std::to_string(det_par.WAVEParam.k);
              if (det_par.WAVEParam.pyramid) {
                  command += " --pyramid";
                }
              std::string fname1 = "WAVE" + std::to_string(synth+rnd1) + ".txt";
              command += " -o " + fname1;
              std::cerr << command <<std::endl;
              system(command.c_str());
              std::ifstream focikp(fname1);
              if (focikp.is_open()) {
                  ReadKPsMik(temp_kp1, focikp);
                }
              focikp.close();
              std::string rm_command = "rm " + fname1;
              system(rm_command.c_str());
              rm_command = "rm " + img_fname;
              system(rm_command.c_str());
            }
          else if (curr_det.compare("WASH")==0)
            {
              doExternalAffineAdaptation = det_par.WASHParam.doBaumberg;
              int rnd1 = (int) getMilliSecs() + (std::rand() % (int)(1001));
              std::string img_fname = "WASH"+std::to_string(synth+rnd1)+".png";
              cv::imwrite(img_fname,temp_img1.pixels);
              std::string command = "./WaSH_linux_64 -i " + img_fname;
              command += " -t "+ std::to_string(det_par.WASHParam.threshold);
              std::string fname1 = img_fname + ".wash";
              std::cerr << command <<std::endl;
              system(command.c_str());
              std::ifstream focikp(fname1);
              if (focikp.is_open()) {
                  ReadKPsMik(temp_kp1, focikp, det_par.ToSMSERParam.scale);
                }
              focikp.close();
              std::string rm_command = "rm " + fname1;
              system(rm_command.c_str());
              rm_command = "rm " + img_fname;
              system(rm_command.c_str());
            }
       /*   else if (curr_det.compare("Saddle")==0)
            {

              doExternalAffineAdaptation = det_par.SaddleParam.doBaumberg;

              cmp::SORB CurrentDetector(det_par.SaddleParam.respThreshold,
                                        det_par.SaddleParam.scalefac,
                                        det_par.SaddleParam.pyrLevels,
                                        det_par.SaddleParam.edgeThreshold,
                                        det_par.SaddleParam.epsilon,
                                        det_par.SaddleParam.WTA_K,
                                        det_par.SaddleParam.scoreType,
                                        det_par.SaddleParam.descSize,
                                        det_par.SaddleParam.doNMS,
                                        cmp::SORB::K_BYTES,
                                        (uchar) det_par.SaddleParam.deltaThr,
                                        det_par.SaddleParam.nfeatures,
                                        det_par.SaddleParam.allC1feats,
                                        det_par.SaddleParam.strictMaximum,
                                        det_par.SaddleParam.subPixPrecision,
                                        det_par.SaddleParam.gravityCenter,
                                        det_par.SaddleParam.innerTstType);


              //cmp::SORB detector(responseThr, scaleFactor, nlevels, edgeThreshold, epsilon, 2, cmp::SORB::DELTA_SCORE , 31,
              //                   doNMS, descSize, deltaThr, nfeatures, allC1feats, strictMaximum, subPixPrecision, gravityCenter, innerTstType);

              Mat dcts, mask;

              printf("Detecting SADDLE points... \n");
              temp_img1.pixels.convertTo(CharImage,CV_8U);
              CurrentDetector(CharImage, mask, keypoints_Sad);//, dcts, false );
              printf("Done \n");

              int kp_size = keypoints_Sad.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_Sad[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_Sad[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_Sad[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_Sad[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_Sad[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_Sad[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = sqrt(keypoints_Sad[kp_num].size);// /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_Sad[kp_num].response;
                  temp_kp1[kp_num].type = DET_SADDLE;
                }

              //              //./saddlepts1 -i image-00547.bmp -o pts_saddle.txt
              //              doExternalAffineAdaptation = det_par.SaddleParam.doBaumberg;
              //              int rnd1 = (int) getMilliSecs() + (std::rand() % (int)(1001));

              //              std::string img_fname = "Saddle"+std::to_string(synth+rnd1)+".png";
              //              cv::imwrite(img_fname,temp_img1.pixels);
              //              std::string command = "./saddlepts_very_new -i " + img_fname;
              //              std::string fname1 = img_fname + ".saddle";
              //              command += " -o "+ fname1;
              //              command += " -t "+ std::to_string(det_par.SaddleParam.threshold);
              //              command += " -l "+ std::to_string(det_par.SaddleParam.pyrLevels);
              //              command += " -s "+ std::to_string(det_par.SaddleParam.scalefac);
              //              command += " -e "+ std::to_string(det_par.SaddleParam.epsilon);
              //              if (det_par.SaddleParam.doNMS) {
              //                  command += " -n ";
              //                };
              //              std::cerr << command << std::endl;
              //              system(command.c_str());
              //              std::ifstream focikp(fname1);
              //              if (focikp.is_open()) {
              //                  ReadKPsMik(temp_kp1, focikp,DET_SADDLE, 5.192);
              //                }
              //              focikp.close();
              //              std::cout << temp_kp1.size() << " saddle points detected" << std::endl;

              //              std::string rm_command = "rm " + fname1;
              //              system(rm_command.c_str());
              //              rm_command = "rm " + img_fname;
              //              system(rm_command.c_str());
            } */
          else if (curr_det.compare("TOS-MSER")==0)
            {
              //./saddlepts1 image-00547.bmp pts_saddle.txt 0/1
              //     doExternalAffineAdaptation = det_par.SaddleParam.doBaumberg;
              int rnd1 = (int) getMilliSecs() + (std::rand() % (int)(1001));
              std::string img_fname = "tos-mser"+std::to_string(synth+rnd1)+".png";
              cv::imwrite(img_fname,temp_img1.pixels);
              std::string command = "./Trees_no_img " + img_fname;
              std::string fname1 = img_fname + ".tosmser";
              command += " "+ fname1;
              command += " "+ std::to_string(det_par.ToSMSERParam.run_mode);

              std::cerr << command << std::endl;
              system(command.c_str());
              std::ifstream focikp(fname1);
              if (focikp.is_open()) {
                  ReadKPsMik(temp_kp1, focikp,DET_TOS_MSER, det_par.ToSMSERParam.scale);
                }
              focikp.close();
              std::cout << temp_kp1.size() << " ToS-MSER points detected" << std::endl;
              //              for (int kp_num=0; kp_num < temp_kp1.size(); kp_num++) {
              //                  temp_kp1[kp_num].det_kp.s = sqrt(temp_kp1[kp_num].det_kp.s);// / (sqrt(3.0)*3.0);
              //                }
              std::string rm_command = "rm " + fname1;
              system(rm_command.c_str());
              rm_command = "rm " + img_fname;
              system(rm_command.c_str());
            }
          else if (curr_det.compare("MIK-MSER")==0)
            {
              int rnd1 = (int) getMilliSecs() + (std::rand() % (int)(1001));
              std::string img_fname = "orig-mser"+std::to_string(synth+rnd1)+".png";
              cv::imwrite(img_fname,temp_img1.pixels);
              std::string command = "./mser.ln -i " + img_fname;
              std::string fname1 = img_fname + ".mikmser";
              command += " -o "+ fname1;
              command += " -t 2";
              std::cerr << command << std::endl;
              system(command.c_str());
              std::ifstream focikp(fname1);
              if (focikp.is_open()) {
                  ReadKPsMik(temp_kp1, focikp,DET_MIK_MSER,det_par.ToSMSERParam.scale);
                }
              focikp.close();
              std::cout << temp_kp1.size() << " MSER points detected" << std::endl;
              std::string rm_command = "rm " + fname1;
              system(rm_command.c_str());
              rm_command = "rm " + img_fname;
              system(rm_command.c_str());
            }
          else if (curr_det.compare("DoG")==0)
            {
              DetectAffineRegions(temp_img1, temp_kp1,det_par.DoGParam,DET_DOG,DetectAffineKeypoints);
            }
          else if (curr_det.compare("HarrisAffine")==0)
            {
              DetectAffineRegions(temp_img1, temp_kp1,det_par.HarrParam,DET_HARRIS,DetectAffineKeypoints);
            }
          else if (curr_det.compare("MSER")==0)
            {
              DetectAffineRegions(temp_img1, temp_kp1,det_par.MSERParam,DET_MSER,DetectMSERs);
            }
          else if (curr_det.compare("TILDE")==0)
            {
              cv::Mat gray_temp = temp_img1.pixels;
              temp_img1.pixels = temp_img1.rgb_pixels;
              DetectAffineRegions(temp_img1, temp_kp1, det_par.TILDEScaleSpaceParam, DET_TILDE, DetectAffineKeypoints);
              temp_img1.pixels = gray_temp;
            }
          else if (curr_det.compare("SURF")==0)
            {
              doExternalAffineAdaptation = det_par.SURFParam.doBaumberg;
              IplImage Iplimg1 = temp_img1.pixels;
              // Create integral-image representation of the image
              int_img = Integral(&Iplimg1);

              int octaves = det_par.SURFParam.octaves;
              int intervals = det_par.SURFParam.intervals;
              int init_sample = det_par.SURFParam.init_sample;
              float thres = det_par.SURFParam.thresh;
              // Create Fast Hessian Object
              FastHessian fh(int_img, ipts1, octaves, intervals, init_sample, thres);

              // Extract interest points and store in vector ipts
              fh.getIpoints();

              int kp_size = ipts1.size();
              temp_kp1.resize(kp_size);
              for (int kp_num=0; kp_num < kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x =ipts1[kp_num].x;
                  temp_kp1[kp_num].det_kp.y = ipts1[kp_num].y;
                  temp_kp1[kp_num].det_kp.a11 = cos(ipts1[kp_num].orientation);
                  temp_kp1[kp_num].det_kp.a12 = sin(ipts1[kp_num].orientation);
                  temp_kp1[kp_num].det_kp.a21 = -sin(ipts1[kp_num].orientation);
                  temp_kp1[kp_num].det_kp.a22 = cos(ipts1[kp_num].orientation);
                  temp_kp1[kp_num].det_kp.s = ipts1[kp_num].scale;
                  temp_kp1[kp_num].type = DET_SURF;
                }
            }
          else if (curr_det.compare("ORB")==0)
            {
              OpenCV_det = true;
              doExternalAffineAdaptation = det_par.ORBParam.doBaumberg;
              //cv::OrbFeatureDetector CurrentDetector(det_par.ORBParam.nfeatures,
              cv::ORB CurrentDetector(det_par.ORBParam.nfeatures,
                                                     det_par.ORBParam.scaleFactor,
                                                     det_par.ORBParam.nlevels,
                                                     det_par.ORBParam.edgeThreshold,
                                                     det_par.ORBParam.firstLevel,
                                                     det_par.ORBParam.WTA_K,
                                                     ORB::HARRIS_SCORE,
                                                     det_par.ORBParam.PEParam.patchSize);//,
                                               //      det_par.ORBParam.doNMS);
              temp_img1.pixels.convertTo(CharImage,CV_8U);
              CurrentDetector.detect(CharImage, keypoints_1);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size  /  det_par.ORBParam.PEParam.mrSize;
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_ORB;
                }
            }
          else if (curr_det.compare("TILDE-plugin")==0)
            {

              doExternalAffineAdaptation = det_par.TILDEScaleSpaceParam.AffineShapePars.doBaumberg;
              keypoints_1 = getTILDEKeyPoints(temp_img1.rgb_pixels,
                                              det_par.TILDEScaleSpaceParam.TILDEParam.pathFilter, det_par.TILDEScaleSpaceParam.TILDEParam.approx,true,false);

              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num < min(kp_size,det_par.TILDEScaleSpaceParam.TILDEParam.maxPoints); kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_TILDE;
                }
            }
          else if (curr_det.compare("KAZE")==0)
            {
              doExternalAffineAdaptation = det_par.FOCIParam.doBaumberg;
              evolution1.Create_Nonlinear_Scale_Space(temp_img1.pixels *1.0/255.0);
              evolution1.Feature_Detection(keypoints_1);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_KAZE;
                }
            }
          else if (curr_det.compare("FAST")==0)
            {
              doExternalAffineAdaptation = det_par.FASTParam.doBaumberg;
              temp_img1.pixels.convertTo(CharImage,CV_8U);
              cv::FASTX(CharImage,keypoints_1,det_par.FASTParam.threshold,
                        det_par.FASTParam.nonmaxSuppression,det_par.FASTParam.type);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_FAST;
                }
            }
          else if (curr_det.compare("BRISK")==0)
            {
              doExternalAffineAdaptation = det_par.BRISKParam.doBaumberg;
              temp_img1.pixels.convertTo(CharImage,CV_8U);
              cv::BRISK CurrentDetector(det_par.BRISKParam.thresh,
                                        det_par.BRISKParam.octaves,
                                        det_par.BRISKParam.patternScale);
              CurrentDetector.detect(CharImage, keypoints_1);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_BRISK;
                }
            }
          else if (curr_det.compare("STAR")==0)
            {
              doExternalAffineAdaptation = det_par.STARParam.doBaumberg;
              temp_img1.pixels.convertTo(CharImage,CV_8U);
              cv::StarFeatureDetector CurrentDetector(det_par.STARParam.maxSize,
                                                      det_par.STARParam.responseThreshold,
                                                      det_par.STARParam.lineThresholdProjected,
                                                      det_par.STARParam.lineThresholdBinarized,
                                                      det_par.STARParam.suppressNonmaxSize);
              CurrentDetector.detect(CharImage, keypoints_1);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_STAR;
                }
            }
          //Baumberg iteration
          if (doExternalAffineAdaptation) {
              AffineRegionVector temp_kp_aff;
              AffineShapeParams afShPar = det_par.BaumbergParam;
              afShPar.affBmbrgMethod = det_par.HessParam.AffineShapePars.affBmbrgMethod;
              // std::cout << "bmbg method: " << (int)afShPar.affBmbrgMethod;
              DetectAffineShape(temp_kp1,
                                temp_kp_aff,
                                temp_img1,
                                afShPar);
              temp_kp1 = temp_kp_aff;
            }

          //
          /// Orientation estimation

          time1 = ((double)(getMilliSecs1() - s_time))/1000;
          TimeSpent.DetectTime += time1;
          s_time = getMilliSecs1();

          AffineRegionVector temp_kp1_SIFT_like_desc;
          AffineRegionVector temp_kp1_HalfSIFT_like_desc;
          AffineRegionVector temp_kp1_upright;

          if (curr_det.compare("ReadAffs") == 0){

            } else {
              ////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Change it!s
              if (SIFT_like_desc) {
                  DetectOrientation(temp_kp1, temp_kp1_SIFT_like_desc, temp_img1,
                                    dom_ori_par.PEParam.mrSize, dom_ori_par.PEParam.patchSize,
                                    false, dom_ori_par.maxAngles,
                                    dom_ori_par.threshold, false);
                }
              if (HalfSIFT_like_desc) {
                  DetectOrientation(temp_kp1, temp_kp1_HalfSIFT_like_desc, temp_img1,
                                    dom_ori_par.PEParam.mrSize, dom_ori_par.PEParam.patchSize,
                                    true, dom_ori_par.maxAngles,
                                    dom_ori_par.threshold, false);
                }
              if (dom_ori_par.addUpRight) {
                  DetectOrientation(temp_kp1, temp_kp1_upright, temp_img1,
                                    dom_ori_par.PEParam.mrSize, dom_ori_par.PEParam.patchSize,
                                    false, 0, 1.0, true);
                }
            }
          ReprojectRegionsAndRemoveTouchBoundary(temp_kp1, temp_img1.H, OriginalImg.cols, OriginalImg.rows);
          temp_kp_map["None"] = temp_kp1;

          for (unsigned int i_desc=0; i_desc < synth_par[curr_det][synth].descriptors.size();i_desc++) {
              std::string curr_desc = synth_par[curr_det][synth].descriptors[i_desc];
              AffineRegionVector temp_kp1_desc;
              AffineRegionVector dsp_desc;
              if (dom_ori_par.addUpRight) {
                  temp_kp1_desc.insert(temp_kp1_desc.end(), temp_kp1_upright.begin(), temp_kp1_upright.end());
                }
              //             ReprojectRegions(temp_kp1_desc, temp_img1.H, OriginalImg.cols, OriginalImg.rows);
              if (curr_det.compare("ReadAffs") == 0) {

                  temp_kp1_desc.insert(temp_kp1_desc.end(), temp_kp1.begin(), temp_kp1.end());
                  std::cerr << "Read detections from provided file" << std::endl;
                }  else {
                  //Add oriented and upright keypoints if any
                  if (HalfSIFT_like_desc) {
                      temp_kp1_desc.insert(temp_kp1_desc.end(), temp_kp1_HalfSIFT_like_desc.begin(),
                                           temp_kp1_HalfSIFT_like_desc.end());
                    }
                  if (SIFT_like_desc && (!HalfSIFT_like_desc)) {

                      temp_kp1_desc.insert(temp_kp1_desc.end(), temp_kp1_SIFT_like_desc.begin(),
                                           temp_kp1_SIFT_like_desc.end());

                    }
                  if (!SIFT_like_desc) {
                      temp_kp1_desc.insert(temp_kp1_desc.end(), temp_kp1.begin(),
                                           temp_kp1.end());
                    }
                  ReprojectRegions(temp_kp1_desc, temp_img1.H, OriginalImg.cols, OriginalImg.rows);
                }

              ///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
              ///Description
              time1 = ((double) (getMilliSecs1() - s_time)) / 1000;
              TimeSpent.OrientTime += time1;
              s_time = getMilliSecs1();

              if (curr_desc.compare("RootSIFT") == 0) //RootSIFT
                {
                  SIFTDescriptor RootSIFTdesc(desc_par.RootSIFTParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, RootSIFTdesc,
                                  desc_par.RootSIFTParam.PEParam.mrSize,
                                  desc_par.RootSIFTParam.PEParam.patchSize,
                                  desc_par.RootSIFTParam.PEParam.FastPatchExtraction,
                                  desc_par.RootSIFTParam.PEParam.photoNorm);
                }
              else if (curr_desc.compare("HalfRootSIFT") == 0) //HalfRootSIFT
                {
                  SIFTDescriptor HalfRootSIFTdesc(desc_par.HalfRootSIFTParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, HalfRootSIFTdesc,
                                  desc_par.HalfRootSIFTParam.PEParam.mrSize,
                                  desc_par.HalfRootSIFTParam.PEParam.patchSize,
                                  desc_par.HalfRootSIFTParam.PEParam.FastPatchExtraction,
                                  desc_par.HalfRootSIFTParam.PEParam.photoNorm);
                }
              else if (curr_desc.compare("HalfSIFT") == 0) //HalfSIFT
                {
                  ///Description
                  SIFTDescriptor HalfSIFTdesc(desc_par.HalfSIFTParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, HalfSIFTdesc,
                                  desc_par.HalfSIFTParam.PEParam.mrSize,
                                  desc_par.HalfSIFTParam.PEParam.patchSize,
                                  desc_par.HalfSIFTParam.PEParam.FastPatchExtraction,
                                  desc_par.HalfSIFTParam.PEParam.photoNorm);
                }

              else if (curr_desc.compare("CAFFE")==0)
                {
#ifdef WITH_CAFFE
                  /// Orientation
                  if (desc_par.CaffeDescParam.DoSIFTLikeOrientation)
                    {
                      DetectOrientation(temp_kp1,temp_kp1_desc,temp_img1,
                                        desc_par.CaffeDescParam.mrSize,
                                        desc_par.CaffeDescParam.patchSize,
                                        desc_par.RootSIFTParam.doHalfSIFT,
                                        desc_par.CaffeDescParam.maxOrientations,
                                        desc_par.CaffeDescParam.orientTh);

                    }
                  else
                    {
                      temp_kp1_desc = temp_kp1;
                    }
                  ReprojectRegions(temp_kp1_desc, temp_img1.H, OriginalImg.cols, OriginalImg.rows);

                  time1 = ((double)(getMilliSecs1() - s_time))/1000;
                  TimeSpent.OrientTime += time1;
                  s_time = getMilliSecs1();
                  ///Description
                  unsigned int i;
                  double mrScale = (double)desc_par.CaffeDescParam.mrSize; // half patch size in pixels of image
                  int patchImageSize = 2*int(mrScale)+1; // odd size
                  double imageToPatchScale = double(patchImageSize) / (double)desc_par.CaffeDescParam.patchSize;
                  // patch size in the image / patch size -> amount of down/up sampling

                  unsigned int n_descs = temp_kp1_desc.size();

                  /// CNN loading

                  std::vector<int> mean_px(3);
                  mean_px[0]=desc_par.CaffeDescParam.MeanB;
                  mean_px[1]=desc_par.CaffeDescParam.MeanG;
                  mean_px[2]=desc_par.CaffeDescParam.MeanR;
                  std::vector<cv::Mat> imgs_to_describe;
                  ///
                  std::vector<cv::Mat> BGR(3);
                  std::vector<cv::Mat> BGR_res(3);
                  if (OriginalImg.channels() == 3) {
                      cv::split(OriginalImg,BGR);
                      for (int cc=0;cc<3;cc++)
                        {
                          BGR[cc].convertTo(BGR[cc],CV_32F);
                        }
                    }
                  for (i = 0; i < n_descs; i++)
                    {
                      cv::Mat patch(desc_par.CaffeDescParam.patchSize,desc_par.CaffeDescParam.patchSize,CV_32FC1);
                      float curr_sc = imageToPatchScale * temp_kp1_desc[i].reproj_kp.s;
                      cv::Mat colorPatch;
                      if (OriginalImg.channels() == 3) {
                          for (int cc=0;cc<3;cc++)
                            {
                              interpolate(BGR[cc],
                                          (float)temp_kp1_desc[i].reproj_kp.x,
                                          (float)temp_kp1_desc[i].reproj_kp.y,
                                          (float)temp_kp1_desc[i].reproj_kp.a11*curr_sc,
                                          (float)temp_kp1_desc[i].reproj_kp.a12*curr_sc,
                                          (float)temp_kp1_desc[i].reproj_kp.a21*curr_sc,
                                          (float)temp_kp1_desc[i].reproj_kp.a22*curr_sc,
                                          patch);
                              BGR_res[cc] = patch.clone();
                            }
                          cv::Mat patch_temp;
                          cv::merge(BGR_res, patch_temp);
                          patch_temp.convertTo(colorPatch,CV_8UC3);

                        } else {
                          interpolate(OriginalImg,
                                      (float)temp_kp1_desc[i].reproj_kp.x,
                                      (float)temp_kp1_desc[i].reproj_kp.y,
                                      (float)temp_kp1_desc[i].reproj_kp.a11*curr_sc,
                                      (float)temp_kp1_desc[i].reproj_kp.a12*curr_sc,
                                      (float)temp_kp1_desc[i].reproj_kp.a21*curr_sc,
                                      (float)temp_kp1_desc[i].reproj_kp.a22*curr_sc,
                                      patch);

                          patch.convertTo(colorPatch,CV_8U);
                          cv::cvtColor(colorPatch.clone(), colorPatch,  CV_GRAY2BGR);
                        }

                      //     cv::imwrite(std::to_string(i)+".jpg",colorPatch);
                      imgs_to_describe.push_back(colorPatch);
                    }

                  //get the blob
                  const int dat_channels = 3;
                  const int dat_height = desc_par.CaffeDescParam.patchSize;
                  const int dat_width = desc_par.CaffeDescParam.patchSize;
                  const int batch_size = desc_par.CaffeDescParam.batchSize;
                  Blob<float>* blob = new Blob<float>(batch_size, dat_channels,  dat_height,  dat_width);

                  //get the blobproto
                  BlobProto blob_proto;
                  blob_proto.set_num(batch_size);
                  blob_proto.set_channels(dat_channels);
                  blob_proto.set_height(dat_height);
                  blob_proto.set_width(dat_width);
                  /// Blob init
                  Datum datum;
                  if (!cvMatToDatum(imgs_to_describe[0], 0,&datum)) {
                      std::cerr << "Cannot transform image to datum" << std::endl;
                    }
                  int size_in_datum = std::max<int>(datum.data().size(),
                                                    datum.float_data_size());

                  for (int img_num=0; img_num<batch_size; img_num++)
                    {
                      for (int i = 0; i < size_in_datum; ++i) {
                          blob_proto.add_data(0.);
                        }
                    }
                  /// Blob init done
                  int n_batches = ceil((double)imgs_to_describe.size() / (double)batch_size);
                  for (int b = 0; b < n_batches; b++) {
                      int start_img = b * batch_size;
                      int finish_img = min((b+1)*batch_size, (int)imgs_to_describe.size());

                      for (int img_num=start_img; img_num<finish_img; img_num++)
                        {
                          Datum datum;
                          if (!cvMatToDatum(imgs_to_describe[img_num], 0,&datum)) {
                              std::cerr << "Cannot transform image to datum" << std::endl;
                            }
                          const string& data = datum.data();
                          int offset_datum = (img_num - start_img)*size_in_datum;
                          if (data.size() != 0) {
                              for (int i = 0; i < size_in_datum; ++i) {
                                  blob_proto.set_data(i+offset_datum, blob_proto.data(i+offset_datum) + (float)((uint8_t)(data[i])));
                                }
                            }
                        }
                      blob->FromProto(blob_proto);
                      float* data_vec = blob->mutable_cpu_data();
                      for (int nn = 0; nn < batch_size; ++nn) {
                          for (int c = 0; c < dat_channels; ++c) {
                              for (int h = 0; h < dat_height; ++h) {
                                  for (int w = 0; w < dat_width; ++w) {
                                      data_vec[nn*dat_width*dat_height*dat_channels + (c*dat_height + h)*dat_width +w] -= mean_px[c];
                                    }
                                }
                            }
                        }
                      vector<Blob<float>*> bottom;
                      bottom.push_back(blob);
                      //fill the vector
                      float type = 0.0;
#pragma omp critical
                      {
                        const vector<Blob<float>*>& result  = caffe_net_ptr->Forward(bottom, &type);
                        if (caffe_net_ptr->has_blob(desc_par.CaffeDescParam.LayerName))
                          {
                            const boost::shared_ptr<Blob<float> > feature_blob = caffe_net_ptr->blob_by_name(desc_par.CaffeDescParam.LayerName);
                            const float* feature_blob_data = feature_blob->cpu_data();
                            const int desc_size = feature_blob->width()* feature_blob->height()*feature_blob->channels();
                            for (int img_num=start_img; img_num<finish_img; img_num++)
                              {
                                temp_kp1_desc[img_num].desc.vec.resize(desc_size);
                                int offset = (img_num - start_img)*desc_size;
                                if (desc_par.CaffeDescParam.Normalization.compare("L2")==0)
                                  {
                                    L2normalize(feature_blob_data+offset,desc_size,temp_kp1_desc[img_num].desc.vec);
                                  }
                                else if (desc_par.CaffeDescParam.Normalization.compare("L1")==0)
                                  {
                                    L1normalize(feature_blob_data+offset,desc_size,temp_kp1_desc[img_num].desc.vec);
                                  }
                                else if (desc_par.CaffeDescParam.Normalization.compare("RootL2")==0)
                                  {
                                    RootNormalize(feature_blob_data+offset,desc_size,temp_kp1_desc[img_num].desc.vec);
                                  }
                                else if (desc_par.CaffeDescParam.Normalization.compare("none")==0)
                                  {
                                    for (int i = 0; i < desc_size; ++i) {
                                        const float v1 = feature_blob_data[i+offset];
                                        temp_kp1_desc[img_num].desc.vec[i] = v1;
                                      }
                                  }
                                temp_kp1_desc[img_num].desc.type=DESC_CAFFE;
                              }
                          }
                        else {
                            std::cerr << "The net has no blob " <<desc_par.CaffeDescParam.LayerName<< std::endl;
                          }
                      }
                    }
#endif
                }

              else if (curr_desc.compare("SIFT") == 0) //SIFT
                {
                  SIFTDescriptor SIFTdesc(desc_par.SIFTParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, SIFTdesc,
                                  desc_par.SIFTParam.PEParam.mrSize,
                                  desc_par.SIFTParam.PEParam.patchSize,
                                  desc_par.SIFTParam.PEParam.FastPatchExtraction,
                                  desc_par.SIFTParam.PEParam.photoNorm);
                }

              else if (curr_desc.compare("DSPSIFT") == 0)
                {
                  SIFTDescriptorParams dspsiftparams = desc_par.SIFTParam;
                  dspsiftparams.useRootSIFT = false;
                  dspsiftparams.doNorm = false;
                  SIFTDescriptor DSPSIFTdesc(dspsiftparams);
                  const int num_domains = desc_par.SIFTParam.DSPParam.numScales;
                  for (int dsp_idx = 0; dsp_idx < num_domains+1; dsp_idx++) {
                      dsp_desc = temp_kp1_desc;
                      const double start_coef = desc_par.SIFTParam.DSPParam.startCoef;
                      const double end_coef = desc_par.SIFTParam.DSPParam.endCoef;
                      const double curr_mrSize = desc_par.SIFTParam.PEParam.mrSize * (
                            start_coef +  dsp_idx * (end_coef - start_coef) / num_domains);
                      //        std::cout << dsp_idx << " " << curr_mrSize << std::endl;

                      DescribeRegions(dsp_desc,
                                      temp_img1, DSPSIFTdesc,
                                      curr_mrSize,
                                      desc_par.SIFTParam.PEParam.patchSize,
                                      desc_par.SIFTParam.PEParam.FastPatchExtraction,
                                      desc_par.SIFTParam.PEParam.photoNorm);
                      if (dsp_idx == 0) {

                          for (int kp_idx = 0; kp_idx < dsp_desc.size(); kp_idx++) {
                              int desc_dim = dsp_desc[kp_idx].desc.vec.size();
                              temp_kp1_desc[kp_idx].desc.vec.resize(desc_dim);

                              for (int desc_el_idx = 0; desc_el_idx < desc_dim; desc_el_idx++) {
                                  temp_kp1_desc[kp_idx].desc.vec[desc_el_idx] = dsp_desc[kp_idx].desc.vec[desc_el_idx];
                                  //             std::cerr << temp_kp1_desc[kp_idx].desc.vec[desc_el_idx] << " ";
                                }
                              //         std::cerr << std::endl;
                            }

                        } else {
                          for (int kp_idx = 0; kp_idx < dsp_desc.size(); kp_idx++) {
                              int desc_dim = dsp_desc[kp_idx].desc.vec.size();
                              temp_kp1_desc[kp_idx].desc.vec.resize(desc_dim);

                              for (int desc_el_idx = 0; desc_el_idx < desc_dim; desc_el_idx++) {
                                  temp_kp1_desc[kp_idx].desc.vec[desc_el_idx] += dsp_desc[kp_idx].desc.vec[desc_el_idx];
                                }
                            }
                        }
                      //   dsp_desc.clear();
                    }
                  dspsiftparams.doNorm = true;
                  SIFTDescriptor DSPSIFTdesc1(dspsiftparams);
                  for (int kp_idx = 0; kp_idx < temp_kp1_desc.size(); kp_idx++) {
                      DSPSIFTdesc1.SIFTnorm(temp_kp1_desc[kp_idx].desc.vec);
                    }
                }
              else if (curr_desc.compare("MagnLessSIFT") == 0)
                {
                  SIFTDescriptor SIFTdesc(desc_par.MagnLessSIFTParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, SIFTdesc,
                                  desc_par.MagnLessSIFTParam.PEParam.mrSize,
                                  desc_par.MagnLessSIFTParam.PEParam.patchSize,
                                  desc_par.MagnLessSIFTParam.PEParam.FastPatchExtraction,
                                  desc_par.MagnLessSIFTParam.PEParam.photoNorm);
                }

              else if (curr_desc.compare("BICE") == 0)
                {
                  BICEDescriptor BICEdesc(desc_par.BICEParam);
                  BICEdesc(temp_img1.pixels,temp_kp1_desc);
                }
              else if (curr_desc.compare("LIOP") == 0) //LIOP
                {
                  LIOPDescriptor LIOPDesc(desc_par.LIOPParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, LIOPDesc,
                                  desc_par.LIOPParam.PEParam.mrSize,
                                  desc_par.LIOPParam.PEParam.patchSize,
                                  desc_par.LIOPParam.PEParam.FastPatchExtraction,
                                  desc_par.LIOPParam.PEParam.photoNorm);
                }
              else if (curr_desc.compare("Pixels") == 0) //Raw Pixels
                {
                  PIXELSDescriptor PixelDesc(desc_par.PixelsParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, PixelDesc,
                                  desc_par.PixelsParam.PEParam.mrSize,
                                  desc_par.PixelsParam.PEParam.patchSize,
                                  desc_par.PixelsParam.PEParam.FastPatchExtraction,
                                  desc_par.PixelsParam.PEParam.photoNorm);
                }
              else if (curr_desc.compare("MROGH") == 0) //MROGH
                {
                  MROGHDescriptor MROGHdesc(desc_par.MROGHParam);
                  MROGHdesc(temp_img1.pixels,temp_kp1,temp_kp1_desc);

                }
              else if (curr_desc.compare("ORB") == 0) //ORB
                {
                  //                  else if (curr_desc.compare("ORB") == 0) //ORB (not uses orientation estimated points)
                  //                    {
                  std::cout << "ORB desc" << std::endl;
                  const double mrSizeORB = 3.0;
                  cv::OrbFeatureDetector CurrentDescriptor(det_par.ORBParam.nfeatures,
                                                           det_par.ORBParam.scaleFactor,
                                                           det_par.ORBParam.nlevels,
                                                           det_par.ORBParam.edgeThreshold,
                                                           det_par.ORBParam.firstLevel,
                                                           det_par.ORBParam.WTA_K,
                                                           ORB::HARRIS_SCORE,
                                                           det_par.ORBParam.PEParam.patchSize);
                  if (OpenCV_det) //no data conversion needed
                    {

                      if (curr_det == "ORB") {
                          unsigned int kp_size = temp_kp1.size();
                   //       keypoints_1.clear();
                          keypoints_1.resize(kp_size);
                          for (unsigned int kp_num = 0; kp_num < kp_size; kp_num++) {
                              cv::KeyPoint temp_pt;
                              temp_pt.pt.x = temp_kp1_desc[kp_num].det_kp.x;
                              temp_pt.pt.y = temp_kp1_desc[kp_num].det_kp.y;
                              temp_pt.angle = atan2( temp_kp1_desc[kp_num].det_kp.a12, temp_kp1_desc[kp_num].det_kp.a12);
                              temp_pt.size = temp_kp1_desc[kp_num].det_kp.s *  det_par.ORBParam.PEParam.mrSize; //?mrSizeORB;
                              keypoints_1[kp_num]=temp_pt;
                            }
                        }
                      CurrentDescriptor.compute(CharImage, keypoints_1, descriptors_1);
                    }
                  else {
                      unsigned int kp_size = temp_kp1.size();
                      keypoints_1.reserve(kp_size);
                      for (unsigned int kp_num = 0; kp_num < kp_size; kp_num++) {
                          cv::KeyPoint temp_pt;
                          temp_pt.pt.x = temp_kp1_desc[kp_num].det_kp.x;
                          temp_pt.pt.y = temp_kp1_desc[kp_num].det_kp.y;
                          temp_pt.angle = 0;
                          temp_pt.size = temp_kp1_desc[kp_num].det_kp.s;
                          keypoints_1.push_back(temp_pt);
                        }
                      temp_img1.pixels.convertTo(CharImage, CV_8U);
                      CurrentDescriptor.compute(CharImage, keypoints_1, descriptors_1);
                    }
                  int kp_size = keypoints_1.size();
                  int desc_size = descriptors_1.cols;

                  temp_kp1_desc.resize(kp_size);

                  for (int kp_num = 0; kp_num < kp_size; kp_num++) {
                      temp_kp1_desc[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                      temp_kp1_desc[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                      temp_kp1_desc[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle * M_PI / 180.0);
                      temp_kp1_desc[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle * M_PI / 180.0);
                      temp_kp1_desc[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle * M_PI / 180.0);
                      temp_kp1_desc[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle * M_PI / 180.0);
                      temp_kp1_desc[kp_num].det_kp.s = keypoints_1[kp_num].size /  det_par.ORBParam.PEParam.mrSize;
                      temp_kp1_desc[kp_num].det_kp.response = keypoints_1[kp_num].response;
                      temp_kp1_desc[kp_num].type = temp_kp1[0].type;
                      temp_kp1_desc[kp_num].desc.type = DESC_ORB;
                      temp_kp1_desc[kp_num].desc.vec.resize(desc_size);

                      unsigned char *descPtr = descriptors_1.ptr<unsigned char>(kp_num);
                      for (int jj = 0; jj < desc_size; jj++, descPtr++)
                        temp_kp1_desc[kp_num].desc.vec[jj] = (float) *descPtr;
                    }
                  //ReprojectRegionsAndRemoveTouchBoundary(temp_kp1_desc, temp_img1.H, OriginalImg.cols, OriginalImg.rows, mrSizeORB);
                  //          std::cout << "new size=" << temp_kp1_desc.size() << std::endl;
                }
              //                  ORBDescriptor ORBDesc(det_par.ORBParam);
              //                  DescribeRegions(temp_kp1_desc,
              //                                  temp_img1, ORBDesc,
              //                                  det_par.ORBParam.PEParam.mrSize,
              //                                  det_par.ORBParam.PEParam.patchSize,
              //                                  det_par.ORBParam.PEParam.FastPatchExtraction,
              //                                  det_par.ORBParam.PEParam.photoNorm);

              //                }
//              else if (curr_desc.compare("KAZE") == 0) //KAZE
//                {
//                  KAZEDescriptor KAZEDesc(desc_par.KAZEParam);
//                  DescribeRegions(temp_kp1_desc,
//                                  temp_img1, KAZEDesc,
//                                  desc_par.KAZEParam.PEParam.mrSize,
//                                  desc_par.KAZEParam.PEParam.patchSize,
//                                  desc_par.KAZEParam.PEParam.FastPatchExtraction,
//                                  desc_par.KAZEParam.PEParam.photoNorm);

//                }
              else if (curr_desc.compare("KAZE") == 0) //KAZE
              {

                  if (OpenCV_det) //no data conversion needed
                    {
                      if (curr_det == "Saddle") {
                          unsigned int kp_size = temp_kp1_desc.size();
                          keypoints_1.clear();
                          keypoints_1.reserve(kp_size);
                          for (unsigned int kp_num = 0; kp_num < kp_size; kp_num++) {
                              cv::KeyPoint temp_pt;
                              temp_pt.pt.x = temp_kp1_desc[kp_num].det_kp.x;
                         //      std::cout << temp_pt.pt.x << " ";
                              temp_pt.pt.y = temp_kp1_desc[kp_num].det_kp.y;
                              temp_pt.angle = atan2( temp_kp1_desc[kp_num].det_kp.a12, temp_kp1_desc[kp_num].det_kp.a11);
                              temp_pt.size = temp_kp1_desc[kp_num].det_kp.s * desc_par.KAZEParam.PEParam.mrSize;
                              temp_pt.class_id = 1;
                              temp_pt.octave = 1;
                              temp_pt.response = 1;
                              keypoints_1.push_back(temp_pt);
                            }
                        }
                      if (curr_det == "ORB") {
                          unsigned int kp_size = temp_kp1_desc.size();
                          keypoints_1.clear();
                          keypoints_1.reserve(kp_size);
                          for (unsigned int kp_num = 0; kp_num < kp_size; kp_num++) {
                              cv::KeyPoint temp_pt;
                              temp_pt.pt.x = temp_kp1_desc[kp_num].det_kp.x;
                           //    std::cout << temp_pt.pt.x << " ";
                              temp_pt.pt.y = temp_kp1_desc[kp_num].det_kp.y;
                              temp_pt.octave = 1;
                              temp_pt.response = 1;
                              temp_pt.class_id = 1;

                              temp_pt.angle = atan2( temp_kp1_desc[kp_num].det_kp.a12, temp_kp1_desc[kp_num].det_kp.a11);
                              temp_pt.size = temp_kp1_desc[kp_num].det_kp.s * desc_par.KAZEParam.PEParam.mrSize;
                              keypoints_1.push_back(temp_pt);
                            }
                        }
                      std::cout << "creating scalespace" << std::endl;

                      evolution1.Create_Nonlinear_Scale_Space(temp_img1.pixels * 1.0 / 255.0);

                      std::cout << "computing descs" << keypoints_1.size() << " " << descriptors_1.size() <<  std::endl;
                      evolution1.Compute_Descriptors(keypoints_1, descriptors_1);
                    }
                  else {
                      unsigned int kp_size = temp_kp1_desc.size();
                      keypoints_1.clear();
                      keypoints_1.reserve(kp_size);
                      for (unsigned int kp_num = 0; kp_num < kp_size; kp_num++) {
                          cv::KeyPoint temp_pt;
                          temp_pt.pt.x = temp_kp1_desc[kp_num].det_kp.x;
                        //  std::cout << temp_pt.pt.x << " ";
                          temp_pt.pt.y = temp_kp1_desc[kp_num].det_kp.y;
                          temp_pt.class_id = 1;
                          temp_pt.angle = atan2( temp_kp1_desc[kp_num].det_kp.a12, temp_kp1_desc[kp_num].det_kp.a11);
                          temp_pt.size = temp_kp1_desc[kp_num].det_kp.s * desc_par.KAZEParam.PEParam.mrSize;
                          keypoints_1.push_back(temp_pt);
                        }
                      //temp_img1.pixels.convertTo(CharImage, CV_8U);
                      std::cout << "creating scalespace" << std::endl;

                      evolution1.Create_Nonlinear_Scale_Space(temp_img1.pixels * 1.0 / 255.0);
                      std::cout << "computing descs" << std::endl;
                      evolution1.Compute_Descriptors(keypoints_1, descriptors_1);
                    }
                  std::cout << "akaze ok" << std::endl;

                int kp_size = keypoints_1.size();
                int desc_size = descriptors_1.cols;

                temp_kp1_desc.resize(kp_size);

                for (int kp_num = 0; kp_num < kp_size; kp_num++) {
                  temp_kp1_desc[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1_desc[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1_desc[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle * M_PI / 180.0);
                  temp_kp1_desc[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle * M_PI / 180.0);
                  temp_kp1_desc[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle * M_PI / 180.0);
                  temp_kp1_desc[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle * M_PI / 180.0);
                  temp_kp1_desc[kp_num].det_kp.s = keypoints_1[kp_num].size / det_par.ORBParam.PEParam.mrSize;; //?
                  temp_kp1_desc[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1_desc[kp_num].type = temp_kp1[0].type;
                  temp_kp1_desc[kp_num].desc.type = DESC_KAZE;
                  temp_kp1_desc[kp_num].desc.vec.resize(desc_size);

                  unsigned char *descPtr = descriptors_1.ptr<unsigned char>(kp_num);
                  for (int jj = 0; jj < desc_size; jj++, descPtr++) {
                    temp_kp1_desc[kp_num].desc.vec[jj] = (float) *descPtr;
                 //   std::cout << (float) *descPtr << " ";
                    }
                //  std::cout << std::endl;
                }
                ReprojectRegions(temp_kp1_desc, temp_img1.H, OriginalImg.cols, OriginalImg.rows);
              }
              else if (curr_desc.compare("SURF") == 0) //SURF
                {
                  SURFDescriptor SURFDesc(desc_par.SURFDescParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, SURFDesc,
                                  desc_par.SURFDescParam.PEParam.mrSize,
                                  desc_par.SURFDescParam.PEParam.patchSize,
                                  desc_par.SURFDescParam.PEParam.FastPatchExtraction,
                                  desc_par.SURFDescParam.PEParam.photoNorm);


                }
//              else if (curr_desc.compare("DALI") == 0)
//                {
//                  DALIDescriptor DALIDesc(desc_par.DALIDescParam);
//                  DescribeRegions(temp_kp1_desc,
//                                  temp_img1, DALIDesc,
//                                  desc_par.DALIDescParam.PEParam.mrSize,
//                                  desc_par.DALIDescParam.PEParam.patchSize,
//                                  desc_par.DALIDescParam.PEParam.FastPatchExtraction,
//                                  desc_par.DALIDescParam.PEParam.photoNorm);


//                }
              else if (curr_desc.compare("SMSLD") == 0)
                {
                  SMSLDDescriptor SMSLDDesc(desc_par.SMSLDDescParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, SMSLDDesc,
                                  desc_par.SMSLDDescParam.PEParam.mrSize,
                                  desc_par.SMSLDDescParam.PEParam.patchSize,
                                  desc_par.SMSLDDescParam.PEParam.FastPatchExtraction,
                                  desc_par.SMSLDDescParam.PEParam.photoNorm);


                }
//              else if (curr_desc.compare("FREAK") == 0) //FREAK
//                {
//                  FREAKDescriptor FREAKDesc(desc_par.FREAKParam);
//                  DescribeRegions(temp_kp1_desc,
//                                  temp_img1, FREAKDesc,
//                                  desc_par.FREAKParam.PEParam.mrSize,
//                                  desc_par.FREAKParam.PEParam.patchSize,
//                                  desc_par.FREAKParam.PEParam.FastPatchExtraction,
//                                  desc_par.FREAKParam.PEParam.photoNorm);

//                }
              else if (curr_desc.compare("FREAK") == 0) //FREAK
                {
                  //                  else if (curr_desc.compare("ORB") == 0) //ORB (not uses orientation estimated points)
                  //                    {
                  std::cout << "FREAK desc" << std::endl;
                //  const double mrSizeORB = 3.0;
                  cv::FREAK CurrentDescriptor(desc_par.FREAKParam.orientationNormalized,
                          desc_par.FREAKParam.scaleNormalized,
                          desc_par.FREAKParam.patternScale,
                          desc_par.FREAKParam.nOctaves);

//                  cv::OrbFeatureDetector CurrentDescriptor(det_par.ORBParam.nfeatures,
//                                                           det_par.ORBParam.scaleFactor,
//                                                           det_par.ORBParam.nlevels,
//                                                           det_par.ORBParam.edgeThreshold,
//                                                           det_par.ORBParam.firstLevel,
//                                                           det_par.ORBParam.WTA_K,
//                                                           ORB::HARRIS_SCORE,
//                                                           det_par.ORBParam.PEParam.patchSize);
                  if (OpenCV_det) //no data conversion needed
                    {

                      if (curr_det == "ORB") {
                          unsigned int kp_size = temp_kp1.size();
                          keypoints_1.clear();
                          keypoints_1.reserve(kp_size);
                          for (unsigned int kp_num = 0; kp_num < kp_size; kp_num++) {
                              cv::KeyPoint temp_pt;
                              temp_pt.pt.x = temp_kp1_desc[kp_num].det_kp.x;
                              temp_pt.pt.y = temp_kp1_desc[kp_num].det_kp.y;

                              temp_pt.angle = atan2( temp_kp1_desc[kp_num].det_kp.a12, temp_kp1_desc[kp_num].det_kp.a12);
                              temp_pt.size = temp_kp1_desc[kp_num].det_kp.s *  desc_par.FREAKParam.PEParam.mrSize; //?mrSizeORB;
                              keypoints_1.push_back(temp_pt);
                            }
                        }
                      CurrentDescriptor.compute(CharImage, keypoints_1, descriptors_1);
                    }
                  else {
                      unsigned int kp_size = temp_kp1.size();
                      keypoints_1.reserve(kp_size);
                      for (unsigned int kp_num = 0; kp_num < kp_size; kp_num++) {
                          cv::KeyPoint temp_pt;
                          temp_pt.pt.x = temp_kp1_desc[kp_num].det_kp.x;
                          temp_pt.pt.y = temp_kp1_desc[kp_num].det_kp.y;
                          temp_pt.angle = 0;
                          temp_pt.size = temp_kp1_desc[kp_num].det_kp.s;
                          keypoints_1.push_back(temp_pt);
                        }
                      temp_img1.pixels.convertTo(CharImage, CV_8U);
                      CurrentDescriptor.compute(CharImage, keypoints_1, descriptors_1);
                    }
                  int kp_size = keypoints_1.size();
                  int desc_size = descriptors_1.cols;

                  temp_kp1_desc.resize(kp_size);

                  for (int kp_num = 0; kp_num < kp_size; kp_num++) {
                      temp_kp1_desc[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                      temp_kp1_desc[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                      temp_kp1_desc[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle * M_PI / 180.0);
                      temp_kp1_desc[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle * M_PI / 180.0);
                      temp_kp1_desc[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle * M_PI / 180.0);
                      temp_kp1_desc[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle * M_PI / 180.0);
                      temp_kp1_desc[kp_num].det_kp.s = keypoints_1[kp_num].size *  desc_par.FREAKParam.PEParam.mrSize;
                      temp_kp1_desc[kp_num].det_kp.response = keypoints_1[kp_num].response;
                      temp_kp1_desc[kp_num].type = temp_kp1[0].type;
                      temp_kp1_desc[kp_num].desc.type = DESC_FREAK;
                      temp_kp1_desc[kp_num].desc.vec.resize(desc_size);

                      unsigned char *descPtr = descriptors_1.ptr<unsigned char>(kp_num);
                      for (int jj = 0; jj < desc_size; jj++, descPtr++)
                        temp_kp1_desc[kp_num].desc.vec[jj] = (float) *descPtr;
                    }
                  //ReprojectRegionsAndRemoveTouchBoundary(temp_kp1_desc, temp_img1.H, OriginalImg.cols, OriginalImg.rows, mrSizeORB);
                  //          std::cout << "new size=" << temp_kp1_desc.size() << std::endl;
                }

              else if (curr_desc.compare("DAISY") == 0) //DAISY
                {
                  DAISYDescriptor DAISYDesc(desc_par.DAISYParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, DAISYDesc,
                                  desc_par.DAISYParam.PEParam.mrSize,
                                  desc_par.DAISYParam.PEParam.patchSize,
                                  desc_par.DAISYParam.PEParam.FastPatchExtraction,
                                  desc_par.DAISYParam.PEParam.photoNorm);

                }
              else if (curr_desc.compare("SSIM") == 0) //DAISY
                {
                  SSIMDescriptor SSIMDesc(desc_par.SSIMParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, SSIMDesc,
                                  desc_par.SSIMParam.PEParam.mrSize,
                                  desc_par.SSIMParam.PEParam.patchSize,
                                  desc_par.SSIMParam.PEParam.FastPatchExtraction,
                                  desc_par.SSIMParam.PEParam.photoNorm);

                }
              else if (curr_desc.compare("BRISK") == 0) //BRISK
                {
                  BRISKDescriptor BRISKDesc(det_par.BRISKParam);
                  DescribeRegions(temp_kp1_desc,
                                  temp_img1, BRISKDesc,
                                  det_par.BRISKParam.PEParam.mrSize,
                                  det_par.BRISKParam.PEParam.patchSize,
                                  det_par.BRISKParam.PEParam.FastPatchExtraction,
                                  desc_par.BRISKParam.PEParam.photoNorm);
                  //                  cv::BRISK CurrentDescriptor(det_par.BRISKParam.thresh,
                  //                                              det_par.BRISKParam.octaves,
                  //                                              det_par.BRISKParam.patternScale);
                  //                  if (OpenCV_det) //no data conversion needed
                  //                    {
                  //                      CurrentDescriptor.compute(CharImage, keypoints_1, descriptors_1);
                  //                    }
                  //                  else {
                  //                      int kp_size = temp_kp1.size();
                  //                      keypoints_1.reserve(kp_size);
                  //                      for (int kp_num = 0; kp_num < kp_size; kp_num++) {
                  //                          cv::KeyPoint temp_pt;
                  //                          temp_pt.pt.x = temp_kp1[kp_num].det_kp.x;
                  //                          temp_pt.pt.y = temp_kp1[kp_num].det_kp.y;
                  //                          temp_pt.angle = 0;
                  //                          temp_pt.size = temp_kp1[kp_num].det_kp.s;
                  //                          keypoints_1.push_back(temp_pt);
                  //                        }
                  //                      temp_img1.pixels.convertTo(CharImage, CV_8U);
                  //                      CurrentDescriptor.compute(CharImage, keypoints_1, descriptors_1);
                  //                    }
                  //                  int kp_size = keypoints_1.size();
                  //                  int desc_size = descriptors_1.cols;

                  //                  temp_kp1_desc.resize(kp_size);

                  //                  for (int kp_num = 0; kp_num < kp_size; kp_num++) {
                  //                      temp_kp1_desc[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  //                      temp_kp1_desc[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  //                      temp_kp1_desc[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle * M_PI / 180.0);
                  //                      temp_kp1_desc[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle * M_PI / 180.0);
                  //                      temp_kp1_desc[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle * M_PI / 180.0);
                  //                      temp_kp1_desc[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle * M_PI / 180.0);
                  //                      temp_kp1_desc[kp_num].det_kp.s = keypoints_1[kp_num].size / 3.0; //?
                  //                      temp_kp1_desc[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  //                      temp_kp1_desc[kp_num].type = temp_kp1[0].type;
                  //                      temp_kp1_desc[kp_num].desc.type = DESC_BRISK;
                  //                      temp_kp1_desc[kp_num].desc.vec.resize(desc_size);

                  //                      unsigned char *descPtr = descriptors_1.ptr<unsigned char>(kp_num);
                  //                      for (int jj = 0; jj < desc_size; jj++, descPtr++)
                  //                        temp_kp1_desc[kp_num].desc.vec[jj] = (float) *descPtr;
                  //                    }
                  //                  ReprojectRegions(temp_kp1_desc, temp_img1.H, OriginalImg.cols, OriginalImg.rows);
                }


              temp_kp_map[curr_desc] = temp_kp1_desc;

              time1 = ((double)(getMilliSecs1() - s_time)) / 1000;
              TimeSpent.DescTime += time1;
              s_time = getMilliSecs1();

              // Deallocate the integral image
              if (curr_det.compare("SURF")==0 )
                cvReleaseImage(&int_img);
            }
          OneDetectorKeypointsMapVector[synth] = temp_kp_map;
        }
      for (unsigned int synth=0; synth<n_synths; synth++)
        AddRegions(OneDetectorKeypointsMapVector[synth],curr_det);
    }
}

void ImageRepresentation::SaveRegionsMichal(std::string fname, int mode) {
  std::vector<std::string> desc_names;
  for (std::map<std::string, AffineRegionVectorMap>::const_iterator
       reg_it = RegionVectorMap.begin(); reg_it != RegionVectorMap.end();  ++reg_it) {
      for (AffineRegionVectorMap::const_iterator desc_it = reg_it->second.begin();
           desc_it != reg_it->second.end(); ++desc_it) {
          if (desc_it->first == "None") {
              continue;
            }
          desc_names.push_back(desc_it->first);
        }
    }
  for (unsigned int desc_num = 0; desc_num < desc_names.size(); desc_num++) {
      std::string current_desc_name = desc_names[desc_num];
      std::ofstream kpfile(fname + current_desc_name);
      if (mode == ios::binary) {
          if (kpfile.is_open()) {
              int magic = '\1ffa';
              kpfile.write((char *) &magic, sizeof(int));

              int num_keys = GetDescriptorsNumber(current_desc_name);
              kpfile.write((char *) &num_keys, sizeof(int));
              if (num_keys == 0)
                {
                  std::cerr << "No keypoints detected" << std::endl;
                  kpfile.close();
                  continue;
                }
              //    std::cerr << num_keys << std::endl;
              int desc_dim;

              for (std::map<std::string, AffineRegionVectorMap>::const_iterator
                   reg_it = RegionVectorMap.begin(); reg_it != RegionVectorMap.end(); ++reg_it) {
                  for (AffineRegionVectorMap::const_iterator desc_it = reg_it->second.begin();
                       desc_it != reg_it->second.end(); ++desc_it) {
                      if (desc_it->first != current_desc_name) {
                          continue;
                        }
                      if (desc_it->second.size() == 0)
                        continue;
                      desc_dim = desc_it->second[0].desc.vec.size();
                    }
                }

              if (desc_dim == 0) {

                  std::cerr << "All descriptors are empty" << std::endl;
                  kpfile.close();
                  continue;



                }
              kpfile.write((char *) &desc_dim, sizeof(int));

              //  std::cerr << desc_dim << std::endl;

              int img_w = OriginalImg.cols;
              kpfile.write((char *) &img_w, sizeof(int));
              // std::cerr << img_w << std::endl;

              int img_h = OriginalImg.rows;
              kpfile.write((char *) &img_h, sizeof(int));
              // std::cerr << img_h << std::endl;

              for (std::map<std::string, AffineRegionVectorMap>::const_iterator
                   reg_it = RegionVectorMap.begin(); reg_it != RegionVectorMap.end(); ++reg_it) {
                  for (AffineRegionVectorMap::const_iterator desc_it = reg_it->second.begin();
                       desc_it != reg_it->second.end(); ++desc_it) {
                      if (desc_it->first != current_desc_name) {
                          continue;
                        }
                      int n_desc = desc_it->second.size();

                      for (int i = 0; i < n_desc; i++) {
                          AffineRegion ar = desc_it->second[i];
                          saveARMichalBinary(ar, kpfile);
                        }
                    }
                }
            }
          else {
              std::cerr << "Cannot open file " << fname << " to save keypoints" << endl;
            }
          kpfile.close();
          //      std::cerr << "END OF FILE" << std::endl;
        }
    }
}

void ImageRepresentation::SaveRegions(std::string fname, int mode) {
  std::ofstream kpfile(fname);
  if (mode == ios::binary) {

    } else {
      if (kpfile.is_open()) {
          //    std::map<std::string, AffineRegionVectorMap>::iterator regions_it;
          //    AffineRegionVectorMap::iterator desc_it;
          kpfile << RegionVectorMap.size() << std::endl;
          for (std::map<std::string, AffineRegionVectorMap>::const_iterator
               reg_it = RegionVectorMap.begin(); reg_it != RegionVectorMap.end();  ++reg_it) {
              kpfile << reg_it->first << " " << reg_it->second.size() << std::endl;
              std::cerr << reg_it->first << " " << reg_it->second.size() << std::endl;

              for (AffineRegionVectorMap::const_iterator desc_it = reg_it->second.begin();
                   desc_it != reg_it->second.end(); ++desc_it) {
                  kpfile << desc_it->first << " " << desc_it->second.size() << std::endl;
                  int n_desc = desc_it->second.size();
                  if (n_desc > 0) {
                      kpfile << (desc_it->second)[0].desc.vec.size() << std::endl;
                    } else {
                      std::cerr << "No descriptor " << desc_it->first << std::endl;
                    }
                  for (int i = 0; i < n_desc ; i++ ) {
                      AffineRegion ar = desc_it->second[i];
                      saveAR(ar, kpfile);
                      kpfile << std::endl;
                    }
                }
            }
        }
      else {
          std::cerr << "Cannot open file " << fname << " to save keypoints" << endl;
        }
      kpfile.close();
    }
}

void ImageRepresentation::LoadRegions(std::string fname) {

  std::ifstream kpfile(fname);
  if (kpfile.is_open()) {
      int numberOfDetectors = 0;
      kpfile >> numberOfDetectors;
      //    std::cerr << "numberOfDetectors=" <<numberOfDetectors << std::endl;
      for (int det = 0; det < numberOfDetectors; det++) {
          std::string det_name;
          int num_of_descs = 0;
          kpfile >> det_name;
          kpfile >> num_of_descs;
          //      std::cerr << det_name << " " << num_of_descs << std::endl;

          //reg_it->first << " " << reg_it->second.size() << std::endl;
          for (int desc = 0; desc < num_of_descs; desc++)  {
              AffineRegionVector desc_regions;
              std::string desc_name;
              kpfile >> desc_name;

              int num_of_kp = 0;
              kpfile >> num_of_kp;
              int desc_size;
              kpfile >> desc_size;
              //        std::cerr << desc_name << " " << num_of_kp << " " << desc_size << std::endl;
              for (int kp = 0; kp < num_of_kp; kp++)  {
                  AffineRegion ar;
                  loadAR(ar, kpfile);
                  desc_regions.push_back(ar);
                }
              AddRegions(desc_regions,det_name,desc_name);
            }
        }
    }
  else {
      std::cerr << "Cannot open file " << fname << " to save keypoints" << endl;
    }
  kpfile.close();
}
void ImageRepresentation::SaveDescriptorsBenchmark(std::string fname1) {
  std::vector<std::string> desc_names;
  int num_keys  = 0;
  std::ofstream kpfile(fname1);
  if (kpfile.is_open()) {
      for (std::map<std::string, AffineRegionVectorMap>::const_iterator
           reg_it = RegionVectorMap.begin(); reg_it != RegionVectorMap.end();  ++reg_it) {
          for (AffineRegionVectorMap::const_iterator desc_it = reg_it->second.begin();
               desc_it != reg_it->second.end(); ++desc_it) {

              if (desc_it->first == "None") {
                  continue;
                }
              num_keys += desc_it->second.size();
            }
        }

      std::cerr << num_keys << std::endl;
      for (std::map<std::string, AffineRegionVectorMap>::const_iterator
           reg_it = RegionVectorMap.begin(); reg_it != RegionVectorMap.end();  ++reg_it) {
          for (AffineRegionVectorMap::const_iterator desc_it = reg_it->second.begin();
               desc_it != reg_it->second.end(); ++desc_it) {

              if (desc_it->first == "None") {
                  continue;
                }
              //   int num_keys = desc_it->second.size();
              for (int i = 0; i < num_keys ; i++ ) {
                  AffineRegion ar = desc_it->second[i];
                  for (int ddd = 0; ddd < ar.desc.vec.size(); ++ddd){
                      kpfile << ar.desc.vec[ddd] << " ";
                    }
                  kpfile << std::endl;
                }
            }
        }
    }  else {
      std::cerr << "Cannot open file " << fname1 << " to save keypoints" << endl;
    }
  kpfile.close();
}
void ImageRepresentation::SaveRegionsBenchmark(std::string fname1, std::string fname2) {
  std::vector<std::string> desc_names;

  std::ofstream kpfile(fname1);
  std::ofstream kpfile2(fname2);
  int num_keys = 0;
  if (kpfile.is_open() && kpfile2.is_open() ) {
      for (std::map<std::string, AffineRegionVectorMap>::const_iterator
           reg_it = RegionVectorMap.begin(); reg_it != RegionVectorMap.end();  ++reg_it) {
          for (AffineRegionVectorMap::const_iterator desc_it = reg_it->second.begin();
               desc_it != reg_it->second.end(); ++desc_it) {

              if (desc_it->first != "None") {
                  continue;
                }
              num_keys += desc_it->second.size();
            }
        }
      kpfile << num_keys << std::endl;
      kpfile2 << num_keys << std::endl;

      for (std::map<std::string, AffineRegionVectorMap>::const_iterator
           reg_it = RegionVectorMap.begin(); reg_it != RegionVectorMap.end();  ++reg_it) {
          for (AffineRegionVectorMap::const_iterator desc_it = reg_it->second.begin();
               desc_it != reg_it->second.end(); ++desc_it) {

              if (desc_it->first != "None") {
                  continue;
                }
              int num_keys1 = desc_it->second.size();

              for (int i = 0; i < num_keys1 ; i++ ) {
                  AffineRegion ar = desc_it->second[i];
                  saveARBench(ar, kpfile,kpfile2);
                  kpfile << std::endl;
                  kpfile2 << std::endl;
                }
            }

        }
    }
  else {
      std::cerr << "Cannot open file " << fname1 << " to save keypoints" << endl;
    }
  kpfile.close();
  kpfile2.close();

}
//}
void ImageRepresentation::SynthDetectDescribeKeypointsBench(IterationViewsynthesisParam &synth_par,
                                                            DetectorsParameters &det_par,
                                                            DescriptorsParameters &desc_par,
                                                            DominantOrientationParams &dom_ori_par, double *H,
                                                            const int width2, const int height2) {
  double time1 = 0;
#ifdef _OPENMP
  omp_set_nested(1);
#endif
#pragma omp parallel for schedule (dynamic,1)
  for (unsigned int det=0; det < DetectorNames.size(); det++)
    {
      std::string curr_det = DetectorNames[det];
      unsigned int n_synths = synth_par[curr_det].size();

      std::vector<AffineRegionVectorMap> OneDetectorKeypointsMapVector;
      OneDetectorKeypointsMapVector.resize(n_synths);

#pragma omp parallel for schedule (dynamic,1)
      for (unsigned int synth=0; synth<n_synths; synth++)
        {
          ///Synthesis
          long s_time = getMilliSecs1();
          AffineRegionVector temp_kp1;
          AffineRegionVectorMap temp_kp_map;
          SynthImage temp_img1;
          GenerateSynthImageCorr(OriginalImg, temp_img1, Name.c_str(),
                                 synth_par[curr_det][synth].tilt,
                                 synth_par[curr_det][synth].phi,
                                 synth_par[curr_det][synth].zoom,
                                 synth_par[curr_det][synth].InitSigma,
                                 synth_par[curr_det][synth].doBlur, synth);
          //    std::cerr << "generated" << std::endl;
          time1 = ((double)(getMilliSecs1() - s_time))/1000;
          TimeSpent.SynthTime += time1;

          ///Structures initialization
          IplImage *int_img; //for SURF
          IpVec ipts1;//for SURF
          cv::Mat CharImage; //for OpenCV detectors

          aka::AKAZEOptions options; //For KAZE
          options.img_width = temp_img1.pixels.cols;
          options.img_height = temp_img1.pixels.rows;
          aka::AKAZE evolution1(options);

          std::vector<cv::KeyPoint> keypoints_1; //for binary-dets
          cv::Mat descriptors_1; //for binary-dets

          bool OpenCV_det = ((curr_det.compare("ORB") == 0) ||
                             (curr_det.compare("FAST") == 0) ||
                             (curr_det.compare("STAR") == 0) ||
                             (curr_det.compare("KAZE") == 0) ||
                             (curr_det.compare("BRISK") == 0) ||
                             (curr_det.compare("Saddle") == 0));
          bool SIFT_like_desc = false;
          bool HalfSIFT_like_desc = false;

          for (unsigned int i_desc=0; i_desc < synth_par[curr_det][synth].descriptors.size();i_desc++) {
              std::string curr_desc = synth_par[curr_det][synth].descriptors[i_desc];
              if  (curr_desc.find("LIOP") != std::string::npos) {
                  SIFT_like_desc = true;
                }
              if  (curr_desc.find("SIFT") != std::string::npos) {
                  if (curr_desc.find("Half") != std::string::npos) {
                      HalfSIFT_like_desc = true;
                    } else {
                      SIFT_like_desc = true;
                    }
                }
            }
          //      int rnd1 = (int) getMilliSecs() + (std::rand() % (int)(1001));
          //      std::string img_fname = "FOCI"+std::to_string(synth+rnd1)+".png";
          //      cv::imwrite(img_fname,temp_img1.pixels);
          /// Detection
          s_time = getMilliSecs1();
          if (curr_det.compare("HessianAffine")==0)
            {
              DetectAffineRegions(temp_img1, temp_kp1,det_par.HessParam,DET_HESSIAN,DetectAffineKeypoints);
            }
          else if (curr_det.compare("ReadAffs") == 0) {
              std::ifstream focikp(det_par.ReadAffsFromFileParam.fname);
              if (focikp.is_open()) {
                  int kp_size;
                  focikp >> kp_size;
                  temp_kp1.reserve(kp_size);
                  for (int kp_num = 0; kp_num < kp_size; kp_num++) {
                      AffineRegion temp_region;
                      temp_region.det_kp.pyramid_scale = -1;
                      temp_region.det_kp.octave_number = -1;
                      temp_region.det_kp.sub_type = 101;
                      focikp >> temp_region.det_kp.x;
                      focikp >> temp_region.det_kp.y;
                      focikp >> temp_region.det_kp.s;
                      focikp >> temp_region.det_kp.a11;
                      focikp >> temp_region.det_kp.a12;
                      focikp >> temp_region.det_kp.a21;
                      focikp >> temp_region.det_kp.a22;
                      temp_region.det_kp.response = 100;
                      temp_region.type = DET_FOCI;
                      temp_kp1.push_back(temp_region);
                    }
                }
              focikp.close();
            }
          else if (curr_det.compare("FOCI")==0)
            {
              //  DetectAffineRegions(temp_img1, temp_kp1,det_par.DoGParam,DET_DOG,DetectAffineKeypoints);
              int rnd1 = (int) getMilliSecs() + (std::rand() % (int)(1001));
              std::string img_fname = "FOCI"+std::to_string(synth+rnd1)+".png";
              cv::imwrite(img_fname,temp_img1.pixels);
              //srand();
              std::string command = "wine EdgeFociAndBice.exe -mi -i " + img_fname;
              //   command += " -mi";
              if (det_par.FOCIParam.numberKPs > 0) {
                  command += " -n "+ std::to_string(det_par.FOCIParam.numberKPs);
                }
              if (det_par.FOCIParam.computeOrientation) {
                  command += " -co";
                  if (det_par.FOCIParam.secondOrientation) {
                      command += " -mo ";
                    }
                }
              std::string fname1 = "FOCI" + std::to_string(synth+rnd1) + ".txt";
              command += " -o " + fname1;
              std::cerr << command <<std::endl;
              system(command.c_str());
              std::ifstream focikp(fname1);
              if (focikp.is_open()) {

                  int kp_size;
                  focikp >> kp_size;

                  temp_kp1.reserve(kp_size);
                  const float initialSigma = 1.6;
                  cv::Mat gmag, gori, orimask;
                  std::vector<unsigned char> workspace;
                  cv::Mat mask, img_foci, imgHes, fx, fy;
                  AffineShapeParams par = det_par.HessParam.AffineShapePars;
                  gmag = cv::Mat(par.patchSize, par.patchSize, CV_32FC1),
                      gori = cv::Mat(par.patchSize, par.patchSize, CV_32FC1),
                      orimask = cv::Mat(par.patchSize, par.patchSize, CV_32FC1),
                      mask = cv::Mat(par.smmWindowSize, par.smmWindowSize, CV_32FC1),
                      img_foci = cv::Mat(par.smmWindowSize, par.smmWindowSize, CV_32FC1),
                      fx = cv::Mat(par.smmWindowSize, par.smmWindowSize, CV_32FC1),
                      fy = cv::Mat(par.smmWindowSize, par.smmWindowSize, CV_32FC1),


                      computeGaussMask(mask);
                  computeCircularGaussMask(orimask, par.smmWindowSize);
                  for (int kp_num=0; kp_num < kp_size; kp_num++)
                    {
                      AffineRegion temp_region;
                      temp_region.det_kp.pyramid_scale = -1;
                      temp_region.det_kp.octave_number = -1;
                      temp_region.det_kp.sub_type = 55;
                      focikp >> temp_region.det_kp.x;
                      focikp >> temp_region.det_kp.y;
                      focikp >> temp_region.det_kp.a11;
                      temp_region.det_kp.a11 = sqrt(temp_region.det_kp.a11);

                      focikp >> temp_region.det_kp.a12;
                      temp_region.det_kp.a12 = sqrt(temp_region.det_kp.a12);
                      temp_region.det_kp.a21 = 0;
                      focikp >> temp_region.det_kp.a22;
                      temp_region.det_kp.a22 = sqrt(temp_region.det_kp.a22);
                      temp_region.det_kp.s = 1.0;  //?
                      focikp >> temp_region.det_kp.response;
                      temp_region.type = DET_FOCI;
                      float angle;
                      focikp >> angle; //Not good yet

                      temp_region.det_kp.s *= sqrt(fabs(temp_region.det_kp.a11*temp_region.det_kp.a22
                                                        - temp_region.det_kp.a12*temp_region.det_kp.a21));
                      //
                      //
                      rectifyAffineTransformationUpIsUp(temp_region.det_kp.a11,
                                                        temp_region.det_kp.a12,
                                                        temp_region.det_kp.a21,
                                                        temp_region.det_kp.a22);

                      if (det_par.FOCIParam.doBaumberg) { // Rewrite this!!!!!!!!
                          float eigen_ratio_act = 0.0f, eigen_ratio_bef = 0.0f;
                          float u11 = 1.0f, u12 = 0.0f, u21 = 0.0f, u22 = 1.0f, l1 = 1.0f, l2 = 1.0f;
                          float lx = temp_region.det_kp.x, ly = temp_region.det_kp.y;
                          float ratio =  temp_region.det_kp.s / (initialSigma);
                          cv::Mat U, V, d, Au, Ap, D;
                          // kernel size...
                          //        std::cerr << "do baum" << std::endl;
                          //        std::cerr << det_par.HessParam.AffineShapePars.smmWindowSize << std::endl;
                          const int maskPixels = det_par.HessParam.AffineShapePars.smmWindowSize
                              * det_par.HessParam.AffineShapePars.smmWindowSize;

                          if (interpolateCheckBorders(temp_img1.pixels.cols,temp_img1.pixels.rows,
                                                      (float) temp_region.det_kp.x,
                                                      (float) temp_region.det_kp.y,
                                                      (float) temp_region.det_kp.a11,
                                                      (float) temp_region.det_kp.a12,
                                                      (float) temp_region.det_kp.a21,
                                                      (float) temp_region.det_kp.a22,
                                                      2*5.0*ratio,
                                                      2*5.0*ratio) ) {
                              continue;
                            }
                          for (int l = 0; l < det_par.HessParam.AffineShapePars.maxIterations; l++)
                            {
                              float a = 0, b = 0, c = 0;

                              // warp input according to current shape matrix
                              //           std::cerr << "before interp ok" << std::endl;

                              interpolate(temp_img1.pixels, lx, ly, u11*ratio, u12*ratio, u21*ratio, u22*ratio, img_foci);
                              //            std::cerr << "after interp ok" << std::endl;
                              // compute SMM on the warped patch
                              float *maskptr = mask.ptr<float>(0);
                              float *pfx = fx.ptr<float>(0), *pfy = fy.ptr<float>(0);
                              //           cv::imwrite("gav.png",img_foci);
                              //           std::cerr << "before grad" << std::endl;
                              // float *img_fociptr = img_foci.ptr<float>(0); //!
                              computeGradient(img_foci, fx, fy);
                              //          std::cerr << "grad ok" << std::endl;

                              // estimate SMM
                              for (int i = 0; i < maskPixels; ++i)
                                {
                                  const float v = (*maskptr);
                                  const float gxx = *pfx;
                                  const float gyy = *pfy;
                                  const float gxy = gxx * gyy;

                                  a += gxx * gxx * v;
                                  b += gxy * v;
                                  c += gyy * gyy * v;
                                  pfx++;
                                  pfy++;
                                  maskptr++;
                                }
                              a /= maskPixels;
                              b /= maskPixels;
                              c /= maskPixels;

                              // compute inverse sqrt of the SMM
                              invSqrt(a, b, c, l1, l2);

                              if ((a != a) || (b != b) || (c !=c)){ //check for nan
                                  break;
                                }

                              // update e igen ratios
                              eigen_ratio_bef = eigen_ratio_act;
                              eigen_ratio_act = 1.0 - l2 / l1;

                              // accumulate the affine shape matrix
                              float u11t = u11, u12t = u12;

                              u11 = a*u11t+b*u21;
                              u12 = a*u12t+b*u22;
                              u21 = b*u11t+c*u21;
                              u22 = b*u12t+c*u22;


                              // compute the eigen values of the shape matrix
                              if (!getEigenvalues(u11, u12, u21, u22, l1, l2))
                                break;

                              // leave on too high anisotropy
                              if ((l1/l2>6) || (l2/l1>6))
                                break;

                              if (eigen_ratio_act < det_par.HessParam.AffineShapePars.convergenceThreshold
                                  && eigen_ratio_bef < det_par.HessParam.AffineShapePars.convergenceThreshold) {
                                  temp_region.det_kp.a11 = u11;
                                  temp_region.det_kp.a12 = u12;
                                  temp_region.det_kp.a21 = u21;
                                  temp_region.det_kp.a22 = u22;

                                  temp_kp1.push_back(temp_region);
                                  break;
                                }
                            }
                        } else {
                          temp_kp1.push_back(temp_region);
                        }
                    }
                }
              //        std::cerr << "cloase ok" << std::endl;

              //std::cerr << temp_kp1.size() << std::endl;
              focikp.close();
              std::string rm_command = "rm " + fname1;
              system(rm_command.c_str());
              rm_command = "rm " + img_fname;
              system(rm_command.c_str());

            }
          else if (curr_det.compare("DoG")==0)
            {
              DetectAffineRegions(temp_img1, temp_kp1,det_par.DoGParam,DET_DOG,DetectAffineKeypoints);
            }
          else if (curr_det.compare("HarrisAffine")==0)
            {
              DetectAffineRegions(temp_img1, temp_kp1,det_par.HarrParam,DET_HARRIS,DetectAffineKeypoints);
            }
          else if (curr_det.compare("MSER")==0)
            {
              DetectAffineRegions(temp_img1, temp_kp1,det_par.MSERParam,DET_MSER,DetectMSERs);
            }
          else if (curr_det.compare("SURF")==0)
            {
              IplImage Iplimg1 = temp_img1.pixels;
              // Create integral-image representation of the image
              int_img = Integral(&Iplimg1);

              int octaves = det_par.SURFParam.octaves;
              int intervals = det_par.SURFParam.intervals;
              int init_sample = det_par.SURFParam.init_sample;
              float thres = det_par.SURFParam.thresh;
              // Create Fast Hessian Object
              FastHessian fh(int_img, ipts1, octaves, intervals, init_sample, thres);

              // Extract interest points and store in vector ipts
              fh.getIpoints();

              int kp_size = ipts1.size();
              temp_kp1.resize(kp_size);
              for (int kp_num=0; kp_num < kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x =ipts1[kp_num].x;
                  temp_kp1[kp_num].det_kp.y = ipts1[kp_num].y;
                  temp_kp1[kp_num].det_kp.a11 = cos(ipts1[kp_num].orientation);
                  temp_kp1[kp_num].det_kp.a12 = sin(ipts1[kp_num].orientation);
                  temp_kp1[kp_num].det_kp.a21 = -sin(ipts1[kp_num].orientation);
                  temp_kp1[kp_num].det_kp.a22 = cos(ipts1[kp_num].orientation);
                  temp_kp1[kp_num].det_kp.s = ipts1[kp_num].scale;
                  temp_kp1[kp_num].type = DET_SURF;
                }
            }
          else if (curr_det.compare("ORB")==0)
            {
              cv::OrbFeatureDetector CurrentDetector(det_par.ORBParam.nfeatures,
                                                     det_par.ORBParam.scaleFactor,
                                                     det_par.ORBParam.nlevels,
                                                     det_par.ORBParam.edgeThreshold,
                                                     det_par.ORBParam.firstLevel,
                                                     det_par.ORBParam.WTA_K,
                                                     ORB::HARRIS_SCORE,
                                                     det_par.ORBParam.PEParam.patchSize);
              temp_img1.pixels.convertTo(CharImage,CV_8U);
              CurrentDetector.detect(CharImage, keypoints_1);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_ORB;
                }
            }
          else if (curr_det.compare("KAZE")==0)
            {
              // evolution1.Create_Nonlinear_Scale_Space(temp_img1.pixels *1.0/255.0);
              // evolution1.Feature_Detection(keypoints_1);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_KAZE;
                }
            }
          else if (curr_det.compare("FAST")==0)
            {
              temp_img1.pixels.convertTo(CharImage,CV_8U);
              cv::FASTX(CharImage,keypoints_1,det_par.FASTParam.threshold,
                        det_par.FASTParam.nonmaxSuppression,det_par.FASTParam.type);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_FAST;
                }
            }
          else if (curr_det.compare("BRISK")==0)
            {
              temp_img1.pixels.convertTo(CharImage,CV_8U);
              cv::BRISK CurrentDetector(det_par.BRISKParam.thresh,
                                        det_par.BRISKParam.octaves,
                                        det_par.BRISKParam.patternScale);
              CurrentDetector.detect(CharImage, keypoints_1);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_BRISK;
                }
            }
          else if (curr_det.compare("STAR")==0)
            {
              temp_img1.pixels.convertTo(CharImage,CV_8U);
              cv::StarFeatureDetector CurrentDetector(det_par.STARParam.maxSize,
                                                      det_par.STARParam.responseThreshold,
                                                      det_par.STARParam.lineThresholdProjected,
                                                      det_par.STARParam.lineThresholdBinarized,
                                                      det_par.STARParam.suppressNonmaxSize);
              CurrentDetector.detect(CharImage, keypoints_1);
              int kp_size = keypoints_1.size();
              temp_kp1.resize(kp_size);

              for (int kp_num=0; kp_num<kp_size; kp_num++)
                {
                  temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
                  temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
                  temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
                  temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
                  temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
                  temp_kp1[kp_num].type = DET_STAR;
                }
            }


          //    std::cerr << "detected" << std::endl;


          time1 = ((double)(getMilliSecs1() - s_time))/1000;
          TimeSpent.DetectTime += time1;
          /// Orientation estimation

          AffineRegionVector temp_kp1_SIFT_like_desc;
          AffineRegionVector temp_kp1_HalfSIFT_like_desc;
          AffineRegionVector temp_kp1_upright;

          if (SIFT_like_desc) {
              DetectOrientation(temp_kp1, temp_kp1_SIFT_like_desc, temp_img1,
                                dom_ori_par.PEParam.mrSize, dom_ori_par.PEParam.patchSize,
                                false,dom_ori_par.maxAngles,
                                dom_ori_par.threshold, false);
              ReprojectRegions(temp_kp1_SIFT_like_desc, temp_img1.H, OriginalImg.cols, OriginalImg.rows);
              ReprojectRegionsBackReal(temp_kp1_SIFT_like_desc, H, width2,height2);
              temp_kp_map["None"] = temp_kp1_SIFT_like_desc;
            }
          if (HalfSIFT_like_desc) {
              DetectOrientation(temp_kp1, temp_kp1_HalfSIFT_like_desc, temp_img1,
                                dom_ori_par.PEParam.mrSize, dom_ori_par.PEParam.patchSize,
                                true,dom_ori_par.maxAngles,
                                dom_ori_par.threshold, false);

              ReprojectRegions(temp_kp1_HalfSIFT_like_desc, temp_img1.H, OriginalImg.cols, OriginalImg.rows);
              ReprojectRegionsBackReal(temp_kp1_HalfSIFT_like_desc, H, width2,height2);
              temp_kp_map["None"] = temp_kp1_HalfSIFT_like_desc;
            }
          if (dom_ori_par.addUpRight) {
              DetectOrientation(temp_kp1, temp_kp1_upright, temp_img1,
                                dom_ori_par.PEParam.mrSize, dom_ori_par.PEParam.patchSize,
                                false,0, 1.0, true);

              ReprojectRegions(temp_kp1_upright, temp_img1.H, OriginalImg.cols, OriginalImg.rows);
              ReprojectRegionsBackReal(temp_kp1_upright, H, width2,height2);

              temp_kp_map["None"] = temp_kp1_upright;
            }
          //   std::cerr << "oriented" << std::endl;

          OneDetectorKeypointsMapVector[synth] = temp_kp_map;
        }
      for (unsigned int synth=0; synth<n_synths; synth++)
        AddRegions(OneDetectorKeypointsMapVector[synth],curr_det);
    }

}
