#ifndef IMAGEREPRESENTATION_H
#define IMAGEREPRESENTATION_H


#include <vector>
#include <string>
#include <map>
#include "synth-detection.hpp"
#include "detectors/structures.hpp"
#include "detectors/detectors_parameters.hpp"
#include "descriptors_parameters.hpp"

//inline long getMilliSecs()
//{
//  timeval t;
//  gettimeofday(&t, NULL);
//  return t.tv_sec*1000 + t.tv_usec/1000;
//}

///
#ifdef WITH_CAFFE
#include "caffe/caffe.hpp"
#endif
class ImageRepresentation
{
public:
  ImageRepresentation();
  ImageRepresentation(cv::Mat _in_img, std::string _name);
  ~ImageRepresentation();
#ifdef WITH_CAFFE
  void InitCaffe(std::shared_ptr<caffe::Net<float> > net_ptr);
#endif
  std::vector< std::map<std::string, SynthImage> > SynthViews;

  descriptor_type GetDescriptorType(std::string desc_name);
  detector_type GetDetectorType(std::string det_name);
  TimeLog GetTimeSpent();
  int GetDescriptorDimension(std::string desc_name);
  int GetRegionsNumber(std::string det_name = "All");
  int GetDescriptorsNumber(std::string desc_name = "All", std::string det_name = "All");
  cv::Mat GetDescriptorsMatByDetDesc(const std::string desc_name,const std::string det_name = "All");
  cv::Mat GetDescriptorsMatByDetDesc(std::vector<Point2f> &coordinates,
                                     const std::string desc_name,const std::string det_name = "All");
  AffineRegionVector GetAffineRegionVector(std::string desc_name, std::string det_name, std::vector<int> idxs);
  AffineRegionVector GetAffineRegionVector(std::string desc_name, std::string det_name);
  AffineRegion GetAffineRegion(std::string desc_name, std::string det_name, int idx);
  void SynthDetectDescribeKeypoints (IterationViewsynthesisParam &synth_par,
                                     DetectorsParameters &det_par,
                                     DescriptorsParameters &desc_par,
                                     DominantOrientationParams &dom_ori_par);
  void SynthDetectDescribeKeypointsBench (IterationViewsynthesisParam &synth_par,
                                          DetectorsParameters &det_par,
                                          DescriptorsParameters &desc_par,
                                          DominantOrientationParams &dom_ori_par, double* H,
                                          const int width2, const int height2);
  cv::Mat OriginalImg;
  void SaveRegions(std::string fname, int mode);
  void SaveRegionsMichal(std::string fname, int mode);
  void SaveRegionsBenchmark(std::string fname1, std::string fname2);
  void SaveDescriptorsBenchmark(std::string fname1);
  void LoadRegions(std::string fname);

protected:
  TimeLog TimeSpent;
  void AddRegions(AffineRegionVector& RegionsToAdd,std::string det_name, std::string desc_name);
  void AddRegions(AffineRegionVectorMap& RegionsMapToAdd,std::string det_name);
  void AddRegionsToList(AffineRegionList &kp_list, AffineRegionList &new_kps);

  std::map<std::string, AffineRegionVectorMap> RegionVectorMap;
  std::string Name;
#ifdef WITH_CAFFE
  std::shared_ptr<caffe::Net<float> > caffe_net_ptr;
  bool NetInUse;
#endif

};

#endif // IMAGEREPRESENTATION_H
