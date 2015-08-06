#ifndef CORRESPONDENCEBANK_H
#define CORRESPONDENCEBANK_H

#include <vector>
#include <string>
#include <map>

#include "detectors/structures.hpp"
#include "detectors/detectors_parameters.hpp"
#include "descriptors_parameters.hpp"
#include "matching/matching.hpp"
#include "imagerepresentation.h"
typedef std::map<std::string, TentativeCorrespListExt> CorrespondencesMap;

class CorrespondenceBank
{
public:
  CorrespondenceBank();
  ~CorrespondenceBank()
  {
    DescriptorsDBIndex = cv::flann::Index();
  }
  int GetCorrespondencesNumber(std::string desc_name = "All", std::string det_name = "All");
  double GetSpentTime(std::string desc_name = "All", std::string det_name = "All");
  TentativeCorrespListExt GetCorresponcesVector(std::string desc_name = "All", std::string det_name = "All");
  int MatchImgReps(ImageRepresentation &imgrep1, ImageRepresentation &imgrep2,
                   IterationViewsynthesisParam &synth_par,const WhatToMatch WhatToMatchNow,
                   const MatchPars &par, const DescriptorsParameters &desc_pars);
  void ClearCorrespondences(std::string det_name, std::string desc_name);
  std::map<std::string, cv::Mat> DescriptorsDBForSNN;
  cv::flann::Index DescriptorsDBIndex;
  cv::Mat GetDescriptorsDB(std::string desc_name);
  cv::Mat DB;
protected:

  void AddCorrespondences(std::map<std::string, TentativeCorrespListExt> & CorrsToAddMap,std::string desc_name);
  void AddCorrespondences(TentativeCorrespListExt& CorrsToAdd,std::string det_name, std::string desc_name);
  void AddCorrespondencesToList(TentativeCorrespListExt& BaseCorrs, TentativeCorrespListExt& CorrsToAdd);
  std::map<std::string, std::map<std::string, double> > MatchingTimeMapMap;
  std::map<std::string, CorrespondencesMap> CorrespondencesMapMap;


private:
  descriptor_type GetDescriptorType(std::string desc_name);
  detector_type GetDetectorType(std::string det_name);

};

#endif // CORRESPONDENCEBANK_H
