/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/
#ifndef DESCRIPTORS_PARAMETERS_HPP
#define DESCRIPTORS_PARAMETERS_HPP

#include <opencv2/features2d/features2d.hpp>
#include "detectors/structures.hpp"
#include "descriptors/freakdescriptor.hpp"
#include "descriptors/orbdescriptor.hpp"
#include "descriptors/briskdescriptor.hpp"

/*
#include "descriptors/daisydescriptor.hpp"
*/
#include "matching/siftdesc.h"
#include "descriptors/mroghdesc.hpp"
#include "descriptors/pixelsdesc.hpp"
#include "descriptors/smslddescriptor.hpp"
#include "descriptors/ssimdescriptor.hpp"

struct DominantOrientationParams {

  int maxAngles;
  float threshold;
  bool addUpRight;
  bool halfSIFTMode;
  PatchExtractionParams PEParam;
  DominantOrientationParams() {
    maxAngles = -1;
    threshold = 0.8;
    addUpRight = false;
    halfSIFTMode = false;
  }
};


struct CaffeDescriptorParams
{
  std::string WeightsFile;
  std::string ProtoTxt;
  double MeanB;
  double MeanG;
  double MeanR;
  int batchSize;
  std::string LayerName;
  std::string Pooling;
  std::string Normalization;
  bool DoSIFTLikeOrientation;
  int maxOrientations;
  bool estimateOrientation;
  double orientTh;
    PatchExtractionParams PEParam;
  CaffeDescriptorParams()
  {
    MeanB=104;
    MeanG=117;
    MeanR=123;
    batchSize = 256;
    Pooling = "none";
    Normalization = "L2";
    DoSIFTLikeOrientation = true;
    maxOrientations = 0;
    estimateOrientation= true;
    orientTh = 0.8;
  }
};

struct DescriptorsParameters {
  SIFTDescriptorParams SIFTParam;
  SIFTDescriptorParams MagnLessSIFTParam;
  SIFTDescriptorParams RootSIFTParam;
  SIFTDescriptorParams HalfSIFTParam;
  SIFTDescriptorParams HalfRootSIFTParam;
//  LIOPDescriptorParams LIOPParam;
  FREAKParams FREAKParam;
  BRISKParams BRISKParam;
  MROGHParams MROGHParam;
  CaffeDescriptorParams CaffeDescParam;
 // BICEParams BICEParam;
  PIXELSDescriptorParams PixelsParam;
  //KAZEParams KAZEParam;
  SURFParams SURFDescParam;
 // DALIParams DALIDescParam;
  SMSLDParams SMSLDDescParam;
 // DAISYParams DAISYParam;
  SSIMParams SSIMParam;
};

#endif // DESCRIPTORS_PARAMETERS_HPP
