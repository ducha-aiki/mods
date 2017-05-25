#ifndef DESCRIPTORS_PARAMETERS_HPP
#define DESCRIPTORS_PARAMETERS_HPP

#include "detectors/structures.hpp"
#include "matching/siftdesc.h"
#include "descriptors/mroghdesc.hpp"
#include "descriptors/pixelsdesc.hpp"
#include "descriptors/surfdescriptor.hpp"
#include "descriptors/freakdescriptor.hpp"
#include "descriptors/kazedescriptor.hpp"
#include "descriptors/bicedescriptor.hpp"
//#include "descriptors/dalidescriptor.hpp"
#include "descriptors/smslddescriptor.hpp"
#include "descriptors/orbdescriptor.hpp"
#include "descriptors/briskdescriptor.hpp"
#include "descriptors/daisydescriptor.hpp"
#include "descriptors/ssimdescriptor.hpp"
#include "matching/liopdesc.hpp"

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

struct CLIDescriptorParams {
    PatchExtractionParams PEParam;
    std::string runfile;
    std::string hardcoded_input_fname;
    std::string hardcoded_output_fname;
    bool hardcoded_run_string;
    CLIDescriptorParams() {
     hardcoded_run_string = true;
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
  int patchSize;
  double mrSize;
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
    mrSize = 5.192;
    batchSize = 256;
    patchSize = 32;
    Pooling = "none";
    Normalization = "L2";
    DoSIFTLikeOrientation = true;
    maxOrientations = 0;
    estimateOrientation= true;
    orientTh = 0.8;
  }
};

struct DescriptorsParameters {
  CLIDescriptorParams CLIDescParam;
  SIFTDescriptorParams SIFTParam;  
  SIFTDescriptorParams ResSIFTParam;
  SIFTDescriptorParams MagnLessSIFTParam;
  SIFTDescriptorParams RootSIFTParam;
  SIFTDescriptorParams HalfSIFTParam;
  SIFTDescriptorParams HalfRootSIFTParam;
  LIOPDescriptorParams LIOPParam;
  FREAKParams FREAKParam;
  BRISKParams BRISKParam;
  MROGHParams MROGHParam;
  CaffeDescriptorParams CaffeDescParam;
  BICEParams BICEParam;
  PIXELSDescriptorParams PixelsParam;
  KAZEParams KAZEParam;
  SURFParams SURFDescParam;
 // DALIParams DALIDescParam;
  SMSLDParams SMSLDDescParam;
  DAISYParams DAISYParam;
  SSIMParams SSIMParam;
};

#endif // DESCRIPTORS_PARAMETERS_HPP
