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
#include "descriptors/orbdescriptor.hpp"
#include "descriptors/briskdescriptor.hpp"
#include "matching/liopdesc.hpp"

struct DominantOrientationParams {

  int maxAngles;
  float threshold;
  bool addUpRight;
  bool halfSIFTMode;
  bool useCNN;
  std::string  external_command;
  PatchExtractionParams PEParam;
  DominantOrientationParams() {
    maxAngles = -1;
    threshold = 0.8;
    addUpRight = false;
    halfSIFTMode = false;
    useCNN = false;
    external_command = "";
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
};

#endif // DESCRIPTORS_PARAMETERS_HPP
