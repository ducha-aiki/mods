#ifndef DETECTORS_PARAMETERS_HPP
#define DETECTORS_PARAMETERS_HPP

#include "structures.hpp"
#include "mser/extrema/extremaParams.h"
#include "affinedetectors/scale-space-detector.hpp"

struct WAVEParams{
  float b_wave;
  float r;
  bool pyramid;
  int s;
  int nms;
  int t;
  float k;
  bool doBaumberg;
  WAVEParams() {
    b_wave=0.166666;
    r=0.05;
    pyramid=true;
    s=12;
    nms=3;
    t=200;
    k = 0.16;
    doBaumberg = false;
  }
};

struct WASHParams{
  int threshold;
  bool doBaumberg;
  WASHParams() {
    threshold=100;
    doBaumberg = false;
  }
};

struct SFOPParams{
  float noise;
  int pThresh;
  float lWeight;
  int nOctaves;
  int nLayers;
  bool doBaumberg;
  SFOPParams() {
    noise=0.02;
    pThresh=0;
    lWeight=2;
    nOctaves=3;
    nLayers=4;
    doBaumberg = false;
  }
};



struct FOCIParams{
  int numberKPs;
  bool computeOrientation;
  bool secondOrientation;
  bool doBaumberg;
  FOCIParams() {
    numberKPs = 0;
    computeOrientation =true;
    secondOrientation = false;
    doBaumberg = false;
  }
};
struct SURFParams
{
  int octaves;
  int intervals;
  int init_sample;
  float thresh;
  bool doBaumberg;

  //  int patchSize;
  //  double mrSize;
  //  bool FastPatchExtraction;
  PatchExtractionParams PEParam;
  SURFParams()
  {
    octaves = 4;
    intervals = 4;
    init_sample=2;
    thresh =0.0004;
    doBaumberg = false;
    //   patchSize = 41;
    //    mrSize =  3.0*sqrt(3.0);
    //    FastPatchExtraction = false;
  }
};
struct FASTParams
{
  float threshold;
  bool nonmaxSuppression;
  int type;
  bool doBaumberg;
  FASTParams()
  {
    doBaumberg = false;
    threshold=10.0;
    nonmaxSuppression=true;
    type=0;
  }
};
struct STARParams
{
  int maxSize;
  int responseThreshold;
  int lineThresholdProjected;
  int lineThresholdBinarized;
  int suppressNonmaxSize;
  bool doBaumberg;
  STARParams()
  {
    doBaumberg = false;
    maxSize=45;
    responseThreshold=30;
    lineThresholdProjected=10;
    lineThresholdBinarized=8;
    suppressNonmaxSize=5;
  }
};
struct BRISKParams
{
  int thresh;
  int octaves;
  float patternScale;
  PatchExtractionParams PEParam;
  bool doBaumberg;
  //  int patchSize;
  //  double mrSize;
  //  bool FastPatchExtraction;
  BRISKParams()
  {
    doBaumberg = false;
    thresh=30;
    octaves=3;
    patternScale=1.0f;
    //   patchSize=41;
    //    mrSize = 3.0*sqrt(3.0);
    //    FastPatchExtraction = false;
  }
};
struct ReadAffsFromFileParams {
  std::string fname;
  ReadAffsFromFileParams() {
    fname="";
  }
};
struct ORBParams
{
  int nfeatures;
  float scaleFactor;
  int nlevels;
  int edgeThreshold;
  int firstLevel;
  int WTA_K;
  PatchExtractionParams PEParam;
  bool doBaumberg;
  //  int patchSize;
  //  double mrSize;
  //  bool FastPatchExtraction;
  //  bool photoNorm;
  ORBParams()
  {
    doBaumberg = false;
    nfeatures = 500;
    scaleFactor = 1.2;
    nlevels = 8;
    edgeThreshold = 31;
    firstLevel = 0;
    WTA_K=2;
    //    patchSize=31;
    //    mrSize = 3.0*sqrt(3.0);
    //    FastPatchExtraction = false;
    //    photoNorm =false;
  }
};

struct DetectorsParameters
{
  extrema::ExtremaParams MSERParam;
  ScaleSpaceDetectorParams HessParam;
  ScaleSpaceDetectorParams HarrParam;
  ScaleSpaceDetectorParams DoGParam;
  ScaleSpaceDetectorParams TILDEScaleSpaceParam;
  SURFParams SURFParam;
  FASTParams FASTParam;
  STARParams STARParam;
  BRISKParams BRISKParam;
  ORBParams ORBParam;
  FOCIParams FOCIParam;
  ReadAffsFromFileParams ReadAffsFromFileParam;
  SFOPParams SFOPParam;
  WASHParams WASHParam;
  WAVEParams WAVEParam;

  AffineShapeParams BaumbergParam;
};


#endif // DETECTORS_PARAMETERS_HPP
