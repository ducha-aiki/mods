#ifndef STRUCTURES_HPP
#define STRUCTURES_HPP
#undef __STRICT_ANSI__

#include <vector>
#include <map>
#include <opencv2/core/core.hpp>


enum detection_mode_t {FIXED_TH,
                       RELATIVE_TH,
                       FIXED_REG_NUMBER,
                       RELATIVE_REG_NUMBER,
                       NOT_LESS_THAN_REGIONS};

enum detector_type {DET_HESSIAN = 0,
                    DET_DOG = 1,
                    DET_HARRIS = 2,
                    DET_MSER = 3,
                    DET_ORB = 4,
                    DET_FAST = 5,
                    DET_SURF = 6,
                    DET_STAR = 7,
                    DET_BRISK = 8,
                    DET_KAZE = 9,
                    DET_FOCI = 10,
                    DET_CAFFE = 11,
                    DET_READ = 12,
                    DET_WAVE = 13,
                    DET_WASH = 14,
                    DET_SFOP = 15,
                    DET_TILDE = 16,
                    DET_UNKNOWN = 1000};


const std::string _DetectorNames [] = {"HessianAffine", "DoG",
                                       "HarrisAffine", "MSER",
                                       "ORB", "FAST", "SURF",
                                       "STAR", "BRISK", "KAZE",
                                       "FOCI","CAFFE", "ReadAffs", "WAVE", "WASH", "SFOP", "TILDE"};


const std::vector<std::string> DetectorNames (_DetectorNames,_DetectorNames +
                                              sizeof(_DetectorNames)/sizeof(*_DetectorNames));

struct TimeLog
{
  double SynthTime;
  double DetectTime;
  double OrientTime;
  double DescTime;
  double MatchingTime;
  double RANSACTime;
  double MiscTime;
  double TotalTime;
  double SCVTime;
  TimeLog()
  {
    SynthTime=0.0;
    DetectTime=0.0;
    OrientTime=0.0;
    DescTime = 0.0;
    MatchingTime=0.0;
    RANSACTime=0.0;
    MiscTime=0.0;
    TotalTime = 0.0;
    SCVTime = 0.0;
  }
};

enum descriptor_type {DESC_SIFT = 0,
                      DESC_ROOT_SIFT = 1,
                      DESC_HALF_SIFT = 2,
                      DESC_HALF_ROOT_SIFT = 3,
                      DESC_INV_SIFT = 4,
                      DESC_ORB = 5,
                      DESC_FREAK = 6,
                      DESC_SURF = 7,
                      DESC_PIXELS = 8,
                      DESC_LIOP = 9,
                      DESC_BRISK = 10,
                      DESC_KAZE = 11,
                      DESC_MROGH = 12,
                      DESC_BICE = 13,
                      DESC_CAFFE = 14,
                      DESC_DALI = 15,
                      DESC_SMSLD = 16,
                      DESC_DAISY = 17,
                      DESC_SSIM = 18,
                      DESC_DSPSIFT = 19,
                      DESC_UNKNOWN = 1000};


const std::string _DescriptorNames [] = {"SIFT", "RootSIFT",
                                     "HalfSIFT", "HalfRootSIFT",
                                     "InvSIFT", "ORB", "FREAK",
                                      "SURF", "Pixels", "LIOP",
                                         "BRISK","KAZE", "MROGH","BICE"
                                        "CAFFE", "DALI", "SMSLD", "DAISY", "SSIM", "DSPSIFT"};

const std::vector<std::string> DescriptorNames (_DescriptorNames,_DescriptorNames +
                                              sizeof(_DescriptorNames)/sizeof(*_DescriptorNames));


/// Basic structures:

struct WLDParams
{
  double a; // WLD = a*DoG(px) / (I(px)/g + b) ;
  double b;
  double g;
  WLDParams()
  {
    a = 3.0;
    b = 5.0;
    g = 5.0;
  }
};


struct PyramidParams
{
  // shall input image be upscaled ( > 0)
  int upscaleInputImage;
  // number of scale per octave
  int  numberOfScales;
  // amount of smoothing applied to the initial level of first octave
  float initialSigma;
  // noise dependent threshold on the response (sensitivity)
  float threshold;
  float rel_threshold;
  int reg_number;
  float rel_reg_number;
  // ratio of the eigenvalues
  double edgeEigenValueRatio;
  // number of pixels ignored at the border of image
  int  border;
  int   doOnWLD; // detect Hessian points on WLD-transformed image
  int   doOnNormal; // detect Hessian points on normal image
  WLDParams WLDPar; //Parameters for WLD-transformation
  detection_mode_t DetectorMode;
  detector_type DetectorType;
  bool iiDoGMode;
  PyramidParams()
  {
    upscaleInputImage = 0;
    numberOfScales = 3;
    initialSigma = 1.6f;
    threshold = 16.0f/3.0f; //0.04f * 256 / 3;
    edgeEigenValueRatio = 10.0f;
    border = 5;
    doOnWLD = 0;
    doOnNormal = 1;
    DetectorMode = FIXED_TH;
    rel_threshold = -1;
    reg_number = -1;
    rel_reg_number = -1;
    DetectorType = DET_HESSIAN;
    iiDoGMode = false;
  }
};
struct Octave
{
  int    id;
  float  pixelDistance;
  float  initScale;

  std::vector<float> scales;
  std::vector<cv::Mat> blurs;
};

struct ScalePyramid
{
  PyramidParams par;
  ScalePyramid()
  {
  }
  std::vector<Octave> octaves;
};

struct SynthImage           // SynthImage: synthesised image from unwarped one
{
  int id;                 // image identifier
  std::string OrigImgName;   // filename of original image
  double tilt;            // tilt - scale factor in vertical direction. (y_synth=y_original / tilt)
  double rotation;        // angle of rotation, befote tilting. Counterclockwise, around top-left pixel, radians
  double zoom;            // scale factor, (x_synth,y_synth) = zoom*(x,y), before tilting and rotating
  double H[3*3];          // homography matrix from original image to synthesised
  cv::Mat pixels;         // image data
  cv::Mat rgb_pixels;      // image data

  ScalePyramid pyramid;
};

struct AffineKeypoint
{
  double x,y;            // subpixel, image coordinates
  double a11, a12, a21, a22;  // affine shape matrix
  double s;                   // scale
  double response;
  int octave_number;
  double pyramid_scale;
  int sub_type; //i.e. dark/bright for DoG
};

struct ViewSynthParameters
{
  double zoom;
  double tilt;
  double phi; //in radians
  double InitSigma;
  int doBlur;
  int DSPlevels;
  double minSigma;
  double maxSigma;
  std::vector<std::string> descriptors;
  std::map <std::string, double> FGINNThreshold;
  std::map <std::string, double> DistanceThreshold;
};

typedef std::map<std::string, std::vector<ViewSynthParameters> > IterationViewsynthesisParam;

struct Descriptor
{
  descriptor_type type;
  std::vector<float> vec;
};
struct AffineRegion{

  int img_id;              //image id, where shape detected
  int img_reproj_id;   //original untilted image id (always =zero)
  int id;                  //region id
  int parent_id;
  detector_type type;
  AffineKeypoint det_kp;   //affine region in detected image
  AffineKeypoint reproj_kp;//reprojected affine region to the original image
  Descriptor desc;

};

struct PatchExtractionParams {

  int patchSize;
  double mrSize;
  bool FastPatchExtraction;
  bool photoNorm;
  PatchExtractionParams() {
    mrSize = 5.1962;
    patchSize = 41;
    FastPatchExtraction = false;
    photoNorm = true;
  }
};


typedef std::vector<AffineRegion> AffineRegionVector;
typedef std::map <std::string, AffineRegionVector> AffineRegionVectorMap;
typedef std::vector<AffineRegion> AffineRegionList;

struct WhatToMatch
{
  std::vector<std::string> group_detectors;
  std::vector<std::string> group_descriptors;
  std::vector<std::string> separate_detectors;
  std::vector<std::string> separate_descriptors;
};

struct TILDEParams {
  bool approx;
  float scaleKeypoint;
  float orientationKeypoint;
  bool doBaumberg;
  std::string pathFilter;
  int maxPoints;
  bool keep_only_positive;
  TILDEParams() {
    approx = false;
    scaleKeypoint = 10.0;
    orientationKeypoint = 0;
    maxPoints = 500;
    doBaumberg = false;
    keep_only_positive = true;
  }
};

#endif // STRUCTURES_HPP
