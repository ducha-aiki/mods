/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/

#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP

#include <vector>
#include <string>

#define WITH_ORSA


//verificator types
#ifdef WITH_ORSA
enum RANSAC_mode_t {LORANSAC,GR_TRUTH,LORANSACF,ORSA,GR_PLUS_RANSAC};
#else
enum RANSAC_mode_t {LORANSAC,GR_TRUTH,LORANSACF,GR_PLUS_RANSAC};
#endif

//detector types
const int HESAFF = 0;
const int DOG = 1;
const int MSER = 2;

const int MSER_TYPES = 2;
const int HA_TYPES = 3;
const int DOG_TYPES = 2;
const int HAR_TYPES = 2;

const int MODE_RANDOM = 0;
const int MODE_FGINN = 1;
const int MODE_DISTANCE = 2;
const int MODE_BIGGER_REGION = 3;





struct drawingParams
{
  int writeImages;
  int drawEpipolarLines;
  int drawOnlyCenters;
  int drawReprojected;
  bool drawDetectedRegions;
  drawingParams()
  {
    writeImages = 1;
    drawOnlyCenters = 1;
    drawEpipolarLines = 0;
    drawReprojected = 1;
    drawDetectedRegions = false;
  }
};

struct outputParams
{
  int verbose;
  int timeLog;
  int writeKeypoints;
  int writeMatches;
  int featureComplemetaryLog;
  int outputAllTentatives;
  int outputEstimatedHorF;
  bool outputMikFormat;
  outputParams()
  {
    verbose = 0;
    timeLog = 1;
    writeKeypoints = 1;
    writeMatches = 1;
    outputAllTentatives = 0;
    featureComplemetaryLog = 0;
    outputEstimatedHorF = 0;
    outputMikFormat = false;
  }
};

struct filteringParams
{
  int useSCV;
  int doBeforeRANSAC;
  double duplicateDist;
  int mode;
  filteringParams()
  {
    useSCV = 0;
    doBeforeRANSAC = 1;
    duplicateDist = 3.0;
    mode = MODE_RANDOM;
  }
};

struct parameters
{
  std::string img1_fname;
  std::string img2_fname;
  std::string out1_fname;
  std::string out2_fname;
  std::string k1_fname;
  std::string k2_fname;
  std::string matchings_fname;
  std::string log_fname;
  std::string ground_truth_fname;
  std::string config_fname;
  std::string iters_fname;
  int doCLAHE;
  int det_type;
  RANSAC_mode_t ver_type;
  int tilt_numb;
  int rot_numb;
  double phi;
  double zoom;
  double initSigma;
  char doBlur;
  std::vector <double> tilt_set;
  std::vector <double> scale_set;
  int logOnly;
  parameters()
  {
    config_fname="config_iter.ini";
    iters_fname="iters.ini";
    det_type = HESAFF;
    ver_type = LORANSAC;
    tilt_numb = 2;
    phi = 72.;
    rot_numb = 1;
    zoom = 1.0;
    initSigma = 0.5;
    doBlur = 1;
    logOnly = 1;
    doCLAHE = 0;
    //  overlap_error = 0.04;
    tilt_set.push_back(1.0);
    scale_set.push_back(1.0);
  }
};
struct logs
{
  int TrueMatch;
  int TrueMatch1st;
  int TrueMatch1stRANSAC;

  int Tentatives;
  int Tentatives1st;
  int Tentatives1stRANSAC;

  double InlierRatio1st;
  double InlierRatio1stRANSAC;

  int OtherTrueMatch;
  int OtherTrueMatch1st;
  int OtherTrueMatch1stRANSAC;

  double OtherInlierRatio1st;
  double OtherInlierRatio1stRANSAC;

  int OtherTentatives;
  int OtherTentatives1st;
  int OtherTentatives1stRANSAC;

  int OrientReg1;
  int OrientReg2;

  int UnorientedReg1;
  int UnorientedReg2;
  double TotalArea;
  int Syms;
  double FinalTime;
  int OverlapMatches;
  int FinalStep;
  RANSAC_mode_t VerifMode;

  double densificationCoef;
  logs()
  {
    TrueMatch = 0;
    TrueMatch1st = 0;
    TrueMatch1stRANSAC = 0;

    Tentatives = 0;
    Tentatives1st = 0;
    Tentatives1stRANSAC = 0;

    OtherTrueMatch = 0;
    OtherTrueMatch1st = 0;
    OtherTrueMatch1stRANSAC = 0;

    OtherTentatives = 0;
    OtherTentatives1st = 0;
    OtherTentatives1stRANSAC = 0;

    OrientReg1 = 0;
    OrientReg2 = 0;
    UnorientedReg1 = 0;
    UnorientedReg2 = 0;
    Syms = 0;
    FinalTime = 0;
    OverlapMatches = 0;
    TotalArea = 1;
    FinalStep=1;
    densificationCoef = 1.0;
  }
};
#endif // CONFIGURATION_HPP
