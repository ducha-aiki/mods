/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/

#undef __STRICT_ANSI__
#include <fstream>
#include <string>
#include <iomanip>
#include <sys/time.h>
#include <map>

#include "io_mods.h"

#include "detectors/mser/extrema/extrema.h"
#include "detectors/helpers.h"
#include "matching/siftdesc.h"
#include "synth-detection.hpp"

#include "detectors/affinedetectors/scale-space-detector.hpp"
#include "detectors/detectors_parameters.hpp"
#include "descriptors_parameters.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "matching.hpp"

#include "configuration.hpp"
#include "imagerepresentation.h"
#include "correspondencebank.h"


//#define SCV

#ifdef SCV
#include "scv/scv_entrypoint.hpp"
#endif

#ifdef WITH_ORSA
#include "orsa.h"
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

inline static bool endsWith(const std::string& str, const std::string& suffix)
{
  return str.size() >= suffix.size() && 0 == str.compare(str.size()-suffix.size(), suffix.size(), suffix);
}

using namespace std;

const int nn_n = 50; //number of nearest neighbours retrieved to get 1st inconsistent

//inline long getMilliSecs()
//{
//  timeval t;
//  gettimeofday(&t, NULL);
//  return t.tv_sec*1000 + t.tv_usec/1000;
//}

int main(int argc, char **argv)
{
  if ((argc < 4))
  {
    std::cerr << " ************************************************************************** " << std::endl
    << " ******** Two-view Matching with On-Demand Synthesis ********************** " << std::endl
    << " ************************************************************************** " << std::endl
    << "Usage: " << argv[0] << " imgIn1.png keys-1.txt config_iter.ini iters.ini" << std::endl
    << "- imgIn1.png: input image " << std::endl
    << "- keys1.txt: affine regions and their descriptors." << std::endl
    << "- config_iter.ini: input file with detectors and descriptors paramaters [optional, default = 'config_iter.ini'] " << std::endl
    << "- iters.ini: input file with parameters of iterative view synthesis [optional, default= 'iters.ini']" << std::endl
    << " ******************************************************************************* " << std::endl;
    return 1;
  }
  long c_start = getMilliSecs();
  double time1;
  TimeLog TimingLog;
  logs log1;
  /// Parameters reading
  configs Config1;
  if (getCLIparamExtractFeatures(Config1,argc,argv)) return 1;
  int VERB = Config1.OutputParam.verbose;
  /// Input images reading
  cv::Mat img1;
  SynthImage tilt_img1;
  tilt_img1.id=0;
  img1 = cv::imread(Config1.CLIparams.img1_fname,Config1.LoadColor); // load grayscale; Try RGB?
  if(!img1.data) {
    std::cerr <<  "Could not open or find the image1 " << Config1.CLIparams.img1_fname << std::endl;
    return 1;
  }
  /// Data structures preparation
  ImageRepresentation ImgRep1;
  if (Config1.CLIparams.doCLAHE)
  {
    long clahe_start = getMilliSecs();

    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(4);

    cv::Mat img1_clahe;
    clahe->apply(img1,img1_clahe);
    double time2 = ((double)(getMilliSecs() - clahe_start))/1000;
    if (VERB) std::cerr << " CLAHE done in "  << time2<< " seconds" << endl;
    ImgRep1 = ImageRepresentation(img1_clahe,Config1.CLIparams.img1_fname);
  }
  else
  {
    ImgRep1 = ImageRepresentation(img1,Config1.CLIparams.img1_fname);
  }
#ifdef WITH_CAFFE
  caffe::Caffe::set_phase(caffe::Caffe::TEST);
  caffe::Caffe::set_mode(caffe::Caffe::CPU);
  caffe::Net<float> caffe_net(Config1.DescriptorPars.CaffeDescParam.ProtoTxt);
  caffe_net.CopyTrainedLayersFrom(Config1.DescriptorPars.CaffeDescParam.WeightsFile);

  ImgRep1.InitCaffe(&caffe_net);
#endif

  /// Affine regions detection
  std::cerr << "View synthesis, detection and description..." << endl;
#ifdef _OPENMP
  omp_set_nested(1);
#endif
  ImgRep1.SynthDetectDescribeKeypoints(Config1.ItersParam[0],
                                       Config1.DetectorsPars,
                                       Config1.DescriptorPars,
                                       Config1.DomOriPars);

  TimeLog img1time = ImgRep1.GetTimeSpent();
  /// Writing images and logs
  std::cerr << "Writing files... " << endl;

//  ImgRep1.SaveRegions(Config1.CLIparams.k1_fname,0);
 // ImgRep1.SaveRegions(Config1.CLIparams.k1_fname,0);
   if (endsWith(Config1.CLIparams.k1_fname,".npz")){
   
       ImgRep1.SaveRegionsNPZ(Config1.CLIparams.k1_fname);
     
   } else {
        
  ImgRep1.SaveRegions(Config1.CLIparams.k1_fname,0);

   
     } 
 return 0;
}


