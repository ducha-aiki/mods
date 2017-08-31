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
                << "Usage: " << argv[0] << " imgIn1.png desc1.txt config_iter.ini iters.ini" << std::endl
                << "- imgIn1.png: input images " << std::endl
                << "- keys1.txt: affine regions and their descriptors of the two images." << std::endl
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
  if (getCLIparamExportDescriptorsBenchmark(Config1,argc,argv)) return 1;
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

      cv::Mat gray_in_img;
      if (img1.channels() == 3)
        {
          cv::Mat gray_img1;
          //cv::cvtColor(img1, gray_img1, CV_BGR2GRAY);
          std::vector<cv::Mat> RGB_planes(3);
          cv::Mat in_32f;
          img1.convertTo(in_32f,CV_32FC3);
          cv::split(in_32f, RGB_planes);
          // gray_img1 = cv::Mat::zeros(img1.cols, img1.rows,CV_32FC1);
          gray_img1 = (RGB_planes[0] + RGB_planes[1] + RGB_planes[2]) / 3.0 ;
          gray_img1.convertTo(gray_in_img,CV_8UC1);
        } else {
          gray_in_img = img1;
        }

      clahe->apply(gray_in_img,img1_clahe);
      double time2 = ((double)(getMilliSecs() - clahe_start))/1000;
      if (VERB) std::cerr << " CLAHE done in "  << time2<< " seconds" << endl;
      ImgRep1 = ImageRepresentation(img1_clahe,Config1.CLIparams.img1_fname);
    }
  else
    {
      ImgRep1 = ImageRepresentation(img1,Config1.CLIparams.img1_fname);
    }
#ifdef WITH_CAFFE
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
   std::shared_ptr<caffe::Net<float> > caffe_net;
   std::cerr << Config1.DescriptorPars.CaffeDescParam.ProtoTxt << std::endl;
  caffe_net.reset(new caffe::Net<float>(Config1.DescriptorPars.CaffeDescParam.ProtoTxt, caffe::TEST));
  caffe_net->CopyTrainedLayersFrom(Config1.DescriptorPars.CaffeDescParam.WeightsFile);

  ImgRep1.InitCaffe(caffe_net);
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
  ImgRep1.SaveDescriptorsBenchmark(Config1.CLIparams.k1_fname);
  return 0;
}


