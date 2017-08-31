/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/

#include "scale-space-detector.hpp"


//using namespace cv;
using cv::Mat;
using namespace std;


int DetectAffineKeypoints(cv::Mat &input, vector<AffineKeypoint> &out1,
                          ScaleSpaceDetectorParams params,
                          ScalePyramid &scale_pyramid,
                          const double tilt, const double zoom)
{
  PyramidParams p1 = params.PyramidPars;
  AffineShapeParams ap = params.AffineShapePars;
  if ((tilt > 2.0) || (zoom < 0.5))
    p1.reg_number = (int)floor(zoom*(double)p1.reg_number/tilt);


  // Detect keypoints on normal image
  if (params.PyramidPars.doOnNormal)
    {
      p1.doOnWLD = 0;
      p1.doOnNormal = params.PyramidPars.doOnNormal;
      AffineDetector detector(input, p1, ap);
      detector.detectPyramidKeypoints(input);
      detector.exportKeypoints(out1);
      detector.exportScaleSpace(scale_pyramid);
    }

 // scale_pyramid = temp_pyr;
  return out1.size();
}

