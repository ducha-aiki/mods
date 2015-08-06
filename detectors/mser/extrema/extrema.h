#ifndef __EXTREMA_H__
#define __EXTREMA_H__
#undef __STRICT_ANSI__
#include "../../structures.hpp"
#include "extremaParams.h"
#include <opencv2/core/core.hpp>

int DetectMSERs(cv::Mat &input, std::vector<AffineKeypoint> &out1, extrema::ExtremaParams params, const double tilt = 1.0, const double zoom = 1.0);
//Entry point

int DetectMSERs(cv::Mat &input, std::vector<AffineKeypoint> &out1, extrema::ExtremaParams params, ScalePyramid &scale_pyramid, const double tilt = 1.0, const double zoom = 1.0);

#endif //__EXTREMA_H__
