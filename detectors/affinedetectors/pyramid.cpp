/*
 * Copyright (C) 2008-12 Michal Perdoch
 * All rights reserved.
 *
 * This file is part of the HessianAffine detector and is made available under
 * the terms of the BSD license (see the COPYING file).
 *
 */
#undef __STRICT_ANSI__
#include <vector>
#include <algorithm>
#include "pyramid.h"
#include "../helpers.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <assert.h>
#include "../../TILDE/c++/src/libTILDE.hpp"
//#include <iostream>

#ifdef _MSC_VER
#define isnan _isnan
#endif
//
#include <iostream>
//
#include <sys/time.h>
inline long getMilliSecs1()
{
  timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec*1000 + t.tv_usec/1000;
}
// it seems 0.6 works better than 0.5 (as in DL paper)
#define MAX_SUBPIXEL_SHIFT 0.6

// we don't care about border effects
#define POINT_SAFETY_BORDER  3
bool responseCompare(AffineKeypoint k1,AffineKeypoint k2) {return (fabs(k1.response) < fabs(k2.response));}
bool responseCompareInvOrder(AffineKeypoint k1,AffineKeypoint k2) {return (fabs(k1.response) > fabs(k2.response));}

const float INT_NORM_EPS = 1e-10;
inline float intensityNormCoef (const float intensity, const float a, const float b, const float g_inv)
{
  return (a / (intensity*g_inv+b+INT_NORM_EPS));
}

using namespace std;

bool isMax(float val, const Mat &pix, int row, int col)
{
  for (int r = row - 1; r <= row + 1; r++)
    {
      const float *row = pix.ptr<float>(r);
      for (int c = col - 1; c <= col + 1; c++)
        if (row[c] > val)
          return false;
    }
  return true;
}

bool isMin(float val, const Mat &pix, int row, int col)
{
  for (int r = row - 1; r <= row + 1; r++)
    {
      const float *row = pix.ptr<float>(r);
      for (int c = col - 1; c <= col + 1; c++)
        if (row[c] < val)
          return false;
    }
  return true;
}

int getPointType(float *ptr, float value, detector_type det_type)
{
  switch (det_type)
    {
    case DET_HESSIAN:
      {
        /* find blob point type from Hessian matrix H,
         we know that:
         - if H is positive definite it is a DARK blob,
         - if H is negative definite it is a BRIGHT blob
         - det H is negative it is a SADDLE point
      */
        if (value < 0)
          return ScaleSpaceDetector::HESSIAN_SADDLE;
        else
          {
            // at this point we know that 2x2 determinant is positive
            // so only check the remaining 1x1 subdeterminant
            float Lxx = (ptr[-1]-2*ptr[0]+ptr[1]);
            if (Lxx < 0)
              return ScaleSpaceDetector::HESSIAN_DARK;
            else
              return ScaleSpaceDetector::HESSIAN_BRIGHT;
          }
        break;
      }
    case DET_DOG:
      {
        if (value < 0)
          return ScaleSpaceDetector::DOG_BRIGHT;
        else
          return ScaleSpaceDetector::DOG_DARK;

        break;
      }
    case DET_HARRIS:
      {
        if (value < 0)
          return ScaleSpaceDetector::HARRIS_BRIGHT;
        else
          return ScaleSpaceDetector::HARRIS_DARK;
        break;
      }
    case DET_TILDE:
      {
         return ScaleSpaceDetector::TILDE;
        break;
      }
    default:
      { //Hessian
        if (value < 0)
          return ScaleSpaceDetector::HESSIAN_SADDLE;
        else
          {
            float Lxx = (ptr[-1]-2*ptr[0]+ptr[1]);
            if (Lxx < 0)
              return ScaleSpaceDetector::HESSIAN_DARK;
            else
              return ScaleSpaceDetector::HESSIAN_BRIGHT;
          }
        break;
      }
    }

}

Mat ScaleSpaceDetector::Response(const Mat &inputImage, float norm)
{
  switch (Pyrpar.DetectorType)
    {
    case DET_HESSIAN:
      {
        if (Pyrpar.iiDoGMode) {
            //          return iiHessianResponse(inputImage, norm);
          } else {
            return HessianResponse(inputImage, norm);
          }
        break;
      }
    case DET_DOG:
      {
        if (Pyrpar.iiDoGMode) {
            return iidogResponse(inputImage, norm);
          } else {
            return dogResponse(inputImage, norm);
          }
        break;
      }
    case DET_HARRIS:
      {
        if (Pyrpar.iiDoGMode) {
            //           return iiHarrisResponse(inputImage, norm);

          } else {
            return HarrisResponse(inputImage, norm);

          }
        break;
      }
    case DET_TILDE:
      {
        return TILDEResponse(inputImage, norm);
        break;
      }
    default:
      {
        return HessianResponse(inputImage, norm);
      }
    }
}
Mat ScaleSpaceDetector::dogResponse(const Mat &inputImage, float norm)
{
  Mat nextBlur = gaussianBlur(inputImage, norm);
  Mat outputImage = inputImage - nextBlur;
  return outputImage;
}
Mat ScaleSpaceDetector::TILDEResponse(const Mat &inputImage, float norm)
{
//  cv::Mat out_resp =
//  int rnd1 = (int) getMilliSecs1();
//  std::string img_fname = "tilde"+std::to_string(norm+rnd1)+".png";

//  double minVal, maxVal;
//                  cv::minMaxLoc(out_resp, &minVal, &maxVal);
//                  double range = maxVal;
//                  out_resp = (out_resp) / range;

//  cv::imwrite(img_fname,255*out_resp);

  return getTILDEResponce(inputImage, filters, false, tilde_only_positive);

}

Mat ScaleSpaceDetector::iidogResponse(const Mat &inputImage, float norm)
{
  const int cols = inputImage.cols;
  const int rows = inputImage.rows;
  Mat nextBlur = gaussianBlur(inputImage, norm);
  Mat DoGResponse = inputImage - nextBlur;

  Mat Sum1 = inputImage + nextBlur;
  Mat outputImage = DoGResponse;

  for (int r = 0; r < rows; r++)
    {
      float *outPtr = outputImage.ptr<float>(r);
      const float *SumPtr =Sum1.ptr<float>(r);
      for (int c = 0; c < cols; c++)
        {
          if (*SumPtr < 255.)
            *outPtr *= (255. / *SumPtr);
          outPtr++; SumPtr++;
        }
    }
  return outputImage;
}

Mat ScaleSpaceDetector::HessianResponse(const Mat &inputImage, float norm)
{
  const int rows = inputImage.rows;
  const int cols = inputImage.cols;
  const int stride = cols;

  // allocate output
  Mat outputImage(rows, cols, CV_32FC1);

  // setup input and output pointer to be centered at 1,0 and 1,1 resp.
  const float *in = inputImage.ptr<float>(1);
  float      *out = outputImage.ptr<float>(1) + 1;

  float norm2 = norm * norm;

  /* move 3x3 window and convolve */
  for (int r = 1; r < rows - 1; ++r)
    {
      float v11, v12, v21, v22, v31, v32;
      /* fill in shift registers at the beginning of the row */
      v11 = in[-stride];
      v12 = in[1 - stride];
      v21 = in[      0];
      v22 = in[1         ];
      v31 = in[+stride];
      v32 = in[1 + stride];
      /* move input pointer to (1,2) of the 3x3 square */
      in += 2;
      for (int c = 1; c < cols - 1; ++c)
        {
          /* fetch remaining values (last column) */
          const float v13 = in[-stride];
          const float v23 = *in;
          const float v33 = in[+stride];

          // compute 3x3 Hessian values from symmetric differences.
          float Lxx = (v21 - 2*v22 + v23);
          float Lyy = (v12 - 2*v22 + v32);
          float Lxy = (v13 - v11 + v31 - v33)/4.0f;

          /* normalize and write out */
          *out = (Lxx * Lyy - Lxy * Lxy)*norm2;

          /* move window */
          v11=v12;
          v12=v13;
          v21=v22;
          v22=v23;
          v31=v32;
          v32=v33;

          /* move input/output pointers */
          in++;
          out++;
        }
      out += 2;
    }
  return outputImage;
}

Mat ScaleSpaceDetector::HarrisResponse(const Mat &inputImage, float norm)
{
  const int rows = inputImage.rows;
  const int cols = inputImage.cols;
  float sigmasq = 0.6*norm;
  float sigma = sqrt(sigmasq);
  // allocate output
  Mat outputImage(rows, cols, CV_32FC1);

  Mat Lx(rows, cols, CV_32FC1);
  Mat Ly(rows, cols, CV_32FC1);
  computeGradient(inputImage,Lx,Ly);

  Mat dx2,dy2,dxdy;
  dx2 = sigmasq*gaussianBlur(Lx.mul(Lx),sigma);
  dy2 = sigmasq*gaussianBlur(Ly.mul(Ly),sigma);
  dxdy = sigmasq*gaussianBlur(Lx.mul(Ly),sigma);

  Mat dx2dy2_sum  = dx2 + dy2;
  outputImage = dx2.mul(dy2) - dxdy.mul(dxdy) - 0.04*dx2dy2_sum.mul(dx2dy2_sum);

  return outputImage;
}


void ScaleSpaceDetector::localizeKeypoint(int r, int c, float curScale, float pixelDistance)
{
  const int cols = cur.cols;
  const int rows = cur.rows;

  float b[3] = {};
  float val = 0;
  // bool converged = false;
  int nr = r, nc = c;
  //

  for (int iter=0; iter<5; iter++)
    {
      // take current position
      r = nr;
      c = nc;

      // preparing data
      const float *cur0Ptr = cur.ptr<float>(r-1);
      const float *cur1Ptr = cur.ptr<float>(r);
      const float *cur2Ptr = cur.ptr<float>(r+1);

      const float *low0Ptr = low.ptr<float>(r-1);
      const float *low1Ptr = low.ptr<float>(r);
      const float *low2Ptr = low.ptr<float>(r+1);

      const float *high0Ptr = high.ptr<float>(r-1);
      const float *high1Ptr = high.ptr<float>(r);
      const float *high2Ptr = high.ptr<float>(r+1);
      //
      float dxx = cur1Ptr[c-1] - 2.0f * cur1Ptr[c] + cur1Ptr[c+1];
      float dyy = cur0Ptr[c]   - 2.0f * cur1Ptr[c] + cur2Ptr[c];
      float dss = low1Ptr[c]   - 2.0f * cur1Ptr[c] + high1Ptr[c];

      float dxy = 0.25f*(cur2Ptr[c+1] - cur2Ptr[c-1] - cur0Ptr[c+1] + cur0Ptr[c-1]);
      // check edge like shape of the response function in first iteration
      if (0 == iter)
        {
          float edgeScore = (dxx + dyy)*(dxx + dyy)/(dxx * dyy - dxy * dxy);
          if (edgeScore >= edgeScoreThreshold || edgeScore < 0)
            // local neighbourhood looks like an edge
            return;
        }
      float dxs = 0.25f*(high1Ptr[c+1] - high1Ptr[c-1] - low1Ptr[c+1] + low1Ptr[c-1]);
      float dys = 0.25f*(high2Ptr[c]   - high0Ptr[c]   - low2Ptr[c]   + low0Ptr[c]  );

      float A[9];
      A[0] = dxx;
      A[1] = dxy;
      A[2] = dxs;
      A[3] = dxy;
      A[4] = dyy;
      A[5] = dys;
      A[6] = dxs;
      A[7] = dys;
      A[8] = dss;

      float dx = 0.5f*(cur1Ptr[c+1] - cur1Ptr[c-1]);
      float dy = 0.5f*(cur2Ptr[c]   - cur0Ptr[c]);
      float ds = 0.5f*(high1Ptr[c]  - low1Ptr[c]);

      b[0] = - dx;
      b[1] = - dy;
      b[2] = - ds;

      solveLinear3x3(A, b);

      // check if the solution is valid
      if (isnan(b[0]) || isnan(b[1]) || isnan(b[2]))
        return;

      // aproximate peak value
      val = cur1Ptr[c] + 0.5f * (dx*b[0] + dy*b[1] + ds*b[2]);

      // if we are off by more than MAX_SUBPIXEL_SHIFT, update the position and iterate again
      if (b[0] >  MAX_SUBPIXEL_SHIFT)
        {
          if (c < cols - POINT_SAFETY_BORDER) nc++;
          else return;
        }
      if (b[1] >  MAX_SUBPIXEL_SHIFT)
        {
          if (r < rows - POINT_SAFETY_BORDER) nr++;
          else return;
        }
      if (b[0] < -MAX_SUBPIXEL_SHIFT)
        {
          if (c >        POINT_SAFETY_BORDER) nc--;
          else return;
        }
      if (b[1] < -MAX_SUBPIXEL_SHIFT)
        {
          if (r >        POINT_SAFETY_BORDER) nr--;
          else return;
        }

      if (nr == r && nc == c)
        {
          // converged, displacement is sufficiently small, terminate here
          // TODO: decide if we want only converged local extrema...
          //   converged = true;
          break;
        }
    }

  // if spatial localization was all right and the scale is close enough...
  if (fabs(b[0]) > 1.5 || fabs(b[1]) > 1.5 || fabs(b[2]) > 1.5 || fabs(val) < finalThreshold || octaveMap.at<unsigned char>(r,c) > 0)
    return;

  // mark we were here already
  octaveMap.at<unsigned char>(r,c) = 1;

  // output keypoint
  float scale = curScale * pow(2.0f, b[2] / Pyrpar.numberOfScales );

  // set point type according to final location
  int type = getPointType(blur.ptr<float>(r)+c, val, Pyrpar.DetectorType);

  // point is now scale and translation invariant, add it...
  localized_points++;
  if (keypointCallback)
    keypointCallback->onKeypointDetected(prevBlur, pixelDistance*(c + b[0]), pixelDistance*(r + b[1]), pixelDistance*scale, pixelDistance, type, val);
}

void ScaleSpaceDetector::findLevelKeypoints(float curScale, float pixelDistance)
{
  assert(Pyrpar.border >= 2);
  const int rows = cur.rows;
  const int cols = cur.cols;

  for (int r = Pyrpar.border; r < (rows - Pyrpar.border); r++)
    {
      const float* curPtr = cur.ptr<float>(r);
      for (int c = Pyrpar.border; c < (cols - Pyrpar.border); c++)
        {
          const float val = curPtr[c];
          if ( (val > positiveThreshold && (isMax(val, cur, r, c) && isMax(val, low, r, c) && isMax(val, high, r, c))) ||
               (val < negativeThreshold && (isMin(val, cur, r, c) && isMin(val, low, r, c) && isMin(val, high, r, c))) )
            //      if ( (val > positiveThreshold && (isMax(val, cur, r, c) && isMax(val, low, r, c) && isMax(val, high, r, c))) ||
            //           (val < negativeThreshold && (isMin(val, cur, r, c) && isMin(val, low, r, c) && isMin(val, high, r, c))) )
            // either positive -> local max. or negative -> local min.
            localizeKeypoint(r, c, curScale, pixelDistance);
        }
    }
}


void ScaleSpaceDetector::detectOctaveKeypoints(const Mat &firstLevel, float pixelDistance, Mat &nextOctaveFirstLevel)
{

  octaveMap = Mat::zeros(firstLevel.rows, firstLevel.cols, CV_8UC1);
  float sigmaStep = pow(2.0f, 1.0f / (float) Pyrpar.numberOfScales);
  float curSigma = Pyrpar.initialSigma;
  int numLevels = 1;
  double a=Pyrpar.WLDPar.a;
  double b=Pyrpar.WLDPar.b;
  double g_inv=1.0/Pyrpar.WLDPar.g;
  blur = firstLevel;
  int nRows = blur.rows;
  int nCols = blur.cols;
  Octave currentOctave;
  currentOctave.id = floor(pixelDistance);
  currentOctave.pixelDistance = pixelDistance;
  currentOctave.initScale = curSigma;
  currentOctave.blurs.reserve(Pyrpar.numberOfScales+2);
  currentOctave.scales.reserve(Pyrpar.numberOfScales+2);

  cur = Response(blur, curSigma*curSigma);

  currentOctave.scales.push_back(curSigma);
  currentOctave.blurs.push_back(blur);

  for (int i = 1; i < Pyrpar.numberOfScales+2; i++)
    {
      // compute the increase necessary for the next level and compute the next level
      float sigma = curSigma * sqrt(sigmaStep * sigmaStep - 1.0f);
      // do the blurring
      Mat nextBlur = gaussianBlur(blur, sigma);
      // the next level sigma
      sigma = curSigma*sigmaStep;
      // compute response for current level

      high = Response(nextBlur, sigma*sigma);

      if (Pyrpar.doOnWLD)
        {

          for(int rr = 0; rr < nRows; ++rr)
            {
              float *highPtr = high.ptr<float>(rr);
              const float *intensityPtr = nextBlur.ptr<float>(rr);
              for (int j = 0; j < nCols; ++j)
                {
                  const float norm_coef = intensityNormCoef(intensityPtr[j],a,b,g_inv);
                  highPtr[j] *= norm_coef*norm_coef;
                }
            }
        }

      numLevels ++;
      // if we have three consecutive responses
      if (numLevels == 3)
        {
          // find keypoints in this part of octave for curLevel

          findLevelKeypoints(curSigma, pixelDistance);
          numLevels--;
        }

      if (i == Pyrpar.numberOfScales)
        // downsample the right level for the next octave
        //nextOctaveFirstLevel = halfImage(nextBlur);
         cv::resize(nextBlur,nextOctaveFirstLevel, cv::Size(0,0), 0.5,0.5, cv::INTER_LINEAR );

//        nextOctaveFirstLevel = halfImage(nextBlur);

      prevBlur = blur;
      blur = nextBlur;

      // shift to the next response
      low = cur;
      cur = high;
      curSigma *= sigmaStep;

      currentOctave.scales.push_back(curSigma);
      currentOctave.blurs.push_back(blur);
    }

  scale_pyramid.octaves.push_back(currentOctave);

}

void ScaleSpaceDetector::detectPyramidKeypoints(const Mat &image)
{
  scale_pyramid.par = Pyrpar;

  float curSigma = 0.5f;
  float pixelDistance = 1.0f;
  Mat firstLevel;
  originalImg = &image;
  if (Pyrpar.upscaleInputImage > 0)
    {
      firstLevel = doubleImage(image);
      pixelDistance *= 0.5f;
      curSigma *= 2.0f;
    }
  else
    firstLevel = image.clone();

  // prepare first octave input image
  if (Pyrpar.initialSigma > curSigma)
    {
      float sigma = sqrt(Pyrpar.initialSigma * Pyrpar.initialSigma - curSigma * curSigma);
      gaussianBlurInplace(firstLevel, sigma);
    }
  // while there is sufficient size of image
  int minSize = 2 * Pyrpar.border + 2;
  while (firstLevel.rows > minSize && firstLevel.cols > minSize)
    {
      Mat nextOctaveFirstLevel;
      detectOctaveKeypoints(firstLevel, pixelDistance, nextOctaveFirstLevel);
      pixelDistance *= 2.0;
      // firstLevel gets destroyed in the process
      firstLevel = nextOctaveFirstLevel;
    }
}
