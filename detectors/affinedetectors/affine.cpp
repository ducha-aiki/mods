/*
 * Copyright (C) 2008-12 Michal Perdoch
 * All rights reserved.
 *
 * This file is part of the HessianAffine detector and is made available under
 * the terms of the BSD license (see the COPYING file).
 *
 */
#include "affine.h"
#include <assert.h>
using cv::Mat;
using namespace std;


//
#include <iostream>
//
const float INT_NORM_EPS = 1e-10;
inline float intensityNormCoef (const float intensity, const float a, const float b, const float g_inv)
{
  return (a / (intensity*g_inv+b+INT_NORM_EPS));
  //  return 1.0;

}

bool AffineShape::findAffineShape(const Mat &blur, float x, float y, float s, float pixelDistance, int type, float response)
{

  float eigen_ratio_act = 0.0f, eigen_ratio_bef = 0.0f;
  float u11 = 1.0f, u12 = 0.0f, u21 = 0.0f, u22 = 1.0f, l1 = 1.0f, l2 = 1.0f;
  float lx = x/pixelDistance, ly = y/pixelDistance;
  //  float ratio = 1.0f;
  float ratio =  s / (par.initialSigma*pixelDistance);

  Mat U, V, d, Au, Ap, D;

  if (par.doBaumberg)
    {
      // kernel size...
      const int maskPixels = par.smmWindowSize * par.smmWindowSize;
      for (int l = 0; l < par.maxIterations; l++)
        {
          float a = 0, b = 0, c = 0;
          if (par.affBmbrgMethod == AFF_BMBRG_SMM) {

              // warp input according to current shape matrix
              interpolate(blur, lx, ly, u11*ratio, u12*ratio, u21*ratio, u22*ratio, img);
              // compute SMM on the warped patch
              float *maskptr = mask.ptr<float>(0);
              float *pfx = fx.ptr<float>(0), *pfy = fy.ptr<float>(0);

              // float *imgptr = img.ptr<float>(0); //!
              computeGradient(img, fx, fy);
              // estimate SMM
              for (int i = 0; i < maskPixels; ++i)
                {
                  const float v = (*maskptr);
                  const float gxx = *pfx;
                  const float gyy = *pfy;
                  const float gxy = gxx * gyy;

                  a += gxx * gxx * v;
                  b += gxy * v;
                  c += gyy * gyy * v;
                  pfx++;
                  pfy++;
                  maskptr++;
                }
              a /= maskPixels;
              b /= maskPixels;
              c /= maskPixels;

              // compute inverse sqrt of the SMM
              invSqrt(a, b, c, l1, l2);

              if ((a != a) || (b != b) || (c !=c)){ //check for nan
                  break;
                }

              // update e igen ratios
              eigen_ratio_bef = eigen_ratio_act;
              eigen_ratio_act = 1.0 - l2 / l1;

              // accumulate the affine shape matrix
              float u11t = u11, u12t = u12;

              u11 = a*u11t+b*u21;
              u12 = a*u12t+b*u22;
              u21 = b*u11t+c*u21;
              u22 = b*u12t+c*u22;

            } else if (par.affBmbrgMethod == AFF_BMBRG_HESSIAN) {
              float Dxx, Dxy, Dyy;
              float affRatio = s * par.affMeasRegion / pixelDistance;
              Ap = (cv::Mat_<float>(2,2) << u11, u12, u21, u22);
              interpolate(blur, lx, ly, u11*affRatio, u12*affRatio, u21*affRatio, u22*affRatio, imgHes);


              Dxx = (      imgHes.at<float>(0,0) - 2.f*imgHes.at<float>(0,1) +     imgHes.at<float>(0,2)
                           + 2.f*imgHes.at<float>(1,0) - 4.f*imgHes.at<float>(1,1) + 2.f*imgHes.at<float>(1,2)
                           +     imgHes.at<float>(2,0) - 2.f*imgHes.at<float>(2,1) +     imgHes.at<float>(2,2));

              Dyy = (      imgHes.at<float>(0,0) + 2.f*imgHes.at<float>(0,1) +     imgHes.at<float>(0,2)
                           - 2.f*imgHes.at<float>(1,0) - 4.f*imgHes.at<float>(1,1) - 2.f*imgHes.at<float>(1,2)
                           +     imgHes.at<float>(2,0) + 2.f*imgHes.at<float>(2,1) +     imgHes.at<float>(2,2));

              Dxy = (      imgHes.at<float>(0,0)           -     imgHes.at<float>(0,2)
                           - imgHes.at<float>(2,0)           +     imgHes.at<float>(2,2));

              // Inv. square root using SVD method, somehow the SMM method does not work
              Au = (cv::Mat_<float>(2,2) << Dxx, Dxy, Dxy, Dyy);
              cv::SVD::compute(Au,d,U,V);

              l1 = d.at<float>(0,0);
              l2 = d.at<float>(0,1);

              eigen_ratio_bef=eigen_ratio_act;
              eigen_ratio_act=1.0-abs(l2)/abs(l1);

              float det = sqrt(abs(l1*l2));
              l2 = sqrt(sqrt(abs(l1)/det));
              l1 = 1./l2;

              D = (cv::Mat_<float>(2,2) << l1, 0, 0, l2);
              Au = U * D * V;
              Ap = Au * Ap * Au;

              u11 = Ap.at<float>(0,0); u12 = Ap.at<float>(0,1);
              u21 = Ap.at<float>(1,0); u22 = Ap.at<float>(1,1);
            }

          // compute the eigen values of the shape matrix
          if (!getEigenvalues(u11, u12, u21, u22, l1, l2)){
//              if (affineShapeCallback)
//                affineShapeCallback->onAffineShapeFound(blur, x, y, s, pixelDistance, 1, 0, 0, 1, type, response, l);
//              return true;
//            }
            break; }

          // leave on too high anisotropy
          if ((l1/l2>6) || (l2/l1>6)) {
//              if (affineShapeCallback)
//                affineShapeCallback->onAffineShapeFound(blur, x, y, s, pixelDistance, 1, 0, 0, 1, type, response, l);
//              return true;
         //   }
            break; }

          if (eigen_ratio_act < par.convergenceThreshold && eigen_ratio_bef < par.convergenceThreshold)
            {
              if (affineShapeCallback)
                affineShapeCallback->onAffineShapeFound(blur, x, y, s, pixelDistance, u11, u12, u21, u22, type, response, l);
              return true;
            }

//          if (l == par.maxIterations - 1){
//              if (affineShapeCallback)
//                affineShapeCallback->onAffineShapeFound(blur, x, y, s, pixelDistance, 1, 0, 0, 1, type, response, l);
//              return true;
//            }
        }
    }
  else
    {
      if (affineShapeCallback)
        affineShapeCallback->onAffineShapeFound(blur, x, y, s, pixelDistance, u11, u12, u21, u22, type, response, 0);
      return true;
    }
  return false;
}

void AffineShape::normalizeAffine(const Mat &img,
                                  float x, float y, float s, float a11, float a12, float a21, float a22,
                                  int type, float response)
{
  assert( fabs(a11*a22-a12*a21 - 1.0f) < 0.01);
  if (normalizedPatchCallback)
    normalizedPatchCallback->onNormalizedPatchAvailable(patch, x, y, s, a11, a12, a21, a22, type, response);
}

//
//template <int bins>
//void smoothCircularBuffer(float *hist)
//{
//  float first = hist[0], prev = hist[bins-1];
//  for (int i = 0; i < bins - 1; i++)
//    {
//      float cur = hist[i];
//      hist[i] = prev + cur + hist[i+1];
//      prev = cur;
//    }
//  hist[bins-1] = prev + hist[bins-1] + first;
//}
//
//template <int bins>
//inline void addPeakAngle(const float *hist, vector<float> &angles, int a, int b, int c, float threshold)
//{
//  if (hist[b] >= threshold && hist[b] > hist[a] && hist[b] > hist[c])
//    {
//      float pp = (hist[a] - hist[c]) / (hist[a] - 2.0f * hist[b] + hist[c]) / 2.0f;
//      angles.push_back(2.0f * float(M_PI) * (b + 0.5f + pp) / bins - float(M_PI));
//    }
//}

//void AffineShape::estimateDominantAngles(const Mat &img, vector<float> &angles)
//{
//  const int bins = 36;
//  float hist[bins+1];
//
//  // clear bins
//  for (int i = 0; i<bins; i++) hist[i] = 0.0f;
//  computeGradientMagnitudeAndOrientation(img, gmag, gori);
//
//  float *maskptr = orimask.ptr<float>(1);
//  float *pmag = gmag.ptr<float>(1), *pori = gori.ptr<float>(1);
//  const int maskPixels = orimask.cols * (orimask.rows-2);
//  for (int i = 0; i < maskPixels; ++i)
//    {
//      if (*maskptr > 0 && *pmag > 1.0)
//        {
//          int bin = (int) (bins * (*pori/float(M_PI) + 1.0f) / 2.0f);
//          assert(bin >= 0 && bin <= bins);
//          hist[bin] += (*pmag) * (*maskptr);
//        }
//
//      pmag++;
//      pori++;
//      maskptr++;
//    }
//
//  for (int i = 0; i < 6; i++) smoothCircularBuffer<bins>(hist);
//  float thresh = 0.0;
//  for (int i = 0; i < bins; i++) if (hist[i] > thresh) thresh = hist[i];
//  thresh *= 0.8f;
//
//  // output all local maxima above threshold
//  angles.clear();
//  addPeakAngle<bins>(hist, angles, bins-1, 0, 1, thresh);
//  for (int i = 1; i < bins-1; i++)
//    addPeakAngle<bins>(hist, angles, i-1, i, i+1, thresh);
//  addPeakAngle<bins>(hist, angles, bins-2, bins-1, 0, thresh);
//}
