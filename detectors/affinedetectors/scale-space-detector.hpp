#ifndef SCALESPACEDETECTOR_HPP
#define SCALESPACEDETECTOR_HPP

#undef __STRICT_ANSI__
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "pyramid.h"
#include "../helpers.h"
#include "affine.h"
#include "../structures.hpp"
#include <iterator>
#include <iostream>


using namespace cv;
using namespace std;

struct ScaleSpaceDetectorParams
{
  AffineShapeParams AffineShapePars;
  PyramidParams PyramidPars;
 // TILDEParams TILDEParam;
  ScaleSpaceDetectorParams()
  {
  }
};

struct AffineDetector : public ScaleSpaceDetector, AffineShape, KeypointCallback, AffineShapeCallback, NormalizedPatchCallback
{
  const Mat image;
  vector<AffineKeypoint> keys;
  int g_numberOfPoints;
  int g_numberOfAffinePoints;
  int g_numberOfDescribedPoints;

public:
  AffineDetector(const Mat &image, const PyramidParams &par, const AffineShapeParams &ap) :
    ScaleSpaceDetector(par),
    AffineShape(ap),
    image(image)
  {
    this->setKeypointCallback(this);
    this->setAffineShapeCallback(this);
    this->setNormalizedPatchCallback(this);
  }

  void onKeypointDetected(const Mat &blur, float x, float y, float s, float pixelDistance, int type, float response)
  {
    g_numberOfPoints++;
   // if (type == ScaleSpaceDetector::TILDE) {
   //     findAffineShape(blur, x, y, s*(scale_coef_tilde), pixelDistance, type, response);
   //   } else {
        findAffineShape(blur, x, y, s, pixelDistance, type, response);
   //   }
  }
  void onAffineShapeFound(
      const Mat &blur, float x, float y, float s, float pixelDistance,
      float a11, float a12, float a21, float a22,
      int type, float response, int iters)
  {
    // convert shape into a up is up frame
    // rectifyAffineTransformationUpIsUp(a11, a12, a21, a22);
    // now sample the patch
    normalizeAffine(image, x, y, s, a11, a12, a21, a22, type, response);
    g_numberOfAffinePoints++;
  }

  void onNormalizedPatchAvailable(
      const Mat &patch,
      float x, float y, float s,
      float a11, float a12, float a21, float a22,
      int type, float response)
  {
    // store the keypoint
    keys.push_back(AffineKeypoint());
    AffineKeypoint &k = keys.back();
    k.x = x;
    k.y = y;
    k.s = s;
    k.a11 = a11;
    k.a12 = a12;
    k.a21 = a21;
    k.a22 = a22;
    k.response = response;
    k.sub_type = type;
    g_numberOfDescribedPoints++;
  }

  void exportKeypoints(vector<AffineKeypoint>& out1)
  {
  //  std::cerr << "Hessian points detected " << g_numberOfPoints << std::endl;
  //  std::cerr << "AffineShapes points detected " << g_numberOfAffinePoints << std::endl;

    prepareKeysForExport();
    unsigned int keys_size = keys.size();
    out1.reserve(out1.size() + keys_size);
    for (size_t i=0; i < keys_size; i++)
      {
        AffineKeypoint &k = keys[i];        
        AffineKeypoint tmpRegion;
   //     k.s *= sqrt(fabs(k.a11*k.a22-k.a12*k.a21));
   //     rectifyAffineTransformationUpIsUp(k.a11, k.a12, k.a21, k.a22);
        tmpRegion.x=k.x;
        tmpRegion.y=k.y;
        tmpRegion.a11=k.a11;
        tmpRegion.a12=k.a12;
        tmpRegion.a21=k.a21;
        tmpRegion.a22=k.a22;
        tmpRegion.s=k.s;
        tmpRegion.response = k.response;
        tmpRegion.sub_type = k.sub_type;

        out1.push_back(tmpRegion);
      };
  }
  void exportScaleSpace(ScalePyramid& exp_scale_pyramid)
  {
    exp_scale_pyramid = scale_pyramid;
  }

private:
  void sortKeys()
  {
    std::sort (keys.begin(), keys.end(), responseCompareInvOrder);
  }
  int prepareKeysForExport()
  {
    if (keys.size() <= 0) return 0;
    if (Pyrpar.DetectorMode == FIXED_TH)
      {
        effectiveThreshold = Pyrpar.threshold;
      }
    else
      {
        sortKeys();
    //    std::cerr << "Keys sorted" << std::endl;
        double maxResponse = fabs(keys[0].response);
        int regNumber = (int) keys.size();

        switch (Pyrpar.DetectorMode)
          {
          case RELATIVE_TH:
            {
              effectiveThreshold = maxResponse * Pyrpar.rel_threshold;
              AffineKeypoint tempKey = keys[0];
              tempKey.response = effectiveThreshold;
              std::vector<AffineKeypoint>::iterator low;
              low = std::lower_bound(keys.begin(), keys.end(), tempKey,responseCompareInvOrder);
              keys.resize(low - keys.begin());
              break;
            }
          case FIXED_REG_NUMBER:
            {
              int newRegNumber = Pyrpar.reg_number;
              if (par.doBaumberg)
                newRegNumber =(int) floor(3.0*(double)newRegNumber);

              if ((newRegNumber < regNumber) && (newRegNumber >=0))
                keys.resize(newRegNumber);

              break;
            }
          case RELATIVE_REG_NUMBER:
            {
              int newRegNumber = (int)floor(Pyrpar.rel_reg_number * (double)keys.size());
              keys.resize(newRegNumber);
              break;
            }
          case NOT_LESS_THAN_REGIONS:
            {
              AffineKeypoint tempKey = keys[0];
              tempKey.response = Pyrpar.threshold;
              std::vector<AffineKeypoint>::iterator low;
              low = std::lower_bound(keys.begin(), keys.end(), tempKey,responseCompareInvOrder);

              int RegsFixThNumber = std::distance( keys.begin(), low);

              if (RegsFixThNumber < Pyrpar.reg_number)
                keys.resize(min(Pyrpar.reg_number,regNumber)); //use reg_number
              else
                keys.resize(min(RegsFixThNumber,regNumber)); //use threshold
              //enough keys, use fixed threshold
              break;
            }

          default:
            break;
          }
        effectiveThreshold = keys[keys.size() - 1].response;

      }

    if ((Pyrpar.DetectorMode == FIXED_REG_NUMBER) && ((int)keys.size() > Pyrpar.reg_number))
      keys.resize(Pyrpar.reg_number);
    //    std::cout << "effectiveThreshold = " << effectiveThreshold << std::endl;
    return keys.size();
  }
//  void doBaumberg()
//  {
//    vector<AffineKeypoint> keys_temp(keys.size()); //temporal archive
//    for (size_t i=0; i < keys.size(); i++)
//      {
//        keys_temp[i].x=keys[i].x;
//        keys_temp[i].y=keys[i].y;
//        keys_temp[i].a11=keys[i].a11;
//        keys_temp[i].a12=keys[i].a12;
//        keys_temp[i].a21=keys[i].a21;
//        keys_temp[i].a22=keys[i].a22;
//        keys_temp[i].s=keys[i].s;
//        keys_temp[i].response = keys[i].response;
//        keys_temp[i].sub_type = keys[i].sub_type;
//      }
//    keys.clear();
//
//    for (size_t i=0; i < keys_temp.size(); i++)
//      findAffineShape(*originalImg,keys_temp[i].x,keys_temp[i].y,keys_temp[i].s, 1.0, keys_temp[i].sub_type,keys_temp[i].response);
//
//    effectiveThreshold = keys[keys.size() - 1].response;
//  }
};

template<class T>
ostream& operator<<(ostream& os, const vector<T>& v)
{
  copy(v.begin(), v.end(), ostream_iterator<T>(cout, " "));
  return os;
}

//int DetectAffineKeypoints(cv::Mat &input, vector<AffineKeypoint> &out1, ScaleSpaceDetectorParams params, const double tilt = 1.0, const double zoom = 1.0);
int DetectAffineKeypoints(cv::Mat &input, vector<AffineKeypoint> &out1, ScaleSpaceDetectorParams params, ScalePyramid &scale_pyramid, const double tilt = 1.0, const double zoom = 1.0);

#endif // SCALESPACEDETECTOR_HPP
