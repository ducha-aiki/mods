/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/
#undef __STRICT_ANSI__

#include <iostream>
#include <opencv2/core/core.hpp>

#include "../utls/ecompat.h"
#include "libExtrema.h"
#include "../utls/timeutls.h"
#include "../../structures.hpp"
#include "matrix.h"
#include "../../helpers.h"
#include "extrema.h"


///
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
///

//using namespace cv;
using namespace std;
using namespace extrema;

#define MAX_PATH_LEN 1024

bool marginCompareInvOrder(AffineKeypoint k1,AffineKeypoint k2) {return (fabs(k1.response) > fabs(k2.response));}

int prepareKeysForExport(vector<AffineKeypoint> &keys, ExtremaParams par, double &effectiveThreshold)
{
  if (keys.size() <= 0) return 0;
  if (par.DetectorMode == FIXED_TH)
    {
      effectiveThreshold = par.min_margin;
      return keys.size(); //we have already filtered regions when detect
    }
  std::sort (keys.begin(), keys.end(), marginCompareInvOrder);
  double maxResponse = fabs(keys[0].response);
  int regNumber = (int) keys.size();

  switch (par.DetectorMode)
    {
    case RELATIVE_TH:
      {
        effectiveThreshold = maxResponse * par.rel_threshold;
        AffineKeypoint tempKey = keys[0];
        tempKey.response = effectiveThreshold;
        std::vector<AffineKeypoint>::iterator low;
        low = std::lower_bound(keys.begin(), keys.end(), tempKey,marginCompareInvOrder);
        keys.resize(low - keys.begin());
        break;
      }
    case FIXED_REG_NUMBER:
      {
        if ((par.reg_number < regNumber) && (par.reg_number >=0))
          keys.resize(par.reg_number);
        break;
      }
    case RELATIVE_REG_NUMBER:
      {
        int newRegNumber = (int)floor(par.rel_reg_number * (double)keys.size());
        keys.resize(newRegNumber);
        break;
      }
    case NOT_LESS_THAN_REGIONS:
      {
        AffineKeypoint tempKey = keys[0];
        tempKey.response = par.min_margin;
        std::vector<AffineKeypoint>::iterator low;
        low = std::lower_bound(keys.begin(), keys.end(), tempKey,marginCompareInvOrder);

        int RegsFixThNumber = std::distance( keys.begin(), low);

        if (RegsFixThNumber < par.reg_number)
          keys.resize(min(par.reg_number,regNumber)); //use reg_number
        else
          keys.resize(min(RegsFixThNumber,regNumber)); //use threshold
              //enough keys, use fixed threshold
        break;
      }


    default:
      break;
    }
  effectiveThreshold = keys[keys.size() - 1].response;
  return keys.size();
}

int DetectMSERs(cv::Mat &input, vector<AffineKeypoint> &out1, ExtremaParams params, const double tilt, const double zoom)
{
  extrema::ExtremaParams ep;
  ep = params;

  if ((tilt > 2.0) || (zoom < 0.5))
    ep.reg_number = (int)floor(zoom*2.0*ep.reg_number/tilt);

  double finalThreshold, effectiveThreshold;

  if (params.DetectorMode !=FIXED_TH)
    finalThreshold = effectiveThreshold = 1.0;
  else
    finalThreshold = effectiveThreshold = params.min_margin;

  ep.min_margin = finalThreshold;

  // MSER on WLD-transformed image
  if (params.doOnWLD)
    {
      vector<AffineKeypoint> keys;
      float *in_ptr;
      ExtremaImage im;
      im.height = input.rows;
      im.width = input.cols;
      im.channels = 1;
      im.data = new unsigned char[im.channels*im.width*im.height];
      unsigned int pixels = im.height*im.width;
      unsigned char *ptr = im.data;

      cv::Mat temp_img2;
   //   calculateWLDfast(input,temp_img2,ep.WLDPar,1.5,0.5,1.6);

      temp_img2 = (temp_img2 + 1.0)*127.0; //to have [0; 255] image

      imwrite("wld.png",temp_img2);

      in_ptr = (float*)temp_img2.data;
      for(unsigned int i = 0; i < pixels; i++, ptr++,in_ptr++)
        *ptr=(unsigned char)*in_ptr;

      RLEExtrema result;
      result = getRLEExtrema(ep, im);
      AffineKeypoint tmpRegion;

      tmpRegion.s = 1.0;
      unsigned int MSERplus_size = result.MSERplus.size();
      unsigned int MSERmin_size  = result.MSERmin.size();
      keys.reserve(MSERmin_size+MSERplus_size);
      for(size_t i=0; i < MSERplus_size; i++)
        {
          const RLERegion *r = &result.MSERplus[i];
          double barX, barY, sumX2, sumY2, sumXY;
          RLE2Ellipse(r->rle, barX, barY, sumX2, sumXY, sumY2);
          utls::Matrix2 C(sumX2, sumXY, sumXY, sumY2);
          utls::Matrix2 U, T1, A;

          // C=C*(scale_factor*scale_factor);
          C.schur_sym(U, T1);
          A = U * T1.sqrt() * U.transpose();

          tmpRegion.x=barX;
          tmpRegion.y=barY;
          tmpRegion.a11=A[0][0];
          tmpRegion.a12=A[0][1];
          tmpRegion.a21=A[1][0];
          tmpRegion.a22=A[1][1];
//          tmpRegion.s = sqrt(fabs(tmpRegion.a11*tmpRegion.a22-tmpRegion.a12*tmpRegion.a21));
//          rectifyAffineTransformationUpIsUp(tmpRegion.a11, tmpRegion.a12, tmpRegion.a21, tmpRegion.a22);
          tmpRegion.response = r->margin;
          tmpRegion.sub_type = 21;

          keys.push_back(tmpRegion);
        };
      for(size_t i=0; i < MSERmin_size; i++)
        {
          const RLERegion *r = &result.MSERmin[i];
          double barX, barY, sumX2, sumY2, sumXY;
          RLE2Ellipse(r->rle, barX, barY, sumX2, sumXY, sumY2);
          utls::Matrix2 C(sumX2, sumXY, sumXY, sumY2);
          utls::Matrix2 U, T1, A;

          // C=C*(scale_factor*scale_factor);
          C.schur_sym(U, T1);
          A = U * T1.sqrt() * U.transpose();

          tmpRegion.x=barX;
          tmpRegion.y=barY;
          tmpRegion.a11=A[0][0];
          tmpRegion.a12=A[0][1];
          tmpRegion.a21=A[1][0];
          tmpRegion.a22=A[1][1];
//          tmpRegion.s = sqrt(fabs(tmpRegion.a11*tmpRegion.a22-tmpRegion.a12*tmpRegion.a21));
//          rectifyAffineTransformationUpIsUp(tmpRegion.a11, tmpRegion.a12, tmpRegion.a21, tmpRegion.a22);
          tmpRegion.response = r->margin;
          tmpRegion.sub_type = 20;
          keys.push_back(tmpRegion);
        };
      delete [] im.data;

      prepareKeysForExport(keys,ep,effectiveThreshold);
    //  std::cout << "MaxResp= " << fabs(keys[0].response) << ", MinResp= " << fabs(effectiveThreshold) << std::endl;
      out1.insert(out1.end(), keys.begin(), keys.end());
    }

  //MSER on normal image
  if (params.doOnNormal)
    {
      vector<AffineKeypoint> keys;
      float *in_ptr;
      ExtremaImage im;
      im.height = input.rows;
      im.width = input.cols;
      im.channels = 1;
      im.data = new unsigned char[im.channels*im.width*im.height];
      unsigned int pixels = im.height*im.width;
      unsigned char *ptr = im.data;

      in_ptr = (float*)input.data;
      for(unsigned int i = 0; i < pixels; i++, ptr++,in_ptr++)
        *ptr=(unsigned char)*in_ptr;

      // copy params

      RLEExtrema result;
      result = getRLEExtrema(ep, im);
      AffineKeypoint tmpRegion;

      tmpRegion.s = 1.0;
      unsigned int MSERplus_size = result.MSERplus.size();
      unsigned int MSERmin_size  = result.MSERmin.size();
      keys.reserve(MSERmin_size+MSERplus_size);
      for(size_t i=0; i < MSERplus_size; i++)
        {
          const RLERegion *r = &result.MSERplus[i];
          double barX, barY, sumX2, sumY2, sumXY;
          RLE2Ellipse(r->rle, barX, barY, sumX2, sumXY, sumY2);
          utls::Matrix2 C(sumX2, sumXY, sumXY, sumY2);
          utls::Matrix2 U, T1, A;

          // C=C*(scale_factor*scale_factor);
          C.schur_sym(U, T1);
          A = U * T1.sqrt() * U.transpose();

          tmpRegion.x=barX;
          tmpRegion.y=barY;
          tmpRegion.a11=A[0][0];
          tmpRegion.a12=A[0][1];
          tmpRegion.a21=A[1][0];
          tmpRegion.a22=A[1][1];
          tmpRegion.s = sqrt(fabs(tmpRegion.a11*tmpRegion.a22-tmpRegion.a12*tmpRegion.a21));
          rectifyAffineTransformationUpIsUp(tmpRegion.a11, tmpRegion.a12, tmpRegion.a21, tmpRegion.a22);
          tmpRegion.response = r->margin;
          tmpRegion.sub_type = 21;
          keys.push_back(tmpRegion);
        };

      for(size_t i=0; i < MSERmin_size; i++)
        {
          const RLERegion *r = &result.MSERmin[i];
          double barX, barY, sumX2, sumY2, sumXY;
          RLE2Ellipse(r->rle, barX, barY, sumX2, sumXY, sumY2);
          utls::Matrix2 C(sumX2, sumXY, sumXY, sumY2);
          utls::Matrix2 U, T1, A;

          // C=C*(scale_factor*scale_factor);
          C.schur_sym(U, T1);
          A = U * T1.sqrt() * U.transpose();

          tmpRegion.x=barX;
          tmpRegion.y=barY;
          tmpRegion.a11=A[0][0];
          tmpRegion.a12=A[0][1];
          tmpRegion.a21=A[1][0];
          tmpRegion.a22=A[1][1];
  //        tmpRegion.s = sqrt(fabs(tmpRegion.a11*tmpRegion.a22-tmpRegion.a12*tmpRegion.a21));
  //        rectifyAffineTransformationUpIsUp(tmpRegion.a11, tmpRegion.a12, tmpRegion.a21, tmpRegion.a22);
          tmpRegion.response = r->margin;
          tmpRegion.sub_type = 20;
          keys.push_back(tmpRegion);
        };
      delete [] im.data;
      prepareKeysForExport(keys,ep,effectiveThreshold);
      //std::cout << "MaxResp= " << fabs(keys[0].response) << ", MinResp= " << fabs(effectiveThreshold) << std::endl;
      out1.insert(out1.end(), keys.begin(), keys.end());

    }


  return out1.size();
}

int DetectMSERs(cv::Mat &input, vector<AffineKeypoint> &out1, ExtremaParams params, ScalePyramid &scale_pyramid, const double tilt, const double zoom)
{
  extrema::ExtremaParams ep;
  ep = params;

  if ((tilt > 2.0) || (zoom < 0.5))
    ep.reg_number = (int)floor(zoom*2.0*ep.reg_number/tilt);

  double finalThreshold, effectiveThreshold;

  if (params.DetectorMode !=FIXED_TH)
    finalThreshold = effectiveThreshold = 1.0;
  else
    finalThreshold = effectiveThreshold = params.min_margin;

  ep.min_margin = finalThreshold;

  // MSER on WLD-transformed image
  if (params.doOnWLD)
    {
      vector<AffineKeypoint> keys;
      float *in_ptr;
      ExtremaImage im;
      im.height = input.rows;
      im.width = input.cols;
      im.channels = 1;
      im.data = new unsigned char[im.channels*im.width*im.height];
      unsigned int pixels = im.height*im.width;
      unsigned char *ptr = im.data;

      cv::Mat temp_img2;
   //   calculateWLDfast(input,temp_img2,ep.WLDPar,1.5,0.5,1.6);

      temp_img2 = (temp_img2 + 1.0)*127.0; //to have [0; 255] image

      imwrite("wld.png",temp_img2);

      in_ptr = (float*)temp_img2.data;
      for(unsigned int i = 0; i < pixels; i++, ptr++,in_ptr++)
        *ptr=(unsigned char)*in_ptr;

      RLEExtrema result;
      result = getRLEExtrema(ep, im);
      AffineKeypoint tmpRegion;

      tmpRegion.s = 1.0;
      unsigned int MSERplus_size = result.MSERplus.size();
      unsigned int MSERmin_size  = result.MSERmin.size();
      keys.reserve(MSERmin_size+MSERplus_size);
      for(size_t i=0; i < MSERplus_size; i++)
        {
          const RLERegion *r = &result.MSERplus[i];
          double barX, barY, sumX2, sumY2, sumXY;
          RLE2Ellipse(r->rle, barX, barY, sumX2, sumXY, sumY2);
          utls::Matrix2 C(sumX2, sumXY, sumXY, sumY2);
          utls::Matrix2 U, T1, A;

          // C=C*(scale_factor*scale_factor);
          C.schur_sym(U, T1);
          A = U * T1.sqrt() * U.transpose();

          tmpRegion.x=barX;
          tmpRegion.y=barY;
          tmpRegion.a11=A[0][0];
          tmpRegion.a12=A[0][1];
          tmpRegion.a21=A[1][0];
          tmpRegion.a22=A[1][1];
          tmpRegion.response = r->margin;
          tmpRegion.sub_type = 21;
  //        tmpRegion.s = sqrt(fabs(tmpRegion.a11*tmpRegion.a22-tmpRegion.a12*tmpRegion.a21));
  //        rectifyAffineTransformationUpIsUp(tmpRegion.a11, tmpRegion.a12, tmpRegion.a21, tmpRegion.a22);
          keys.push_back(tmpRegion);
        };
      for(size_t i=0; i < MSERmin_size; i++)
        {
          const RLERegion *r = &result.MSERmin[i];
          double barX, barY, sumX2, sumY2, sumXY;
          RLE2Ellipse(r->rle, barX, barY, sumX2, sumXY, sumY2);
          utls::Matrix2 C(sumX2, sumXY, sumXY, sumY2);
          utls::Matrix2 U, T1, A;

          // C=C*(scale_factor*scale_factor);
          C.schur_sym(U, T1);
          A = U * T1.sqrt() * U.transpose();

          tmpRegion.x=barX;
          tmpRegion.y=barY;
          tmpRegion.a11=A[0][0];
          tmpRegion.a12=A[0][1];
          tmpRegion.a21=A[1][0];
          tmpRegion.a22=A[1][1];
    //      tmpRegion.s = sqrt(fabs(tmpRegion.a11*tmpRegion.a22-tmpRegion.a12*tmpRegion.a21));
   //       rectifyAffineTransformationUpIsUp(tmpRegion.a11, tmpRegion.a12, tmpRegion.a21, tmpRegion.a22);
          tmpRegion.response = r->margin;
          tmpRegion.sub_type = 20;
          keys.push_back(tmpRegion);
        };
      delete [] im.data;

      prepareKeysForExport(keys,ep,effectiveThreshold);
    //  std::cout << "MaxResp= " << fabs(keys[0].response) << ", MinResp= " << fabs(effectiveThreshold) << std::endl;
      out1.insert(out1.end(), keys.begin(), keys.end());
    }

  //MSER on normal image
  if (params.doOnNormal)
    {
      vector<AffineKeypoint> keys;
      float *in_ptr;
      ExtremaImage im;
      im.height = input.rows;
      im.width = input.cols;
      im.channels = 1;
      im.data = new unsigned char[im.channels*im.width*im.height];
      unsigned int pixels = im.height*im.width;
      unsigned char *ptr = im.data;

      in_ptr = (float*)input.data;
      for(unsigned int i = 0; i < pixels; i++, ptr++,in_ptr++)
        *ptr=(unsigned char)*in_ptr;

      // copy params

      RLEExtrema result;
      result = getRLEExtrema(ep, im);
      AffineKeypoint tmpRegion;

      tmpRegion.s = 1.0;
      unsigned int MSERplus_size = result.MSERplus.size();
      unsigned int MSERmin_size  = result.MSERmin.size();
      keys.reserve(MSERmin_size+MSERplus_size);
      for(size_t i=0; i < MSERplus_size; i++)
        {
          const RLERegion *r = &result.MSERplus[i];
          double barX, barY, sumX2, sumY2, sumXY;
          RLE2Ellipse(r->rle, barX, barY, sumX2, sumXY, sumY2);
          utls::Matrix2 C(sumX2, sumXY, sumXY, sumY2);
          utls::Matrix2 U, T1, A;

          // C=C*(scale_factor*scale_factor);
          C.schur_sym(U, T1);
          A = U * T1.sqrt() * U.transpose();

          tmpRegion.x=barX;
          tmpRegion.y=barY;
          tmpRegion.a11=A[0][0];
          tmpRegion.a12=A[0][1];
          tmpRegion.a21=A[1][0];
          tmpRegion.a22=A[1][1];
  //        tmpRegion.s = sqrt(fabs(tmpRegion.a11*tmpRegion.a22-tmpRegion.a12*tmpRegion.a21));
  //        rectifyAffineTransformationUpIsUp(tmpRegion.a11, tmpRegion.a12, tmpRegion.a21, tmpRegion.a22);
          tmpRegion.response = r->margin;
          tmpRegion.sub_type = 21;
          keys.push_back(tmpRegion);
        };

      for(size_t i=0; i < MSERmin_size; i++)
        {
          const RLERegion *r = &result.MSERmin[i];
          double barX, barY, sumX2, sumY2, sumXY;
          RLE2Ellipse(r->rle, barX, barY, sumX2, sumXY, sumY2);
          utls::Matrix2 C(sumX2, sumXY, sumXY, sumY2);
          utls::Matrix2 U, T1, A;

          // C=C*(scale_factor*scale_factor);
          C.schur_sym(U, T1);
          A = U * T1.sqrt() * U.transpose();

          tmpRegion.x=barX;
          tmpRegion.y=barY;
          tmpRegion.a11=A[0][0];
          tmpRegion.a12=A[0][1];
          tmpRegion.a21=A[1][0];
          tmpRegion.a22=A[1][1];
    //      tmpRegion.s = sqrt(fabs(tmpRegion.a11*tmpRegion.a22-tmpRegion.a12*tmpRegion.a21));
    //      rectifyAffineTransformationUpIsUp(tmpRegion.a11, tmpRegion.a12, tmpRegion.a21, tmpRegion.a22);
          tmpRegion.response = r->margin;
          tmpRegion.sub_type = 20;
          keys.push_back(tmpRegion);
        };
      delete [] im.data;
      prepareKeysForExport(keys,ep,effectiveThreshold);
      //std::cout << "MaxResp= " << fabs(keys[0].response) << ", MinResp= " << fabs(effectiveThreshold) << std::endl;
      out1.insert(out1.end(), keys.begin(), keys.end());

    }


  return out1.size();
}
