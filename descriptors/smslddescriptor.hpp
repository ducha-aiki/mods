#ifndef SMSLDDESCRIPTOR_HPP
#define SMSLDDESCRIPTOR_HPP

#include "detectors/structures.hpp"
//#include "../smsld/descriptor.h"
//#include "../smsld/Match.h"

struct SMSLDParams
{
    PatchExtractionParams PEParam;
//  int patchSize;
//  double mrSize;
//  bool FastPatchExtraction;
  SMSLDParams()
  {

//    patchSize = 41;
//    mrSize =  3.0*sqrt(3.0);
//    FastPatchExtraction = false;
  }
};

//void GetIplImageData(double* pImageData,const cv::Mat &cvImg)
//{
//  int nWidth= cvImg.cols;
//  int nHeight = cvImg.rows;
//  for(int i = 0; i < nHeight; i++)
//    for(int j =0; j < nWidth; j++)
//      {
//        int k1 = i*nLineWidth + j;
//        char charmp = (char)cvImg->imageData[k1];
//        int k2 = i*nWidth + j;
//        pImageData[k2] = (double)charmp;
//      }
//}

struct SMSLDDescriptor
{
public:
  SMSLDDescriptor(const SMSLDParams &par)
  {
    this->par = par;
    type = DESC_SMSLD;

    temp_pt.pt.x = par.PEParam.patchSize / 2;
    temp_pt.pt.y = par.PEParam.patchSize / 2;
    temp_pt.angle = 0;
    temp_pt.size = float (par.PEParam.patchSize) / par.PEParam.mrSize;
    temp_pt.octave = 1;
    temp_pt.class_id = 1;
    //   keypoints_1.push_back(temp_pt);
  }
  void operator()(cv::Mat &patch, std::vector<float>& desc)
  {
    std::cerr << "Not implemented!" << std::endl;
//    cv::Mat patch64;
//    patch.convertTo(patch64,CV_64F);
//    double* patchPtr = patch64.ptr<double>(0);

//    //Get points and describe lines.
//    int nLineCount1		= 1;
//    int szCountForEachLine1[nMaxLineCount];
//    float scalesForEachLine1[2*nMaxLineCount];
//    float angleForEachLine1[2*nMaxLineCount];
//    double *pLinePts = new double[2];
//    pLinePts[0] = temp_pt.pt.x;
//    pLinePts[1] = temp_pt.pt.y;

//    float* pDes1 = NULL;
//    char*  pByValidFlag1 = new char[nLineCount1];
//    pDes1 = new float[nDesDim*nLineCount1];
//    ComputeDes(	pDes1,pByValidFlag1,
//                patchPtr, par.PEParam.patchSize,par.PEParam.patchSize,
//                pLinePts,nLineCount1,szCountForEachLine1,scalesForEachLine1,angleForEachLine1);
//    desc.resize(nDesDim*nLineCount1);
//    int jj=0;
//    for (int k=0; k<nDesDim*nLineCount1; k++) {
//        desc[jj] = (float) pDes1[k];
//      }
  }
public:
  descriptor_type type;
  int desc_size;

private:
  SMSLDParams par;
  //  std::vector<cv::KeyPoint> keypoints_1; //for binary-dets
  cv::KeyPoint temp_pt;
  //  SMSLD_params_t params;
  //  SMSLD_info_t info;
  //  SMSLD_t *desc_SMSLD1;
};


#endif // SMSLDDESCRIPTOR_HPP
