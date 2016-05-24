// libTILDE.cpp --- 
// 
// Filename: libTILDE.cpp
// Description: 
// Author: Yannick Verdie, Kwang Moo Yi, Alberto Crivella
// Maintainer: Yannick Verdie, Kwang Moo Yi
// Created: Tue Mar  3 17:53:46 2015 (+0100)
// Version: 0.5a
// Package-Requires: ()
// Last-Updated: Thu May 28 13:18:32 2015 (+0200)
//           By: Kwang
//     Update #: 40
// URL: 
// Doc URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Change Log:
// 
// 
// 
// 
// Copyright (C), EPFL Computer Vision Lab.
// 
// 

// Code:

#include "libTILDE.hpp"



vector < Mat > getLuv_fast(const Mat & input_color_image)
{
  if (input_color_image.channels() != 3) {
      throw std::runtime_error("Need a 3-channnel image");
    }
  vector < Mat > luvImage(3);
  for (int idxC = 0; idxC < 3; ++idxC) {
      luvImage[idxC].create(input_color_image.rows, input_color_image.cols, CV_32F);
    }

  //init
  const float y0=(float) ((6.0/29)*(6.0/29)*(6.0/29));
  const float a= (float) ((29.0/3)*(29.0/3)*(29.0/3));
  const double XYZ[3][3] = {  {  0.430574,  0.341550,  0.178325 },
                              {  0.222015,  0.706655,  0.071330 },
                              {  0.020183,  0.129553,  0.939180 }   };

  const double Un_prime   = 0.197833;
  const double Vn_prime   = 0.468331;
  const double maxi 		= 1.0/270;
  const double minu 		= -88*maxi;
  const double minv 		= -134*maxi;
  const double Lt     = 0.008856;
  static float lTable[1064];
  for(int i=0; i<1025; i++)
    {
      float y = (float) (i/1024.0);
      float l = y>y0 ? 116*(float)pow((double)y,1.0/3.0)-16 : y*a;
      lTable[i] = l*maxi;
    }

  // Get Max idx using Magnitude
  cv::parallel_for( cv::BlockedRange (0, input_color_image.rows), [=] (const cv::BlockedRange &r)
  {

      Rect roi(0, r.begin(), input_color_image.cols, r.end() - r.begin());
      Mat in(input_color_image, roi);


      Mat out1(luvImage[0],roi);
      Mat out2(luvImage[1],roi);
      Mat out3(luvImage[2],roi);


      //Rect roi(0, r.begin(), convt_image[idxDim].cols, r.end() - r.begin());
      for (int j = 0; j < in.rows; j++)
        {
          for (int i = 0; i < in.cols; i++)//row
            {
              cv::Vec3b rgb = in.at<cv::Vec3b>(j,i);
              float r = rgb[2] / 255.0f;
              float g = rgb[1] / 255.0f;
              float b = rgb[0] / 255.0f;

              //RGB to LUV conversion

              //delcare variables
              double  x, y, z, u_prime, v_prime, constant, L, u, v;

              //convert RGB to XYZ...
              x       = XYZ[0][0]*r + XYZ[0][1]*g + XYZ[0][2]*b;
              y       = XYZ[1][0]*r + XYZ[1][1]*g + XYZ[1][2]*b;
              z       = XYZ[2][0]*r + XYZ[2][1]*g + XYZ[2][2]*b;

              //convert XYZ to LUV...

              //compute ltable(y*1024)
              L = lTable[(int)(y*1024)];

              //compute u_prime and v_prime
              constant    = 1/(x + 15 * y + 3 * z + 1e-35);   //=z

              u_prime = (4 * x) * constant;   //4*x*z
              v_prime = (9 * y) * constant;


              //compute u* and v*
              u = (float) (13 * L * (u_prime - Un_prime)) - minu;
              v = (float) (13 * L * (v_prime - Vn_prime)) - minv;

              out1.at<float>(j,i) = L*270*2.55;
              out2.at<float>(j,i) = ((u*270-88)+ 134.0)* 255.0 / 354.0;
              out3.at<float>(j,i) = ((v*270-134)+ 140.0)* 255.0 / 256.0;

            }
        }

    });

  return luvImage;
}


vector < Mat > getGrad_fast(const Mat & input_color_image)
{
  if (input_color_image.channels() != 3) {
      throw std::runtime_error("Need a 3-channel image");
    }
  //the output
  vector < Mat > gradImage(3);//,Mat(input_color_image.rows, input_color_image.cols, CV_32F));
  //return gradImage;
  vector < Mat > color_channels(3);
  vector < Mat > gx(3);
  vector < Mat > gy(3);

  // The derivative5 kernels
  Mat d1 = (Mat_ < float >(1, 5) << 0.109604, 0.276691, 0.000000, -0.276691, -0.109604);
  Mat d1T = (Mat_ < float >(5, 1) << 0.109604, 0.276691, 0.000000, -0.276691, -0.109604);
  Mat p = (Mat_ < float >(1, 5) << 0.037659, 0.249153, 0.426375, 0.249153, 0.037659);
  Mat pT = (Mat_ < float >(5, 1) << 0.037659, 0.249153, 0.426375, 0.249153, 0.037659);

  // split the channels into each color channel
  split(input_color_image, color_channels);
  // // prepare output
  for (int idxC = 0; idxC < 3; ++idxC) {
      gradImage[idxC].create(color_channels[0].rows, color_channels[0].cols, CV_32F);
    }



  //for each channel do the derivative 5
  for (int idxC = 0; idxC < 3; ++idxC)
    {
      sepFilter2D(color_channels[idxC], gx[idxC], CV_32F, d1, p, Point(-1, -1), 0,
                  BORDER_REFLECT);
      sepFilter2D(color_channels[idxC], gy[idxC], CV_32F, p, d1, Point(-1, -1), 0,
                  BORDER_REFLECT);
      // since we do the other direction, just flip signs
      gx[idxC] = -gx[idxC];
      gy[idxC] = -gy[idxC];

      // the magnitude image
      //sqrt(gx[idxC].mul(gx[idxC]) + gy[idxC].mul(gy[idxC]), mag[idxC]);
    }

  // Get Max idx using Magnitude
  cv::parallel_for( cv::BlockedRange (0, gx[0].rows), [=] (const cv::BlockedRange &r)
  {
      vector<Mat> inx(3);
      Rect roi(0, r.begin(), gx[0].cols, r.end() - r.begin());
      inx[0] = Mat(gx[0], roi);
      inx[1] = Mat(gx[1], roi);
      inx[2] = Mat(gx[2], roi);

      vector<Mat> iny(3);
      iny[0] = Mat(gy[0], roi);
      iny[1] = Mat(gy[1], roi);
      iny[2] = Mat(gy[2], roi);


      Mat out1(gradImage[0],roi);
      Mat out2(gradImage[1],roi);
      Mat out3(gradImage[2],roi);
      //Rect roi(0, r.begin(), convt_image[idxDim].cols, r.end() - r.begin());
      for (int j = 0; j < inx[0].rows; j++)
        {
          for (int i = 0; i < inx[0].cols; i++)//row
            {
              float maxVal = -1;float maxValx;float maxValy;
              float val_squared;
              float valx;float valy;
              for (int idxC = 0; idxC < 3; ++idxC)
                {
                  valx = inx[idxC].at < float >(j, i);
                  valy = iny[idxC].at < float >(j, i);
                  val_squared = (valx*valx+valy*valy);
                  if (val_squared > maxVal)
                    {
                      maxVal = val_squared ;
                      maxValx = valx;
                      maxValy = valy;
                    }
                }

              out1.at < float >(j, i) = maxValx * 0.5 + 128.0;
              out2.at < float >(j, i) = maxValy * 0.5 + 128.0;
              out3.at < float >(j, i) = sqrt(maxVal);

            }
        }

    });

  return gradImage;
}






// Function which return in Keypoint Structure
vector < KeyPoint > getTILDEKeyPoints(const Mat & indatav, const string & filter_name, const bool useApprox,
                                      const bool sortMe, const bool keepPositiveScoreOnly, Mat * score)
{
  bool bUseDescriptorField = false; // disabled by default - for
  // compatibility with future use

  cv::Mat img = indatav.clone();//we copy the input data here, because we will resize it before filtering

  // Read the txt file to get the filter
  vector < float > param;
  TILDEobjects  tilde_obj = getTILDEObject(filter_name, &param,  useApprox, bUseDescriptorField);

  // Apply filtering
  // NOTE: score is CV_32FC1
  vector < Point3f > curPart;
  if (useApprox) {
      curPart = applyApproxFilters(img, tilde_obj, param, bUseDescriptorField, sortMe, keepPositiveScoreOnly, score);
    } else {
      curPart = applyNonApproxFilters(img, tilde_obj.nonApprox_filters, param, false, sortMe, keepPositiveScoreOnly, score);
    }

  const float scaleKeypoint = 10.0;const float orientation = 0;
  vector < KeyPoint > res;
  for (int i = 0; i < curPart.size(); i++) {
      res.push_back(KeyPoint(Point2f(curPart[i].x, curPart[i].y), scaleKeypoint,orientation,curPart[i].z));
    }


  return res;
}

vector < KeyPoint > getTILDEKeyPoints_fast(const Mat & indatav, const string & filter_name, const bool sortMe, const bool keepPositiveScoreOnly, Mat * score)
{
  const float scaleKeypoint = 10.0;const float orientationKeypoint = 0;

  cv::Mat img = indatav.clone();//we copy the input data here, because we will resize it before filtering

  // Read the txt file to get the filter
  vector < float > param;
  TILDEobjects  tilde_obj = getTILDEObject(filter_name, &param,  true, false);

  param[1] = scaleKeypoint;
  param[2] = orientationKeypoint;

  return applyApproxFilters_fast(img, tilde_obj, param, sortMe, keepPositiveScoreOnly, score);
}


Mat normalizeScore(const Mat& score)
{
  Mat output = score.clone();
  // if (score != NULL) {
  double minVal, maxVal;
  minMaxLoc(output, &minVal, &maxVal);
  double range = maxVal - minVal;

  if (range == 0)
    output = (output - minVal);//the score is a constant value, returns zero
  else
    output = (output - minVal) / range;

  return output;
}

void prepareData(const Mat & indatav,
		 const float& resizeRatio,
		 const bool& useDescriptorField,
		 vector < Mat > *output)
{

  Mat indata_resized = indatav;
  if (resizeRatio != 1)
    resize(indatav, indata_resized, Size(0, 0), resizeRatio, resizeRatio);

  // vector < Mat > &convt_image = output;

  if (useDescriptorField) {
      *output = getNormalizedDescriptorField(indatav);
    } else {

      vector < Mat > gradImage = getGradImage(indata_resized);
      vector < Mat > luvImage = getLuvImage(indata_resized);

      //convt_image.clear();
      copy(gradImage.begin(), gradImage.end(), std::back_inserter(*output));
      copy(luvImage.begin(), luvImage.end(), std::back_inserter(*output));

      if (output->size() != 6)
        throw std::runtime_error("Error during creation of the features (LUV+Grad)");

    }
}

void prepareData_fast(const Mat & indatav,
		      const float& resizeRatio,
		      const bool& useDescriptorField,
		      vector < Mat > *output)
{

  Mat indata_resized = indatav;
  if (resizeRatio != 1)
    resize(indatav, indata_resized, Size(0, 0), resizeRatio, resizeRatio);

  // vector < Mat > &convt_image = output;

  if (useDescriptorField) {
      *output = getNormalizedDescriptorField(indatav);
    } else {

      vector < Mat > gradImage = getGrad_fast(indata_resized);
      vector < Mat > luvImage = getLuv_fast(indata_resized);

      // vector < Mat > luvImage2 = getLuvImage(indata_resized);
      // imshow("luv1",(luvImage[1])/255);
      // imshow("luv2",(luvImage2[1])/255);
      // waitKey(0);

      //convt_image.clear();
      copy(gradImage.begin(), gradImage.end(), std::back_inserter(*output));
      copy(luvImage.begin(), luvImage.end(), std::back_inserter(*output));

      //*output =  getGrad_LUV_fast(indata_resized);

      if (output->size() != 6)
        throw std::runtime_error("Error during creation of the features (LUV+Grad)");

    }
}


void getCombinedScore(const vector < vector < Mat > >& cascade_responses, const bool &keep_only_positive, Mat *output)
{
  for (int idxCascade = 0; idxCascade < cascade_responses.size(); ++idxCascade)
    {
      Mat respImageCascade = cascade_responses[idxCascade][0];

      for (int idxDepth = 1; idxDepth < cascade_responses[idxCascade].size(); ++idxDepth)
        respImageCascade =
            max(respImageCascade, cascade_responses[idxCascade][idxDepth]);

      respImageCascade = idxCascade % 2 == 0 ? -respImageCascade : respImageCascade;
      if (idxCascade == 0)
        *output = respImageCascade;
      else
        *output = respImageCascade + *output;
    }

  //post process
  const float stdv = 2;
  const int sizeSmooth = 5 * stdv * 2 + 1;
  GaussianBlur(*output, *output, Size(sizeSmooth, sizeSmooth), stdv, stdv);

  if (keep_only_positive)
    *output = max(*output, 0);


}

//template <typename T>
vector < KeyPoint > applyApproxFilters_fast(const Mat & indatav, const TILDEobjects & tilde_obj,
					    const vector < float >&param,
					    const bool sortMe, const bool keep_only_positive,
					    Mat * score)
{
  const float resizeRatio = param[0];
  if (resizeRatio == 0)
    throw std::runtime_error("The resize ratio is zero, if you dont want any resize, use 1");

  const float scaleKeypoint = param[1];
  const float orientationKeypoint = param[2];

  Mat respImageFinal;

  vector < Mat > convt_image;
  prepareData_fast(indatav,resizeRatio, false,&convt_image);
  getScoresandCombine_Approx(tilde_obj, convt_image,keep_only_positive,&respImageFinal);


  if (score != NULL)
    *score = respImageFinal.clone();


  // perform non-max suppression
  vector < KeyPoint > res_with_score = NonMaxSup_resize_format(respImageFinal, resizeRatio, scaleKeypoint, orientationKeypoint); //return x,y,score for each keypoint, such as we can sort them later

  if (sortMe) {
      std::sort(res_with_score.begin(), res_with_score.end(),
                [](const KeyPoint & a, const KeyPoint & b) {
          return a.response > b.response;}
      );
    }

  return res_with_score;
}

vector < Point3f > applyApproxFilters(const Mat & indatav, const TILDEobjects & tilde_obj,
				      const vector < float >&param, const bool useDescriptorField,
				      const bool sortMe, const bool keep_only_positive,
				      Mat * score)
{
  const float scaleKeypoint = 10.0;const float orientation = 0;
  float resizeRatio = 1.0;
  resizeRatio = param[0];
  if (resizeRatio == 0)
    throw std::runtime_error("The resize ratio is zero, if you dont want any resize, use 1");

  vector < Mat > convt_image;
  prepareData(indatav,resizeRatio, useDescriptorField,&convt_image);

  vector < vector < Mat > >cascade_responses = getScoresForApprox(tilde_obj, convt_image);

  // apply the cascade structure and retrieve single channel response image
  Mat respImageFinal;

  getCombinedScore(cascade_responses, keep_only_positive, &respImageFinal);

  if (score != NULL)
    *score = respImageFinal.clone();

  // perform non-max suppression
  vector < Point3f > res_with_score = NonMaxSup(respImageFinal); //return x,y,score for each keypoint, such as we can sort them later

  if (sortMe) {
      std::sort(res_with_score.begin(), res_with_score.end(),
                [](const Point3f & a, const Point3f & b) {
          return a.z > b.z;}
      );
    }
  // resize back

  // resize back
  resizeRatio = 1. / resizeRatio;
  for (int i = 0; i < res_with_score.size(); ++i) {
      res_with_score[i].x = res_with_score[i].x * resizeRatio;
      res_with_score[i].y = res_with_score[i].y * resizeRatio;
    }

  return res_with_score;
}


// --------------------------------------------------------------------------------------
// THIS PART IS THE NEW FAST ONE!

class Parallel_process:public cv::ParallelLoopBody {

private:
  const TILDEobjects & cas;
  vector < Mat > &curRes;
  const int nbApproximatedFilters;
  const vector < Mat > &convt_image;

public:
  Parallel_process(const vector < Mat > &conv, const int nb, const TILDEobjects & p,
                   vector < Mat > &v):convt_image(conv), cas(p), curRes(v),
    nbApproximatedFilters(nb) {
  } virtual void operator() (const cv::Range & range)const {

    for (int idxFilter = range.start; idxFilter < range.end; idxFilter++) {

        // the separable filters
        Mat kernelX = cas.filters[idxFilter * 2 + 1];	// IMPORTANT!
        // NOTE THE ORDER!
        Mat kernelY = cas.filters[idxFilter * 2];


        // the channel this filter is supposed to be applied to
        const int idxDim = idxFilter / nbApproximatedFilters;
        Mat res;
        sepFilter2D(convt_image[idxDim], res, -1, kernelX, kernelY, Point(-1, -1),
                    0, BORDER_REFLECT);
        curRes[idxFilter] = res.clone();

      }}};

vector < vector < Mat > >getScoresForApprox(const TILDEobjects & cas,
                                            const vector < Mat > &convt_image)
{
  const vector < float >param = cas.parameters;
  if (param.size() == 0) {
      throw std::runtime_error("No parameter loaded !");
    }

  vector < vector < Mat > >res;
  int nbMax = param[1];	//4
  int nbSum = param[2];	//4
  int nbOriginalFilters = nbMax * nbSum;
  int nbApproximatedFilters = param[3];	//4
  int nbChannels = param[4];	//6
  int sizeFilters = param[5];	//21
  //--------------------

  // allocate res
  res.resize(nbSum);
  for (int idxSum = 0; idxSum < nbSum; ++idxSum) {
      res[idxSum].resize(nbMax);
    }

  // calculate separable responses
  int idxSum = 0;
  int idxMax = 0;

  vector < Mat > curRes((int)cas.filters.size() / 2, Mat(convt_image[0].size(), CV_32F));	// temp storage

  parallel_for_(Range(0, (int)cas.filters.size() / 2),
                Parallel_process(convt_image, nbApproximatedFilters, cas, curRes));

  for (int idxFilter = 0; idxFilter < cas.filters.size() / 2; idxFilter++) {
      //int idxOrig = 0;
      for (int idxOrig = 0; idxOrig < nbSum * nbMax; ++idxOrig) {
          int idxSum = idxOrig / nbMax;
          int idxMax = idxOrig % nbMax;

          if (idxFilter == 0) {
              res[idxSum][idxMax] =
                  cas.coeffs[idxOrig][idxFilter] *
                  curRes[idxFilter].clone();
            } else {
              res[idxSum][idxMax] =
                  res[idxSum][idxMax] +
                  cas.coeffs[idxOrig][idxFilter] * curRes[idxFilter];
            }

        }
    }

  // add the bias
  int idxOrig = 0;
  for (int idxSum = 0; idxSum < nbSum; ++idxSum) {
      for (int idxMax = 0; idxMax < nbMax; ++idxMax) {
          res[idxSum][idxMax] += cas.bias[idxOrig];
          idxOrig++;
        }
    }

  return res;
}


void getScoresandCombine_Approx(const TILDEobjects & cas,
				const vector < Mat > &convt_image,
				const bool keep_only_positive,
				Mat *output)
{
  const vector < float >param = cas.parameters;
  if (param.size() == 0) {
      throw std::runtime_error("No parameter loaded !");
    }

  int nbMax = param[1];	//4
  int nbSum = param[2];	//4
  int nbOriginalFilters = nbMax * nbSum;
  int nbApproximatedFilters = param[3];	//4
  int nbChannels = param[4];	//6
  int sizeFilters = param[5];	//21
  //--------------------

  *output = Mat::zeros(convt_image[0].size(), CV_32F);

  vector < vector < Mat > >res(nbSum,vector < Mat >(nbMax));

  // calculate separable responses
  int idxSum = 0;
  int idxMax = 0;

  vector < Mat > curRes((int)cas.filters.size() / 2, Mat::zeros(convt_image[0].size(), CV_32F));	// temp storage

  parallel_for_(Range(0, (int)cas.filters.size() / 2),
                Parallel_process(convt_image, nbApproximatedFilters, cas, curRes));


  Mat maxVal;
  int count = 0;
  for (int idxOrig = 0; idxOrig < nbSum * nbMax; ++idxOrig)
    {
      int idxSum = idxOrig / nbMax;
      int idxMax = idxOrig % nbMax;

      Mat result = res[idxSum][idxMax];

      for (int idxFilter = 0; idxFilter < cas.filters.size() / 2; idxFilter++)
        result = result + cas.coeffs[idxOrig][idxFilter] * curRes[idxFilter];

      res[idxSum][idxMax] = result + cas.bias[idxMax + idxSum*nbMax];

      if (idxOrig % nbMax == 0)
        maxVal = res[idxSum][idxMax];
      else
        maxVal = max(res[idxSum][idxMax],maxVal);

      if ((idxOrig+1) % nbMax == 0)//the last one
        {
          // sign and sum
          *output = (idxSum % 2 == 0 ? -maxVal : maxVal) + *output;
        }
    }

  //post process
  const float stdv = 2;
  const int sizeSmooth = 5 * stdv * 2 + 1;
  GaussianBlur(*output, *output, Size(sizeSmooth, sizeSmooth), stdv, stdv);

  if (keep_only_positive)
    *output = max(*output, 0);
}

// --------------------------------------------------------------------------------------


vector < vector < lfilter > >getTILDENonApproxFilters(const string & name, void *_p)
{
  vector < float >*param = (vector < float >*)_p;
  vector < vector < lfilter > >res;

  std::ifstream fic(name, ios::in);
  bool isOpen = fic.is_open();
  if (!isOpen) {
      std::cerr << name << std::endl;
      throw std::runtime_error("Cannot open filters");
    }

  std::string lineread;
  std::vector < std::string > tokens;

  //get parameters
  getline(fic, lineread);
  tokens.clear();
  Tokenize(lineread, tokens);

  if (param != NULL)
    for (int i = 0; i < tokens.size(); i++)
      param->push_back(std::stof(delSpaces(tokens[i])));

  //start processing...
  getline(fic, lineread);
  tokens.clear();
  Tokenize(lineread, tokens);
  if (tokens.size() < 2) {
      throw std::runtime_error("Wrong formating for the filters");
    }

  int nbFilters = std::stoi(delSpaces(tokens[0]));
  int nbChannels = std::stoi(delSpaces(tokens[1]));
  int sizeFilters = std::stoi(delSpaces(tokens[2]));
  int row = 0;

  Mat M(sizeFilters, sizeFilters, CV_32FC1);

  vector < lfilter > myCascade;
  lfilter myfilter;

  while (getline(fic, lineread)) {

      tokens.clear();
      Tokenize(lineread, tokens);

      for (int i = 0; i < sizeFilters; i++)
        M.at < float >(row, i) = std::stof(delSpaces(tokens[i]));

      if (row == sizeFilters - 1) {
          myfilter.push_back(M);

          //reset
          M = Mat(sizeFilters, sizeFilters, CV_32FC1);
          row = 0;
        } else
        row++;

      if (myfilter.size() == nbChannels) {
          //get b
          getline(fic, lineread);
          tokens.clear();
          Tokenize(lineread, tokens);

          myfilter.b = std::stof(delSpaces(tokens[0]));

          myCascade.push_back(myfilter);

          myfilter = lfilter();

        }

      if (myCascade.size() == nbFilters) {
          //push cascade
          res.push_back(myCascade);

          //get next info
          getline(fic, lineread);

          if (fic.fail())	//eof
            return res;

          tokens.clear();
          Tokenize(lineread, tokens);
          if (tokens.size() < 2) {
              throw std::runtime_error("Wrong formating for the filters");
            }

          nbFilters = std::stoi(delSpaces(tokens[0]));
          nbChannels = std::stoi(delSpaces(tokens[1]));
          sizeFilters = std::stoi(delSpaces(tokens[2]));
          //--done

          //init
          M = Mat(sizeFilters, sizeFilters, CV_32FC1);
          myCascade.clear();
        }
    }

  return res;
}

vector < Point3f > applyNonApproxFilters(const Mat & indatav,
					 const vector < vector < lfilter >> &dual_cascade_filters,
					 const vector < float >&param,
					 const bool useDescriptorField, const bool sortMe,
					 const bool keep_only_positive, Mat * score)
{
  const float stdv = 2;
  const int sizeSmooth = 5 * stdv * 2 + 1;

  float resizeRatio = 1.0;
  if (param.size() > 0)
    resizeRatio = param[0];

  if (resizeRatio == 0)
    throw std::runtime_error("The resize ratio is zero, if you dont want any resize, use 1");

  Mat indatav_resized = indatav;
  if (resizeRatio != 1)
    resize(indatav, indatav_resized, Size(0, 0), resizeRatio, resizeRatio);

  vector < Mat > convt_image;
  if (useDescriptorField) {
      convt_image = getNormalizedDescriptorField(indatav);
    } else {
      vector < Mat > gradImage = getGradImage(indatav_resized);
      vector < Mat > luvImage = getLuvImage(indatav_resized);

      copy(gradImage.begin(), gradImage.end(), std::back_inserter(convt_image));
      copy(luvImage.begin(), luvImage.end(), std::back_inserter(convt_image));
    }

  // filter the image using all filters
  float fourierMultiplier =
      dual_cascade_filters[0][0].w[0].rows * dual_cascade_filters[0][0].w[0].cols;
  vector < vector < Mat >> cascade_responses(dual_cascade_filters.size());
  for (int idxCascade = 0; idxCascade < dual_cascade_filters.size(); ++idxCascade) {
      cascade_responses[idxCascade].resize(dual_cascade_filters[idxCascade].size());
      for (int idxDepth = 0; idxDepth < dual_cascade_filters[idxCascade].size();
           ++idxDepth) {
          // current multichannel filter
          lfilter cur_filter = dual_cascade_filters[idxCascade][idxDepth];
          // responses for each channel
          vector < Mat > cur_responses(cur_filter.w.size());
          // perform filtering
          for (int idxChannel = 0; idxChannel < cur_filter.w.size(); ++idxChannel) {
              filter2D(convt_image[idxChannel], cur_responses[idxChannel], -1,
                       cur_filter.w[idxChannel], Point(-1, -1), 0,
                       BORDER_REFLECT);
            }
          // sum the channels up
          Mat cur_response =
              fourierMultiplier * sumMatArray(cur_responses) + cur_filter.b;
          cascade_responses[idxCascade][idxDepth] = cur_response;
        }
    }

  // apply the cascade structure and retrieve single channel response image
  Mat respImageFinal;
  for (int idxCascade = 0; idxCascade < dual_cascade_filters.size(); ++idxCascade) {
      Mat respImageCascade = cascade_responses[idxCascade][0];
      for (int idxDepth = 1; idxDepth < dual_cascade_filters[idxCascade].size();
           ++idxDepth) {
          respImageCascade =
              max(respImageCascade, cascade_responses[idxCascade][idxDepth]);
        }
      respImageCascade = idxCascade % 2 == 0 ? -respImageCascade : respImageCascade;
      if (idxCascade == 0) {
          respImageFinal = respImageCascade;
        } else {
          respImageFinal = respImageCascade + respImageFinal;
        }
    }

  GaussianBlur(respImageFinal, respImageFinal, Size(sizeSmooth, sizeSmooth), stdv, stdv);

  if (keep_only_positive)
    respImageFinal = max(respImageFinal, 0);

  if (score != NULL)
    *score = respImageFinal.clone();

  // perform non-max suppression
  vector < Point3f > res_with_score = NonMaxSup(respImageFinal);

  if (sortMe)
    std::sort(res_with_score.begin(), res_with_score.end(),
              [](const Point3f & a, const Point3f & b) {
        return a.z > b.z;}
    );

  // resize back
  resizeRatio = 1. / resizeRatio;
  for (int i = 0; i < res_with_score.size(); ++i) {
      res_with_score[i].x = res_with_score[i].x * resizeRatio;
      res_with_score[i].y = res_with_score[i].y * resizeRatio;
    }

  return res_with_score;
}


void Tokenize(const std::string & mystring, std::vector < std::string > &tok,
              const std::string & sep, int lp, int p)
{
  lp = mystring.find_first_not_of(sep, p);
  p = mystring.find_first_of(sep, lp);
  if (std::string::npos != p || std::string::npos != lp) {
      tok.push_back(mystring.substr(lp, p - lp));
      Tokenize(mystring, tok, sep, lp, p);
    }
}

std::string delSpaces(std::string & str)
{
  std::stringstream trim;
  trim << str;
  trim >> str;

  return str;
}

Mat convBGR2PlaneWiseRGB(const Mat & in)
{
  Mat res = in.clone();

  int numel = in.rows * in.cols;
  for (int j = 0; j < in.rows; j++) {
      for (int i = 0; i < in.cols; i++) {
          ((float *)res.data)[2 * numel + (j * in.cols + i)] = ((float *)in.data)[3 * (j * in.cols + i) + 0];	// B
          ((float *)res.data)[1 * numel + (j * in.cols + i)] = ((float *)in.data)[3 * (j * in.cols + i) + 1];	// G
          ((float *)res.data)[0 * numel + (j * in.cols + i)] = ((float *)in.data)[3 * (j * in.cols + i) + 2];	// R
        }
    }

  return res;
}

Mat convPlaneWiseRGB2RGB(const Mat & in)
{
  Mat res = in.clone();

  int numel = in.rows * in.cols;
  for (int j = 0; j < in.rows; j++) {
      for (int i = 0; i < in.cols; i++) {
          ((float *)res.data)[3 * (j * in.cols + i) + 0] = ((float *)in.data)[0 * numel + (j * in.cols + i)];	// R
          ((float *)res.data)[3 * (j * in.cols + i) + 1] = ((float *)in.data)[1 * numel + (j * in.cols + i)];	// G
          ((float *)res.data)[3 * (j * in.cols + i) + 2] = ((float *)in.data)[2 * numel + (j * in.cols + i)];	// B
        }
    }

  return res;
}

Mat sumMatArray(const vector < Mat > &MatArray)
{
  Mat res = MatArray[0].clone();

  for (int idxMat = 1; idxMat < MatArray.size(); ++idxMat) {
      res += MatArray[idxMat];
    }

  return res;
}




vector < Mat > getGradImage(const Mat & input_color_image)
{
  if (input_color_image.channels() != 3) {
      throw std::runtime_error("Need a 3-channel image");
    }
  //the output
  vector < Mat > gradImage(3);

  vector < Mat > color_channels(3);
  vector < Mat > gx(3);
  vector < Mat > gy(3);

  // The derivative5 kernels
  Mat d1 = (Mat_ < float >(1, 5) << 0.109604, 0.276691, 0.000000, -0.276691, -0.109604);
  Mat d1T = (Mat_ < float >(5, 1) << 0.109604, 0.276691, 0.000000, -0.276691, -0.109604);
  Mat p = (Mat_ < float >(1, 5) << 0.037659, 0.249153, 0.426375, 0.249153, 0.037659);
  Mat pT = (Mat_ < float >(5, 1) << 0.037659, 0.249153, 0.426375, 0.249153, 0.037659);

  // split the channels into each color channel
  split(input_color_image, color_channels);
  // prepare output
  for (int idxC = 0; idxC < 3; ++idxC) {
      gradImage[idxC].create(color_channels[0].rows, color_channels[0].cols, CV_32F);
    }
  //	return gradImage;

  // for each channel do the derivative 5
  for (int idxC = 0; idxC < 3; ++idxC) {
      sepFilter2D(color_channels[idxC], gx[idxC], CV_32F, d1, p, Point(-1, -1), 0,
                  BORDER_REFLECT);
      sepFilter2D(color_channels[idxC], gy[idxC], CV_32F, p, d1, Point(-1, -1), 0,
                  BORDER_REFLECT);
      // since we do the other direction, just flip signs
      gx[idxC] = -gx[idxC];
      gy[idxC] = -gy[idxC];
    }

  // the magnitude image
  vector < Mat > mag(3);
  for (int idxC = 0; idxC < 3; ++idxC) {
      sqrt(gx[idxC].mul(gx[idxC]) + gy[idxC].mul(gy[idxC]), mag[idxC]);
    }

  // Get Max idx using Magnitude
  Mat maxIdxMat(mag[0].rows, mag[0].cols, CV_32F);
  float curVal, maxVal; int maxIdx;
  for (int j = 0; j < mag[0].rows; j++)
    {
      for (int i = 0; i < mag[0].cols; i++)
        {
          maxIdx = 0;
          maxVal = 0;
          for (int idxC = 0; idxC < 3; ++idxC) {
              curVal = mag[idxC].at < float >(j, i);
              if (maxVal < curVal) {
                  maxIdx = idxC;
                  maxVal = curVal;
                }
            }
          maxIdxMat.at < float >(j, i) = maxIdx;
        }
    }

  int idxC;
  // Select and save the max channel
  for (int j = 0; j < mag[0].rows; j++) {
      for (int i = 0; i < mag[0].cols; i++) {
          idxC = maxIdxMat.at < float >(j, i);
          gradImage[0].at < float >(j, i) = gx[idxC].at < float >(j, i) * 0.5 + 128.0;
          gradImage[1].at < float >(j, i) = gy[idxC].at < float >(j, i) * 0.5 + 128.0;
          gradImage[2].at < float >(j, i) = mag[idxC].at < float >(j, i);
        }
    }

  return gradImage;
}


vector < Mat > getLuvImage(const Mat & input_color_image)
{

  if (input_color_image.channels() != 3) {
      throw std::runtime_error("Need a 3-channnel image");
    }

  Mat Input;

  input_color_image.convertTo(Input, CV_32FC3, 1. / 255.);

  Input = convBGR2PlaneWiseRGB(Input);
  Mat luv(Input.rows, Input.cols, CV_32FC3);

  rgb2luv((float *)(Input.data), (float *)(luv.data), Input.rows * Input.cols, (float)1.f);
  luv = convPlaneWiseRGB2RGB(luv);
  //the output
  //printf("1211\n");
  vector < Mat > luvImage(3);
  split(luv, luvImage);
  for (int idxC = 0; idxC < 3; ++idxC) {
      luvImage[idxC].convertTo(luvImage[idxC], CV_32F);
      luvImage[idxC] *= 270.0;	//revert dollar's conversion

      switch (idxC) {
        case 0:	// L
          luvImage[idxC] *= 2.55;
          break;
        case 1:	// U
          luvImage[idxC] -= 88.0;	//revert dollar's conversion
          luvImage[idxC] = (luvImage[idxC] + 134.0) * 255.0 / 354.0;
          break;
        case 2:	// V
          luvImage[idxC] -= 134.0;	//revert dollar's conversion
          luvImage[idxC] = (luvImage[idxC] + 140.0) * 255.0 / 256.0;
          break;
        }
    }

  return luvImage;
}



void ComputeImageDerivatives(const cv::Mat & image, cv::Mat & imageDx, cv::Mat & imageDy)
{
  int ddepth = -1;	//same image depth as source
  double scale = 1 / 32.0;	// normalize wrt scharr mask for having exact gradient
  double delta = 0;

  Scharr(image, imageDx, ddepth, 1, 0, scale, delta, BORDER_REFLECT);
  Scharr(image, imageDy, ddepth, 0, 1, scale, delta, BORDER_REFLECT);
}

void NormalizeImage(Mat & image)
{
  Scalar mean, stddev;
  meanStdDev(image, mean, stddev);
  image = (image - mean) / stddev[0];
}

vector < Mat > getNormalizedDescriptorField(const Mat & im)
{
  Mat dx, dy;
  ComputeImageDerivatives(im, dx, dy);
  assert(dx.isContinuous());
  assert(dy.isContinuous());

  Size imSize = im.size();
  Mat dxPos(imSize, CV_32F, Scalar(0));
  Mat dxNeg(imSize, CV_32F, Scalar(0));
  Mat dyPos(imSize, CV_32F, Scalar(0));
  Mat dyNeg(imSize, CV_32F, Scalar(0));

  float dxPixel, dyPixel;

  for (int iRow(0); iRow < im.rows; ++iRow) {
      for (int iCol(0); iCol < im.cols; ++iCol) {
          dxPixel = ((float *)dx.data)[dx.cols * iRow + iCol];
          dyPixel = ((float *)dy.data)[dx.cols * iRow + iCol];

          if (dxPixel > 0)
            ((float *)dxPos.data)[dx.cols * iRow + iCol] = 10 * dxPixel;	//10 is just a factor for numerical stability, with no particular meaning
          else
            ((float *)dxNeg.data)[dx.cols * iRow + iCol] = -10 * dxPixel;

          if (dyPixel > 0)
            ((float *)dyPos.data)[dx.cols * iRow + iCol] = 10 * dyPixel;
          else
            ((float *)dyNeg.data)[dx.cols * iRow + iCol] = -10 * dyPixel;
        }
    }
  vector < Mat > channels;
  channels.push_back(dxPos);
  channels.push_back(dxNeg);
  channels.push_back(dyPos);
  channels.push_back(dyNeg);

  //return channels;
  for (uint i = 0; i < channels.size(); ++i)
    NormalizeImage(channels[i]);

  return channels;
}

TILDEobjects getTILDEObject(const string & name, void *_p, bool useApprox, bool useDescriptorField)
{
  TILDEobjects res;

  if (useApprox) {
      res = getTILDEApproxObjects(name, _p);
    } else {
      res.nonApprox_filters = getTILDENonApproxFilters(name, _p);
    }

  res.name = name;
  res.isApprox = useApprox;
  res.useDescriptorField = useDescriptorField;
  return res;
}


TILDEobjects getTILDEApproxObjects(const string & name, void *_p)
{
  TILDEobjects res;

  vector < float >*param = (vector < float >*)_p;

  std::ifstream fic(name, ios::in);
  bool isOpen = fic.is_open();
  if (!isOpen) {
      throw std::runtime_error("Cannot open filter");
    }

  std::string lineread;
  std::vector < std::string > tokens;

  //get parameters
  getline(fic, lineread);
  tokens.clear();
  Tokenize(lineread, tokens);

  if (param != NULL) {	//load param 1st lines
      for (int i = 0; i < tokens.size(); i++) {
          param->push_back(std::stof(delSpaces(tokens[i])));
        }
    } else {		// just push it on the parameters
      for (int i = 0; i < tokens.size(); i++) {
          res.parameters.push_back(std::stof(delSpaces(tokens[i])));
        }
    }

  //start processing...
  getline(fic, lineread);
  tokens.clear();
  Tokenize(lineread, tokens);
  if (tokens.size() != 5) {
      throw std::runtime_error("Filter not compatible");

    }
  int nbMax = std::stoi(delSpaces(tokens[0]));
  int nbSum = std::stoi(delSpaces(tokens[1]));
  int nbOriginalFilters = nbMax * nbSum;
  int nbApproximatedFilters = std::stoi(delSpaces(tokens[2]));
  int nbChannels = std::stoi(delSpaces(tokens[3]));
  int sizeFilters = std::stoi(delSpaces(tokens[4]));

  if (param != NULL)	//load param 1st lines
    {
      param->push_back(nbMax);
      param->push_back(nbSum);
      param->push_back(nbApproximatedFilters);
      param->push_back(nbChannels);
      param->push_back(sizeFilters);
      res.parameters = *param;
    } else {
      res.parameters.push_back(nbMax);
      res.parameters.push_back(nbSum);
      res.parameters.push_back(nbApproximatedFilters);
      res.parameters.push_back(nbChannels);
      res.parameters.push_back(sizeFilters);
    }
  //--------------------

  //get bias
  getline(fic, lineread);
  tokens.clear();
  Tokenize(lineread, tokens);
  if (tokens.size() != nbOriginalFilters) {
      throw std::runtime_error("Wrong number of cascades");
    }
  //bias
  res.bias = vector < float >(nbOriginalFilters);
  for (int i = 0; i < tokens.size(); i++)
    res.bias[i] = std::stof(delSpaces(tokens[i]));


  //coeffs
  res.coeffs = vector < vector < float >>(nbOriginalFilters,
                                          vector <
                                          float >(nbApproximatedFilters * nbChannels));
  int row = 0;
  while (getline(fic, lineread)) {
      tokens.clear();
      Tokenize(lineread, tokens);
      for (int i = 0; i < nbApproximatedFilters * nbChannels; i++)
        res.coeffs[row][i] = std::stof(delSpaces(tokens[i]));

      if (++row == nbOriginalFilters)
        break;
    }
  //-------------

  //filters
  res.filters = vector < Mat > (nbApproximatedFilters * nbChannels * 2,
                                Mat(1, sizeFilters, CV_32FC1));
  row = 0;
  while (getline(fic, lineread)) {
      tokens.clear();
      Tokenize(lineread, tokens);

      vector < float >r(sizeFilters);
      for (int i = 0; i < sizeFilters; i++)
        r[i] = std::stof(delSpaces(tokens[i]));

      res.filters[row] = Mat(r).clone();

      if (++row == nbApproximatedFilters * nbChannels * 2)
        break;
    }

  return res;
}


//
// libTILDE.cpp ends here
//#include <sys/time.h>
//inline long getMilliSecs1()
//{
//  timeval t;
//  gettimeofday(&t, NULL);
//  return t.tv_sec*1000 + t.tv_usec/1000;
//}

Mat getTILDEResponce(
    const Mat & indatav,
    const string & nameFilter,
    const bool useApprox,
    const bool keepPositiveScoreOnly) {

  cv::Mat resp;
  bool bUseDescriptorField = false; // disabled by default - for

//   double start = (double) getMilliSecs1();

  cv::Mat img = indatav.clone();//we copy the input data here, because we will resize it before filtering
  // Read the txt file to get the filter
  vector < float > param;
  TILDEobjects  tilde_obj = getTILDEObject(nameFilter, &param,  useApprox, bUseDescriptorField);
//std::cerr <<   ((double) getMilliSecs1() - start)/1000.0 << " sec for load" << std::endl;

  // Apply filtering
  // NOTE: score is CV_32FC1

  if (useApprox) {
      float resizeRatio = 1.0;
      vector < Mat > convt_image;
      prepareData(img,resizeRatio, bUseDescriptorField,&convt_image);

      vector < vector < Mat > >cascade_responses = getScoresForApprox(tilde_obj, convt_image);

      // apply the cascade structure and retrieve single channel response image
      getCombinedScore(cascade_responses, keepPositiveScoreOnly, &resp);

    } else {
      float resizeRatio = 1.0;

      if (param.size() > 0)
        resizeRatio = param[0];

      if (resizeRatio == 0)
        throw std::runtime_error("The resize ratio is zero, if you dont want any resize, use 1");


      const float stdv = 2.0;
      const int sizeSmooth = 5 * stdv * 2 + 1;

      Mat indatav_resized = img;
      if (resizeRatio != 1)
        resize(indatav, indatav_resized, Size(0, 0), resizeRatio, resizeRatio);

      vector < Mat > convt_image;

      if (bUseDescriptorField) {
          convt_image = getNormalizedDescriptorField(img);

        } else {
          vector < Mat > gradImage = getGradImage(indatav_resized);
          vector < Mat > luvImage = getLuvImage(indatav_resized);
          copy(gradImage.begin(), gradImage.end(), std::back_inserter(convt_image));
          copy(luvImage.begin(), luvImage.end(), std::back_inserter(convt_image));
        }


      // filter the image using all filters
      float fourierMultiplier =
          tilde_obj.nonApprox_filters[0][0].w[0].rows * tilde_obj.nonApprox_filters[0][0].w[0].cols;
      vector < vector < Mat >> cascade_responses(tilde_obj.nonApprox_filters.size());
      for (int idxCascade = 0; idxCascade < tilde_obj.nonApprox_filters.size(); ++idxCascade) {
          cascade_responses[idxCascade].resize(tilde_obj.nonApprox_filters[idxCascade].size());
          for (int idxDepth = 0; idxDepth < tilde_obj.nonApprox_filters[idxCascade].size();
               ++idxDepth) {
              // current multichannel filter
              lfilter cur_filter = tilde_obj.nonApprox_filters[idxCascade][idxDepth];
              // responses for each channel
              vector < Mat > cur_responses(cur_filter.w.size());
              // perform filtering
              for (int idxChannel = 0; idxChannel < cur_filter.w.size(); ++idxChannel) {
                  filter2D(convt_image[idxChannel], cur_responses[idxChannel], -1,
                           cur_filter.w[idxChannel], Point(-1, -1), 0,
                           BORDER_REFLECT);
                }
              // sum the channels up
              Mat cur_response =
                  fourierMultiplier * sumMatArray(cur_responses) + cur_filter.b;
              cascade_responses[idxCascade][idxDepth] = cur_response;
            }
        }

      // apply the cascade structure and retrieve single channel response image
      for (int idxCascade = 0; idxCascade < tilde_obj.nonApprox_filters.size(); ++idxCascade) {
          Mat respImageCascade = cascade_responses[idxCascade][0];
          for (int idxDepth = 1; idxDepth < tilde_obj.nonApprox_filters[idxCascade].size();
               ++idxDepth) {
              respImageCascade =
                  max(respImageCascade, cascade_responses[idxCascade][idxDepth]);
            }
          respImageCascade = idxCascade % 2 == 0 ? -respImageCascade : respImageCascade;
          if (idxCascade == 0) {
              resp = respImageCascade;
            } else {
              resp = respImageCascade + resp;
            }
        }

    //  GaussianBlur(resp, resp, Size(sizeSmooth, sizeSmooth), stdv, stdv);

      if (keepPositiveScoreOnly)
        resp = max(resp, 0);

      if (resizeRatio != 1)
        resize(resp, resp, Size(0, 0), 1.0/resizeRatio, 1.0/resizeRatio);

    }

//  resp = normalizeScore(resp);
  return resp;
}
