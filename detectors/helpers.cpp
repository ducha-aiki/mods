/*
 * Copyright (C) 2008-12 Michal Perdoch
 * All rights reserved.
 *
 * This file is part of the HessianAffine detector and is made available under
 * the terms of the BSD license (see the COPYING file).
 *
 */

#undef __STRICT_ANSI__
#include <helpers.h>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <iostream>
using cv::Mat;
using namespace std;

template <typename ValueType>
void swap(ValueType *a, ValueType *b)
{
  ValueType tmp = *a;
  *a = *b;
  *b = tmp;
}
#define CONST1	1.05839816339744830962 //M_PI/4 + 0.273

const double M_PI_255 = 255.0/M_PI;

const double ATAN_LUT[256] = {0.0000000000,0.0039215485,0.0078429764,0.0117641631,0.0156849881,0.0196053309,
                              0.0235250710,0.0274440882,0.0313622624,0.0352794736,0.0391956019,0.0431105278,
                              0.0470241318,0.0509362949,0.0548468980,0.0587558227,0.0626629506,0.0665681638,
                              0.0704713446,0.0743723758,0.0782711405,0.0821675224,0.0860614053,0.0899526737,
                              0.0938412126,0.0977269074,0.1016096438,0.1054893085,0.1093657884,0.1132389710,
                              0.1171087446,0.1209749978,0.1248376255,0.1286965013,0.1325515323,0.1364026044,
                              0.1402496096,0.1440924408,0.1479309912,0.1517651553,0.1555948280,0.1594199049,
                              0.1632402828,0.1670558588,0.1708665312,0.1746721990,0.1784727620,0.1822681208,
                              0.1860581771,0.1898428334,0.1936219929,0.1973955598,0.2011634395,0.2049255380,
                              0.2086817623,0.2124320205,0.2161762215,0.2199142752,0.2236460927,0.2273715857,
                              0.2310906672,0.2348032511,0.2385092525,0.2422085871,0.2459011721,0.2495869254,
                              0.2532657662,0.2569376146,0.2606023917,0.2642600199,0.2679104224,0.2715535237,
                              0.2751892491,0.2788175253,0.2824382800,0.2860514417,0.2896569404,0.2932547070,
                              0.2968446734,0.3004267728,0.3040009393,0.3075671084,0.3111252164,0.3146752558,
                              0.3182170002,0.3217505544,0.3252758042,0.3287926915,0.3323011594,0.3358011520,
                              0.3392926145,0.3427754932,0.3462497357,0.3497152904,0.3531721069,0.3566201360,
                              0.3600593294,0.3634896400,0.3669110217,0.3703234297,0.3737268255,0.3771211497,
                              0.3805063771,0.3838824615,0.3872493632,0.3906070437,0.3939554653,0.3972945915,
                              0.4006243869,0.4039448169,0.4072558481,0.4105574480,0.4138495853,0.4171322295,
                              0.4204053512,0.4236689219,0.4269229141,0.4301673014,0.4334020581,0.4366271598,
                              0.4398425828,0.4430483044,0.4462443029,0.4494305575,0.4526070482,0.4557737560,
                              0.4589306629,0.4620777516,0.4652150058,0.4683424102,0.4714599501,0.4745676117,
                              0.4776653824,0.4807532499,0.4838312032,0.4868992318,0.4899573263,0.4930054778,
                              0.4960436784,0.4990719209,0.5020901990,0.5050985071,0.5080968402,0.5110851942,
                              0.5140635659,0.5170319525,0.5199903521,0.5229387636,0.5258771863,0.5288056206,
                              0.5317240673,0.5346325278,0.5375310045,0.5404195003,0.5432980185,0.5461665634,
                              0.5490251398,0.5518737530,0.5547124091,0.5575411147,0.5603598769,0.5631687036,
                              0.5659676030,0.5687565842,0.5715356566,0.5743048302,0.5770641155,0.5798135236,
                              0.5825530662,0.5852827553,0.5880026035,0.5907126240,0.5934128303,0.5961032364,
                              0.5987838570,0.6014547069,0.6041158015,0.6067671569,0.6094087892,0.6120407151,
                              0.6146629519,0.6172755171,0.6198784285,0.6224717045,0.6250553640,0.6276294258,
                              0.6301939095,0.6327488350,0.6352942223,0.6378300921,0.6403564651,0.6428733625,
                              0.6453808058,0.6478788169,0.6503674179,0.6528466311,0.6553164793,0.6577769856,
                              0.6602281731,0.6626700655,0.6651026865,0.6675260602,0.6699402110,0.6723451634,
                              0.6747409422,0.6771275725,0.6795050796,0.6818734889,0.6842328261,0.6865831172,
                              0.6889243882,0.6912566655,0.6935799756,0.6958943450,0.6981998008,0.7004963699,
                              0.7027840796,0.7050629571,0.7073330300,0.7095943260,0.7118468729,0.7140906986,
                              0.7163258312,0.7185522990,0.7207701302,0.7229793534,0.7251799971,0.7273720900,
                              0.7295556609,0.7317307387,0.7338973524,0.7360555311,0.7382053040,0.7403467003,
                              0.7424797493,0.7446044805,0.7467209234,0.7488291075,0.7509290624,0.7530208178,
                              0.7551044035,0.7571798492,0.7592471847,0.7613064400,0.7633576449,0.7654008294,
                              0.7674360235,0.7694632573,0.7714825607,0.7734939638,0.7754974968,0.7774931897,
                              0.7794810727,0.7814611759,0.7834335294,0.7853981634};
double atan2LUTif(double y,double x) //D. Mishkin
{
  double absx, absy, val;

  if (x == 0 && y == 0) {
    return 0;
  }
  absy = fabs(y);
  absx = fabs(x);
  if (absy - absx == absy) {
    /* x negligible compared to y */
    return y < 0 ? -M_PI_2 : M_PI_2;
  }
  if (absx - absy == absx) {
    /* y negligible compared to x */
    val = 0.0;
  }
  else
  {
    if (y>0) {
      if (absx > absy)
        val = ATAN_LUT[(int)(255*absy/absx)];//1st octant
      else
        val = M_PI_2 - ATAN_LUT[(int)(255*absx/absy)];//2nd octant
      val = x < 0 ? (M_PI - val) : val; //3-4th octants from 2-1
    }
    else {
      if (absx > absy)
        val = -ATAN_LUT[(int)(255*absy/absx)];//8th octant
      else
        val = -M_PI_2 + ATAN_LUT[(int)(255*absx/absy)];//7th octant
      val = x < 0 ? -M_PI - val : val; //5-6th octants from 8-7
    }
  }

  return val;

}
double atan2LUT(double y,double x) //D. Mishkin
{
  double absx, absy;
  absy = fabs(y);//faster than y < 0 ? -y : y;
  absx = fabs(x);//faster than x < 0 ? -x : x;
  short octant = ((x<0) << 2) + ((y<0) << 1 ) + (absx <= absy);
  switch (octant) {
    case 0: {
      if (x == 0 && y == 0)
        return 0;
      return ATAN_LUT[(int)(255*absy/absx)]; //1st octant
      break;
    }
    case 1:{
      if (x == 0 && y == 0)
        return 0.0;
      return M_PI_2 - ATAN_LUT[(int)(255*absx/absy)]; //2nd octant
      break;
    }
    case 2: {
      return -ATAN_LUT[(int)(255*absy/absx)]; //8th octant
      break;
    }
    case 3: {
      return -M_PI_2 + ATAN_LUT[(int)(255*absx/absy)];//7th octant
      break;
    }
    case 4: {
      return  M_PI - ATAN_LUT[(int)(255*absy/absx)];  //4th octant
    }
    case 5: {
      return  M_PI_2 + ATAN_LUT[(int)(255*absx/absy)];//3rd octant
      break;
    }
    case 6: {
      return -M_PI + ATAN_LUT[(int)(255*absy/absx)]; //5th octant
      break;
    }
    case 7: {
      return -M_PI_2 - ATAN_LUT[(int)(255*absx/absy)]; //6th octant
      break;
    }
    default:
      return 0.0;
  }
}
# define M_PI_2f		1.57079632679489661923f	/* pi/2 */
# define M_PIf		3.14159265358979323846f	/* pi */

float atan2LUTff(float y,float x) //D. Mishkin
{
  //float absx, absy;
  //absy = fabs(y);//faster than y < 0 ? -y : y;
  //absx = fabs(x);//faster than x < 0 ? -x : x;
  if (x > 0.f) {
    if (y > 0.f) {
      if (x > y) {
        // 1st
        return ATAN_LUT[(int)(255.f*y/x)]; //1st octant
      } else {
        // 2nd
        return M_PI_2f - ATAN_LUT[(int)(255*x/y)]; //2nd octant
      }
    } else {
      float absy = fabs(y);
      if (x > absy) {
        // 8th
        return -ATAN_LUT[(int)(255.f*absy/x)]; //8th octant
      } else {
        // 7th
        return -M_PI_2f + ATAN_LUT[(int)(255.f*x/absy)];//7th octant
      }
    }
  } else if (y > 0.f) {
    float absx = fabs(x);
    if (absx > y) {
      // 4th
      return  M_PIf - ATAN_LUT[(int)(255.f*y/absx)];  //4th octant
    } else {
      // 3rd
      return  M_PI_2f + ATAN_LUT[(int)(255.f*absx/y)];//3rd octant
    }
  } else {
    float absx = fabs(x);
    float absy = fabs(y);
    if (absx > absy) {
      // 5th
      return -M_PIf + ATAN_LUT[(int)(255.f*absy/absx)]; //5th octant
    } else {
      // 6th
      if (x == 0.f)
        return 0.f;
      return -M_PI_2f - ATAN_LUT[(int)(255.f*absx/absy)]; //6th octant
    }
  }
  return 0.f;
}
double atan2approx(double y,double x)  //bad, for test only, atan(x) ~ x*pi/4, x=[-1; 1]
{
  double absx, absy;
  absy = fabs(y);//faster than y < 0 ? -y : y;
  absx = fabs(x);//faster than x < 0 ? -x : x;
  short octant = ((x<0) << 2) + ((y<0) << 1 ) + (absx <= absy);
  switch (octant) {
    case 0: {
      if (x == 0 && y == 0)
        return 0;
      return M_PI_4*absy/absx; //1st octant
      break;
    }
    case 1:{
      if (x == 0 && y == 0)
        return 0.0;
      return M_PI_2 - M_PI_4*absx/absy; //2nd octant
      break;
    }
    case 2: {
      return -M_PI_4*absy/absx; //8th octant
      break;
    }
    case 3: {
      return -M_PI_2 + M_PI_4*absx/absy;//7th octant
      break;
    }
    case 4: {
      return  M_PI - M_PI_4*absy/absx;  //4th octant
    }
    case 5: {
      return  M_PI_2 + M_PI_4*absx/absy;//3rd octant
      break;
    }
    case 6: {
      return -M_PI + M_PI_4*absy/absx; //5th octant
      break;
    }
    case 7: {
      return -M_PI_2 - M_PI_4*absx/absy; //6th octant
      break;
    }
    default:
      return 0.0;
  }
}

double atan2approx2(double y,double x)  //not bad, atan(x) ~ x*(pi/4+0.273-0.273*x), x=[0; 1]
{
  double absx, absy;
  absy = fabs(y);//faster than y < 0 ? -y : y;
  absx = fabs(x);//faster than x < 0 ? -x : x;
  short octant = ((x<0) << 2) + ((y<0) << 1 ) + (absx <= absy);
  switch (octant) {
    case 0: {
      if (x == 0 && y == 0)
        return 0;
      double val = absy/absx;
      return (CONST1 - 0.273*val)*val; //1st octant
      break;
    }
    case 1:{
      if (x == 0 && y == 0)
        return 0.0;
      double val = absx/absy;
      return M_PI_2 - (CONST1 - 0.273*val)*val; //2nd octant
      break;
    }
    case 2: {
      double val =absy/absx;
      return -(CONST1 - 0.273*val)*val; //8th octant
      break;
    }
    case 3: {
      double val =absx/absy;
      return -M_PI_2 + (CONST1 - 0.273*val)*val;//7th octant
      break;
    }
    case 4: {
      double val =absy/absx;
      return  M_PI - (CONST1 - 0.273*val)*val;  //4th octant
    }
    case 5: {
      double val =absx/absy;
      return  M_PI_2 + (CONST1 - 0.273*val)*val;//3rd octant
      break;
    }
    case 6: {
      double val =absy/absx;
      return -M_PI + (CONST1 - 0.273*val)*val; //5th octant
      break;
    }
    case 7: {
      double val =absx/absy;
      return -M_PI_2 - (CONST1 - 0.273*val)*val; //6th octant
      break;
    }
    default:
      return 0.0;
  }
}
void solveLinear3x3(float *A, float *b)
{
  // find pivot of first column
  int i = 0;
  float *pr = A;
  float vp = abs(A[0]);
  float tmp = abs(A[3]);
  if (tmp > vp)
  {
    // pivot is in 1st row
    pr = A+3;
    i = 1;
    vp = tmp;
  }
  if (abs(A[6]) > vp)
  {
    // pivot is in 2nd row
    pr = A+6;
    i = 2;
  }

  // swap pivot row with first row
  if (pr != A)
  {
    swap(pr, A);
    swap(pr+1, A+1);
    swap(pr+2, A+2);
    swap(b+i, b);
  }

  // fixup elements 3,4,5,b[1]
  vp = A[3] / A[0];
  A[4] -= vp*A[1];
  A[5] -= vp*A[2];
  b[1] -= vp*b[0];

  // fixup elements 6,7,8,b[2]]
  vp = A[6] / A[0];
  A[7] -= vp*A[1];
  A[8] -= vp*A[2];
  b[2] -= vp*b[0];

  // find pivot in second column
  if (abs(A[4]) < abs(A[7]))
  {
    swap(A+7, A+4);
    swap(A+8, A+5);
    swap(b+2, b+1);
  }

  // fixup elements 7,8,b[2]
  vp = A[7] / A[4];
  A[8] -= vp*A[5];
  b[2] -= vp*b[1];

  // solve b by back-substitution
  b[2] = (b[2]                    )/A[8];
  b[1] = (b[1]-A[5]*b[2]          )/A[4];
  b[0] = (b[0]-A[2]*b[2]-A[1]*b[1])/A[0];
}
//
//void rectifyAffineTransformationUpIsUpF(float &a11, float &a12, float &a21, float &a22)
//{
//  double a = a11, b = a12, c = a21, d = a22;
//  double det = sqrt(abs(a*d-b*c));
//  double b2a2 = sqrt(b*b + a*a);
//  a11 = b2a2/det;
//  a12 = 0;
//  a21 = (d*b+c*a)/(b2a2*det);
//  a22 = det/b2a2;
//}
void rectifyAffineTransformationUpIsUp(float &a11, float &a12, float &a21, float &a22)
{
  double a = a11, b = a12, c = a21, d = a22;
  double det = sqrt(abs(a*d-b*c));
  double b2a2 = sqrt(b*b + a*a);
  a11 = b2a2/det;
  a12 = 0;
  a21 = (d*b+c*a)/(b2a2*det);
  a22 = det/b2a2;
}

//void rectifyAffineTransformationUpIsUp(float *U)
//{
//  rectifyAffineTransformationUpIsUp(U[0], U[1], U[2], U[3]);
//}

void rectifyAffineTransformationUpIsUp(double *U)
{
  rectifyAffineTransformationUpIsUp(U[0], U[1], U[2], U[3]);
}

void rectifyAffineTransformationUpIsUp(double &a11, double &a12, double &a21, double &a22)
{
  double a = a11, b = a12, c = a21, d = a22;
  double det = sqrt(abs(a*d-b*c));
  double b2a2 = sqrt(b*b + a*a);
  a11 = b2a2/det;
  a12 = 0;
  a21 = (d*b+c*a)/(b2a2*det);
  a22 = det/b2a2;
}
void computeGaussMask(Mat &mask)
{
  int size = mask.cols;
  int halfSize = size >> 1;
  // fit 3*sigma into half_size
  float scale = float(halfSize)/3.0f;

  float scale2 = -2.0f * scale * scale;
  float *tmp = new float[halfSize+1];
  for (int i = 0; i<= halfSize; i++)
    tmp[i] = exp((float(i*i)/scale2));

  int endSize = int(ceil(scale*5.0f)-halfSize);
  for (int i = 1; i< endSize; i++)
    tmp[halfSize-i] += exp((float((i+halfSize)*(i+halfSize))/scale2));

  for (int i=0; i<=halfSize; i++)
  { float *maskPtr_i_hS = mask.ptr<float>(i+halfSize);
    float *maskPtr_mi_hS = mask.ptr<float>(-i+halfSize);

    for (int j=0; j<=halfSize; j++)
    {
      maskPtr_i_hS[-j+halfSize] =
      maskPtr_mi_hS[ j+halfSize] =
      maskPtr_i_hS[ j+halfSize] =
      maskPtr_mi_hS[-j+halfSize] = tmp[i]*tmp[j];
    }
  }
  delete [] tmp;
}

void computeCircularGaussMask(Mat &mask, float sigma)
{
  int size = mask.cols;
  int halfSize = size >> 1;
  float r2 = float(halfSize * halfSize);
  float sigma2;
  if (sigma == 0)
    sigma2 = 0.9f*r2;
  else
    sigma2 = 2*sigma*sigma;
  float disq;
  float *mp = mask.ptr<float>(0);
  for(int i=0; i<mask.rows; i++)
    for(int j=0; j<mask.cols; j++)
    {
      disq = float((i-halfSize)*(i-halfSize)+(j-halfSize)*(j-halfSize));
      *mp++ = (disq < r2) ? exp(- disq / sigma2) : 0;
//      *mp++ = exp(- disq / sigma2);
    }
}

void invSqrt(float &a, float &b, float &c, float &l1, float &l2)
{
  double t, r;
  if (b != 0)
  {
    r = double(c-a)/(2*b);
    if (r>=0) t = 1.0/(r+::sqrt(1+r*r));
    else t = -1.0/(-r+::sqrt(1+r*r));
    r = 1.0/::sqrt(1+t*t); /* c */
    t = t*r;               /* s */
  }
  else
  {
    r = 1;
    t = 0;
  }
  double x,z,d;

  x = 1.0/sqrt(r*r*a-2*r*t*b+t*t*c);
  z = 1.0/sqrt(t*t*a+2*r*t*b+r*r*c);

  d = sqrt(x*z);
  x /= d;
  z /= d;
  // let l1 be the greater eigenvalue
  if (x < z)
  {
    l1 = float(z);
    l2 = float(x);
  }
  else
  {
    l1 = float(x);
    l2 = float(z);
  }
  // output square root
  a = float( r*r*x+t*t*z);
  b = float(-r*t*x+t*r*z);
  c = float( t*t*x+r*r*z);
}

bool getEigenvalues(float a, float b, float c, float d, float &l1, float &l2)
{
  float trace = a+d;
  float delta1 = (trace*trace-4*(a*d-b*c));
  if (delta1 < 0)
    return false;
  float delta = sqrt(delta1);

  l1 = (trace+delta)/2.0f;
  l2 = (trace-delta)/2.0f;
  return true;
}

bool interpolateCheckBorders(const cv::Mat &im,const float ofsx,const float ofsy,
                             const float a11, const float a12,const float a21,const float a22, const cv::Mat &res)
{
  return interpolateCheckBorders(im.cols, im.rows, ofsx,ofsy,a11, a12, a21, a22, res.cols, res.rows);

}
// check if we are not too close to boundary of the image/
bool interpolateCheckBorders(const int orig_img_w, const int orig_img_h, const float ofsx,const float ofsy,
                             const float a11, const float a12,const float a21,const float a22, const int res_w, const int res_h)
{
  const int width = orig_img_w - 2;
  const int height = orig_img_h - 2;
  const float halfWidth  =  ceil((float)res_w / 2.0);
  const float halfHeight = ceil((float)res_h / 2.0);
  float x[4];
  x[0] = -halfWidth;
  x[1] = -halfWidth;
  x[2] = +halfWidth;
  x[3] = +halfWidth;
  float y[4];
  y[0] = -halfHeight;
  y[1] = +halfHeight;
  y[2] = -halfHeight;
  y[3] = +halfHeight;
  for (int i=0; i<4; i++)
  {
    float imx = ofsx + x[i]*a11 + y[i]*a12;
    float imy = ofsy + x[i]*a21 + y[i]*a22;
    if (floor(imx) <= 0 || floor(imy) <= 0 || ceil(imx) >= width || ceil(imy) >= height)
      return true;
  }
  return false;
}

bool interpolate(const Mat &im,const float ofsx,const float ofsy,
                 const float a11,const float a12,const float a21,const float a22, Mat &res)
{
  bool ret = false;
  // input size (-1 for the safe bilinear interpolation)
  const int width =   im.cols - 1;
  const int height =  im.rows - 1;
  // output size
  const int halfWidth  = res.cols / 2;
  const int halfHeight = res.rows / 2;
  float *out = res.ptr<float>(0);

  float rx = ofsx - (float)halfHeight * a12;
  float ry = ofsy - (float)halfHeight * a22;
  bool touch_boundary = interpolateCheckBorders(im,ofsx,ofsy,a11,a12,a21,a22,res);
  if (!touch_boundary)
  {
    for (int j=-halfHeight; j<res.rows - halfHeight ; ++j)
    {
      float WX = rx - (float)halfWidth*a11;
      float WY = ry - (float)halfWidth*a21;
      for(int i=-halfWidth; i<res.cols - halfWidth; ++i)
      {
        const int x = (int) (WX);
        const int y = (int) (WY);
        // compute weights
        const float wx = WX - (float)x;
        //  bilinear interpolation
        const float* Row0= im.ptr<float>(y);
        const float* Row1= im.ptr<float>(y+1);
        const float I1 = wx * (Row0[x+1] - Row0[x]) + Row0[x];
        *out++ =  (WY - y)*(wx * (Row1[x+1] - Row1[x]) + Row1[x]-I1)+I1;
        WX += a11;
        WY += a21;
      }
      rx +=a12;
      ry +=a22;
    }
  }
  else
  {
    for (int j=-halfHeight; j< res.rows - halfHeight ; ++j)
    {
      float WX = rx - halfWidth*a11;
      float WY = ry - halfWidth*a21;
     for(int i=-halfWidth; i<res.cols - halfWidth; ++i)
      {
//        const int x = (int) (WX);
//        const int y = (int) (WY);
        const int x = (int) floor(WX);
        const int y = (int) floor(WY);

        if (WX>= 0 && WY>= 0 && x < width && y < height)
        {
          // compute weights
          const float wx = WX - x;
          //  bilinear interpolation
          const float* Row0= im.ptr<float>(y);
          const float* Row1= im.ptr<float>(y+1);
          const float I1 = wx * (Row0[x+1] - Row0[x]) + Row0[x];
          *out++ =  (WY - y)*(wx * (Row1[x+1] - Row1[x]) + Row1[x]-I1)+I1;
        }
        else
        {
          *out++ = 0;
          ret =  true; // touching boundary of the input
        }
        WX += a11;
        WY += a21;
      }
      rx +=a12;
      ry +=a22;
    }
  }
  return ret;
}
/*
//original one (easy to read, not optimized)
bool interpolate(const Mat &im, float ofsx, float ofsy, float a11, float a12, float a21, float a22, Mat &res)
{
  bool ret = false;
  // input size (-1 for the safe bilinear interpolation)
  const int width = im.cols-1;
  const int height = im.rows-1;
  // output size
  const int halfWidth  = res.cols >> 1;
  const int halfHeight = res.rows >> 1;
  float *out = res.ptr<float>(0);
  for (int j=-halfHeight; j<=halfHeight; ++j)
  {
     const float rx = ofsx + j * a12;
     const float ry = ofsy + j * a22;
     for(int i=-halfWidth; i<=halfWidth; ++i)
     {
        float wx = rx + i * a11;
        float wy = ry + i * a21;
        const int x = (int) floor(wx);
        const int y = (int) floor(wy);
        if (x >= 0 && y >= 0 && x < width && y < height)
        {
           // compute weights
           wx -= x; wy -= y;
           // bilinear interpolation
           *out++ =
              (1.0f - wy) * ((1.0f - wx) * im.at<float>(y,x)   + wx * im.at<float>(y,x+1)) +
              (       wy) * ((1.0f - wx) * im.at<float>(y+1,x) + wx * im.at<float>(y+1,x+1));
        } else {
           *out++ = 0;
           ret =  true; // touching boundary of the input
        }
     }
  }
  return ret;
}
*/
void photometricallyNormalize(Mat &image, const Mat &binaryMask, float &sum, float &var)
{
  const int width = image.cols;
  const int height = image.rows;
  sum=0;
  float gsum=0;

  for (int j=0; j < height; j++)
  {
    const float* binaryMaskRow = binaryMask.ptr<float>(j);
    const float* imgRow = image.ptr<float>(j);
    for (int i=0; i < width; i++)
    {
      if (binaryMaskRow[i]>0)
      {
        sum += imgRow[i];
        gsum ++;
      }
    }
  }
  sum = sum / gsum;

  var=0;
  for (int j=0; j < height; j++)
  {
    const float* binaryMaskRow = binaryMask.ptr<float>(j);
    const float* imgRow = image.ptr<float>(j);
    for (int i=0; i < width; i++)
    {
      if (binaryMaskRow[i]>0)
        var += (sum - imgRow[i])*(sum - imgRow[i]);
    }
  }
  var = ::sqrt(var / gsum);
  if (var < 0.0001)
    // if variance is too low, don't do anything
    return;

  float fac = 50.0f/var;
  for (int j=0; j < height; j++)
  {
    float* imgRow = image.ptr<float>(j);
    for (int i=0; i < width; i++,imgRow++)
    {
      *imgRow = 128 + fac * (*imgRow - sum);
      if (*imgRow > 255) *imgRow = 255;
      if (*imgRow < 0)  *imgRow=0;
    }
  }
}

Mat gaussianBlur(const Mat input, float sigma)
{
  Mat ret(input.rows, input.cols, input.type());
  int size = (int)(2.0 * 3.0 * sigma + 1.0);
  if (size % 2 == 0) size++;
  GaussianBlur(input, ret, cv::Size(size, size), sigma, sigma, cv::BORDER_REPLICATE);
  return ret;
}

void gaussianBlurInplace(Mat &inplace, float sigma)
{
  int size = (int)(2.0 * 3.0 * sigma + 1.0);
  if (size % 2 == 0) size++;
  GaussianBlur(inplace, inplace, cv::Size(size, size), sigma, sigma, cv::BORDER_REPLICATE);
}

Mat doubleImage(const Mat &input)
{
  Mat n(input.rows*2, input.cols*2, input.type());
  const float *in = input.ptr<float>(0);

  for (int r = 0; r < input.rows-1; r++)
    for (int c = 0; c < input.cols-1; c++)
    {
      const int r2 = r << 1;
      const int c2 = c << 1;
      n.at<float>(r2,c2)     = in[0];
      n.at<float>(r2+1,c2)   = 0.5f *(in[0]+in[input.step]);
      n.at<float>(r2,c2+1)   = 0.5f *(in[0]+in[1]);
      n.at<float>(r2+1,c2+1) = 0.25f*(in[0]+in[1]+in[input.step]+in[input.step+1]);
      ++in;
    }
  for (int r = 0; r < input.rows-1; r++)
  {
    const int r2 = r << 1;
    const int c2 = (input.cols-1) << 1;
    n.at<float>(r2,c2)   = input.at<float>(r,input.cols-1);
    n.at<float>(r2+1,c2) = 0.5f*(input.at<float>(r,input.cols-1) + input.at<float>(r+1,input.cols-1));
  }
  for (int c = 0; c < input.cols - 1; c++)
  {
    const int r2 = (input.rows-1) << 1;
    const int c2 = c << 1;
    n.at<float>(r2,c2)   = input.at<float>(input.rows-1,c);
    n.at<float>(r2,c2+1) = 0.5f*(input.at<float>(input.rows-1,c) + input.at<float>(input.rows-1,c+1));
  }
  n.at<float>(n.rows-1, n.cols-1) = n.at<float>(input.rows-1, input.cols-1);
  return n;
}

Mat halfImage(const Mat &input)
{
  Mat n(input.rows/2, input.cols/2, input.type());
  float *out = n.ptr<float>(0);
  for (int r = 0, ri = 0; r < n.rows; r++, ri += 2)
  {
    const float *inputPtr = input.ptr<float>(ri);
    for (int c = 0, ci = 0; c < n.cols; c++, ci += 2)
      *out++ = inputPtr[ci];
  }
  return n;
}
void computeGradient(const Mat &img, Mat &gradx, Mat &grady)
{
  const int width = img.cols;
  const int height = img.rows;
  for (int r = 0; r < height; ++r)
    for (int c = 0; c < width; ++c)
    {
      float xgrad, ygrad;
      if (c == 0) xgrad = img.at<float>(r,c+1) - img.at<float>(r,c); else
      if (c == width-1) xgrad = img.at<float>(r,c) - img.at<float>(r,c-1); else
        xgrad = img.at<float>(r,c+1) - img.at<float>(r,c-1);

      if (r == 0) ygrad = img.at<float>(r+1,c) - img.at<float>(r,c); else
      if (r == height-1) ygrad = img.at<float>(r,c) - img.at<float>(r-1,c); else
        ygrad = img.at<float>(r+1,c) - img.at<float>(r-1,c);
      gradx.at<float>(r,c) = xgrad;
      grady.at<float>(r,c) = ygrad;
    }
}

//void computeGradient(const Mat &img, Mat &gradx, Mat &grady)
//{
//  const int width = img.cols;
//  const int height = img.rows;
//  for (int r = 0; r < height; ++r)
//  {
//    const float* Row0 = img.ptr<float>(r); //got rid of at<>`s
//    float* gradxPtr = gradx.ptr<float>(r);
//    float* gradyPtr = grady.ptr<float>(r);
//
//    for (int c = 0; c < width; ++c)
//    {
//      if (c == 0)  {
//        gradxPtr[c] = Row0[c+1]- Row0[c];
//      }
//      else {
//        if (c == width - 1) {
//          gradxPtr[c] = Row0[c] - Row0[c - 1];
//        }
//        else {
//          gradxPtr[c] = Row0[c + 1] - Row0[c - 1];
//        }
//      }
//      if (r == 0) {
//        const float* Row1 = img.ptr<float>(r+1);
//        gradyPtr[c] = Row1[c] - Row0[c];
//      }
//      else  {
//        const float* Rowi1 = img.ptr<float>(r-1);
//        if (r == height-1) {
//          gradyPtr[c] = Row0[c] - Rowi1[c];
//        }
//        else
//        {
//          const float* Row1 = img.ptr<float>(r+1);
//          gradyPtr[c] = Row1[c]-Rowi1[c]; }
//      }
//    }
//  }
//}

void computeGradientMagnitudeAndOrientation(const Mat &img, Mat &mag, Mat &ori)
{
  const int width = img.cols;
  const int height = img.rows;

  for (int r = 1; r < height-1; ++r)
  {
    const float* Row0 = img.ptr<float>(r); //got rid of at<>`s
    const float* Row1 = img.ptr<float>(r+1);
    const float* Rowi1 = img.ptr<float>(r-1);
    float* magPtr = mag.ptr<float>(r);
    float* oriPtr = ori.ptr<float>(r);
    for (int c = 1; c < width-1; ++c)
    {
      float xgrad, ygrad;
      xgrad = Row0[c+1] - Row0[c-1];
      ygrad = Row1[c]-Rowi1[c];

      magPtr[c] = ::sqrt(xgrad * xgrad + ygrad * ygrad);
      oriPtr[c] = atan2LUTff(ygrad, xgrad);
      //         oriPtr[c] = atan2(ygrad, xgrad);
    }
  }
}
//void computeGradientMagnitudeAndOrientationWLD(const Mat &img, Mat &mag, Mat &ori)
//{
//  const int width = img.cols;
//  const int height = img.rows;
//  const double a = 3.0;
//  const double b = 5.0;
//  const double g_i = 1/5.0;
//
//  for (int r = 1; r < height-1; ++r)
//  {
//    const float* Row0 = img.ptr<float>(r); //got rid of at<>`s
//    const float* Row1 = img.ptr<float>(r+1);
//    const float* Rowi1 = img.ptr<float>(r-1);
//    float* magPtr = mag.ptr<float>(r);
//    float* oriPtr = ori.ptr<float>(r);
//    for (int c = 1; c < width-1; ++c)
//    {
//      float xgrad, ygrad;
//      xgrad = M_PI_255*atan(a*(Row0[c+1] - Row0[c-1])/ (Row0[c+1]*g_i+b));
//      ygrad =  M_PI_255*atan(a*(Row1[c] - Rowi1[c])/ (Row1[c]*g_i+b));
//
//      magPtr[c] = ::sqrt(xgrad * xgrad + ygrad * ygrad);
//      oriPtr[c] = atan2LUTff(ygrad, xgrad);
//      //        oriPtr[c] = atan2(ygrad, xgrad);
//    }
//  }
//
//}
//
//void calculateWLDfast(const cv::Mat &inImg, cv::Mat &outImg,
//                      const WLDParams pars,
//                      const double sigmaNext, const double sigmaStart,const double sigmaEnd)
//{
//  const double g_i = 1.0/pars.g;
//  cv::Mat WLD,blurred, diff_img;
//  // blur to reduce noise
//  int k_size = floor(2.0 * 4.0 * sigmaStart + 1.0);
//  if (k_size % 2 == 0)
//    k_size++;
//  if (k_size < 3) k_size = 3;
//
//  cv::GaussianBlur(inImg,WLD,cv::Size(k_size, k_size),sigmaStart,sigmaStart,cv::BORDER_DEFAULT);
//  k_size = floor(2.0 * 4.0 * sigmaNext + 1.0);
//  if (k_size % 2 == 0)
//    k_size++;
//  if (k_size < 3) k_size = 3;
//
//  cv::GaussianBlur(inImg,blurred,cv::Size(k_size, k_size),sigmaNext,sigmaNext,cv::BORDER_DEFAULT);
//  diff_img = blurred - WLD;
//
//  cv::Mat min_img = min(blurred,WLD);
//  //cv::Mat min_img = (blurred+WLD)/2.0;
//
//  //WLD Differential Excitation
//  int nRows = WLD.rows;
//  int nCols = WLD.cols;
//  for(int i = 0; i < nRows; ++i)
//  {
//    const float *dPtr = diff_img.ptr<float>(i);
//    const float *minPtr = min_img.ptr<float>(i);
//    float *resPtr = WLD.ptr<float>(i);
//    for (int j = 0; j < nCols; ++j)
//    {
//      const double x = exp(-pars.a*dPtr[j] / (minPtr[j]*g_i+pars.b));
//      resPtr[j] = (1.0-x) / (1.0+x);
//      //      resPtr[j] =  pars.a*dPtr[j] / (minPtr[j]*g_i+pars.b);
//    }
//  }
//
//  k_size = floor(2.0 * 4.0 * sigmaEnd + 1.0);
//  if (k_size % 2 == 0)
//    k_size++;
//  if (k_size < 3) k_size = 3;
//
//  if (sigmaEnd > 0)
//  {
//    cv::GaussianBlur(WLD,diff_img,cv::Size(k_size, k_size),sigmaEnd,sigmaEnd,cv::BORDER_DEFAULT);
//    outImg = diff_img.clone();
//  }
//  else
//  {
//    outImg = WLD.clone();
//  }
//}
