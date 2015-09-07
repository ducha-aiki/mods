/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/

#undef __STRICT_ANSI__
#include "synth-detection.hpp"
#include "detectors/helpers.h"
#include "matching/siftdesc.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "opensurf/surflib.h"

//
#include "detectors/mser/extrema/extrema.h"
//

//#define AREA_INTERP
#ifdef _OPENMP
#include <omp.h>
#endif


const double k_sigma = 2 * 3.0 * sqrt(3.0);//to compare ellipses in 3*sigma size
const double eps1 = 0.01;
using namespace std;

#define VERBOSE 0
#define VERTICAL 1
#define HORIZONTAL 0


//#define ADD_UP_IS_UP

const int MAX_HEIGHT = 10000;
const int MAX_WIDTH = 10000;

const double COS_PI_2 = cos(M_PI/2);
const double SIN_PI_2 = sin(M_PI/2);


void rectifyTransformation(double &a11, double &a12, double &a21, double &a22)
{
  double a = a11, b = a12, c = a21, d = a22;
  double det = sqrt(fabs(a*d-b*c));
  double b2a2 = sqrt(b*b + a*a);
  a11 = b2a2/det;
  a12 = 0;
  a21 = (d*b+c*a)/(b2a2*det);
  a22 = det/b2a2;
}

int SetVSPars(const std::vector <double> &scale_set,
              const std::vector <double> &tilt_set,
              const double phi_base,
              const std::vector <double> &FGINNThreshold,
              const std::vector <double> &DistanceThreshold,
              const std::vector <std::string> descriptors,
              std::vector<ViewSynthParameters> &par,
              std::vector<ViewSynthParameters> &prev_par,
              const double InitSigma,
              const int doBlur,
              const int dsplevels,
              const double minSigma, const double maxSigma)
{
  par.clear();
  std::vector<ViewSynthParameters> prev_par_tmp(prev_par);
  std::vector<ViewSynthParameters> pars_tmp;

  if ((scale_set.size() ==0) || (tilt_set.size() == 0))
    {
      ViewSynthParameters temp_par;
      temp_par.phi = 0;
      temp_par.tilt = 0;
      temp_par.zoom = 0;
      temp_par.InitSigma = InitSigma;
      temp_par.doBlur = 0;
      temp_par.DSPlevels = dsplevels;
      temp_par.descriptors = descriptors;
      for (unsigned int desc=0; desc<descriptors.size(); desc++)
        {
          temp_par.DistanceThreshold[descriptors[desc]]=DistanceThreshold[desc];
          temp_par.FGINNThreshold[descriptors[desc]]=FGINNThreshold[desc];
        }
      pars_tmp.push_back(temp_par);
    }
  for (unsigned int sc=0; sc < scale_set.size(); sc++)
    for (unsigned int t=0; t < tilt_set.size(); t++)
      {
        if (fabs(tilt_set[t] - 1) > eps1)
          {
            int n_rot1 = floor(180.0*tilt_set[t]/phi_base);
            double delta_phi = M_PI/n_rot1;
            if (n_rot1 < 0) { //no rotation mode if negative, add vertical tilt
                n_rot1 = 1;
                delta_phi = 0;
                double phi = 0;
                assert (phi >= 0);
                ViewSynthParameters temp_par;
                temp_par.phi = phi;
                temp_par.tilt = -tilt_set[t];
                temp_par.zoom = scale_set[sc];
                temp_par.InitSigma = InitSigma;
                temp_par.doBlur = doBlur;
                temp_par.DSPlevels = dsplevels;
                temp_par.minSigma = minSigma;
                temp_par.maxSigma = maxSigma;
                temp_par.descriptors = descriptors;

                for (unsigned int desc=0; desc<descriptors.size(); desc++)
                  {
                    temp_par.DistanceThreshold[descriptors[desc]]=DistanceThreshold[desc];
                    temp_par.FGINNThreshold[descriptors[desc]]=FGINNThreshold[desc];
                  }
                pars_tmp.push_back(temp_par);

              }
            for (int r=0 ; r<n_rot1; r++)
              {
                double phi = delta_phi * r;
                assert (phi >= 0);
                ViewSynthParameters temp_par;
                temp_par.phi = phi;
                temp_par.tilt = tilt_set[t];
                temp_par.zoom = scale_set[sc];
                temp_par.InitSigma = InitSigma;
                temp_par.doBlur = doBlur;
                temp_par.DSPlevels = dsplevels;
                temp_par.minSigma = minSigma;
                temp_par.maxSigma = maxSigma;
                temp_par.descriptors = descriptors;

                for (unsigned int desc=0; desc<descriptors.size(); desc++)
                  {
                    temp_par.DistanceThreshold[descriptors[desc]]=DistanceThreshold[desc];
                    temp_par.FGINNThreshold[descriptors[desc]]=FGINNThreshold[desc];
                  }
                pars_tmp.push_back(temp_par);
              }
          }
        else
          {
            ViewSynthParameters temp_par;
            temp_par.phi = 0;
            temp_par.tilt = tilt_set[t];
            temp_par.zoom = scale_set[sc];
            temp_par.InitSigma = InitSigma;
            temp_par.doBlur = doBlur;
            temp_par.DSPlevels = dsplevels;
            temp_par.minSigma = minSigma;
            temp_par.maxSigma = maxSigma;
            temp_par.descriptors = descriptors;
            for (unsigned int desc=0; desc<descriptors.size(); desc++)
              {
                temp_par.DistanceThreshold[descriptors[desc]]=DistanceThreshold[desc];
                temp_par.FGINNThreshold[descriptors[desc]]=FGINNThreshold[desc];
              }
            pars_tmp.push_back(temp_par);
            continue;
          }
      }
  std::vector<char> isUnique(pars_tmp.size());
  for (unsigned int i=0; i<pars_tmp.size(); i++)
    isUnique[i]=1;

  for (unsigned int i=0; i<pars_tmp.size(); i++)
    for (unsigned int j=0; j<prev_par_tmp.size(); j++)
      if ((fabs(pars_tmp[i].zoom - prev_par_tmp[j].zoom) <= eps1) &&
          (fabs(pars_tmp[i].tilt - prev_par_tmp[j].tilt) <= eps1) &&
          (fabs(pars_tmp[i].phi - prev_par_tmp[j].phi) <= eps1))
        {
          isUnique[i]=0;
          break;
        }

  std::vector<ViewSynthParameters>::iterator ptr = pars_tmp.begin();
  for (unsigned int i=0; i<pars_tmp.size(); i++, ptr++)
    if (isUnique[i]) par.push_back(*ptr);

  for (unsigned int i=0; i<par.size(); i++)
    prev_par_tmp.push_back(par[i]);
  prev_par = prev_par_tmp;
  return (int)par.size();
}

void GenerateSynthImageCorr(const cv::Mat &in_img,
                            SynthImage &out_img,
                            const std::string in_img_name,
                            double tilt,
                            const double phi,
                            const double zoom,
                            const double InitSigma,
                            const int doBlur,
                            const int img_id,
                            const bool convert2gray)
{
  int zoomed=0;
  bool vertical_tilt = false;
  if (tilt < 0) { // vertical tilt
      tilt = -tilt;
      vertical_tilt = true;
    }
  if (fabs(zoom-1.0f)>=0.05) zoomed = 1;
  cv::Mat temp_img;
  cv::Mat gray_in_img;
  if ((in_img.channels() == 3) && (convert2gray))
    {
      std::vector<cv::Mat> RGB_planes(3);
      cv::Mat in_32f;
      in_img.convertTo(in_32f,CV_32FC3);
      cv::split(in_32f, RGB_planes);
      gray_in_img = (RGB_planes[0] + RGB_planes[1] + RGB_planes[2]) / 3.0 ;
    } else
    {
      gray_in_img = in_img;
    }

  double sigma_aa, sigma_aa_2, sigma_x,sigma_y;
  int wS1=0, hS1=0;
  int w =in_img.cols;
  int h = in_img.rows;
  double phi_deg = phi*180/M_PI;

  out_img.OrigImgName= in_img_name;

  wS1 = (int) (w * zoom);
  hS1 = (int) (h * zoom);
  if ((fabs(tilt - 1.) <=0.1) && (abs(phi) <= 0.2) && (fabs(zoom - 1.) <=0.1)) //original image
    { out_img.rotation= 0.0;
      out_img.tilt= 1.0;
      out_img.zoom= 1.0;
      out_img.id = 0;
      out_img.H[0]=1.0; out_img.H[1]=0;   out_img.H[2]=0;
      out_img.H[3]=0;   out_img.H[4]=1.0; out_img.H[5]=0;
      out_img.H[6]=0;   out_img.H[7]=0;   out_img.H[8]=1.0;
      out_img.pixels = gray_in_img;//.clone();
      //    std::cerr << "No image synth, original image cloned" << std::endl;
      return;
    }
  // else {
  out_img.id = img_id;
  /// Affine transfromation matrix

  double d,d2,w_new,h_new;
  double kV=1.;
  double kH=1.;
  if (zoomed){
      kV = (double)w/(double)wS1;
      kH = (double)h/(double)hS1;
    };
  if (vertical_tilt) {
      if ((phi>=0) && (phi<M_PI/2))
        {
          w_new=floor((0.5+cos(phi)*w+sin(phi)*h)/(kH));
          h_new=floor((0.5+sin(phi)*w+cos(phi)*h)/(tilt*kV));
          out_img.H[0]= cos(phi)/kH;        out_img.H[1]= sin(phi)/kH;         out_img.H[2]= 0;
          out_img.H[3]=-sin(phi)/(tilt*kV); out_img.H[4] = cos(phi)/(tilt*kV); out_img.H[5]=floor(0.5+sin(phi)*w/(tilt*kV));
          out_img.H[6]= 0;                  out_img.H[7]=0;                    out_img.H[8]=1;
        }
      else
        {
          w_new=floor((0.5-cos(phi)*w+sin(phi)*h)/(kH));
          h_new=floor((0.5+sin(phi)*w-cos(phi)*h)/(tilt*kV));
          d=-floor(cos(phi)*w/kH);
          d2=floor(0.5+(sin(phi)*w-cos(phi)*h)/(tilt*kV));
          out_img.H[0]=cos(phi)/kH;         out_img.H[1]=sin(phi)/kH;         out_img.H[2]=d;
          out_img.H[3]=-sin(phi)/(tilt*kV); out_img.H[4]=cos(phi)/(tilt*kV);  out_img.H[5]=d2;
          out_img.H[6]= 0;                  out_img.H[7]=0;                   out_img.H[8]=1;
        };


    } else {
      if ((phi>=0) && (phi<M_PI/2))
        {
          w_new=floor((0.5+cos(phi)*w+sin(phi)*h)/(tilt*kH));
          h_new=floor((0.5+sin(phi)*w+cos(phi)*h)/(kV));
          out_img.H[0]= cos(phi)/(tilt*kH); out_img.H[1]= sin(phi)/(tilt*kH);  out_img.H[2]= 0;
          out_img.H[3]=-sin(phi)/kV;        out_img.H[4] = cos(phi)/kV;        out_img.H[5]=floor(0.5+sin(phi)*w/kV);
          out_img.H[6]= 0;                  out_img.H[7]=0;                    out_img.H[8]=1;
        }
      else
        {
          w_new=floor((0.5-cos(phi)*w+sin(phi)*h)/(tilt*kH));
          h_new=floor((0.5+sin(phi)*w-cos(phi)*h)/(kV));
          d=-floor(cos(phi)*w/(tilt*kH));
          d2=floor(0.5+(sin(phi)*w-cos(phi)*h)/kV);
          out_img.H[0]=cos(phi)/(tilt*kH);  out_img.H[1]=sin(phi)/(tilt*kH);  out_img.H[2]=d;
          out_img.H[3]=-sin(phi)/kV;        out_img.H[4]=cos(phi)/kV;         out_img.H[5]=d2;
          out_img.H[6]= 0;                  out_img.H[7]=0;                   out_img.H[8]=1;
        };

    }


  out_img.rotation=phi_deg;
  out_img.tilt=tilt;
  out_img.zoom = zoom;

  /// Anti-aliasing filtering
  if (zoomed)
    sigma_aa_2 = InitSigma / (4.0*zoom);
  else
    sigma_aa_2 = InitSigma / 2.0;

  sigma_aa = InitSigma * tilt / (2.0*zoom);
  if (vertical_tilt) {
      sigma_x = sigma_aa_2;
      sigma_y = sigma_aa;

    } else {
      sigma_x = sigma_aa;
      sigma_y = sigma_aa_2;
    }
  int w_new_rot;
  int h_new_rot;
  double warpRot[6];

  if ((phi>=0) && (phi<M_PI/2))
    {
      w_new_rot=floor((0.5+cos(phi)*w+sin(phi)*h));
      h_new_rot=floor((0.5+sin(phi)*w+cos(phi)*h));
      warpRot[0]= cos(phi); warpRot[1]= sin(phi);  warpRot[2]= 0;
      warpRot[3]=-sin(phi); warpRot[4]= cos(phi); warpRot[5]=floor(0.5+sin(phi)*w);
    }
  else
    {
      w_new_rot=floor((0.5-cos(phi)*w+sin(phi)*h));
      h_new_rot=floor((0.5+sin(phi)*w-cos(phi)*h));
      d=-floor(cos(phi)*w);
      d2=floor(0.5+(sin(phi)*w-cos(phi)*h));
      warpRot[0]=cos(phi);  warpRot[1]=sin(phi);  warpRot[2]=d;
      warpRot[3]=-sin(phi); warpRot[4]=cos(phi);  warpRot[5]=d2;
    };
  cv::Mat warpMatrixRot(2,3,CV_64F,warpRot);
  cv::warpAffine(gray_in_img, temp_img, warpMatrixRot,
                 cv::Size(w_new_rot,h_new_rot),cv::INTER_LINEAR, cv::BORDER_CONSTANT,cv::Scalar(128,128,128));
#ifdef AREA_INTERP
  /// simulate a tilt-zoom
  double warp_tilt_zoom[6];


  warp_tilt_zoom[0]=1.0/(tilt*kH); warp_tilt_zoom[1]=0;  warp_tilt_zoom[2]=0;
  warp_tilt_zoom[3]=0;        warp_tilt_zoom[4]=1.0/kV;  warp_tilt_zoom[5]=0;

  cv::Mat warpMatrix(2,3,CV_64F,warp_tilt_zoom);
  cv::warpAffine(temp_img, out_img.pixels, warpMatrix,
                 cv::Size(w_new,h_new),cv::INTER_AREA, cv::BORDER_CONSTANT,cv::Scalar(128,128,128));

#else
  if (doBlur)
    {
      int k_size_x = floor(2.0 * 3.0 * sigma_x + 1.0);
      if (k_size_x % 2 == 0)
        k_size_x++;
      if (k_size_x < 3) k_size_x = 3;

      int k_size_y = floor(2.0 * 3.0 * sigma_y + 1.0);
      if (k_size_y % 2 == 0)
        k_size_y++;
      if (k_size_y < 3) k_size_y = 3;
      cv::GaussianBlur(temp_img,temp_img,cv::Size(k_size_x, k_size_y),sigma_x,sigma_y);
    }

  /// simulate a tilt-zoom
  double warp_tilt_zoom[6];

  if (vertical_tilt) {
      warp_tilt_zoom[0]=1.0/kH; warp_tilt_zoom[1]=0;  warp_tilt_zoom[2]=0;
      warp_tilt_zoom[3]=0;        warp_tilt_zoom[4]=1.0/(tilt*kV);  warp_tilt_zoom[5]=0;

    } else {
      warp_tilt_zoom[0]=1.0/(tilt*kH); warp_tilt_zoom[1]=0;  warp_tilt_zoom[2]=0;
      warp_tilt_zoom[3]=0;        warp_tilt_zoom[4]=1.0/kV;  warp_tilt_zoom[5]=0;
    }
  cv::Mat warpMatrix(2,3,CV_64F,warp_tilt_zoom);
  cv::warpAffine(temp_img, out_img.pixels, warpMatrix,
                 cv::Size(w_new,h_new),cv::INTER_LINEAR, cv::BORDER_CONSTANT,cv::Scalar(128,128,128));
#endif
  // }
}
void GenerateSynthImageByH(const cv::Mat &in_img, SynthImage &out_img,const double* H,const double InitSigma,const int doBlur, const int img_id)
{
  cv::Mat temp_img;
  /// Anti-aliasing filtering
  double sigma_aa_2 = InitSigma / (4.0);
  double sigma_aa = InitSigma / (4.0);
  double sigma_x = sigma_aa;
  double sigma_y = sigma_aa_2;
  double xmin,xmax,ymin,ymax;

  if (doBlur)
    {
      int k_size_x = floor(2.0 * 4.0 * sigma_x + 1.0);
      if (k_size_x % 2 == 0)
        k_size_x++;
      if (k_size_x < 3) k_size_x = 3;

      int k_size_y = floor(2.0 * 4.0 * sigma_y + 1.0);
      if (k_size_y % 2 == 0)
        k_size_y++;
      if (k_size_y < 3) k_size_y = 3;
      cv::GaussianBlur(in_img,temp_img,cv::Size(k_size_x, k_size_y),sigma_x,sigma_y,cv::BORDER_DEFAULT);

    }
  else temp_img=in_img;
  /// simulate a tilt-roration-zoom
  for (int i=0; i<9; i++)
    out_img.H[i] = H[i];
  out_img.id = img_id;
  double bx1=0,by1=0,bx2=in_img.cols,by2=in_img.rows;
  cv::Point p1,p2,p3,p4;
  p1.x=(H[0]*bx1+H[1]*by1+H[2]) /(H[6]*bx1+H[7]*by1+H[8]);
  p1.y=(H[3]*bx1+H[4]*by1+H[5]) /(H[6]*bx1+H[7]*by1+H[8]);

  p2.x=(H[0]*bx1+H[1]*by2+H[2]) /(H[6]*bx1+H[7]*by2+H[8]);
  p2.y=(H[3]*bx1+H[4]*by2+H[5]) /(H[6]*bx1+H[7]*by2+H[8]);

  p3.x=(H[0]*bx2+H[1]*by1+H[2]) /(H[6]*bx2+H[7]*by1+H[8]);
  p3.y=(H[3]*bx2+H[4]*by1+H[5]) /(H[6]*bx2+H[7]*by1+H[8]);

  p4.x=(H[0]*bx2+H[1]*by2+H[2]) /(H[6]*bx2+H[7]*by2+H[8]);
  p4.y=(H[3]*bx2+H[4]*by2+H[5]) /(H[6]*bx2+H[7]*by2+H[8]);


  xmin = min(min(p1.x,p2.x),min(p3.x,p4.x));
  ymin = min(min(p1.y,p2.y),min(p3.y,p4.y));
  xmax = max(max(p1.x,p2.x),max(p3.x,p4.x));
  ymax = max(max(p1.y,p2.y),max(p3.y,p4.y));

  int dx, dy;
  dx = (int)floor(xmax);
  dy = (int)floor(ymax);
  if (dx > MAX_WIDTH) dx = MAX_WIDTH;
  if (dy > MAX_HEIGHT) dy = MAX_HEIGHT;
  cv::Mat warpMatrix(3,3,CV_64F,out_img.H);
  cv::warpPerspective(temp_img, out_img.pixels, warpMatrix,
                      cv::Size(dx,dy),cv::INTER_LINEAR, cv::BORDER_CONSTANT,cv::Scalar(128,128,128));
}

void ReprojectByH(AffineKeypoint in_kp, AffineKeypoint &out_kp, double* H) //For H=[h11 h12 h13; h21 h22 h23; 0 0 1];
{
  out_kp.x=(H[0]*in_kp.x+H[1]*in_kp.y+H[2]);// /(H[6]*in_kp.x+H[7]*in_kp.y+H[8]);
  out_kp.y=(H[3]*in_kp.x+H[4]*in_kp.y+H[5]);// /(H[6]*in_kp.x+H[7]*in_kp.y+H[8]);
  out_kp.a11=(H[0]*in_kp.a11+H[1]*in_kp.a21);
  out_kp.a12=(H[0]*in_kp.a12+H[1]*in_kp.a22);
  out_kp.a21=(H[3]*in_kp.a11+H[4]*in_kp.a21);
  out_kp.a22=(H[3]*in_kp.a12+H[4]*in_kp.a22);
}
void ReprojectByHReal(AffineKeypoint in_kp, AffineKeypoint &out_kp, double* H) //For H=[h11 h12 h13; h21 h22 h23; 0 0 1];
{
  out_kp.x=(H[0]*in_kp.x+H[1]*in_kp.y+H[2]) /(H[6]*in_kp.x+H[7]*in_kp.y+H[8]);
  out_kp.y=(H[3]*in_kp.x+H[4]*in_kp.y+H[5]) /(H[6]*in_kp.x+H[7]*in_kp.y+H[8]);

  double Hlin[4];
  linH(in_kp.x, in_kp.y, H, Hlin);
  //  linH(out_kp.x, out_kp.y, H, Hlin);
  out_kp.a11=(Hlin[0]*in_kp.a11+Hlin[1]*in_kp.a21);
  out_kp.a12=(Hlin[0]*in_kp.a12+Hlin[1]*in_kp.a22);
  out_kp.a21=(Hlin[2]*in_kp.a11+Hlin[3]*in_kp.a21);
  out_kp.a22=(Hlin[2]*in_kp.a12+Hlin[3]*in_kp.a22);
}

int ReprojectRegionsBackReal(AffineRegionList &keypoints, double *H, const int width2, const int height2) {
  AffineRegionList::iterator ptr = keypoints.begin();
  AffineRegionList reproj_keypoints;
  reproj_keypoints.reserve(keypoints.size());
  for ( unsigned int i=0; i<keypoints.size(); i++, ptr++)
    {
      ptr->det_kp  = ptr->reproj_kp;
      ReprojectByHReal(ptr->reproj_kp, ptr->det_kp, H);
    }
  ptr = keypoints.begin();
  for (unsigned int i=0; i < keypoints.size(); i++, ptr++)
    {
      if ( (ptr->det_kp.x < width2) && (ptr->det_kp.y < height2)
           && (ptr->det_kp.x > 0) && (ptr->det_kp.y > 0)) {  //center is inside
          if ( !interpolateCheckBorders(width2, height2,
                                        ptr->det_kp.x, ptr->det_kp.y,
                                        ptr->det_kp.a11, ptr->det_kp.a12,
                                        ptr->det_kp.a21, ptr->det_kp.a22,
                                        k_sigma * ptr->det_kp.s,
                                        k_sigma * ptr->det_kp.s)) {

              reproj_keypoints.push_back(*ptr);
            }
        }
    }
  keypoints = reproj_keypoints;
}

bool HIsEye(double* H) {
  return (fabs(H[0] - 1.0) + fabs(H[1]) + fabs(H[2])
      + fabs(H[3]) + fabs(H[4] - 1.0) + fabs(H[5]) +
      fabs(H[6]) + fabs(H[7]) + fabs(H[8] - 1.0) < eps1);

}
int ReprojectRegions(AffineRegionList &keypoints, double *H, int orig_w, int orig_h) {
  double k;//3 sigma
  cv::Mat H1(3, 3, CV_64F, H);
  cv::Mat Hinv(3, 3, CV_64F);
  cv::invert(H1, Hinv, cv::DECOMP_LU);
  double *HinvPtr = (double *) Hinv.data;

  AffineRegionList::iterator ptr = keypoints.begin();
  if (HIsEye(H)) {
      for (unsigned i = 0; i < keypoints.size(); i++, ptr++) {
          ptr->reproj_kp = ptr->det_kp;
        }
    } else {
      for (unsigned i = 0; i < keypoints.size(); i++, ptr++) {
          ptr->reproj_kp = ptr->det_kp;
          ReprojectByH(ptr->det_kp, ptr->reproj_kp, HinvPtr);
        }
    }

  AffineRegionList temp_keypoints;
  temp_keypoints.reserve(keypoints.size());
  ptr = keypoints.begin();
  //  const int bound_size=2;
  //  const int bound_w = orig_w - bound_size;
  //  const int bound_h = orig_h - bound_size;

  for (unsigned int i=0; i < keypoints.size(); i++, ptr++)
    {
      if ( (ptr->reproj_kp.x < orig_w) && (ptr->reproj_kp.y < orig_h)
           && (ptr->reproj_kp.x > 0) && (ptr->reproj_kp.y > 0)) {  //center is inside
          if ( !interpolateCheckBorders(orig_w, orig_h,
                                        ptr->reproj_kp.x, ptr->reproj_kp.y,
                                        ptr->reproj_kp.a11, ptr->reproj_kp.a12,
                                        ptr->reproj_kp.a21, ptr->reproj_kp.a22,
                                        k_sigma * ptr->reproj_kp.s,
                                        k_sigma * ptr->reproj_kp.s)) {

              temp_keypoints.push_back(keypoints[i]);
              //       std::cerr << "Passed " << std::endl;
            }
          //      k=k_sigma*ptr->reproj_kp.s;
          //      double tangx = ptr->reproj_kp.a12 / ptr->reproj_kp.a11 ;
          //      double tangx2;
          //      if (ptr->reproj_kp.a21 !=0) tangx2 = ptr->reproj_kp.a22 / ptr->reproj_kp.a21; //      else tangx2 = tangx;
          //
          //      double den = sqrt(1.0+tangx*tangx);
          //      double den2 = sqrt(1.0+tangx2*tangx2);
          //
          //      double cs = 1.0/den;
          //      double si = tangx/den;
          //
          //      double cs2 = 1.0/den2;
          //      double si2 = tangx2/den2;
          //
          //      double delta_x =k*fabs((ptr->reproj_kp.a11*cs + ptr->reproj_kp.a12 * si));
          //      double delta_y =k*fabs((ptr->reproj_kp.a21*cs2 + ptr->reproj_kp.a22 * si2));
          ////      double delta_x =k*fabs((ptr->reproj_kp.a11 + ptr->reproj_kp.a12  ));
          ////      double delta_y =k*fabs((ptr->reproj_kp.a21 + ptr->reproj_kp.a22 ));
          //
          //      double xmin=ptr->reproj_kp.x - delta_x;
          //      double xmax=ptr->reproj_kp.x + delta_x;
          //      double ymin=ptr->reproj_kp.y - delta_y;
          //      double ymax=ptr->reproj_kp.y + delta_y;
          //      if ((xmin > bound_size) && (ymin > bound_size) && (xmax < bound_w) && (ymax < bound_h)) { //is Inside
          //        temp_keypoints.push_back(keypoints[i]);
          //      }
          //      else
          //      {
          //     //   temp_keypoints.push_back(keypoints[i]);
          //     //   std::cerr << "reprojected region is outside. Deleted" << std::endl;
          //      }
        }
    }
  keypoints = temp_keypoints;
  return (int)keypoints.size();
}

double ellipseOverlapPrep(AffineKeypoint ref_kp, AffineKeypoint test_kp, const double max_error)
{

  double diff, dist, x1,y1,x2,y2;
  double APtr[4]= {ref_kp.a11, ref_kp.a12,
                   ref_kp.a21, ref_kp.a22
                  };

  x1 = ref_kp.x;
  y1 = ref_kp.y;

  x2 = (APtr[0]*(double)test_kp.x+APtr[1]*(double)test_kp.y);
  y2 = (APtr[2]*(double)test_kp.x+APtr[3]*(double)test_kp.y);

  dist=(x2-x1)*(x2-x1)+(y2-y1)*(y2-y1); //distance between ellipse centers in canonical coordinate frame

  if (dist>max_error) return dist; //speed-up

  double B1Ptr[4]= {test_kp.a11, test_kp.a12,
                    test_kp.a21, test_kp.a22
                   };
  cv::Mat B1(2,2,CV_64F, B1Ptr);
  cv::Mat A(2,2,CV_64F, APtr);

  rectifyTransformation(B1Ptr[0], B1Ptr[1],B1Ptr[2], B1Ptr[3]);


  cv::gemm(A,k_sigma*test_kp.s*B1,1, B1,0,A);
  rectifyTransformation(APtr[0], APtr[1],APtr[2], APtr[3]);

  diff = 0.5*((1 - APtr[0])*(1 - APtr[0]) +  APtr[1]* APtr[1] + APtr[2]* APtr[2]+ (1-APtr[3])*(1-APtr[3]));
  //distance between ellipse shapes in canonical coordinate frame
  return (diff+dist);
}

double ellipseOverlap(AffineKeypoint ref_kp, AffineKeypoint test_kp, const double max_error)
{
  double diff, dist;

  double B1Ptr[4]= {test_kp.a11, test_kp.a12,
                    test_kp.a21, test_kp.a22
                   };
  cv::Mat B1(2,2,CV_64F, B1Ptr);

  double A1Ptr[4]= {ref_kp.a11, ref_kp.a12,
                    ref_kp.a21, ref_kp.a22
                   };

  rectifyTransformation(A1Ptr[0], A1Ptr[1],A1Ptr[2], A1Ptr[3]); //to make no difference of point orientation
  cv::Mat A1(2,2,CV_64F, A1Ptr);
  cv::Mat Ainv(2,2,CV_64F);
  cv::invert(k_sigma*ref_kp.s*A1,Ainv, cv::DECOMP_LU); //inverting A

  double* AinvPtr = (double*)Ainv.data;

  double x1 = (AinvPtr[0]*(double)ref_kp.x+AinvPtr[1]*(double)ref_kp.y);
  double y1 = (AinvPtr[2]*(double)ref_kp.x+AinvPtr[3]*(double)ref_kp.y);

  double x2 = (AinvPtr[0]*(double)test_kp.x+AinvPtr[1]*(double)test_kp.y);
  double y2 = (AinvPtr[2]*(double)test_kp.x+AinvPtr[3]*(double)test_kp.y);

  dist=(x2-x1)*(x2-x1)+(y2-y1)*(y2-y1); //distance between ellipse centers in canonical coordinate frame

  if (dist>max_error) return dist; //speed-up

  cv::gemm(Ainv,k_sigma*test_kp.s*B1,1, B1,0,Ainv);
  rectifyTransformation(AinvPtr[0], AinvPtr[1],AinvPtr[2], AinvPtr[3]);

  diff = 0.5*((1 - AinvPtr[0])*(1 - AinvPtr[0]) +  AinvPtr[1]* AinvPtr[1] + AinvPtr[2]* AinvPtr[2]+ (1-AinvPtr[3])*(1-AinvPtr[3]));
  //distance between ellipse shapes in canonical coordinate frame
  return (diff+dist);
}

void AddRegionsToList(AffineRegionList &kp_list, AffineRegionList &new_kps)
{
  int size = (int)kp_list.size();
  unsigned int new_size = size + new_kps.size();
  AffineRegionList::iterator ptr = new_kps.begin();
  for (unsigned int i=size; i< new_size; i++, ptr++)
    {
      AffineRegion temp_reg = *ptr;
      temp_reg.id += size;
      temp_reg.parent_id +=size;
      kp_list.push_back(temp_reg);
    }
}

void AddRegionsToListByType(AffineRegionList &kp_list, AffineRegionList &new_kps,int type)
{
  int size = (int)kp_list.size();
  AffineRegionList::iterator ptr =new_kps.begin();
  unsigned int new_size = size + new_kps.size();
  for (unsigned int i=size; i< new_size; i++, ptr++)
    {
      if (ptr->type == type)
        {
          AffineRegion temp_reg = *ptr;
          temp_reg.id += size;
          temp_reg.parent_id +=size;
          kp_list.push_back(temp_reg);
        }
    }
}
template <int bins>
void smoothCircularBuffer(float *hist)
{
  float first = hist[0], prev = hist[bins-1];
  for (int i = 0; i < bins - 1; i++)
    {
      float cur = hist[i];
      hist[i] = prev + cur + hist[i+1];
      prev = cur;
    }
  hist[bins-1] = prev + hist[bins-1] + first;
}

template <int bins>
inline void addPeakAngle(const float *hist, vector<float> &angles, int a, int b, int c,
                         float threshold, vector<float> &peak_values)
{
  if (hist[b] >= threshold && hist[b] > hist[a] && hist[b] > hist[c])
    {
      float pp = (hist[a] - hist[c]) / (hist[a] - 2.0f * hist[b] + hist[c]) / 2.0f;
      angles.push_back(2.0f * float(M_PI) * (b + 0.5f + pp) / bins - float(M_PI));
      peak_values.push_back(hist[b]);
    }
}

struct EstimateDominantAnglesFunctor
{
private:
  cv::Mat gmag;
  cv::Mat gori;
  cv::Mat orimask;
  int pS;
  double magThresh;
  int doHalfSIFT;
public:
  EstimateDominantAnglesFunctor(int patchSize,const int doHalfSIFT1 = 0) : pS(patchSize),doHalfSIFT(doHalfSIFT1)
  {
    gmag = cv::Mat (pS, pS, CV_32FC1, cv::Scalar(0));
    gori = cv::Mat (pS, pS, CV_32FC1, cv::Scalar(0));
    orimask = cv::Mat(pS, pS, CV_32FC1);
    computeCircularGaussMask(orimask, pS/3.0f);
  }
  void operator()(const cv::Mat &img, vector<float> &angles1,
                  const double max_th=0.8, int maxAngles= -1)
  {
    if (maxAngles == 0) {
        angles1.clear();
        return;
      }
    const int bins = 36;
    float hist[bins+1];
    vector<float> peak_values;
    for (int i = 0; i<bins; i++) hist[i] = 0.0f;

    computeGradientMagnitudeAndOrientation(img, gmag, gori);

    float *maskptr = orimask.ptr<float>(1);
    float *pmag = gmag.ptr<float>(1), *pori = gori.ptr<float>(1);
    const int maskPixels = orimask.cols * (orimask.rows-2);

    for (int i = 0; i < maskPixels; ++i)
      {
        if (*maskptr > 0 && *pmag > 1.0)
          {
            int bin = (int) (bins * (*pori/float(M_PI) + 1.0f) / 2.0f);
            assert(bin >= 0 && bin <= bins);
            hist[bin] += (*pmag) * (*maskptr);
          }
        pmag++;
        pori++;
        maskptr++;
      }

    for (int i = 0; i < 6; i++)
      smoothCircularBuffer<bins>(hist);
    float thresh = 0.0;
    for (int i = 0; i < bins; i++)
      if (hist[i] > thresh) thresh = hist[i];
    thresh *= max_th;

    if (doHalfSIFT) {
        int halfbins = bins / 2;
        for (int i = 0; i < halfbins; i++)
          {
            hist[i] += hist[i+halfbins];
            hist[i+halfbins] = 0;
          }
      }

    // output all local maxima above threshold
    angles1.clear();
    addPeakAngle<bins>(hist, angles1, bins-1, 0, 1, thresh,peak_values);
    for (int i = 1; i < bins-1; i++)
      addPeakAngle<bins>(hist, angles1, i-1, i, i+1, thresh,peak_values);

    addPeakAngle<bins>(hist, angles1, bins-2, bins-1, 0, thresh,peak_values);
    if (maxAngles == -1) {
        maxAngles = 100000000;
      }
    maxAngles = min(maxAngles, (int)peak_values.size());
    if (maxAngles > 0) {
        vector<float> peak_values_sorted = peak_values;
        std::sort(peak_values_sorted.begin(), peak_values_sorted.end());
        vector<float> ang_tmp;
        for (int ang = 0; ang < maxAngles; ang++)
          {
            if (peak_values[ang] >= thresh) {
                ang_tmp.push_back(angles1[ang]);
              } else {
                break;
              }
          }
        angles1 = ang_tmp;
      } else {
        angles1.clear();
      }
  }

};

int DetectOrientation(AffineRegionList &in_kp_list,
                      AffineRegionList &out_kp_list,
                      SynthImage &img,
                      const double mrSize,
                      const int patchSize,
                      const int doHalfSIFT,
                      const int maxAngNum,
                      const double th,
                      const bool addUpRight) {
  AffineRegionList temp_kp_list;
  temp_kp_list.reserve(in_kp_list.size());

  AffineRegion temp_region, const_temp_region;
  unsigned int count = 0;
  //unsigned int i;
  double mrScale = (double)mrSize; // half patch size in pixels of image
  int patchImageSize = 2*int(mrScale)+1; // odd size
  vector<float> angles1;//, angles2;
  //  angles1.reserve(5);
  // angles2.reserve(5);
  double imageToPatchScale = double(patchImageSize) / (double)patchSize;
  // patch size in the image / patch size -> amount of down/up sampling

  cv::Mat patch(patchSize,patchSize,CV_32FC1);

  cv::Mat H1(3,3,CV_64F,img.H);
  cv::Mat Hinv(3,3,CV_64F);
  cv::invert(H1,Hinv, cv::DECOMP_LU);

  EstimateDominantAnglesFunctor EstDomOri(patchSize,doHalfSIFT);
  for (int i=0; i < in_kp_list.size(); i++)
    {
      const_temp_region=in_kp_list[i];
      angles1.clear();
      float curr_sc = imageToPatchScale*const_temp_region.det_kp.s;

      if (interpolateCheckBorders(img.pixels.cols,img.pixels.rows,
                                  (float) in_kp_list[i].det_kp.x,
                                  (float) in_kp_list[i].det_kp.y,
                                  (float) in_kp_list[i].det_kp.a11,
                                  (float) in_kp_list[i].det_kp.a12,
                                  (float) in_kp_list[i].det_kp.a21,
                                  (float) in_kp_list[i].det_kp.a22,
                                  k_sigma * in_kp_list[i].det_kp.s,
                                  k_sigma * in_kp_list[i].det_kp.s) ) {
          continue;
        }
      if (maxAngNum > 0) {
          const_temp_region.id = count; //because we add new orientations not to the end of the list, we have to renumerate next regions.
          //      float curr_sc = imageToPatchScale*const_temp_region.det_kp.s;

          interpolate(img.pixels,(float)const_temp_region.det_kp.x,
                      (float)const_temp_region.det_kp.y,
                      (float)const_temp_region.det_kp.a11*curr_sc,
                      (float)const_temp_region.det_kp.a12*curr_sc,
                      (float)const_temp_region.det_kp.a21*curr_sc,
                      (float)const_temp_region.det_kp.a22*curr_sc,
                      patch);
          EstDomOri(patch,angles1,th,maxAngNum);
          for (size_t j = 0; j < angles1.size(); j++)
            {
              double ci = cos(-angles1[j]);
              double si = sin(-angles1[j]);

              temp_region=const_temp_region;
              temp_region.det_kp.a11 = const_temp_region.det_kp.a11*ci-const_temp_region.det_kp.a12*si;
              temp_region.det_kp.a12 = const_temp_region.det_kp.a11*si+const_temp_region.det_kp.a12*ci;
              temp_region.det_kp.a21 = const_temp_region.det_kp.a21*ci-const_temp_region.det_kp.a22*si;
              temp_region.det_kp.a22 = const_temp_region.det_kp.a21*si+const_temp_region.det_kp.a22*ci;
              temp_kp_list.push_back(temp_region);
            }
        }
      if (addUpRight) {
          temp_kp_list.push_back(const_temp_region);
        }
    }
  out_kp_list=temp_kp_list;
  return (int)temp_kp_list.size();
}


int DetectAffineShape(AffineRegionList &in_kp_list,
                      AffineRegionList &out_kp_list,
                      SynthImage &img,
                      const AffineShapeParams par) {

  out_kp_list.clear();
  int kp_size = in_kp_list.size();
  const float initialSigma = 1.6;
  cv::Mat gmag, gori, orimask;
  //  std::vector<unsigned char> workspace;
  cv::Mat mask, patch, imgHes, fx, fy;

  gmag = cv::Mat(par.patchSize, par.patchSize, CV_32FC1);
  gori = cv::Mat(par.patchSize, par.patchSize, CV_32FC1);
  orimask = cv::Mat(par.patchSize, par.patchSize, CV_32FC1);
  mask = cv::Mat(par.smmWindowSize, par.smmWindowSize, CV_32FC1);
  patch = cv::Mat(par.smmWindowSize, par.smmWindowSize, CV_32FC1);
  fx = cv::Mat(par.smmWindowSize, par.smmWindowSize, CV_32FC1);
  fy = cv::Mat(par.smmWindowSize, par.smmWindowSize, CV_32FC1);

  computeGaussMask(mask);
  computeCircularGaussMask(orimask, par.smmWindowSize);
  for (int kp_num=0; kp_num < kp_size; kp_num++)
    {
      AffineRegion temp_region = in_kp_list[kp_num];
      float eigen_ratio_act = 0.0f, eigen_ratio_bef = 0.0f;
      float u11 = 1.0f, u12 = 0.0f, u21 = 0.0f, u22 = 1.0f, l1 = 1.0f, l2 = 1.0f;
      float lx = temp_region.det_kp.x, ly = temp_region.det_kp.y;
      float ratio =  temp_region.det_kp.s / (initialSigma);
      cv::Mat U, V, d, Au, Ap, D;

      const int maskPixels = par.smmWindowSize * par.smmWindowSize;
      if (interpolateCheckBorders(img.pixels.cols,img.pixels.rows,
                                  (float) temp_region.det_kp.x,
                                  (float) temp_region.det_kp.y,
                                  (float) temp_region.det_kp.a11,
                                  (float) temp_region.det_kp.a12,
                                  (float) temp_region.det_kp.a21,
                                  (float) temp_region.det_kp.a22,
                                  2*5.0*ratio,
                                  2*5.0*ratio) ) {
          continue;
        }
      for (int l = 0; l < par.maxIterations; l++)
        {
          float a = 0, b = 0, c = 0;
          if (par.affBmbrgMethod == AFF_BMBRG_SMM) {
              // warp input according to current shape matrix
              interpolate(img.pixels, lx, ly, u11*ratio, u12*ratio, u21*ratio, u22*ratio, patch);
              //            std::cerr << "after interp ok" << std::endl;
              // compute SMM on the warped patch
              float *maskptr = mask.ptr<float>(0);
              float *pfx = fx.ptr<float>(0), *pfy = fy.ptr<float>(0);
              computeGradient(patch, fx, fy);

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

            } else {
              if (par.affBmbrgMethod == AFF_BMBRG_HESSIAN) {
                  float Dxx, Dxy, Dyy;
                  float affRatio = temp_region.det_kp.s * 0.5;
                  Ap = (cv::Mat_<float>(2,2) << u11, u12, u21, u22);
                  interpolate(img.pixels, lx, ly, u11*affRatio, u12*affRatio, u21*affRatio, u22*affRatio, imgHes);

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
            }
          // compute the eigen values of the shape matrix
          if (!getEigenvalues(u11, u12, u21, u22, l1, l2))
            break;

          // leave on too high anisotropy
          if ((l1/l2>6) || (l2/l1>6))
            break;

          if (eigen_ratio_act < par.convergenceThreshold
              && eigen_ratio_bef < par.convergenceThreshold) {
              temp_region.det_kp.a11 = u11;
              temp_region.det_kp.a12 = u12;
              temp_region.det_kp.a21 = u21;
              temp_region.det_kp.a22 = u22;
              out_kp_list.push_back(temp_region);
              break;
            }
        }
    }
}

void WriteKPs(AffineRegionList &keys, std::ostream &out1)
{

  //  AffineRegionList::iterator ptr = keys.begin();
  //  int desc_size = ptr->desc.size();
  //  out1 << desc_size << " " << (int) keys.size() << std::endl;
  //  for(int i=0; i < (int) keys.size(); i++, ptr++)
  //    {
  //      out1 << ptr->reproj_kp.x << " " << ptr->reproj_kp.y << " "
  //           << ptr->reproj_kp.s << " " <<ptr->reproj_kp.a11 << " "
  //           << ptr->reproj_kp.a12 << " " << ptr->reproj_kp.a21 << " "
  //           << ptr->reproj_kp.a22 << " ";
  //      for (int ii = 0; ii < desc_size; ii++)
  //        {
  //       //   out1  << ptr->desc[ii] << " ";
  //        }

  //      out1 << std::endl;
  //    }

}


void ReadKPs(AffineRegionList &keys, std::istream &in1)
{
  //  int desc_size, keys_number;
  //  in1 >> desc_size >> keys_number;
  //  AffineRegion temp_reg;
  //  AffineRegionList temp_list;
  //  temp_list.resize(keys_number);
  //  temp_reg.described = 0;
  //  temp_reg.group_id = 0;
  //  temp_reg.img_id = 1;
  //  temp_reg.desc.resize(desc_size);
  //  for(int i=0; i < keys_number; i++)
  //    {
  //      temp_reg.id = i;

  //      in1 >> temp_reg.reproj_kp.x >> temp_reg.reproj_kp.y >> temp_reg.reproj_kp.s
  //          >> temp_reg.reproj_kp.a11 >> temp_reg.reproj_kp.a12
  //          >> temp_reg.reproj_kp.a21 >> temp_reg.reproj_kp.a22;
  //      for (int ii = 0; ii < desc_size; ii++)
  //        in1  >> temp_reg.desc[ii];
  //      temp_reg.det_kp = temp_reg.reproj_kp;
  //      temp_list[i] = temp_reg;
  //    }
  //  keys = temp_list;

}
void ReadKPsMik(AffineRegionList &keys, std::istream &in1) //Mikolajczuk.
{
  AffineRegionList temp_keys;
  AffineRegion temp_reg;
  int n_regs;
  double rub;
  double a,b,c;
  in1 >> rub;
  in1 >> n_regs;
  temp_reg.img_id = 1;
  for(int i=0; i < n_regs; i++)
    {
      temp_reg.id = i;
      in1 >> temp_reg.det_kp.x >> temp_reg.det_kp.y >> a >> b >> c;
      utls::Matrix2 C(a, b, b, c);
      utls::Matrix2 U, T1, A;
      C.inv();
      C.schur_sym(U, T1);
      A = U * T1.sqrt() * U.transpose();

      temp_reg.det_kp.a11=A[0][0];
      temp_reg.det_kp.a12=A[0][1];
      temp_reg.det_kp.a21=A[1][0];
      temp_reg.det_kp.a22=A[1][1];
      temp_reg.det_kp.response = 11.1;
      temp_reg.det_kp.s = 1/sqrt(temp_reg.det_kp.a11*temp_reg.det_kp.a22 - temp_reg.det_kp.a12*temp_reg.det_kp.a21);
      rectifyTransformation(temp_reg.det_kp.a11,temp_reg.det_kp.a12,temp_reg.det_kp.a21,temp_reg.det_kp.a22);
      temp_reg.reproj_kp =  temp_reg.det_kp;
      temp_keys.push_back(temp_reg);
    }
  keys = temp_keys;
}

void linH(const double x, const double y, double *H, double *linearH)
{
  double den, den_sq, num1_densq, num2_densq, a11,a12,a21,a22;

  den =(H[6]*x + H[7]*y +H[8]);
  den_sq=den*den;

  num1_densq = (H[0]*x + H[1]*y +H[2])/den_sq;
  num2_densq = (H[3]*x + H[4]*y +H[5])/den_sq;
  a11 = H[0]/den - num1_densq*H[6];
  a12 = H[1]/den - num1_densq*H[7];

  a21 = H[3]/den - num2_densq*H[6];
  a22 = H[4]/den - num2_densq*H[7];

  linearH[0]=a11;
  linearH[1]=a12;
  linearH[2]=a21;
  linearH[3]=a22;

}


void GetOpenCVRegionsTime (const SynthImage &orig_img,std::vector<ViewSynthParameters> &synth_par,
                           AffineRegionList &regs, int desc_type, int &unOrientedRegs, TimeLog &times1)
//Function detects interest point and describes them using given detector and descriptor.
{
  //  AffineRegionList flat_list;
  //  std::vector<AffineRegionList> reg_list1;
  //  reg_list1.resize(synth_par.size());

  //  int UnOrient1=0;
  //  double time1 = 0;
  //#pragma omp parallel for reduction (+:UnOrient1) schedule (dynamic,1)
  //  for (unsigned int i=0; i < synth_par.size(); i++)
  //    {
  //      AffineRegionList temp_kp1;
  //      SynthImage temp_img1;
  //      long s_time = getMilliSecs1();
  //      GenerateSynthImageCorr(orig_img.pixels,temp_img1,orig_img.OrigImgName,synth_par[i].tilt,
  //                             synth_par[i].phi,synth_par[i].zoom,synth_par[i].InitSigma,
  //                             synth_par[i].doBlur, i+orig_img.id);

  //      time1 = ((double)(getMilliSecs1() - s_time))/1000;
  //      times1.SynthTime += time1;
  //      s_time = getMilliSecs1();

  //      std::vector<cv::KeyPoint> keypoints_1;
  //      cv::Mat descriptors_1;

  //      switch (desc_type)
  //        {
  //        case (D_SURF):
  //          {

  //            IplImage Iplimg1 = temp_img1.pixels;
  //            IpVec ipts1;
  //            surfDetDes(&Iplimg1,ipts1,false,4,4,2,0.0004f);
  //            time1 = ((double)(getMilliSecs1() - s_time))/1000;
  //            times1.DetectTime += time1;
  //            s_time = getMilliSecs1();

  //            int kp_size = ipts1.size();
  //            temp_kp1.resize(kp_size);
  //            int desc_size = 64; //OpenSURF descriptor size

  //            for (int kp_num=0; kp_num < kp_size; kp_num++)
  //              {
  //                temp_kp1[kp_num].det_kp.x =ipts1[kp_num].x;
  //                temp_kp1[kp_num].det_kp.y = ipts1[kp_num].y;
  //                temp_kp1[kp_num].det_kp.a11 = cos(ipts1[kp_num].orientation);
  //                temp_kp1[kp_num].det_kp.a12 = sin(ipts1[kp_num].orientation);
  //                temp_kp1[kp_num].det_kp.a21 = -sin(ipts1[kp_num].orientation);
  //                temp_kp1[kp_num].det_kp.a22 = cos(ipts1[kp_num].orientation);
  //                temp_kp1[kp_num].det_kp.s = ipts1[kp_num].scale; //?
  //                temp_kp1[kp_num].desc.resize(desc_size);
  //                for (int jj=0; jj<desc_size; jj++)
  //                  temp_kp1[kp_num].desc[jj] = ipts1[kp_num].descriptor[jj];
  //              }

  //            time1 = ((double)(getMilliSecs1() - s_time))/1000;
  //            times1.DescTime += time1;
  //            s_time = getMilliSecs1();

  //            break;
  //          }
  //        case (D_ORB):
  //          {

  //            cv::OrbFeatureDetector CurrentDetector;
  //            CurrentDetector.detect(temp_img1.pixels, keypoints_1);

  //            time1 = ((double)(getMilliSecs1() - s_time))/1000;
  //            times1.DetectTime += time1;
  //            s_time = getMilliSecs1();

  //            cv::OrbFeatureDetector CurrentDescriptor;
  //            CurrentDescriptor.compute(temp_img1.pixels,keypoints_1,descriptors_1);

  //            break;
  //          }
  //        case (D_BRISK):
  //          {
  //            //            cv::OrbFeatureDetector CurrentDetector;
  //            //            CurrentDetector.detect(temp_img1.pixels, keypoints_1);

  //            //            cv::Ptr<cv::FeatureDetector> CurrentDetector;
  //            //            CurrentDetector = new cv::BRISK(20,0);
  //            //            CurrentDetector = new cv::BriskFeatureDetector(20,0);
  //            //            CurrentDetector->detect(temp_img1.pixels,keypoints_1);


  //            cv::BRISK CurrentDetector(40,0,1.0);
  //            CurrentDetector.detect(temp_img1.pixels,keypoints_1);

  //            time1 = ((double)(getMilliSecs1() - s_time))/1000;
  //            times1.DetectTime += time1;
  //            s_time = getMilliSecs1();
  //            CurrentDetector.compute(temp_img1.pixels,keypoints_1,descriptors_1);

  //            time1 = ((double)(getMilliSecs1() - s_time))/1000;
  //            times1.DetectTime += time1;
  //            s_time = getMilliSecs1();
  //            break;
  //          }
  //        case (D_FREAK_AGAST):
  //          {

  //            //    cv::Ptr<cv::FeatureDetector> CurrentDetector;
  //            //    CurrentDetector = new cv::BriskFeatureDetector(30,0);
  //            //    CurrentDetector->detect(temp_img1.pixels,keypoints_1);

  //            cv::FAST(temp_img1.pixels,keypoints_1,30);

  //            time1 = ((double)(getMilliSecs1() - s_time))/1000;
  //            times1.DetectTime += time1;
  //            s_time = getMilliSecs1();

  //            cv::FREAK CurrentDescriptor;
  //            CurrentDescriptor.compute(temp_img1.pixels,keypoints_1,descriptors_1);
  //            break;
  //          }
  //        case (D_FREAK_SURF):
  //          {

  //            cv::SurfFeatureDetector CurrentDetector(400);
  //            CurrentDetector.detect(temp_img1.pixels, keypoints_1);


  //            time1 = ((double)(getMilliSecs1() - s_time))/1000;
  //            times1.DetectTime += time1;
  //            s_time = getMilliSecs1();

  //            cv::FREAK CurrentDescriptor;
  //            CurrentDescriptor.compute(temp_img1.pixels,keypoints_1,descriptors_1);
  //            break;
  //          }
  //        }
  //      if (desc_type != D_SURF)
  //        {
  //          int kp_size = keypoints_1.size();
  //          int desc_size = descriptors_1.cols;

  //          temp_kp1.resize(kp_size);

  //          for (int kp_num=0; kp_num<kp_size; kp_num++)
  //            {
  //              temp_kp1[kp_num].det_kp.x = keypoints_1[kp_num].pt.x;
  //              temp_kp1[kp_num].det_kp.y = keypoints_1[kp_num].pt.y;
  //              temp_kp1[kp_num].det_kp.a11 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
  //              temp_kp1[kp_num].det_kp.a12 = sin(keypoints_1[kp_num].angle*M_PI/180.0);
  //              temp_kp1[kp_num].det_kp.a21 = -sin(keypoints_1[kp_num].angle*M_PI/180.0);
  //              temp_kp1[kp_num].det_kp.a22 = cos(keypoints_1[kp_num].angle*M_PI/180.0);
  //              temp_kp1[kp_num].det_kp.s = keypoints_1[kp_num].size /3.0; //?
  //              temp_kp1[kp_num].det_kp.response = keypoints_1[kp_num].response;
  //              temp_kp1[kp_num].desc.resize(desc_size);

  //              unsigned char *descPtr = descriptors_1.ptr<unsigned char>(kp_num);
  //              for (int jj=0; jj<desc_size; jj++, descPtr++)
  //                temp_kp1[kp_num].desc[jj] = (float) *descPtr;

  //            }
  //          time1 = ((double)(getMilliSecs1() - s_time))/1000;
  //          times1.DescTime += time1;
  //          s_time = getMilliSecs1();
  //        }
  //      //put everything into standard array;
  //      UnOrient1 +=ReprojectRegions(temp_kp1, temp_img1.H, orig_img.pixels.cols, orig_img.pixels.rows);
  //      reg_list1[i] = temp_kp1;

  //    }
  //  for (unsigned int i=0 ; i < synth_par.size(); i++)
  //    AddRegionsToList(flat_list,reg_list1[i]);

  //  regs = flat_list;
  //  unOrientedRegs += UnOrient1;
}

