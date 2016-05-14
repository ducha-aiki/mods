/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/

#ifndef MATCHING_HPP
#define MATCHING_HPP

#include "../detectors/structures.hpp"
#include "../descriptors_parameters.hpp"
#include "siftdesc.h"
#include "../degensac/Fcustomdef.h"
#include <opencv2/flann/flann.hpp>

#ifdef __cplusplus
extern "C"
#endif
void FDsSym (const double *u, const double *F, double *p, int len);


//matching libraries
//#define LIB_FLANN 0
//#define LIB_VLFEAT 1
//#define LIB_BRUTEFORCE 2

//RANSAC_errors

#define MIN_POINTS 8 //threshold for symmetrical error check
#define USE_SECOND_BAD 1//uncomment if you need to use/output 1st geom.inconsistent region
///
#include "../configuration.hpp"
///

struct TentativeCorresp
{
  AffineRegion first;
  AffineRegion second;
};

struct TentativeCorrespExt : TentativeCorresp
{
#ifdef USE_SECOND_BAD
  AffineRegion secondbadby2ndcl;
  AffineRegion secondbad;
#endif
  double d1;
  double d2;
  double d2by2ndcl;
  double d2byDB;
  double ratio;
  int isTrue;
};

struct Keypoint4Match     //structure for the bruteforce matching only
{
  float x,y;
  int parent_id;           //parent region id (when detect orientation). = -1 when parent is undefined;
  int group_id;            //group id
  unsigned char desc[128]; //SIFT descriptor

};
struct Keypoint4OverlapMatch
{
  double x,y, a11,a12,a21,a22,s;
};

typedef std::vector<Keypoint4Match> Keypoint4MatchList;
typedef std::vector<Keypoint4OverlapMatch> Keypoint4OverlapMatchList;


struct TentativeCorrespList
{
  std::vector<TentativeCorresp> TCList;
  double H[3*3]; // by default H[i] = -1, if no H-estimation done
  TentativeCorrespList()
  {
    for (int i=0; i<9; i++)
      H[i] = -1;
  }

};


struct TentativeCorrespListExt : TentativeCorrespList
{
  std::vector<TentativeCorrespExt> TCList;
  double H[3*3]; // by default H[i] = -1, if no H-estimation done
  TentativeCorrespListExt()
  {
    for (int i=0; i<9; i++)
      H[i] = -1;
  }

};

enum RANSAC_error_t {SAMPSON,SYMM_MAX,SYMM_SUM};

struct MatchPars
{
  std::vector <WhatToMatch> IterWhatToMatch;
  std::map <std::string, double> FGINNThreshold;
  std::map <std::string, double> DistanceThreshold;
  double currMatchRatio;
  double matchDistanceThreshold;

  double contradDist;
  int standard_2nd_closest;
  int kd_trees;
  int knn_checks;
  int RANSACforStopping;
  int minMatches;
  int maxSteps;
  int doBothRANSACgroundTruth;
  int doOverlapMatching;
  double overlapError;
  cvflann::flann_algorithm_t binary_matcher;
  cvflann::flann_algorithm_t vector_matcher;
  cvflann::flann_distance_t binary_dist;
  cvflann::flann_distance_t vector_dist;
  int doDensification;
  double FPRate;
  int useDBforFGINN;
  std::string SIFTDBfile;
  MatchPars()
  {
    SIFTDBfile="100_db.txt";
    useDBforFGINN=0;
    currMatchRatio = -1.0;
    contradDist = 10.0;
    standard_2nd_closest = 0;
    kd_trees = 4;
    knn_checks = 128;
    RANSACforStopping=1;
    minMatches = 15;
    maxSteps = 4;
    doBothRANSACgroundTruth = 1;
    doOverlapMatching = 0;
    overlapError = 0.09;
    binary_matcher = cvflann::FLANN_INDEX_HIERARCHICAL;
    vector_matcher = cvflann::FLANN_INDEX_KDTREE;
    doDensification=0;
    FPRate = 0.8;
    matchDistanceThreshold = 64.0;
  }
};

struct RANSACPars
{
  int useF;
  double err_threshold;
  double confidence;
  int max_samples;
  int localOptimization;
  double LAFCoef;
  double HLAFCoef;
  RANSAC_error_t errorType;
  int doSymmCheck;
  int justMarkOutliers;
  RANSACPars()
  {
    useF=0;
    err_threshold = 2.0;
    confidence = 0.99;
    max_samples = 1e5;
    localOptimization = 1;
    LAFCoef = 3.0;
    HLAFCoef = 10.0;
    errorType = SYMM_SUM;
    doSymmCheck = 0;
    justMarkOutliers=0;
  }
};
/* Correspondence for drawing: */
typedef std::pair<cv::Point2f,cv::Point2f> corresp;

void AddMatchingsToList(TentativeCorrespListExt &tent_list, TentativeCorrespListExt &new_tents);

template<typename T>
double ellipseOverlapH(T ref_kp, T test_kp, double *H, const double max_error=10000.)
//Computes overlap error between two ellipses
//E=0.5||I-AHB^-1||f +d(a,b),
//I = [1 0; 0 1], A,B - ellipse matrices, H - homography matrix, d(a,b) - distance between ellipse centers in canonical coordinate frame.
//d(a,b) is computed first. if  d(a,b)> max_error, other part doesn`t computed - for speed reasons.
//E=0 means that ellipses are the same.
//Function isn`t used there. Instead optimised version ellipseOverlapHPrep, that requires prepared data, is used)
{
  double Hlin[4];
  linH(test_kp.x, test_kp.y, H, Hlin);

  double k = 3.0; //to compare ellipses in 3*sigma size
  double diff, dist;
  double B1Ptr[4]= {test_kp.a11, test_kp.a12,
                    test_kp.a21, test_kp.a22
                   };
  cv::Mat B1(2,2,CV_64F, B1Ptr);
  double A1Ptr[4]= {ref_kp.a11, ref_kp.a12,
                    ref_kp.a21, ref_kp.a22
                   };
  cv::Mat A1(2,2,CV_64F, A1Ptr);
  cv::Mat H1lin(2,2,CV_64F, Hlin);
  cv::Mat Ainv(2,2,CV_64F);
  cv::invert(k*ref_kp.s*A1,Ainv, cv::DECOMP_LU); //inverting A
  double* AinvPtr = (double*)Ainv.data;

  double x1 = (AinvPtr[0]*(double)ref_kp.x+AinvPtr[1]*(double)ref_kp.y);
  double y1 = (AinvPtr[2]*(double)ref_kp.x+AinvPtr[3]*(double)ref_kp.y);

  double den = (H[6]*(double)test_kp.x+H[7]*(double)test_kp.y + H[8]);

  double x2b = (H[0]*(double)test_kp.x+H[1]*(double)test_kp.y + H[2])/den;
  double y2b = (H[3]*(double)test_kp.x+H[4]*(double)test_kp.y + H[5])/den;

  double x2 = (AinvPtr[0]*x2b+AinvPtr[1]*y2b);
  double y2 = (AinvPtr[2]*x2b+AinvPtr[3]*y2b);

  dist=(x2-x1)*(x2-x1)+(y2-y1)*(y2-y1); //distance between ellipse centers in canonical coordinate frame
  if (dist>max_error) return dist; //speed-up

  cv::gemm(Ainv,H1lin,1, H1lin,0,Ainv);
  cv::gemm(Ainv,k*test_kp.s*B1,1, B1,0,Ainv);

  diff = 0.5*((1 - AinvPtr[0])*(1 - AinvPtr[0]) +  AinvPtr[1]* AinvPtr[1] + AinvPtr[2]* AinvPtr[2]+ (1-AinvPtr[3])*(1-AinvPtr[3]));
  //distance between ellipse shapes in canonical coordinate frame
  return (diff+dist);
}

template<typename T>
double ellipseOverlapHPrep(T ref_kp, T test_kp,const double max_error=10000., const int matchOriented = 1)
//Computes overlap error between two ellipses
{
  double diff, dist;
  double A1Ptr[4]= {ref_kp.a11, ref_kp.a12,
                    ref_kp.a21, ref_kp.a22
                   };
  double x1,x2,y1,y2, dx, dy;

  x1 = ref_kp.x;
  y1 = ref_kp.y;

  x2 = (A1Ptr[0]*test_kp.x+A1Ptr[1]*test_kp.y);
  y2 = (A1Ptr[2]*test_kp.x+A1Ptr[3]*test_kp.y);

  dx=x2-x1;
  dy=y2-y1;

  dist=dx*dx+dy*dy; //distance between ellipse centers in canonical coordinate frame

  if (dist>max_error) return dist; //speed-up
  double BPtr[4]= {test_kp.a11, test_kp.a12,
                   test_kp.a21, test_kp.a22
                  };
  cv::Mat B(2,2,CV_64F, BPtr);
  cv::Mat A(2,2,CV_64F, A1Ptr);
  double* APtr = (double*)A.data;

  cv::gemm(A,B,1, B,0,A);
  if (!matchOriented) rectifyAffineTransformationUpIsUp(APtr);
  diff = 0.5*((1 - APtr[0])*(1 - APtr[0]) +  APtr[1]* APtr[1] + APtr[2]* APtr[2]+ (1-APtr[3])*(1-APtr[3]));
  //distance between ellipse shapes in canonical coordinate frame
  return (diff+dist);
}
int MatchKeypoints(std::vector< std::map<std::string, AffineRegionList> > &list1,
                   std::vector< std::map<std::string, AffineRegionList> > &list2,
                   std::map<std::string, TentativeCorrespListExt> tentatives,
                   const MatchPars &par,
                   const DescriptorsParameters &desc_pars);
cv::flann::Index GenFLANNIndex(cv::Mat keys, cvflann::flann_algorithm_t indexType, cvflann::flann_distance_t dist_type, const int nTrees = 4);

int MatchFlannFGINN(const AffineRegionList &list1, const AffineRegionList &list2,
                  TentativeCorrespListExt &corresp,const MatchPars &par, const int nn=50);
int MatchFlannFGINNPlusDB(const AffineRegionList &list1, const AffineRegionList &list2,
                          TentativeCorrespListExt &corresp,const MatchPars &par,cv::Mat *DB, const int nn=50);

int MatchFLANNDistance(const AffineRegionList &list1, const AffineRegionList &list2,
                  TentativeCorrespListExt &corresp,const MatchPars &par, const int nn=50);


int DensificationByHomography(const AffineRegionList &list1, const AffineRegionList &list2,double *H,
                              TentativeCorrespListExt &in_corresp,TentativeCorrespListExt &out_corresp,const MatchPars &par, const double max_error=0.09,const int matchOriented=1);

int MatchRegionsByOverlapFastFLANN(const AffineRegionList &list1, const AffineRegionList &list2, double *H,
                                   TentativeCorrespListExt &corresp,const double max_error = 0.09, const int matchOriented = 1);
//Function does overlap FLANN matching of the affine regions. "Fast" means that ellipseOverlapHPrep is used
//Iterative functions checks "matched" parameter for not calculating already calculated distances. (for speed reasons)
int LORANSACFiltering(TentativeCorrespListExt &in_corresp,
                      TentativeCorrespListExt &out_corresp, double *H,
                      const RANSACPars pars);
//Functions finds the inliers using LO-RANSAC and puts them into out_corresp list. Also it stores
//homography matrix H or fundamental matrix F.

#ifdef WITH_ORSA
int ORSAFiltering(TentativeCorrespListExt &in_corresp, TentativeCorrespListExt &ransac_corresp,double *F, const RANSACPars pars, int w, int h);
#endif
int HMatrixFiltering(TentativeCorrespListExt &in_corresp,
                     TentativeCorrespListExt &true_corresp,
                     double *H, const int isExtended = 0,
                     const RANSACPars pars = RANSACPars());
//Functions finds the inliers using ground truth homography matrix H.

//void DuplicateFiltering(TentativeCorrespList &in_corresp, const double r = 3.0);
void DuplicateFiltering(TentativeCorrespListExt &in_corresp, const double r = 3.0, const int mode = MODE_RANDOM);
//Function does pairwise computing of the distance between ellipse centers in 1st and 2nd images.
//If distance^2 < r_sq in both images, correspondences are considered as duplicates and
//second point is deleted.


cv::Mat DrawRegions(const cv::Mat &in_img,
                         const AffineRegionList kps,
                         const int r1 = 7,
                         const cv::Scalar color1 = cv::Scalar(255,0,0));

void DrawMatchingsSimple(const cv::Mat &in_img, cv::Mat &out_img, const cv::Mat &H1,
                         std::vector<corresp> matchings,const int order = 1,
                         const int r1 = 7,const int r2 = 4,
                         const cv::Scalar color1 = cv::Scalar(255,0,0),
                         const cv::Scalar color2 = cv::Scalar(0,255,0));
//Draws correspondences. Points from original image are drawn as circles radius r1 and color color1,
//while points from second image are reprojected be matrix h and drawn as circles radius r2 and color color2
//Flag "order" shows which points from the correspondences list are "original"[1 - first, 0 - second]

void DrawMatches(const cv::Mat &in_img1,const cv::Mat &in_img2, cv::Mat &out_img1,cv::Mat &out_img2,const cv::Mat &H,
                 TentativeCorrespListExt matchings,
                 const int DrawCentersOnly = 1,
                 const int ReprojectToOneImage = 1,
                 const int r1=2,
                 const int r2=2,
                 const int drawEpipolarLines =0,
                 const int useSCV=0,
                 const double LAFcoef = 0,
                 const cv::Scalar color1= cv::Scalar(255,0,0),
                 const cv::Scalar color2= cv::Scalar(0,255,0));


void DrawMatchesWithError(const cv::Mat &in_img1,const cv::Mat &in_img2, cv::Mat &out_img1, cv::Mat &out_img2,const cv::Mat &H1,
                          std::vector<double> Errors, double max_err,
                          TentativeCorrespListExt matchings,
                          const int DrawCentersOnly = 1,
                          const int ReprojectToOneImage = 0,
                          const int r1 = 2,
                          const int r2 = 2,
                          const int drawEpipolarLines = 1,
                          const int useSCV = 0,
                          const double LAFcoef = 0,
                          const cv::Scalar color1= cv::Scalar(255,0,0),
                          const cv::Scalar color2= cv::Scalar(0,255,0),
                          const cv::Scalar color_err = cv::Scalar(0,0,255));

void DrawMatchingRegions(const cv::Mat &in_img, cv::Mat &out_img,const cv::Mat &H1, TentativeCorrespList matchings,
                         const int order = 1,
                         const int r1 = 2,const int r2 = 2,
                         const cv::Scalar color1 = cv::Scalar(255,0,0),
                         const cv::Scalar color2 = cv::Scalar(0,255,0));
#ifdef USE_SECOND_BAD
void DrawChangedMatchingRegions(const cv::Mat &in_img, cv::Mat &out_img,const cv::Mat &H1, TentativeCorrespListExt matchings,
                                TentativeCorrespListExt matchings2nd,
                                const int order = 1,
                                const int r1 = 2,const int r2 = 2,
                                const cv::Scalar color1 = cv::Scalar(255,0,0),
                                const cv::Scalar color2 = cv::Scalar(0,255,0));
#endif
void DrawMatchingRegions3D(const cv::Mat &in_img1,const cv::Mat &in_img2,
                           cv::Mat &out_img,const cv::Mat &F1,
                           TentativeCorrespListExt matchings,
                           const int conc_horiz = 1,
                           const int r1 = 2,const int r2 = 2,
                           const cv::Scalar color1 = cv::Scalar(255,0,0),
                           const cv::Scalar color2 = cv::Scalar(0,255,0));


//void WriteMatchings(TentativeCorrespList &match, std::ostream &out1);
void WriteMatchings(TentativeCorrespListExt &match, std::ostream &out1, int writeWithRatios = 0);
//Writes matchings in format: number x1 y1 x2 y2

//void WriteMatchingsAll(TentativeCorrespListExt &match, std::ostream &out1);
//Writes matchings in format: number x1 y1 x2 y2 [1/0] (correct/incorrect)

int NaiveHCheck(TentativeCorrespListExt &corresp,double *H,const double error);
//Performs check if the symmetrical reprojection error > given error. Returns number of "bad" points

int F_LAF_check(std::vector<TentativeCorrespExt> &in_matches, double *F, std::vector<TentativeCorrespExt> &res,const double affineFerror = 12.0,FDsPtr FDS1= FDsSym);
//Performs check if the full local affine frame is consistent with F-matrix.
//Error function is given in FDsPtr by user

void WriteH(double* H, std::ostream &out1);
//Writes homography matrix 3*3 into stream or file

double L2_scalar(Keypoint4Match &k1,Keypoint4Match &k2);
//Function to fast check L2 SIFT distance
//(k1[i]-k2[i])^2 = k1[i]^2 + k2[i]^2 - 2*k1[i]*k2[i]. Sum(k1[i]^2) = Sum(k2[i]^2) = 512^2, because
//SIFT vectors are normalized to 512.
//So, it is possible to compute sum(k1[i]*k2[i]) only.

#endif // MATCHING_HPP
