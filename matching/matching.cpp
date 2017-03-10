/*------------------------------------------------------*/
/* Copyright 2013, Dmytro Mishkin  ducha.aiki@gmail.com */
/*------------------------------------------------------*/

#undef __STRICT_ANSI__

#include "../degensac/exp_ranF.h"
#include "../degensac/exp_ranH.h"
#include "../opencv_3_0_compatibility.hpp"
#include "ranH.h"
#include "ranF.h"
//#include "rtools.h"
//#include "Htools.h"

#include "matching.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <fstream>

#define DO_TRANSFER_H_CHECK



#ifdef WITH_VLFEAT
#include <kdtree.h>
#include <host.h>
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef WITH_ORSA
#include "../orsa.h"
#endif

#define WRITE_H 1
#define VERB 0
using namespace std;

//
#ifdef __cplusplus
extern "C"
#endif
void FDsfull (const double *u, const double *F, double *p, int len);

#ifdef __cplusplus
extern "C"
#endif
void FDs (const double *u, const double *F, double *p, int len);


#ifdef __cplusplus
extern "C"
#endif
void exFDs (const double *u, const double *F, double *p, double *w, int len);

#ifdef __cplusplus
extern "C"
#endif
void FDsSym (const double *u, const double *F, double *p, int len);


#ifdef __cplusplus
extern "C"
#endif
void exFDsSym (const double *u, const double *F, double *p, double *w, int len);

bool CompareCorrespondenceByRatio(TentativeCorrespExt corr1, TentativeCorrespExt corr2) {return (fabs(corr1.ratio) < fabs(corr2.ratio));}
bool CompareCorrespondenceByDistance(TentativeCorrespExt corr1, TentativeCorrespExt corr2) {return (fabs(corr1.d1) < fabs(corr2.d1));}
bool CompareCorrespondenceByScale(TentativeCorrespExt corr1, TentativeCorrespExt corr2) {return (fabs(corr1.first.reproj_kp.s) < fabs(corr2.first.reproj_kp.s));}

//
cv::flann::Index GenFLANNIndex(cv::Mat keys, cvflann::flann_algorithm_t indexType, cvflann::flann_distance_t dist_type, const int nTrees)
{
  switch (indexType)
    {
    case cvflann::FLANN_INDEX_KDTREE:
      {
        return  cv::flann::Index(keys,cv::flann::KDTreeIndexParams(nTrees),dist_type);
        break;
      }
    case cvflann::FLANN_INDEX_COMPOSITE:
      {
        return  cv::flann::Index(keys,cv::flann::CompositeIndexParams(nTrees),dist_type);
        break;
      }
    case cvflann::FLANN_INDEX_AUTOTUNED:
      {
        return cv::flann::Index(keys,cv::flann::AutotunedIndexParams(0.8,0.9),dist_type);
        break;
      }
    case cvflann::FLANN_INDEX_KMEANS:
      {
        return cv::flann::Index(keys,cv::flann::KMeansIndexParams(),dist_type);
        break;
      }
    case cvflann::FLANN_INDEX_LSH:
      {
        return cv::flann::Index(keys,cv::flann::LshIndexParams(30, 8, 2),dist_type);
        break;
      }
    case cvflann::FLANN_INDEX_LINEAR:
      {
        return cv::flann::Index(keys,cv::flann::LinearIndexParams(),dist_type);
        break;
      }
    case cvflann::FLANN_INDEX_HIERARCHICAL:
      {
        return cv::flann::Index(keys,cv::flann::HierarchicalClusteringIndexParams(),dist_type);
        break;
      }
    default:
      {
        return cv::flann::Index(keys,cv::flann::KDTreeIndexParams(nTrees),dist_type);
        break;
      }
    }

}

void  GetEpipoles (double *F, double *e1, double *e2)
{
  cv::Mat Fmat (3,3,CV_64F,F);
  cv::Mat U,D,V;
  cv::SVDecomp(Fmat,D,U,V,4);


  e2[0] = U.at<double>(0,2) / U.at<double>(2,2);
  e2[1] = U.at<double>(1,2) / U.at<double>(2,2);
  e2[2] = 1.0;

  e1[0] = V.at<double>(0,2) / V.at<double>(2,2);
  e1[1] = V.at<double>(1,2) / V.at<double>(2,2);
  e1[2] = 1.0;

}
void GetEpipolarLine(double *e, double *pt, double *l, double &k, double &b)
{
  l[0] = e[1]*pt[2] - e[2]*pt[1];
  l[1] = e[2]*pt[0] - e[0]*pt[2];
  l[2] = e[0]*pt[1] - e[1]*pt[0];

  double x_crossx = - l[2] / l[0];
  double x_crossy = 0;
  double y_crossx = 0;
  double y_crossy = -l[2] / l[1];
  k = (y_crossx - y_crossy)/(x_crossx - x_crossy);
  b = y_crossy;
}


void GetEpipolarLineF(double *F, double *pt, double *l, double &k, double &b)
{

  l[0] = pt[0]*F[0] + pt[1]*F[3] + pt[2]*F[6];
  l[1] = pt[0]*F[1] + pt[1]*F[4] + pt[2]*F[7];
  l[2] = pt[0]*F[2] + pt[1]*F[5] + pt[2]*F[8];

  double x_crossx = - l[2] / l[0];
  double x_crossy = 0;
  double y_crossx = 0;
  double y_crossy = -l[2] / l[1];
  k = (y_crossx - y_crossy)/(x_crossx - x_crossy);
  b = y_crossy;
}
//
const double k_sigma = 3.0;

inline double distanceSq (const AffineKeypoint &kp1,const AffineKeypoint &kp2)
{
  double dx = kp1.x - kp2.x;
  double dy = kp1.y - kp2.y;
  return dx*dx + dy*dy;
}
inline void oppositeDirection (AffineRegion &kp1)
{
  kp1.reproj_kp.a11 = - kp1.reproj_kp.a11;
  kp1.reproj_kp.a12 = - kp1.reproj_kp.a12;
  kp1.reproj_kp.a21 = - kp1.reproj_kp.a21;
  kp1.reproj_kp.a22 = - kp1.reproj_kp.a22;

  kp1.det_kp.a11 = - kp1.det_kp.a11;
  kp1.det_kp.a12 = - kp1.det_kp.a12;
  kp1.det_kp.a21 = - kp1.det_kp.a21;
  kp1.det_kp.a22 = - kp1.det_kp.a22;
}

int F_LAF_check(std::vector<TentativeCorrespExt> &in_matches, double *F, std::vector<TentativeCorrespExt> &res,const double affineFerror, FDsPtr FDS1)
{
  int n_tents = (int)in_matches.size();
  int bad_pts=0;
  std::vector<TentativeCorrespExt> good_matches;
  std::vector<int> good_pts(n_tents);
  for (int a=0; a<n_tents; a++)
    good_pts[a]=1; //initialization


  if (affineFerror > 0)
    {

      std::vector<TentativeCorrespExt>::iterator ptr =  in_matches.begin();
      for (int l=0; l<n_tents; l++,ptr++)
        {
          double u[18],err[3];
          u[0] = ptr->first.reproj_kp.x;
          u[1] = ptr->first.reproj_kp.y;
          u[2] = 1.0;

          u[3] = ptr->second.reproj_kp.x;
          u[4] = ptr->second.reproj_kp.y;
          u[5] = 1.0;

          u[6] = u[0]+k_sigma*ptr->first.reproj_kp.a12*ptr->first.reproj_kp.s;
          u[7] = u[1]+k_sigma*ptr->first.reproj_kp.a22*ptr->first.reproj_kp.s;
          u[8] = 1.0;

          u[9]  = u[3]+k_sigma*ptr->second.reproj_kp.a12*ptr->second.reproj_kp.s;
          u[10] = u[4]+k_sigma*ptr->second.reproj_kp.a22*ptr->second.reproj_kp.s;
          u[11] = 1.0;

          u[12] = u[0]+k_sigma*ptr->first.reproj_kp.a11*ptr->first.reproj_kp.s;
          u[13] = u[1]+k_sigma*ptr->first.reproj_kp.a21*ptr->first.reproj_kp.s;
          u[14] = 1.0;

          u[15] = u[3]+k_sigma*ptr->second.reproj_kp.a11*ptr->second.reproj_kp.s;
          u[16] = u[4]+k_sigma*ptr->second.reproj_kp.a21*ptr->second.reproj_kp.s;
          u[17] = 1.0;

          FDS1(u,F,err,3);
          double sumErr=sqrt(err[0])+sqrt(err[1])+sqrt(err[2]);
          if (sumErr > affineFerror)
            // if ((sqrt(err[0]) > affineFerror) || (sqrt(err[1]) > affineFerror) || (sqrt(err[2]) > affineFerror))
            {
              good_pts[l]=0;
              bad_pts++;
            }
        }
      good_matches.reserve(n_tents - bad_pts);
      for (int l=0; l<n_tents; l++)
        if (good_pts[l]) good_matches.push_back(in_matches[l]);
      res = good_matches;
    }
  else res = in_matches;
  return res.size();
}
int H_LAF_check(std::vector<TentativeCorrespExt> &in_matches, double *H, std::vector<TentativeCorrespExt> &res,const double affineFerror, HDsPtr HDS1)
{
  int n_tents = (int)in_matches.size();
  int bad_pts=0;
  std::vector<TentativeCorrespExt> good_matches;
  std::vector<int> good_pts(n_tents);
  for (int a=0; a<n_tents; a++)
    good_pts[a]=1; //initialization

  double *lin2Ptr = new double[n_tents*6], *lin;
  lin=lin2Ptr;

  if (affineFerror > 0)
    {
      std::vector<TentativeCorrespExt>::iterator ptr =  in_matches.begin();
      for (int l=0; l<n_tents; l++,ptr++)
        {
          double u[18],err[3];
          u[0] = ptr->first.reproj_kp.x;
          u[1] = ptr->first.reproj_kp.y;
          u[2] = 1.0;

          u[3] = ptr->second.reproj_kp.x;
          u[4] = ptr->second.reproj_kp.y;
          u[5] = 1.0;

          u[6] = u[0]+k_sigma*ptr->first.reproj_kp.a12*ptr->first.reproj_kp.s;
          u[7] = u[1]+k_sigma*ptr->first.reproj_kp.a22*ptr->first.reproj_kp.s;
          u[8] = 1.0;

          u[9]  = u[3]+k_sigma*ptr->second.reproj_kp.a12*ptr->second.reproj_kp.s;
          u[10] = u[4]+k_sigma*ptr->second.reproj_kp.a22*ptr->second.reproj_kp.s;
          u[11] = 1.0;

          u[12] = u[0]+k_sigma*ptr->first.reproj_kp.a11*ptr->first.reproj_kp.s;
          u[13] = u[1]+k_sigma*ptr->first.reproj_kp.a21*ptr->first.reproj_kp.s;
          u[14] = 1.0;

          u[15] = u[3]+k_sigma*ptr->second.reproj_kp.a11*ptr->second.reproj_kp.s;
          u[16] = u[4]+k_sigma*ptr->second.reproj_kp.a21*ptr->second.reproj_kp.s;
          u[17] = 1.0;
          HDS1(lin,u,H,err,3);

          double sumErr=sqrt(err[0] + err[1] + err[2]);
          if (sumErr > affineFerror)
            {
              good_pts[l]=0;
              bad_pts++;
            }
        }
      good_matches.reserve(n_tents - bad_pts);
      for (int l=0; l<n_tents; l++)
        if (good_pts[l]) good_matches.push_back(in_matches[l]);
      res = good_matches;
    }
  else res = in_matches;
  delete [] lin;
  return res.size();
}
void AddMatchingsToList(TentativeCorrespListExt &tent_list, TentativeCorrespListExt &new_tents)
{
  int size = (int)tent_list.TCList.size();
  unsigned int new_size = size + (int)new_tents.TCList.size();
  std::vector<TentativeCorrespExt>::iterator ptr =new_tents.TCList.begin();
  for (unsigned int i=size; i< new_size; i++, ptr++)
    tent_list.TCList.push_back(*ptr);
}


float BFOverlapMatchPrepFLANN(Keypoint4OverlapMatch& key, Keypoint4OverlapMatchList& klist, std::vector<int> &indices, int &min, double &first_dist, double &second_dist, const double max_error = 1000., const int matchOriented = 1)
{
  unsigned int i;
  float overlap_err_curr, overlap_err1;
  overlap_err1 = first_dist;
  for (i=0; i< indices.size(); i++)
    {
      overlap_err_curr = ellipseOverlapHPrep(key, klist[indices[i]],max_error, matchOriented);
      if (overlap_err_curr < overlap_err1)
        {
          overlap_err1 = overlap_err_curr;
          min = indices[i];
        }
    }
  first_dist = overlap_err1;
  return overlap_err1;

  //uncomment, if want to use first-to-second ratio
  /*
     unsigned int i;
     float overlap_err_curr, overlap_err1, overlap_err2;
     overlap_err1 = first_dist;
     for (i=0; i< klist.size(); i++){
                 overlap_err_curr = ellipseOverlapHPrep(key, klist[i],max_error);
         if (overlap_err_curr < overlap_err1) {
                 overlap_err2 = overlap_err1;
             overlap_err1 = overlap_err_curr;
             min = i;
         } else if (overlap_err_curr < overlap_err2)
                 overlap_err2 = overlap_err_curr;
     }
     first_dist = overlap_err1;
     second_dist = overlap_err2;
     return overlap_err1/overlap_err2; */

}

int MatchFlannFGINN(const AffineRegionList &list1, const AffineRegionList &list2, TentativeCorrespListExt &corresp,const MatchPars &par, const int nn)
{
  double sqminratio = par.currMatchRatio* par.currMatchRatio;
  double contrDistSq = par.contradDist *par.contradDist;
  unsigned int i,j;
  int matches = 0;
  if (list1.size() == 0) return 0;
  if (list2.size() == 0) return 0;

  unsigned int desc_size = list1[0].desc.vec.size();

  corresp.TCList.reserve((int)(list1.size()/10));

  cv::Mat keys1,keys2;
  keys1 = cv::Mat(list1.size(), desc_size, CV_32F);
  keys2 = cv::Mat(list2.size(), desc_size, CV_32F);

  for (i=0; i <list1.size(); i++)
    {
      float* Row = keys1.ptr<float>(i);
      for (j=0; j < desc_size; j++)
        Row[j] = list1[i].desc.vec[j];
    }

  for (i=0; i <list2.size(); i++)
    {
      float* Row = keys2.ptr<float>(i);
      for (j=0; j < desc_size; j++)
        Row[j] = list2[i].desc.vec[j];
    }

  cv::flann::Index tree = GenFLANNIndex(keys2,par.vector_matcher,par.vector_dist,par.kd_trees);

  cv::Mat indices;//(numQueries, k, CV_32S);
  cv::Mat dists;//(numQueries, k, CV_32F);


  cv::flann::SearchParams SearchParams1(par.knn_checks);
  tree.knnSearch(keys1, indices, dists, nn, SearchParams1);

  if (sqminratio >= 1.0) //to get all points (for example, for calculating PDF)
    {
      for (i=0; i< list1.size(); i++)
        {
          int* indicesRow=indices.ptr<int>(i);
          float* distsRow=dists.ptr<float>(i);
          for (int j=1; j<nn; j++)
            {
              double ratio = distsRow[0]/distsRow[j];
              double dist1 = distanceSq(list2[indicesRow[0]].reproj_kp,list2[indicesRow[j]].reproj_kp);
              if ((j == nn-1) || (dist1 > contrDistSq) /*|| (ratio <= sqminratio) */)
                {
                  TentativeCorrespExt tmp_corr;
                  tmp_corr.first = list1[i];
                  tmp_corr.second = list2[indicesRow[0]];
#ifdef USE_SECOND_BAD
                  tmp_corr.secondbad = list2[indicesRow[j]];
                  tmp_corr.secondbadby2ndcl = list2[indicesRow[1]];
                  tmp_corr.d2by2ndcl = distsRow[1];

#endif
                  tmp_corr.d1 = distsRow[0];
                  tmp_corr.d2 = distsRow[j];
                  tmp_corr.ratio = sqrt(ratio);
                  corresp.TCList.push_back(tmp_corr);
                  matches++;
                  break;
                };
            }
        }

    }
  else
    {
      for (i=0; i< list1.size(); i++)
        {
          int* indicesRow=indices.ptr<int>(i);
          float* distsRow=dists.ptr<float>(i);
          for (int j=1; j<nn; j++)
            {
              double ratio = distsRow[0]/distsRow[j];
              if ((ratio <= sqminratio ))// || (distsRow[0] <= (float)par.matchDistanceThreshold))
                {
                  TentativeCorrespExt tmp_corr;
                  tmp_corr.first = list1[i];
                  tmp_corr.second = list2[indicesRow[0]];
#ifdef USE_SECOND_BAD
                  tmp_corr.secondbad = list2[indicesRow[j]];
                  tmp_corr.secondbadby2ndcl = list2[indicesRow[1]];
                  tmp_corr.d2by2ndcl = distsRow[1];
#endif
                  tmp_corr.d1 = distsRow[0];
                  tmp_corr.d2 = distsRow[j];
                  tmp_corr.ratio = sqrt(ratio);
                  corresp.TCList.push_back(tmp_corr);
                  matches++;
                  break;
                };
              double dist1 = distanceSq(list2[indicesRow[0]].reproj_kp,list2[indicesRow[j]].reproj_kp);
              if (dist1 > contrDistSq) break; //first contradictive
            }
        }
    }
  return matches;
}
int MatchFlannFGINNPlusDB(const AffineRegionList &list1, const AffineRegionList &list2, TentativeCorrespListExt &corresp,const MatchPars &par, cv::Mat *DB, const int nn)
{
  double sqminratio = par.currMatchRatio* par.currMatchRatio;
  double contrDistSq = par.contradDist *par.contradDist;
  unsigned int i,j;
  int matches = 0;
  if (list1.size() == 0) return 0;
  if (list2.size() == 0) return 0;

  unsigned int desc_size = list1[0].desc.vec.size();

  corresp.TCList.reserve((int)(list1.size()/10));

  cv::Mat keys1,keys2;
  keys1 = cv::Mat(list1.size(), desc_size, CV_32F);
  keys2 = cv::Mat(list2.size(), desc_size, CV_32F);
  for (i=0; i <list1.size(); i++)
    {
      float* Row = keys1.ptr<float>(i);
      for (j=0; j < desc_size; j++)
        Row[j] = list1[i].desc.vec[j];
    }

  for (i=0; i <list2.size(); i++)
    {
      float* Row = keys2.ptr<float>(i);
      for (j=0; j < desc_size; j++)
        Row[j] = list2[i].desc.vec[j];
    }

  cv::flann::Index tree = GenFLANNIndex(keys2,par.vector_matcher,par.vector_dist,par.kd_trees);

  cv::Mat indices;//(numQueries, k, CV_32S);
  cv::Mat dists;//(numQueries, k, CV_32F);
  cv::Mat indicesDB;//(numQueries, k, CV_32S);
  cv::Mat distsDB;//(numQueries, k, CV_32F);

  cv::flann::SearchParams SearchParams1(par.knn_checks);
  tree.knnSearch(keys1, indices, dists, nn, SearchParams1);

  cv::flann::Index DBIndex = GenFLANNIndex(*DB,par.vector_matcher,par.vector_dist,par.kd_trees);
  DBIndex.knnSearch(keys1, indicesDB, distsDB, 1, SearchParams1);

  if (sqminratio >= 1.0) //to get all points (for example, for calculating PDF)
    {
      for (i=0; i< list1.size(); i++)
        {
          int* indicesRow=indices.ptr<int>(i);
          float* distsRow=dists.ptr<float>(i);
          float* distsRowDB=distsDB.ptr<float>(i);
          for (int j=1; j<nn; j++)
            {
              double ratio = distsRow[0]/distsRow[j];
              double dist1 = distanceSq(list2[indicesRow[0]].reproj_kp,list2[indicesRow[j]].reproj_kp);
              if ((j == nn-1) || (dist1 > contrDistSq) /*|| (ratio <= sqminratio) */)
                {
                  TentativeCorrespExt tmp_corr;
                  tmp_corr.first = list1[i];
                  tmp_corr.second = list2[indicesRow[0]];
#ifdef USE_SECOND_BAD
                  tmp_corr.secondbad = list2[indicesRow[j]];
                  tmp_corr.secondbadby2ndcl = list2[indicesRow[1]];
                  tmp_corr.d2by2ndcl = distsRow[1];
#endif
                  tmp_corr.d1 = distsRow[0];
                  tmp_corr.d2 = distsRow[j];
                  tmp_corr.d2byDB = distsRowDB[0];
                  tmp_corr.ratio = sqrt(ratio);
                  corresp.TCList.push_back(tmp_corr);
                  matches++;
                  break;
                };
            }
        }
    }
  else
    {
      for (i=0; i< list1.size(); i++)
        {
          int* indicesRow=indices.ptr<int>(i);
          float* distsRow=dists.ptr<float>(i);
          float* distsRowDB=distsDB.ptr<float>(i);
          double ratioDB = distsRow[0]/distsRowDB[0];
          for (int j=1; j<nn; j++)
            {
              double ratio = distsRow[0]/distsRow[j];
              ratio = max(ratio,ratioDB);
              if (ratio <= sqminratio)
                {
                  TentativeCorrespExt tmp_corr;
                  tmp_corr.first = list1[i];
                  tmp_corr.second = list2[indicesRow[0]];
#ifdef USE_SECOND_BAD
                  tmp_corr.secondbad = list2[indicesRow[j]];
                  tmp_corr.secondbadby2ndcl = list2[indicesRow[1]];
                  tmp_corr.d2by2ndcl = distsRow[1];
#endif
                  tmp_corr.d1 = distsRow[0];
                  tmp_corr.d2 = distsRow[j];
                  tmp_corr.d2byDB = distsRowDB[0];
                  tmp_corr.ratio = sqrt(ratio);
                  corresp.TCList.push_back(tmp_corr);
                  matches++;
                  break;
                };
              double dist1 = distanceSq(list2[indicesRow[0]].reproj_kp,list2[indicesRow[j]].reproj_kp);
              if (dist1 > contrDistSq) break; //first contradictive
            }
        }
    }
  return matches;


  //      for (i=0; i<list1.size(); i++)
  //        for (int j=1; j<nn; j++)
  //          {
  //            double ratio = dists[i][0]/dists[i][j];
  //            double ratio_db = dists[i][0]/dists_db[i][0];
  //            if ((ratio <= sqminratio ) && (ratio_db <= sqminratio))// || (dists[i][0] <= (float)par.matchDistanceThreshold))
  //              {
  //                if (par.MatchTheSameTypeOnly  && (list1[i].det_kp.type != list2[indices[i][0]].det_kp.type ))
  //                  break;
  //                TentativeCorrespExt tmp_corr;
  //                tmp_corr.first = list1[i];
  //                tmp_corr.second = list2[indices[i][0]];
  //#ifdef USE_SECOND_BAD
  //                tmp_corr.secondbad = list2[indices[i][j]];
  ////                tmp_corr.secondbadby2ndcl = list2[indices[i][1]];
  //                tmp_corr.secondbadby2ndcl = keypoints_db[indices_db[i][0]];
  //                tmp_corr.d2by2ndcl = dists_db[i][0];
  //#endif
  //                tmp_corr.d1 = dists[i][0];
  //                tmp_corr.d2 = dists[i][j];
  //                tmp_corr.ratio = sqrt(ratio);
  //                corresp.TCList.push_back(tmp_corr);
  //                matches++;
  //                break;
  //              };
  //            double dist1 = distanceSq(list2[indices[i][0]].reproj_kp,list2[indices[i][j]].reproj_kp);

  //            if (dist1 > contrDistSq)
  //              break; //first contradictive
  //          }
}

int MatchFLANNDistance(const AffineRegionList &list1, const AffineRegionList &list2, TentativeCorrespListExt &corresp,const MatchPars &par, const int nn)
{

  int max_distance = (int)float(par.matchDistanceThreshold);

  unsigned int i,j;
  int matches = 0;
  if (list1.size() == 0) return 0;
  if (list2.size() == 0) return 0;

  unsigned int desc_size = list1[0].desc.vec.size();

  corresp.TCList.clear();
  corresp.TCList.reserve((int)(list1.size()/10));

  cv::Mat keys1,keys2;
  keys1 = cv::Mat(list1.size(), desc_size, CV_8U);
  keys2 = cv::Mat(list2.size(), desc_size, CV_8U);

  for (i=0; i <list1.size(); i++)
    {
      unsigned char* Row = keys1.ptr<unsigned char>(i);
      for (j=0; j < desc_size; j++)
        Row[j] = floor(list1[i].desc.vec[j]);
    }

  for (i=0; i <list2.size(); i++)
    {
      unsigned char* Row = keys2.ptr<unsigned char>(i);
      for (j=0; j < desc_size; j++)
        Row[j] = floor(list2[i].desc.vec[j]);
    }
  cv::flann::SearchParams SearchParams1(par.knn_checks);
  cv::flann::Index tree = GenFLANNIndex(keys2,par.binary_matcher,par.binary_dist,par.kd_trees);

  //  cv::flann::Index tree(keys2,setFlannIndexParams(par.binary_matcher,par.kd_trees),par.binary_dist);
  cv::Mat indices, dists;

  tree.knnSearch(keys1, indices, dists, 2, SearchParams1);

  for (i=0; i<list1.size(); i++)
    {
      int* indicesRow=indices.ptr<int>(i);
      int* distsRow=dists.ptr<int>(i);
      if (distsRow[0] <= max_distance)
        {
          TentativeCorrespExt tmp_corr;
          tmp_corr.first = list1[i];
          tmp_corr.second = list2[indicesRow[0]];
          tmp_corr.d1 = distsRow[0];
          tmp_corr.d2 = distsRow[1];
          tmp_corr.ratio = (double)tmp_corr.d1 / (double)tmp_corr.d2;
          corresp.TCList.push_back(tmp_corr);
          matches++;
        }
    }

  tree.release();
  return matches;
}


int DensificationByHomography(const AffineRegionList &list1, const AffineRegionList &list2,double *H,
                              TentativeCorrespListExt &in_corresp,TentativeCorrespListExt &out_corresp,const MatchPars &par, const double max_error,const int matchOriented)
{
  double Ht[9];
  Ht[0]=H[0];
  Ht[1]=H[3];
  Ht[2]=H[6];
  Ht[3]=H[1];
  Ht[4]=H[4];
  Ht[5]=H[7];
  Ht[6]=H[2];
  Ht[7]=H[5];
  Ht[8]=H[8];
  MatchRegionsByOverlapFastFLANN(list1,list2, Ht, out_corresp,max_error,matchOriented);
  for (unsigned int i=0; i< in_corresp.TCList.size();i++)
    out_corresp.TCList.push_back(in_corresp.TCList[i]);

  return out_corresp.TCList.size();
}


int MatchRegionsByOverlapFastFLANN(const AffineRegionList &list1, const AffineRegionList &list2, double *H,
                                   TentativeCorrespListExt &corresp,const double max_error, const int matchOriented)
{
  //  unsigned int i;
  //  double first_overlap_error, second_overlap_error;
  //  corresp.TCList.clear();
  //  corresp.TCList.reserve((int)(list1.size()/5));
  //  Keypoint4OverlapMatchList keys1(list1.size());
  //  double Ht[9];
  //  Ht[0]=H[0];
  //  Ht[1]=H[3];
  //  Ht[2]=H[6];
  //  Ht[3]=H[1];
  //  Ht[4]=H[4];
  //  Ht[5]=H[7];
  //  Ht[6]=H[2];
  //  Ht[7]=H[5];
  //  Ht[8]=H[8];
  //  cv::Mat h1cv(3,3,CV_64F,Ht);
  //  cv::Mat h1inv(3,3,CV_64F);
  //  cv::invert(h1cv,h1inv,cv::DECOMP_LU);
  //  double* HinvPtr = (double*)h1inv.data;

  //  for (i=0; i < list1.size(); i++)
  //    {

  //      double A1Ptr[4]= {list1[i].reproj_kp.a11, list1[i].reproj_kp.a12,
  //                        list1[i].reproj_kp.a21, list1[i].reproj_kp.a22
  //                       };
  //      cv::Mat A1(2,2,CV_64F, A1Ptr);
  //      cv::Mat Ainv(2,2,CV_64F);
  //      cv::invert(k_sigma*list1[i].reproj_kp.s*A1,Ainv, cv::DECOMP_LU);
  //      double* AinvPtr = (double*)Ainv.data;
  //      keys1[i].x = (AinvPtr[0]*(double)list1[i].reproj_kp.x+AinvPtr[1]*(double)list1[i].reproj_kp.y);
  //      keys1[i].y = (AinvPtr[2]*(double)list1[i].reproj_kp.x+AinvPtr[3]*(double)list1[i].reproj_kp.y);

  //      keys1[i].a11 = AinvPtr[0];
  //      keys1[i].a12 = AinvPtr[1];
  //      keys1[i].a21 = AinvPtr[2];
  //      keys1[i].a22 = AinvPtr[3];
  //      keys1[i].s = list1[i].reproj_kp.s;
  //    }
  //  Keypoint4OverlapMatchList keys2(list2.size());
  //  for (i=0; i < list2.size(); i++)
  //    {
  //      double den = (HinvPtr[6]*(double)list2[i].reproj_kp.x+HinvPtr[7]*(double)list2[i].reproj_kp.y + HinvPtr[8]);
  //      keys2[i].x = (HinvPtr[0]*(double)list2[i].reproj_kp.x+HinvPtr[1]*(double)list2[i].reproj_kp.y + HinvPtr[2])/den;
  //      keys2[i].y = (HinvPtr[3]*(double)list2[i].reproj_kp.x+HinvPtr[4]*(double)list2[i].reproj_kp.y + HinvPtr[5])/den;
  //      double Hlin[4];
  //      linH(list2[i].reproj_kp.x, list2[i].reproj_kp.y, HinvPtr, Hlin);


  //      double B[4]= {list2[i].reproj_kp.a11, list2[i].reproj_kp.a12,
  //                    list2[i].reproj_kp.a21, list2[i].reproj_kp.a22
  //                   };


  //      keys2[i].a11 = k_sigma*list2[i].reproj_kp.s*(Hlin[0]*B[0]+Hlin[1]*B[2]);
  //      keys2[i].a12 = k_sigma*list2[i].reproj_kp.s*(Hlin[0]*B[1]+Hlin[1]*B[3]);
  //      keys2[i].a21 = k_sigma*list2[i].reproj_kp.s*(Hlin[2]*B[0]+Hlin[3]*B[2]);
  //      keys2[i].a22 = k_sigma*list2[i].reproj_kp.s*(Hlin[2]*B[1]+Hlin[3]*B[3]);

  //      keys2[i].s = list2[i].reproj_kp.s;
  //    }

  //  int nn = 30;

  //  flann::Matrix<float> keys2FLANN;
  //  keys2FLANN = flann::Matrix<float>(new float[list2.size()*2], list2.size(), 2);
  //  for (i=0; i <list2.size(); i++)
  //    {
  //      keys2FLANN[i][0] = (float)keys2[i].x;
  //      keys2FLANN[i][1] = (float)keys2[i].y;
  //    }
  //  flann::Matrix<float> queryFLANN;
  //  queryFLANN = flann::Matrix<float>(new float[list1.size()*2], list1.size(), 2);
  //  for (i=0; i <list1.size(); i++)
  //    {
  //      queryFLANN[i][0] = (float)list1[i].reproj_kp.x;
  //      queryFLANN[i][1] = (float)list1[i].reproj_kp.y;
  //    }

  //  flann::Index<flann::L2<float> > index(keys2FLANN, flann::KDTreeIndexParams(4));
  //  index.buildIndex();
  //  flann::Matrix<int> indices(new int[queryFLANN.rows*nn], queryFLANN.rows, nn);
  //  flann::Matrix<float> dists(new float[queryFLANN.rows*nn], queryFLANN.rows, nn);

  //  flann::SearchParams SearchParams(128);
  //  SearchParams.cores = 0;
  //  // do a knn search, using 128 checks
  //  index.knnSearch(queryFLANN, indices, dists, nn, SearchParams);
  //  for (i=0; i< keys1.size(); i++)
  //    {
  //      int match_numb=-2;
  //      first_overlap_error = 1000.0f;
  //      second_overlap_error = 1000.0f;
  //      std::vector<int> ind(nn);
  //      for (int k=0; k<nn; k++)
  //        ind[k] = indices[i][k];
  //      BFOverlapMatchPrepFLANN(keys1[i], keys2, ind, match_numb, first_overlap_error, second_overlap_error,max_error,matchOriented);
  //      if (first_overlap_error < max_error)
  //        {
  //          TentativeCorrespExt tmp_corr;
  //          tmp_corr.first = list1[i];
  //          tmp_corr.second = list2[match_numb];
  //          corresp.TCList.push_back(tmp_corr);
  //        }
  //    }
  //  delete[] keys2FLANN.ptr();
  //  delete[] queryFLANN.ptr();
  //  delete[] dists.ptr();
  //  delete[] indices.ptr();

  //  return corresp.TCList.size();
}

int LORANSACFiltering(TentativeCorrespListExt &in_corresp, TentativeCorrespListExt &ransac_corresp,double *H, const RANSACPars pars)
{
  int do_lo = pars.localOptimization;
  unsigned int i;
  unsigned int tent_size = in_corresp.TCList.size();
  int  true_size = 0;
  ransac_corresp.TCList.clear();
  int max_samples = pars.max_samples;
  if (tent_size <=20) max_samples = 1000;
  int oriented_constr = 1;
  HDsPtr HDS1;
  HDsiPtr HDSi1;
  HDsidxPtr HDSidx1;
  FDsPtr FDS1;
  exFDsPtr EXFDS1;
  switch (pars.errorType)
    {
    case SAMPSON:
      {
        HDS1 = &HDs;
        HDSi1 = &HDsi;
        HDSidx1 = &HDsidx;
        FDS1 = &FDs;
        EXFDS1 = &exFDs;
        break;
      }
    case SYMM_MAX:
      {
        HDS1 = &HDsSymMax;
        HDSi1 = &HDsiSymMax;
        HDSidx1 = &HDsSymidxMax;
        FDS1 = &FDsSym;
        EXFDS1 = &exFDsSym;
        break;
      }
    default: //case SYMM_SUM:
      {
        HDS1 = &HDsSym;
        HDSi1 = &HDsiSym;
        HDSidx1 = &HDsSymidx;
        FDS1 = &FDsSym;
        EXFDS1 = &exFDsSym;
        break;
      }
    }
  if (tent_size >= MIN_POINTS)
    {
      double Hloran[3*3];
      double *u2Ptr = new double[tent_size*6], *u2;
      u2=u2Ptr;
      typedef unsigned char uchar;
      unsigned char *inl2 = new uchar[tent_size];
      std::vector<TentativeCorrespExt>::iterator ptr1 = in_corresp.TCList.begin();
      for(i=0; i < tent_size; i++, ptr1++)
        {
          *u2Ptr =  ptr1->first.reproj_kp.x;
          u2Ptr++;

          *u2Ptr =  ptr1->first.reproj_kp.y;
          u2Ptr++;
          *u2Ptr =  1.;
          u2Ptr++;

          *u2Ptr =  ptr1->second.reproj_kp.x;
          u2Ptr++;

          *u2Ptr =  ptr1->second.reproj_kp.y;
          u2Ptr++;
          *u2Ptr =  1.;
          u2Ptr++;
        };
      if (pars.useF)
        {
          int* data_out = (int *) malloc(tent_size * 18 * sizeof(int));
          double *resids;
          int I_H = 0;
          int *Ihptr = &I_H;
          double HinF [3*3];
          exp_ransacFcustom(u2,tent_size, pars.err_threshold*pars.err_threshold,pars.confidence,pars.max_samples,Hloran,inl2,data_out,do_lo,0,&resids, HinF,Ihptr,EXFDS1,FDS1, pars.doSymmCheck);
          free(resids);
          free(data_out);
          // if (VERB) std::cout << "Inliers in homography inside = " << I_H << std::endl;
        }
      else {
          int* data_out = (int *) malloc(tent_size * 18 * sizeof(int));
          double *resids;
          exp_ransacHcustom(u2, tent_size, pars.err_threshold*pars.err_threshold, pars.confidence, max_samples, Hloran, inl2,4, data_out,oriented_constr ,0,&resids,HDS1,HDSi1,HDSidx1,pars.doSymmCheck);
          free(resids);
          free(data_out);
        }
      // writing ransac matchings list
      std::vector<TentativeCorrespExt>::iterator ptr2 = in_corresp.TCList.begin();
      if (!pars.justMarkOutliers)
        {
          for(i=0; i < tent_size; i++, ptr2++)
            {
              ptr2->isTrue=inl2[i];
              if (inl2[i])  {
                  true_size++;
                ransac_corresp.TCList.push_back(*ptr2);
                }
            };
        }
      else
        {
          for(i=0; i < tent_size; i++, ptr2++)
            {
              ptr2->isTrue=inl2[i];
              if (inl2[i])  {
                true_size++;
                }
              ransac_corresp.TCList.push_back(*ptr2);
            };

        }

      delete [] u2;
      delete [] inl2;

      //Empirical checks
      if (!(pars.useF)) //H
        {
          cv::Mat Hlor(3,3,CV_64F, Hloran);
          cv::Mat Hinv(3,3,CV_64F);
          cv::invert(Hlor.t(),Hinv, cv::DECOMP_LU);
          double* HinvPtr = (double*)Hinv.data;
          int HIsNotZeros = 0;
          for (i=0; i<9; i++)
            HIsNotZeros = (HIsNotZeros || (HinvPtr[i] != 0.0));
          if (!HIsNotZeros)
            {
              ransac_corresp.TCList.clear();
              return 0;
            }
          for (i=0; i<9; i++)
            {
              ransac_corresp.H[i]=HinvPtr[i];
              H[i] = HinvPtr[i];
            }
          ///
          TentativeCorrespListExt checked_corresp;

#ifdef DO_TRANSFER_H_CHECK
          int checked_numb=0;
          checked_numb = NaiveHCheck(ransac_corresp,ransac_corresp.H, 10.0); //if distance between point and reprojected point in both images <=10 px - additional check for degeneracy
          if (checked_numb < MIN_POINTS) {
              //     cerr << "Can`t get enough good points after naive check" << std::endl
              //                   <<  checked_numb << " good points out of " << ransac_corresp.TCList.size() <<std::endl;

              ransac_corresp.TCList.clear();
            }
#endif
          H_LAF_check(ransac_corresp.TCList,Hloran,checked_corresp.TCList,3.0*pars.HLAFCoef*pars.err_threshold,&HDsSymMax);
          if (checked_corresp.TCList.size() < MIN_POINTS)
            checked_corresp.TCList.clear();

          // std::cerr << checked_corresp.TCList.size() << " out of " << ransac_corresp.TCList.size() << " left after H-LAF-check" << std::endl;
          ransac_corresp.TCList = checked_corresp.TCList;

        }
      else   //F
        {
          TentativeCorrespListExt checked_corresp;
          F_LAF_check(ransac_corresp.TCList,Hloran,checked_corresp.TCList,pars.LAFCoef*pars.err_threshold,FDS1);
          if (checked_corresp.TCList.size() < MIN_POINTS)
            checked_corresp.TCList.clear();

          std::cerr << checked_corresp.TCList.size() << " out of " << ransac_corresp.TCList.size() << " left after LAF-check" << std::endl;
          ransac_corresp.TCList = checked_corresp.TCList;
          for (i=0; i<9; i++)
            ransac_corresp.H[i]=Hloran[i];
        }
    }
  else
    {
      if (VERB)  cout << tent_size << " points is not enought points to do RANSAC" << endl;
      ransac_corresp.TCList.clear();
      return 0;
    }
  return true_size;
}
#ifdef WITH_ORSA
int ORSAFiltering(TentativeCorrespListExt &in_corresp, TentativeCorrespListExt &ransac_corresp,double *F, const RANSACPars pars, int w, int h)
{
  /// For LAF-check
  FDsPtr FDS1;
  switch (pars.errorType)
    {
    case SAMPSON:
      {
        FDS1 = &FDs;
        break;
      }
    case SYMM_MAX:
      {
        FDS1 = &FDsSym;
        break;
      }
    default: //case SYMM_SUM:
      {
        FDS1 = &FDsSym;
        break;
      }
    }


  ///
  unsigned int tent_size = in_corresp.TCList.size();
  ransac_corresp.TCList.clear();

  double F_tmp[9];
  if (tent_size >= MIN_POINTS)
    {
      //////// Use ORSA to filter out the incorrect matches.
      // store the coordinates of the matching points
      vector<Match> match_coor;
      match_coor.reserve(in_corresp.TCList.size());
      std::vector<TentativeCorrespExt>::iterator ptr1 = in_corresp.TCList.begin();
      for(int i=0; i < (int) tent_size; i++, ptr1++)
        {
          Match match1_coor;
          match1_coor.x1 = ptr1->second.reproj_kp.x;
          match1_coor.y1 = ptr1->second.reproj_kp.y;
          match1_coor.x2 = ptr1->first.reproj_kp.x;
          match1_coor.y2 = ptr1->first.reproj_kp.y;
          match_coor.push_back(match1_coor);
        }

      std::vector<float> index;

      int t_value=10000;
      int verb_value=0;
      int n_flag_value=0;
      int mode_value=2;
      int stop_value=0;
      float nfa_max = -2;
      float nfa = orsa(w, h, match_coor,index,t_value,verb_value,n_flag_value,mode_value,stop_value, F_tmp);


      // if the matching is significant, register the good matches
      if ( nfa < nfa_max )
        {
          cout << "The two images match! " << ransac_corresp.TCList.size() << " matchings are identified. log(nfa)=" << nfa << "." << endl;

          F[0] = F_tmp[0];    F[1] = F_tmp[3];    F[2] = F_tmp[6];
          F[3] = F_tmp[1];    F[4] = F_tmp[4];    F[5] = F_tmp[7];
          F[6] = F_tmp[2];    F[7] = F_tmp[5];    F[8] = F_tmp[8];
          for (int cc = 0; cc < (int) index.size(); cc++ )
            {
              ransac_corresp.TCList.push_back(in_corresp.TCList[cc]);
            }
          TentativeCorrespListExt checked_corresp;
          F_LAF_check(ransac_corresp.TCList,F,checked_corresp.TCList,pars.LAFCoef*pars.err_threshold,FDS1);
          if (checked_corresp.TCList.size() < MIN_POINTS)
            checked_corresp.TCList.clear();

          std::cerr << checked_corresp.TCList.size() << " out of " << ransac_corresp.TCList.size() << " left after LAF-check" << std::endl;
          ransac_corresp.TCList = checked_corresp.TCList;

        }
      else
        {
          cout << "The two images do not match. The matching is not significant: log(nfa)=" << nfa << "." << endl;
        }
    }
  else
    {
      if (VERB)  cout << tent_size << " points is not enought points to do ORSA" << endl;
      ransac_corresp.TCList.clear();
      return 0;
    }
  return ransac_corresp.TCList.size();
}
#endif
int HMatrixFiltering(TentativeCorrespListExt &in_corresp, TentativeCorrespListExt &true_corresp,double *H, const int isExtended, const RANSACPars pars)
{
  unsigned int tent_size = in_corresp.TCList.size();
  unsigned int i, j;
  int true_size = 0;
  true_corresp.TCList.clear();
  HDsPtr HDS1;
  switch (pars.errorType)
    {
    case SAMPSON:
      {
        HDS1 = &HDs;
        break;
      }
    case SYMM_MAX:
      {
        HDS1 = &HDsSymMax;
        break;
      }
    default: //case SYMM_SUM:
      {
        HDS1 = &HDsSym;
        break;
      }
    }
  double *u2Ptr = new double[tent_size*6], *u2;
  u2=u2Ptr;
  typedef unsigned char uchar;
  unsigned char *inl2 = new uchar[tent_size];
  std::vector<TentativeCorrespExt>::iterator ptr1 = in_corresp.TCList.begin();
  for(i=0; i < tent_size; i++, ptr1++)
    {
      *u2Ptr =  ptr1->second.reproj_kp.x;
      u2Ptr++;

      *u2Ptr =  ptr1->second.reproj_kp.y;
      u2Ptr++;
      *u2Ptr =  1.;
      u2Ptr++;

      *u2Ptr =  ptr1->first.reproj_kp.x;
      u2Ptr++;

      *u2Ptr =  ptr1->first.reproj_kp.y;
      u2Ptr++;
      *u2Ptr =  1.;
      u2Ptr++;
    };
  double *Z = (double *) malloc(tent_size * 18 * sizeof(double));
  double *d = new double[tent_size];
  int *p;
  p = (int *)malloc(tent_size * sizeof(int));
  for (unsigned int i = 0; i < tent_size; i ++) p[i] = i;

  float th = pars.err_threshold*pars.err_threshold;
  lin_hg(u2, Z, p, tent_size);
  HDS1(Z, u2, H, d, tent_size);

  for (j = 0; j < tent_size; j++)
    {
      if (d[j] <= th)
        {
          inl2[j] = 1;
          true_size++;
        }
      else
        {
          inl2[j] = 0;
        }
    }

  if (isExtended)
    {
      true_corresp.TCList = in_corresp.TCList;
      for(i=0; i < tent_size; i++)
        true_corresp.TCList[i].isTrue = inl2[i];
    }
  else
    for(i=0; i < tent_size; i++)
      if (inl2[i]) true_corresp.TCList.push_back(in_corresp.TCList[i]);

  free(p);
  free(Z);
  delete [] d;
  delete [] u2;
  delete [] inl2;
  true_corresp.H[0]=H[0];
  true_corresp.H[1]=H[3];
  true_corresp.H[2]=H[6];
  true_corresp.H[3]=H[1];
  true_corresp.H[4]=H[4];
  true_corresp.H[5]=H[7];
  true_corresp.H[6]=H[2];
  true_corresp.H[7]=H[5];
  true_corresp.H[8]=H[8];
  return true_size;
}
int NaiveHCheck(TentativeCorrespListExt &corresp,double *H,const double error)
{
  double err_sq = error*error;
  int corr_numb=0;
  int size = corresp.TCList.size();

  cv::Mat h1cv(3,3,CV_64F,H);
  cv::Mat h1inv(3,3,CV_64F);
  cv::invert(h1cv,h1inv,cv::DECOMP_LU);

  double *Hinv = (double*)h1inv.data;
  std::vector<TentativeCorrespExt>::iterator ptrOut = corresp.TCList.begin();
  for (int i=0; i<size; i++, ptrOut++)
    {
      double xa = (H[0]*ptrOut->first.reproj_kp.x+H[1]*ptrOut->first.reproj_kp.y+H[2])/(H[6]*ptrOut->first.reproj_kp.x+H[7]*ptrOut->first.reproj_kp.y+H[8]);
      double ya = (H[3]*ptrOut->first.reproj_kp.x+H[4]*ptrOut->first.reproj_kp.y+H[5])/(H[6]*ptrOut->first.reproj_kp.x+H[7]*ptrOut->first.reproj_kp.y+H[8]);
      //std::cout << "x=" << ptrOut->second.reproj_kp.x << " y=" << ptrOut->second.reproj_kp.y <<  "xa=" << xa << " ya=" << ya << std::endl;

      double d1=(ptrOut->second.reproj_kp.x-xa)*(ptrOut->second.reproj_kp.x-xa)+(ptrOut->second.reproj_kp.y-ya)*(ptrOut->second.reproj_kp.y-ya);

      xa = (Hinv[0]*ptrOut->second.reproj_kp.x+Hinv[1]*ptrOut->second.reproj_kp.y+Hinv[2])/(Hinv[6]*ptrOut->second.reproj_kp.x+Hinv[7]*ptrOut->second.reproj_kp.y+Hinv[8]);
      ya = (Hinv[3]*ptrOut->second.reproj_kp.x+Hinv[4]*ptrOut->second.reproj_kp.y+Hinv[5])/(Hinv[6]*ptrOut->second.reproj_kp.x+Hinv[7]*ptrOut->second.reproj_kp.y+Hinv[8]);
      double d2=(ptrOut->first.reproj_kp.x-xa)*(ptrOut->first.reproj_kp.x-xa)+(ptrOut->first.reproj_kp.y-ya)*(ptrOut->first.reproj_kp.y-ya);
      //std::cout << "x=" << ptrOut->first.reproj_kp.x << " y=" << ptrOut->first.reproj_kp.y <<  "xa=" << xa << " ya=" << ya << std::endl;

      //std::cout << "d1="<< sqrt(d1) << " d2=" << sqrt(d2) << std::endl;
      if ((d1 <=err_sq) && (d2<=(err_sq))) corr_numb++;
    }
  return corr_numb;
}


cv::Mat DrawRegions(const cv::Mat &in_img,
                         const AffineRegionList kps,
                         const int r1,
                         const cv::Scalar color1) {
  cv::Mat out_img;
  double k_scale = 3.0;//3 sigma
  if (in_img.channels() == 1)
    cv::cvtColor(in_img,out_img,CV_GRAY2RGB);
  else
    out_img=in_img.clone();

  double cosine_sine_table[44];
  double cosine_sine_table3d[66];
//  cosine_sine_table[21]=0;
//  cosine_sine_table[43]=0;
  for (int l=0; l<22; l++) {
      cosine_sine_table[l]=cos(l*M_PI/10);
      cosine_sine_table[22+l]=sin(l*M_PI/10);
    }
  for (int l=0; l<44; l++)
    cosine_sine_table3d[l]=cosine_sine_table[l];
  for (int l=44; l<66; l++)
    cosine_sine_table3d[l]=1.0;

  cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);
 // cv::Mat cs_table3d(3,22,CV_64F, cosine_sine_table3d);

  /// Image 1
  AffineRegionList::const_iterator ptrOut = kps.begin();
  for(unsigned int i=0; i < kps.size(); i++, ptrOut++)
    {

      double A[4]= {k_scale*ptrOut->reproj_kp.s*ptrOut->reproj_kp.a11, k_scale*ptrOut->reproj_kp.s*ptrOut->reproj_kp.a12,
                    k_scale*ptrOut->reproj_kp.s*ptrOut->reproj_kp.a21, k_scale*ptrOut->reproj_kp.s*ptrOut->reproj_kp.a22
                   };
      cv::Mat A1(2,2,CV_64F, A);
      cv::Mat X;
      cv::gemm(A1,cs_table,1,A1,0,X);
      vector<cv::Point> contour;
      for (int k=0; k<22; k++)
        contour.push_back(cv::Point(floor(X.at<double>(0,k)+ptrOut->reproj_kp.x),floor(X.at<double>(1,k)+ptrOut->reproj_kp.y)));

      const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
      int npts = cv::Mat(contour).rows;
      polylines(out_img, &pts,&npts, 1,
                false, 			// draw closed contour (i.e. joint end to start)
                color1,// colour RGB ordering (here = green)
                r1, 		        // line thickness
                CV_AA, 0);

   }
  return out_img;
}

void DrawMatches(const cv::Mat &in_img1,const cv::Mat &in_img2, cv::Mat &out_img1, cv::Mat &out_img2,const cv::Mat &H1,
                 TentativeCorrespListExt matchings,
                 const int DrawCentersOnly,
                 const int ReprojectToOneImage,
                 const int r1,
                 const int r2,
                 const int drawEpipolarLines,
                 const int useSCV,
                 const double LAFcoef,
                 const cv::Scalar color1,
                 const cv::Scalar color2)
{
  cv::Mat out_tmp1, out_tmp2;
  double k_scale = 3.0;//3 sigma
  double *H = (double*)H1.data;
  double ransac_th = 2*2.0;
  double affineFerror = LAFcoef * ransac_th;
  double Ht[9];
  Ht[0] = H[0];
  Ht[1] = H[3];
  Ht[2] = H[6];
  Ht[3] = H[1];
  Ht[4] = H[4];
  Ht[5] = H[7];
  Ht[6] = H[2];
  Ht[7] = H[5];
  Ht[8] = H[8];

  //  ///
  //  H[6] /=2.0;
  //  H[7] /=2.0;
  //  H[2] *=2.0;
  //  H[5] *=2.0;
  /////
  double e1[3],e2[3];
  std::vector< std::vector<double> > Ferrors(matchings.TCList.size());
  for (unsigned int i=0; i<Ferrors.size(); i++)
    Ferrors[i].resize(3);

  if (affineFerror > 0)
    GetEpipoles(H,e1,e2);
  int bad_count = 0;
  if (ReprojectToOneImage)
    {
      //  double *H = (double*)H1.data;
      cv::Mat h1inv(3,3,CV_64F);
      cv::invert(H1,h1inv,cv::DECOMP_LU);
      double *Hinv = (double*)h1inv.data;

      if (in_img1.channels() != 3)
        cv::cvtColor(in_img1,out_tmp1,CV_GRAY2RGB);
      else
        out_tmp1=in_img1.clone();
      if (in_img2.channels() != 3)
        cv::cvtColor(in_img2,out_tmp2,CV_GRAY2RGB);
      else
        out_tmp2=in_img2.clone();

   //   cv::Mat tmpimage1 (in_img1.rows,in_img1.cols,CV_32FC3,cv::Scalar(255, 255,255));
     // tmpimage1=cv::Scalar(255, 255,255);
      //   cv::addWeighted(out_tmp1,1.0,tmpimage1,-0.15,0.,out_tmp1); //make darker

   //   cv::Mat tmpimage2 (in_img2.rows,in_img2.cols,CV_32FC3,cv::Scalar(255, 255,255));
    //  tmpimage2=cv::Scalar(255, 255,255);
      //   cv::addWeighted(out_tmp2,1.0,tmpimage2,-0.15,0.,out_tmp2); //make darker

      std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();
      if(!DrawCentersOnly)
        {

          double cosine_sine_table[44];
          double cosine_sine_table3d[66];
          cosine_sine_table[21]=0;
          cosine_sine_table[43]=0;
          for (int l=0; l<21; l++)
            {
              cosine_sine_table[l]=cos(l*M_PI/10);
              cosine_sine_table[22+l]=sin(l*M_PI/10);
            }
          for (int l=0; l<44; l++)
            cosine_sine_table3d[l]=cosine_sine_table[l];
          for (int l=44; l<66; l++)
            cosine_sine_table3d[l]=1.0;

          cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);
          cv::Mat cs_table3d(3,22,CV_64F, cosine_sine_table3d);

          /// Image 1
          ptrOut = matchings.TCList.begin();
          for(unsigned int i=0; i < matchings.TCList.size(); i++, ptrOut++)
            {
              if (!(ptrOut->isTrue)) continue;
              double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                            k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                           };
              cv::Mat A1(2,2,CV_64F, A);
              cv::Mat X;
              cv::gemm(A1,cs_table,1,A1,0,X);
              vector<cv::Point> contour;
              for (int k=0; k<22; k++)
                contour.push_back(cv::Point(floor(X.at<double>(0,k)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,k)+ptrOut->first.reproj_kp.y)));

              const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
              int npts = cv::Mat(contour).rows;
              polylines(out_tmp1, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color1,// colour RGB ordering (here = green)
                        r1, 		        // line thickness
                        CV_AA, 0);
              double B[9]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12, ptrOut->second.reproj_kp.x,
                            k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22, ptrOut->second.reproj_kp.y,
                            0, 0, 1
                           };
              cv::Mat B1(3,3,CV_64F, B);
              cv::gemm(h1inv,B1,1,B1,0,B1);
              cv::Mat X2;
              cv::gemm(B1,cs_table3d,1,B1,0,X2);
              vector<cv::Point> contour2;
              for (int k=0; k<22; k++)
                contour2.push_back(cv::Point(floor(X2.at<double>(0,k) / X2.at<double>(2,k)),floor(X2.at<double>(1,k) / X2.at<double>(2,k))));

              const cv::Point *pts2 = (const cv::Point*) cv::Mat(contour2).data;
              int npts2 = cv::Mat(contour2).rows;
              polylines(out_tmp1, &pts2,&npts2, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color2,
                        r2, 		        // line thickness
                        CV_AA, 0);

            }
          /// Image 2
          ptrOut = matchings.TCList.begin();
          for(unsigned int i=0; i < matchings.TCList.size(); i++, ptrOut++)
            {
              if (!(ptrOut->isTrue)) continue;
              double A[4]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12,
                            k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22
                           };
              cv::Mat A1(2,2,CV_64F, A);
              cv::Mat X;
              cv::gemm(A1,cs_table,1,A1,0,X);
              vector<cv::Point> contour;
              for (int k=0; k<22; k++)
                contour.push_back(cv::Point(floor(X.at<double>(0,k)+ptrOut->second.reproj_kp.x),floor(X.at<double>(1,k)+ptrOut->second.reproj_kp.y)));

              const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
              int npts = cv::Mat(contour).rows;
              polylines(out_tmp2, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color1,// colour RGB ordering (here = green)
                        r1, 		        // line thickness
                        CV_AA, 0);
              double B[9]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12, ptrOut->first.reproj_kp.x,
                            k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22, ptrOut->first.reproj_kp.y,
                            0, 0, 1
                           };
              cv::Mat B1(3,3,CV_64F, B);
              cv::gemm(H1,B1,1,B1,0,B1);
              cv::Mat X2;
              cv::gemm(B1,cs_table3d,1,B1,0,X2);

              vector<cv::Point> contour2;
              for (int k=0; k<22; k++)
                contour2.push_back(cv::Point(floor(X2.at<double>(0,k) / X2.at<double>(2,k)),floor(X2.at<double>(1,k) / X2.at<double>(2,k))));

              const cv::Point *pts2 = (const cv::Point*) cv::Mat(contour2).data;
              int npts2 = cv::Mat(contour2).rows;
              polylines(out_tmp2, &pts2,&npts2, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color2,
                        r2, 		        // line thickness
                        CV_AA, 0);
            }
        }
      /// Draw centers
      ptrOut = matchings.TCList.begin();
      //Image1
      for(unsigned int i=0; i < matchings.TCList.size(); i++, ptrOut++)
        {
          if (!(ptrOut->isTrue)) continue;
          cv::circle(out_tmp1, cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)),r1+2,color1,-1); //draw original points
          double xa,ya;
          xa = (Hinv[0]*ptrOut->second.reproj_kp.x+Hinv[1]*ptrOut->second.reproj_kp.y+Hinv[2])/(Hinv[6]*ptrOut->second.reproj_kp.x+Hinv[7]*ptrOut->second.reproj_kp.y+Hinv[8]);
          ya = (Hinv[3]*ptrOut->second.reproj_kp.x+Hinv[4]*ptrOut->second.reproj_kp.y+Hinv[5])/(Hinv[6]*ptrOut->second.reproj_kp.x+Hinv[7]*ptrOut->second.reproj_kp.y+Hinv[8]);
          cv::circle(out_tmp1, cv::Point(int(xa),int(ya)),r2,color2,-1); //draw correpspondent point
          cv::line(out_tmp1,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)), color2);
        }
      //Image2
      ptrOut = matchings.TCList.begin();
      for(unsigned int i=0; i < matchings.TCList.size(); i++, ptrOut++)
        {
          if (!(ptrOut->isTrue)) continue;

          cv::circle(out_tmp2, cv::Point(int(ptrOut->second.reproj_kp.x),int(ptrOut->second.reproj_kp.y)),r1+2,color1,-1); //draw original points
          double xa,ya;
          xa = (H[0]*ptrOut->first.reproj_kp.x+H[1]*ptrOut->first.reproj_kp.y+H[2])/(H[6]*ptrOut->first.reproj_kp.x+H[7]*ptrOut->first.reproj_kp.y+H[8]);
          ya = (H[3]*ptrOut->first.reproj_kp.x+H[4]*ptrOut->first.reproj_kp.y+H[5])/(H[6]*ptrOut->first.reproj_kp.x+H[7]*ptrOut->first.reproj_kp.y+H[8]);
          cv::circle(out_tmp2, cv::Point(int(xa),int(ya)),r2,color2,-1); //draw correpspondent point
          cv::line(out_tmp2,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->second.reproj_kp.x),int(ptrOut->second.reproj_kp.y)), color2);
        }
    }
  else
    {
      int n_tents = matchings.TCList.size();
      std::vector<int> good_pts(n_tents);
      for (int a=0; a<n_tents; a++)
        good_pts[a]=1; //initialization

      int w1 = in_img1.cols;
      int h1 = in_img1.rows;

      int w2 = in_img2.cols;
      int h2 = in_img2.rows;

      /* if (useSCV)
        {

          cv::Mat img1tmp = in_img1.clone();
          cv::Mat img2tmp = in_img2.clone();
          TentativeCorrespListExt scv_list;

          img1tmp.convertTo(img1tmp, CV_64F);
          img2tmp.convertTo(img2tmp, CV_64F);

          img1tmp = img1tmp.t();
          img2tmp = img2tmp.t();

          double* img1Ptr = (double*)img1tmp.data;
          double* img2Ptr = (double*)img2tmp.data;

          std::vector <std::vector<double> > AffRegs1,AffRegs2, M, DS;
          AffRegs1.resize(n_tents);
          AffRegs2.resize(n_tents);

          std::vector<double> SIFTRatio(n_tents);
          std::vector<double> lratio(n_tents);
          double k=3.0;
          for (int i=0; i<n_tents; i++)
            {
              AffRegs1[i].resize(6);
              AffRegs2[i].resize(6);
            }

          std::vector<TentativeCorrespExt>::iterator ptr =  matchings.TCList.begin();
          for (int i=0; i<n_tents; i++,ptr++)
            {
              AffRegs1[i][0]=k*ptr->first.reproj_kp.a11*ptr->first.reproj_kp.s;
              AffRegs1[i][1]=k*ptr->first.reproj_kp.a12*ptr->first.reproj_kp.s;
              AffRegs1[i][2]=ptr->first.reproj_kp.x;

              AffRegs1[i][3]=k*ptr->first.reproj_kp.a21*ptr->first.reproj_kp.s;
              AffRegs1[i][4]=k*ptr->first.reproj_kp.a22*ptr->first.reproj_kp.s;
              AffRegs1[i][5]=ptr->first.reproj_kp.y;

              AffRegs2[i][0]=k*ptr->second.reproj_kp.a11*ptr->second.reproj_kp.s;
              AffRegs2[i][1]=k*ptr->second.reproj_kp.a12*ptr->second.reproj_kp.s;
              AffRegs2[i][2]=ptr->second.reproj_kp.x;

              AffRegs2[i][3]=k*ptr->second.reproj_kp.a21*ptr->second.reproj_kp.s;
              AffRegs2[i][4]=k*ptr->second.reproj_kp.a22*ptr->second.reproj_kp.s;
              AffRegs2[i][5]=ptr->second.reproj_kp.y;

              SIFTRatio[i] = ptr->ratio;
            }

          DS.resize(76);
          for (int i=0; i<76; i++)
            DS[i].resize(4);

          ifstream svm_step_file("DS_svm.dat");
          if (svm_step_file.is_open())
            {
              for (int i=0; i<4; i++)
                for (int j=0; j<76; j++)
                  svm_step_file >> DS[j][i];
            }
          else
            {
              cerr << "Cannot SVM-parameters file DS_svm.dat" << endl;
            }
          svm_step_file.close();


          M.resize(76);
          for (int i=0; i<76; i++)
            M[i].resize(8);

          ifstream svm_model_file("M_svm.dat");
          if (svm_model_file.is_open())
            {
              for (int i=0; i<8; i++)
                for (int j=0; j<76; j++)
                  svm_model_file >> M[j][i];

            }
          else
            {
              cerr << "Cannot read SVM-parameters file M_svm.dat" << endl;
            }
          svm_model_file.close();
          int left=0;
          left = scv(img1Ptr,w1,h1,img2Ptr,w2,h2,AffRegs1,AffRegs2,lratio,good_pts,M,DS,SIFTRatio,2,0.5,0.001,-5.0,5.0,-5.0,5.0,0,2);
          std::cout << left << " correspondences left after SCV " << endl;

          for (int l=0; l<n_tents; l++)
            if (good_pts[l])
              scv_list.TCList.push_back(matchings.TCList[l]);
        }
*/
      if (0 /*affineFerror > 0*/)
        {

          std::vector<TentativeCorrespExt>::iterator ptr =  matchings.TCList.begin();
          for (int l=0; l<n_tents; l++,ptr++)
            {
              double u[18],err[3];
              u[0] = ptr->first.reproj_kp.x;
              u[1] = ptr->first.reproj_kp.y;
              u[2] = 1.0;

              u[3] = ptr->second.reproj_kp.x;
              u[4] = ptr->second.reproj_kp.y;
              u[5] = 1.0;

              u[6] = u[0]+k_scale*ptr->first.reproj_kp.a12*ptr->first.reproj_kp.s;
              u[7] = u[1]+k_scale*ptr->first.reproj_kp.a22*ptr->first.reproj_kp.s;
              u[8] = 1.0;


              u[9]  = u[3]+k_scale*ptr->second.reproj_kp.a12*ptr->second.reproj_kp.s;
              u[10] = u[4]+k_scale*ptr->second.reproj_kp.a22*ptr->second.reproj_kp.s;
              u[11] = 1.0;

              u[12] = u[0]+k_scale*ptr->first.reproj_kp.a11*ptr->first.reproj_kp.s;
              u[13] = u[1]+k_scale*ptr->first.reproj_kp.a21*ptr->first.reproj_kp.s;
              u[14] = 1.0;

              u[15] = u[3]+k_scale*ptr->second.reproj_kp.a11*ptr->second.reproj_kp.s;
              u[16] = u[4]+k_scale*ptr->second.reproj_kp.a21*ptr->second.reproj_kp.s;
              u[17] = 1.0;
              FDsfull (u,H,err,3);
              for (int jj=0; jj<3; jj++)
                Ferrors[l][jj]=err[jj];

              double sumErr=sqrt(err[0])+sqrt(err[1])+sqrt(err[2]);

              if (sumErr>affineFerror)
                {
                  good_pts[l]=0;
                  bad_count++;
                }

              //      std::cout << sqrt(err[0]) << " " << sqrt(err[1]) << " " << sqrt(err[2]) << " " << sumErr << std::endl;

            }
        }

      unsigned int i;
      cv::Scalar color_corr = color2;
      int sep=20;
      cv::Mat roiImg1 = in_img1(cv::Rect(0,0,in_img1.cols,in_img1.rows));
      cv::Mat roiImg2 = in_img2(cv::Rect(0,0,in_img2.cols,in_img2.rows));

      out_tmp1 = cv::Mat (max(in_img1.rows,in_img2.rows),in_img1.cols+in_img2.cols+sep,in_img1.type(), cv::Scalar(255,255,255));
   //   out_tmp1 = cv::Scalar(255,255,255);

      cv::Mat roiImgResult_Left = out_tmp1(cv::Rect(0,0,in_img1.cols,in_img1.rows));
      cv::Mat roiImgResult_Right = out_tmp1(cv::Rect(in_img1.cols+sep,0,in_img2.cols,in_img2.rows));
      roiImg1.copyTo(roiImgResult_Left); //Img1 will be on the left of imgResult
      roiImg2.copyTo(roiImgResult_Right); //Img2 will be on the right of imgResult
//      if (out_tmp1.channels() < 3)
//          cv::cvtColor(out_tmp1.clone(),out_tmp1,CV_GRAY2RGB);


      out_tmp2 = cv::Mat(in_img1.rows+in_img2.rows+sep, max(in_img1.cols,in_img2.cols),in_img2.type(),cv::Scalar(255,255,255));
    //  out_tmp2 = cv::Scalar(255,255,255);

      cv::Mat roiImgResult_Up = out_tmp2(cv::Rect(0,0,in_img1.cols,in_img1.rows));
      cv::Mat roiImgResult_Down = out_tmp2(cv::Rect(0,in_img1.rows+sep, in_img2.cols,in_img2.rows));
      roiImg1.copyTo(roiImgResult_Up); //Img1 will be on the left of imgResult
      roiImg2.copyTo(roiImgResult_Down); //Img2 will be on the right of imgResult

//      if (out_img2.channels() < 3)
//        cv::cvtColor(out_tmp2.clone(),out_tmp2,CV_GRAY2RGB);

      if(!DrawCentersOnly)
        {
          double cosine_sine_table[44];
          cosine_sine_table[21]=0;
          cosine_sine_table[43]=0;

          for (int l=0; l<21; l++)
            {
              cosine_sine_table[l]=cos(l*M_PI/10);
              cosine_sine_table[22+l]=sin(l*M_PI/10);
            }
          cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);

          /// Image 1 Regions
          std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();
          for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
            {
              if (!(ptrOut->isTrue)) continue;

              if (!good_pts[i])
                {
                  color_corr = cv::Scalar(0,0,255);
                }
              else color_corr = color2;

              double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                            k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                           };
              cv::Mat A1(2,2,CV_64F, A);
              cv::Mat X;
              cv::gemm(A1,cs_table,1,A1,0,X);
              vector<cv::Point> contour;
              for (int l=0; l<22; l++)
                contour.push_back(cv::Point(floor(X.at<double>(0,l)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,l)+ptrOut->first.reproj_kp.y)));
              const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
              int npts = cv::Mat(contour).rows;

              polylines(out_tmp1, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color_corr,// colour RGB ordering (here = green)
                        r1, 		        // line thickness
                        CV_AA, 0);

              double A2[4]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12,
                             k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22
                            };
              A1 = cv::Mat (2,2,CV_64F, A2);
              cv::gemm(A1,cs_table,1,A1,0,X);
              contour.clear();
              for (int l=0; l<22; l++)
                contour.push_back(cv::Point(floor(X.at<double>(0,l)+ptrOut->second.reproj_kp.x+in_img1.cols+sep),floor(X.at<double>(1,l)+ptrOut->second.reproj_kp.y)));

              pts = (const cv::Point*) cv::Mat(contour).data;

              npts = cv::Mat(contour).rows;
              polylines(out_tmp1, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color_corr,
                        r2, 		        // line thickness
                        CV_AA, 0);


            }
          /// Image 2
          ptrOut = matchings.TCList.begin();
          for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
            {
              if (!(ptrOut->isTrue)) continue;

              if (!good_pts[i]) color_corr = cv::Scalar(0,0,255);
              else color_corr = color2;

              double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                            k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                           };
              cv::Mat A1(2,2,CV_64F, A);
              cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);
              cv::Mat X;
              cv::gemm(A1,cs_table,1,A1,0,X);
              vector<cv::Point> contour;
              for (int l=0; l<22; l++)
                contour.push_back(cv::Point(floor(X.at<double>(0,l)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,l)+ptrOut->first.reproj_kp.y)));
              const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
              int npts = cv::Mat(contour).rows;
              polylines(out_tmp2, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color_corr,// colour RGB ordering (here = green)
                        r1, 		        // line thickness
                        CV_AA, 0);
              double A2[4]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12,
                             k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22
                            };
              A1 = cv::Mat (2,2,CV_64F, A2);
              cs_table =  cv::Mat (2,22,CV_64F, cosine_sine_table);
              cv::gemm(A1,cs_table,1,A1,0,X);
              contour.clear();
              for (int l=0; l<22; l++)
                contour.push_back(cv::Point(floor(X.at<double>(0,l)+ptrOut->second.reproj_kp.x),floor(X.at<double>(1,l)+ptrOut->second.reproj_kp.y+in_img1.rows+sep)));

              pts = (const cv::Point*) cv::Mat(contour).data;
              npts = cv::Mat(contour).rows;
              polylines(out_tmp2, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color_corr,
                        r2, 		        // line thickness
                        CV_AA, 0);

            }
        }

      std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();

      for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
        {
          if (!(ptrOut->isTrue)) continue;

          double xa,ya;
          xa = in_img1.cols+sep + ptrOut->second.reproj_kp.x;
          ya = ptrOut->second.reproj_kp.y;

          cv::circle(out_tmp1, cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)),r1+2,color1,-1); //draw original points
          cv::circle(out_tmp1, cv::Point(int(xa),int(ya)),r1+2,color1,-1); //draw correpspondent point
          if (good_pts[i]) color_corr = color2;
          else color_corr = cv::Scalar(0,0,255); //red color for non-scv matches
          if (drawEpipolarLines)

            {
              double l[3], l2[3], k,b,k2,b2;
              double pt[3], pt2[3];

              pt[0] = ptrOut->first.reproj_kp.x;
              pt[1] = ptrOut->first.reproj_kp.y;
              pt[2] = 1.0;

              pt2[0] = ptrOut->second.reproj_kp.x;
              pt2[1] = ptrOut->second.reproj_kp.y;
              pt2[2] = 1.0;

              GetEpipolarLineF(H,pt2,l,k,b);
              GetEpipolarLineF(Ht,pt,l2,k2,b2);

              cv::Point sp,ep;
              cv::Scalar EpLineColor = cv::Scalar(255,255,0);
              cv::Rect img1rect1 = cv::Rect(0, 0, w1, h1);
              cv::Rect img1rect2 = cv::Rect(w1+sep, 0, w2, h2);

              cv::Rect img2rect1 = cv::Rect(0, 0, w1, h1);
              cv::Rect img2rect2 = cv::Rect(0, h1+sep, w2, h2);


              sp = cv::Point(0,int(b));
              ep = cv::Point(w1,int(k*w1+b));
              cv::clipLine(img1rect1,sp,ep);
              cv::line(out_tmp1,sp,ep,EpLineColor);

              sp = cv::Point(w1+sep,int(b2));
              ep = cv::Point(w2+w1+sep,int(k2*w2+b2));
              cv::clipLine(img1rect2,sp,ep);
              cv::line(out_tmp1,sp,ep,EpLineColor);

              sp = cv::Point(0,int(b));
              ep = cv::Point(w1,int(k*w1+b));
              cv::clipLine(img2rect1,sp,ep);
              cv::line(out_tmp2,sp,ep,EpLineColor);

              sp = cv::Point(0,int(b2)+h1+sep);
              ep = cv::Point(w2,int(k2*w2+b2)+h1+sep);
              cv::clipLine(img2rect2,sp,ep);
              cv::line(out_tmp2,sp,ep,EpLineColor);

            }

          cv::line(out_tmp1,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)), color_corr);


          xa = ptrOut->second.reproj_kp.x;
          ya = in_img1.rows+sep +ptrOut->second.reproj_kp.y;
          cv::circle(out_tmp2, cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)),r1+2,color1,-1); //draw original points
          cv::circle(out_tmp2, cv::Point(int(xa),int(ya)),r1+2,color1,-1); //draw correpspondent point
          cv::line(out_tmp2,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)), color_corr);
        }
    }
  out_img1 = out_tmp1.clone();
  out_img2 = out_tmp2.clone();
}

void DrawMatchesWithError(const cv::Mat &in_img1,const cv::Mat &in_img2, cv::Mat &out_img1, cv::Mat &out_img2,const cv::Mat &H1,
                          std::vector<double> Errors, double max_err,
                          TentativeCorrespListExt matchings,
                          const int DrawCentersOnly,
                          const int ReprojectToOneImage,
                          const int r1,
                          const int r2,
                          const int drawEpipolarLines,
                          const int useSCV,
                          const double LAFcoef,
                          const cv::Scalar color1,
                          const cv::Scalar color2,
                          const cv::Scalar color_err)
{
  cv::Mat out_tmp1, out_tmp2;
  double k_scale = 3.0;//3 sigma
  double *H = (double*)H1.data;
  double ransac_th = 2*2.0;
  double affineFerror = LAFcoef * ransac_th;
  double Ht[9];
  Ht[0] = H[0];
  Ht[1] = H[3];
  Ht[2] = H[6];
  Ht[3] = H[1];
  Ht[4] = H[4];
  Ht[5] = H[7];
  Ht[6] = H[2];
  Ht[7] = H[5];
  Ht[8] = H[8];

  double e1[3],e2[3];
  std::vector< std::vector<double> > Ferrors(matchings.TCList.size());
  for (unsigned int i=0; i<Ferrors.size(); i++)
    Ferrors[i].resize(3);

  if (affineFerror > 0) GetEpipoles(H,e1,e2);
  int bad_count = 0;
  if (ReprojectToOneImage)
    {
      //  double *H = (double*)H1.data;
      cv::Mat h1inv(3,3,CV_64F);
      cv::invert(H1,h1inv,cv::DECOMP_LU);
      double *Hinv = (double*)h1inv.data;

      if (in_img1.channels() < 3)
        cv::cvtColor(in_img1,out_tmp1,CV_GRAY2RGB);
      else
        out_img1=in_img1.clone();
      if (in_img2.channels() < 3)
        cv::cvtColor(in_img2,out_tmp2,CV_GRAY2RGB);
      else
        out_img2=in_img2.clone();

      cv::Mat tmpimage1 (in_img1.rows,in_img1.cols,CV_32FC3,cv::Scalar(255, 255,255));
    //  tmpimage1=cv::Scalar(255, 255,255);
      cv::addWeighted(out_tmp1,1.0,tmpimage1,-0.15,0.,out_tmp1); //make darker

      cv::Mat tmpimage2 (in_img2.rows,in_img2.cols,CV_32FC3,cv::Scalar(255, 255,255));
     // tmpimage2=cv::Scalar(255, 255,255);
      cv::addWeighted(out_tmp2,1.0,tmpimage2,-0.15,0.,out_tmp2); //make darker
      cv::Scalar color_corr = color2;

      std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();
      if(!DrawCentersOnly)
        {

          double cosine_sine_table[44];
          double cosine_sine_table3d[66];
          cosine_sine_table[0]=0;
          cosine_sine_table[22]=0;
          for (int l=0; l<21; l++)
            {
              cosine_sine_table[l]=cos(l*M_PI/10);
              cosine_sine_table[22+l]=sin(l*M_PI/10);
            }
          for (int l=0; l<44; l++)
            cosine_sine_table3d[l]=cosine_sine_table[l];
          for (int l=44; l<66; l++)
            cosine_sine_table3d[l]=1.0;

          cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);
          cv::Mat cs_table3d(3,22,CV_64F, cosine_sine_table3d);

          /// Image 1
          ptrOut = matchings.TCList.begin();
          for(unsigned int i=0; i < matchings.TCList.size(); i++, ptrOut++)
            {

              double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                            k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                           };
              cv::Mat A1(2,2,CV_64F, A);
              cv::Mat X;
              cv::gemm(A1,cs_table,1,A1,0,X);
              vector<cv::Point> contour;
              for (int k=0; k<22; k++)
                contour.push_back(cv::Point(floor(X.at<double>(0,k)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,k)+ptrOut->first.reproj_kp.y)));

              const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
              int npts = cv::Mat(contour).rows;
              polylines(out_tmp1, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color1,// colour RGB ordering (here = green)
                        r1, 		        // line thickness
                        CV_AA, 0);
              double B[9]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12, ptrOut->second.reproj_kp.x,
                            k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22, ptrOut->second.reproj_kp.y,
                            0, 0, 1
                           };
              cv::Mat B1(3,3,CV_64F, B);
              cv::gemm(h1inv,B1,1,B1,0,B1);
              cv::Mat X2;
              cv::gemm(B1,cs_table3d,1,B1,0,X2);
              vector<cv::Point> contour2;
              for (int k=0; k<22; k++)
                contour2.push_back(cv::Point(floor(X2.at<double>(0,k) / X2.at<double>(2,k)),floor(X2.at<double>(1,k) / X2.at<double>(2,k))));

              const cv::Point *pts2 = (const cv::Point*) cv::Mat(contour2).data;
              int npts2 = cv::Mat(contour2).rows;
              polylines(out_tmp1, &pts2,&npts2, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color2,
                        r2, 		        // line thickness
                        CV_AA, 0);

            }
          /// Image 2
          ptrOut = matchings.TCList.begin();
          for(unsigned int i=0; i < matchings.TCList.size(); i++, ptrOut++)
            {
              double A[4]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12,
                            k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22
                           };
              cv::Mat A1(2,2,CV_64F, A);
              cv::Mat X;
              cv::gemm(A1,cs_table,1,A1,0,X);
              vector<cv::Point> contour;
              for (int k=0; k<22; k++)
                contour.push_back(cv::Point(floor(X.at<double>(0,k)+ptrOut->second.reproj_kp.x),floor(X.at<double>(1,k)+ptrOut->second.reproj_kp.y)));

              const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
              int npts = cv::Mat(contour).rows;
              polylines(out_tmp2, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color1,// colour RGB ordering (here = green)
                        r1, 		        // line thickness
                        CV_AA, 0);
              double B[9]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12, ptrOut->first.reproj_kp.x,
                            k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22, ptrOut->first.reproj_kp.y,
                            0, 0, 1
                           };
              cv::Mat B1(3,3,CV_64F, B);
              cv::gemm(H1,B1,1,B1,0,B1);
              cv::Mat X2;
              cv::gemm(B1,cs_table3d,1,B1,0,X2);

              vector<cv::Point> contour2;
              for (int k=0; k<22; k++)
                contour2.push_back(cv::Point(floor(X2.at<double>(0,k) / X2.at<double>(2,k)),floor(X2.at<double>(1,k) / X2.at<double>(2,k))));

              const cv::Point *pts2 = (const cv::Point*) cv::Mat(contour2).data;
              int npts2 = cv::Mat(contour2).rows;
              polylines(out_tmp2, &pts2,&npts2, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color2,
                        r2, 		        // line thickness
                        CV_AA, 0);
            }
        }
      /// Draw centers
      ptrOut = matchings.TCList.begin();
      //Image1
      for(unsigned int i=0; i < matchings.TCList.size(); i++, ptrOut++)
        {
          cv::circle(out_tmp1, cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)),r1+2,color1,-1); //draw original points
          double xa,ya;
          xa = (Hinv[0]*ptrOut->second.reproj_kp.x+Hinv[1]*ptrOut->second.reproj_kp.y+Hinv[2])/(Hinv[6]*ptrOut->second.reproj_kp.x+Hinv[7]*ptrOut->second.reproj_kp.y+Hinv[8]);
          ya = (Hinv[3]*ptrOut->second.reproj_kp.x+Hinv[4]*ptrOut->second.reproj_kp.y+Hinv[5])/(Hinv[6]*ptrOut->second.reproj_kp.x+Hinv[7]*ptrOut->second.reproj_kp.y+Hinv[8]);
          color_corr = color2 *((max_err-Errors[i])/max_err) + color_err*(Errors[i]/max_err);
          cv::circle(out_tmp1, cv::Point(int(xa),int(ya)),r2,color_corr,-1); //draw correpspondent point
          cv::line(out_tmp1,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)), color_corr);
        }
      //Image2
      ptrOut = matchings.TCList.begin();
      for(unsigned int i=0; i < matchings.TCList.size(); i++, ptrOut++)
        {
          cv::circle(out_tmp2, cv::Point(int(ptrOut->second.reproj_kp.x),int(ptrOut->second.reproj_kp.y)),r1+2,color1,-1); //draw original points
          double xa,ya;
          xa = (H[0]*ptrOut->first.reproj_kp.x+H[1]*ptrOut->first.reproj_kp.y+H[2])/(H[6]*ptrOut->first.reproj_kp.x+H[7]*ptrOut->first.reproj_kp.y+H[8]);
          ya = (H[3]*ptrOut->first.reproj_kp.x+H[4]*ptrOut->first.reproj_kp.y+H[5])/(H[6]*ptrOut->first.reproj_kp.x+H[7]*ptrOut->first.reproj_kp.y+H[8]);
          color_corr = color2 *((max_err-Errors[i])/max_err) + color_err*(Errors[i]/max_err);
          cv::circle(out_tmp2, cv::Point(int(xa),int(ya)),r2,color_corr,-1); //draw correpspondent point
          cv::line(out_tmp2,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->second.reproj_kp.x),int(ptrOut->second.reproj_kp.y)), color_corr);
        }
    }
  else
    {
      int n_tents = matchings.TCList.size();
      std::vector<int> good_pts(n_tents);
      for (int a=0; a<n_tents; a++)
        good_pts[a]=1; //initialization

      int w1 = in_img1.cols;
      int h1 = in_img1.rows;

      int w2 = in_img2.cols;
      int h2 = in_img2.rows;
      /*
      if (useSCV)
        {

          cv::Mat img1tmp = in_img1.clone();
          cv::Mat img2tmp = in_img2.clone();
          TentativeCorrespListExt scv_list;

          img1tmp.convertTo(img1tmp, CV_64F);
          img2tmp.convertTo(img2tmp, CV_64F);

          img1tmp = img1tmp.t();
          img2tmp = img2tmp.t();

          double* img1Ptr = (double*)img1tmp.data;
          double* img2Ptr = (double*)img2tmp.data;

          std::vector <std::vector<double> > AffRegs1,AffRegs2, M, DS;
          AffRegs1.resize(n_tents);
          AffRegs2.resize(n_tents);

          std::vector<double> SIFTRatio(n_tents);
          std::vector<double> lratio(n_tents);
          double k=3.0;
          for (int i=0; i<n_tents; i++)
            {
              AffRegs1[i].resize(6);
              AffRegs2[i].resize(6);
            }

          std::vector<TentativeCorrespExt>::iterator ptr =  matchings.TCList.begin();
          for (int i=0; i<n_tents; i++,ptr++)
            {
              AffRegs1[i][0]=k*ptr->first.reproj_kp.a11*ptr->first.reproj_kp.s;
              AffRegs1[i][1]=k*ptr->first.reproj_kp.a12*ptr->first.reproj_kp.s;
              AffRegs1[i][2]=ptr->first.reproj_kp.x;

              AffRegs1[i][3]=k*ptr->first.reproj_kp.a21*ptr->first.reproj_kp.s;
              AffRegs1[i][4]=k*ptr->first.reproj_kp.a22*ptr->first.reproj_kp.s;
              AffRegs1[i][5]=ptr->first.reproj_kp.y;

              AffRegs2[i][0]=k*ptr->second.reproj_kp.a11*ptr->second.reproj_kp.s;
              AffRegs2[i][1]=k*ptr->second.reproj_kp.a12*ptr->second.reproj_kp.s;
              AffRegs2[i][2]=ptr->second.reproj_kp.x;

              AffRegs2[i][3]=k*ptr->second.reproj_kp.a21*ptr->second.reproj_kp.s;
              AffRegs2[i][4]=k*ptr->second.reproj_kp.a22*ptr->second.reproj_kp.s;
              AffRegs2[i][5]=ptr->second.reproj_kp.y;

              SIFTRatio[i] = ptr->ratio;
            }

          DS.resize(76);
          for (int i=0; i<76; i++)
            DS[i].resize(4);

          ifstream svm_step_file("DS_svm.dat");
          if (svm_step_file.is_open())
            {
              for (int i=0; i<4; i++)
                for (int j=0; j<76; j++)
                  svm_step_file >> DS[j][i];
            }
          else
            {
              cerr << "Cannot SVM-parameters file DS_svm.dat" << endl;
            }
          svm_step_file.close();


          M.resize(76);
          for (int i=0; i<76; i++)
            M[i].resize(8);

          ifstream svm_model_file("M_svm.dat");
          if (svm_model_file.is_open())
            {
              for (int i=0; i<8; i++)
                for (int j=0; j<76; j++)
                  svm_model_file >> M[j][i];

            }
          else
            {
              cerr << "Cannot read SVM-parameters file M_svm.dat" << endl;
            }
          svm_model_file.close();
          int left=0;
          left = scv(img1Ptr,w1,h1,img2Ptr,w2,h2,AffRegs1,AffRegs2,lratio,good_pts,M,DS,SIFTRatio,2,0.5,0.001,-5.0,5.0,-5.0,5.0,0,2);
          std::cout << left << " correspondences left after SCV " << endl;

          for (int l=0; l<n_tents; l++)
            if (good_pts[l])
              scv_list.TCList.push_back(matchings.TCList[l]);
        }
*/
      if (0/*affineFerror > 0*/)
        {

          std::vector<TentativeCorrespExt>::iterator ptr =  matchings.TCList.begin();
          for (int l=0; l<n_tents; l++,ptr++)
            {
              double u[18],err[3];
              u[0] = ptr->first.reproj_kp.x;
              u[1] = ptr->first.reproj_kp.y;
              u[2] = 1.0;

              u[3] = ptr->second.reproj_kp.x;
              u[4] = ptr->second.reproj_kp.y;
              u[5] = 1.0;

              u[6] = u[0]+k_scale*ptr->first.reproj_kp.a12*ptr->first.reproj_kp.s;
              u[7] = u[1]+k_scale*ptr->first.reproj_kp.a22*ptr->first.reproj_kp.s;
              u[8] = 1.0;


              u[9]  = u[3]+k_scale*ptr->second.reproj_kp.a12*ptr->second.reproj_kp.s;
              u[10] = u[4]+k_scale*ptr->second.reproj_kp.a22*ptr->second.reproj_kp.s;
              u[11] = 1.0;

              u[12] = u[0]+k_scale*ptr->first.reproj_kp.a11*ptr->first.reproj_kp.s;
              u[13] = u[1]+k_scale*ptr->first.reproj_kp.a21*ptr->first.reproj_kp.s;
              u[14] = 1.0;

              u[15] = u[3]+k_scale*ptr->second.reproj_kp.a11*ptr->second.reproj_kp.s;
              u[16] = u[4]+k_scale*ptr->second.reproj_kp.a21*ptr->second.reproj_kp.s;
              u[17] = 1.0;
              FDsfull (u,H,err,3);
              for (int jj=0; jj<3; jj++)
                Ferrors[l][jj]=err[jj];

              double sumErr=sqrt(err[0])+sqrt(err[1])+sqrt(err[2]);

              if (sumErr>affineFerror)
                {
                  good_pts[l]=0;
                  bad_count++;
                }

              std::cout << sqrt(err[0]) << " " << sqrt(err[1]) << " " << sqrt(err[2]) << " " << sumErr << std::endl;

            }
        }

      unsigned int i;
      cv::Scalar color_corr = color2;
      int sep=20;
      cv::Mat roiImg1 = in_img1(cv::Rect(0,0,in_img1.cols,in_img1.rows));
      cv::Mat roiImg2 = in_img2(cv::Rect(0,0,in_img2.cols,in_img2.rows));

      out_tmp1 = cv::Mat (max(in_img1.rows,in_img2.rows),in_img1.cols+in_img2.cols+sep,in_img1.type());
      out_tmp1=cv::Scalar(0,0,0);

      cv::Mat roiImgResult_Left = out_tmp1(cv::Rect(0,0,in_img1.cols,in_img1.rows));
      cv::Mat roiImgResult_Right = out_tmp1(cv::Rect(in_img1.cols+sep,0,in_img2.cols,in_img2.rows));
      roiImg1.copyTo(roiImgResult_Left); //Img1 will be on the left of imgResult
      roiImg2.copyTo(roiImgResult_Right); //Img2 will be on the right of imgResult
      if (out_img1.channels() !=3)
        cv::cvtColor(out_tmp1.clone(),out_tmp1,CV_GRAY2RGB);


      out_tmp2 = cv::Mat (in_img1.rows+in_img2.rows+sep, max(in_img1.cols,in_img2.cols),in_img1.type());
      out_tmp2=cv::Scalar(0,0,0);

      cv::Mat roiImgResult_Up = out_tmp2(cv::Rect(0,0,in_img1.cols,in_img1.rows));
      cv::Mat roiImgResult_Down = out_tmp2(cv::Rect(0,in_img1.rows+sep, in_img2.cols,in_img2.rows));
      roiImg1.copyTo(roiImgResult_Up); //Img1 will be on the left of imgResult
      roiImg2.copyTo(roiImgResult_Down); //Img2 will be on the right of imgResult
      if (out_img2.channels() !=3)
        cv::cvtColor(out_tmp2.clone(),out_tmp2,CV_GRAY2RGB);

      if(!DrawCentersOnly)
        {
          double cosine_sine_table[44];
          cosine_sine_table[0]=0;
          cosine_sine_table[22]=0;
          for (int l=0; l<21; l++)
            {
              cosine_sine_table[l]=cos(l*M_PI/10);
              cosine_sine_table[22+l]=sin(l*M_PI/10);
            }
          cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);

          /// Image 1 Regions
          std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();
          for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
            {
              if (!good_pts[i])
                {
                  color_corr = cv::Scalar(0,0,255);
                }
              else color_corr = color2;

              double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                            k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                           };
              cv::Mat A1(2,2,CV_64F, A);
              cv::Mat X;
              cv::gemm(A1,cs_table,1,A1,0,X);
              vector<cv::Point> contour;
              for (int l=0; l<22; l++)
                contour.push_back(cv::Point(floor(X.at<double>(0,l)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,l)+ptrOut->first.reproj_kp.y)));
              const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
              int npts = cv::Mat(contour).rows;

              polylines(out_tmp1, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color_corr,// colour RGB ordering (here = green)
                        r1, 		        // line thickness
                        CV_AA, 0);

              double A2[4]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12,
                             k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22
                            };
              A1 = cv::Mat (2,2,CV_64F, A2);
              cv::gemm(A1,cs_table,1,A1,0,X);
              contour.clear();
              for (int l=0; l<22; l++)
                contour.push_back(cv::Point(floor(X.at<double>(0,l)+ptrOut->second.reproj_kp.x+in_img1.cols+sep),floor(X.at<double>(1,l)+ptrOut->second.reproj_kp.y)));

              pts = (const cv::Point*) cv::Mat(contour).data;

              npts = cv::Mat(contour).rows;
              polylines(out_tmp1, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color_corr,
                        r2, 		        // line thickness
                        CV_AA, 0);


            }
          /// Image 2
          ptrOut = matchings.TCList.begin();
          for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
            {
              if (!good_pts[i]) color_corr = cv::Scalar(0,0,255);
              else color_corr = color2;

              double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                            k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                           };
              cv::Mat A1(2,2,CV_64F, A);
              cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);
              cv::Mat X;
              cv::gemm(A1,cs_table,1,A1,0,X);
              vector<cv::Point> contour;
              for (int l=0; l<22; l++)
                contour.push_back(cv::Point(floor(X.at<double>(0,l)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,l)+ptrOut->first.reproj_kp.y)));
              const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
              int npts = cv::Mat(contour).rows;
              polylines(out_tmp2, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color_corr,// colour RGB ordering (here = green)
                        r1, 		        // line thickness
                        CV_AA, 0);
              double A2[4]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12,
                             k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22
                            };
              A1 = cv::Mat (2,2,CV_64F, A2);
              cs_table =  cv::Mat (2,22,CV_64F, cosine_sine_table);
              cv::gemm(A1,cs_table,1,A1,0,X);
              contour.clear();
              for (int l=0; l<22; l++)
                contour.push_back(cv::Point(floor(X.at<double>(0,l)+ptrOut->second.reproj_kp.x),floor(X.at<double>(1,l)+ptrOut->second.reproj_kp.y+in_img1.rows+sep)));

              pts = (const cv::Point*) cv::Mat(contour).data;
              npts = cv::Mat(contour).rows;
              polylines(out_tmp2, &pts,&npts, 1,
                        false, 			// draw closed contour (i.e. joint end to start)
                        color_corr,
                        r2, 		        // line thickness
                        CV_AA, 0);

            }
        }

      std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();

      for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
        {
          double xa,ya;
          xa = in_img1.cols+sep + ptrOut->second.reproj_kp.x;
          ya = ptrOut->second.reproj_kp.y;

          cv::circle(out_tmp1, cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)),r1+2,color1,-1); //draw original points
          cv::circle(out_tmp1, cv::Point(int(xa),int(ya)),r1+2,color1,-1); //draw correpspondent point
          if (max_err == 0) max_err = 10000;
          color_corr = color2 *((max_err-Errors[i])/max_err) + color_err*(Errors[i]/max_err);
          if (drawEpipolarLines)

            {
              double l[3], l2[3], k,b,k2,b2;
              double pt[3], pt2[3];

              pt[0] = ptrOut->first.reproj_kp.x;
              pt[1] = ptrOut->first.reproj_kp.y;
              pt[2] = 1.0;

              pt2[0] = ptrOut->second.reproj_kp.x;
              pt2[1] = ptrOut->second.reproj_kp.y;
              pt2[2] = 1.0;

              GetEpipolarLineF(H,pt2,l,k,b);
              GetEpipolarLineF(Ht,pt,l2,k2,b2);

              cv::Point sp,ep;
              cv::Scalar EpLineColor = cv::Scalar(255,255,0);
              cv::Rect img1rect1 = cv::Rect(0, 0, w1, h1);
              cv::Rect img1rect2 = cv::Rect(w1+sep, 0, w2, h2);

              cv::Rect img2rect1 = cv::Rect(0, 0, w1, h1);
              cv::Rect img2rect2 = cv::Rect(0, h1+sep, w2, h2);


              sp = cv::Point(0,int(b));
              ep = cv::Point(w1,int(k*w1+b));
              cv::clipLine(img1rect1,sp,ep);
              cv::line(out_tmp1,sp,ep,EpLineColor);

              sp = cv::Point(w1+sep,int(b2));
              ep = cv::Point(w2+w1+sep,int(k2*w2+b2));
              cv::clipLine(img1rect2,sp,ep);
              cv::line(out_tmp1,sp,ep,EpLineColor);

              sp = cv::Point(0,int(b));
              ep = cv::Point(w1,int(k*w1+b));
              cv::clipLine(img2rect1,sp,ep);
              cv::line(out_tmp2,sp,ep,EpLineColor);

              sp = cv::Point(0,int(b2)+h1+sep);
              ep = cv::Point(w2,int(k2*w2+b2)+h1+sep);
              cv::clipLine(img2rect2,sp,ep);
              cv::line(out_tmp2,sp,ep,EpLineColor);
            }
          cv::line(out_tmp1,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)), color_corr);
          xa = ptrOut->second.reproj_kp.x;
          ya = in_img1.rows+sep +ptrOut->second.reproj_kp.y;
          cv::circle(out_tmp2, cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)),r1+2,color1,-1); //draw original points
          cv::circle(out_tmp2, cv::Point(int(xa),int(ya)),r1+2,color1,-1); //draw correpspondent point
          cv::line(out_tmp2,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)), color_corr);
        }
    }
  out_img1 = out_tmp1.clone();
  out_img2 = out_tmp2.clone();
}

void DrawMatchingsSimple(const cv::Mat &in_img, cv::Mat &out_img,const cv::Mat &H1, std::vector<corresp> matchings, const int order, const int r1,
                         const int r2,const cv::Scalar color1,const cv::Scalar color2)
{
  unsigned int i;
  double *H = (double*)H1.data;
  cv::Mat tmpimage1;
  if (in_img.channels() !=3)
    {
      cv::cvtColor(in_img,out_img,CV_GRAY2RGB);
      cv::cvtColor(in_img,tmpimage1,CV_GRAY2RGB);

    }
  else {
      out_img = in_img.clone();
      tmpimage1 = in_img.clone();
    }
  tmpimage1=cv::Scalar(255, 255,255);
  cv::addWeighted(out_img,1.0,tmpimage1,-0.15,0.,out_img); //make darker
  std::vector<corresp>::iterator ptrOut = matchings.begin();
  if (order)
    for(i=0; i < matchings.size(); i++, ptrOut++)
      {
      //  if (!(ptrOut->isTrue)) continue;

        cv::circle(out_img, cv::Point(int(ptrOut->first.x),int(ptrOut->first.y)),r1,color1,-1); //draw original points
        double xa,ya;
        xa = (H[0]*ptrOut->second.x+H[1]*ptrOut->second.y+H[2])/(H[6]*ptrOut->second.x+H[7]*ptrOut->second.y+H[8]);
        ya = (H[3]*ptrOut->second.x+H[4]*ptrOut->second.y+H[5])/(H[6]*ptrOut->second.x+H[7]*ptrOut->second.y+H[8]);
        cv::circle(out_img, cv::Point(int(xa),int(ya)),r2,color2,-1); //draw correpspondent point
        cv::line(out_img,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->first.x),int(ptrOut->first.y)), color2);
      }
  else
    for(i=0; i < matchings.size(); i++, ptrOut++)
      {
        cv::circle(out_img, cv::Point(int(ptrOut->second.x),int(ptrOut->second.y)),r1,color1,-1); //draw original points
        double xa,ya;
        xa = (H[0]*ptrOut->first.x+H[1]*ptrOut->first.y+H[2])/(H[6]*ptrOut->first.x+H[7]*ptrOut->first.y+H[8]);
        ya = (H[3]*ptrOut->first.x+H[4]*ptrOut->first.y+H[5])/(H[6]*ptrOut->first.x+H[7]*ptrOut->first.y+H[8]);
        cv::circle(out_img, cv::Point(int(xa),int(ya)),r2,color2,-1); //draw correpspondent point
        cv::line(out_img,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->second.x),int(ptrOut->second.y)), color2);
      }
}

void DrawMatchingRegions3D(const cv::Mat &in_img1,const cv::Mat &in_img2, cv::Mat &out_img,const cv::Mat &F1, TentativeCorrespListExt matchings, const int conc_horiz, const int r1,
                           const int r2,const cv::Scalar color1,const cv::Scalar color2)
{
  double k_scale=3.0;

  unsigned int i;
  //double *F = (double*)F1.data;
  cv::Scalar color_corr;
  //double s=2.0; //"bad" coefficient
  int sep=20;
  cv::Mat roiImg1 = in_img1(cv::Rect(0,0,in_img1.cols,in_img1.rows));
  cv::Mat roiImg2 = in_img2(cv::Rect(0,0,in_img2.cols,in_img2.rows));
  cv::Mat tmpimage1;


  if (conc_horiz)
    {
      tmpimage1 = cv::Mat (max(in_img1.rows,in_img2.rows),in_img1.cols+in_img2.cols+sep,in_img1.type());
      tmpimage1=cv::Scalar(0,0,0);

      cv::Mat roiImgResult_Left = tmpimage1(cv::Rect(0,0,in_img1.cols,in_img1.rows));
      cv::Mat roiImgResult_Right = tmpimage1(cv::Rect(in_img1.cols+sep,0,in_img2.cols,in_img2.rows));

      roiImg1.copyTo(roiImgResult_Left); //Img1 will be on the left of imgResult
      roiImg2.copyTo(roiImgResult_Right); //Img2 will be on the right of imgResult
      if (tmpimage1.channels() !=3)
        {
          cv::cvtColor(tmpimage1,tmpimage1,CV_GRAY2RGB);
        }
      std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();

      double cosine_sine_table[44];

      cosine_sine_table[0]=0;
      cosine_sine_table[22]=0;

      for (int l=0; l<21; l++)
        {
          cosine_sine_table[l]=cos(l*M_PI/10);
          cosine_sine_table[22+l]=sin(l*M_PI/10);
        }
      //int count = 0;
      for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
        {
          if (!(ptrOut->isTrue)) continue;

#ifdef USE_SECOND_BAD
          if ((ptrOut->secondbad.id == ptrOut->secondbadby2ndcl.id)/* || (count > 2)*/) continue;
#endif

          double xa,ya;
          xa = in_img1.cols+sep + ptrOut->second.reproj_kp.x;
          ya = ptrOut->second.reproj_kp.y;

          double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                        k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                       };
          cv::Mat A1(2,2,CV_64F, A);
          cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);
          cv::Mat X;
          cv::gemm(A1,cs_table,1,A1,0,X);
          vector<cv::Point> contour;
          for (int i=0; i<22; i++)
            contour.push_back(cv::Point(floor(X.at<double>(0,i)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,i)+ptrOut->first.reproj_kp.y)));
          const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
          int npts = cv::Mat(contour).rows;
          polylines(tmpimage1, &pts,&npts, 1,
                    false, 			// draw closed contour (i.e. joint end to start)
                    color1,// colour RGB ordering (here = green)
                    r1, 		        // line thickness
                    CV_AA, 0);
          //1st inc
#ifdef USE_SECOND_BAD

          double A3[4]= {k_scale*ptrOut->secondbad.reproj_kp.s*ptrOut->secondbad.reproj_kp.a11, k_scale*ptrOut->secondbad.reproj_kp.s*ptrOut->secondbad.reproj_kp.a12,
                         k_scale*ptrOut->secondbad.reproj_kp.s*ptrOut->secondbad.reproj_kp.a21, k_scale*ptrOut->secondbad.reproj_kp.s*ptrOut->secondbad.reproj_kp.a22
                        };
          A1 = cv::Mat (2,2,CV_64F, A3);
          cs_table =  cv::Mat (2,22,CV_64F, cosine_sine_table);
          cv::gemm(A1,cs_table,1,A1,0,X);
          contour.clear();
          for (int i=0; i<22; i++)
            contour.push_back(cv::Point(floor(X.at<double>(0,i)+ptrOut->secondbad.reproj_kp.x+in_img1.cols+sep),floor(X.at<double>(1,i)+ptrOut->secondbad.reproj_kp.y)));

          pts = (const cv::Point*) cv::Mat(contour).data;
          npts = cv::Mat(contour).rows;
          polylines(tmpimage1, &pts,&npts, 1,
                    false, 			// draw closed contour (i.e. joint end to start)
                    cv::Scalar(0,255,255),
                    r2, 		        // line thickness
                    CV_AA, 0);
          //2nd closest
          double A4[4]= {k_scale*ptrOut->secondbadby2ndcl.reproj_kp.s*ptrOut->secondbadby2ndcl.reproj_kp.a11, k_scale*ptrOut->secondbadby2ndcl.reproj_kp.s*ptrOut->secondbadby2ndcl.reproj_kp.a12,
                         k_scale*ptrOut->secondbadby2ndcl.reproj_kp.s*ptrOut->secondbadby2ndcl.reproj_kp.a21, k_scale*ptrOut->secondbadby2ndcl.reproj_kp.s*ptrOut->secondbadby2ndcl.reproj_kp.a22
                        };
          A1 = cv::Mat (2,2,CV_64F, A4);
          cs_table =  cv::Mat (2,22,CV_64F, cosine_sine_table);
          cv::gemm(A1,cs_table,1,A1,0,X);
          contour.clear();
          for (int i=0; i<22; i++)
            contour.push_back(cv::Point(floor(X.at<double>(0,i)+ptrOut->secondbadby2ndcl.reproj_kp.x+in_img1.cols+sep),floor(X.at<double>(1,i)+ptrOut->secondbadby2ndcl.reproj_kp.y)));

          pts = (const cv::Point*) cv::Mat(contour).data;
          npts = cv::Mat(contour).rows;
          polylines(tmpimage1, &pts,&npts, 1,
                    false, 			// draw closed contour (i.e. joint end to start)
                    cv::Scalar(0,0,255),
                    r2+1, 		        // line thickness
                    CV_AA, 0);
#endif

          //Matched
          double A2[4]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12,
                         k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22
                        };
          A1 = cv::Mat (2,2,CV_64F, A2);
          cs_table =  cv::Mat (2,22,CV_64F, cosine_sine_table);
          cv::gemm(A1,cs_table,1,A1,0,X);
          contour.clear();
          for (int i=0; i<22; i++)
            contour.push_back(cv::Point(floor(X.at<double>(0,i)+ptrOut->second.reproj_kp.x+in_img1.cols+sep),floor(X.at<double>(1,i)+ptrOut->second.reproj_kp.y)));

          pts = (const cv::Point*) cv::Mat(contour).data;
          npts = cv::Mat(contour).rows;
          polylines(tmpimage1, &pts,&npts, 1,
                    false, 			// draw closed contour (i.e. joint end to start)
                    color2,
                    r2, 		        // line thickness
                    CV_AA, 0);
          cv::circle(tmpimage1, cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)),r1,color1,-1); //draw original points
          cv::circle(tmpimage1, cv::Point(int(xa),int(ya)),r2,color1,-1); //draw correpspondent point
          cv::line(tmpimage1,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)), color2);

        }
    }
  else
    {
      double cosine_sine_table[44];

      cosine_sine_table[0]=0;
      cosine_sine_table[22]=0;

      for (int l=0; l<21; l++)
        {
          cosine_sine_table[l]=cos(l*M_PI/10);
          cosine_sine_table[23+l]=sin(l*M_PI/10);
        }


      tmpimage1 = cv::Mat (in_img1.rows+in_img2.rows+sep, max(in_img1.cols,in_img2.cols),in_img1.type());
      tmpimage1=cv::Scalar(0,0,0);

      cv::Mat roiImgResult_Up = tmpimage1(cv::Rect(0,0,in_img1.cols,in_img1.rows));
      cv::Mat roiImgResult_Down = tmpimage1(cv::Rect(0,in_img1.rows+sep, in_img2.cols,in_img2.rows));
      roiImg1.copyTo(roiImgResult_Up); //Img1 will be on the left of imgResult

      roiImg2.copyTo(roiImgResult_Down); //Img2 will be on the right of imgResult

      if (tmpimage1.channels() !=3) {
          cv::cvtColor(tmpimage1,tmpimage1,CV_GRAY2RGB);
      }

      std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();
      for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
        {
          //                           if (!good_pts[i]) color_corr = cv::Scalar(0,0,255);
          //                   else color_corr = color2;

          if (!(ptrOut->isTrue)) continue;

          double xa,ya;
          xa = ptrOut->second.reproj_kp.x;
          ya = in_img1.rows+sep +ptrOut->second.reproj_kp.y;

          double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                        k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                       };
          cv::Mat A1(2,2,CV_64F, A);
          cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);
          cv::Mat X;
          cv::gemm(A1,cs_table,1,A1,0,X);
          vector<cv::Point> contour;
          for (int i=0; i<22; i++)
            contour.push_back(cv::Point(floor(X.at<double>(0,i)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,i)+ptrOut->first.reproj_kp.y)));
          const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
          int npts = cv::Mat(contour).rows;
          polylines(tmpimage1, &pts,&npts, 1,
                    false, 			// draw closed contour (i.e. joint end to start)
                    color_corr,// colour RGB ordering (here = green)
                    r1, 		        // line thickness
                    CV_AA, 0);
          double A2[4]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12,
                         k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22
                        };
          A1 = cv::Mat (2,2,CV_64F, A2);
          cs_table =  cv::Mat (2,22,CV_64F, cosine_sine_table);
          cv::gemm(A1,cs_table,1,A1,0,X);
          contour.clear();
          for (int i=0; i<22; i++)
            contour.push_back(cv::Point(floor(X.at<double>(0,i)+ptrOut->second.reproj_kp.x),floor(X.at<double>(1,i)+ptrOut->second.reproj_kp.y+in_img1.rows+sep)));

          pts = (const cv::Point*) cv::Mat(contour).data;
          npts = cv::Mat(contour).rows;
          polylines(tmpimage1, &pts,&npts, 1,
                    false, 			// draw closed contour (i.e. joint end to start)
                    color_corr,
                    r2, 		        // line thickness
                    CV_AA, 0);
          cv::circle(tmpimage1, cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)),r1,color1,-1); //draw original points
          cv::circle(tmpimage1, cv::Point(int(xa),int(ya)),r2,color1,-1); //draw correpspondent point
          cv::line(tmpimage1,cv::Point(int(xa),int(ya)),cv::Point(int(ptrOut->first.reproj_kp.x),int(ptrOut->first.reproj_kp.y)), color_corr);

        }


    }
  out_img=tmpimage1.clone();

}
void DrawMatchingRegions(const cv::Mat &in_img, cv::Mat &out_img,const cv::Mat &H1, TentativeCorrespListExt matchings, const int order, const int r1,
                         const int r2,const cv::Scalar color1,const cv::Scalar color2)
{
  double k_scale=3.0;
  unsigned int i;
  //double *H = (double*)H1.data;
  cv::Mat tmpimage1;
  if (in_img.channels() !=3) {
      cv::cvtColor(in_img,out_img,CV_GRAY2RGB);
      cv::cvtColor(in_img,tmpimage1,CV_GRAY2RGB);
    }
  else {
      out_img=in_img.clone();
      tmpimage1=in_img.clone();
    }

  tmpimage1=cv::Scalar(255, 255,255);
  cv::addWeighted(out_img,1.0,tmpimage1,-0.15,0.,out_img); //make darker
  std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();
  double cosine_sine_table[44];
  double cosine_sine_table3d[66];

  cosine_sine_table[0]=0;
  cosine_sine_table[22]=0;

  for (int l=0; l<21; l++)
    {
      cosine_sine_table[l]=cos(l*M_PI/10);
      cosine_sine_table[23+l]=sin(l*M_PI/10);
    }
  for (int l=0; l<44; l++)
    {
      cosine_sine_table3d[l]=cosine_sine_table[l];
    }
  for (int l=44; l<66; l++)
    {
      cosine_sine_table3d[l]=1.0;
    }

  if (order)
    for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
      {
        if (!(ptrOut->isTrue)) continue;

        double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                      k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                     };
        cv::Mat A1(2,2,CV_64F, A);
        cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);
        cv::Mat X;
        cv::gemm(A1,cs_table,1,A1,0,X);
        vector<cv::Point> contour;
        for (int i=0; i<22; i++)
          contour.push_back(cv::Point(floor(X.at<double>(0,i)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,i)+ptrOut->first.reproj_kp.y)));

        const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
        int npts = cv::Mat(contour).rows;
        polylines(out_img, &pts,&npts, 1,
                  false, 			// draw closed contour (i.e. joint end to start)
                  color1,// colour RGB ordering (here = green)
                  r1, 		        // line thickness
                  CV_AA, 0);
        double B[9]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12, ptrOut->second.reproj_kp.x,
                      k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22, ptrOut->second.reproj_kp.y,
                      0, 0, 1
                     };
        cv::Mat B1(3,3,CV_64F, B);
        cv::gemm(H1,B1,1,B1,0,B1);
        cv::Mat cs_table3d(3,22,CV_64F, cosine_sine_table3d);
        cv::Mat X2;
        cv::gemm(B1,cs_table3d,1,B1,0,X2);

        vector<cv::Point> contour2;
        for (int i=0; i<22; i++)
          contour2.push_back(cv::Point(floor(X2.at<double>(0,i) / X2.at<double>(2,i)),floor(X2.at<double>(1,i) / X2.at<double>(2,i))));

        const cv::Point *pts2 = (const cv::Point*) cv::Mat(contour2).data;
        int npts2 = cv::Mat(contour2).rows;
        polylines(out_img, &pts2,&npts2, 1,
                  false, 			// draw closed contour (i.e. joint end to start)
                  color2,
                  r2, 		        // line thickness
                  CV_AA, 0);

      }
  else
    {

      cosine_sine_table[0]=0;
      cosine_sine_table[22]=0;

      for (int l=0; l<21; l++)
        {
          cosine_sine_table[l]=cos(l*M_PI/10);
          cosine_sine_table[23+l]=sin(l*M_PI/10);
        }
      for (int l=0; l<44; l++)
        {
          cosine_sine_table3d[l]=cosine_sine_table[l];
        }
      for (int l=44; l<66; l++)
        {
          cosine_sine_table3d[l]=1.0;
        }

      for(i=0; i < matchings.TCList.size(); i++, ptrOut++)
        {
          if (!(ptrOut->isTrue)) continue;

          double A[4]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12,
                        k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22
                       };
          cv::Mat A1(2,2,CV_64F, A);
          cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);
          cv::Mat X;
          cv::gemm(A1,cs_table,1,A1,0,X);
          vector<cv::Point> contour;
          for (int l=0; l<22; l++)
            contour.push_back(cv::Point(floor(X.at<double>(0,l)+ptrOut->second.reproj_kp.x),floor(X.at<double>(1,l)+ptrOut->second.reproj_kp.y)));

          const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
          int npts = cv::Mat(contour).rows;
          polylines(out_img, &pts,&npts, 1,
                    false, 			// draw closed contour (i.e. joint end to start)
                    color1,// colour RGB ordering (here = green)
                    r1, 		        // line thickness
                    CV_AA, 0);
          double B[9]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12, ptrOut->first.reproj_kp.x,
                        k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22, ptrOut->first.reproj_kp.y,
                        0, 0, 1
                       };
          cv::Mat B1(3,3,CV_64F, B);
          cv::gemm(H1,B1,1,B1,0,B1);
          cv::Mat cs_table3d(3,22,CV_64F, cosine_sine_table3d);
          cv::Mat X2;
          cv::gemm(B1,cs_table3d,1,B1,0,X2);

          vector<cv::Point> contour2;
          for (int l=0; l<22; l++)
            contour2.push_back(cv::Point(floor(X2.at<double>(0,l) / X2.at<double>(2,l)),floor(X2.at<double>(1,l) / X2.at<double>(2,l))));

          const cv::Point *pts2 = (const cv::Point*) cv::Mat(contour2).data;
          int npts2 = cv::Mat(contour2).rows;
          polylines(out_img, &pts2,&npts2, 1,
                    false, 			// draw closed contour (i.e. joint end to start)
                    color2,
                    r2, 		        // line thickness
                    CV_AA, 0);

        }
    }
}


#ifdef USE_SECOND_BAD
void DrawChangedMatchingRegions(const cv::Mat &in_img, cv::Mat &out_img,const cv::Mat &H1, TentativeCorrespListExt matchings,TentativeCorrespListExt matchings2nd, const int order, const int r1,
                                const int r2,const cv::Scalar color1,const cv::Scalar color2)
{
  double k_scale=3.0;

  cv::Mat tmpimage1;
  if (in_img.channels() != 3)
    {
  cv::cvtColor(in_img,out_img,CV_GRAY2RGB);
  cv::cvtColor(in_img,tmpimage1,CV_GRAY2RGB);
    }
  else {
      out_img = in_img.clone();
      tmpimage1 = in_img.clone();
    }

  tmpimage1=cv::Scalar(255, 255,255);
  cv::addWeighted(out_img,1.0,tmpimage1,-0.15,0.,out_img); //make darker

  cv::Mat out_tmp = out_img.clone();

  std::vector<TentativeCorrespExt>::iterator ptrOut = matchings.TCList.begin();
  double cosine_sine_table[44];
  double cosine_sine_table3d[66];

  cosine_sine_table[0]=0;
  cosine_sine_table[22]=0;

  for (int l=0; l<21; l++)
    {
      cosine_sine_table[l]=cos(l*M_PI/10);
      cosine_sine_table[22+l]=sin(l*M_PI/10);
    }
  for (int l=0; l<44; l++)
    {
      cosine_sine_table3d[l]=cosine_sine_table[l];
    }
  for (int l=44; l<66; l++)
    {
      cosine_sine_table3d[l]=1.0;
    }
  cv::Mat cs_table3d(3,22,CV_64F, cosine_sine_table3d);
  cv::Mat cs_table(2,22,CV_64F, cosine_sine_table);

  int count = 0;
  for(unsigned int f=0; f < matchings.TCList.size(); f++, ptrOut++)
    if (matchings.TCList[f].secondbad.id != matchings.TCList[f].secondbadby2ndcl.id && count < 2)
      {
        if (!(ptrOut->isTrue)) continue;

        double A[4]= {k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a11, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a12,
                      k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a21, k_scale*ptrOut->first.reproj_kp.s*ptrOut->first.reproj_kp.a22
                     };
        cv::Mat A1(2,2,CV_64F, A);
        cv::Mat X;
        cv::gemm(A1,cs_table,1,A1,0,X);
        vector<cv::Point> contour;
        for (int i=0; i<22; i++)
          contour.push_back(cv::Point(floor(X.at<double>(0,i)+ptrOut->first.reproj_kp.x),floor(X.at<double>(1,i)+ptrOut->first.reproj_kp.y)));

        const cv::Point *pts = (const cv::Point*) cv::Mat(contour).data;
        int npts = cv::Mat(contour).rows;
        polylines(out_tmp, &pts,&npts, 1,
                  false, 			// draw closed contour (i.e. joint end to start)
                  color1,// colour RGB ordering (here = green)
                  r1, 		        // line thickness
                  CV_AA, 0);

        double B[9]= {k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a11, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a12, ptrOut->second.reproj_kp.x,
                      k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a21, k_scale*ptrOut->second.reproj_kp.s*ptrOut->second.reproj_kp.a22, ptrOut->second.reproj_kp.y,
                      0, 0, 1
                     };
        cv::Mat B1(3,3,CV_64F, B);
        cv::gemm(H1,B1,1,B1,0,B1);
        cv::Mat X2;
        cv::gemm(B1,cs_table3d,1,B1,0,X2);

        vector<cv::Point> contour2;
        for (int i=0; i<22; i++)
          contour2.push_back(cv::Point(floor(X2.at<double>(0,i) / X2.at<double>(2,i)),floor(X2.at<double>(1,i) / X2.at<double>(2,i))));

        const cv::Point *pts2 = (const cv::Point*) cv::Mat(contour2).data;
        int npts2 = cv::Mat(contour2).rows;
        polylines(out_tmp, &pts2,&npts2, 1,
                  false, 			// draw closed contour (i.e. joint end to start)
                  color2,
                  r2, 		        // line thickness
                  CV_AA, 0);



        //second bad
        const cv::Scalar color22 = cv::Scalar(0,255,255);

        double B2[9]= {k_scale*ptrOut->secondbad.reproj_kp.s*ptrOut->secondbad.reproj_kp.a11, k_scale*ptrOut->secondbad.reproj_kp.s*ptrOut->secondbad.reproj_kp.a12, ptrOut->secondbad.reproj_kp.x,
                       k_scale*ptrOut->secondbad.reproj_kp.s*ptrOut->secondbad.reproj_kp.a21, k_scale*ptrOut->secondbad.reproj_kp.s*ptrOut->secondbad.reproj_kp.a22, ptrOut->secondbad.reproj_kp.y,
                       0, 0, 1
                      };
        cv::Mat B12(3,3,CV_64F, B2);
        cv::gemm(H1,B12,1,B12,0,B12);
        cv::Mat X22;
        cv::gemm(B12,cs_table3d,1,B12,0,X22);

        vector<cv::Point> contour22;
        for (int i=0; i<22; i++)
          contour22.push_back(cv::Point(floor(X22.at<double>(0,i) / X22.at<double>(2,i)),floor(X22.at<double>(1,i) / X22.at<double>(2,i))));

        const cv::Point *pts22 = (const cv::Point*) cv::Mat(contour22).data;
        int npts22 = cv::Mat(contour22).rows;
        polylines(out_tmp, &pts22,&npts22, 1,
                  false, 			// draw closed contour (i.e. joint end to start)
                  color22,
                  r2+1, 		        // line thickness
                  CV_AA, 0);

        //second bad by 2nd closest
        const cv::Scalar color223 = cv::Scalar(0,0,255);
        double B23[9]= {k_scale*ptrOut->secondbadby2ndcl.reproj_kp.s*ptrOut->secondbadby2ndcl.reproj_kp.a11, k_scale*ptrOut->secondbadby2ndcl.reproj_kp.s*ptrOut->secondbadby2ndcl.reproj_kp.a12, ptrOut->secondbadby2ndcl.reproj_kp.x,
                        k_scale*ptrOut->secondbadby2ndcl.reproj_kp.s*ptrOut->secondbadby2ndcl.reproj_kp.a21, k_scale*ptrOut->secondbadby2ndcl.reproj_kp.s*ptrOut->secondbadby2ndcl.reproj_kp.a22, ptrOut->secondbadby2ndcl.reproj_kp.y,
                        0, 0, 1
                       };
        cv::Mat B123(3,3,CV_64F, B23);
        cv::gemm(H1,B123,1,B123,0,B123);
        cv::Mat X223;
        cv::gemm(B123,cs_table3d,1,B123,0,X223);

        vector<cv::Point> contour223;
        for (int i=0; i<22; i++)
          contour223.push_back(cv::Point(floor(X223.at<double>(0,i) / X223.at<double>(2,i)),floor(X223.at<double>(1,i) / X223.at<double>(2,i))));

        const cv::Point *pts223 = (const cv::Point*) cv::Mat(contour223).data;
        int npts223 = cv::Mat(contour223).rows;
        polylines(out_tmp, &pts223,&npts223, 1,
                  false, 			// draw closed contour (i.e. joint end to start)
                  color223,
                  r2, 		        // line thickness
                  CV_AA, 0);
        count++;

      }
  out_img = out_tmp.clone();

}
#endif


void WriteMatchings(TentativeCorrespListExt &match, std::ostream &out1, int writeWithRatios)
{
//  out1 << (int) match.TCList.size() << std::endl;
  std::vector<TentativeCorrespExt>::iterator ptr = match.TCList.begin();
    if (writeWithRatios)
    {
        out1 << "x1,y1,x2,y2,FGINN_ratio,SNN_ratio,detector,descriptor,is_correct " << std::endl;

      for(int i=0; i < (int) match.TCList.size(); i++, ptr++)
        out1 << ptr->first.reproj_kp.x << "," << ptr->first.reproj_kp.y << "," << ptr->second.reproj_kp.x << "," << ptr->second.reproj_kp.y << ","
             << sqrt(ptr->d1 / ptr->d2) << "," << sqrt(ptr->d1 / ptr->d2by2ndcl) << "," << DetectorNames[ptr->first.type] << "," << DescriptorNames[ptr->first.desc.type] << "," << ptr->isTrue << std::endl;
    }
  else
    {
      for(int i=0; i < (int) match.TCList.size(); i++, ptr++)
        out1 << ptr->first.reproj_kp.x << " " << ptr->first.reproj_kp.y << " " << ptr->second.reproj_kp.x << " " << ptr->second.reproj_kp.y  << std::endl;
    }
}

//void DuplicateFiltering(TentativeCorrespList &in_corresp, const double r)
//{
//  unsigned int i,j;
//  unsigned int tent_size = in_corresp.TCList.size();
//  double d1_sq, d2_sq;
//  double r_sq = r*r;
//  vector <char> flag_unique;
//  flag_unique = vector <char> (tent_size);
//  for (i=0; i<tent_size; i++)
//    flag_unique[i] = 1;

//  std::vector<TentativeCorresp>::iterator ptr1 = in_corresp.TCList.begin();
//  for(i=0; i < tent_size; i++, ptr1++)
//    {
//      if (flag_unique[i] == 0) continue;
//      std::vector<TentativeCorresp>::iterator ptr2 = ptr1+1;
//      for(j=i+1; j < tent_size; j++, ptr2++)
//        {
//          if (flag_unique[j] == 0) continue;
//          double dx = (ptr1->first.reproj_kp.x - ptr2->first.reproj_kp.x);
//          double dy = (ptr1->first.reproj_kp.y - ptr2->first.reproj_kp.y);
//          d1_sq = dx*dx+dy*dy;
//          if (d1_sq > r_sq)
//            continue;
//          dx = (ptr1->second.reproj_kp.x - ptr2->second.reproj_kp.x);
//          dy = (ptr1->second.reproj_kp.y - ptr2->second.reproj_kp.y);
//          d2_sq = dx*dx+dy*dy;
//          if (d2_sq <= r_sq)
//            flag_unique[j] = 0;
//        }
//    }
//  TentativeCorrespList unique_list;
//  unique_list.TCList.reserve(0.7*in_corresp.TCList.size());
//  for (i=0; i<9; i++)
//    unique_list.H[i] = in_corresp.H[i];

//  for (i=0; i<tent_size; i++)
//    if (flag_unique[i] == 1)
//      unique_list.TCList.push_back(in_corresp.TCList[i]);

//  in_corresp.TCList = unique_list.TCList;
//}
void DuplicateFiltering(TentativeCorrespListExt &in_corresp, const double r, const int mode)
{
  if (r <= 0) return; //no filtering
  unsigned int i,j;
  unsigned int tent_size = in_corresp.TCList.size();
  double r_sq = r*r;
  double d1_sq, d2_sq;
  vector <char> flag_unique;
  flag_unique = vector <char> (tent_size);
  for (i=0; i<tent_size; i++)
    flag_unique[i] = 1;

  switch (mode) {
    case MODE_RANDOM:
      break;
    case MODE_FGINN:
      {
        std::sort(in_corresp.TCList.begin(),in_corresp.TCList.end(),CompareCorrespondenceByRatio);
        break;
      }
    case MODE_DISTANCE:
      {
        std::sort(in_corresp.TCList.begin(),in_corresp.TCList.end(),CompareCorrespondenceByDistance);
        break;
      }
    case MODE_BIGGER_REGION:
      {
        std::sort(in_corresp.TCList.begin(),in_corresp.TCList.end(),CompareCorrespondenceByScale);
        break;
      }
    default:
      break;
    }

  std::vector<TentativeCorrespExt>::iterator ptr1 = in_corresp.TCList.begin();
  for(i=0; i < tent_size; i++, ptr1++)
    {
      if (flag_unique[i] == 0) continue;
      std::vector<TentativeCorrespExt>::iterator ptr2 = ptr1+1;
      for(j=i+1; j < tent_size; j++, ptr2++)
        {
          if (flag_unique[j] == 0) continue;
          double dx = (ptr1->first.reproj_kp.x - ptr2->first.reproj_kp.x);
          double dy = (ptr1->first.reproj_kp.y - ptr2->first.reproj_kp.y);
          d1_sq = dx*dx+dy*dy;
          if (d1_sq > r_sq)
            continue;
          dx = (ptr1->second.reproj_kp.x - ptr2->second.reproj_kp.x);
          dy = (ptr1->second.reproj_kp.y - ptr2->second.reproj_kp.y);
          d2_sq = dx*dx+dy*dy;
          if (d2_sq <= r_sq)
            flag_unique[j] = 0;
        }
    }
  TentativeCorrespListExt unique_list;
  unique_list.TCList.reserve(0.8*in_corresp.TCList.size());
  for (i=0; i<9; i++)
    unique_list.H[i] = in_corresp.H[i];

  for (i=0; i<tent_size; i++)
    if (flag_unique[i] == 1)
      unique_list.TCList.push_back(in_corresp.TCList[i]);

  in_corresp.TCList = unique_list.TCList;
}

void WriteH(double* H, std::ostream &out1)
{
  out1  << H[0] << " " << H[1] << " " << H[2] << endl
                << H[3] << " " << H[4] << " " << H[5] << endl
                << H[6] << " " << H[7] << " " << H[8] << endl;
}
double L2_scalar(Keypoint4Match &k1,Keypoint4Match &k2)
{
  cv::Mat A(1,128,CV_8U,k1.desc);
  cv::Mat B(1,128,CV_8U,k2.desc);
  double distsq = A.dot(B); //(k1[i]-k2[i])^2 = k1[i]^2 + k2[i]^2 - 2*k1[i]*k2[i]. Sum(k1[i]^2) = Sum(k2[i]^2) = 512^2;
  return distsq;              //So, it is possible to compute sum(k1[i]*k2[i]) only.
}
double BFMatch(Keypoint4Match& key, Keypoint4MatchList& klist, int &min, double &dot_prod1, double &dot_prod2, const double ContrDistSq = 100.0)
{
  unsigned int i, geom_inc;
  double cur_scal, scal_product1, scal_product2;
  scal_product1 = dot_prod1;
  scal_product2 = dot_prod2;

  for (i=0; i< klist.size(); i++)
    {
      cur_scal = L2_scalar(key, klist[i]);
      if (cur_scal > scal_product1)
        {
          double distsq = (klist[min].x-klist[i].x)*(klist[min].x-klist[i].x)
              + (klist[min].y-klist[i].y)*(klist[min].y-klist[i].y);
          if (distsq > ContrDistSq)
            {
              scal_product2 = scal_product1;
              geom_inc = min;
            }
          scal_product1 = cur_scal;
          min = i;
        }
      else if (cur_scal > scal_product2)
        {
          double distsq = (klist[min].x-klist[i].x)*(klist[min].x-klist[i].x)
              + (klist[min].y-klist[i].y)*(klist[min].y-klist[i].y);
          if (distsq > ContrDistSq)
            {
              scal_product2 = cur_scal;
              geom_inc = i;
            }

        }
    }
  dot_prod1 = scal_product1;
  dot_prod2 = scal_product2;
  return (524288 - 2*scal_product1)/(524288 - 2*scal_product2);
}

