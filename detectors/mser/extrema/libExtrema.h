/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

/**
   \file libExtrema.h
   * Interface of MSER detector.

   This file contains main external interface of the MSERs detector.
 */

#ifndef __LIB_EXTREMA_H__
#define __LIB_EXTREMA_H__
#undef __STRICT_ANSI__
#include <algorithm>
#include <vector>
//#include <stdio.h>

#include "ary.h"
#include "extremaConfig.h"
#include "extremaParams.h"

using namespace std;

namespace extrema
{
  //! A structure that holds coordinates of a point in BoundaryRegion i.e. the boundary representation of a region.
  struct BoundaryPoint
  {
    int line;
    int col;
  };

  //! A structure that holds coordinates of a RLE element in RLERegion i.e. the RLE representation of a region.
  struct RLEItem
  {
    int line;
    int col1, col2;
  };

  //! A structure with common statistics of BoundaryRegion and RLERegion.
  struct Region
  {
    //! Unique region's label.
    int label;
    //! Minimum intensity.
    int minI;
    //! Maximum intensity.
    int maxI;
    //! Stability i.e. the length of stable intensities range.
    int margin;
    //! Thresholded intensity.
    int threshold;
    //! Position of initial point of the region, a zero-based X coordinate.
    int extremumX;
    //! Position of initial point of the region, a zero-based Y coordinate .
    int extremumY;
    //! A region area at the thresholded intensity level.
    int area;
    //! A region border length at the thresholded intensity level.
    int border;
    //! Centroid of the region at the thresholded level.
    double cx;
    //! Centroid of the region at the thresholded level.
    double cy;
    //! Second moments of the region at the thresholded level.
    double sxx;
    //! Second moments of the region at the thresholded level.
    double sxy;
    //! Second moments of the region at the thresholded level.
    double syy;
    //! Unique region id, i.e. index of the region.
    int rid;
  };

  //! Description of a RLE region.
  struct RLERegion : public Region
  {
    //! Vector that holds RLE representation of the region.
    vector <RLEItem> rle;

    //! Stability ordering operator.
    bool operator<(const RLERegion &a) const
    {
      return (margin>a.margin);
    }
  };

  //! Description of a boundary region.
  struct BoundaryRegion : public Region
  {
    //! Vector that holds boundary representation of the region.
    vector <BoundaryPoint> boundary;

    //! Stability ordering operator.
    bool operator<(const BoundaryRegion &a) const
    {
      return (margin>a.margin);
    }
  };

  //! Timing statistics of the detector, gathered only if TIME_STATS is set.
  struct ExtremaStats
  {
    int    num_extrema_replaced;
    double initial_time;
    double preprocess_time;
    double replace_time;
    double chisto_time;
    double histo_time;
    double extrema_p_time;
    double output_p_time;
    double extrema_m_time;
    double output_m_time;
    double total_time;
    void DumpTimeStats(bool both_runs=true);
  };

  //! Old interface structure that holds result of getRLEExtrema
  struct RLEExtrema
  {
    vector <RLERegion> MSERplus;
    vector <RLERegion> MSERmin;
  };

  //! Old interface structure that holds result of getBoundaryExtrema
  struct BoundaryExtrema
  {
    vector <BoundaryRegion> MSERplus;
    vector <BoundaryRegion> MSERmin;
  };

  //! Old interface function, produces BoundaryRegions for a given image. Computes MSERs: both=1 +, both = 2 -, or both = 3 + and -
  BoundaryExtrema getBoundaryExtrema (const ExtremaParams &params, const ExtremaImage &image, int both=3);

  //! Old interface function, produces RLERegions for a given image. Computes MSERs: both=1 +, both = 2 -, or both = 3 + and -
  RLEExtrema getRLEExtrema (const ExtremaParams &par,
                            const ExtremaImage &image,
                            int both=3);

  /* Extrema state interface */

  /**
    \brief Prepares image for detection of MSER regions.

    \param params a structure ExtremaPars with detector parameters.
    \param image a structure ExtremaImage with image data.

    Preparation involves preprocessing i.e. performs demanded preprocess operation
    given by ExtremaParams.preprocess. Image is copied into internal structure and it's
    boundary is extended by one pixel.
    \see EXTREMA_PREPROCESS, ExtremaParams.
*/
  void extremaPrepareImage(const ExtremaParams &params, const ExtremaImage &image);

  /**
    \brief Assigns already prepared image for detection of MSER regions.

    \param params a structure ExtremaParams with detector parameters.
    \param image a structure utls::BAry (byte image) with image, please note image boundaries should be extended by one pixel, i.e.
    for an image of size width x height one should use constructor BAry(-1,-1,width,height), that results in array of width+2 x height+2 elements.

    \see utls::BAry, ExtremaParams.
*/
  void extremaAttachImage(const ExtremaParams &params, utls::BAry *image);

  /**
    \brief Inverts image in internal structure.
*/
  void extremaInvertImage();

  /**
   \brief Detects MSERs inverted or not inverted image.

    \param params a structure ExtremaParams with detector parameters.
    \param inverted a boolean that signalise if internal image structure was or was not inverted.
    \param result a vector of BoundaryRegion structures containing regions.

    \note This function does not compute and thus fill values of centroid and second moments in Region structure.
    \see BoundaryRegion, ExtremaParams
*/
  void extremaBoundaryRegions(const ExtremaParams &params, bool inverted, vector<BoundaryRegion> &result);

  /**
   \brief Detects MSERs inverted or not inverted image and computes centroids and second moments of each region.

    \param params a structure ExtremaParams with detector parameters.
    \param inverted a boolean that signalise if internal image structure was or was not inverted.
    \param result a vector of BoundaryRegion structures containing regions.

    \see BoundaryRegion, ExtremaParams
*/
  void extremaBoundaryEllRegions(const ExtremaParams &params, bool inverted, vector<BoundaryRegion> &result);

  /**
   \brief Detects MSERs inverted or not inverted image and computes centroids and second moments of each region.

    \param params a structure ExtremaParams with detector parameters.
    \param inverted a boolean that signalise if internal image structure was or was not inverted.
    \param result a vector of RLERegion structures containing regions.

    \see RLERegion, ExtremaParams
*/
  void extremaRLERegions(const ExtremaParams &params, bool inverted, vector<RLERegion> &result);

  /**
    \brief Cleans up internal image structure.

    \param detach_only a boolean that specifies if the image is deallocated (false) or not.
*/
  void extremaCleanup(bool detach_only = false);

  /**
    \brief Returns timing statistics.

    \retval A structure ExtremaStats.
*/
  const ExtremaStats &extremaStats();

  void exportRLEVector(FILE *fid, vector<RLERegion> &rle_vector);
  void exportBoundaryVector(FILE *fid, vector<BoundaryRegion> &boundary_vector);
  void exportBoundaryVectorGF(FILE *fid, vector<BoundaryRegion> &boundary_vector);
  void exportAffVector(FILE *fid, vector<RLERegion> &rle_vector, double factor, int krys_compat);

  void RLE2Ellipse(const vector <RLEItem> &rle,
                   double &barX, double &barY,
                   double &sumX2, double &sumXY, double &sumY2);
  void ReducedBoundary2RLE(vector<BoundaryPoint> &reduced_boundary, vector<RLEItem> &rle);

};
#endif
