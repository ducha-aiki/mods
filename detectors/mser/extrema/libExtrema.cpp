/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

//#include <error.h>
#include <assert.h>
#include <math.h>


#include <ary.h>
#include <matrix.h>
#include <timeutls.h>

#include "preprocess.h"
#include "suballoc.h"
#include "libExtrema.h"
#include "sortPixels.h"
#include "optThresh.h"
#include "getExtrema.h"
#include "boundary.h"

using namespace utls;
using namespace std;

namespace extrema
{
#ifdef TIME_STATS
  ExtremaStats g_extrema_stats;
#endif
  void ExtremaStats::DumpTimeStats(bool both_runs)
  {
    double  tt = total_time - initial_time, tmp;
    tmp = preprocess_time-initial_time;
    printf("\nPreprocessing        : %.4f sec (%5.2f %%)\n", tmp, tmp*100/tt);
    tmp = replace_time-preprocess_time;
    printf("Replace extrema      : %.4f sec (%5.2f %%)\n", tmp, tmp*100/tt);
    tmp = chisto_time-replace_time;
    printf("Histogram            : %.4f sec (%5.2f %%)\n", tmp, tmp*100/tt);
    tmp = histo_time-chisto_time;
    printf("Sort                 : %.4f sec (%5.2f %%)\n", tmp, tmp*100/tt);
    tmp = extrema_p_time-histo_time;
    printf("Extrema and thresh + : %.4f sec (%5.2f %%)\n", tmp, tmp*100/tt);
    tmp = output_p_time-extrema_p_time;
    printf("Output             + : %.4f sec (%5.2f %%)\n", tmp, tmp*100/tt);
    if (both_runs)
      {
        tmp = extrema_m_time-output_p_time;
        printf("Extrema and thresh - : %.4f sec (%5.2f %%)\n", tmp, tmp*100/tt);
        tmp = output_m_time-extrema_m_time;
        printf("Output             - : %.4f sec (%5.2f %%)\n", tmp, tmp*100/tt);
      }
    printf("Total                : %.4f sec\n\n", tt);
  }

  void HistoImage(const ExtremaParams &par, BAry *&img, t_sortpixels &pixels)
  {
    g_cols = img->cols();
#ifdef TIME_STATS
    g_extrema_stats.num_extrema_replaced = 0;
#endif
    if (par.replace_with_ext)
      {
#ifdef TIME_STATS
        g_extrema_stats.num_extrema_replaced =
    #endif
            ReplaceExtWithSecond(img);
      }
#ifdef TIME_STATS
    g_extrema_stats.replace_time = get_time();
#endif
    CalcHistogram(img, pixels);
#ifdef TIME_STATS
    g_extrema_stats.chisto_time = get_time();
#endif
    BinSortPixels(img, pixels);
#ifdef TIME_STATS
    g_extrema_stats.histo_time = get_time();
#endif
  }

  bool PreparePixels(const ExtremaImage &image, const ExtremaParams &par, BAry *&img)
  {
    switch (image.channels)
      {
      case 1:
        if (par.preprocess!=0)
          {
            assert(!"Preprocessing defined on one channel image!");
            return false;
          }
        break;
      case 3:
        if (par.preprocess==0)
          {
            assert(!"No preprocessing defined on three channel image!");
            return false;
          }
        break;
      default:
        assert(!"Unsupported number of colour channels!");
        return false;
      }
    CPreprocessRGBSeq p;
    /* setup preprocessing params */
    p.preprocess(image, par.preprocess, img);
    if (!img)
      {
        assert(!"Can't preprocess image.");
        return false;
      }
#ifdef TIME_STATS
    g_extrema_stats.preprocess_time = get_time();
#endif
    return true;
  }

  void RLE2Ellipse(const vector <RLEItem> &rle,
                   double &barX, double &barY,
                   double &sumX2, double &sumXY, double &sumY2)
  {
    double area       = 0;
    double sumX       = 0;
    double sumY       = 0;
    int j;
    for (j=0; j<(int)rle.size(); j++)
      {
        double line = rle[j].line;
        double m    = rle[j].col1;
        double n    = 1+rle[j].col2;
        // maple: int(int(x, x=a..b), y=c..c+1);
        sumX += (n*n-m*m) / 2;
        // maple: int(int(y, y=c..c+1), x=a..b);
        sumY += (n-m)*(2*line+1)/2;
        area += n-m;
      }
    barX = (double) sumX / (double) area;
    barY = (double) sumY / (double) area;

    sumX2 = sumY2 = sumXY = 0;

    for(j=0; j <(int)rle.size(); j++)
      {
        double line = rle[j].line - barY;
        double m    = rle[j].col1 - barX;
        double n    = 1+rle[j].col2 - barX;
        double l2 = line*line;
        double m2=m*m;
        double n2=n*n;
        // maple: int(int(x^2, x=a..b), y=c..c+1);
        sumX2 += (n2*n-m2*m)/3;
        // maple: int(int(y^2, y=c..c+1), x=a..b);
        sumY2 += (n-m)*(3*l2+3*line+1)/3;
        // maple: simplify(int(int((x-m), x=a..b)*(y-n), y=c..c+1));
        sumXY += -.25*(m2-n2)*(2*line+1);
      }
    sumX2 /= (double) area;
    sumY2 /= (double) area;
    sumXY /= (double) area;
  }

  void ReducedBoundary2RLE(vector<BoundaryPoint> &reduced_boundary, vector<RLEItem> &rle)
  {
    bool startRLE = true;
    // calculate RLE and ellipse
    RLEItem run;
    for (vector<BoundaryPoint>::iterator it=reduced_boundary.begin(); it!=reduced_boundary.end(); it++)
      {
        /* boundary -> rle */
        if (startRLE)
          {
            run.line = it->line;
            run.col1 = it->col + 1;
          }
        else
          {
            if (run.line != it->line)
              printf("error-start [%d, %d] end [%d, %d]\n",
                     run.line, run.col1,
                     it->line, it->col);
            run.col2 = it->col - 1;
            rle.push_back(run);
          }
        startRLE = !startRLE;
      }
  }

  void ExtremaRegion2Region(t_region *p_r, t_thresh_def *p_t, Region &region)
  {
    BoundaryRegion b;
    region.label = p_r->label;
    region.minI = p_r->minimum_int;
    region.maxI = p_r->maximum_int;
    region.threshold = p_t->thresh;
    region.margin = p_t->margin;
    region.extremumX = p_r->minimum_pos.x;
    region.extremumY = p_r->minimum_pos.y;
    region.area = p_r->pixels[p_t->thresh];
    region.border = p_r->borders[p_t->thresh];
  }

  static inline void ConvertBoundary(point_vector *p_boundary,
                                     vector<BoundaryPoint> &boundary)
  {
    boundary.reserve(p_boundary->size());
    BoundaryPoint pt;
    for (point_vector::iterator it=p_boundary->begin(); it!=p_boundary->end(); it++)
      {
        pt.line = it->ofs/g_cols;
        pt.col = it->ofs - g_cols*pt.line;
        pt.col--;
        pt.line--;
        boundary.push_back(pt);
      }
  }

  static inline void ConvertAndReduceBoundary(point_vector *p_boundary,
                                              vector<BoundaryPoint> &boundary,
                                              vector<BoundaryPoint> &reduced_boundary)
  {
    boundary.reserve(p_boundary->size());
    reduced_boundary.reserve(p_boundary->size());
    BoundaryPoint pt;
    for (point_vector::iterator it=p_boundary->begin(); it!=p_boundary->end(); it++)
      {
        pt.line = it->ofs/g_cols;
        pt.col = it->ofs - g_cols*pt.line;
        pt.col--;
        pt.line--;
        boundary.push_back(pt);
        if ((it->direct & 0x05)==0)
          reduced_boundary.push_back(pt);
      }
  }

  static inline void ReduceBoundary(point_vector *p_boundary,
                                    vector<BoundaryPoint> &reduced_boundary)
  {
    reduced_boundary.reserve(p_boundary->size());
    BoundaryPoint pt;
    for (point_vector::iterator it=p_boundary->begin(); it!=p_boundary->end(); it++)
      {
        if ((it->direct & 0x05)==0)
          {
            pt.line = it->ofs/g_cols;
            pt.col = it->ofs - g_cols*pt.line;
            pt.col--;
            pt.line--;
            reduced_boundary.push_back(pt);
          }
      }
  }

  /* generate boundary for further processing */
  void OutputExtBoundary(t_LL regions, vector<BoundaryRegion> &boundary_extrema)
  {
    t_region     *p_r;
    t_thresh_def *p_t;
    int           id = 0;
    ForeachTyLL_M(regions, p_r, t_region*)
    {
      id++;
      if(p_r->thresholds && !IsEmptyLL(p_r->thresholds))
        {
          ForeachTyLL_M(p_r->thresholds, p_t, t_thresh_def*)
          {
            point_vector  *boundary = p_t->boundary;
            BoundaryRegion b;
            b.rid = id;
            ExtremaRegion2Region(p_r, p_t, b);
            ConvertBoundary(boundary, b.boundary);
            boundary_extrema.push_back(b);
          }
        }
    }
  }

  /* generate boundary and ellipse for further processing */
  void OutputExtBoundaryAndEll(t_LL regions, vector<BoundaryRegion> &boundary_extrema)
  {
    t_region     *p_r;
    t_thresh_def *p_t;
    int           id = 0;
    ForeachTyLL_M(regions, p_r, t_region*)
    {
      id++;
      if(p_r->thresholds && !IsEmptyLL(p_r->thresholds))
        {
          ForeachTyLL_M(p_r->thresholds, p_t, t_thresh_def*)
          {
            point_vector         *boundary = p_t->boundary;
            vector<BoundaryPoint> reduced_boundary;
            vector<RLEItem>       rle;

            // create BoundaryRegion
            BoundaryRegion b;
            b.rid = id;
            ExtremaRegion2Region(p_r, p_t, b);
            ConvertAndReduceBoundary(boundary, b.boundary, reduced_boundary);
            // calc covariance matrix
            ReducedBoundary2RLE(reduced_boundary, rle);
            RLE2Ellipse(rle, b.cx, b.cy, b.sxx, b.sxy, b.syy);
            boundary_extrema.push_back(b);
          }
        }
    }
  }

  /* generate RLE regions and Ellipse for further processing */
  void OutputRLEAndEll(t_LL regions, vector<RLERegion> &RLEextrema)
  {
    t_region     *p_r;
    t_thresh_def *p_t;
    int           id = 0;
    ForeachTyLL_M(regions, p_r, t_region*)
    {
      id++;
      if(p_r->thresholds && !IsEmptyLL(p_r->thresholds))
        {
          ForeachTyLL_M(p_r->thresholds, p_t, t_thresh_def*)
          {
            point_vector         *p_boundary = p_t->boundary;
            vector<BoundaryPoint> reduced_boundary;
            ReduceBoundary(p_boundary, reduced_boundary);
            RLERegion reg;
            reg.rid = id;
            ExtremaRegion2Region(p_r, p_t, reg);
            ReducedBoundary2RLE(reduced_boundary, reg.rle);
            // calc covariance matrix
            RLE2Ellipse(reg.rle, reg.cx, reg.cy, reg.sxx, reg.sxy, reg.syy);
            RLEextrema.push_back(reg);
          }
        }
    }
  }

  /* state interface helper variables */
  t_sortpixels g_preprocessed_pixels;
  BAry  *g_preprocessed_image=0;
#pragma omp threadprivate(g_preprocessed_image, g_preprocessed_pixels)

  void extremaPrepareImage(const ExtremaParams &params,
                           const ExtremaImage &image)
  {
#ifdef TIME_STATS
    g_extrema_stats.initial_time = get_time();
#endif
    // printf("Preparing\n");
    if (g_preprocessed_image!=0)
      assert(!"Preprocessed image already assigned. Missing call to extremaCleanup?");
    /* do all important global assigments here */
    PreparePixels(image, params, g_preprocessed_image);
    HistoImage(params, g_preprocessed_image, g_preprocessed_pixels);
  }

  void extremaAttachImage(const ExtremaParams &params, BAry *image)
  {
#ifdef TIME_STATS
    g_extrema_stats.initial_time = get_time();
#endif
    //printf("Attaching\n");
    if (g_preprocessed_image!=0)
      assert(!"Preprocessed image already assigned. Missing call to extremaCleanup?");
    /* do all important global assigments here */
    g_preprocessed_image=image;
    HistoImage(params, g_preprocessed_image, g_preprocessed_pixels);
  }

  void extremaInvertImage()
  {
    /* invert image and swap histogram */
    InvertImageAndHistogram(g_preprocessed_image, g_preprocessed_pixels);
  }

  void extremaBoundaryRegions(const ExtremaParams &params, bool inverted, vector<BoundaryRegion> &result)
  {
    t_LL regions;
    regions = GetExtrema(g_preprocessed_image, g_preprocessed_pixels, params, inverted);
#ifdef TIME_STATS
    g_extrema_stats.extrema_m_time = get_time();
#endif
    RegionBoundaries(g_preprocessed_image, regions);
    OutputExtBoundary(regions, result);
    DestRegions(regions);
    DestRegionRecycling();
#ifdef TIME_STATS
    g_extrema_stats.output_m_time = get_time();
#endif
  }

  void extremaBoundaryEllRegions(const ExtremaParams &params, bool inverted, vector<BoundaryRegion> &result)
  {
    t_LL regions;
    regions = GetExtrema(g_preprocessed_image, g_preprocessed_pixels, params, inverted);
#ifdef TIME_STATS
    g_extrema_stats.extrema_m_time = get_time();
#endif
    RegionBoundaries(g_preprocessed_image, regions);
    OutputExtBoundaryAndEll(regions, result);
    DestRegions(regions);
    DestRegionRecycling();
#ifdef TIME_STATS
    g_extrema_stats.output_m_time = get_time();
#endif
  }

  void extremaRLERegions(const ExtremaParams &params, bool inverted, vector<RLERegion> &result)
  {
    t_LL regions;
    regions = GetExtrema(g_preprocessed_image, g_preprocessed_pixels, params, inverted);
#ifdef TIME_STATS
    g_extrema_stats.extrema_m_time = get_time();
#endif
    RegionBoundaries(g_preprocessed_image, regions);
    OutputRLEAndEll(regions, result);
    DestRegions(regions);
    DestRegionRecycling();
#ifdef TIME_STATS
    g_extrema_stats.output_m_time = get_time();
#endif
  }

  void extremaCleanup(bool detach_only)
  {
    /* clean up all global strctures */
    free(g_preprocessed_pixels.block);
    if (!detach_only)
      delete g_preprocessed_image;
    g_preprocessed_image = 0;
    // printf("Cleaned\n");
#ifdef TIME_STATS
    g_extrema_stats.total_time = get_time();
#endif
  }


  const ExtremaStats &extremaStats()
  {
    return g_extrema_stats;
  }

  BoundaryExtrema getBoundaryExtrema (const ExtremaParams &params,
                                      const ExtremaImage &image,
                                      int both)
  {
    BoundaryExtrema result;
    extremaPrepareImage(params, image);
    /* counting MSER+ */
    if (both & GENERATE_MSER_PLUS)
      extremaBoundaryRegions(params, false, result.MSERplus);
    extremaInvertImage();
#ifdef TIME_STATS
    g_extrema_stats.extrema_p_time = g_extrema_stats.extrema_m_time;
    g_extrema_stats.output_p_time = g_extrema_stats.output_m_time;
#endif
    if (both & GENERATE_MSER_MINUS)
      extremaBoundaryRegions(params, true, result.MSERmin);
    extremaCleanup();
    if (params.debug>0) g_extrema_stats.DumpTimeStats();
    return result;
  };

  RLEExtrema getRLEExtrema (const ExtremaParams &params,
                            const ExtremaImage &image,
                            int both)
  {
    RLEExtrema result;
    extremaPrepareImage(params, image);

    if (both & GENERATE_MSER_PLUS)
      extremaRLERegions(params, false, result.MSERplus);
    extremaInvertImage();
#ifdef TIME_STATS
    g_extrema_stats.extrema_p_time = g_extrema_stats.extrema_m_time;
    g_extrema_stats.output_p_time = g_extrema_stats.output_m_time;
#endif
    if (both & GENERATE_MSER_MINUS)
      extremaRLERegions(params, true, result.MSERmin);

    extremaCleanup();
    if (params.debug>0) g_extrema_stats.DumpTimeStats();
    return result;
  };

  void exportRLEVector(FILE *fid, vector<RLERegion> &rle_vector)
  {
    size_t i, j;
    sort(rle_vector.begin(), rle_vector.end());

    fprintf(fid, "%d\n", (int)rle_vector.size());
    for(i=0; i < rle_vector.size(); i++)
      {
        const RLERegion *r = &rle_vector[i];
        fprintf(fid, "%d", (int) r->rle.size());
        for (j=0; j<r->rle.size(); j++)
          fprintf(fid, " %d %d %d", r->rle[j].line,
                  r->rle[j].col1, r->rle[j].col2);
        fprintf(fid, "\n");
      }
  }

  void exportBoundaryVector(FILE *fid, vector<BoundaryRegion>
                            &boundary_vector)
  {
    size_t i, j;
    sort(boundary_vector.begin(), boundary_vector.end());
    fprintf(fid, "%d\n", (int)boundary_vector.size());
    for(i=0; i < boundary_vector.size(); i++)
      {
        const BoundaryRegion *r = &boundary_vector[i];
        fprintf(fid, "%d", (int)r->boundary.size());
        for (j=0; j<r->boundary.size(); j++)
          fprintf(fid, " %d %d", r->boundary[j].col, r->boundary[j].line);
        fprintf(fid, "\n");
      }
  }

  void exportBoundaryVectorGF(FILE *fid, vector<BoundaryRegion>
                              &boundary_vector)
  {
    int id=0;
    size_t i, j;
    char format[1024];
    sort(boundary_vector.begin(), boundary_vector.end());
    sprintf(format, "num reg_id mini thresh margin min_y min_x");
    strcat(format, " area border Pointset");

    fprintf(fid, "Format %s\n", format);

    for(i=0; i < boundary_vector.size(); i++)
      {
        const BoundaryRegion *r = &boundary_vector[i];
        fprintf(fid, "%-3d ", ++id);
        fprintf(fid, "%-5d ", r->label);
        fprintf(fid, "%3d ", r->minI);
        fprintf(fid, "%3d ", r->threshold);
        fprintf(fid, "%3d   ", r->margin);
        fprintf(fid, "%3d %3d   ", r->extremumX, r->extremumY);
        fprintf(fid, "%6d %6d ", r->area, r->border);
        fprintf(fid, "%d   ", (int)r->boundary.size());
        for (j=0; j<r->boundary.size(); j++)
          fprintf(fid, " %d %d", r->boundary[j].col, r->boundary[j].line);
        fprintf(fid, "\n");
      }
    fprintf(fid, "@\n");
  }

  void exportAffVector(FILE *fid, vector<RLERegion> &rle_vector,
                       double factor, int krys_compat)
  {
    size_t i;
    if (!krys_compat)
      fprintf(fid, "%d\n", (int)rle_vector.size());

    sort(rle_vector.begin(), rle_vector.end());

    for(i=0; i < rle_vector.size(); i++)
      {
        const RLERegion *r = &rle_vector[i];
        double barX, barY, sumX2, sumY2, sumXY;
        RLE2Ellipse(r->rle, barX, barY, sumX2, sumXY, sumY2);
        Matrix2 C(sumX2, sumXY, sumXY, sumY2);
        Matrix2 U, T, A;
        C=C*(factor*factor);
        if (krys_compat)
          {
            A = C.inv();
            fprintf(fid, "%g %g %g %g %g %d\n", barX, barY,
                    A[0][0], A[0][1], A[1][1], r->margin);
          }
        else
          {
            C.schur_sym(U, T);
            A = U * T.sqrt() * U.transpose();
            fprintf(fid, "%g %g %g %g %g %g %d\n", barX, barY,
                    A[0][0], A[0][1], A[1][0], A[1][1], r->margin);
          }
      }
  }
}
