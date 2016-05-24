/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#include <math.h>
#include <string.h>
#include "optThresh.h"
#include "getExtrema.h"
#include "extremaParams.h"

namespace extrema
{
  /*----------------------------------------------------------------*/

  static void SuppresOverlappingTresholds4StableRegions(t_region * p_r, int * cummulAreas)
  {
    t_thresh_def *p_t;
    if (p_r->thresholds == NULL)
      return;
    ForeachTyLL_M(p_r->thresholds, p_t, t_thresh_def*)
    {
      while(!IsLastElmLL(p_t))
        {
          t_thresh_def * p_next = (t_thresh_def *) NextElmLL(p_t);
          if ((p_t->pos + p_t->margin < p_next->thresh) &&
              (p_t->thresh            < p_next->pos))
            /* no overlap */
            break;

          /* overlap, remove the lower quality(=margin) region */
          if (p_next->margin <= p_t->margin)
            {
              /* merge regions */
              DelElmLL(p_next);
            }
          else
            {
              p_t = (t_thresh_def *) DelElmPrLL(p_t);
              /* current removed, break out */
              break;
            }
        }
    }

    /* test - merge regions with small (10%) diference of area */
    ForeachTyLL_M(p_r->thresholds, p_t, t_thresh_def*)
    {
      while(!IsLastElmLL(p_t))
        {
          t_thresh_def * p_next = (t_thresh_def *) NextElmLL(p_t);

          if (p_t->pos + p_t->margin < p_next->pos)
            break;

          if (cummulAreas[p_next->thresh] - cummulAreas[p_t->thresh] <=
              0.1 * cummulAreas[p_t->thresh])
            {
              p_t->margin = p_next->pos - p_t->pos + p_next->margin;
              p_t->thresh = p_t->pos + p_t->margin / 2;
              DelElmLL(p_next);
            }
          else break;
        }
    }
  }

  /*----------------------------------------------------------------*/

  void FastSetOptThresholds4StableRegion(t_region *p_r)
  {
    if (p_r->pixel_total < g_thresh_params.min_size )
      return;

    /* see below quality */
    int invertCons  = (g_thresh_params.invert) ? 255  : 0;
    int invertMulti = (g_thresh_params.invert) ? (-1) : 1;

    int *cummulAreas = p_r->pixels;
    int *cummulBorders = p_r->borders;

    int i;

    /* calculate all cummulative stats */
    for(i=p_r->minimum_int+1; i <= p_r->maximum_int; i++)
      {
        p_r->pixels[i] += p_r->pixels[i-1];
        p_r->borders[i] += p_r->borders[i-1];
      }

    int up, localMaxMargin = -1, localMaxPos = -1;

    /* look for threshold that guarantee area bigger than min_size*/
    i=p_r->minimum_int;

    /* continue with smallest i for which the area is big enough */
    do
      {
        int area_i = cummulAreas[i];
        int radius_i = cummulBorders[i];

        /* test from the first acceptable threshold */
        up = int(i + g_thresh_params.min_margin);

        if (up > p_r->maximum_int)
          break;

        /* evaluate stability criterion */
        while ((cummulAreas[up] - area_i < radius_i) && (up < p_r->maximum_int)) up++;

        int margin  = up - i;
        double quality = (double) margin;
        if (g_thresh_params.relative_margin)
          quality /= invertCons + invertMulti * (i + (margin/2));

        /* non-maximum suppression */
        if (quality > g_thresh_params.min_margin &&
            margin >= localMaxMargin)
          {
            /* if margin are not descending & are higher than
               min_margin, actualise local maxima position */
            localMaxMargin = margin;
            localMaxPos = i;
          }
        else
          {
            /* margin is bellow the min_margin or margin
               function is descending */
            if (localMaxPos >= 0)
              {
                t_thresh_def t;
                t.thresh = localMaxPos + localMaxMargin/2;
                if (cummulAreas[t.thresh] <= g_thresh_params.max_size && cummulAreas[t.thresh]>g_thresh_params.min_size)
                  {
                    t.pos    = localMaxPos;
                    t.margin = localMaxMargin;
                    t.boundary = 0;
                    if (p_r->thresholds==0)
                      p_r->thresholds = ConsLL();
                    InsLastLL(p_r->thresholds, t);
                  }
                localMaxPos = -1;
              }
            localMaxMargin = margin;
          }
        i++;
      }
    while (up < p_r->maximum_int);

    /* process last local maximum */
    if (localMaxPos >= 0)
      {
        t_thresh_def t;
        t.thresh = localMaxPos + localMaxMargin/2;
        if (cummulAreas[t.thresh] <= g_thresh_params.max_size && cummulAreas[t.thresh]>g_thresh_params.min_size)
          {
            t.pos    = localMaxPos;
            t.margin = localMaxMargin;
            t.boundary = 0;
            if (p_r->thresholds==0)
              p_r->thresholds = ConsLL();
            InsLastLL(p_r->thresholds, t);
          }
        localMaxMargin = localMaxPos = -1;
      }
    SuppresOverlappingTresholds4StableRegions(p_r, cummulAreas);
  }
}
