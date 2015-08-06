/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include <string.h>
#include "getExtrema.h"
#include "optThresh.h"

#ifdef _OPENMP
#include <omp.h>
#endif


/*
Label equivalency included in the label image:

pointer with bit 0,1 = 0 -> pointer to labels_ptr
pointer with bit 0 set   -> pointer to a min_reg
pointer with bit 1 set   -> pointer to a region

equivalency tree building:

ConsRegion - sets label to min_reg's pointer

UpgradeRegion - sets label to region's pointer

InsMarkPixel - sets label to the labels_ptr pixel which points to region/min_reg pointer

MergePixel - sets the label_ptr pixel which points to merged region/min_reg to the labels_ptr pixel which points to surviving region/min_reg pointer

GetLabelled - resolves label numbers up to the root region and stores the label_ptr pixel which points to the region in labelled array. it also flatten
              any more-than-one-level indirections
*/

using namespace utls;

namespace extrema
{

  /* exported variables */

  int g_region_statistics = 0;
  t_thresh_par g_thresh_params;

  //#define DEBUG_LABELS
  /*----------------------------------------------------------------*/
#ifdef A64
  static UI64Ary*   labels;
#else
  static IAry*      labels;
#endif

  static t_label      *labels_ptr;
  int                  g_cols = 0;

  t_suballocator *regionSuballocator = 0;
  t_suballocator *minimal_regionSuballocator = 0;
#pragma omp threadprivate(regionSuballocator, minimal_regionSuballocator, labels_ptr, labels, g_region_statistics )

  /*----------------------------------------------------------------*/

  void InitRegionRecycling()
  {
    if (regionSuballocator==0)
      {
        int size = sizeof(t_region);
        regionSuballocator = (t_suballocator *)malloc(sizeof(t_suballocator));
        InitSuballocator(regionSuballocator, 100, size, 1);
      }
  }

  /*----------------------------------------------------------------*/

  void DestRegionRecycling()
  {
    if (regionSuballocator!=0)
      {
        DestSuballocator(regionSuballocator);
        free(regionSuballocator);
        regionSuballocator = 0;
      }
  }

  int min(int a, int b)
  {
    return a<b?a:b;
  }

  int max(int a, int b)
  {
    return a>b?a:b;
  }

  /*----------------------------------------------------------------*/

  static int border_num = 0;
#pragma omp threadprivate (border_num)


  inline static t_region *UpgradeRegion(t_label *region_label, int intensity, t_LL regions)
  {
    /* if there are some already created regions, reuse one of them */
    t_region *pRegion = (t_region*)SuballocatorGetItem(regionSuballocator);
#ifdef DEBUG_LABELS
    if ((t_label(pRegion) & LABELPTR_MASK) != 0)
      printf("non aligned address detected\n");
#endif

    /* resolve region address from the label pointer by masking all region flags */
    t_mregion min_reg = (t_mregion)(*region_label & LABEL_MASK);

    /* relabel region from MINREG to real reg */
    pRegion->label = t_label(pRegion) | REGION_FLAG;
    pRegion->merge_label = 0;

    /* setup counters */
    pRegion->pixel_total  = (min_reg & REGION_SIZE_MASK) >> REGION_SIZE_SHIFT;
    pRegion->border_total  = min_reg >> BORDER_SIZE_SHIFT;

    /* remember location of the first pixel */
    pRegion->minimum_pos.x = (t_label)(region_label - labels_ptr);

    /* setup current intensity as initial */
    pRegion->minimum_int = intensity;
    pRegion->maximum_int = intensity;

    pRegion->pixels[intensity]  = pRegion->pixel_total;
    pRegion->borders[intensity] = pRegion->border_total;

    pRegion->thresholds = 0;

    /* insert region to the regions list */
    LinkInsLastLL(regions, *pRegion);

    /* tie the region with current label */
    *region_label = pRegion->label;

    return pRegion;
  }

  /* add pixel on position POS with intensity INTENSITY to the region PREGION */
  inline static void InsMarkPixel(t_label *region_label, t_offset ofs, int intensity, t_LL regions)
  {
    /* mark pixel with region's label */
    labels_ptr[ofs] = (t_label)region_label;

    if (*region_label & MINREG_FLAG)
      {
        /* increase pixel size and border_num+4 */
        *region_label += 0x00080004 - (border_num << BORDER_SIZE_SHIFT);
        if (int(*region_label & REGION_SIZE_MASK) >= g_thresh_params.min_size_int)
          UpgradeRegion(region_label, intensity, regions);
      }
    else
      {
        /* standard region */
        t_region *pRegion = (t_region*)(*region_label & LABEL_MASK);

        pRegion->maximum_int = intensity;

        pRegion->pixel_total ++;
        pRegion->border_total += 4 - border_num;

        /* update border counts */
        pRegion->pixels[intensity] ++;
        pRegion->borders[intensity] += 4 - border_num;
      }
  }

  /*----------------------------------------------------------------*/
  inline static void ConsRegion(t_offset ofs, int intensity, t_LL regions)
  {
    /* default minsize region: region size = 1; border size = 4; */
    labels_ptr[ofs] = 0x00080004 | MINREG_FLAG;
  }

  /*----------------------------------------------------------------*/

  static t_label *labelled[4];
  static int      label_num = 0;
#pragma omp threadprivate (label_num,labelled)

  /*----------------------------------------------------------------*/

  static inline t_label *FindEquivLabel(t_label *label)
  {
    /* deal with one level case faster (no relabeling) */
    t_label *ptr = (t_label*)*label;

    if ((*ptr & LABELPTR_MASK) != 0)
      /* this label_ptr pixel points to region */
      return ptr;

    /* find real label (the root of the labels merging tree) */
    do
      {
        ptr = (t_label *)(*ptr);
      }
    while ((*ptr & LABELPTR_MASK) == 0);

    t_label *final = ptr;
    ptr            = label;

    /* flatten this branch of the tree to the first level */
    while ((*ptr & LABELPTR_MASK) == 0)
      {
        ptr = (t_label *)(*ptr);
        *label = (t_label)final;
        label = ptr;
      }
    return final;
  }

  static inline void GetLabelled(t_offset ofs)
  {
    t_label *p = labels_ptr + ofs;

    /* pick more probable nonzero labels first */
    t_label *l1 = p-g_cols;
    t_label *l2 = p-1;
    t_label *l3 = p+1;
    t_label *l4 = p+g_cols;

    label_num = 0;
    border_num = 0;
    if (*l1!=0)
      {
        /* check if pointer is a region or not (root pointer check) */
        if ((*l1 & LABELPTR_MASK) == 0) l1 = FindEquivLabel(l1);
        labelled[label_num++] = l1;
        border_num++;
      }

    if (*l2!=0)
      {
        /* check if pointer is a region or not (root pointer check) */
        if ((*l2 & LABELPTR_MASK) == 0) l2 = FindEquivLabel(l2);
        if (l2!=l1)
          labelled[label_num++] = l2;
        border_num++;
      }

    if (*l3!=0)
      {
        /* check if pointer is a region or not (root pointer check) */
        if ((*l3 & LABELPTR_MASK) == 0) l3 = FindEquivLabel(l3);
        if (l3!=l1 && l3!=l2)
          labelled[label_num++] = l3;
        border_num++;
      }

    if (*l4!=0)
      {
        /* check if pointer is a region or not (root pointer check) */
        if ((*l4 & LABELPTR_MASK) == 0) l4 = FindEquivLabel(l4);
        if (l4!=l1 && l4!=l2 && l4!=l3)
          labelled[label_num++] = l4;
        border_num++;
      }
    /* calculate number of 4-connected neighbours */
    border_num = 2*border_num;
  }
  /*----------------------------------------------------------------*/
  static void MergeRegions(t_offset ofs, int intensity, t_LL regions)
  {
    unsigned int   maxSize  = 0;
    t_label      * maxLabel = labelled[0];
    int            num_large = 0;
    /* find the region that was the largest at level: intensity-1
       this strategy makes sure that region with smallest size increase
       survives the merge */
    for(int i=0; i < label_num; i++)
      {
        t_mregion r = (t_mregion)(*labelled[i]);
        if (!(r & MINREG_FLAG))
          {
            t_region * region = (t_region *)(r & LABEL_MASK);
            unsigned int size = region->pixel_total - region->pixels[intensity];
            num_large++;
            if (size > maxSize)
              {
                maxSize  = size;
                maxLabel = labelled[i];
              }
          }
      }
    if (!num_large)
      {
        /* simple merge */
        for(int i=1; i<label_num; i++)
          {
            *maxLabel += (*labelled[i] & LABEL_MASK);
            *labelled[i] = (t_label)maxLabel;
          }
      }
    else
      {
        t_region     * maxRegion;
        int max_has_minstats = (*maxLabel & MINREG_FLAG);
        maxRegion = (t_region *)(*maxLabel & LABEL_MASK);
        /* finalise small regions, add their pixels totals to the largest */
        for(int i=0; i < label_num; i++)
          {
            t_label      *label = labelled[i];
            if (label != maxLabel)
              {
                t_mregion min_reg = (t_mregion) *label;
                t_region * region = (t_region *)(min_reg & LABEL_MASK);
                *label = (t_label)maxLabel;
                int pixel_total, border_total;
                int merging_min_reg = min_reg & MINREG_FLAG;
                /* pick the correct stats */
                if (merging_min_reg)
                  {
                    pixel_total = (min_reg & REGION_SIZE_MASK) >> REGION_SIZE_SHIFT;
                    border_total = min_reg >> BORDER_SIZE_SHIFT;
                  }
                else
                  {
                    pixel_total = region->pixel_total;
                    border_total = region->border_total;
                  }

                if (max_has_minstats)
                  {
                    *maxLabel += (pixel_total << REGION_SIZE_SHIFT) + (border_total << BORDER_SIZE_SHIFT);
                  }
                else
                  {
                    /* merge region stats with the largest one */
                    maxRegion->pixel_total        += pixel_total;
                    maxRegion->border_total       += border_total;
                    maxRegion->pixels[intensity]  += pixel_total;
                    maxRegion->borders[intensity] += border_total;
                  }

                if (!merging_min_reg)
                  {
                    /* check region, use current intensity as maximum, in case that this is first
                       pixel at this level */
                    if (!g_thresh_params.relative_margin && (intensity - region->minimum_int + 1) <= g_thresh_params.min_margin)
                      SuballocatorReturnItem(regionSuballocator, UnlinkLL(region));
                    else
                      {
                        /* set the final intensity */
                        region->maximum_int = intensity;
                        region->merge_label = (t_label)maxLabel;

                        FastSetOptThresholds4StableRegion(region);
                        if (!region->thresholds)
                          SuballocatorReturnItem(regionSuballocator, UnlinkLL(region));
                      }
                  }
              }
          }
      }
    InsMarkPixel(maxLabel, ofs, intensity, regions);
  }

  static inline void ProcessPixel(t_LL regions, t_offset * pelm, int intensity)
  {
    switch(label_num)
      {
      case 0:
        ConsRegion(*pelm, intensity, regions);
        break;
      case 1:
        InsMarkPixel(labelled[0], *pelm, intensity, regions);
        break;
      default:
        MergeRegions(*pelm, intensity, regions);
      }
  }

  void PrepareThresholds(BAry *img, const ExtremaParams &par, t_thresh_par &thr_par, bool invert)
  {
    thr_par.min_size = par.min_size;
    thr_par.min_size_int = min(10000,par.min_size)*4; // *4 for fast com
    thr_par.max_size = (int)((img->cols()-2) * (img->rows()-2) * par.max_area);
    thr_par.min_margin = par.min_margin;
    if (par.relative) thr_par.min_margin /= 100.0;
    thr_par.invert = invert;
    thr_par.relative_margin = par.relative;
  }

  /*----------------------------------------------------------------*/
  t_LL GetExtrema(BAry* img, t_sortpixels pixels, const ExtremaParams &par, bool invert)
  {
    int     i, rows, cols;
    t_LL    regions = ConsLL();

    cols = img->cols();
    rows = img->rows();
    PrepareThresholds(img, par, g_thresh_params, invert);

    /* allocate array for labels */
#ifdef A64
    labels = new UI64Ary(img->lb1, img->ub1, img->lb2, img->ub2);
#else
    labels = new IAry(img->lb1, img->ub1, img->lb2, img->ub2);
#endif
    labels_ptr = (t_label *)&labels->el[-1][-1];

    /* setup all pointers to zero */
    memset(labels_ptr, 0, sizeof(t_label)*rows*cols);

    /* create suballocator and initialize memory */
    InitRegionRecycling();

    /* for all intensity levels */
    for(i=0; i < c_maxByte; i++)
      {
        t_offset *pend = pixels.data[i] + pixels.hist[i];
        /* all pixels of this intensity */
        for (t_offset *pelm = pixels.data[i]; pelm < pend; pelm++)
          {
            /* get the label set around pixel */
            GetLabelled(*pelm);

            /* standard getExtrema without ConsRegion */
            ProcessPixel(regions, pelm, i);
          }
      }

    /* process the last region (root), take any label and find root */
    {
      t_label *root_label = &labels_ptr[g_cols+1];
      if ((*root_label & LABELPTR_MASK) == 0) root_label = FindEquivLabel(root_label);
      t_region *r = (t_region*)(*root_label & LABEL_MASK);
      FastSetOptThresholds4StableRegion(r);
    }
    delete labels;
    return regions;
  }

  void DestRegions(t_LL regions)
  {
    t_region * pRegion;

    ForeachTyLL_M(regions, pRegion, t_region *)
    {
      if (pRegion->thresholds)
        {
          t_thresh_def *p_t;
          ForeachTyLL_M(pRegion->thresholds, p_t, t_thresh_def *)
          {
            if (p_t->boundary)
              delete p_t->boundary;
          }
          DestLL(pRegion->thresholds);
        }
    }
    /* avoid double free in DestLL(regions); */
    SuballocatorReturnItemsLL(regionSuballocator, regions);
    free(regions);
  }

}
