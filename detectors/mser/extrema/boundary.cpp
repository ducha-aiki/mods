/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include "extremaTypes.h"
#include "getExtrema.h"
#include "boundary.h"
#include "suballoc.h"

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace utls;

namespace extrema
{

  /*----------------------------------------------------------------*/

  static unsigned char *image_ptr = 0;
  static unsigned char *bck_ptr = 0;
#pragma omp threadprivate(image_ptr, bck_ptr)


#define is_inside(position, max_int) (image_ptr[position] <= max_int && bck_ptr[position]!=255)

  /*----------------------------------------------------------------*/
  inline static void add_if_inside(point_vector &inside, point_vector &boundary,
                                   unsigned char max_int, int position,
                                   unsigned char direct, unsigned char marker)
  {
    unsigned char v = bck_ptr[position];
    if (marker == v)
      return;
    // allocate
    t_borderpixel p;

    p.ofs = position;
    p.direct = direct;

    if (image_ptr[position] <= max_int && v!=255)
      {
        inside.push_back(p);
        bck_ptr[position] = marker;
      }
    else
      boundary.push_back(p);
  }

  /*----------------------------------------------------------------*/
  static void ConnectedComponent(unsigned char thresh, int marker, point_vector &inside, point_vector &boundary)
  {
    // simple connected component
    while (!inside.empty())
      {
        t_offset ofs = inside.back().ofs;
        inside.pop_back();
        add_if_inside(inside, boundary, thresh, ofs + g_cols, 4, marker);
        add_if_inside(inside, boundary, thresh, ofs - g_cols, 1, marker);
        add_if_inside(inside, boundary, thresh, ofs + 1,      2, marker);
        add_if_inside(inside, boundary, thresh, ofs - 1,      8, marker);
      }

    /* sort region's boundary */
    std::sort(boundary.begin(), boundary.end());
  }

  t_LL g_thresholds[c_maxByte];

#pragma omp threadprivate (g_thresholds)

  /*----------------------------------------------------------------*/

  int SortRegionThresholds(t_LL regions)
  {
    int num_regions=0;
    t_region * p_r;
    // clear thresholds histogram
    memset(g_thresholds, 0, sizeof(g_thresholds));
    // build new
    ForeachTyLL_M(regions, p_r, t_region*)
    {
      if (p_r->thresholds)
        {
          t_thresh_def *p_t;
          ForeachTyLL_M(p_r->thresholds, p_t, t_thresh_def*)
          {
            t_LL *thresh = g_thresholds + p_t->thresh;
            if (!*thresh)
              *thresh=ConsLL();
            t_sorted_region_threshold t;
            t.p_r=p_r;
            t.p_t=p_t;
            InsLastLL(*thresh, t);
            num_regions++;
          }
        }
    }
    return num_regions;
  }

  /*----------------------------------------------------------------*/
  void RegionBoundaries(BAry *img, t_LL regions)
  {
    int i, cols, rows;
    cols = img->cols()-2;
    rows = img->rows()-2;
    BAry * bck = 0;
    /* get image pointer */
    image_ptr = &img->el[-1][-1];

    int num_regions = SortRegionThresholds(regions);

    // if there is something to do...
    if (num_regions>0)
      {
        // create copy of the image -> "background picture"
        bck = new BAry(img->lb1, img->ub1, img->lb2, img->ub2);
        bck_ptr = &bck->el[-1][-1];
        memset(bck_ptr, 0, sizeof(unsigned char)*(rows+2)*(cols+2));
        for (i=0; i<cols; i++) bck->el[-1][i] = bck->el[rows][i] = 255;
        for (i=0; i<rows; i++) bck->el[i][-1] = bck->el[i][cols] = 255;

        int dir_tab[9];
        dir_tab[1]=g_cols;
        dir_tab[2]=-1;
        dir_tab[4]=-g_cols;
        dir_tab[8]=1;
        // ignore c_maxByte threshold -> whole image
        point_vector inside;
        inside.reserve(g_thresh_params.max_size);
        for (int i=0; i<c_maxByte-1; i++)
          {
            if (g_thresholds[i])
              {
                inside.clear();
                t_sorted_region_threshold *p_rt;
                ForeachTyLL_M(g_thresholds[i], p_rt, t_sorted_region_threshold*)
                {
                  t_region * p_r = p_rt->p_r;
                  t_thresh_def *p_t = p_rt->p_t;
                  // create new regions boundary list
                  point_vector *boundary = new point_vector;
                  boundary->reserve(g_cols);
                  unsigned char thresh = p_t->thresh;
                  int marker = thresh;

                  if (p_t->boundary)
                    {
                      // there was some inner region already processed...
                      // we need to relabel 1px 8 connected inner boundary to avoid
                      // another filling of already labeled regions
                      point_vector *tmp = p_t->boundary;
                      while (!tmp->empty())
                        {
                          t_borderpixel &curr = tmp->back();
                          if (!is_inside(curr.ofs, thresh))
                            // point is not inside next intensity thresh, leave it in boundary list
                            boundary->push_back(curr);
                          else
                            {
                              // relabel originating pixel
                              bck_ptr[curr.ofs+dir_tab[curr.direct]] = marker;
                              // mark pixel
                              if (bck_ptr[curr.ofs] != marker)
                                {
                                  inside.push_back(curr);
                                  bck_ptr[curr.ofs] = marker;
                                }
                            }
                          tmp->pop_back();
                        }
                      delete p_t->boundary;
                    }
                  else
                    {
                      /* insert pixel into inside list and mark it with label */
                      t_borderpixel p;
                      p.ofs = p_r->minimum_pos.x;
                      p.direct = 0;
                      inside.push_back(p);
                      bck_ptr[p.ofs] = marker;
                    }
                  p_t->boundary = boundary;
                  ConnectedComponent(thresh, marker, inside, *boundary);
                  if (!IsLastElmLL(p_t))
                    {
                      // there is another threshold with higher thresh value...
                      p_t = (t_thresh_def*)NextElmLL(p_t);
                      p_t->boundary = new point_vector(*boundary);
                    }
                }
                DestLL(g_thresholds[i]);
                g_thresholds[i]=0;
              }
          }
      }
    if (bck)
      {
        delete(bck);
        bck=0;
      }
  }

}
