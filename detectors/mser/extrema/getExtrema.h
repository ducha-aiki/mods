/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#ifndef __GET_EXTREMA_H__
#define __GET_EXTREMA_H__

#include <LL.h>
#include <ary.h>
#include "extremaParams.h"
#include "extremaTypes.h"
#include "suballoc.h"

#ifdef _OPENMP
#include <omp.h>
#endif


#ifdef A64

#define LABEL_MASK       0xfffffffffffffffcl
#define MINREG_MASK      0xfffffffffffffffel
#define REGION_MASK      0xfffffffffffffffdl
#define PIXREG_MASK      0xfffffffffffffffcl
#define REGION_SIZE_MASK 0x000000000001fffcl

#else

#define LABEL_MASK       0xfffffffcl
#define MINREG_MASK      0xfffffffel
#define REGION_MASK      0xfffffffdl
#define PIXREG_MASK      0xfffffffcl
#define REGION_SIZE_MASK 0x0001fffcl

#endif

#define LABELPTR_MASK 0x3l
#define MINREG_FLAG 0x00000001l
#define REGION_FLAG 0x00000002l
#define PIXREG_FLAG 0x00000003l
#define REGION_SIZE_SHIFT 2
#define BORDER_SIZE_SHIFT 17

namespace extrema
{

  extern t_thresh_par g_thresh_params;
  extern int g_cols;
#pragma omp threadprivate (g_cols,g_thresh_params)

  void InitRegionRecycling();
  void DestRegionRecycling();
  t_LL GetExtrema(utls::BAry* img, t_sortpixels pixels, const ExtremaParams &par, bool invert);
  void DestRegions(t_LL regions);

}
#endif
