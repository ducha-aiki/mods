/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#ifndef __SORT_PIXELS_H__
#define __SORT_PIXELS_H__

#include <ary.h>
#include "extremaTypes.h"

namespace extrema
{

  int ReplaceExtWithSecond(utls::BAry * &img);
  void CalcHistogram(utls::BAry * &img, t_sortpixels &pixels);
  void BinSortPixels(utls::BAry * &img, t_sortpixels &pixels);
  void InvertImageAndHistogram(utls::BAry *img, t_sortpixels &pixels);

}
#endif
