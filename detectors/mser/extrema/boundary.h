/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#ifndef __BOUNDARY_H__
#define __BOUNDARY_H__

#include <ary.h>
#include <LL.h>
#include "extremaTypes.h"

namespace extrema
{
  void RegionBoundaries(utls::BAry *img, t_LL regions);
}
#endif
