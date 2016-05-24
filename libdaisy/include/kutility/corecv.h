#ifndef KUTILITY_CORECV_H
#define KUTILITY_CORECV_H

#include "math.h"

namespace kutility
{
   void point_transform_via_homography( double* H, double x, double y, double &u, double &v );

   double epipolar_line_slope( double y, double x, double* F );


}



#endif
