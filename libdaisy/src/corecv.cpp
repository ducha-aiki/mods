#include "corecv.h"

namespace kutility
{
    /// transform a point via the homography
   void point_transform_via_homography( double* H, double x, double y, double &u, double &v )
   {
      double kxp = H[0]*x + H[1]*y + H[2];
      double kyp = H[3]*x + H[4]*y + H[5];
      double kp  = H[6]*x + H[7]*y + H[8];
      u = kxp / kp;
      v = kyp / kp;
   }

   double epipolar_line_slope( double y, double x, double* F )
   {
      double line[3];
      line[0] = F[0]*x + F[1]*y + F[2];
      line[1] = F[3]*x + F[4]*y + F[5];
      line[2] = F[6]*x + F[7]*y + F[8];

      double m = -line[0]/line[1];
      double slope = atan( m )*180.0 / 3.1415926;

      if( slope <    0 ) slope += 360;
      if( slope >= 360 ) slope -= 360;

      return slope;
   }

}
