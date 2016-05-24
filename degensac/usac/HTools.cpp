#include "HTools.h"

namespace HTools
{
//TODO move to MathTools
void computeDataMatrix(double* data_matrix, unsigned int num_points, double* points)
{
    // linearizes corresp. with respect to entries of homography matrix,
    // so that u' = H u -> A h

    const double *data_ptr;
    double *p;
    unsigned int offset = 2*num_points;

    for (unsigned int i = 0; i < num_points; ++i)
    {
        data_ptr = points + 6*i;
        p = data_matrix + 2*i;

        *p				= 0;
        *(p + offset)	= 0;
        *(p + 2*offset) = 0;
        *(p + 3*offset) = -data_ptr[0];
        *(p + 4*offset) = -data_ptr[1];
        *(p + 5*offset) = -data_ptr[2];
        *(p + 6*offset) = data_ptr[4] * data_ptr[0];
        *(p + 7*offset) = data_ptr[4] * data_ptr[1];
        *(p + 8*offset) = data_ptr[4] * data_ptr[2];

        p = data_matrix + 2*i + 1;

        *p				= data_ptr[0];
        *(p + offset)	= data_ptr[1];
        *(p + 2*offset) = data_ptr[2];
        *(p + 3*offset) = 0;
        *(p + 4*offset) = 0;
        *(p + 5*offset) = 0;
        *(p + 6*offset) = -data_ptr[3] * data_ptr[0];
        *(p + 7*offset) = -data_ptr[3] * data_ptr[1];
        *(p + 8*offset) = -data_ptr[3] * data_ptr[2];
    }
} // end computeDataMatrix

/* duplicity with MathTools
void crossprod(double *out, const double *a, const double *b, unsigned int st)
{
	unsigned int st2 = 2 * st;
	out[0] = a[st]*b[st2] - a[st2]*b[st];
	out[1] = a[st2]*b[0]  - a[0]*b[st2];
	out[2] = a[0]*b[st]   - a[st]*b[0];
}*/

}
