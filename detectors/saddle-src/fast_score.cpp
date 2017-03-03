/* This is FAST corner detector, contributed to OpenCV by the author, Edward Rosten.
   Below is the original copyright and the references */

/*
Copyright (c) 2006, 2008 Edward Rosten
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

    *Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

    *Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

    *Neither the name of the University of Cambridge nor the names of
     its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
The references are:
 * Machine learning for high-speed corner detection,
   E. Rosten and T. Drummond, ECCV 2006
 * Faster and better: A machine learning approach to corner detection
   E. Rosten, R. Porter and T. Drummond, PAMI, 2009
*/

#include "fast_score.hpp"
#include "stdio.h"
#include "sorb.h"

#define VERIFY_CORNERS 0

namespace cmp {

void makeOffsets(int pixel[25], int rowStride, int patternSize)
{
    static const int offsets16[][2] =
    {
        {0,  3}, { 1,  3}, { 2,  2}, { 3,  1}, { 3, 0}, { 3, -1}, { 2, -2}, { 1, -3},
        {0, -3}, {-1, -3}, {-2, -2}, {-3, -1}, {-3, 0}, {-3,  1}, {-2,  2}, {-1,  3}
    };

    static const int offsets12[][2] =
    {
        {0,  2}, { 1,  2}, { 2,  1}, { 2, 0}, { 2, -1}, { 1, -2},
        {0, -2}, {-1, -2}, {-2, -1}, {-2, 0}, {-2,  1}, {-1,  2}
    };

    static const int offsets8[][2] =
    {
        {0,  1}, { 1,  1}, { 1, 0}, { 1, -1},
        {0, -1}, {-1, -1}, {-1, 0}, {-1,  1}
    };

    const int (*offsets)[2] = patternSize == 16 ? offsets16 :
                              patternSize == 12 ? offsets12 :
                              patternSize == 8  ? offsets8  : 0;

    CV_Assert(pixel && offsets);

    int k = 0;
    for( ; k < patternSize; k++ )
        pixel[k] = offsets[k][0] + offsets[k][1] * rowStride;
    for( ; k < 25; k++ )
        pixel[k] = pixel[k - patternSize];
}

#if VERIFY_CORNERS
static void testCorner(const uchar* ptr, const int pixel[], int K, int N, int threshold) {
    // check that with the computed "threshold" the pixel is still a corner
    // and that with the increased-by-1 "threshold" the pixel is not a corner anymore
    for( int delta = 0; delta <= 1; delta++ )
    {
        int v0 = std::min(ptr[0] + threshold + delta, 255);
        int v1 = std::max(ptr[0] - threshold - delta, 0);
        int c0 = 0, c1 = 0;

        for( int k = 0; k < N; k++ )
        {
            int x = ptr[pixel[k]];
            if(x > v0)
            {
                if( ++c0 > K )
                    break;
                c1 = 0;
            }
            else if( x < v1 )
            {
                if( ++c1 > K )
                    break;
                c0 = 0;
            }
            else
            {
                c0 = c1 = 0;
            }
        }
        CV_Assert( (delta == 0 && std::max(c0, c1) > K) ||
                   (delta == 1 && std::max(c0, c1) <= K) );
    }
}
#endif

template<>
int cornerScore<16>(const uchar* ptr, const int pixel[], int threshold)
{
    const int K = 8, N = K*3 + 1;
    int k, v = ptr[0];
    short d[N];
    for( k = 0; k < N; k++ )
        d[k] = (short)(v - ptr[pixel[k]]);

#if CV_SSE2
    __m128i q0 = _mm_set1_epi16(-1000), q1 = _mm_set1_epi16(1000);
    for( k = 0; k < 16; k += 8 )
    {
        __m128i v0 = _mm_loadu_si128((__m128i*)(d+k+1));
        __m128i v1 = _mm_loadu_si128((__m128i*)(d+k+2));
        __m128i a = _mm_min_epi16(v0, v1);
        __m128i b = _mm_max_epi16(v0, v1);
        v0 = _mm_loadu_si128((__m128i*)(d+k+3));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+4));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+5));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+6));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+7));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+8));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k));
        q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
        q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
        v0 = _mm_loadu_si128((__m128i*)(d+k+9));
        q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
        q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
    }
    q0 = _mm_max_epi16(q0, _mm_sub_epi16(_mm_setzero_si128(), q1));
    q0 = _mm_max_epi16(q0, _mm_unpackhi_epi64(q0, q0));
    q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 4));
    q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 2));
    threshold = (short)_mm_cvtsi128_si32(q0) - 1;
#else
    int a0 = threshold;
    for( k = 0; k < 16; k += 2 )
    {
        int a = std::min((int)d[k+1], (int)d[k+2]);
        a = std::min(a, (int)d[k+3]);
        if( a <= a0 )
            continue;
        a = std::min(a, (int)d[k+4]);
        a = std::min(a, (int)d[k+5]);
        a = std::min(a, (int)d[k+6]);
        a = std::min(a, (int)d[k+7]);
        a = std::min(a, (int)d[k+8]);
        a0 = std::max(a0, std::min(a, (int)d[k]));
        a0 = std::max(a0, std::min(a, (int)d[k+9]));
    }

    int b0 = -a0;
    for( k = 0; k < 16; k += 2 )
    {
        int b = std::max((int)d[k+1], (int)d[k+2]);
        b = std::max(b, (int)d[k+3]);
        b = std::max(b, (int)d[k+4]);
        b = std::max(b, (int)d[k+5]);
        if( b >= b0 )
            continue;
        b = std::max(b, (int)d[k+6]);
        b = std::max(b, (int)d[k+7]);
        b = std::max(b, (int)d[k+8]);

        b0 = std::min(b0, std::max(b, (int)d[k]));
        b0 = std::min(b0, std::max(b, (int)d[k+9]));
    }

    threshold = -b0-1;
#endif

#if VERIFY_CORNERS
    testCorner(ptr, pixel, K, N, threshold);
#endif
    return threshold;
}

template<>
int cornerScore<12>(const uchar* ptr, const int pixel[], int threshold)
{
    const int K = 6, N = K*3 + 1;
    int k, v = ptr[0];
    short d[N + 4];
    for( k = 0; k < N; k++ )
        d[k] = (short)(v - ptr[pixel[k]]);
#if CV_SSE2
    for( k = 0; k < 4; k++ )
        d[N+k] = d[k];
#endif

#if CV_SSE2
    __m128i q0 = _mm_set1_epi16(-1000), q1 = _mm_set1_epi16(1000);
    for( k = 0; k < 16; k += 8 )
    {
        __m128i v0 = _mm_loadu_si128((__m128i*)(d+k+1));
        __m128i v1 = _mm_loadu_si128((__m128i*)(d+k+2));
        __m128i a = _mm_min_epi16(v0, v1);
        __m128i b = _mm_max_epi16(v0, v1);
        v0 = _mm_loadu_si128((__m128i*)(d+k+3));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+4));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+5));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k+6));
        a = _mm_min_epi16(a, v0);
        b = _mm_max_epi16(b, v0);
        v0 = _mm_loadu_si128((__m128i*)(d+k));
        q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
        q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
        v0 = _mm_loadu_si128((__m128i*)(d+k+7));
        q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
        q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
    }
    q0 = _mm_max_epi16(q0, _mm_sub_epi16(_mm_setzero_si128(), q1));
    q0 = _mm_max_epi16(q0, _mm_unpackhi_epi64(q0, q0));
    q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 4));
    q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 2));
    threshold = (short)_mm_cvtsi128_si32(q0) - 1;
#else
    int a0 = threshold;
    for( k = 0; k < 12; k += 2 )
    {
        int a = std::min((int)d[k+1], (int)d[k+2]);
        if( a <= a0 )
            continue;
        a = std::min(a, (int)d[k+3]);
        a = std::min(a, (int)d[k+4]);
        a = std::min(a, (int)d[k+5]);
        a = std::min(a, (int)d[k+6]);
        a0 = std::max(a0, std::min(a, (int)d[k]));
        a0 = std::max(a0, std::min(a, (int)d[k+7]));
    }

    int b0 = -a0;
    for( k = 0; k < 12; k += 2 )
    {
        int b = std::max((int)d[k+1], (int)d[k+2]);
        b = std::max(b, (int)d[k+3]);
        b = std::max(b, (int)d[k+4]);
        if( b >= b0 )
            continue;
        b = std::max(b, (int)d[k+5]);
        b = std::max(b, (int)d[k+6]);

        b0 = std::min(b0, std::max(b, (int)d[k]));
        b0 = std::min(b0, std::max(b, (int)d[k+7]));
    }

    threshold = -b0-1;
#endif

#if VERIFY_CORNERS
    testCorner(ptr, pixel, K, N, threshold);
#endif
    return threshold;
}

template<>
int cornerScore<8>(const uchar* ptr, const int pixel[], int threshold)
{
    const int K = 4, N = K*3 + 1;
    int k, v = ptr[0];
    short d[N];
    for( k = 0; k < N; k++ )
        d[k] = (short)(v - ptr[pixel[k]]);

#if CV_SSE2
    __m128i v0 = _mm_loadu_si128((__m128i*)(d+1));
    __m128i v1 = _mm_loadu_si128((__m128i*)(d+2));
    __m128i a = _mm_min_epi16(v0, v1);
    __m128i b = _mm_max_epi16(v0, v1);
    v0 = _mm_loadu_si128((__m128i*)(d+3));
    a = _mm_min_epi16(a, v0);
    b = _mm_max_epi16(b, v0);
    v0 = _mm_loadu_si128((__m128i*)(d+4));
    a = _mm_min_epi16(a, v0);
    b = _mm_max_epi16(b, v0);
    v0 = _mm_loadu_si128((__m128i*)(d));
    __m128i q0 = _mm_min_epi16(a, v0);
    __m128i q1 = _mm_max_epi16(b, v0);
    v0 = _mm_loadu_si128((__m128i*)(d+5));
    q0 = _mm_max_epi16(q0, _mm_min_epi16(a, v0));
    q1 = _mm_min_epi16(q1, _mm_max_epi16(b, v0));
    q0 = _mm_max_epi16(q0, _mm_sub_epi16(_mm_setzero_si128(), q1));
    q0 = _mm_max_epi16(q0, _mm_unpackhi_epi64(q0, q0));
    q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 4));
    q0 = _mm_max_epi16(q0, _mm_srli_si128(q0, 2));
    threshold = (short)_mm_cvtsi128_si32(q0) - 1;
#else
    int a0 = threshold;
    for( k = 0; k < 8; k += 2 )
    {
        int a = std::min((int)d[k+1], (int)d[k+2]);
        if( a <= a0 )
            continue;
        a = std::min(a, (int)d[k+3]);
        a = std::min(a, (int)d[k+4]);
        a0 = std::max(a0, std::min(a, (int)d[k]));
        a0 = std::max(a0, std::min(a, (int)d[k+5]));
    }

    int b0 = -a0;
    for( k = 0; k < 8; k += 2 )
    {
        int b = std::max((int)d[k+1], (int)d[k+2]);
        b = std::max(b, (int)d[k+3]);
        if( b >= b0 )
            continue;
        b = std::max(b, (int)d[k+4]);

        b0 = std::min(b0, std::max(b, (int)d[k]));
        b0 = std::min(b0, std::max(b, (int)d[k+5]));
    }

    threshold = -b0-1;
#endif

#if VERIFY_CORNERS
    testCorner(ptr, pixel, K, N, threshold);
#endif
    return threshold;
}


double saddleScore(const uchar* ptr, const int pixel[])
{
	// ptr and pixel maybe pointing to a blur image
	double Ixx = (double)ptr[pixel[2]] + (double)ptr[pixel[6]] - 2*(double)ptr[0];
	double Iyy = (double)ptr[pixel[0]] + (double)ptr[pixel[4]] - 2*(double)ptr[0];
	double Ixy = (double)ptr[pixel[0]] + (double)ptr[pixel[2]] + (double)ptr[pixel[4]] + (double)ptr[pixel[6]] +
				 0.5*((double)ptr[pixel[1]] + (double)ptr[pixel[3]] + (double)ptr[pixel[5]] + (double)ptr[pixel[7]]) -
				 6.0*(double)ptr[0];
	// Multiplied by -1 to invert the sign of the Hessian response
//	return (Ixx * Iyy - Ixy * Ixy) * (Ixx * Iyy - Ixy * Ixy);
	return (Ixx * Iyy - Ixy * Ixy) * -1.0;
}

double saddleScore2(const uchar* ptr, const int pixel[], double norm2)
{
	double Lxx = -(double)ptr[pixel[6]] + 2.0*(double)ptr[0] -(double)ptr[pixel[2]];
	double Lyy = -(double)ptr[pixel[4]] + 2.0*(double)ptr[0] - (double)ptr[pixel[0]];
	double Lxy = ((double)ptr[pixel[5]] - (double)ptr[pixel[7]] - (double)ptr[pixel[3]] + (double)ptr[pixel[1]])/4.0;
//	return (Lxx * Lyy - Lxy * Lxy) * (Lxx * Lyy - Lxy * Lxy) * norm2 * norm2;
	return (Lxx * Lyy - Lxy * Lxy) * -1.0 * norm2;
}

double cmpFeatureScore(const uchar* ptr, const int pixel[], const int* labels, double v, uchar delta, int scoreType)
{
	double greenredsum=0;
	double sum_sum = 0;
	int count = 0;

	switch (scoreType)
	{
		case SORB::ZERO_SCORE:

			return 0;
			break;

		case SORB::DELTA_SCORE:

			return (double)delta;
			break;

		case SORB::SUMOFABS_SCORE:

			for (int iElem=0; iElem<16; iElem++)
			{
				if (labels[iElem] != 0)
					greenredsum += abs((double)ptr[pixel[iElem]] - v);
			}
			return greenredsum;
			break;
///Add one relative one
	      case SORB::NORM_SCORE:

		      for (int iElem=0; iElem<16; iElem++)
		      {
			      if (labels[iElem] != 0){
				      //greenredsum += abs((double)ptr[pixel[iElem]] - v);
				      sum_sum += (double)ptr[pixel[iElem]];
				      count++;
				}
		      }
		      sum_sum = (sum_sum / (double) (count));;
		      return abs( v - sum_sum) / min(255., sum_sum + v);
		      break;
	  case SORB::HESS_SCORE:
		    return saddleScore(ptr,pixel);
		    break;

	      case SORB::AVGOFABS_SCORE:

			int greenrednum = 0;

			for (int iElem=0; iElem<16; iElem++)
			{
				if (labels[iElem] != 0)
				{
					greenredsum += abs((double)ptr[pixel[iElem]] - v);
					greenrednum++;
				}
			}
			return greenredsum/greenrednum;
			break;
	}
}


void print_m128i_epi8(__m128i var)
{
    uint8_t *val = (uint8_t*) &var;
    printf("Numerical: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i \n",
           val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15]);
}

} // namespace cv
