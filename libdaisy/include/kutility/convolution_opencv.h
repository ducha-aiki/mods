#ifndef KUTILITY_CONVOLUTION_OPENCV_TCC
#define KUTILITY_CONVOLUTION_OPENCV_TCC

#if defined(WITH_OPENCV) && defined(WITH_OPENCV_EXTRAS)

#include "cv.h"
#include "highgui.h"

namespace kutility
{
   inline void conv_horizontal( float* image, int h, int w, float* kernel, int ksize )
   {
      CvMat cvI; cvInitMatHeader(&cvI, h, w, CV_32FC1, (float*)image);
      CvMat cvK; cvInitMatHeader(&cvK, 1, ksize, CV_32FC1, (float*)kernel );
      cvFilter2D( &cvI, &cvI, &cvK );
   }
   inline void conv_horizontal( double* image, int h, int w, double* kernel, int ksize )
   {
      CvMat cvI; cvInitMatHeader(&cvI, h, w, CV_64FC1, (double*)image);
      CvMat cvK; cvInitMatHeader(&cvK, 1, ksize, CV_64FC1, (double*)kernel );
      cvFilter2D( &cvI, &cvI, &cvK );
   }

   inline void conv_vertical( float* image, int h, int w, float* kernel, int ksize )
   {
      CvMat cvI; cvInitMatHeader(&cvI, h, w, CV_32FC1, (float*)image);
      CvMat cvK; cvInitMatHeader(&cvK, ksize, 1, CV_32FC1, (float*)kernel );
      cvFilter2D( &cvI, &cvI, &cvK );
   }

   inline void conv_vertical( double* image, int h, int w, double* kernel, int ksize )
   {
      CvMat cvI; cvInitMatHeader(&cvI, h, w, CV_64FC1, (double*)image);
      CvMat cvK; cvInitMatHeader(&cvK, ksize, 1, CV_64FC1, (double*)kernel );
      cvFilter2D( &cvI, &cvI, &cvK );
   }

   template<typename T> inline
   void convolve_sym_( T* image, int h, int w, T* kernel, int ksize )
   {
      conv_horizontal( image, h, w, kernel, ksize );
      conv_vertical  ( image, h, w, kernel, ksize );
   }
}

#endif

#endif
