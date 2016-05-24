#ifndef KUTILITY_CONVOLUTION_DEFAULT_TCC
#define KUTILITY_CONVOLUTION_DEFAULT_TCC

namespace kutility
{

/// do not call directly. use through conv_horizontal and conv_vertical
   template<class T1, class T2> inline
   void conv_buffer_(T1* buffer, T2* kernel, int rsize, int ksize)
   {
      for ( int i=0; i<rsize; i++ )
      {
         float sum = 0;
         for( int j=0; j<ksize; j++ )
         {
            sum += buffer[i+j] * kernel[j];
         }
         buffer[i]=sum;
      }
   }

   template<class T1, class T2> inline
   void conv_horizontal(T1* image, int h, int w, T2 *kernel, int ksize)
   {
      int halfsize = ksize / 2;
      assert(w + ksize < 4096);

      T1  buffer[4096];
      for( int r=0; r<h; r++)
      {
         int rw = r*w;

         for( int i=0; i<halfsize; i++)
            buffer[i] = image[rw];

         memcpy( &(buffer[halfsize]), &(image[rw]), w*sizeof(T1) );

         T1 temp = image[rw+w-1];
         for( int i=0; i<halfsize; i++)
            buffer[i+halfsize+w] = temp;

         conv_buffer_(buffer, kernel, w, ksize);

         for( int c=0; c<w; c++)
            image[rw+c] = buffer[c];
      }
   }

   template<class T1, class T2> inline
   void conv_vertical(T1* image, int h, int w, T2 *kernel, int ksize)
   {
      T1  buffer[4096];

      int halfsize = ksize / 2;
      assert(h + ksize < 4096);

      int h_1w = (h-1)*w;

      for( int c=0; c<w; c++)
      {
         for(int i=0; i<halfsize; i++)
            buffer[i] = image[c];

         for( int i=0; i<h; i++)
            buffer[halfsize+i] = image[i*w+c];

         for( int i=0; i<halfsize; i++)
            buffer[halfsize+i+h] = image[h_1w+c];

         conv_buffer_(buffer, kernel, h, ksize);

         for(int r=0; r<h; r++ )
         {
            image[r*w+c] = buffer[r];
         }
      }
   }

   template<typename T> inline
   void convolve_sym_( T* image, int h, int w, T* kernel, int ksize )
   {
      conv_horizontal( image, h, w, kernel, ksize );
      conv_vertical  ( image, h, w, kernel, ksize );
   }
}

#endif
