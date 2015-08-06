#ifndef KUTILITY_IMAGE_IO_PNM_H
#define KUTILITY_IMAGE_IO_PNM_H

#include <fstream>
#include "string.h"
#include <cstdlib>
#include "limits.h"

#ifndef uchar
typedef unsigned char uchar;
#endif

namespace kutility
{
   void load_pbm(const char* name, uchar* &data, int &height, int &width);
   void load_pgm(const char* name, uchar* &data, int &height, int &width);
   void load_ppm(const char* name, uchar* &data, int &height, int &width);

   void save_pbm(const char* name, uchar *im, int height, int width);

   template<class T>
   void save_pgm(const char* name, T *im, int height, int width)
   {
      std::ofstream file(name, std::ios::out | std::ios::binary);

      file << "P5\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";

      for( int k=0; k<width*height; k++ )
      {
         file <<(uchar)im[k];
      }
      file.close();
   }

   template<class T>
   void save_ppm(const char* name, T *im, int height, int width)
   {
      std::ofstream file(name, std::ios::out | std::ios::binary);

      file << "P6\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
      for( int k=0; k<3*width*height; k++ )
      {
         file <<(uchar)im[k];
      }
      file.close();
   }

   void get_size_ppm(const char* name, int &height, int &width);
}

#endif
