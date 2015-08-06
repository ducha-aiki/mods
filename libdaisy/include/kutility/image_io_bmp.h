#ifndef KUTILITY_IMAGE_IO_BMP_H
#define KUTILITY_IMAGE_IO_BMP_H

#if defined(WIN32)
#pragma warning( disable : 4996 )
#endif

#include <fstream>

using std::string;

#ifndef uchar
typedef unsigned char uchar;
#endif

namespace kutility
{
   ///  converts an integer number to a hex string.
   inline void convert_hex(int number, int* hex_array)
   {
      for(int i=0; i<4; i++)
      {
         hex_array[i] = number%256;
         number = number/256;
      }
   }

   // void savebmp(string str, uchar* body, int h, int w, int channel);
   void save_bmp(const char* str, uchar* body, int h, int w, int channel);

}

#endif
