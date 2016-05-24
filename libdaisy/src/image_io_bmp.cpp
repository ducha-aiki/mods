#include "kutility/image_io_bmp.h"

namespace kutility
{
   void save_bmp(const char* str, uchar* body, int h, int w, int channel)
   {
      int image_data_size = w * h;

      // if( channel == 3 )
      image_data_size *= 4;

      int hexWidth[4];
      int hexHeight[4];
      int hexFileSize[4];
      int hexIdent[4];

      convert_hex(w, hexWidth);
      convert_hex(h, hexHeight);
      convert_hex(image_data_size+54,hexFileSize);
      convert_hex(image_data_size,hexIdent);

      FILE * maskFile  = fopen( str , "w+b");

      char headerArray[54];
      headerArray[0] =(char)0x42 ;
      headerArray[1] =(char)0x4D ;
      headerArray[2] =(char)hexFileSize[0] ;
      headerArray[3] =(char)hexFileSize[1] ;
      headerArray[4] =(char)hexFileSize[2] ;
      headerArray[5] =(char)hexFileSize[3] ;
      headerArray[6] = (char)0x0;
      headerArray[7] = (char)0x0;
      headerArray[8] = (char)0x0;
      headerArray[9] = (char)0x0;
      headerArray[10] = (char)0x36;
      headerArray[11] = (char)0x0;
      headerArray[12] = (char)0x0;
      headerArray[13] = (char)0x0;
      headerArray[14] = (char)0x28;
      headerArray[15] = (char)0x0;
      headerArray[16] = (char)0x0;
      headerArray[17] = (char)0x0;
      headerArray[18] = (char)hexWidth[0];
      headerArray[19] = (char)hexWidth[1];
      headerArray[20] = (char)hexWidth[2];
      headerArray[21] = (char)hexWidth[3];
      headerArray[22] = (char)hexHeight[0];
      headerArray[23] = (char)hexHeight[1];
      headerArray[24] = (char)hexHeight[2];
      headerArray[25] = (char)hexHeight[3];
      headerArray[26] = (char)0x01;
      headerArray[27] = (char)0x0;
      headerArray[28] = (char)0x20;
      headerArray[29] = (char)0x0;
      headerArray[30] = (char)0x0;
      headerArray[31] = (char)0x0;
      headerArray[32] = (char)0x0;
      headerArray[33] = (char)0x0;
      headerArray[34] = (char)hexIdent[0];
      headerArray[35] = (char)hexIdent[1];
      headerArray[36] = (char)hexIdent[2];
      headerArray[37] = (char)hexIdent[3];
      headerArray[38] = (char)0xC4;
      headerArray[39] = (char)0x0E;
      headerArray[40] = (char)0x0;
      headerArray[41] = (char)0x0;
      headerArray[42] = (char)0xC4;
      headerArray[43] = (char)0x0E;
      headerArray[44] = (char)0x0;
      headerArray[45] = (char)0x0;
      headerArray[46] = (char)0x0;
      headerArray[47] = (char)0x0;
      headerArray[48] = (char)0x0;
      headerArray[49] = (char)0x0;
      headerArray[50] = (char)0x0;
      headerArray[51] = (char)0x0;
      headerArray[52] = (char)0x0;
      headerArray[53] = (char)0x0;

      fwrite(headerArray, sizeof(char), 54, maskFile);
      fclose(maskFile);
      maskFile  = fopen( str , "a+b");

      uchar* data = new uchar[image_data_size];

      int index=0;
      //create bitmap data//
      for(int m=0; m<h; m++)
      {
         for(int n=0; n<w; n++)
         {
            index   = m*w+n;
            int indexM  = (h-m-1)*w+n;

            if( channel == 3 )
            {
               data[4*indexM  ] = (uchar)(body[3*index  ]);
               data[4*indexM+1] = (uchar)(body[3*index+1]);
               data[4*indexM+2] = (uchar)(body[3*index+2]);
               data[4*indexM+3] = 0;
            }
            else if( channel == 1 )
            {
               data[4*indexM  ] = (uchar)(body[index]);
               data[4*indexM+1] = (uchar)(body[index]);
               data[4*indexM+2] = (uchar)(body[index]);
               data[4*indexM+3] = 0;
            }
         }
      }
      fwrite(data, sizeof(char), image_data_size, maskFile);
      fclose(maskFile);

      delete []data;
      data = NULL;
   }

}
