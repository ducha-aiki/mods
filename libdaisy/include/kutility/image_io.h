#ifndef KUTILITY_IMAGE_IO_H
#define KUTILITY_IMAGE_IO_H

#include "kutility/image_io_bmp.h"
#include "kutility/image_io_pnm.h"

#ifdef WITH_JPEG
#include "kutility/image_io_jpeg.h"
#endif

#ifdef WITH_PNG
#include "kutility/image_io_png.h"
#endif

#include "kutility/image_manipulation.h"


//inline int load_image(string filename, uchar* &body, int &h, int &w, int &ch );
//inline int load_gray_image( string filename, uchar* &body, int &h, int &w );
//template<class T> inline int tload_image(string filename, T* &body, int& h, int& w, int& ch);
//template<class T> inline int tload_gray_image(string filename, T* &body, int& h, int& w);
//inline void save_image( string filename, uchar* body, int h, int w, int ch );
//template<class T> inline void tsave_image(string filename, T* body, int h, int w, int d);

namespace kutility
{
   inline int load_image(string filename, uchar* &body, int &h, int &w, int &ch )
   {
      string format = get_file_format( filename );
      if( !format.compare( "jpg" ) || !format.compare( "jpeg" ) )
      {
#ifdef WITH_JPEG
         return load_jpg(filename.c_str(), body, h, w, ch);
#else
         cout<<"cannot load jpeg file: "<<filename<<". compile library with WITH_JPEG\n";
         exit(1);
#endif
      }
      else if( !format.compare( "png" ) )
      {
#ifdef WITH_PNG
         return load_png(filename.c_str(), body, h, w, ch);
#else
         cout<<"cannot load png file: "<<filename<<". compile library with WITH_PNG\n";
         exit(1);
#endif
      }
      else if( !format.compare( "ppm" ) )
      {
         load_ppm(filename.c_str(), body, h, w); ch = 3;
         return 0;
      }
      else if( !format.compare( "pgm" ) )
      {
         load_pgm(filename.c_str(), body, h, w); ch = 1;
         return 0;
      }
      else
      {
         cout<<"unknown type: "<<format<<endl;
         return 1;
      }
   }
   inline int load_gray_image( string filename, uchar* &body, int &h, int &w )
   {
      int ch=0;
      if( !load_image(filename,body,h,w,ch) )
      {
         uchar* graydata = new uchar[h*w];
         if( ch != 1 )
         {
            rgb_to_y(body, h, w, graydata);
            delete []body; body = graydata;
         }
         return 0;
      }
      cout<<"could not load: load_gray_image"<<endl;
      return 1;
   }

   template<class T> inline int tload_image(string filename, T* &body, int& h, int& w, int& ch)
   {
      uchar* data = NULL;
      if( !load_image(filename, data, h, w, ch ) )
      {
         body = type_cast<T,uchar>(data, h*w*ch);
         delete []data;
         return 0;
      }
      cout<<"could not load: tload_image"<<endl;
      return 1;
   }
   template<class T> inline int tload_gray_image(string filename, T* &body, int& h, int& w)
   {
      uchar* data = NULL;
      if( !load_gray_image( filename, data, h, w ) )
      {
         body = type_cast<T,uchar>(data, h*w);
         delete []data;
         return 0;
      }
      cout<<"could not load: tload_gray_image"<<endl;
      return 1;
   }
   inline void save_image( string filename, uchar* body, int h, int w, int ch )
   {
      string format = get_file_format( filename );
      if( !format.compare("jpg") || !format.compare("jpeg") )
      {
#ifdef WITH_JPEG
         save_jpg(filename.c_str(), body, h, w, ch, 100);
#else
         cout<<"cannot save jpeg file: "<<filename<<". compile library with WITH_JPEG\n";
         exit(1);
#endif
      }
      else if( !format.compare("png") )
      {
#ifdef WITH_PNG
         save_png(filename.c_str(), body, h, w, ch);
#else
         cout<<"cannot save png file: "<<filename<<". compile library with WITH_PNG\n";
         exit(1);
#endif
      }
      else if( !format.compare("ppm") )
      {
         if( ch == 1 )
         {
            warning("image is grayscale. might wanna save it pgm");
            uchar* rgbdata = new uchar[h*w*3];
            y_to_rgb(body, h, w, rgbdata);
            save_ppm(filename.c_str(), rgbdata, h, w );
            delete []rgbdata;
         }
         else
            save_ppm(filename.c_str(), body, h, w );
      }
      else if( !format.compare("pgm") )
      {
         if( ch == 3 )
         {
            warning("image is colored. might wanna save it ppm");
            uchar* graydata = new uchar[h*w];
            rgb_to_y(body, h, w, graydata);
            save_pgm(filename.c_str(), graydata, h, w);
            delete []graydata;
         }
         else
            save_pgm(filename.c_str(), body, h, w);
      }
      else if( !format.compare("bmp") )
      {
         save_bmp(filename.c_str(), body, h, w, ch );
      }
      else
      {
         warning("unknown format:", format);
         exit(1);
      }
   }
   template<class T> inline void tsave_image(string filename, T* body, int h, int w, int d)
   {
      uchar* tdata = type_cast<uchar, T>(body, h*w*d);
      save_image(filename, tdata, h, w, d);
      delete []tdata;
   }

}

#endif
