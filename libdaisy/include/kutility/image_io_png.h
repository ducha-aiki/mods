#ifdef WITH_PNG

#ifndef KUTILITY_IMAGE_IO_PNG_H
#define KUTILITY_IMAGE_IO_PNG_H

extern "C" {
#include "png.h"
}

#include "kutility/kutility.def"

namespace kutility
{
   int  load_png(const char* file_name, uchar* &body, int &h, int &w, int &ch);
   void save_png(const char* file_name, uchar* body, int height, int width, int chl);
}

typedef struct _write_png_info
{
   double gamma;
   long width;
   long height;
   time_t modtime;
   FILE *infile;
   FILE *outfile;
   void *png_ptr;
   void *info_ptr;
   uchar *image_data;
   uchar **row_pointers;
   char *title;
   char *author;
   char *desc;
   char *copyright;
   char *email;
   char *url;
   int channel_no;
   int filter;    /* command-line-filter flag, not PNG row filter! */
   // int pnmtype;
   int sample_depth;
   int interlaced;
   int have_time;
   jmp_buf jmpbuf;
   uchar bg_red;
   uchar bg_green;
   uchar bg_blue;
} write_png_info;

void wpng_cleanup(write_png_info* a);

void writepng_version_info ();
int  writepng_init         (write_png_info *png_ptr);
int  writepng_encode_image (write_png_info *png_ptr);
int  writepng_encode_row   (write_png_info *png_ptr);
int  writepng_encode_finish(write_png_info *png_ptr);
void writepng_cleanup      (write_png_info *png_ptr);

#endif

#endif
