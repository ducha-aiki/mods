#ifdef WITH_JPEG

#ifndef KUTILITY_IMAGE_IO_JPEG
#define KUTILITY_IMAGE_IO_JPEG

#include <stdio.h>
extern "C" {
#include "jpeglib.h"
}
#include <cstdlib>
#include <string>

#ifndef uchar
typedef unsigned char uchar;
#endif

using std::string;

void save_jpg(const char* filename, uchar* body, int h, int w, int ch, int quality);
int  load_jpg(const char* filename, uchar* &body, int &h, int &w, int &ch);

#endif

#endif
