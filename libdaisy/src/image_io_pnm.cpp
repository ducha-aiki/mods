#include "kutility/image_io_pnm.h"
#define PNM_BUFFER_SIZE 256
// using namespace std;

namespace kutility
{
   void read_packed(unsigned char *data, int size, std::ifstream &f)
   {
      unsigned char c = 0;

      int bitshift = -1;
      for (int pos = 0; pos < size; pos++) {
         if (bitshift == -1) {
            c = f.get();
            bitshift = 7;
         }
         data[pos] = (c >> bitshift) & 1;
         bitshift--;
      }
   }
   void write_packed(unsigned char *data, int size, std::ofstream &f)
   {
      unsigned char c = 0;

      int bitshift = 7;
      for (int pos = 0; pos < size; pos++) {
         c = c | (data[pos] << bitshift);
         bitshift--;
         if ((bitshift == -1) || (pos == size-1)) {
            f.put(c);
            bitshift = 7;
            c = 0;
         }
      }
   }
   void pnm_read(std::ifstream &file, char *buf)
   {
      char doc[PNM_BUFFER_SIZE];
      char c;

      file >> c;
      while (c == '#') {
         file.getline(doc, PNM_BUFFER_SIZE);
         file >> c;
      }
      file.putback(c);

      file.width(PNM_BUFFER_SIZE);
      file >> buf;
      file.ignore();
   }
   void get_size_ppm(const char *name, int &height, int &width)
   {
      char buf[PNM_BUFFER_SIZE];
      //char doc[PNM_BUFFER_SIZE]
      // read header
      std::ifstream file(name, std::ios::in | std::ios::binary);
      pnm_read(file, buf);
      if (strncmp(buf, "P6", 2))
      {
         printf("type mismatch\n");
         exit(1);
      }

      pnm_read(file, buf);
      width = atoi(buf);

      pnm_read(file, buf);
      height = atoi(buf);

      file.close();
      return;
   }

   void load_pbm(const char* name, uchar* &im, int &height, int &width)
   {
      char buf[PNM_BUFFER_SIZE];

      /* read header */
      std::ifstream file(name, std::ios::in | std::ios::binary);
      pnm_read(file, buf);
      if (strncmp(buf, "P4", 2))
      {
         printf("type mismatch\n");
         exit(1);
      }

      pnm_read(file, buf);
      width = atoi(buf);

      pnm_read(file, buf);
      height = atoi(buf);

      /* read data */
      if( im != NULL) delete[]im;
      im = new uchar[width*height];
      for (int i = 0; i < height; i++)
         read_packed(im+(width*i), width, file);
   }
   void load_pgm(const char* name, uchar* &im, int &height, int& width)
   {
      char buf[PNM_BUFFER_SIZE];

      /* read header */
      std::ifstream file(name, std::ios::in | std::ios::binary);
      pnm_read(file, buf);
      if (strncmp(buf, "P5", 2))
      {
         printf("type mismatch\n");
         exit(1);
      }

      pnm_read(file, buf);
      width = atoi(buf);

      pnm_read(file, buf);
      height = atoi(buf);

      pnm_read(file, buf);
      if (atoi(buf) > UCHAR_MAX)
      {
         printf("type mismatch\n");
         exit(1);
      }

      /* read data */
      if( im != NULL ) delete[] im;
      im = new uchar[width*height];
      file.read( (char *)im, width * height * sizeof(uchar));
   }

   void load_ppm(const char* name, uchar* &im, int &height, int &width)
   {
      char buf[PNM_BUFFER_SIZE];
      //char doc[PNM_BUFFER_SIZE]
      std::ifstream file(name, std::ios::in | std::ios::binary);
      pnm_read(file, buf);
      if (strncmp(buf, "P6", 2))
      {
         printf("type mismatch\n");;
         exit(1);
      }
      pnm_read(file, buf);
      width = atoi(buf);

      pnm_read(file, buf);
      height = atoi(buf);

      pnm_read(file, buf);
      if (atoi(buf) > UCHAR_MAX)
      {
         printf("type mismatch\n");;
         exit(1);
      }

      /* read data */
      if( im != NULL ) delete[] im;
      im = new uchar[width*height*3];
      file.read((char *)im, width * height * 3 * sizeof(uchar));
   }

   void save_pbm(const char* name, uchar* im, int height, int width )
   {
      std::ofstream file(name, std::ios::out | std::ios::binary);

      file << "P4\n" << width << " " << height << "\n";
      for (int i = 0; i < height; i++)
         write_packed(im+(width*i), width, file);
   }


}
