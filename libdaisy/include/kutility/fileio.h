#ifndef KUTILITY_FILEIO_H
#define KUTILITY_FILEIO_H

#include "kutility/general.h"

namespace kutility
{
   enum data_types {TYPE_CHAR, TYPE_FLOAT, TYPE_DOUBLE, TYPE_INT};

   // ascii
   template<typename T> inline void save_ascii( ofstream& fout, T* data, int h, int w, int nb, int type );
   template<typename T> inline void save_ascii( string filename, T* data, int h, int w, int nb, int type );
   template<typename T> inline void load_ascii( ifstream& fin, T* &data, int &h, int &w, int &nb );
   template<typename T> inline void load_ascii( string filename, T* &data, int &h, int &w, int &nb );

   // binary
   template<class T> inline void save_binary(ofstream& fout, T* data, int h, int w, int nb, int type );
   template<class T> inline int  save_binary(string filename, T* data, int h, int w, int nb, int type );
   inline int load_binary(ifstream &fin, float*  &data, int &h, int &w, int &nb );
   inline int load_binary(ifstream &fin, int*    &data, int &h, int &w, int &nb );
   inline int load_binary(ifstream &fin, double* &data, int &h, int &w, int &nb );
   inline int load_binary(ifstream &fin, char*   &data, int &h, int &w, int &nb );
   template<typename T> inline int load_binary(string filename, T* &data, int &h, int &w, int &nb );

   template<class T> inline void save_plain(ofstream& fout, T* data, int sz );
   template<class T> inline void save_plain(ofstream& fout, T* data, int rs, int cs );
   template<class T> inline void save(string filename, T* data, int sz );
   template<class T> inline void save(string filename, T* data, int rs, int cs);

   template<class T> inline int load( ifstream& fin, T* &out, int size=-1 );
   template<class T> inline int load( string filename, T* &out, int size=-1 );
   inline void* load_array( string filename, int size, int type=1 );

   #include "fileio.tcc"
}

#endif

