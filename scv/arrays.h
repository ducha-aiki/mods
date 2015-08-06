#ifndef ARRAYS_HPP
#define ARRAYS_HPP

#include <cstdio>
/*
This is free library for a non-commercial purpose. Commercial use
requires a licence. The software is distributed AS IS, WITHOUT ANY
IMPLIED OR EXPRESSED WARRANTY and WITHOUT ANY FURTHER SUPPORT for
the desired and/or other purpose.
(c) Jan Cech (cechj@cmp.felk.cvut.cz) FEE CTU Prague, Apr 30, 2009
*/
/*

arrays.h
--------
Simple library for convenient handling with 2D arrays.


template<class T>
class ARRAY_2D {
 public:
   T* data; //pointer to the data (linear array in memory)
   int M,N;      //height, width
   bool need_to_release; //1 if memory must be released,
                         //0 otherwise (released by matlab)

   ARRAY_2D<T>(); //construct empty 2D array
   ARRAY_2D<T>(const mxArray* D); //construct 2D array from mxArray
   ARRAY_2D<T>(const mxArray* D, cmlxFLAG); //construct array from cmplx-mxArray
   ARRAY_2D<T>(int height, int width);//construct new 2D array(given dimensions)

   void assign(const mxArray* D); //assign a mxArray to empty array(pointer op.)
   void assignI(const mxArray* D);//assign IMAGINARY part mxArray to empty array
   void assign(ARRAY_2D<T>& D); //assign an array to empty array(pointer op.)
   void copy(ARRAY_2D<T>& D); //copy the content of the array to current array
   mxArray *convert2mx(); //convert an array to a mxArray

   T& i(const int& ind); //linear index (one based)
   T& operator[](const int& ind); //operator [] (same as linear index)
   T& s(const int& r, const int& c); //2D index (one based)
   int sub2ind(const int& r, const int& c); //convert subscript to linear index

   void clear(); //clears the array and release the memory if needed
   ~ARRAY_2D(); //destructor
}

 example:
   ARRAY_2D<double> A(prhs[0]); //2D array constructed from mxArray
   ARRAY_2D<int> B(100,250);    //2D array constructed defining dimensions
   ARRAY_2D<double> C;          //2D array empty

   A.i(100) = 1;  //linear one-based indexing
   A[100] = 1;    //equivalent command
   A.data[100-1] = 1; //another equivalent option (A.data is the direct pointer)
   mexPrintf("%f",A[100]); //writes 1.000000
   A.s(10,5) = 2; //(2D) subscript one-based index
   mexPrintf("%f",A.s(10,5)); //writes 2.000000

   B.M //number of rows: 100
   B.N //number of columns: 250

   plhs[0] = mxCreateDoubleMatrix(3,3,mxREAL);
   C.assign(plhs[0]); //same as ARRAY_2D<double> C(plhs[0]);

   ARRAY_2D<double> D(5,5);
   ARRAY_2D<double> E(5,5);
   mxArray *M;
   D.copy(E); //copy the content of E to D
   M = D.convert2mx(); //convert D to mxArray M (copy content)
   mexCallMATLAB(0,NULL,1,&M,"disp"); //display in Matlab enviroment
   plhs[1] = E.convert2mx(); //output of the array E into matlab

   //complex matrices
   plhs[0] = mxCreateDoubleMatrix(R.M,R.N,mxCOMPLEX);
   ARRAY_2D<double> Cr(plhs[0],REAL);
   ARRAY_2D<double> Ci(plhs[0],IMAG);
   //Cr.assign(plhs[0]); //equivalent
   //Ci.assignI(plhs[0]);

template<class T>
class ARRAY_3D {...}; //analogical functionality as ARRAY_2D


*/

enum cmplxFLAG {REAL, IMAG};

template<class T>
class ARRAY_2D
{
public:
    T* data; //pointer to the data (linear array in memory)
    int M,N;      //height, width
    bool need_to_release; //1 if memory must be released,
    //0 otherwise (released by matlab)

    ARRAY_2D<T>()   //construct empty 2D array
    {
        M = 0;
        N = 0;
        data = NULL;
        need_to_release = 0;
    }

    ARRAY_2D<T>(int height, int width, T* ptr)  //construct 2D array from ptr
    {
        M = height;
        N = width;
        data = ptr;
        need_to_release = 0;
    }
    /*  ARRAY_2D<T>(const mxArray* D){ //construct 2D array from mxArray
         M = mxGetM(D);
         N = mxGetN(D);
         data = mxGetPr(D);
         need_to_release = 0;
      }
                                         //construct 2D array from real/imag mxArray
      ARRAY_2D<T>(const mxArray* D, cmplxFLAG cf){
         M = mxGetM(D);
         N = mxGetN(D);
         need_to_release = 0;

         if (cf==0) {
            data = mxGetPr(D);
         }
         else {
            data = mxGetPi(D);
         };
      }
    */
    ARRAY_2D<T>(int height, int width)  //construct new 2D array(given dimensions)
    {
        M = height;
        N = width;
        data = new T[M*N];
        need_to_release = 1;
    }

    /*  void assign(const mxArray* D){ //assign a mxArray to empty array
         if (data!=NULL)  mexErrMsgTxt("Only empty array can be assigned.");
         M = mxGetM(D);
         N = mxGetN(D);
         data = mxGetPr(D);
         need_to_release = 0;
      }

      void assignI(const mxArray* D){ //assign IMAGINARY part mxArray to empty array
         if (data!=NULL)  mexErrMsgTxt("Only empty array can be assigned.");
         M = mxGetM(D);
         N = mxGetN(D);
         data = mxGetPi(D);
         need_to_release = 0;
      }
    */
    void assign(ARRAY_2D<T>& D)   //assign an array to empty array
    {
        //   if (data!=NULL)  mexErrMsgTxt("Only empty array can be assigned.");
        //   int i;
        M = D.M;
        N = D.N;
        //  data = new T[M*N];
        //  for (i=0;i<M*N;i++) { //Mishkin
        //      data[i] = D.data[i];
        //    }
        //     need_to_release = 1;

        data = D.data;
        need_to_release = 0; //the release is left on the original array
    }

    void copy(ARRAY_2D<T>& D)   //copy the content of the array to current array
    {
        int i;
        assert (M==D.M & N==D.N);
        for (i=0; i<M*N; i++)
        {
            data[i] = D.data[i];
        }
    }

    /* mxArray* convert2mx() { //convert an array to a mxArray (copy content)
       mxArray *tmp;
       double *d;
       int i;
       tmp = mxCreateDoubleMatrix(M, N, mxREAL);
       d=mxGetPr(tmp);
       for (i=0; i<M*N; i++) {
         d[i] = data[i];
       }
       return tmp;
     }
    */
    T& i(const int& ind)  //linear index (one based)
    {
        return data[ind-1];
    }

    T& operator[](const int& ind)  //operator [] (same as linear index)
    {
        return data[ind-1];
    }

    T& s(const int& r, const int& c)  //2D index (one based)
    {
        return data[(c-1)*M+(r-1)];
    }

    int sub2ind(const int& r, const int& c)  //convert subscript to linear index
    {
        return (c-1)*M+(r);
    }

    void clear()   //clears the array and release the memory if needed
    {
        if (need_to_release)
        {
            delete [] data;
            //mexPrintf("Memory released (%ix%i).\n",M,N);
            need_to_release = 0;
        };
        data = NULL;
        M=0;
        N=0;
    }

    ~ARRAY_2D()   //destructor
    {
        clear();
        //mexPrintf("Done.");
    }


};

// ---------------------------------------------------------------------------

template<class T>
class ARRAY_3D
{
public:
    T* data; //pointer to the data (linear array in memory)
    int M,N,O;      //height, width
    bool need_to_release; //1 if memory must be released,
    //0 otherwise (released by matlab)

    ARRAY_3D<T>()   //construct empty 2D array
    {
        M = 0;
        N = 0;
        O = 0;
        data = NULL;
        need_to_release = 0;
    }
    /*
      ARRAY_3D<T>(const mxArray* D){ //construct 2D array from mxArray
         const int *tmp;
         tmp = mxGetDimensions(D);
         M = tmp[0];
         N = tmp[1];
         O = tmp[2];
         data = mxGetPr(D);
         need_to_release = 0;
      }
                                         //construct 2D array from real/imag mxArray
      ARRAY_3D<T>(const mxArray* D, cmplxFLAG cf){
         const int *tmp;
         tmp = mxGetDimensions(D);
         M = tmp[0];
         N = tmp[1];
         O = tmp[2];
         need_to_release = 0;

         if (cf==0) {
            data = mxGetPr(D);
         }
         else {
            data = mxGetPi(D);
         };
      }
    */
    ARRAY_3D<T>(int height, int width, int depth)  //construct new 2D array(given dimensions)
    {
        M = height;
        N = width;
        O = depth;
        data = new double[M*N*O];
        need_to_release = 1;
    }
    /*
      void assign(const mxArray* D){ //assign a mxArray to empty array
         int *tmp;
         if (data!=NULL)  mexErrMsgTxt("Only empty array can be assigned.");
         tmp = mxGetDimensions(D);
         M = tmp[0];
         N = tmp[1];
         O = tmp[2];
         data = mxGetPr(D);
         need_to_release = 0;
      }

      void assignI(const mxArray* D){ //assign IMAGINARY part mxArray to empty array
         int *tmp;
         if (data!=NULL)  mexErrMsgTxt("Only empty array can be assigned.");
         tmp = mxGetDimensions(D);
         M = tmp[0];
         N = tmp[1];
         O = tmp[2];
         data = mxGetPi(D);
         need_to_release = 0;
      }
    */
    void assign(ARRAY_3D<T>& D)   //assign an array to empty array
    {
        //   if (data!=NULL)  mexErrMsgTxt("Only empty array can be assigned.");
        M = D.M;
        N = D.N;
        O = D.O;
        data = D.data;
        need_to_release = 0; //the release is left on the original array
    }

    void copy(ARRAY_3D<T>& D)   //copy the content of the array to current array
    {
        int i;
        if (M==D.M & N==D.N & O==D.O)
        {
            for (i=0; i<M*N*O; i++)
            {
                data[i] = D.data[i];
            }
        }
        else
        {
            //    mexErrMsgTxt("Array dimensions must agree.");
        };
    }
    /*
      mxArray* convert2mx() { //convert an array to a mxArray (copy content)
        mxArray *tmp;
        double *d;
        int i;
        const int dims[3]={M,N,O};
        tmp = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
        d=mxGetPr(tmp);
        for (i=0; i<M*N*O; i++) {
          d[i] = data[i];
        }
        return tmp;
      }
    */
    T& i(const int& ind)  //linear index (one based)
    {
        return data[ind-1];
    }

    T& operator[](const int& ind)  //operator [] (same as linear index)
    {
        return data[ind-1];
    }

    T& s(const int& m, const int& n, const int& o)  //2D index (one based)
    {
        return data[(o-1)*M*N+(n-1)*M+(m-1)];
    }

    int sub2ind(const int& m, const int& n, const int& o)  //convert subscript to linear index
    {
        return (o-1)*M*N+(n-1)*M+(m);
    }

    void clear()   //clears the array and release the memory if needed
    {
        if (need_to_release)
        {
            delete [] data;
            //mexPrintf("Memory released (%ix%i).\n",M,N);
            need_to_release = 0;
        };
        data = NULL;
        M=0;
        N=0;
    }

    ~ARRAY_3D()   //destructor
    {
        clear();
        //mexPrintf("Done.");
    }
};


#endif // ARRAYS_HPP
