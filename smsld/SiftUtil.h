// SiftUtil.h -- common definitions and functions.

#ifndef SIFTUTIL_H
#define SIFTUTIL_H


#include <cmath>
#include <cstdlib>
#include <iostream>
#include <cassert>
#include <ctime>
#include <vector>

#pragma warning (disable: 4786)

using namespace std;

typedef unsigned char uchar;



template <class T>
inline void Swap(T &v1, T &v2)
{
	T temp = v1;
	v1 = v2;
	v2 = temp;
}

template <class T>
inline T Max(T x, T y)
{
	return (x > y) ? x : y;
}

template <class T>
inline T Min(T x, T y)
{
	return (x < y) ? x : y;
}


// Simple error handling
inline void FatalError(const char *msg)
{
	cerr << msg << endl;
	exit(1);
}


#endif
