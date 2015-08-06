#ifndef KUTILITY_DEF_H
#define KUTILITY_DEF_H

#include <vector>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include "assert.h"
#include <string>
#include "float.h"

#if !defined(PI)
#define PI     3.141592653589793
#endif
#if !defined(RADIAN)
#define RADIAN 0.017453292519943 // pi/180
#endif
#if !defined(DEGREE)
#define DEGREE 57.29577951308232 // 180/pi
#endif

#ifndef UNCHAR
#define UNCHAR
typedef unsigned char uchar;
#endif

#ifndef U_INT
#define U_INT
typedef unsigned int uint;
#endif

#ifdef WIN32
#include "omp.h"
#define isnan(x) ((x) != (x))
#pragma warning( disable : 4996 )
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

using std::string;
using std::ostream;
using std::ofstream;
using std::ifstream;
using std::cout;
using std::cin;
using std::endl;
using std::ios_base;
using std::flush;
using std::vector;



#endif
