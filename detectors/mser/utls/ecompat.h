#ifdef WIN32
#include <limits.h>
#include <float.h>

#define snprintf _snprintf
#define strdup   _strdup

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795028841971693993751
#endif

#ifndef _MSC_VER
#define __max max
#define __min min
#define CONST_TEMPLATE_PARAMETER const
#else
#define CONST_TEMPLATE_PARAMETER
#endif

#endif
