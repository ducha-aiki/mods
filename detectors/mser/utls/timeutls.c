#include "timeutls.h"

#ifdef _WIN32
#include <windows.h>

/* returns time spent */
double get_user_time()
{
  LARGE_INTEGER r;
  if (QueryPerformanceCounter(&r))
    return (double)r.QuadPart;
  else
    return GetTickCount();
}

/* returns number of clock_t's units per second */
double get_time_unit()
{
  LARGE_INTEGER r;
  if (QueryPerformanceFrequency(&r))
    return (double)r.QuadPart;
  else
    /* gettickcount resolution is 1ms */
    return 1000;

}

#else

#include <sys/times.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

/* returns time spent in user space */
double get_user_time()
{
  /*      struct tms t;
        times(&t);
        return t.tms_utime;*/

  struct timespec ts;
  if (!clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts))
    return (double)(1000*1000*1000) * ts.tv_sec + ts.tv_nsec;
  else
    {
      struct timeval tv;
      gettimeofday(&tv, 0);
      return (double)(1000*1000) * tv.tv_sec + tv.tv_usec;
    }
}

/* returns number of clock_t's units per second */
double get_time_unit()
{
  struct timespec ts;
  if (!clock_getres(CLOCK_PROCESS_CPUTIME_ID, &ts))
    return ts.tv_sec + (double)(1000000000)/ts.tv_nsec;
  else
    /* gettimeofday resolution */
    return 1000*1000;
}

#endif 

double get_time()
{
  return ((double)get_user_time())/get_time_unit();
}   

