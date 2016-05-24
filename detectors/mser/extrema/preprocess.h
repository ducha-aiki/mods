/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#ifndef __PREPROCESS_H__
#define __PREPROCESS_H__

#include <math.h>
#include <ary.h>
#include "extremaTypes.h"
#include "extremaParams.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace extrema
{

  class CPreprocess
  {
  public:

    double min(double a, double b) const
    {
      if (a<b)
        return a;
      else
        return b;
    }

    double max(double a, double b) const
    {
      if (a>b)
        return a;
      else
        return b;
    }

    /* channels conversion functions */
    int rgb_to_none(const unsigned char *r,
                    const unsigned char *g,
                    const unsigned char *b) const
    {
      return *r;
    }

    int rgb_to_intensity(const unsigned char *r,
                         const unsigned char *g,
                         const unsigned char *b) const
    {
      return (((*r)+(*g)+(*b))/3);
    }

    int rgb_to_intensity_half(const unsigned char *r,
                              const unsigned char *g,
                              const unsigned char *b) const
    {
      return (((*r)+(*g)+(*b))/6);
    }

    int rgb_to_saturation(const unsigned char *r,
                          const unsigned char *g,
                          const unsigned char *b) const
    {
      unsigned char gray = (*r + *g + *b)/3;
      int dr = *r - gray;
      int dg = *g - gray;
      int db = *b - gray;
      return int(min(2*sqrt(double(dr*dr+dg*dg+db*db)), 255));
    }

    int rgb_to_hue(const unsigned char *r,
                   const unsigned char *g,
                   const unsigned char *b) const
    {
      double k2 = 1 / sqrt (2.0);
      double k6 = 1 / sqrt (6.0);
      double b1 = k6 * (2*(*b) - (*r) - (*g));
      if (b1 != 0)
        {
          double x1 = k2 * ((*g) - (*r));
          double hue = atan (x1 / b1);
          if (*g > *r && hue < 0) hue += M_PI;
          if (*g < *r && hue > 0) hue += M_PI;
          if (*g == *r && *r > *b)  hue = M_PI;
          return (unsigned char)(hue / M_PI*128);
        }
      else
        return 0;
    }

    int rgb_to_red(const unsigned char *r,
                   const unsigned char *g,
                   const unsigned char *b) const
    {
      return *r;
    }

    int rgb_to_green(const unsigned char *r,
                     const unsigned char *g,
                     const unsigned char *b) const
    {
      return *g;
    }

    int rgb_to_blue(const unsigned char *r,
                    const unsigned char *g,
                    const unsigned char *b) const
    {
      return *b;
    }

    int rgb_to_redblue(const unsigned char *r,
                       const unsigned char *g,
                       const unsigned char *b) const
    {
      int u = ((*r) + 255 - (*b))/2;
      return (int)min (max (2*u-128, 0), 255);
    }

    /* intensity preprocessing functions */
    int inten_to_none(int intensity) const
    {
      return intensity;
    }

  };

  /* this macro defines a sequential conversion function */
#define def_preprocess_function_seq(channel_conversion, intensity_processing)\
  void preprocess_## channel_conversion ## _ ## intensity_processing    \
  (const ExtremaImage &image, int preprocess_type, utls::BAry *&img);

  /* this macro defines a planewise conversion function */
#define def_preprocess_function_plan(channel_conversion, intensity_processing)\
  void preprocess_## channel_conversion ## _ ## intensity_processing    \
  (const ExtremaImage &image, int preprocess_type, int swap, utls::BAry *&img);

  class CPreprocessRGBSeq : public CPreprocess
  {
  public:
    void preprocess(const ExtremaImage &image, int preprocess_type, utls::BAry *&img);
  private:
    def_preprocess_function_seq(none,none)
    def_preprocess_function_seq(intensity,none)
    def_preprocess_function_seq(saturation,none)
    def_preprocess_function_seq(hue,none)
    def_preprocess_function_seq(red,none)
    def_preprocess_function_seq(green,none)
    def_preprocess_function_seq(blue,none)
    def_preprocess_function_seq(redblue,none)
    def_preprocess_function_seq(intensity_half, none);
  };

  class CPreprocessRGBPlanes : public CPreprocess
  {
  public:
    void preprocess(const ExtremaImage &image, int preprocess_type, int swap, utls::BAry *&img);
  private:
    def_preprocess_function_plan(none,none)
    def_preprocess_function_plan(intensity,none)
    def_preprocess_function_plan(saturation,none)
    def_preprocess_function_plan(hue,none)
    def_preprocess_function_plan(red,none)
    def_preprocess_function_plan(green,none)
    def_preprocess_function_plan(blue,none)
    def_preprocess_function_plan(redblue,none)
    def_preprocess_function_plan(intensity_half, none);
  };
}
#endif
