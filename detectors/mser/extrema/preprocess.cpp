/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#include <math.h>
#include "preprocess.h"

using namespace utls;

namespace extrema
{

#define impl_preprocess_function_seq(channel_conversion, intensity_processing)\
  void CPreprocessRGBSeq::                                              \
  preprocess_## channel_conversion ## _ ## intensity_processing         \
  (const ExtremaImage &image, int preprocess_type,                      \
  BAry *&img)                                                          \
  {                                                                     \
  unsigned int x, y;                                                 \
  unsigned char *sptr, *dptr, v;                                     \
  if (!img)                                                          \
  img = new BAry(-1,image.height, -1, image.width);               \
  dptr = &img->el[0][-1];                                            \
  sptr = image.data;                                                 \
  for (y = 0; y < image.height; y ++)                                \
  {                                                                  \
  dptr++;                                                         \
  for (x = 0; x < image.width; x ++)                              \
  {                                                               \
  v = (unsigned char)                                          \
  inten_to_ ## intensity_processing (                       \
  rgb_to_ ## channel_conversion (sptr, sptr+1, sptr+2)   \
  );                                                     \
  *(dptr++) = v;                                               \
  sptr+=image.channels;                                        \
}                                                               \
  dptr++;                                                         \
}                                                                  \
}

#define impl_preprocess_function_plan(channel_conversion,intensity_processing)\
  void CPreprocessRGBPlanes::                                           \
  preprocess_## channel_conversion ## _ ## intensity_processing         \
  (const ExtremaImage &image, int preprocess_type, int swap,            \
  BAry *&img)                                                          \
  {                                                                     \
  unsigned int x, y, size, rowinc, colinc, ofs;                      \
  unsigned char *R, *r, *G, *g, *B, *b, *dptr, v;                    \
  if (swap) { rowinc = image.height;  colinc = 1; } else             \
  { rowinc = 1; colinc = image.width; }                              \
  if (!img)                                                          \
  img = new BAry(-1,image.height, -1, image.width);               \
  size = image.width * image.height * sizeof(unsigned char);         \
  dptr = &img->el[0][-1];                                            \
  R = image.data; G = R + size; B = G + size;                        \
  for (y = 0; y < image.height; y ++)                                \
  {                                                                  \
  ofs = y * colinc; r = R + ofs; g = G + ofs; b = B + ofs;        \
  dptr++;                                                         \
  for (x = 0; x < image.width; x ++)                              \
  {                                                               \
  v = (unsigned char)                                          \
  inten_to_ ## intensity_processing (                       \
  rgb_to_ ## channel_conversion (r, g, b)                \
  );                                                     \
  *(dptr++) = v;                                               \
  r+=rowinc; g+=rowinc; b+=rowinc;                             \
}                                                               \
  dptr++;                                                         \
}                                                                  \
}

  impl_preprocess_function_seq(none,none);
  impl_preprocess_function_seq(intensity,none);
  impl_preprocess_function_seq(saturation,none);
  impl_preprocess_function_seq(hue,none);
  impl_preprocess_function_seq(red,none);
  impl_preprocess_function_seq(green,none);
  impl_preprocess_function_seq(blue,none);
  impl_preprocess_function_seq(redblue,none);
  impl_preprocess_function_seq(intensity_half,none);

#undef case_preprocess_function
#undef impl_preprocess_function_seq

#define case_preprocess_function(channel_conversion, intensity_processing)\
  case PREPROCESS_CHANNEL_ ## channel_conversion +  \
  PREPROCESS_INTENSITY_ ## intensity_processing: \
  preprocess_## channel_conversion ## _ ## intensity_processing( \
  image, preprocess_type, img \
  ); \
  break;


  void CPreprocessRGBSeq::preprocess(const ExtremaImage &image,
                                     int preprocess_type, BAry *&img)
  {
    switch (preprocess_type)
      {
      case_preprocess_function(none,none);
      case_preprocess_function(intensity,none);
      case_preprocess_function(saturation,none);
      case_preprocess_function(hue,none);
      case_preprocess_function(red,none);
      case_preprocess_function(green,none);
      case_preprocess_function(blue,none);
      case_preprocess_function(redblue,none);
      case_preprocess_function(intensity_half,none);

      default:
        printf("Unknown preprocessing type %0x\n", preprocess_type);
      }
  }

#undef case_preprocess_function
#undef impl_preprocess_function_seq

#define case_preprocess_function(channel_conversion, intensity_processing)\
  case PREPROCESS_CHANNEL_ ## channel_conversion +  \
  PREPROCESS_INTENSITY_ ## intensity_processing: \
  preprocess_## channel_conversion ## _ ## intensity_processing( \
  image, preprocess_type, swap, img \
  ); \
  break;

  impl_preprocess_function_plan(none,none);
  impl_preprocess_function_plan(intensity,none);
  impl_preprocess_function_plan(saturation,none);
  impl_preprocess_function_plan(hue,none);
  impl_preprocess_function_plan(red,none);
  impl_preprocess_function_plan(green,none);
  impl_preprocess_function_plan(blue,none);
  impl_preprocess_function_plan(redblue,none);
  impl_preprocess_function_plan(intensity_half,none);

  void CPreprocessRGBPlanes::preprocess(const ExtremaImage &image,
                                        int preprocess_type, int swap,
                                        BAry *&img)
  {
    switch (preprocess_type)
      {
      case_preprocess_function(none,none);
      case_preprocess_function(intensity,none);
      case_preprocess_function(saturation,none);
      case_preprocess_function(hue,none);
      case_preprocess_function(red,none);
      case_preprocess_function(green,none);
      case_preprocess_function(blue,none);
      case_preprocess_function(redblue,none);
      case_preprocess_function(intensity_half,none);

      default:
        printf("Unknown preprocessing type %0x\n", preprocess_type);
      }
  }

#undef case_preprocess_function
#undef impl_preprocess_function_plan

}
