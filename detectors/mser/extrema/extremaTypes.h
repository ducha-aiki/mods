/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#ifndef __EXTREMA_TYPES_H__
#define __EXTREMA_TYPES_H__

#include <LL.h>
#include <vector>
#include "extremaConfig.h"

#define c_maxByte 256

namespace extrema
{

#ifdef A64

typedef unsigned long t_label;
typedef unsigned long t_mregion;

#else

typedef unsigned int t_label;
typedef unsigned int t_mregion;

#endif

typedef unsigned int t_offset;

//! Internal structure, holds 2D point coordinates.
typedef struct
{
    int x;
    int y;
} t_ipoint;

//! Internal structure with intensity histogram.
typedef struct s_sortpixels
{
    t_offset * block;
    t_offset * data[c_maxByte];
    int     hist[c_maxByte];
} t_sortpixels;

//! Internal region structure.
typedef struct s_region
{
    t_label   label;
    int       minimum_int;
    int       pixel_total;
    int       border_total;
    t_ipoint  minimum_pos;
    int       maximum_int;
    t_label   merge_label;
    t_LL      thresholds;
    int       pixels[c_maxByte];
    int       borders[c_maxByte];
} t_region;

//! Internal structure with processed detector's parameters.
typedef struct s_thresh_par
{
    //! minimum size of the region in pixels
    int    min_size;

    //! scaled internal min_size of the region in pixels*4
    int    min_size_int;

    //! maximum size of the region in pixels
    int    max_size;

    //! minimum margin and upper boundary for hystheresis thresholding
    double min_margin;

    //! margin relative to intesity level
    bool   relative_margin;

    //! do inverted margin
    int    invert;
} t_thresh_par;

//! Structure with pixel of the extended boundary.
typedef struct s_borderpixel
{
    t_offset      ofs;
    unsigned char direct; // N = 1, E = 2, S = 4, W = 8; 0 = unknown direction
    bool operator<(const s_borderpixel &other) const
    {
        return ofs<other.ofs;
    }
} t_borderpixel;

//! Vector with extended boundary.
typedef std::vector<t_borderpixel> point_vector;

//! Internal structure holding threshold paramaters.
typedef struct s_thresh_def
{
    int            thresh;
    int            pos;
    int            margin;
    point_vector  *boundary;
} t_thresh_def;

//! Internal structure used in threshold sorting.
struct t_sorted_region_threshold
{
    t_region     *p_r;
    t_thresh_def *p_t;
};
}
#endif
