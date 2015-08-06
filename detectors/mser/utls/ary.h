#ifndef _NEWARY_H_
#define _NEWARY_H_

#include <assert.h>
#include <cmath>
#include <stdio.h>
#include "dtypes.h"

namespace utls
{
template <typename PixelType>
struct Ary
{
private:
    typedef Ary<PixelType> AryBase;

public:
    typedef PixelType   value;
    typedef PixelType*  pointer;
    typedef PixelType** row_pointer;
    typedef int         size_type;
    typedef int         coord_type;

    /* construction */
    Ary()
    {
        lb1=lb2=ub1=ub2=0;
        num_rows=num_cols=0;
        data=0;
    }
    Ary(coord_type fr, coord_type lr,
        coord_type fc, coord_type lc, pointer attach_to = 0)
    {
        cons(fr, lr, fc, lc, attach_to);
    }

    Ary(coord_type nrows, coord_type ncols, pointer attach_to = 0)
    {
        cons(0, nrows-1, 0, ncols-1, attach_to);
    }

    Ary(const Ary &other, bool do_copy=true, bool do_attach = false)
    {
        if (!do_attach)
        {
            cons(other.lb1, other.ub1, other.lb2, other.ub2, 0);
            if (do_copy)
                copy(other);
        }
        else
            cons(other.lb1, other.ub1, other.lb2, other.ub2, other.data);
    }

    ~Ary()
    {
        deallocate();
    }

    /* basics */
    pointer   ptr() const
    {
        return data;
    }
    size_type rows() const
    {
        return num_rows;
    }
    size_type cols() const
    {
        return num_cols;
    }
    size_type size() const
    {
        return num_rows*num_cols*sizeof(value);
    }
    bool isin(coord_type row, coord_type col) const
    {
        return
            row>=lb1 && row<=ub1 && col>=lb2 && col<=ub2;
    }

    /* allocation */
    void cons(coord_type firstrow,
              coord_type lastrow,
              coord_type firstcol,
              coord_type lastcol,
              pointer    attach_to)
    {
        num_rows = lastrow - firstrow + 1;
        num_cols = lastcol - firstcol + 1;
        lb1 = firstrow;
        ub1 = lastrow;
        lb2 = firstcol;
        ub2 = lastcol;
        el = new pointer[num_rows];
        el = el - firstrow;
        if (!attach_to)
            data = new value[num_rows*num_cols];
        else
            data = attach_to;
        pointer mem = data - firstcol;
        for (coord_type r = firstrow; r <= lastrow; ++r)
        {
            el[r] = mem;
            mem = mem + num_cols;
        }
    }

    void clear()
    {
        for (int i = 0; i < num_rows*num_cols; i++)
            data[i]=(PixelType)0;
    }

    void set(const PixelType &value)
    {
        for (int i = 0; i < num_rows*num_cols; i++)
            data[i]=value;
    }

    void detach()
    {
        if (el)
        {
            el = el + lb1;
            delete [] el;
        }
        el = 0;
        data = 0;
        num_rows = num_cols = lb1 = lb2 = ub1 = ub2 = 0;
    }

    /* create a copy of an image */
    Ary* copy() const
    {
        AryBase *newary = new AryBase(lb1, ub1, lb1, ub2);
        for (int i = 0; i < num_rows*num_cols; i++)
            newary->data[i] = data[i];
        return newary;
    }

    /* create a copy of an image */
    void copy(const Ary &from)
    {
        for (int i = 0; i < num_rows*num_cols; i++)
            data[i] = from.data[i];
    }

    /* create a copy of an image */
    void copy(const Ary *from)
    {
        for (int i = 0; i < num_rows*num_cols; i++)
            data[i] = from->data[i];
    }

    void deallocate()
    {
        if (data)
            delete [] data;
        detach();
    }

public:
    coord_type    lb1, lb2, ub1, ub2;
    size_type     num_rows, num_cols;
    pointer       data;
    row_pointer   el;
    /* leave some space for aasociated user's variable */
    int           tag;
    void          *user_data;
};

/* one channel */
typedef Ary<unsigned char>    BAry;
typedef Ary<int>              IAry;
typedef Ary<long unsigned int> UI64Ary;
typedef Ary<unsigned int>     LAry;
typedef Ary<float>            FAry;
typedef Ary<double>           DAry;
typedef Ary<void *>           PAry;
}

#endif // _NEWARY_H_
