#ifndef _UTLS__DTYPES_H_
#define _UTLS__DTYPES_H_

namespace utls
{
typedef unsigned char                                          t_byte;
typedef union
{
    t_byte arr [3];
    struct
    {
        t_byte r,g,b;
    }  st;
}   t_rgb;
typedef union
{
    float  arr [3];
    struct
    {
        float r,g,b;
    }   st;
}   t_frgb;
}

#endif // _UTLS__DTYPES_H_
