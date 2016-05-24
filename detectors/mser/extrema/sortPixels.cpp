/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include "sortPixels.h"

using namespace utls;

namespace extrema
{

  int ReplaceExtWithSecond(BAry * &img)
  {
    int i, rows = img->rows()-2, cols = img->cols()-2;
    int d=0;
    unsigned char *line_end = &img->el[1][cols-1];
    unsigned char *p = &img->el[0][1];
    unsigned char *c = &img->el[1][1];
    unsigned char *n = &img->el[2][1];
    for(i=1; i < rows-1; i++)
      {
        // 32bit manipulation is more convient for the 32bit processor
        int prev, curr, next;
        prev = *(c-1);
        curr = *c;
        next = *(c+1);
        while (c<line_end)
          {
            if ((curr-prev)*(curr-next)>0)
              {
                if (curr>prev)
                  {
                    if (curr>*p && curr>*n)
                      {
                        // value of prev is not important in next pixel, use it as max
                        if (prev<next)  prev = next;
                        if (prev<*p)    prev = *p;
                        if (prev<*n)    prev = *n;
                        // update curr and memory
                        *c = curr = prev;
                        d++;
                      }
                  }
                else
                  {
                    if (curr<*p && curr<*n)
                      {
                        // value of prev is not important in next pixel, use it as min
                        if (prev>next)  prev = next;
                        if (prev>*p)    prev = *p;
                        if (prev>*n)    prev = *n;
                        // update curr and memory
                        *c = curr = prev;
                        d++;
                      }
                  }
              }
            p++;
            c++;
            n++;
            prev=curr;
            curr=next;
            next=*(c+1);
          }
        // prepare pointers for next line
        line_end+=cols+2;
        p+=4;
        c+=4;
        n+=4;
      }
    return d;
  }

  void CalcHistogram(BAry * &img, t_sortpixels &pixels)
  {
    int i, j, rows = img->rows()-2, cols = img->cols()-2;
    for(i=0; i < c_maxByte; i++)
      pixels.hist[i] = 0;
    unsigned char *src = &img->el[0][0];
    for (i=0; i < rows; i++)
      {
        for (j=0; j < cols; j++)
          pixels.hist[*src++]++;
        src+=2;
      }
  }

  void BinSortPixels(BAry * &img, t_sortpixels &pixels)
  {
    int i, j, cumsize=0, rows = img->rows()-2, cols = img->cols()-2;
    t_offset *last[c_maxByte], *tmp;
    /* initialize pixels counts */
    for(i=0; i < c_maxByte; i++)
      {
        last[i] = 0;
        cumsize+=pixels.hist[i]+1;
      }
    tmp = pixels.block = (t_offset *)malloc(cumsize*sizeof(t_offset));

    /* allocate memory for pixel coordinates */
    for(i=0; i < c_maxByte; i++)
      {
        if (pixels.hist[i])
          {
            last[i] = tmp;
            tmp += pixels.hist[i]+1;
          }
        pixels.data[i] = last[i];
      }

    /* fill pixel lists with positions */
    unsigned char *src = &img->el[0][0];
    t_offset offset = cols+2;
    for (i=0; i < rows; i++)
      {
        /* offset ++ : the image of labels is padded on left and right */
        offset++;
        for (j=0; j < cols; j++)
          {
            *last[*src++]++ = offset;
            offset++;
          }
        offset++;
        src+=2;
      }
  }

  /* inverts padded intensity image and it's intesity histogram */
  void InvertImageAndHistogram(BAry *img, t_sortpixels &pixels)
  {
    int i, rows = img->rows()-2, cols = img->cols();
    unsigned char *src = &img->el[0][0];
    for (i=0; i < rows*cols; i++)
      {
        *src = 255 - *src;
        src++;
      }

    int     auxHist;
    t_offset * auxData;
    for(i=0; i < (c_maxByte / 2) ; i++)
      {
        auxHist = pixels.hist[i];
        pixels.hist[i] = pixels.hist[c_maxByte - i - 1];
        pixels.hist[c_maxByte - i - 1] = auxHist;

        auxData = pixels.data[i];
        pixels.data[i] = pixels.data[c_maxByte - i - 1];
        pixels.data[c_maxByte - i - 1] = auxData;
      }
  }

}
