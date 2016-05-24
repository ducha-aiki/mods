/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#ifndef __SUBALLOC_H__
#define __SUBALLOC_H__

#include <LL.h>
#include <string.h>

namespace extrema
{

  //! Internal structure with suballocator's data.
  typedef struct s_suballocator
  {
    t_LL   items;
    t_LL   blocks;
    size_t free_items;
    size_t item_size;
    size_t block_size;
    int    clear_blocks;
  } t_suballocator;

  void InitSuballocator(t_suballocator *s, size_t blocksize, size_t itemsize,
                        int clear_blocks=0);
  void DestSuballocator(t_suballocator *s);
  void SuballocatorAddBlock(t_suballocator *s);

  /*----------------------------------------------------------------*/

  static inline void *SuballocatorGetItem(t_suballocator *s)
  {
    if (s->free_items==0)
      SuballocatorAddBlock(s);

    s->free_items--;
    return UnlinkLL(FirstElmLL(s->items));
  }

  /*----------------------------------------------------------------*/

  static inline void SuballocatorReturnItem(t_suballocator *s, void *item)
  {
    s->free_items++;
    LinkFirstLL(s->items, item);
  }

  /*----------------------------------------------------------------*/

  static inline void SuballocatorReturnItemsLL(t_suballocator *s, t_LL list)
  {
    void *item, *next;
    SafeForeachLL_M(list, item, next)
    {
      next = NextElmLL(item);
      SuballocatorReturnItem(s, UnlinkLL(item));
    }
  }

}
#endif // __SUBALLOC_H__
