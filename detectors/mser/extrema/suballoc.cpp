/*--------------------------------------------------------------------------*/
/* Copyright 2006, Jiri Matas & Michal Perdoch       matas@cmp.felk.cvut.cz */
/*--------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "suballoc.h"

namespace extrema
{

/*----------------------------------------------------------------*/

void InitSuballocator(t_suballocator *s, size_t blocksize, size_t itemsize,
                      int clear_blocks)
{
    s->items = ConsLL();
    s->blocks = ConsLL();
    s->block_size = blocksize;
    s->item_size = itemsize;
    s->clear_blocks = clear_blocks;
    s->free_items = 0;
}

/*----------------------------------------------------------------*/

void SuballocatorAddBlock(t_suballocator *s)
{
    /* inserts new block with items into list of suballocation blocks */
    int real_item_size = sizeof(t_linkLL) + s->item_size;
    int subblock_size = s->block_size * real_item_size;
    t_linkLL *pBlock = (t_linkLL *)malloc(sizeof(t_linkLL) + subblock_size);

    /* shift pointer to skip the link */
    t_linkLL *pItem = ++pBlock;

    if (s->clear_blocks)
        memset(pBlock, 0, subblock_size);

    assert(sizeof(unsigned char)==1);

    /* skip link stuff */
    pItem++;
    for (size_t i=0; i<s->block_size; i++)
    {
        LinkInsLastLLf(s->items, s->item_size, pItem);
        /* skip item's data */
        pItem = (t_linkLL *)(((unsigned char *)pItem)+real_item_size);
    }
    s->free_items+=s->block_size;
    LinkInsLastLLf(s->blocks, subblock_size, pBlock);
}

/*----------------------------------------------------------------*/

void DestSuballocator(t_suballocator *s)
{
    /* avoid double free, all items are in suballoc blocks */
    free(s->items);
    s->items=0;
    DestLL(s->blocks);
    s->blocks=0;
    s->free_items = s->item_size = s->block_size = 0;
}

/*----------------------------------------------------------------*/
}
