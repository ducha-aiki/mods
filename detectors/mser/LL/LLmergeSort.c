/*------- LL Double linked list library: mergeSort --------------------- */
/*  author: G. Matas                           (g.matas@ee.surrey.ac.uk) */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1992, 1993, George Matas.                               | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+ */
/*----------------------------------------------------------------------*/
static char sccsid[]="@(#)LLmergeSort.c	8.4	95/02/14 g.matas@ee.surrey.ac.uk";
typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/

#include "LL.h"
#include "linkLL.h"

#ifdef _OPENMP
#include <omp.h>
#endif

static l_list *  *listsort(l_list* *head, unsigned n);
static int (*listcompare)(const void * el1, const void * el2)=NULL;
#pragma omp threadprivate (listcompare)


/*----------------------------------------------------------------------*/
t_LL MergeSortLL(t_LL list, int (*compar) (const void*, const void*))
{
  int listSize = SizeLL(list);
  
  l_list *prevLink = list2link(list);
  int i;

  listcompare = compar;

  listsort(&(prevLink->forward),listSize);

  for(i=0; i<listSize;i++)     /* fix the backward links */
    {
      prevLink->forward->backward=prevLink;
      prevLink=prevLink->forward;
    }
  prevLink->forward->backward=prevLink;

  return list;
}
/*----------------------------------------------------------------------*/
t_LL MergeSortPassLL(t_LL l1, t_LL l2, int (*compar) (const void*, const void*))
{
  void * pL1 = FirstElmLL(l1);
  void * pL2 = FirstElmLL(l2);

  if (!IsElmLL(pL2)) return l1;                    /*  l2 empty */
  if (!IsElmLL(pL2)) return MoveListLastLL(l1,l2);   /*  l1 empty */

  while(1)
    {
      while(compar(pL1,pL2)<=0)
        {
          pL1 = NextElmLL(pL1);
          if(!IsElmLL(pL1)) return MoveTailLastLL(l1,l2,pL2);
        }

      while(compar(pL1,pL2)>0)
        {
          LinkBefLL(pL1,UnlinkLL(pL2));
          pL2 = FirstElmLL(l2);
          if (!IsElmLL(pL2)) return l1;
        }
    }
}
/*----------------------------------------------------------------------*/



#define listleq(p1,p2) ((*listcompare)(link2elm(p1),link2elm(p2))<=0)
#define getlink(elm) ((elm)->forward) 


/*----------------------------------------------------------------------*/
/*
  The code bellow was kindly made available by David Kastrup under
  these conditions:
*/

/* listsort.c
 * Copyright (c) 1992 David Kastrup, Goethestra"se~20/22, W-5100~Aachen,
 *       Germany
 * You are allowed to use this software in any form, even
 * in commercial software, as long as you do not restrain the right of
 * those using your software to obtain this code. That is, you must inform
 * your customer that this piece of code is in your program, and must provide
 * the unmodified source to him at request, at not more than a moderate
 * copying charge. You can save yourself this work if you include this in
 * source in your distribution. It is small enough.
 *
 * Other than that, you are free to use this software at will.
 */

/* The sort routine. Arguments are a pointer to the head pointer of
   a list to be sorted, as well as the number of elements to sort.
   Only n elements will be sorted, the rest of the list will not be
   disturbed. listsort returns a pointer to the head pointer of the
   rest of the list, located in the last element of the sorted part
   of the list. Thus if listsort calls itself recursively to sort
   the first half of a list, this call returns the head pointer of
   the second half to be sorted, list traversal thus being done on
   the fly.
*/

l_list* *listsort(l_list* *head, unsigned n)
{
  register l_list* p1, *p2;
  l_list **h2, **t2;
  unsigned m;

  switch (n) {
    case 0:
      return head;
      /* The trivial case of 0 was included, so that you may say for any
   accumulated list of n elements that is not yet NULL-ended something
   like: *listsort(&head, n) = NULL;
   even if the list is yet empty.
*/
    case 1:
      return &getlink(*head);
      /* Sorting one element must be provided, or recursion will fail. This
   is still sort of trivial
*/
    case 2:
      p2 = getlink(p1 = *head);
      /* p1 points now to first element, p2 to second */
      if (listleq(p1, p2))
        return &getlink(p2);
      /* if they were in order, return the tail link of the second */
      getlink(p1) = getlink(*head=p2);
      /* let head point to the second, and the first to the tail of the
   second
*/
      return &getlink(getlink(p2) = p1);
      /* and let the second point to the first, returning the taillink of the
   first as tail
*/
      /* Sorting two elements is provided for efficiency reasons. You could
   provide more cases fixed-coded as well, but test them out completely:
   they should preserve order of equal elements! AND they should work
   cleanly. And if you provide too much cases, chances are that you
   LOSE efficiency because the gains do not outweigh the disadvantage
   that the code does no longer fit in the processors code cache.
*/
    }
  /* Sorry that the default case appears outside of the switch. */
  n -= m = n / 2;
  /* n now has length of first sublist, m of second one */
  t2 = listsort(h2 = listsort(head, n), m);
  /* first n elements are sorted in *head, remaining m elements
   in *h2, rest of list hangs at *t2
*/
  if (listleq(p1 = *head, p2 = *h2)) {
      do {
          if (!--n)
            return *h2 = p2, t2;
        } while (listleq(p1=*(head=&getlink(p1)), p2));
    }
  /* The above caters efficiently for the condition that some or
   all of the first sublist may be smaller than the second sublist
*/

  /* The rest does a straight merge on the rest, starting with the
   inclusion of the first element of the second sublist which has
   tested as being smaller than the rest of the first sublist.
*/
  for (;;) {
      *head = p2;
      do {
          if (!--m)
            return *h2 = *t2, *t2 = p1, h2;
        } while (!listleq(p1, p2=*(head=&getlink(p2))));
      *head = p1;
      do {
          if (!--n)
            return *h2 = p2, t2;
        } while (listleq(p1=*(head=&getlink(p1)), p2));
    }
}
