/*------- LL Double linked list library: core functions ---------------- */
/*  author: G. Matas                           (g.matas@ee.surrey.ac.uk) */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1992, 1993, George Matas.                               | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+
 */

/*
  G. Matas, 30-Dec-93 v5.5
  - long history list deleted; available in the 5.5 delta.
  The list became redundant as LL was put under SCCS control.
*/
/*  based on a link library by Duane Morse                                  */
/*--------------------------------------------------------------------------*/
static char sccsid[]="@(#)LL.c		8.5	95/02/17 g.matas@ee.surrey.ac.uk";
typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include "LL.h"
#include "linkLL.h"
#include <stdlib.h>
#include <memory.h>

/* common error messages */
static char * NullMall = "malloc returned NULL";

/*------------- LinkIn/Out an element ----------------------------*/
void * LinkFirstLL(t_LL list,void * newEl)
{ l_lafter(list2link(list),elm2link(newEl)); return newEl; }
void * LinkLastLL(t_LL list,void * newEl)
{ l_lbefore(list2link(list),elm2link(newEl)); return newEl; }
void * LinkAftLL(void * curr,void * newEl)
{ l_lafter(elm2link(curr),elm2link(newEl)); return newEl; }
void * LinkBefLL(void * curr,void * newEl)
{ l_lbefore(elm2link(curr),elm2link(newEl)); return newEl; }

void * UnlinkLL(void * el) { l_unlink(elm2link(el)); return el; }

void * UnlinkNeLL(void * el)
{  
  void * next = NextElmLL(el);
  l_unlink(elm2link(el));
  return next;
}

void * UnlinkPrLL(void * el)
{  
  void * prev = PrevElmLL(el);
  l_unlink(elm2link(el));
  return prev;
}

/*-------------- Insert wihtout alloc ----------------------------------------*/
void * LinkInsBefLLf (void *el, size_t size, void *newEl)
{ 
  l_list *nel = elm2link(newEl);
  l_lbefore(elm2link(el), nel);
  nel->size=size;
  return newEl;
}

void * LinkInsAftLLf (void * el, size_t size, void * newEl)
{ 
  l_list *nel = elm2link(newEl);
  l_lafter(elm2link(el), nel);
  nel->size=size;
  return newEl;
}

void * LinkInsLastLLf (t_LL list, size_t size, void * newEl)
{ 
  l_list *nel = elm2link(newEl);
  l_lbefore(list2link(list), nel);
  nel->size=size;
  return newEl;
}

void * LinkInsFirstLLf (t_LL  list, size_t size, void * newEl)
{
  l_list *nel = elm2link(newEl);
  l_lafter(list2link(list), nel);
  nel->size=size;
  return newEl;
}

/*---------------- Insert Element ----------------------------------------*/
static void *
InsLLf(l_list *li, size_t size, void *data, void (*linkin)(l_list *,l_list *))
{
  l_list * newEl;

  if (NULL==(newEl=(l_list*)malloc(size+sizeof(t_linkLL))))
    AbortLL_M("InsLLf",NullMall);
  linkin(li,newEl);
  memcpy(link2elm(newEl),data,size);
  newEl->size=size;
  return (link2elm(newEl));
}


void * InsBefLLf (void *el, size_t size, void *data)
{ return InsLLf(elm2link(el),size,data,l_lbefore); }
void * InsAftLLf (void * el, size_t size, void * data)
{ return InsLLf(elm2link(el),size,data,l_lafter); }
void * InsLastLLf (t_LL list, size_t size, void * data)
{ return InsLLf(list2link(list),size,data,l_lbefore); }
void * InsFirstLLf (t_LL  list, size_t size, void * data)
{ return InsLLf(list2link(list),size,data,l_lafter); }

/*---------------- Delete Element ----------------------------------------*/
void  DelElmLL  (void * el)
{ 
  l_unlink( elm2link(el));
  free    ( elm2link(el));
}
void * DelElmNeLL(void * el)
{ 
  void * next = NextElmLL(el);
  DelElmLL(el);
  return (next);
}

void * DelElmPrLL(void * el)
{ 
  void * prev = PrevElmLL(el);
  DelElmLL(el);
  return (prev);
}

/*----------------- Get an element ---------------------------------------*/
void * FirstElmLL(t_LL list)  { return link2elm(l_nextl(list2link(list))); }
void *  LastElmLL(t_LL list)  { return link2elm(l_prevl(list2link(list))); }
void *  PrevElmLL(void * el)  { return link2elm(l_prevl(elm2link(el))); }
void *  NextElmLL(void * el)  { return link2elm(l_nextl(elm2link(el))); }

void *  NthElmLL(t_LL list, t_LLsize num)
{                  /* 2.1 : NthElmLL can be called with a negative value */
  l_list * link = list2link(list);

  if (num >0) while (num--) link = l_nextl(link);
  else        while (num++) link = l_prevl(link);

  return link2elm(link);
}

void *  RelNthElmLL(void * el, t_LLrelsize num)
{
  l_list * link = elm2link(el);

  if (num >0) while (num--) link = l_nextl(link);
  else        while (num++) link = l_prevl(link);

  return link2elm(link);
}


void *  PrevCElmLL(void * el)
{
  l_list * prevLink        = l_prevl(elm2link(el));
  if (prevLink->size == 0)   prevLink=l_prevl(prevLink);
  return link2elm(prevLink);
}

void *  NextCElmLL(void * el)
{
  l_list * nextLink        = l_nextl(elm2link(el));
  if (nextLink->size == 0)   nextLink=l_nextl(nextLink);
  return link2elm(nextLink);
}

void *  RelCNthElmLL(void * el, t_LLrelsize num)
{
  l_list * link = elm2link(el);

  if(num>0)while(num--){link=l_nextl(link);if(link->size==0)link=l_nextl(link);}
  else     while(num++){link=l_prevl(link);if(link->size==0)link=l_prevl(link);}

  return link2elm(link);
}

/*--------------------------------------------------------------------------*/
int IsElmLL     (void * el) { return elm2link(el)->size ; }
int IsLastElmLL(void *el)  {return !IsElmLL(NextElmLL(el)); }
int IsFirstElmLL(void *el) {return !IsElmLL(PrevElmLL(el)); }

int IsNthElmLL(t_LL list, t_LLsize n, void *el)
{return el == NthElmLL(list,n); }

/*--------------------------------------------------------------------------*/
t_LL InitLL(struct s_LL* head)
{
  l_linit(list2link(head));
  head->links.u.ll.size= 0;
  return (head);
}

/*--------------------------------------------------------------------------*/
t_LL ConsLL(void)
{
  t_LL   head;

  if (NULL==(head=(t_LL) malloc(sizeof(*head))))
    AbortLL_M("CreatLL",NullMall);

  return InitLL(head);
}

/*--------------------------------------------------------------------------*/
int  IsEmptyLL(t_LL list) { return (l_lempty(list2link(list))); }
t_LL EmptyLL(t_LL list)
{
  l_list  * head = list2link(list);
  l_list  * link = l_nextl(head);
  l_list  * old  =link;

  while(head != link){
      old = link;
      link = l_nextl(link);
      free(old);
    }

  l_linit(head);

  return list;
}

/*--------------------------------------------------------------------------*/
void  *DestLL(t_LL list)
{
  EmptyLL(list);
  free(list);
  return NULL;
}

/*--------------------------------------------------------------------------*/
void * ApplyLL (t_LL list, void * (*apply) (void*))
{
  void * el, * ret_el, *next;

  SafeForeachLL_M (list,el,next){
    next = NextElmLL(el);
    if ((ret_el=(*apply)(el)) != NULL ) return ret_el;
  }

  return NULL;
}

/*--------------------------------------------------------------------------*/
t_LL ReverseLL (t_LL list)
{
  l_list * head = list2link(list);
  l_list * link = head;
  l_list * temp;

  do{
      temp= link->forward;               /* swap */
      link->forward = link->backward;
      link->backward= temp;

      link=l_prevl(link);                /* move */
    }
  while (head != link) ;
  
  return list;
}

/*--------------------------------------------------------------------------*/
t_LLsize SizeLL(t_LL list)
{
  t_LLsize i=0;
  l_list * head = list2link(list);
  l_list * link;

  ForeachLink_M(head,link) i++;
  
  return i;
}
/*--------------------------------------------------------------------------*/ 
int IsShorterThanLL(t_LL list, int max)
{
  t_LLsize i=0;
  l_list * head = list2link(list);
  l_list * link;

  ForeachLink_M(head,link)
  {
    if(++i == max) return 0;
  }
  return 1;
}

/*--------------------------------------------------------------------------*/
t_LL  ConsPtrLL(t_LL src)
{
  void * el ;
  t_LL dest= ConsLL();

  ForeachLL_M (src,el)
      InsLastLL(dest,el);

  return dest;
}
/*--------------------------------------------------------------------------*/
t_LL  ConsCopyLL(t_LL src)
{
  void * el ;
  t_LL dest= ConsLL();

  ForeachLL_M (src,el)
      InsLastLLf(dest,elm2link(el)->size, el);

  return dest;
}


/*--------------------------------------------------------------------------*/
/* cut what is required and paste it after dest */
static void CutPaste(l_list *first_out, l_list *first_not_out, l_list *dest)
{
  l_list *last_out = first_not_out->backward;

  if (first_out==first_not_out) return;

  first_out->backward->forward = first_not_out;      /* cut */
  first_not_out->backward      = first_out->backward;

  last_out->forward   = dest->forward;
  first_out->backward = dest;

  dest->forward->backward = last_out;
  dest->forward           = first_out;
  
}
/*-------------- Move List -----------------------------------------------*/
t_LL  MoveListFirstLL(t_LL  dest, t_LL src)
{
  CutPaste(elm2link(FirstElmLL(src)),list2link(src),list2link(dest));
  return dest;
}
t_LL  MoveListLastLL(t_LL  dest, t_LL src)
{
  CutPaste(elm2link(FirstElmLL(src)),list2link(src),elm2link(LastElmLL(dest)));
  return dest;
}
void *  MoveListAftLL(void *el,  t_LL src)
{
  CutPaste(elm2link(FirstElmLL(src)),list2link(src),elm2link(el));
  return el;
}
void *  MoveListBefLL(void *el,  t_LL src)
{
  CutPaste(elm2link(FirstElmLL(src)),list2link(src),elm2link(PrevElmLL(el)));
  return el;
}
/*-------------- Move Head -----------------------------------------------*/
t_LL  MoveHeadFirstLL(t_LL  dest, t_LL src, void *head)
{
  CutPaste(elm2link(FirstElmLL(src)),elm2link(head),list2link(dest));
  return dest;
}
t_LL  MoveHeadLastLL(t_LL  dest, t_LL src, void *head)
{
  CutPaste(elm2link(FirstElmLL(src)),elm2link(head),elm2link(LastElmLL(dest)));
  return dest;
}
void *  MoveHeadAftLL(void *el,  t_LL src, void *head)
{
  CutPaste(elm2link(FirstElmLL(src)),elm2link(head),elm2link(el));
  return el;
}
void *  MoveHeadBefLL(void *el,  t_LL src, void *head)
{
  CutPaste(elm2link(FirstElmLL(src)), elm2link(head),elm2link(PrevElmLL(el)));
  return el;
}
/*-------------- Move Tail -----------------------------------------------*/
t_LL  MoveTailFirstLL(t_LL  dest, t_LL src, void *tail)
{
  CutPaste(elm2link(tail),list2link(src),list2link(dest));
  return dest;
}

t_LL  MoveTailLastLL(t_LL  dest, t_LL src, void *tail)
{
  CutPaste(elm2link(tail),list2link(src),elm2link(LastElmLL(dest)));
  return dest;
}
void *  MoveTailAftLL(void *el,  t_LL src, void *tail)
{
  CutPaste(elm2link(tail),list2link(src),elm2link(el));
  return el;
}

void *  MoveTailBefLL(void *el,  t_LL src, void *tail)
{
  CutPaste(elm2link(tail),list2link(src),elm2link(PrevElmLL(el)));
  return el;
}

/*-------- Create a look up table into a list for random access ------------*/
void * LookInLL(t_LL list)
{

  void * * array = (void **) malloc ((SizeLL(list)+ 1) * sizeof(void *));
  /* array has one element more then the size of the list  */
  /* so that the first element is array[1]                 */
  /* array[0] is the head of the list*/

  void * el;
  int i = 1;

  array[0]=list;
  ForeachLL_M (list,el)
      array[i++] = el;

  return array;
}

/*--------------------------------------------------------------------------*/
t_LLsize IndexElmLL(t_LL list, void *ind_el)
{
  void *el;
  t_LLsize i=1;

  ForeachLL_M (list,el)
      if (el==ind_el) return i;
  else i++;
  
  return 0;
}

static int (*UserCompare) (const void * el1, const void * el2);
#pragma omp threadprivate (UserCompare)

static int IntCompare(const void *el1, const void*el2)
{ return (*UserCompare) (*(void *const*)el1, *(void *const*)el2);}

/*--------------------------------------------------------------------------*/
t_LL SysSortLL(t_LL list,  int (*compar) (const void*, const void*))
{
  int ListSize = SizeLL(list);
  l_list *      head_link;

  void * el;
  void * * array = (void **) malloc (ListSize * sizeof(void *));
  int i = 0;
  
  ForeachLL_M (list,el)
      array[i++] = el;

  UserCompare = compar;
  qsort(array,ListSize,sizeof(void *),IntCompare);

  head_link = list2link(list);
  l_linit(head_link);

  for(i=0; i<ListSize; i++)
    l_lbefore(head_link,elm2link(array[i]));
  
  free(array);
  return list;
}
