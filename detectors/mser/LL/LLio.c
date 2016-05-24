/*------- LL Double linked list library: I/O functions ----------------- */
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
/*-----------------------------------------------------------------------*/
static char sccsid[]="@(#)LLio.c	8.3	94/12/20 g.matas@ee.surrey.ac.uk";
typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/

#include "LL.h"
#include "linkLL.h"
//#include <stdio.h>
#include <stdlib.h>
#ifdef _OPENMP
#include <omp.h>
#endif


/*-------------- Input --------------------------------------------------*/
/*-----------------------------------------------------------------------*/
static t_LL     GetList(void);
static t_LLsize  Gett_LLsize(void);
static char     GetChar(void);

static FILE *fileR = NULL;

t_LL ReadLL(char * filename)
{
  t_LL   list;

  if('-' == *filename) fileR = stdin;
  else if (NULL==(  fileR = fopen(filename,"rb")))
    AbortLL_M("ReadLL","fopen failed");

  /* .LL file starts #LL followed by a (t_LLsize) 0*/
  if('#'!=GetChar() || 'L'!=GetChar() || 'L'!=GetChar() || 0!=Gett_LLsize())
    AbortLL_M("ReadLL","file not in LL format!");

  list = GetList();

  {                              /* test whether the whole file was read */
    char c;
    if(0 != fread(&c,sizeof(c),1,fileR))
      AbortLL_M("ReadLL","trailing chars found");
  }
  
  fclose(fileR);
  return list;
}

/*---------- low level reads -------------------------------------------*/
static t_LLsize Gett_LLsize(void)
{
  t_LLsize l;
  if (1 != fread(&l,sizeof(l),1,fileR))
    AbortLL_M("Gett_LLsize","fread failed");

  return l;
}

static char GetChar(void)
{
  char c;
  if (1 != fread(&c,sizeof(c),1,fileR))
    AbortLL_M("GetChar","fread failed");

  return c;
}

static t_ELMsize Gett_ELMsize(void)
{
  t_ELMsize l;
  if (1 != fread(&l,sizeof(l),1,fileR))
    AbortLL_M("Gett_ELMsize","fread failed");

  return l;
}

/*---------- empty element allocation and and linking ------------------*/
static void *
InsEmptyBefLLf(l_list *li, size_t size)
{
  l_list * newEl;

  if (NULL==(newEl=(l_list*)malloc(size+sizeof(t_linkLL))))
    AbortLL_M("InsEmptyBefLLf","malloc failed");

  l_lbefore(li,newEl);
  newEl->size=size;
  return (link2elm(newEl));
}

/*-----------------------------------------------------------------------*/
static t_LL GetList(void)
{
  t_LL list = ConsLL();
  t_LLsize size = Gett_LLsize();

  while(size-- > 0)
    {
      t_ELMsize elemSize = Gett_ELMsize();
      if (0 == elemSize )
        {  /* this element is a list, get it by a recursive call */
          t_LL listElem = GetList();
          InsLastLL(list,listElem);
        }
      else
        {
          void * elemData = InsEmptyBefLLf(list2link(list),elemSize);
          if (1!=fread(elemData,elemSize,1,fileR))
            AbortLL_M("GetList","fread failed");
        }
    }

  return list;
}

/*-------------- Output --------------------------------------------------*/
/*-----------------------------------------------------------------------*/
static void PutListLev1(t_LL list);
static void PutListLev2(t_LL list);
static void PutListLev3(t_LL list);
static void PutListLevN(t_LL list, int level);

static void WritefLL(char * filename, t_LL list, void (*PutList)(t_LL));

void WriteLev1LL(char *name, t_LL list) { WritefLL(name,list,PutListLev1);} 
void WriteLev2LL(char *name, t_LL list) { WritefLL(name,list,PutListLev2);} 
void WriteLev3LL(char *name, t_LL list) { WritefLL(name,list,PutListLev3);} 

/*-----------------------------------------------------------------------*/
static FILE * fileW = NULL;

/*-----------------------------------------------------------------------*/
static void Putt_LLsize(t_LLsize l)
{
  if (1 != fwrite(&l,sizeof(l),1,fileW))
    AbortLL_M("Putt_LLsize","fwrite failed");
}

static void Putt_ELMsize(t_ELMsize l)
{
  if (1 != fwrite(&l,sizeof(l),1,fileW))
    AbortLL_M("Putt_ELMsize","fwrite failed");
}

/*-----------------------------------------------------------------------*/
static void WritefLL(char * filename, t_LL list, void (*PutList)(t_LL l))
{
  if('-' == *filename) fileW = stdout;
  else if (NULL==(  fileW = fopen(filename,"wb")))
    AbortLL_M("WriteLL","fopen failed");

  fprintf(fileW,"#LL");
  
  PutList(list);

  fclose(fileW);
}
/*-----------------------------------------------------------------------*/
static int wlistLevel;
#pragma omp threadprivate (wlistLevel)

static void PutListStatN(t_LL list) {PutListLevN(list,wlistLevel);}

void WriteLevNLL(char * f_name, t_LL list, int l)
{
  wlistLevel = l;
  WritefLL(f_name,list,PutListStatN);
}
/*-----------------------------------------------------------------------*/
static void PutElem(void * elem)
{
  t_ELMsize elemSize = elm2link(elem)->size;
  Putt_ELMsize(elemSize);
  if (1!=fwrite(elem,(int)elemSize,1,fileW))
    AbortLL_M("PutListLev1","fwrite failed");
}
/*-----------------------------------------------------------------------*/
static void PutListLev1(t_LL list)
{
  void * elem;

  Putt_LLsize(0);
  Putt_LLsize(SizeLL(list));
  ForeachLL_M(list,elem)
      PutElem(elem);
} 

/*-----------------------------------------------------------------------*/
static void PutListLevN(t_LL list, int level)
{
  if(1==level) PutListLev1(list);
  else
    {
      t_LL * pList;

      Putt_LLsize(0);
      Putt_LLsize(SizeLL(list));

      ForeachLL_M(list,pList)
          PutListLevN(*pList,level-1);
    }
} 

/*-----------------------------------------------------------------------*/
static void PutListLev2(t_LL list) {PutListLevN(list,2);}
static void PutListLev3(t_LL list) {PutListLevN(list,3);}

