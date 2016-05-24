#ifndef LL_h
#define LL_h
/*---------- Double linked list handler  ---------------------------------- */
/*  author: G. Matas   (g.matas@ee.surrey.ac.uk)                            */
/*
      G. Matas, 30-Dec-93 v5.3
         - history list deleted; available in the 5.5 delta.
           The list became redundant as LL.h was put under SCCS control.

  sccs: "@(#)%E% g.matas@ee.surrey.ac.uk %I% %M%"
*/

/*  based on a link library by Duane Morse (the circularity trick)          */
/*--------------------------------------------------------------------------*/
#undef __STRICT_ANSI__
#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {               /* can be used directly from C++ */
#endif

#include <stdio.h>                     /* to get NULL */
#include <stddef.h>                    /* to get size_t  */

#define t_ELMsize unsigned long
/* must be large enough to hold sizof the large list elem */
#define t_LLsize unsigned long
/* must be large enough to hold size of the largest list  */
/* if space saving is important, could be reduced to int or char */
#define t_LLrelsize long

typedef  struct s_list
{
  struct s_list *forward;
  struct s_list *backward;
  t_ELMsize size;        /* size of elmement stored; 0 for the list head */
} l_list;

typedef struct
{
  union
  {
    l_list ll;      /* the linking stuff */

    void *v;
    char c;
    unsigned char uc;
    short s;
    unsigned short us;
    int i;
    unsigned int ui;
    long l;
    unsigned long ul;
    float f;
    double d;

    /* this union makes sure that s_list is properly aligned  for all types
             * listed in the union. Therefore s_list +  sizeof(s_list) =
             * <the address where user data /list contents/ start>
             * will be properly aligned for any of the types
             *
             * NOTE: Because of alignment, additional 'padding' bytes will be added
             *       to s_list; if SPACE SAVINGS are important, and you are SURE
             *       that this lib will be used for a limited subset of types, delete
             *       redundant fields (this may/may not save space)
             */
  } u;
} t_linkLL ;

typedef struct s_LL
{
  t_linkLL  links;
}      *t_LL;

/*------------------- error handling       ------------------------------- */
#define AbortLL_M(where,error)\
{fprintf(stderr,"In %s: %s \n",where,error); exit (-1); }

/*------------------- basic list functions ------------------------------- */
/*--------------------------------------------------------------------------*/
t_LL   ConsLL (void);              /*                   list constructor   */
t_LL   InitLL(struct s_LL* head);  /*  init head, the list is its addr.    */

void * DestLL (t_LL list);         /*                   list destructor    */

t_LL ConsCopyLL(t_LL src);   /* construct a copy of src                    */
t_LL ConsPtrLL (t_LL src);   /* construct a list of pointers to data in src*/

t_LL   EmptyLL(t_LL list);         /* delete all elmements from a list     */
int    IsEmptyLL(t_LL list);       /* test for an empty list               */
int    IsShorterThanLL(t_LL list, int l); /*      */

t_LL   ReverseLL(t_LL list);

void * ApplyLL(t_LL list, void * (*apply) (void *));
/* apply a function to every elmement   */

#define SortLL MergeSortLL
t_LL  MergeSortLL(t_LL list,  int (*compar) (const void *, const void*));
t_LL  SysSortLL(t_LL list,  int (*compar) (const void *, const void*));
/* sort the list according to compare function */
/* compare receives    p_element (same as qsort)*/

t_LL MergeSortPassLL(t_LL l1, t_LL l2,  int (*cmp)(const void *, const void*));
/* assumes l1 and l2 sorted, moves elems of l2 into l1 in */
/* sorted order. l2 is emptied by the procedure */


t_LLsize SizeLL(t_LL list); /* return the number of  elmements in  a list */

void * LookInLL(t_LL list);
/* create a look-up table for random access into list */
/* to get n-th element, write Look[n] (after Look=..  */


/*------- Read/Write into a text file (=for list of char *) --------------*/
t_LL File2LL(char * name);            /* exit   if fopen fails */
t_LL FileNoExit2LL(char * name);      /* return empty list if fopen fails */
void LL2File(t_LL list, char * name);
char ** LL2ArrStr(t_LL list);

/*------- Read/Write into .LL file (=external representation) --------------*/
t_LL ReadLL(char * filename);

void WriteLev1LL(char * f_name, t_LL list); /* write a list of simple elems.*/
void WriteLev2LL(char * f_name, t_LL list); /* write a list of lists */
void WriteLev3LL(char * f_name, t_LL list); /* write l of l of l of elms*/
void WriteLevNLL(char * f_name, t_LL list, int l); /* write list of lev l*/

/*-------Insert/Delete     elmements ---------------------- */
/* INSERT a  new  elmement in the  list                                     */
/*    Bef/Aft    - before/after a given   p_element                         */
/*    first/Last - as a first or last elmement of the list                  */
/*         returns address of the new element                               */

#define InsBefLL(p_el,data)   InsBefLLf(p_el,   sizeof(data), &data)
#define InsAftLL(p_el,data)   InsAftLLf(p_el,   sizeof(data), &data)
#define InsFirstLL(list,data) InsFirstLLf(list,   sizeof(data), &data)
#define InsLastLL(list,data)  InsLastLLf(list,   sizeof(data), &data)

void * InsBefLLf (void * p_elm, size_t size, void * data);
void * InsAftLLf (void * p_elm, size_t size, void * data);
void * InsFirstLLf (t_LL list, size_t size, void * data);
void * InsLastLLf (t_LL list,  size_t size, void * data);

#define LinkInsBefLL(p_el,data)   LinkInsBefLLf(p_el,  sizeof(data),&(data))
#define LinkInsAftLL(p_el,data)   LinkInsAftLLf(p_el,  sizeof(data),&(data))
#define LinkInsFirstLL(list,data) LinkInsFirstLLf(list,sizeof(data),&(data))
#define LinkInsLastLL(list,data)  LinkInsLastLLf(list, sizeof(data),&(data))

void * LinkInsFirstLLf(t_LL list, size_t size, void * newEl);
void * LinkInsLastLLf(t_LL list,  size_t size, void * newEl);
void * LinkInsAftLLf(void * curr, size_t size, void * newEl);
void * LinkInsBefLLf(void * curr, size_t size, void * newEl);

void  DelElmLL   (void * p_elm);        /* Delete   p_element from the list */
void * DelElmNeLL(void * p_elm);        /* Delete p_elem, return p to next */
void * DelElmPrLL(void * p_elm);        /* Delete p_elem, return p to prev */
/*--------------------------------------------------------------------------*/

/*-------- Moves (Cut & Paste) involving 2 lists ------------------------*/
/* NOTE: for all moves: pointers to moved elems are still valid          */

/* move the whole list  to dest, src becomes empty */
t_LL  MoveListFirstLL(t_LL  dest, t_LL src);
t_LL  MoveListLastLL(t_LL  dest, t_LL src);
void *  MoveListAftLL(void *el,  t_LL src);
void *  MoveListBefLL(void *el,  t_LL src);

/* move head (elements from start to head <excluding> to dest */
/* head must be an element of src ! */
t_LL  MoveHeadFirstLL(t_LL  dest, t_LL src, void *head);
t_LL  MoveHeadLastLL(t_LL  dest, t_LL src, void *head);
void *  MoveHeadAftLL(void *el,  t_LL src, void *head);
void *  MoveHeadBefLL(void *el,  t_LL src, void *head);

/* move tail (elements from tail (including) to end of list to dest */
/* tail must be an element fo src! */
t_LL  MoveTailFirstLL(t_LL  dest, t_LL src, void *tail);
t_LL  MoveTailLastLL(t_LL  dest, t_LL src, void *tail);
void *  MoveTailAftLL(void *el,  t_LL src, void *tail);
void *  MoveTailBefLL(void *el,  t_LL src, void *tail);

/*--------------    Moves of Element ptrs-----------------------------------*/
/*    get (move to) the First/Last/                                         */
/*                  Nth    - n-th   element in the list                     */
/*                  RelNth - n-th   p_element after the given one           */

void * FirstElmLL (t_LL list);
void *  LastElmLL (t_LL list);
void *   NthElmLL (t_LL list, t_LLsize num);
void *  RelNthElmLL (void * p_elm, t_LLrelsize num);
void *  NextElmLL (void * p_elm);
void *  PrevElmLL (void * p_elm);
void *  NextCElmLL (void * p_elm);
void *  PrevCElmLL (void * p_elm);
void *  RelCNthElmLL (void * p_elm, t_LLrelsize num);

t_LLsize  IndexElmLL    (t_LL list, void *ind_el); /* element position */

/*---------- macros for scanning through a list --------------------------*/
#define ForeachLL_M(list,p_elm)\
  for(p_elm=FirstElmLL(list); IsElmLL(p_elm); p_elm=NextElmLL(p_elm))

#define ForeachTyLL_M(list,p_elm,type)\
  for(p_elm=(type)FirstElmLL(list); IsElmLL(p_elm); p_elm=(type)NextElmLL(p_elm))

#define ForeachDownLL_M(list,p_elm)\
  for(p_elm=LastElmLL(list); IsElmLL(p_elm); p_elm=PrevElmLL(p_elm))

#define SafeForeachLL_M(list,p_elm,next_p_elm)\
  for(p_elm=FirstElmLL(list); IsElmLL((void *)p_elm); p_elm=next_p_elm)

/* Ordered Pairs */
#define OrderedPairsLL_M(list,prev_elm,p_elm)\
  for(p_elm=NextElmLL(prev_elm); IsElmLL(p_elm); p_elm=NextElmLL(p_elm))


/*-------------- Linking in/out from a list -------------------------------*/
void * LinkFirstLL(t_LL list,void * newEl);
void * LinkLastLL(t_LL list,void * newEl);
void * LinkAftLL(void * curr,void * newEl);   /* link new after current */
void * LinkBefLL(void * curr,void * newEl);   /* link new before current */

void * UnlinkLL(void * el);
void * UnlinkNeLL(void * el);
void * UnlinkPrLL(void * el);

/*---------------------- Misc. functions-------------------------------------*/
int   IsElmLL  (void * p_elm);      /* Test for the end of the list    */
int   IsFirstElmLL  (void * p_elm); /* is p_elm the last elm in the list? */
int   IsLastElmLL   (void * p_elm); /* is p_elm the first elm in the list?*/
int   IsNthElmLL(t_LL list, t_LLsize num,void *el);
/* test for the n-th element */

/*----------------- debugging etc.  ---------------------------------------*/
void DebugLL( void   );          /* make sure ConsistentLL and Print/Scan */
/* are linked in */
void ConsistentLL(t_LL list);     /* check if the list structure looks OK */
/*----------- conversion to/from a string ----------------------------------*/
/* WARNING: the functions bellow are NOT PORTABLE and NOT SAFE, if used
     *          for lists containing STRUCTURES.  They work for all primitive
     *          types (int, char *, double, float, ..)  and for those structures
     *          that don't have 'padding' bytes inside because of alignment;
     *          (whether a structure contains these bytes is compiler/OS depend.)
     *            I recommend these functions be used only with a single
     *          conversion specification (eg. "%s" or "%d"). Conversions with
     *          more than one spec. should be used for DEBUGGING purposes
     *          (called directly from the debug. command line) or possibly
     *          for fast prototyping and I don't recommend their use in final code.
     */
char * FprintLL(t_LL list, FILE * file,char *bef,char *control, char *aft);
char * printLL(t_LL list,  char * control);
char * SscanLL(t_LL list, char *string, char * control, int termination);

/* disabled. Couldn't make it safe because of internal buffer size problems
    char * SprintLL(t_LL list, char * string, char *bef, char *control, char *aft);
    */

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif

#endif
