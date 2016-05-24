/*------- LL Double linked list handler: printf/scanf ------------------ */
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
/*-----------------------------------------------------------------------*/
static char sccsid[]="@(#)LLstr.c		8.4	94/12/20 g.matas@ee.surrey.ac.uk";
typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/

#include "LL.h"
#include <string.h>
#include <stdlib.h>

#ifdef _OPENMP
#include <omp.h>
#endif
/*-----------------------------------------------------------------------*/
static t_LL  dbLL = NULL;
#pragma omp threadprivate (dbLL)

//void DebugLL(void) { ConsistentLL(dbLL); }
/* a dummy function used for forcing this file to be linked in */

/*-----------------------------------------------------------------------*/
char ** LL2ArrStr(t_LL list)
{
  char ** arr = malloc(sizeof(char*) * (SizeLL(list)+1));
  char  * str;
  int     i = 0;

  if(NULL==arr)AbortLL_M("LL2ArrStr","malloc returned NULL");

  ForeachLL_M(list,str)
      arr[i++] = str;

  arr[i]=NULL;
  return arr;
}
/*-----------------------------------------------------------------------*/
static char * GenPrintLL(t_LL list,char *bef,char * control,char *aft,int out);

static FILE * FiLL;
/*-----------------------------------------------------------------------*/
char * FprintLL(t_LL list, FILE * file, char *bef, char * control, char
                *aft)
{
  FiLL = file;
  return GenPrintLL(list,bef,control,aft,1);
}

/*-----------------------------------------------------------------------*/
char * printLL(t_LL list,  char * control)
{ return FprintLL(list,stdout,"",control,"\n"); }

/*-----------------------------------------------------------------------*/
/*  
#define MAX_LINE_LENGHT 300000
char BuffLL[MAX_LINE_LENGHT]; 
*/
#define MAX_LINE_LENGHT 100
char BuffLL[MAX_LINE_LENGHT];

char *stringLL, *stLL;
#pragma omp threadprivate (BuffLL,stLL,stringLL)

/*
static
 char * SprintLL(t_LL list, char * string, char *bef,char *control, char* aft)
{
   if (NULL == string)  stLL=stringLL=BuffLL;
   else                 stLL=stringLL=string;
   return GenPrintLL(list,bef,control,aft,0);
}
*/

/*-----------------------------------------------------------------------*/
static char * GenPrintLL(t_LL list, char *bef, char *control, char *aft,int out)
{
  char  conv = '%';
  char  *curr_cont_start  ,*curr_conv_start,* curr_conv_end;
  int curr_conv_length;

  char * conv_type = "diuoxXfegcsS%";
  char curr_control[200];
  void * elm, * curr_item;
  int n_char;

  if(out)       fprintf(FiLL,bef);
  else          {
      sprintf(stLL,bef);
      stLL+=strlen(bef);
    }
  ForeachLL_M(list,elm){
    curr_item = elm;
    curr_cont_start = control;

    while(1){
        if (NULL == (curr_conv_start = strchr(curr_cont_start,conv))){
            if(out)       fprintf(FiLL,curr_cont_start);
            else          {
                sprintf(stLL,curr_cont_start);
                stLL+=strlen(curr_cont_start);
              }
            /*print chars after last cont.*/
            break;                      /* find the start of conversion spec. */
          }

        if(NULL == (curr_conv_end = strpbrk(curr_conv_start+1,conv_type)))
          break;    /*  find the converstion type */

        curr_conv_length = curr_conv_end - curr_cont_start + 1;

        strncpy(curr_control,curr_cont_start,curr_conv_length);
        /* copy the part of control string with % into current */
        curr_control[curr_conv_length] = '\0';
        /* terminate the control string */

        if(!out) strcat(curr_control,"%n");

        switch (curr_control[curr_conv_length-1]){

          case 'd': case 'i' :
            if (*(curr_conv_start+1) != '*'){
                if(out)  fprintf(FiLL,curr_control,*(int *) curr_item);
                else  sprintf(stLL,curr_control,*(int *) curr_item,&n_char);
              }
            curr_item = (int *) curr_item + 1;
            break;

          case 'u': case 'o':
            if (*(curr_conv_start+1) != '*'){
                if(out)  fprintf(FiLL,curr_control,*(unsigned int *) curr_item);
                else sprintf(stLL,curr_control,*(unsigned int *) curr_item,&n_char);
              }
            curr_item = (unsigned int *) curr_item + 1;
            break;

          case 'e':  case'f': case 'g':
            if (*(curr_conv_start+1) != '*'){
                if (curr_control[curr_conv_length-2]=='l')
                  if(out)  fprintf(FiLL,curr_control,*(double*) curr_item);
                  else  sprintf(stLL,curr_control,*(double*) curr_item,&n_char);
                else
                  if(out)  fprintf(FiLL,curr_control,*(float*) curr_item);
                  else  sprintf(stLL,curr_control,*(float*) curr_item,&n_char);
              }
            if (curr_control[curr_conv_length-2]=='l')
              curr_item = (double *) curr_item + 1;
            else
              curr_item = (float *) curr_item + 1;
            break;

          case 'S':  /* an array of char */
            if (*(curr_conv_start+1) != '*'){
                curr_control[curr_conv_length-1]='s';
                if(out)  fprintf(FiLL,curr_control,(char *) curr_item);
                else  sprintf(stLL,curr_control,(char *) curr_item,&n_char);
              }
            curr_item = (char *) curr_item + strlen(curr_item)+1;
            /* skip the string */
            break;

          case 's':
            if (*(curr_conv_start+1) != '*'){
                if(out)  fprintf(FiLL,curr_control,*(char **) curr_item);
                else  sprintf(stLL,curr_control,*(char **) curr_item,&n_char);
              }
            curr_item = (char **) curr_item + 1;
            break;

          case 'c':
            if (*(curr_conv_start+1) != '*'){
                if(out)  fprintf(FiLL,curr_control,*(char *) curr_item);
                else  sprintf(stLL,curr_control,*(char *) curr_item,&n_char);
              }
            curr_item = (char *) curr_item + 1;
            break;

          case '%':
            if (*(curr_conv_start+1) != '*'){
                if(out)  fprintf(FiLL,curr_control);
                else  sprintf(stLL,curr_control,&n_char);
              }
            curr_item = (char *) curr_item + 1;
            break;

          default:
            break;
          }
        curr_cont_start=curr_conv_end + 1;
        if (!out){
            stLL += n_char;
            if (stLL - stringLL > MAX_LINE_LENGHT){
                fprintf(stderr,"string buffer overflow in SprintfLL\n");
                exit(-1);
              }
          }
      }
  }
  if(out)       fprintf(FiLL,aft);
  else          {
      sprintf(stLL,aft);
      stLL+=strlen(aft);
    }

  if (!out) return stLL;
  else      return NULL;
}

static char * StrDup (char * s)
{
  char * copy;
  if (NULL == s) { fprintf(stderr,"NULL passed to StrDup \n"); exit(-1);}
  if (NULL == (copy = malloc(strlen(s) + 1)))
    {
      fprintf(stderr,"malloc returned NULL in StrDup\n");
      exit(-1);
    }
  strcpy(copy,s);
  return copy;
}

char * SscanLL(t_LL list, char *String, char * control, int termination)
{
  char  conv = '%';
  char  *curr_cont_start  ,*curr_conv_start,* curr_conv_end;
  int curr_conv_length;

  char * conv_type = "diuoxXfegsS%";
  char curr_control[200];
  int n_char;
  char * curr_item;
  long i;
  long size=0;
  char * s = String;

  if (termination == -1){
      sscanf(s,"%d%n",&termination,&n_char);
      s+=n_char;
    }

  for(i=1;i<=termination || (termination==0); i++) {
      curr_cont_start = control;
      curr_item= BuffLL;
      while(1){
          if (NULL == (curr_conv_start = strchr(curr_cont_start,conv)))
            break;

          if(NULL == (curr_conv_end = strpbrk(curr_conv_start+1,conv_type)))
            break;    /*  find the converstion type */

          curr_conv_length = curr_conv_end - curr_cont_start + 1;

          strncpy(curr_control,curr_cont_start,curr_conv_length);
          /* copy the part of control string with % into current */
          curr_control[curr_conv_length] = '\0';
          /* terminate the control string */

          strcat(curr_control,"%n");

          switch (curr_control[curr_conv_length-1]){

            case 'd': case 'i' :
              if (*(curr_conv_start+1) != '*')
                sscanf(s,curr_control,curr_item,&n_char);
              else
                sscanf(s,curr_control,&n_char);
              size = sizeof(int);
              break;

            case 'u': case 'o':
              if (*(curr_conv_start+1) != '*')
                sscanf(s,curr_control, curr_item,&n_char);
              else
                sscanf(s,curr_control,&n_char);
              size= sizeof(unsigned int *);
              break;

            case 'e':  case'f': case 'g':
              if (*(curr_conv_start+1) != '*')
                sscanf(s,curr_control,curr_item,&n_char);
              else
                sscanf(s,curr_control,&n_char);
              if (curr_control[curr_conv_length-2]=='l') size= sizeof(double *);
              else                                       size= sizeof(float *);
              break;

            case 'S':  /* an array of char */
              curr_control[curr_conv_length-1]='s';
              if (*(curr_conv_start+1) != '*')
                sscanf(s,curr_control, curr_item,&n_char);
              else
                sscanf(s,curr_control,&n_char);
              size= strlen(curr_item)+1;
              /* skip the string */

              break;

            case 's':
              if (*(curr_conv_start+1) != '*')
                sscanf(s,curr_control, curr_item,&n_char);
              else
                sscanf(s,curr_control,&n_char);
              curr_item = StrDup(curr_item);
              size= sizeof(char *);
              break;

            case 'c':
              if (*(curr_conv_start+1) != '*')
                sscanf(s,curr_control, curr_item,&n_char);
              else
                sscanf(s,curr_control,&n_char);
              size= sizeof(char *);
              break;

            case '%':
              if (*(curr_conv_start+1) != '*')
                sscanf(s,curr_control,&n_char);
              else
                sscanf(s,curr_control,&n_char);
              size= sizeof(char *);
              break;

            default:
              break;
            }
          if (*(curr_conv_start+1) != '*')
            curr_item += size;

          s += n_char;
          curr_cont_start=curr_conv_end + 1;
        }
      InsLastLLf(list,(char*)curr_item-BuffLL,BuffLL);
      if (*s == '\0') break;
    }

  if (termination!=0 && i!=termination) return NULL;
  return s;
}
