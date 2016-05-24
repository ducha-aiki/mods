/*------ Command line parser - functions processing double parameters -- */
/*  author: G. Matas                           (g.matas@ee.surrey.ac.uk) */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1993, George Matas.                                     | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+
*/
/* 22-Feb-93, J. Matas - created */
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)optionDouble.c	3.5	94/09/02 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <ecompat.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "optionGM.h"
#include "optionPriv.h"

#define BUF_SIZE 100000

char * ConsStr(char * format, ...)
{
  va_list args;

  int converted;
  char buff[BUF_SIZE];  

  va_start(args,format);
  converted = vsprintf(buff,format,args);

  if (converted >= BUF_SIZE)
  {
    fprintf(stderr,"string to long in ConsStr\n");
    exit(-1);
  }

  va_end(args);

  return strdup(buff);
}

/*------- get a single double  -------------------------------*/
double OptionDouble(char * name, double def, char * comment)
{
  double value = def;
  char ** option;

  IsInitialized();

  if (NULL != (option=GetOption(name,1)))
    if (1 != sscanf(option[1],"%lf",&value)) ErrIncorrectType(name);

  Usage(ConsStr("%s (%.3f) [%.3f] %s",OptName(name),value,def,comment));

  return value;
}

/*--------  get an integer pair   -----------------------------*/
void
OptionDoubleDouble
 (char * name, double *v1,double *v2, double def1, double def2, char * comment)
{
  char ** option;

  IsInitialized();

  * v1 = def1;
  * v2 = def2;

  if (NULL != (option=GetOption(name,2)))
  {
    if (1 != sscanf(option[1],"%lf ",v1)) ErrIncorrectType(name);
    if (1 != sscanf(option[2],"%lf ",v2)) ErrIncorrectType(name);
  }     

  Usage(ConsStr("%s (%.3f %.3f) [%.3f %.3f]  %s",
               OptName(name),*v1,*v2,def1,def2,comment)); 

}

/*---------  get an double array    ---------------------------*/
void  OptionDoubleArr(char *name,double *arr,int elems,char * comment)
{
  char ** option;
  int i;
  int optFound = 0;
  char buff[1000];
  
  IsInitialized();

  if (NULL != (option=GetOption(name,elems)))
  {   
    for(i=0; i<elems;i++)
      if (1 != sscanf(option[i+1],"%lf ",&arr[i]))
      {
	ErrIncorrectType(name);
	break;
      }
    if (i == elems) optFound = 1;
  }

  /* a more efficient version with %n instead of strlen didn't work ??gcc */

  sprintf(buff,"%s (",OptName(name)); 
  for(i=0;i<elems;i++)
    sprintf(buff+strlen(buff),"%.3f ",arr[i]);
  sprintf(buff+strlen(buff),") [?] %s",comment);

  Usage(strdup(buff));
}
