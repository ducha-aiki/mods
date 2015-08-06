/*------ Command line parser - functions processing int parameters ----- */
/*  author: G. Matas                           (g.matas@ee.surrey.ac.uk) */
/* +-------------------------------------------------------------------+ */
/* | Copyright 1993, George Matas.                                     | */
/* |   Permission to use, copy, modify, and distribute this software   | */
/* |   and its documentation for any purpose and without fee is hereby | */
/* |   granted, provided that the above copyright notice appear in all | */
/* |   copies and that both that copyright notice and this permission  | */
/* |   notice appear in supporting documentation.  This software is    | */
/* |   provided "as is" without express or implied warranty.           | */
/* +-------------------------------------------------------------------+ */
/* 22-Feb-93, J. Matas - created                                         */
/*-----------------------------------------------------------------------*/
static char sccsid[]="@(#)optionInt.c	3.5	94/09/02 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <ecompat.h>
#include <stdio.h>
#include <string.h>

#include "optionGM.h"
#include "optionPriv.h"

/*------- get a single integer  -------------------------------*/
int OptionInt(char * name, int def, char * comment)
{

  int value = def;
  char ** option;

  IsInitialized();

  if (NULL != (option=GetOption(name,1)))
    if (1 != sscanf(option[1],"%d",&value)) ErrIncorrectType(name);

  Usage(ConsStr("%s (%d) [%d] %s",OptName(name),value,def,comment)); 

  return value;
}
 
/*--------  get an integer pair   -----------------------------*/
void
OptionIntInt(char * name, int *v1,int *v2, int def1, int def2, char * comment)
{
  char ** option;

  IsInitialized();

  * v1 = def1;
  * v2 = def2;

  if (NULL != (option=GetOption(name,2)))
  {
    if (1 != sscanf(option[1],"%d ",v1)) ErrIncorrectType(name);
    if (1 != sscanf(option[2],"%d ",v2)) ErrIncorrectType(name);
  }     

  Usage( ConsStr("%s (%d %d) [%d %d]  %s",
            OptName(name),*v1,*v2,def1,def2,comment)); 
}

/*---------  get an integer array    ---------------------------*/
void  OptionIntArr(char *name,int *arr,int elems,char * comment)
{
  char ** option;
  int i;
  int optFound = 0;
  char buff[1000];
  
  IsInitialized();

  if (NULL != (option=GetOption(name,elems)))
  {   
    for(i=0; i<elems;i++)
      if (1 != sscanf(option[i+1],"%d ",&arr[i]))
      {
	ErrIncorrectType(name);
	break;
      }
    if (i == elems) optFound = 1;
  }

  /* a more efficient version with %n instead of strlen didn't work ??gcc */

  sprintf(buff,"%s (",OptName(name)); 
  for(i=0;i<elems;i++)
    sprintf(buff+strlen(buff),"%d ",arr[i]);
  sprintf(buff+strlen(buff),") [?] %s",comment);

  Usage(strdup(buff));
}
