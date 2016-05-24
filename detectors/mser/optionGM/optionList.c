/*------ Command line parser - function for list parameters --- */
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
/*-----------------------------------------------------------------------*/
static char sccsid[]="@(#)optionList.c	3.5	94/09/02 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <ecompat.h>
#include <stdio.h>
#include <string.h>

#include <LL.h>
#include "optionGM.h"
#include "optionPriv.h"

t_LL  OptionLL(char *name,char * comment)
{
  char ** option;
  t_LL list = ConsLL();
  char * str;
  char buff[1000];
  int i;
   
  IsInitialized();

  if (NULL != (option=GetOption(name,-1)))
  {   
    for(i=1; NULL !=option[i] ;i++)
      InsLastLLf(list,strlen(option[i])+1,option[i]);
  }

  /* a more efficient version with %n instead of strlen didn't work ??gcc */

  sprintf(buff,"%s (",OptName(name)); 
  ForeachLL_M(list,str)
    sprintf(buff+strlen(buff),"%s ",str);
  sprintf(buff+strlen(buff),") [?] %s",comment);

  Usage(strdup(buff));

  return list;
}
