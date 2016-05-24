/*------ Command line parser - functions processing misc. parameters --- */
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
static char sccsid[]="@(#)optionMisc.c	3.5	94/09/02 g.matas@ee.surrey.ac.uk";
   typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <ecompat.h>
#include <stdio.h>
#include <string.h>
#include "optionGM.h"
#include "optionPriv.h"

/*-------------------------------------------------------------*/
static char fixNull[5]="null";
static char * FixNull(char * s) { return (NULL == s) ? fixNull : s; }
char * OptionStr(char * name, char * def, char * comment)
{
  char * value = def;
  char ** option;

  IsInitialized();

  if (NULL != (option=GetOption(name,1)))
     value = strdup(option[1]);

  Usage(ConsStr( "%s (%s) [%s] %s",
	  OptName(name),FixNull(value),FixNull(def),comment));

  return value;
}


/*-------------------------------------------------------------*/
int OptionToggle(char * name, int def, char * comment)
{
  int value = def;
  char ** option;

  IsInitialized();

  if (NULL != (option=GetOption(name,0)))
    value = (def == 1) ? 0 : 1;         /* if option present negate */

  Usage(ConsStr("%s (%d) [%d] %s",OptName(name),value,def,comment));

  return value;
}

/*-------------------------------------------------------------*/
unsigned char OptionChar(char * name, unsigned char def, char * comment)
{
  unsigned char value = def;
  char ** option;

  IsInitialized();

  if (NULL != (option=GetOption(name,1)))
    if (1 != sscanf(option[1],"%c",&value)) ErrIncorrectType(name);

  Usage(ConsStr("%s (%c) [%c] %s",OptName(name),value,def,comment));

  return value;
}

