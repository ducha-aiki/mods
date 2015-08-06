/*---------- Command line parser - core -------------------------------- */
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
/*
  23-Jun-93, J. Matas
  - put under SCCS control, versID string replaced by sccsID

  12-Mar-92, J. Matas
  - function calls that had sprintf(optBuf, ... ) as a parameters
  replaced by ConsStr; 1. to orig. was not ANSI conformant 
  (assuming that sprintf returns char *) 2. optBuf was made
  public (unsage, difficult to check if not overwritten)

  1-Mar-93, J. Matas
  - function OptionIf() added

  18-Feb-93, J. Matas 
  - created
*/
/*---------------------------------------------------------------------*/
static char sccsid[]="@(#)option.c	3.9	95/02/01 g.matas@ee.surrey.ac.uk";
typedef char _s_foo[sizeof(sccsid)];/*stop gcc warning: unused var sccsid*/ 

#include <ecompat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "optionGM.h"
#include "optionPriv.h"

static int     optInitialized = 0;   /* has the parser been initialized ?*/
static int     copiedArgs     = 0;   /* option lib works on copied args */

static char ** currentArgv = NULL;   /* array of unprocessed options and pars*/
static int   * pcurrentArgc= 0;      /* number of unprocessed opt.  */

static char ** rerunArgv   = NULL;   /* array of unprocessed options -rerun*/
static int     rerunArgc   = 0;      /* */
static int   * prerunArgc  = &rerunArgc;  /* number of unprocessed opt. -rerun*/
static int     rerun       = 0;

static char ** used;                 /* array of processed options and pars. */
static int     usedc;                /* no. of processed opt. and pars.  */

#define MAX_ERRC     50
static char * errv[MAX_ERRC];        /* array of error messages */
static int     errc = 0;             /* no. of error messages */

#define MAX_OPTIONS 100
static char *  usage[MAX_OPTIONS];   /* array containing complete help/usage */
static int     usagec=0;             /* no. of entries in the help      */

#define MAX_DEPEND 50
static char *  dep[MAX_DEPEND];      /* array of dependency information */
static int     depc=0;               /* current no. of dependencies */


/*------------ Usage Registration --------------------------------------*/ 
void Usage(char * s)
/* append a string into the usage array */
{
   if (usagec+1 >= MAX_OPTIONS)
   {
      fprintf(stderr,"Too many options! See ees2gm\n"); exit(-1);
   }
   usage[usagec]  =s;
   usagec++;
   usage[usagec]=NULL;
}

void OptionAddUsage(char * s)
{
   Usage(s);
}

/*------------- Useful utilites ----------------------*/
#define EMPTY_LENGTH 6
static char emptyName[EMPTY_LENGTH] ="arg_1";
char * OptName(char * name)
/* Fix option name, forced by the usage of "" options (options without */
/* specifier);the empty string must be converted to something visible  */
{
   static char nameBuff[100];
   if (name[0] =='\0')              /* empty string, convert  */
   {
      strcpy(nameBuff,emptyName);
      emptyName[EMPTY_LENGTH-2]++;
   }
   else                             /* else put the '-' prefix */
      sprintf(nameBuff,"-%s",name);

   /*return DupStr(nameBuff);*/       /* should be freed it somewhere */
   return nameBuff;   /* does not leak, but values are valid till next call!  */
}


static int IsPar(char *s) 
/* defines which comm. line arguments are option specifiers and 
 * which are consider to be parameters 
 * PARAMETERs must not start with -, unless it is a single character 
 * '-' (allowing typical definition of pipe in/out) or the '-' is followed
 * by a digit (assuming a negative number 
 * Consequently, OPTIONS start with a '-' followed by at least one  letter
 */
   
{
   if (s[0]!='-' || s[1]=='\0' || isdigit(s[1])) return 1;
   else                         return 0;
}

/*-- does string 's' match option 'name'(passed in without leading -) ?----*/
static int MatchOption(char *s, char * name)
{
   if ((!IsPar(s)) && (!strcmp(name,&s[1]))) return 1;
   else return 0;
}

/*--- get the number of pars following an option in position 'pos' ---*/
/*static int NumOfPars(char * name, int pos)*/
static int NumOfPars(int pos)
{
   int i;

   for (i=pos+1;i<*pcurrentArgc;i++)
      if (!IsPar(currentArgv[i])) break;
  
   return i-(pos+1);
}
/*------------ Error Checking and  Registration ----------------------------*/ 
void IsInitialized(void )
{
   if (optInitialized == 0)
   {
      fprintf(stderr,"Command line processing not initialized!\n");
      exit(-1);
   }
}
   
static void ErrRegister(char * s)    /* append string s to error array */
{
   if (errc >= MAX_ERRC)
   {
      fprintf(stderr,"Too many errors!\n"); exit(-1);
   }
   errv[errc++]=s;
}

/*-------------------------------------------------------------*/
static int MultipleRequest(char * name)
/* check if option name hasn't been already processed, ie.
 * if OptionXX name hasn't been called more than once for
 */
{
   int i;
   size_t name_length = strlen(name);

   if ('\0' == name[0]) return 0; /* <NO_OPT> can be multiply  requested */

   for(i=0;i<usagec;i++)      /* assumes usage always starts with -option */
      if(   !strncmp(&usage[i][1],name,name_length)
            && usage[i][name_length+1]==' ')
      {
         ErrRegister(ConsStr("Option %s processed more than once",OptName(name)));  
         return 1;
      }
   return 0;
}

/*-------------------------------------------------------------*/
static int NotEnoughPars(char * name, int expected, int pars)
{ 
   if (expected >pars)
   {
      ErrRegister(ConsStr("Not enough parameters for option %s",OptName(name)));
      return 1;
   }
   return 0;
}

/*-------------------------------------------------------------*/
static void ErrCompulsory(char * name)
{
   ErrRegister(ConsStr("Missing compulsory option %s",OptName(name)));
}

/*-------------------------------------------------------------*/
void ErrIncorrectType(char * name)
{
   ErrRegister(ConsStr(
                  "Incorrect type of parameters in option %s",OptName(name)));
}

/*-------------------------------------------------------------*/
static int MultipleSpec(char * name)
/* check if this option hasn't appeared more then once on the comm. line */
{
   int i;

   if ('\0' == name[0]) return 0; /* <NO_OPT> can be multiply  spec. */

   for(i=0;i<usedc;i++)
      if (MatchOption(used[i],name))
      {
         ErrRegister(ConsStr( "Option %s used more then once",OptName(name)));
         return 1;
      }

   return 0;
}

/*------------ useful private functions --------------------------------*/
static int FindOption(char *name)
{
   int i;

   if (name[0] != '\0')
   {
      for(i=1;i<*pcurrentArgc; i++)
         if (MatchOption(currentArgv[i],name)) return i; 
   }
   else   /* for "" option any parameter is the value */
   {
      for(i=1;i<*pcurrentArgc; i++)
         if(IsPar(currentArgv[i])) return i-1; 
   }
  
   return -1;
}

static int FindReRunOption(char *name)
{
   int i;

   if (name[0] != '\0')
   {
      for(i=1;i<*prerunArgc; i++)
         if (MatchOption(rerunArgv[i],name)) return i; 
   }
   else   /* for "" option any parameter is the value */
   {
      for(i=1;i<*prerunArgc; i++)
         if(IsPar(rerunArgv[i])) return i-1; 
   }
  
   return -1;
}

static int FindUsedOption(char * name)
{
   int i;
   for(i=0;i<usedc;i++)
      if (MatchOption(used[i],name)) return i; 

   return -1;
}
  

/*-------------------------------------------------------------*/
static int numArgs = 0;   /* count the number of empty options */
static char **  MoveOption(int position,int pars,char * name, 
                           char ** tkn, int * pNum)
{
   int i;
 
   if('\0' == name[0])                /* "" doesn't have a specifier, adjust*/
   {
      numArgs++;
      position++;
      pars--;
   }

   for(i=position;i<=position+pars;i++)  /* copy option + pars into used */
      used[usedc++] = tkn[i];
   used[usedc] = NULL;  /* NULL terminations enables to find out */
   /* the number of args passed out (useful for list */

   for(i=position+pars+1	;i< *pNum;i++)  /* shift option in input */
      tkn[i-pars-1]=tkn[i];
  
   * pNum -= (pars+1)	;

   if('\0' == name[0]) pars++;       /* "" doesn't have a specifier, adjust */

   return &used[usedc-pars-1];
}

/*-------------------------------------------------------------*/
char ** GetOption(char * name,int expectedPars)
{
   int position;
   int pars; 

   if(NULL == name) 
   {fprintf(stderr,"NULL passed as option name!!\n"); exit(-1);};

   position = FindOption(name);
 
   if (MultipleRequest(name))               return NULL;
   if (-1==position)
   {
      if (!rerun) return NULL;

      position = FindReRunOption(name);
      if (-1 == position) return NULL;
      return MoveOption(position,expectedPars,name,rerunArgv,prerunArgc);
   }

   /* pars = NumOfPars(name,position); */
   pars = NumOfPars(position); 

   if (-1 == expectedPars ) expectedPars=pars;
   /* -1 pars means as many pars as can be found (useful for lists)*/

   if (NotEnoughPars(name,expectedPars,pars)  )
   {
      MoveOption(position,pars,name,currentArgv,pcurrentArgc);
      return NULL; 
   }
   if (MultipleSpec(name))
   {
      MoveOption(position,expectedPars,name,currentArgv,pcurrentArgc);
      return NULL; 
   }

   return MoveOption(position,expectedPars,name,currentArgv,pcurrentArgc);
}
/*------------ Public funtions ---------------------------------------*/
void OptionInit(char ** orgv, int * orgc)
{

   currentArgv = orgv;
   pcurrentArgc= orgc;

   if (optInitialized == 1)
   {
      fprintf(stderr,
              "Command line processing re-initializition permitted only \n"
              "after a call to OptionClose!\n ");
      exit(-1);
   }
   if (NULL == (used = (char**)malloc(sizeof(char *) * (*orgc))))
   {fprintf(stderr,"Not enough memory in Init\n"); exit(-1);};
   /*  cast malloc's return to conform to C++ */
   optInitialized=1;
}

/*-------------------------------------------------------------*/
void OptionInitCopy(char ** orgv, int * orgc)
{
   int i;
   char ** copiedOrgv;
   static int copiedOrgc ;
 
   copiedArgs = 1;
   copiedOrgc = * orgc;

   if (NULL == (copiedOrgv = (char**)malloc(sizeof(char *) * (*orgc))))
   {fprintf(stderr,"Not enough memory in Init\n"); exit(-1);};
   /*  cast malloc's return to conform to C++ */
    
   for(i=0;i<*orgc;i++)
      copiedOrgv[i]=orgv[i];

   OptionInit(copiedOrgv,&copiedOrgc);
}

/*-------------------------------------------------------------*/
/*#if 0*/
void OptionClose(void)
{
   int i;

   IsInitialized();

   /*--- clean-up ----*/
   for(i=0;i<usagec;i++) free(usage[i]);
   usagec=0;
  
   for(i=0;i<depc;i++) free(dep[i]);
   depc=0;

   for(i=0;i<errc;i++) free(errv[i]);
   errc=0;

   free(used);
   usedc=0;

   if (copiedArgs) free(currentArgv);

   if(rerun)
   {
      /* rerunArgv is leaking */
      rerun = 0;
   }

   currentArgv    = NULL;
   optInitialized = 0;
   copiedArgs     = 0;
}
/*#endif*/
/*-------------------------------------------------------------*/
/* unprocessed stuff not considered an error */
static int optionLeftOK = 0;
void OptionLeftOK(void) { optionLeftOK = 1;}


void OptionCheck(void)
{
   IsInitialized();
   {
      int i;
      int help       = OptionToggle("help",0,"print out usage info");
      int printUsage =                            /* Print Usage info when:  */
         ((*pcurrentArgc > 1) && !optionLeftOK) || 
         /* unproc. opt. left on cmdline (but see OptionLeftOK)*/
         errc != 0 ||                   /* errors detected (eg. wrong pars)*/
         help;                                           /* help requested */

      if (printUsage)
      {
         fprintf(stderr,"\n");
         fprintf(stderr,"Usage: %s [options]\n",currentArgv[0]);
         for(i=0;i<usagec;i++)
            fprintf(stderr,"    %s\n",usage[i]);
         fprintf(stderr,"Dependencies:\n");
         for(i=0;i<depc;i++)
            fprintf(stderr,"    %s\n",dep[i]);
      }
      
      /* NOTE: diagnostics (missing compulsory opions etc) printed  */
      /*       only  when -help NOT specified                       */
      
      if(help == 0)
      {
         if ( ((*pcurrentArgc > 1) && !optionLeftOK))
         {
            fprintf(stderr,"Unknown (unprocessed) options and parameters:\n  ");
            for(i=1;i<*pcurrentArgc;i++) fprintf(stderr,"%s ",currentArgv[i]);
            fprintf(stderr,"\n");
         }

         if (errc != 0) 
         {             
            fprintf(stderr,"Errors detected during  option parsing:\n");
            for(i=0;i<errc;i++) fprintf(stderr,"    %s\n",errv[i]);
         }
      }

      if (printUsage)
      {
         fprintf(stderr,"\n");
         exit(-1);
      }
   }
}

void OptionCheckDesc(char *desc)
{
   IsInitialized();
   {
      int i;
      int help       = OptionToggle("help",0,"print out usage info");
      int printUsage =                            /* Print Usage info when:  */
         ((*pcurrentArgc > 1) && !optionLeftOK) || 
         /* unproc. opt. left on cmdline (but see OptionLeftOK)*/
         errc != 0 ||                   /* errors detected (eg. wrong pars)*/
         help;                                           /* help requested */

      if (printUsage)
      {
         fprintf(stderr,"\n");
         fprintf(stderr,"Usage: %s [options]\n",currentArgv[0]);
         for(i=0;i<usagec;i++)
            fprintf(stderr,"    %s\n",usage[i]);
         fprintf(stderr,"Dependencies:\n");
         for(i=0;i<depc;i++)
            fprintf(stderr,"    %s\n",dep[i]);
         fprintf(stderr,desc);
      }
      
      /* NOTE: diagnostics (missing compulsory opions etc) printed  */
      /*       only  when -help NOT specified                       */
      
      if(help == 0)
      {
         if ( ((*pcurrentArgc > 1) && !optionLeftOK))
         {
            fprintf(stderr,"Unknown (unprocessed) options and parameters:\n  ");
            for(i=1;i<*pcurrentArgc;i++) fprintf(stderr,"%s ",currentArgv[i]);
            fprintf(stderr,"\n");
         }

         if (errc != 0) 
         {             
            fprintf(stderr,"Errors detected during  option parsing:\n");
            for(i=0;i<errc;i++) fprintf(stderr,"    %s\n",errv[i]);
         }
      }

      if (printUsage)
      {
         fprintf(stderr,"\n");
         exit(-1);
      }
   }
}

/*-------------------------------------------------------------*/
char** OptionUsage(void)
{
   IsInitialized();
   return usage;
}
/*-------------------------------------------------------------*/
#include <time.h>
void OptionSave(char * fname)
{
   FILE*f = (fname[0]=='-')?stdout:fopen(fname,"w");
  
   if(NULL==f)
   {
      fprintf(stderr,"Can't open %s in OptionSave!\n",fname); exit(-1);
   }

   {
      int i;
      time_t  now = time(NULL);

      fprintf(f,"%s\n%s",currentArgv[0],ctime(&now));
      for(i=0;i<usagec;i++)
      {
         char  wasUsed = ' ';
         char   name[1000];
         if('-' == usage[i][0])
         {
            char * s ;
            strcpy(name,&usage[i][1]);
            s = strchr(name,' ');

            if (NULL != s) *s = '\0';
            if (OptionOnCommLine(name)) wasUsed='!';
         }
         else  /* argument arg_X, X in [5]*/
            if (usage[i][4]-'0'<=numArgs) wasUsed='!';

         fprintf(f,"%c%s\n",wasUsed,usage[i]);
      }
   }
   if(stdout!=f)fclose(f);
}
/*------------------------------ dependency Check ------------------------*/ 
static void DepRegister(char * s)
{
   if (depc+1 >= MAX_OPTIONS)
   {
      fprintf(stderr,"Too many dependencies! See ees2gm\n"); exit(-1);
   }
   dep[depc]  =s;
   depc++;
   dep[depc]=NULL;
}

/*-------------------------------------------------------------*/
static int OptionNumbers(char * options)
{
   int i;
   int matches = 0;
   char optionUsed[100];
   char opt[200];


   sprintf(opt," %s ",options);
   for(i=0;i<usedc;i++)
      if(!IsPar(used[i]))
      {
         sprintf(optionUsed," %s ",&used[i][1]);      
         if(NULL != strstr(opt	,optionUsed)) matches++;
      }
   
   for(i=1;i<*pcurrentArgc; i++)
      if(!IsPar(currentArgv[i]))
      {
         sprintf(optionUsed,"%s ",&currentArgv[i][1]);      
         if(NULL != strstr(opt,optionUsed)) matches++;
      }

   return matches;
}

/*-------------------------------------------------------------*/
int  OptionOnCommLine(char * name)
{
   return ( OptionNumbers(name) > 0);
}

/*-------------------------------------------------------------*/
void OptionDependXor(char * xor_opt)
{
   int matches;

   IsInitialized();

   DepRegister(ConsStr("Options '%s' are mutually exclusive",xor_opt));
  

   matches = OptionNumbers(xor_opt);
   if(matches>1)
      ErrRegister(ConsStr("%d of mutally exclusive options '%s' specified",
                          matches,xor_opt));
}
/*-------------------------------------------------------------*/
void OptionIf(int enableCond, char * depend, char * comment)
{
   IsInitialized();

   DepRegister(ConsStr( "Option %s can be used only if: %s",
                        OptName(depend), comment)); 

   if (enableCond) return;

   if (-1 != FindUsedOption(depend) || -1 != FindOption(depend))
      ErrRegister(ConsStr( "Option %s can be used only if: %s",
                           OptName(depend), comment));
} 

void OptionMultIf(int enableCond, char * depend, char * comment)
{
   int matches;

   IsInitialized();

   DepRegister(ConsStr("Options '%s' can be used only if: %s", depend, comment));

   if (enableCond) return;

   matches = OptionNumbers(depend);
   if(matches>0)
      ErrRegister(ConsStr( 
                     "Options '%s' can be used only if: %s", depend, comment));
}

/*-------------------------------------------------------------*/
void OptionDependIf(char * cond, int enableVal, int val, char * depend)
{
   IsInitialized();

   {
      char * dep = ConsStr("%s",OptName(depend));
      /* second call to  OptName  would overwrite the return string */;
      DepRegister(ConsStr(
                     "Option %s can be used only when option %s is %d('%c')",
                     dep, OptName(cond), enableVal,enableVal)); 
      free(dep); 
   }

   if (val == enableVal) return;

   if (-1 != FindUsedOption(depend) || -1 != FindOption(depend))
   {
      char * dep = ConsStr("%s",OptName(depend));
      /* second call to  OptName  would overwrite the return string */;
      ErrRegister(ConsStr( 
                     "Option %s can be used only when option %s is %d(char: '%c')",
                     dep, OptName(cond), enableVal,enableVal));
		free(dep);  

   } 
} 

/*-------------------------------------------------------------*/
void OptionCompulsory(char * name)

{
   IsInitialized();

   DepRegister(ConsStr( "Option %s is compulsory", OptName(name))); 

   if (-1 == FindUsedOption(name) && -1 == FindOption(name))
      ErrCompulsory(name);
}
/*-------------------------------------------------------------*/
void OptionCompulsoryArgs(int num)
{
   IsInitialized();

   DepRegister(ConsStr("At least %d argument(s) must be specified",num));

   if(num > numArgs)
      ErrRegister(ConsStr(
                     "Only %d argument(s) were found on the command line",numArgs));
}
