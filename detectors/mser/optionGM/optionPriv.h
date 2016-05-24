#ifndef OPTION_PRIV_h
#define OPTION_PRIV_h
/*---------- Command line parser private functions   --------------------- */
/*  author: G. Matas   (g.matas@ee.surrey.ac.uk) */
/*
   12-Mar-93,  J. Matas
      - optBuf removed

   21-Feb-93,  George Matas
      - created
*/
/*--------------------------------------------------------------------------*/

/*------------------ utilities for optionTypeX --------------------*/
void Usage(char * s);
char * OptName(char * name);
void IsInitialized(void );
char ** GetOption(char * name,int expectedPars);
void ErrIncorrectType(char * name) ;
char * ConsStr(char * format, ...);

#endif
