#ifndef OPTION_h
#define OPTION_h
/*---------- Command line parser         ---------------------------------- */
/*  author: G. Matas   (g.matas@ee.surrey.ac.uk) */
/*
   18-Feb-93,  George Matas
      - created
*/
/*---------------------------------------------------------------------*/
#if defined(__cplusplus) || defined(c_plusplus)
extern "C" {               /* can be used directly from C++ */
#endif
    /*---------------------------------------------------------------------*/

    /*--- OptionInit/OptionInitCopy must be called before any other func! ------*/

    void OptionInit(char ** argv, int * argc);
    /* initilise the system, options and pars. are removed form agrv, argc */

    void OptionInitCopy(char ** argv, int * argc);
    /* initilise the system, leave argv, argc untouched, work on a copy  */

    void OptionCheck(void);
    /* ends command line processing, if errors are detected their list */
    /* together with usage/help is printed out */
    void OptionCheckDesc(char *desc);

    int  OptionOnCommLine(char * name);
    /* was the option on the command line? */
    void OptionLeftOK(void);     /* don't consider unprocessed options an error */
    char** OptionUsage(void) ;   /* returns complete usage (NULL terminated) */
    void OptionSave(char *fname);/* store usage and time in file */

    void OptionClose(void);      /* clean internals, ready for a new OptionInit*/
    void OptionAddUsage(char * s);

    /*----------------------- get   options ---------------------------------*/
    /*-------- integer, integer pair, integer array ------------*/
    int  OptionInt(char * name, int def, char * comment);
    void OptionIntInt(char * name, int *v1,int *v2, int def1, int def2, char * c);
    void OptionIntArr(char *name, int * arr, int elems, char * comment);

    /*--------- double, double pair, double array ---------------*/
    double OptionDouble(char * name, double def, char * comment);
    void OptionDoubleDouble
    (char * name, double *v1,double *v2, double def1, double def2, char * c);
    void  OptionDoubleArr(char *name, double * arr, int elems, char * comment);


    /*--------- string and char and toggle (boolean) -------------*/
    char * OptionStr(char * name, char * def, char * comment);
    unsigned char OptionChar(char * name, unsigned char def, char * comment) ;
    int OptionToggle(char * name, int def, char * comment);

    /* list of values */
#ifdef LL_h
    t_LL OptionLL(char * name, char * comment );
#endif

    /*---------- dependency definition functions -------------------*/
    void OptionDependXor(char * xor_opt);
    /* xor_opt defines a set of mutually exclusive options */
    /* options are specified in xor_opt without '-', eg. 'alpha beta x'   */

    void OptionIf(int enableCond, char * depend, char * comment);
    void OptionMultIf(int enableCond, char * depend, char * comment);


    void OptionDependIf(char * cond, int enableVal, int val, char * depend);
    /* option 'depend' can be specified only if option 'cond' has value  */
    /* 'enableVal'. Current value of 'depend' is passed explicitly in val*/

    void OptionCompulsory(char * name);
    /* emit error message if 'name' was not found on the command line */

    void OptionCompulsoryArgs(int num);
    /* define the number of compulsory arguments */
    /* must be used after all Option*("",...) have been called !*/

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif

#endif
