#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <matutls/matutl.h>

#include "ranF.h"
#include "Ftools.h"
#include "rtools.h"
#include "utools.h"

#include <gandalf/linalg/mat_gen.h>
#include <gandalf/linalg/mat_square.h>
#include <gandalf/linalg/mat_qr.h>

//#define USE_QR

int iterFo(double *u, int len, int **iinls, double th, double ths, int steps,
           double *F, double *err, double* sgn, double *buffer, int **max_inl)
{

  /* iinls[0] ... used for calculations
     iinls[1] ... the best result */

  double *w, *buff;
  double f[9], dth;
  int it, I, Is, maxI, i;
  int *inliers, *d;

  w = buffer;
  buff = buffer + len;
  dth = (ths - th) / (steps); 

  /* F from the sample inliers by th */

  maxI = inlidxso(err, sgn, len, th, iinls[1], &inliers);
  *max_inl = inliers;
    u2f(u, inliers, maxI, f, buff);

  /*iterate */

  for (it = 0; it < steps; it ++)
    {
      I = exFDso (u, f, err, w, len, ths, iinls[0], &inliers);

	/*   Is = inlidxs (d, len, th, inliers); //dodelat!

      if (Is > maxI) 
	{
	  maxI = Is;
	  errs[1] = errs[0];
	  errs[0] = d;
	  d = errs[1];
          memcpy(F,f,9*sizeof(double));
	}  */

      if (I < 8)
	{
         return maxI;
	}
      u2fw(u, inliers, w, I, f, buff);
      ths -= dth;
    } 
 
  FDso (u, f, err, sgn, len);
  I = inlidxso(err, sgn, len, th, iinls[0], &inliers);
  if (I > maxI)
    {
      maxI = I;
      d = iinls[1]; iinls[1] = iinls[0]; iinls[0] = d;
      *max_inl = inliers;
      memcpy(F,f,9*sizeof(double)); /*!!!*/
    }

  return maxI;
}

int inFranio (double *u, int len, int *inliers, int psz, int ninl, double th,
              int **iinls, double *buffer, double *F, int rep, int ** max_inl)
{
  /* iinls[0] ... used in iterFo
     iinls[1] ... max from iterFo
     iinls[2] ... max over all inner samples 
     inliers  ... sampling pool  */

  int I, maxI, ssiz, i;
  double f[9], *err, *sgn, *buff;
  int *sample, *curr_inl;
  int *d;

  err = buffer; sgn = buffer + len; buff = buffer + 2*len;

  if (psz < 16) return 0;
  ssiz = psz /2;
  if (ssiz > 14) ssiz = 14;

  maxI = ninl;

  for (i = 0; i < rep; i++)
    {
      sample = randsubset(inliers, psz, ssiz);
      u2f(u, sample, ssiz, f, buff);
      FDso (u, f, err, sgn, len);

      I = iterFo(u, len, iinls, th, TC*th, 4, f, err, sgn, buff, &curr_inl);

      if (I > maxI)
	{
	  maxI = I;
          d = iinls[2]; iinls[2] = iinls[1]; iinls[1] = d;
          *max_inl = curr_inl;
          memcpy(F,f,9*sizeof(double)); /*!!!*/
	}
    }

  /*   FDso (u, F, err, sgn, len);
   I = iterFo(u, len, iinls, th, TC*th, 4, f, err, sgn, buff, &curr_inl);
   if (I > maxI)
	{
	  //          printf("!");
	  maxI = I;
          d = iinls[2]; iinls[2] = iinls[1];  iinls[1] = d;
          *max_inl = curr_inl;
          memcpy(F,f,9*sizeof(double)); 
	  } */

  return maxI;
}

/*********************   PROSAC   ************************/

#define wspacesize (4*9*9)
#define proTC 16
#define ITERNO 30

int prosacF(double *u, int len, double th, double conf,
            int* gf, double *F, unsigned char * inl,
            int* data_out, double* outn)
{
  int *pool, no_sam, *max_sams, stoplen, new_sam, max_sam;
  double *Z, *M, *buffer;
  double *f1, *f2;

  double poly[4], roots[3], f[9], *err, *sgn;
  int *inls[4], *iinls[3];
  int nsol, i, j, *inliers, new_max, do_iterate;
  int *maxI, maxIs, I, max_tot, cI;
  int *samidx, *d, *curr_inl, *iter_inl, *max_inl;
  int nullsize, nullbuff [18];

  int n = 7, mR = 20, R;

  Gan_Matrix mA, mQ;
  double *adWorkspace; 

  int MINSAM[] = {  8,  10,  13,  17,  22,  28,  34,  42,  51,  61,
                   72,  84,  97, 110, 125, 141, 157, 175, 193, 213,
                  233, 255, 277, 300, 324, 349, 375, 402, 430, 459,
                  489, 519, 551, 583, 616, 651, 686, 722, 759, 797,
                  835, 875, 915, 957, 999, 1000000};

  /* to eliminate */
  int iter_cnt = 0, LmaxI;

  /* allocations */

  pool = (int *)malloc(len * sizeof(int));
  max_sams = (int *)malloc(len * sizeof(int));
  maxI = (int *)malloc(len * sizeof(int));

  j = 0;
  for (i = 0; i < len; i ++)
    {
      if (MINSAM[j] <= i) j++;
      pool[i] = i;
      max_sams[i] = MAX_SAMPLES;
      maxI[i] = 8+j; 
    }
  maxI[len-1] = 8;
 

  Z = (double *) malloc(len * 9 * sizeof(double));
  lin_fm(u, Z, pool, len);
 
  buffer = (double *) malloc(len * 12 * sizeof(double)); /* 9+1+2 */

  err = (double *) malloc(len * 4 * sizeof(double));
  sgn = (double *) malloc(len * sizeof(double));

  inliers = (int *) malloc(8 * sizeof(int) * len); /* 5 + 3 */
  /*inls[0-2] ... 1-3 solutions
    inls[3]   ... iter soulution
    inls[4]   ... the best solution*/
  for (i=0; i<5; i++)
    inls[i] = inliers + (i*len);
  for (i=0; i<3; i++)
    iinls[i] = inliers + ((i+5) * len);

  maxIs = 16;
  max_tot = 16;
  no_sam = 0;
  stoplen = len;
  R = mR;

  if (gf == NULL) { /* no PROSAC just RANSAC*/
    samidx = pool + len - 7;
    n = len+1;
    R = len-1;
    mR = len;
  } else
    samidx = pool;

  /* Gandalf */
  gan_mat_form (&mA, 9, 9); 
  gan_mat_form (&mQ, 9, 9); 
  adWorkspace = (double*) malloc(wspacesize * sizeof(double));

#ifdef USE_QR
   f1 = mQ.data + 7*9;
   f2 = mQ.data + 8*9; 
#else
  f1 = mQ.data;
  f2 = f1 + 9;
#endif

  M = mA.data;
  for (i=7*9; i<9*9; i++)
     M[i] = 0.0;

  /*  srand(RAND_SEED++); */
  while(no_sam < max_sams[stoplen-1])
    {
      no_sam ++;

      if (no_sam % 10000 == 0) printf("%d\n",no_sam);

      if (n > stoplen)
	{
	    rsampleTn(Z, 9, pool, 7, stoplen, len, M);
	} else
	  {
	    rsampleTn(Z, 9, pool, 6, n-1, len, M);
            addcorrT(Z+n, 9, len, M+54);
	    if (gf[n] <= no_sam)
             {
               n++;
	       samidx ++;
               if (R < n-1) R = n-1;
	     }
	  };

      /*         for(i=0;i<7;i++) printf("%d ",samidx[i]);
            printf("\n");

	    printf("\n");
	    for (j=0;j <9; j++)
	      {
		for(i=0; i<9; i++)
		  printf("%3.1f ",M[9*j+i]);
		printf("\n");
	      }
	      printf("\n"); */

#ifdef USE_QR
      /* QR */
     if ( gan_mat_qr(&mA, &mQ, NULL, adWorkspace, wspacesize) == GAN_FALSE)
       printf("Gndalf routine gan_mat_qr() failed."); 
#else
     /* LU */
      nullsize = nullspace(M, f1, 9, nullbuff);
      if (nullsize != 2)
	{
	  printf ("Null space size %d\n",nullsize);
	  continue;
	} 
#endif

      slcm (f1, f2, poly);  
      nsol = rroots3(poly, roots);

      new_max = 0; do_iterate = 0;
      LmaxI = 0;
      for (i = 0; i < nsol; i++)
        {
          for (j = 0; j < 9; j++)
	    f[j] = f1[j] * roots[i] + f2[j] * (1 -roots[i]);

          /* orient. constr. */
	  //if (!all_ori_valid(f, u, samidx, 7))  continue; 
        
          FDso (u, f, err, sgn, len);
          I = inlidxso(err, sgn, len, th, inls[i], &curr_inl);

          if(I > maxIs)
	    {
	      do_iterate = 1; 
	      maxIs = I;
	      iter_inl = curr_inl;
	    }

          if(I > max_tot)
	    {
              d = inls[i]; inls[i] = inls[4]; inls[4] = d;
              max_inl = curr_inl;
	      max_tot = I;
	      memcpy(F,f,9*sizeof(double)); /*!!!*/
	      new_max = 1;
	    }
        }

      if (do_iterate)
	{
          if (maxIs < 16) break;

	  //          printf("%d:",no_sam);
          //for (i=0; i<7; i++) printf("%d ",samidx[i]);
          //printf("- %d\n",I);
	  iter_cnt ++;
          cI = maxIs;

          I = inFranio (u, len, iter_inl, cI, cI, th, 
                       iinls, buffer, f, ITERNO, &curr_inl);
          while (I > cI)
	    {
              iter_inl = curr_inl;
              d = iinls[2]; iinls[2] = inls[3]; inls[3] = d;
	      cI = I;
              I = inFranio (u, len, iter_inl, cI, cI, th, 
                            iinls, buffer, f, ITERNO, &curr_inl);
	    }

          if(I > max_tot)
            {
   	       d = inls[4]; inls[4] = iinls[2]; iinls[2] = d;
	       max_tot = I;
               max_inl = iter_inl;
	       memcpy(F,f,9*sizeof(double)); /*!!!*/
               new_max = 1;
            }
	}

      if (new_max)
	{
          max_sam = max_sams[stoplen-1];
       
          for (i = 0; i < len; i++) inl[i] = 0; 
          for (i = 0; i < max_tot; i++) inl[max_inl[i]] = 1;

          I = 0;
          for(i=0; i<mR-1; i++)
            I += inl[i];

          for(i=mR-1; i<len; i++)
            {
              I += inl[i];

              if (maxI[i] < I)
	         {
	           maxI[i] = I;
		   if ((i == len-1) || (inl[i] && !inl[i+1]))
		     {
                       new_sam = nsamples(I+1, i+1, 7, conf);
                       if (i < R) new_sam += no_sam - gf[i];
                       if (new_sam < max_sams[i]) 
	                 {
	                   max_sams[i] =  new_sam;
	                   if ((new_sam < max_sam)||((new_sam == max_sam) &&
				                     (i >= stoplen)))
		             {
		               stoplen = i+1;
		               max_sam = new_sam;
		             }
	                 }
		     } 
	         }

	    }
	  if (stoplen < R+1) 
             for (i=0; i<=R; i++) pool[i] = i;
	}
    }

  /* deallocations */

  free(pool);
  free(max_sams);
  free(maxI);
  free(Z);
  free(err);
  free(sgn);
  free(inliers);
  free(buffer);

  /* Gandalf */
  gan_mat_free(&mA); 
  gan_mat_free(&mQ); 
  free(adWorkspace);

  *data_out = no_sam; 
  data_out[1] = iter_cnt;
  data_out[2] = stoplen;
  return max_tot;
}
