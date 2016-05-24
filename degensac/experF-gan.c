#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "proF.h"
#include "ranF.h"
#include "Ftools.h"
#include "rtools.h"
#include "utools.h"
#include "waldtools.h"
#include <matutls/matutl.h>

#include <gandalf/linalg/mat_gen.h>
#include <gandalf/linalg/mat_square.h>
#include <gandalf/linalg/mat_qr.h>

double round (double a)
{
	return floor(a + .5);
}

#define wspacesize (4*9*9)
#define proTC 16
#define ITERNO 5
#define USE_LU

/* *************** TRUE WALD ****************** */

#define C_tM  200
#define C_ms  2.38

#define xalloc(a) malloc(a); ALLOC += (a);

int ALLOC;

int singleFDs (const double *u, const double *F, double *p, int * pool,
                int len, int pos)
{
  double rx, ry, rwc, ryc, rxc, r;
  int j;

  j = sample(pool, len, pos);
  u += 6 * j;

  rxc = _f1 * u4 + _f4 * u5 + _f7;
  ryc = _f2 * u4 + _f5 * u5 + _f8;
  rwc = _f3 * u4 + _f6 * u5 + _f9;
  r =(u1 * rxc + u2 * ryc + rwc);
  rx = _f1 * u1 + _f2 * u2 + _f3;
  ry = _f4 * u1 + _f5 * u2 + _f6; 

  p[j] = r*r / (rxc*rxc + ryc*ryc + rx*rx + ry*ry);
  return j;
}

int singleFDs2 (const double *u, const double *F, double *p, int * pool,
                int *pool2, int len, int* pos)
{
  double rx, ry, rwc, ryc, rxc, r;
  int j;

  j = pool[pool2[*pos]];
  *pos += 1;
  if (*pos >= len-7) *pos = 0;
  u += 6 * j;

  rxc = _f1 * u4 + _f4 * u5 + _f7;
  ryc = _f2 * u4 + _f5 * u5 + _f8;
  rwc = _f3 * u4 + _f6 * u5 + _f9;
  r =(u1 * rxc + u2 * ryc + rwc);
  rx = _f1 * u1 + _f2 * u2 + _f3;
  ry = _f4 * u1 + _f5 * u2 + _f6; 

  p[j] = r*r / (rxc*rxc + ryc*ryc + rx*rx + ry*ry);
  return j;
}



/* WALD-SAC * WALD-SAC * WALD-SAC * WALD-SAC * WALD-SAC * WALD-SAC */


int waldF(double *u, int len, double th, double conf,
          int verif_type, double ep, double del, 
          double *F, unsigned char * inl,
          int* data_out, int seed, double* hist,
	  struct samhist **shout)
{
  int *pool, no_sam,  new_sam, max_sam, max_sam_wald;
  double *Z, *M, *buffer;
  double *f1, *f2;
  int no_succ, no_try, last_del_chg, del_estim_len;

  int no_mod = 0, no_ver = 0, no_pass = 0;
  double A, linl, lout, la, prolong, new_del, new_ep;
  int lastAchg = 0, max_sam_updated = 1;
  struct samhist *shistory = NULL, *sh;

  double poly[4], roots[3], f[9], *err, *sgn;
  int *inls[5], *iinls[3];
  int nsol, i, j, *inliers, new_max, do_iterate, pos;
  int maxI, maxIs, I, cI, min_iter_inl;
  int *samidx, *d, *curr_inl, *iter_inl, *max_inl;
  int *pool2, ver_st = 0;
  int nullsize, nullbuff [18];
  int * bailI;

  double P_iter = 0, P_sam, Pep1, Pep2;

  Gan_Matrix mA, mQ;
  double *adWorkspace; 

  /* to eliminate */
  int iter_cnt = 0, LmaxI;

  /* allocations */
  ALLOC = 0;

  pool = (int *)xalloc(len * sizeof(int));

  j = 0;
  for (i = 0; i < len; i ++)
    {
      pool[i] = i;
    }
  maxI = 8; 
  maxIs = 8;
  min_iter_inl = 16;
 
  pool2 = (int *) xalloc((len-7) * sizeof(int));
  for (i=0;i<len-7;i++)
    pool2[i] =i;
  randsubset(pool2,len-7,len-8);

  Z = (double *) xalloc(len * 9 * sizeof(double));
  lin_fm(u, Z, pool, len);
 
  buffer = (double *) xalloc(len * 12 * sizeof(double)); /* 9+1+2 */

  err = (double *) xalloc(len * 4 * sizeof(double));
  sgn = (double *) xalloc(len * sizeof(double));

  inliers = (int *) xalloc(8 * sizeof(int) * len); /* 5 + 3 */
  /*inls[0-2] ... 1-3 solutions
    inls[3]   ... iter soulution
    inls[4]   ... the best solution*/
  for (i=0; i<5; i++)
    inls[i] = inliers + (i*len);
  for (i=0; i<3; i++)
    iinls[i] = inliers + ((i+5) * len);

  no_sam = 0;

  samidx = pool + len - 7;

  /* Gandalf */
  gan_mat_form (&mA, 9, 9); 
  gan_mat_form (&mQ, 9, 9); 
  adWorkspace = (double*) xalloc(wspacesize * sizeof(double));
  
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

  max_sam = 1000000;
  max_sam_wald = max_sam;

  /* verification */
  switch (verif_type)
    {
    case 0: /* RANSAC */
      prolong = 1;
    case 1: /* R-RANSAC T(1,1) */
      A = 1;
      prolong = 1;
      break;
    case 2:
    case 3: /* R-RANSAC Wald */
      A = wlad_getA (ep, del, &linl, &lout, C_tM, C_ms);
      del_estim_len = 2*len;
      no_try = del_estim_len;
      no_succ = (int) round(((double)no_try) * del);
      last_del_chg = del_estim_len;
      prolong = 1/(1-(1/A));
      break;
    case 4: /* Bail-out by Capel */
      break;
    }

  srand(seed++);
  while(no_sam < max_sam)
    {
      no_sam ++;
      if (verif_type == 3)
      {
        if ((no_sam >= max_sam_wald) & !max_sam_updated)
	  {
            max_sam= wald_nsamples(maxI,len,7,conf,A,shistory); 
	    max_sam_updated = 1;
	  }
          if (no_try - last_del_chg > del_estim_len)
	  {
	  new_del = ((double)no_succ)/no_try;
          last_del_chg = no_try;
	  if (fabs(del - new_del) / del > .1)
	    {
	      shistory = wald_addsamhist(ep,del,A,shistory,no_sam,&lastAchg);
	      del = new_del;
              A = wlad_getA (ep, del, &linl, &lout, C_tM, C_ms);
	    }
	    }
      }

      //if (no_sam % 100 == 0) printf("%d (%d)\n",no_sam,maxI); 
      rsampleT(Z, 9, pool, 7, len, M);

#ifdef USE_QR
      /* QR */
     if ( gan_mat_qr(&mA, &mQ, NULL, adWorkspace, wspacesize) == GAN_FALSE)
       printf("Gndalf routine gan_mat_qr() failed."); 
#else
     /* LU */
      nullsize = nullspace(M, f1, 9, nullbuff);
      if (nullsize != 2)
	{
//	  printf ("Null space size %d\n",nullsize);
	  continue;
	} 
#endif

      slcm (f1, f2, poly);  
      nsol = rroots3(poly, roots);

      for (i = 0; i < nsol; i++)
        {
          new_max = 0;
     
          no_mod ++;
          for (j = 0; j < 9; j++)
	    f[j] = f1[j] * roots[i] + f2[j] * (1 -roots[i]);

          /* orient. constr. */
	  //if (!all_ori_valid(f, u, samidx, 7))  continue; 

          I = 0;
          j = 0;
	  la = 1;
          d = inls[i];

/*    -    -    -    -    -    -    -    -    -    -    -    -    -    */

          if (verif_type == 0) /* RANSAC */
	    { 
              FDs(u, f, err, len);
              j = 7;

	      while(j < len)
	      {
                no_ver ++;
  	        if (err[pool[j]] <= th)
		 {
                   d[I] = pool[j];
		   I ++;
		 }
	        j++;
	      }
	    }

/*    -    -    -    -    -    -    -    -    -    -    -    -    -    */

          if (verif_type == 1) /* R-RANSAC */
	    { 
              j = 7;
              while (j < 7+A)
       	      {
	        no_ver ++;
		/*    pos = singleFDs (u, f, err, pool, len, j); */
		pos = singleFDs2 (u, f, err, pool, pool2, len, &ver_st); 
	        if (err[pos] <= th)
		  {
		    d[I] = pos;
		    I ++;
		  } else break;
	        j++;
	      }
	      if (j < 7+A) continue;

              FDs(u, f, err, len);

	      while(j < len)
	      {
                no_ver ++;
  	        if (err[pool[j]] <= th)
		 {
                   d[I] = pool[j];
		   I ++;
		 }
	        j++;
	      }
	    }

/*    -    -    -    -    -    -    -    -    -    -    -    -    -    */


          if ((verif_type == 2) || (verif_type == 3))  /* RANSAC Wald*/
	    { 
	      j = 7; 
	      while (j < len)
	      {
                no_ver ++;
                no_try ++;
		/*    pos = singleFDs (u, f, err, pool, len, j); */
		pos = singleFDs2 (u, f, err, pool, pool2, len, &ver_st); 

	        if (err[pos] <= th)
		  {
                    la = la * linl;
                    d[I] = pos;
		    I ++;
		    no_succ ++;
		  } else
		    la = la * lout;
	        if (la >= A) break;
	        j ++;
	      }

	      /*hist[I*len+j-7] += 1;*/

              if (j < len) continue;
	      no_try -= len-7;
	      no_succ -= I;
	    }
	    

/*    -    -    -    -    -    -    -    -    -    -    -    -    -    */

         if (verif_type == 4)  /* Bail out */
	   { 
	      j = 7; 
	      while (j < len)
	      {
                no_ver ++;
		pos = singleFDs2 (u, f, err, pool, pool2, len, &ver_st); 

	        if (err[pos] <= th)
		  {
                    d[I] = pos;
		    I ++;
		  } 
	        if (I < bailI[j]) break;
	        j ++;
	      }
              if (j < len) continue;
	   }

/*    -    -    -    -    -    -    -    -    -    -    -    -    -    */

          no_pass++;
          for (j = 0; j<7; j++)
	    {
              d[I] = samidx[j];
	      I ++;
	    }

	  /*Pep1 = (double)I/(double)len;
          Pep2 = Pep1 * Pep1;
          P_sam = Pep2 * Pep2 * Pep2 * Pep1;
	  P_iter += P_sam;*/

	  curr_inl = inls[i];

          if(I > maxI)
	    {
              d = inls[i]; inls[i] = inls[4]; inls[4] = d;
              max_inl = curr_inl;
	      maxI = I;
	      memcpy(F,f,9*sizeof(double)); /*!!!*/
              new_max = 1;
	    }

	  if (I > maxIs)
	    {
	      maxIs = I;
      	      do_iterate = 1;
	      if (!new_max)
                  d = inls[i]; inls[i] = inls[3]; inls[3] = d;
              iter_inl = curr_inl;
	    }

	   do_iterate = 0;
	  // if (do_iterate & (maxIs > min_iter_inl) & (P_iter * no_sam > 1))
	  if (do_iterate)
	  {
	  do_iterate = 0;
      	  //          printf("%d:",no_sam);
          //for (i=0; i<7; i++) printf("%d ",samidx[i]);
          //printf("- %d\n",I);
	  iter_cnt ++;
          cI = maxIs;
          j = 5;

          I = inFranio (u, len, iter_inl, cI, cI, th, 
                       iinls, buffer, f, ITERNO, &curr_inl);
          while ((I > cI) & (j > 0))
	    {
              iter_inl = curr_inl;
              d = iinls[2]; iinls[2] = inls[3]; inls[3] = d;
	      cI = I;
              I = inFranio (u, len, iter_inl, cI, cI, th, 
                            iinls, buffer, f, ITERNO, &curr_inl);
	      j--;
	    }

          if(I > maxI)
            {
	      printf("#");
   	       d = inls[4]; inls[4] = iinls[2]; iinls[2] = d;
	       maxI = I;
               max_inl = iter_inl;
	       memcpy(F,f,9*sizeof(double)); /*!!!*/
               new_max = 1;
            }
	}

        if(new_max)
	  {
              new_ep = ((double)maxI)/len;
	      switch (verif_type)
		{
		case 1: 
		  prolong = 1;
		  for (j=0; j<A; j++) prolong /= new_ep;
		  break;
		case 3:
		  prolong = 1; 
                  if (new_ep > ep){
   	          shistory=wald_addsamhist(ep,del,A,shistory,no_sam,&lastAchg);
                  A = wlad_getA (new_ep, del, &linl, &lout, C_tM, C_ms);
                  ep = new_ep;
		  }
		  break;
		}
              new_sam = nsamples(maxI, len, 7, conf) * prolong; 
	      if (verif_type == 3)
		{
		  if (new_sam < max_sam_wald)
		    {
		      max_sam_wald = new_sam;
		      max_sam_updated = 0;
		    }
		} else
		  if (new_sam < max_sam)
		    max_sam = new_sam;
	  }

        }
      
    }

    for (i = 0; i < len; i++) inl[i] = 0; 
    for (i = 0; i < maxI; i++) inl[max_inl[i]] = 1;

  /* deallocations */

    /*    while (shistory != NULL)
      {
	sh = shistory->next;
        free(shistory);
        shistory = sh;
	} */
    *shout =wald_addsamhist(ep,del,A,shistory,no_sam,&lastAchg);

  free(pool);
  free(Z);
  free(err);
  free(sgn);
  free(inliers);
  free(buffer);
  free(pool2);

  /* Gandalf */
  gan_mat_free(&mA); 
  gan_mat_free(&mQ); 
  free(adWorkspace);

  *data_out = no_sam; 
  data_out[1] = no_mod;
  data_out[2] = no_ver;
  data_out[3] = no_pass;
  data_out[4] = iter_cnt;

  /*  printf("Total alloc: %d kB\n", ALLOC / (1024));*/

  return maxI;
}
