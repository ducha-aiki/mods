#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ranH.h"
#include "Htools.h"
#include "rtools.h"
#include "waldtools.h"
#include <matutl.h>

double round (double);

#define wspacesize (4*9*9)
#define proTC 16
#define ITERNO 5

/* *************** TRUE WALD ****************** */

#define C_tM  50
#define C_ms  1

#define xalloc(a) malloc(a); ALLOC += (a);

int ALLOC;

int singleHDs2 (const double *u, const double *H, double *p, int * pool,
                int *pool2, int len, int* pos, const double * Z)
{
  int i, j, shift = 2*len;
  const double *l;
  double pJ[8];
  double r1, r2, a, b, c, d, e;

  i = pool[pool2[*pos]];
  *pos += 1;
  if (*pos >= len-4) *pos = 0;

  r1 = 0;
  r2 = 0;
  l = Z + 2*i;
  u += 6*i;

  for (j = 0; j < 9; j++)
    {
      r1 += H[j] * *l;
      r2 += H[j] * l[1];
      l += shift;
    }

  a = H[0] - H[2] * u[0];
  b = H[3] - H[5] * u[0];
  c = -H[8] - H[2] * u[3] - H[5] * u[4];
  d = H[1] - H[2] * u[1];
  e = H[4] - H[5] * u[1];

  pinvJ(a,b,c,d,e,pJ);


  p[i] = 0;
  for (j = 0; j < 4; j++)
    {
      a = pJ[j] * r1 + pJ[j+4] * r2;
      p[i] += a * a;
    }

  return i;
}


void dbgprnt(void * p, int len)
{
  int i,s = 0, *pt;
  pt = (int *) p;
  for(i=0; i<len/sizeof(int); i++)
    s += pt[i];
  printf("%x ",s);
}

/* WALD-SAC * WALD-SAC * WALD-SAC * WALD-SAC * WALD-SAC * WALD-SAC */


int waldH(double *u, int len, double th, double conf,
          int verif_type, double ep, double del, 
          double *H, unsigned char * inl,
          int* data_out, int seed, double* hist,
	  struct samhist **shout)
{
  const int m = 4;
  int *pool, no_sam,  new_sam, max_sam, max_sam_wald;
  double *Z, *M, *buffer;
  double *h;
  int no_succ, no_try, last_del_chg, del_estim_len;

  int no_mod = 0, no_ver = 0, no_pass = 0;
  double A, linl, lout, la, prolong, new_del, new_ep;
  int lastAchg = 0, max_sam_updated = 1;
  struct samhist *shistory = NULL, *sh;

  double poly[4], roots[3], f[9], *err, *sgn;
  int *inls[4], *iinls[3];
  int nsol, i, j, *inliers, new_max, do_iterate, pos;
  int maxI, maxIs, I, cI, min_iter_inl;
  int *samidx, *d, *curr_inl, *iter_inl, *max_inl = NULL;
  int *pool2, ver_st = 0;

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
  maxI = m+1; 
  maxIs = m+1;
  min_iter_inl = 2*m;
 
  pool2 = (int *) xalloc((len-m) * sizeof(int));
  for (i=0;i<len-m;i++)
    pool2[i] =i;
  randsubset(pool2,len-m,len-m-1);

  Z = (double *) malloc(len * 18 * sizeof(double));
  lin_hg(u, Z, pool, len);
 
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

  samidx = pool + len - m;

  /* Gandalf */
  gan_mat_form (&mA, 9, 9); 
  gan_mat_form (&mQ, 9, 9); 
  adWorkspace = (double*) xalloc(wspacesize * sizeof(double));
  h = mQ.data + 8*9;
  M = mA.data;
  for (i=8*9; i<9*9; i++)
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
    }

  srand(seed++);
  while(no_sam < max_sam)
    {
      no_sam ++;
      if (verif_type == 3)
      {
        if ((no_sam >= max_sam_wald) & !max_sam_updated)
	  {
            max_sam= wald_nsamples(maxI,len,m,conf,A,shistory); 
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

      /*      if ((no_sam > 4000) && (no_sam % 100 == 0)) 
	{
          printf("%d (%d) j:%d len:%d \n",no_sam,maxI,j,len); 
	  for(i=0;i<4;i++) printf("%3d ", samidx[i]);
          printf("\n");
          dbgprnt(Z,len*18*sizeof(double));
          dbgprnt(pool,len*sizeof(int));
          dbgprnt(u,len*6*sizeof(double));
          dbgprnt(&h,sizeof(double *));
          printf("\n");
	  } */

      multirsampleT(Z, 9, 2, pool, 4, len, M);

      /*      for (i=0;i<9;i++)
	{
	  for(j=0;j<9;j++)
	    {
	      printf("%.2f ",M[i*9+j]);
	    }
	  printf("\n");
	}
	printf("\n\n"); */

      /* QR */
     if ( gan_mat_qr(&mA, &mQ, NULL, adWorkspace, wspacesize) == GAN_FALSE)
        printf("Gndalf routine gan_mat_qr() failed.");

        {
          new_max = 0;
     
          no_mod ++;

          /* orient. constr. */
	  //if (!all_ori_valid(f, u, samidx, 7))  continue; 

          I = 0;
          j = 0;
	  la = 1;
          d = inls[1];

          if (verif_type == 0) /* RANSAC */
	    { 
              HDs(Z, u, h, err, len);
              j = m;

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

          if (verif_type == 1) /* R-RANSAC */
	    { 
              j = m;
              while (j < m+A)
       	      {
	        no_ver ++;
		/*    pos = singleFDs (u, f, err, pool, len, j); */
		pos = singleHDs2 (u, h, err, pool, pool2, len, &ver_st, Z); 
	        if (err[pos] <= th)
		  {
		    d[I] = pos;
		    I ++;
		  } else break;
	        j++;
	      }
	      if (j < m+A) continue;

              HDs(Z, u, h, err, len);

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

          if (verif_type >= 2) /* RANSAC Wald*/
	    { 
	      j = m; 
	      while (j < len)
	      {
                no_ver ++;
                no_try ++;
		pos = singleHDs2 (u, h, err, pool, pool2, len, &ver_st, Z); 

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
	      no_try -= len-m;
	      no_succ -= I;
	    }
	    
          no_pass++;
          for (i = 0; i<m; i++)
	    {
              d[I] = samidx[i];
	      I ++;
	    }

	  /*Pep1 = (double)I/(double)len;
          Pep2 = Pep1 * Pep1;
          P_sam = Pep2 * Pep2 * Pep2 * Pep1;
	  P_iter += P_sam;*/

	  curr_inl = inls[1];

          if(I > maxI)
	    {
              d = inls[1]; inls[1] = inls[4]; inls[4] = d;
              max_inl = curr_inl;
	      maxI = I;
	      memcpy(H,h,9*sizeof(double)); /*!!!*/
              new_max = 1;
	    }

	  if (I > maxIs)
	    {
	      maxIs = I;
      	      do_iterate = 1;
	      if (!new_max)
                  d = inls[1]; inls[1] = inls[3]; inls[3] = d;
              iter_inl = curr_inl;
	    }

	   do_iterate = 0;
//	   if (do_iterate & (maxIs > min_iter_inl) & (P_iter * no_sam > 1))
	  if (do_iterate)
	  {
	    do_iterate = 0;
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
              new_sam = nsamples(maxI, len, m, conf) * prolong; 
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
  if (max_inl != NULL)
    for (i = 0; i < maxI; i++)
{
     inl[max_inl[i]] += 1;
}

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
