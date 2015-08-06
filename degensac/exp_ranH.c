#undef __STRICT_ANSI__
#include <stdlib.h>
#include <memory.h>
#include <math.h>
#include "../matutls/matutl.h"
#include "utools.h"
#include "lapwrap.h"
#include "rtools.h"
#include "hash.h"


#include "exp_ranH.h"
#define CHECK_COEF 9.0 //3*3
#define MIN_GOOD_SYM_PTS 5
#define __HASHING__


//#define FULL_SYMM
Score exp_iterH(double *u, int len, int *inliers, double th, double ths,
                int steps, double *H, double *Z, double **errs, double *buffer,
                int iterID, unsigned inlLimit, double *resids)
{
  double *d = errs[1];
  double h[9], dth;
  int it;
  Score maxS = {0,0}, S = {0,0}, Ss;
#ifdef __D3__
  int * detachedInl;
  unsigned detachedCount;
#endif
#ifdef __HASHING__
  int iterIDret;
  uint32_t hash;
#endif //__HASHING__

  dth = (ths - th) / (steps);

  /* H from the sample inliers by th */

  maxS = inlidxs(errs[4], len, th, inliers);
  if (maxS.I < 4)
    return S;
  S = inlidxs(errs[4], len, th*MWM, inliers);
#ifdef __D3__
  //D3 - calculate number of inliers detached for LSQ
  detachedCount = (int)(S.I * D3_H_RATIO);
  if (detachedCount < D3_H_MIN) {
      detachedCount = D3_H_MIN;
    }
  if (detachedCount > inlLimit) {
      detachedCount = inlLimit;
    }
  if (detachedCount < 4) {
      detachedCount = 4;
    }
  if (detachedCount >= S.I) { // if we want to use more than (or all) we have, just use what we have without shuffling
      u2h(u, inliers, S.I, h, buffer);
    } else {
      detachedInl = randsubset (inliers, S.I, detachedCount);
      u2h(u, detachedInl, detachedCount, h, buffer);
    }
#else
  u2h(u, inliers, S.I, h, buffer);
#endif

  /*iterate */

  for (it = 0; it < steps; it ++)
    {
#ifdef FULL_SYMM
      HDsSym (Z, u, h, d, len);
#else
      HDs (Z, u, h, d, len);
#endif
      memcpy(resids + it*len, d, len*sizeof(double));
      Ss = inlidxs(d, len, th, inliers);
#ifdef __HASHING__
      hash = SuperFastHash((const char *)inliers, Ss.I * sizeof(*inliers));
      iterIDret = htContains(&HASH_TABLE, hash, Ss.I, iterID);
      if (iterIDret != -1 && iterIDret != iterID) {
          S.I = 0;
          S.J = 0;
          return S;
        }
      if (iterIDret == -1) {
          htInsert(&HASH_TABLE, hash, Ss.I, iterID);
        }
#endif //__HASHING__
      S = inlidxs (d, len, ths*MWM, inliers);
      
      if (scoreLess(maxS, Ss))
        {
          maxS = Ss;
          errs[1] = errs[0];
          errs[0] = d;
          d = errs[1];
          memcpy(H,h,9*sizeof(double)); /*!!!*/
        }

      if (S.I < 4)
        {
          return maxS;
        }
      
#ifdef __D3__
      //D3 - calculate number of inliers detached for LSQ
      detachedCount = (int)(S.I * D3_H_RATIO);
      if (detachedCount < D3_H_MIN) {
          detachedCount = D3_H_MIN;
        }
      if (detachedCount > inlLimit) {
          detachedCount = inlLimit;
        }
      if (detachedCount < 4) {
          detachedCount = 4;
        }
      if (detachedCount >= S.I) { // if we want to use more than (or all) we have, just use what we have without shuffling
          u2h(u, inliers, S.I, h, buffer);
        } else {
          detachedInl = randsubset (inliers, S.I, detachedCount);
          u2h(u, detachedInl, detachedCount, h, buffer);
        }
#else
      u2h(u, inliers, S.I, h, buffer);
#endif
      
      ths -= dth;
    }
#ifdef FULL_SYMM
  HDsSym (Z, u, h, d, len);
#else
  HDs (Z, u, h, d, len);
#endif
  memcpy(resids + 4*len, d, len*sizeof(double));
  S = inlidxs (d, len, th, inliers);
  if (scoreLess(maxS, S))
    {
      maxS = S;
      errs[1] = errs[0];
      errs[0] = d;
      memcpy(H,h,9*sizeof(double));
    }

  return maxS;
}



Score exp_inHrani (double *u, int len, int *inliers, int ninl,
                   double th, double *Z, double **errs,
                   double *buffer, double *H, int rep,
                   int * iterID, unsigned inlLimit, double *resids)
{
  int ssiz, i;
  Score S, maxS = {0,0};
  double *d, h[9];
  int *sample;
  int *intbuff;

  intbuff = (int *) malloc(sizeof(int) * len);

  if (ninl < 8) {
      memset(resids, 0xFF, (RESIDS_M-2)*len*sizeof(double));
      free(intbuff);
      return maxS;
    }
  ssiz = ninl /2;
  if (ssiz > 12) ssiz = 12;


  d = errs[2];
  errs[2] = errs[0];
  errs[0] = d;

  for (i = 0; i < rep; i++)
    {
      sample = randsubset(inliers, ninl, ssiz);
      u2h(u, sample, ssiz, h, buffer);
#ifdef FULL_SYMM
      HDsSym(Z, u, h, errs[0], len);
#else
      HDs (Z, u, h, errs[0], len);
#endif
      memcpy(resids + i*6*len, errs[0], len*sizeof(double)); // pointer to resids already moved to the 3rd field of current part
      errs[4] = errs[0];

      S = exp_iterH(u, len, intbuff, th, TC*th, ILSQ_ITERS, h, Z, errs, buffer, ++*iterID, inlLimit, resids + i*6*len + len);

      if (scoreLess(maxS, S))
        {
          maxS = S;
          d = errs[2];
          errs[2] = errs[0];
          errs[0] = d;
          memcpy(H,h,9*sizeof(double)); /*!!!*/
        }
    }

  d = errs[2];
  errs[2] = errs[0];
  errs[0] = d;

  free(intbuff);
  return maxS;
}


Score exp_ransacH (double *u, int len, double th, double conf, int max_sam,
                   double *H, unsigned char * inl,
                   int iter_type, int * data_out,
                   int oriented_constraint, unsigned inlLimit, double **resids)
{
  int *pool, no_sam, new_sam, *samidx, bestsamidx[4];
  double *Z, *buffer;
  double tol, v;
  double M[9*9], sol[9*9], *h;

  double *err, *d;
  double *errs[5];
  int i, j, *inliers;
  Score maxS = {0,0}, maxSs = {0,0}, S = {0,0};
  unsigned seed;
  int do_iterate;
  int iter_cnt = 0, no_rej = 0, iterID = 0;
  char new_max = 0;
  int nullspace_buff[2*9], nullsize;

  if (inlLimit == 0) { /* in the case of unlimited least squares */
      inlLimit = 1e6;
    }
  h = sol;

#ifdef __HASHING__
  htInit(&HASH_TABLE);
#endif // __HASHING__

  /* allocations */

  pool = (int *)malloc(len * sizeof(int));
  for (i = 0; i < len; i ++)
    pool[i] = i;

  Z = (double *) malloc(len * 18 * sizeof(double));
  lin_hg(u, Z, pool, len);

  buffer = (double *) malloc (len * 18 * sizeof(double));

  err = (double *) malloc(len * 4 * sizeof(double));
  for (i=0; i<4; i++)
    errs[i] = err + i * len;
  errs[4] = errs[3];

  inliers = (int *) malloc(sizeof(int) * len);

  no_sam = 0;
  seed = rand();

  samidx = pool + len - 4;

  *resids = (double *) malloc (iter_cnt * RESIDS_M * len * sizeof(double));

  /* RANSAC */

  while(no_sam < max_sam)
    {
      no_sam ++;
      srand(seed);
      multirsampleT(Z, 9, 2, pool, 4, len, M);
      seed = rand();

      /* orientation */
#ifndef __OC_OFF__
      if (oriented_constraint && !all_Hori_valid (u, samidx))
        {
          no_rej ++;
          continue;
        }
#endif
      /* nullspace */
      for (i = 9*8; i < 9*9; ++i) { /* Fill with zeros to square */
          M[i] = 0.0;
        }
      nullsize = nullspace(M, sol, 9, nullspace_buff); /* nullspace function expects M row-wise */
      if (nullsize != 1) {
          continue;
        }

      v = det3(h); tol = h[8];
      if (tol == 0) {
          for (i = 0; i < 9; ++i) { /* Frobenius norm */
              tol += h[i]*h[i];
            }
          tol = sqrt(tol);
          tol *= 0.001; /* typical ratio H(3,3)/||H||_F */
        }
      tol = tol*tol*tol;
      if (fabs(v/tol) < 10e-2) continue; /* reject H's close to singular */

      d = errs[0];
#ifdef FULL_SYMM
      HDsSym(Z, u, h, d, len);
#else
      HDs(Z, u, h, d, len);
#endif
      S.I = 0;
      S.J = 0;
      
      for (j = 0; j < len; j++) {
          if (d[j] <= th) S.I++;
          S.J += truncQuad(d[j], th);
        }
      if(scoreLess(maxS, S))
        {
          errs[0] = errs[3];
          errs[3] = d;
          maxS = S;
          new_max = 1;
          memcpy(H, h, 9 * sizeof(double));
        }

      if(scoreLess(maxSs, S))
        {
          do_iterate = no_sam > ITER_SAM;
          maxSs = S;
          errs[4] = d;
          memcpy(bestsamidx, samidx, 4 * sizeof(int));
        } else
        do_iterate = 0;

      /*      data_out[I+2] ++; */
      if ((no_sam >= ITER_SAM) && (iter_cnt == 0) && (maxSs.I > 4))
        do_iterate = 1;

      if (do_iterate)
        {
          /* ITERATIONS */
          iter_cnt ++;

          *resids = (double *) realloc(*resids, iter_cnt * RESIDS_M * len * sizeof(double));
          switch(iter_type)
            {
            case 0:
              break;
            case 1:
              S = inlidxs(errs[4], len, TC*th, inliers);
              u2h(u, inliers, S.I, h, buffer);
              d = errs[0];
#ifdef FULL_SYMM
              HDsSym(Z, u, h, d, len);
#else
              HDs(Z, u, h, d, len);
#endif
              S.I = 0;
              S.J = 0;
              for (j = 0; j < len; j++) {
                  if (d[j] <= th) S.I++;
                  S.J += truncQuad(d[j], th);
                }
              break;
            case 2:
              S = exp_iterH(u, len, inliers, th, TC*th, 4, h, Z, errs, buffer, ++iterID, inlLimit, *resids + 2*len + (iter_cnt-1)*RESIDS_M*len);
              break;
            case 3:
              d = errs[0];
              S = inlidxs(errs[4], len, TC*th, inliers);
              u2h(u, inliers, S.I, h, buffer);
#ifdef FULL_SYMM
              HDsSym(Z, u, h, d, len);
#else
              HDs(Z, u, h, d, len);
#endif
              S = inlidxs(d, len, th, inliers);

              //I = inHran (u, len, inliers, I, th, Z, errs, buffer, h, RAN_REP); //because of deleting unnecessary inHran. It MUST be back there if using case 3!!! (+once more in ALO)
              break;
            case 4:
              memcpy(*resids + RESIDS_M*(iter_cnt - 1)*len, errs[4], len*sizeof(double));
#ifdef __LSQ_BEFORE_LO__
              d = errs[0];
#ifdef __LSBL_MCE__
	      hMCEs(Z, u, d, samidx, len, errs[4], TC*th*MWM);
	      S = inlidxs(d, len, TC*th*MWM, inliers);
#else /* __LSBL_MCE__ */
	      S = inlidxs(errs[4], len, TC*th*MWM, inliers);
#endif /* __LSBL_MCE__ */
	      u2h(u, inliers, S.I, h, buffer);
#ifdef FULL_SYMM
	      HDsSym(Z, u, h, d, len);
#else
	      HDs(Z, u, h, d, len);
#endif
#ifdef __IB_MCE__
              hMCEs(Z, u, d, samidx, len, d, th);
#endif /* __IB_MCE__ */
              S = inlidxs(d, len, th, inliers);
              memcpy(*resids + RESIDS_M*(iter_cnt - 1)*len + len, d, len*sizeof(double));
#else /* __LSQ_BEFORE_LO__ */
              S = inlidxs(errs[4], len, th, inliers);
#endif /* __LSQ_BEFORE_LO__ */
              S = exp_inHrani (u, len, inliers, S.I, th, Z, errs, buffer, h, RAN_REP, &iterID, inlLimit, *resids + 2*len + (iter_cnt-1)*RESIDS_M*len);
              break;
            }

          tol = h[8];
          if (tol == 0) {
              for (i = 0; i < 9; ++i) { /* Frobenius norm */
                  tol += h[i]*h[i];
                }
              tol = sqrt(tol);
              tol *= 0.001; /* typical ratio H(3,3)/||H||_F */
            }
          tol = tol*tol*tol;
          if(scoreLess(maxS, S) && (fabs(det3(h)/tol) > 10e-2))
            {
              d = errs[0];
              errs[0] = errs[3];
              errs[3] = d;
              maxS = S;
              new_max = 1;
              memcpy(H,h,9*sizeof(double));
            }
        }
      if (new_max)
        {
          new_sam = nsamples(maxS.I+1, len, 4, conf);
          if (new_sam < max_sam)
            max_sam = new_sam;
          new_max = 0;
        }
    }
  /*If there were no LOs, do at least one NOW!*/
  if (iter_cnt == 0 && iter_type != 0)
    {
      /* ITERATIONS */
      iter_cnt ++;
      *resids = (double *) realloc(*resids, iter_cnt * RESIDS_M * len * sizeof(double));
      switch(iter_type)
        {
        case 0:
          break;
        case 1:
          S = inlidxs(errs[4], len, TC*th, inliers);
          u2h(u, inliers, S.I, h, buffer);
          d = errs[0];
#ifdef FULL_SYMM
          HDsSym(Z, u, h, d, len);
#else
          HDs(Z, u, h, d, len);
#endif
          S.I = 0;
          S.J = 0;
          for (j = 0; j < len; j++) {
              if (d[j] <= th) S.I++;
              S.J += truncQuad(d[j], th);
            }
          break;
        case 2:
          S = exp_iterH(u, len, inliers, th, TC*th, 4, h, Z, errs, buffer, ++iterID, inlLimit, *resids + 2*len + (iter_cnt-1)*RESIDS_M*len);
          break;
        case 3:
          d = errs[0];
          S = inlidxs(errs[4], len, TC*th, inliers);
          u2h(u, inliers, S.I, h, buffer);
#ifdef FULL_SYMM
          HDsSym(Z, u, h, d, len);
#else
          HDs(Z, u, h, d, len);
#endif
          S = inlidxs(d, len, th, inliers);

          //I = inHran (u, len, inliers, I, th, Z, errs, buffer, h, RAN_REP);
          break;
        case 4:
          memcpy(*resids + RESIDS_M*(iter_cnt - 1)*len, errs[4], len*sizeof(double));
#ifdef __LSQ_BEFORE_LO__
          d = errs[0];
#ifdef __LSBL_MCE__
	  hMCEs(Z, u, d, bestsamidx, len, errs[4], TC*th*MWM);
	  S = inlidxs(d, len, TC*th*MWM, inliers);
#else /* __LSBL_MCE__ */
	  S = inlidxs(errs[4], len, TC*th*MWM, inliers);
#endif /* __LSBL_MCE__ */
	  u2h(u, inliers, S.I, h, buffer);
#ifdef FULL_SYMM
	  HDsSym(Z, u, h, d, len);
#else
	  HDs(Z, u, h, d, len);
#endif
#ifdef __IB_MCE__
          hMCEs(Z, u, d, bestsamidx, len, d, th);
#endif /* __IB_MCE__ */
          S = inlidxs(d, len, th, inliers);
          memcpy(*resids + RESIDS_M*(iter_cnt - 1)*len + len, d, len*sizeof(double));
#else /* __LSQ_BEFORE_LO__ */
          S = inlidxs(errs[4], len, th, inliers);
#endif /* __LSQ_BEFORE_LO__ */
          S = exp_inHrani (u, len, inliers, S.I, th, Z, errs, buffer, h, RAN_REP, &iterID, inlLimit, *resids + 2*len + (iter_cnt-1)*RESIDS_M*len);
          break;
        }
      
      tol = h[8];
      if (tol == 0) {
          for (i = 0; i < 9; ++i) { /* Frobenius norm */
              tol += h[i]*h[i];
            }
          tol = sqrt(tol);
          tol *= 0.001; /* typical ratio H(3,3)/||H||_F */
        }
      tol = tol*tol*tol;
      if(scoreLess(maxS, S) && (fabs(det3(h)/tol) > 10e-2))
        {
          d = errs[0];
          errs[0] = errs[3];
          errs[3] = d;
          maxS = S;
          memcpy(H,h,9*sizeof(double));
        }
    }

  d = errs[3];

#ifdef __FINAL_LSQ__
  S = inlidxs(d, len, th, inliers); //Final LSq
  u2h(u, inliers, S.I, H, buffer);
#ifdef FULL_SYMM
  HDsSym(Z, u, H, d, len);
#else
  HDs(Z, u, H, d, len);
#endif
#endif

  for (j = 0; j < len; j++) {
      if (d[j] <= th) {
          inl[j] = 1;
        }
      else inl[j] = 0;
    }
  //   printf("Orientation: %.2f, det: %f\n", 100.0 *no_rej / no_sam,fabs(det3(H)/H[8]));

  /* deallocations */

#ifdef __HASHING__
  htClear(&HASH_TABLE);
#endif // __HASHING__

  free(pool);
  free(Z);
  free(buffer);
  free(err);
  free(inliers);

  *data_out = no_sam;
  data_out[1] = iter_cnt;
  if (iter_type == 0) {
      data_out[1] = 0;
    }
  data_out[2] = no_rej;

  return maxS;
}

void hMCEs(double *Z, double *u, double *d, int *samidx, int len, double * errs, double thr) {
  int i, j;
  double C[9*9], V[9*9], *Cc = V;
  double A1[3], A2[3];
  double A[10*9];
  double h[3*3], e[4];

  normu (u, samidx, 4, A1, A2); /* Constrained optimization according to Hartley&Zisserman, A5.4 */
  lin_hgN(u, A, samidx, 4, A1, A2);

  for (i = 0; i < len; ++i) {
      if (errs[i] <= thr) {
          d[i] = errs[i];
          continue;
        }
#ifndef __MCE_SOFT__
      lin_hgN(u, C, &i, 1, A1, A2);
      for (j = 2*9; j < 9*9; ++j) {
          C[j] = 0.0;
        }
      trnm(C, 9); /* LAPACK needs C stored column-wise */
      nullsize = 7;
      /*		nullsize = nullspace(C, Cc, 9, nullspace_buff);*/
      lap_SVD (ACc, C, CcT, 9, V, 9); /* V <- V^T column-wise, ignore ACc & CcT */
      trnm(V, 9); /* V column-wise */
      Cc = V + 9*(9-nullsize); /* C complement column-wise */
      mattr(CcT, Cc, nullsize, 9); /* C complement row-wise */
      rmmult(ACc, A, CcT, 8, 9, nullsize);
      cov_mat(C, ACc, 8, nullsize);
      lap_eig(C, Cc, nullsize); /* Least squares, x' in C */
      rmmult(h, CcT, C, 9, nullsize, 1); /* x = Cc . x' */
#else /* __MCE_SOFT__ */
      lin_hgN(u, A + 8*9, &i, 1, A1, A2); /* Add one pair of equations */
      scalmul(A + 8*9, MCE_SOFT_WEIGHT, 2*9, 1); /* Weight the equations */
      cov_mat(C, A, 10, 9);
      lap_eig(C, Cc, 9); /* Solve for h */
      memcpy(h, C, 3*3*sizeof(double));
#endif /* __MCE_SOFT__ */
      denormH(h, A1, A2);
#ifdef FULL_SYMM
      HDsiSym(Z, u, h, e, len, samidx, 4);
#else
      HDsi(Z, u, h, e, len, samidx, 4);
#endif
      d[i] = 0;
      for (j = 0; j < 4; ++j) {
          d[i] += e[j];
        }
      d[i] /= 4;
    }


}


Score exp_iterHcustom(double *u, int len, int *inliers, double th, double ths,
                      int steps, double *H, double *Z, double **errs, double *buffer,
                      int iterID, unsigned inlLimit, double *resids, HDsPtr HDS1)
{
  double *d = errs[1];
  double h[9], dth;
  int it;
  Score maxS = {0,0}, S = {0,0}, Ss;
#ifdef __D3__
  int * detachedInl;
  unsigned detachedCount;
#endif
#ifdef __HASHING__
  int iterIDret;
  uint32_t hash;
#endif //__HASHING__

  dth = (ths - th) / (steps);

  /* H from the sample inliers by th */

  maxS = inlidxs(errs[4], len, th, inliers);
  if (maxS.I < 4)
    return S;
  S = inlidxs(errs[4], len, th*MWM, inliers);
#ifdef __D3__
  //D3 - calculate number of inliers detached for LSQ
  detachedCount = (int)(S.I * D3_H_RATIO);
  if (detachedCount < D3_H_MIN) {
      detachedCount = D3_H_MIN;
    }
  if (detachedCount > inlLimit) {
      detachedCount = inlLimit;
    }
  if (detachedCount < 4) {
      detachedCount = 4;
    }
  if (detachedCount >= S.I) { // if we want to use more than (or all) we have, just use what we have without shuffling
      u2h(u, inliers, S.I, h, buffer);
    } else {
      detachedInl = randsubset (inliers, S.I, detachedCount);
      u2h(u, detachedInl, detachedCount, h, buffer);
    }
#else
  u2h(u, inliers, S.I, h, buffer);
#endif

  /*iterate */

  for (it = 0; it < steps; it ++)
    {
      HDS1 (Z, u, h, d, len);

      memcpy(resids + it*len, d, len*sizeof(double));
      Ss = inlidxs(d, len, th, inliers);
#ifdef __HASHING__
      hash = SuperFastHash((const char *)inliers, Ss.I * sizeof(*inliers));
      iterIDret = htContains(&HASH_TABLE, hash, Ss.I, iterID);
      if (iterIDret != -1 && iterIDret != iterID) {
          S.I = 0;
          S.J = 0;
          return S;
        }
      if (iterIDret == -1) {
          htInsert(&HASH_TABLE, hash, Ss.I, iterID);
        }
#endif //__HASHING__
      S = inlidxs (d, len, ths*MWM, inliers);

      if (scoreLess(maxS, Ss))
        {
          maxS = Ss;
          errs[1] = errs[0];
          errs[0] = d;
          d = errs[1];
          memcpy(H,h,9*sizeof(double)); /*!!!*/
        }

      if (S.I < 4)
        {
          return maxS;
        }

#ifdef __D3__
      //D3 - calculate number of inliers detached for LSQ
      detachedCount = (int)(S.I * D3_H_RATIO);
      if (detachedCount < D3_H_MIN) {
          detachedCount = D3_H_MIN;
        }
      if (detachedCount > inlLimit) {
          detachedCount = inlLimit;
        }
      if (detachedCount < 4) {
          detachedCount = 4;
        }
      if (detachedCount >= S.I) { // if we want to use more than (or all) we have, just use what we have without shuffling
          u2h(u, inliers, S.I, h, buffer);
        } else {
          detachedInl = randsubset (inliers, S.I, detachedCount);
          u2h(u, detachedInl, detachedCount, h, buffer);
        }
#else
      u2h(u, inliers, S.I, h, buffer);
#endif

      ths -= dth;
    }
  HDS1 (Z, u, h, d, len);

  memcpy(resids + 4*len, d, len*sizeof(double));
  S = inlidxs (d, len, th, inliers);
  if (scoreLess(maxS, S))
    {
      maxS = S;
      errs[1] = errs[0];
      errs[0] = d;
      memcpy(H,h,9*sizeof(double));
    }

  return maxS;
}



Score exp_inHranicustom (double *u, int len, int *inliers, int ninl,
                         double th, double *Z, double **errs,
                         double *buffer, double *H, int rep,
                         int * iterID, unsigned inlLimit, double *resids, HDsPtr HDS1)
{
  int ssiz, i;
  Score S, maxS = {0,0};
  double *d, h[9];
  int *sample;
  int *intbuff;

  intbuff = (int *) malloc(sizeof(int) * len);

  if (ninl < 8) {
      memset(resids, 0xFF, (RESIDS_M-2)*len*sizeof(double));
      free(intbuff);
      return maxS;
    }
  ssiz = ninl /2;
  if (ssiz > 12) ssiz = 12;


  d = errs[2];
  errs[2] = errs[0];
  errs[0] = d;

  for (i = 0; i < rep; i++)
    {
      sample = randsubset(inliers, ninl, ssiz);
      u2h(u, sample, ssiz, h, buffer);
      HDS1 (Z, u, h, errs[0], len);
      memcpy(resids + i*6*len, errs[0], len*sizeof(double)); // pointer to resids already moved to the 3rd field of current part
      errs[4] = errs[0];

      S = exp_iterHcustom(u, len, intbuff, th, TC*th, ILSQ_ITERS, h, Z, errs, buffer, ++*iterID, inlLimit, resids + i*6*len + len,HDS1);

      if (scoreLess(maxS, S))
        {
          maxS = S;
          d = errs[2];
          errs[2] = errs[0];
          errs[0] = d;
          memcpy(H,h,9*sizeof(double)); /*!!!*/
        }
    }

  d = errs[2];
  errs[2] = errs[0];
  errs[0] = d;

  free(intbuff);
  return maxS;
}


Score exp_ransacHcustom (double *u, int len, double th, double conf, int max_sam,
                         double *H, unsigned char * inl,
                         int iter_type, int * data_out,
                         int oriented_constraint, unsigned inlLimit, double **resids, HDsPtr HDS1, HDsiPtr HDSi1, HDsidxPtr HDSidx1, int doSymCheck)
{
  int *pool, no_sam, new_sam, *samidx, bestsamidx[4];
  double *Z, *buffer;
  double tol, v;
  double M[9*9], sol[9*9], *h;

  double *err, *d, *d_check;
  double *errs[5];
  int i, j, *inliers;
  Score maxS = {0,0}, maxSs = {0,0}, S = {0,0}, Scheck= {0,0};
  unsigned seed;
  int do_iterate;
  int iter_cnt = 0, no_rej = 0, iterID = 0;
  char new_max = 0;
  int bad_model = 0; //Mishkin
  // int rr;
  int nullspace_buff[2*9], nullsize;

  if (inlLimit == 0) { /* in the case of unlimited least squares */
      inlLimit = 1e6;
    }
  h = sol;
  //
  srand(time(NULL)); //Mishkin - randomization

#ifdef __HASHING__
  htInit(&HASH_TABLE);
#endif // __HASHING__

  /* allocations */

  pool = (int *)malloc(len * sizeof(int));
  for (i = 0; i < len; i ++)
    pool[i] = i;

  Z = (double *) malloc(len * 18 * sizeof(double));
  lin_hg(u, Z, pool, len);

  buffer = (double *) malloc (len * 18 * sizeof(double));

  err = (double *) malloc(len * 4 * sizeof(double));
  d_check = (double *) malloc(len * sizeof(double));

  for (i=0; i<4; i++)
    errs[i] = err + i * len;
  errs[4] = errs[3];

  inliers = (int *) malloc(sizeof(int) * len);

  no_sam = 0;
  seed = rand();

  samidx = pool + len - 4;

  *resids = (double *) malloc (iter_cnt * RESIDS_M * len * sizeof(double));

  /* RANSAC */

  while(no_sam < max_sam)
    {
      no_sam++;
      srand(seed);
      multirsampleT(Z, 9, 2, pool, 4, len, M);
      seed = rand();

      /* orientation */
#ifndef __OC_OFF__
      if (oriented_constraint && !all_Hori_valid (u, samidx))
        {
          no_rej ++;
          continue;
        }
#endif

      /* nullspace */
      for (i = 9*8; i < 9*9; ++i) { /* Fill with zeros to square */
          M[i] = 0.0;
        }
      nullsize = nullspace(M, sol, 9, nullspace_buff); /* nullspace function expects M row-wise */
      if (nullsize != 1) {
          continue;
        }

      v = det3(h); tol = h[8];
      if (tol == 0) {
          for (i = 0; i < 9; ++i) { /* Frobenius norm */
              tol += h[i]*h[i];
            }
          tol = sqrt(tol);
          tol *= 0.001; /* typical ratio H(3,3)/||H||_F */
        }
      tol = tol*tol*tol;
      if (fabs(v/tol) < 10e-2) continue; /* reject H's close to singular */

      d = errs[0];
      HDS1(Z, u, h, d, len);

      S.I = 0;
      S.J = 0;

      for (j = 0; j < len; j++) {
          if (d[j] <= th) S.I++;
          S.J += truncQuad(d[j], th);
        }

      if(scoreLess(maxS, S))
        {
          ///
          if (doSymCheck) //Mishkin. Check by symmetrical distance
            {
              //              bad_model = 0;
              //              double Hinvtemp[9];
              //              for (rr=0; rr<9; rr++)
              //                Hinvtemp[rr] = h[rr];
              //              minv(Hinvtemp,3);

              //              double tol1 = Hinvtemp[8];
              //              if (tol1 == 0) {
              //                  for (rr = 0; rr < 9; ++rr) { /* Frobenius norm */
              //                      tol1 += Hinvtemp[rr]*Hinvtemp[rr];
              //                    }
              //                  tol1 = sqrt(tol1);
              //                  tol1 *= 0.001; /* typical ratio H(3,3)/||H||_F */
              //                }
              //              tol1 = tol1*tol1*tol1;
              //              if  (fabs(det3(Hinvtemp)/tol1) <= 10e-2)
              //                bad_model = 1;

              HDsSym(Z, u, h, d_check, len);
              Scheck.I = 0;
              bad_model = 0;
              double th_check =  CHECK_COEF*th;
              for (j = 0; j < len; j++)
                if (d_check[j] <= th_check) Scheck.I++;
              //      printf("Inliers Symm = %d, inliers_Samp = %d\n",Scheck.I,S.I);
              if (Scheck.I <= MIN_GOOD_SYM_PTS)
                bad_model = 1;
            }
          if (bad_model) continue;
          else {
              ////
              errs[0] = errs[3];
              errs[3] = d;
              maxS = S;
              new_max = 1;
              memcpy(H, h, 9 * sizeof(double));
            }
        }
      if(scoreLess(maxSs, S))
        {
          do_iterate = no_sam > ITER_SAM;
          maxSs = S;
          errs[4] = d;
          memcpy(bestsamidx, samidx, 4 * sizeof(int));
        } else
        do_iterate = 0;

      /*      data_out[I+2] ++; */
      if ((no_sam >= ITER_SAM) && (iter_cnt == 0) && (maxSs.I > 4))
        do_iterate = 1;

      if (do_iterate)
        {
          /* ITERATIONS */
          iter_cnt ++;

          *resids = (double *) realloc(*resids, iter_cnt * RESIDS_M * len * sizeof(double));
          switch(iter_type)
            {
            case 0:
              break;
            case 1:
              S = inlidxs(errs[4], len, TC*th, inliers);
              u2h(u, inliers, S.I, h, buffer);
              d = errs[0];

              HDS1(Z, u, h, d, len);
              S.I = 0;
              S.J = 0;
              for (j = 0; j < len; j++) {
                  if (d[j] <= th) S.I++;
                  S.J += truncQuad(d[j], th);
                }
              break;
            case 2:
              S = exp_iterHcustom(u, len, inliers, th, TC*th, 4, h, Z, errs, buffer, ++iterID, inlLimit, *resids + 2*len + (iter_cnt-1)*RESIDS_M*len,HDS1);
              break;
            case 3:
              d = errs[0];
              S = inlidxs(errs[4], len, TC*th, inliers);
              u2h(u, inliers, S.I, h, buffer);
              HDS1(Z, u, h, d, len);

              S = inlidxs(d, len, th, inliers);

              //I = inHran (u, len, inliers, I, th, Z, errs, buffer, h, RAN_REP); //because of deleting unnecessary inHran. It MUST be back there if using case 3!!! (+once more in ALO)
              break;
            case 4:
              memcpy(*resids + RESIDS_M*(iter_cnt - 1)*len, errs[4], len*sizeof(double));
#ifdef __LSQ_BEFORE_LO__
              d = errs[0];
#ifdef __LSBL_MCE__
	      hMCEscustom(Z, u, d, samidx, len, errs[4], TC*th*MWM,HDSi1);
	      S = inlidxs(d, len, TC*th*MWM, inliers);
#else /* __LSBL_MCE__ */
	      S = inlidxs(errs[4], len, TC*th*MWM, inliers);
#endif /* __LSBL_MCE__ */
	      u2h(u, inliers, S.I, h, buffer);
	      HDS1(Z, u, h, d, len);

#ifdef __IB_MCE__
              hMCEscustom(Z, u, d, samidx, len, d, th,HDSi1);
#endif /* __IB_MCE__ */
              S = inlidxs(d, len, th, inliers);
              memcpy(*resids + RESIDS_M*(iter_cnt - 1)*len + len, d, len*sizeof(double));
#else /* __LSQ_BEFORE_LO__ */
              S = inlidxs(errs[4], len, th, inliers);
#endif /* __LSQ_BEFORE_LO__ */
              S = exp_inHranicustom(u, len, inliers, S.I, th, Z, errs, buffer, h, RAN_REP, &iterID, inlLimit, *resids + 2*len + (iter_cnt-1)*RESIDS_M*len,HDS1);
              break;
            }

          tol = h[8];
          if (tol == 0) {
              for (i = 0; i < 9; ++i) { /* Frobenius norm */
                  tol += h[i]*h[i];
                }
              tol = sqrt(tol);
              tol *= 0.001; /* typical ratio H(3,3)/||H||_F */
            }
          tol = tol*tol*tol;

          //  printf("Check = %f\n", fabs(det3(h)/tol));
          if(scoreLess(maxS, S) && (fabs(det3(h)/tol) > 10e-2))
            {
              ///
              if (doSymCheck) //Mishkin. Check by symmetrical distance
                {
                  HDsSym(Z, u, h, d_check, len);
                  Scheck.I = 0;
                  bad_model = 0;
                  double th_check =  CHECK_COEF*th;
                  for (j = 0; j < len; j++)
                    if (d_check[j] <= th_check) Scheck.I++;
                  if (Scheck.I <= MIN_GOOD_SYM_PTS)
                    bad_model = 1;

                  //                  bad_model = 0;
                  //                  double Hinvtemp[9];
                  //                  for (rr=0; rr<9; rr++)
                  //                    Hinvtemp[rr] = h[rr];
                  //                  minv(Hinvtemp,3);

                  //                  double tol1 = Hinvtemp[8];
                  //                  if (tol1 == 0) {
                  //                      for (rr = 0; rr < 9; ++rr) { /* Frobenius norm */
                  //                          tol1 += Hinvtemp[rr]*Hinvtemp[rr];
                  //                        }
                  //                      tol1 = sqrt(tol1);
                  //                      tol1 *= 0.001; /* typical ratio H(3,3)/||H||_F */
                  //                    }
                  //                  tol1 = tol1*tol1*tol1;
                  //                  if  (fabs(det3(Hinvtemp)/tol1) <= 10e-2)
                  //                    bad_model = 1;
                }
              if (!bad_model) {
                  ////
                  d = errs[0];
                  errs[0] = errs[3];
                  errs[3] = d;
                  maxS = S;
                  new_max = 1;
                  memcpy(H,h,9*sizeof(double));
                }
            }
        }
      if (new_max)
        {
          new_sam = nsamples(maxS.I+1, len, 4, conf);
          if (new_sam < max_sam)
            max_sam = new_sam;
          new_max = 0;
        }
    }
  /*If there were no LOs, do at least one NOW!*/
  if (iter_cnt == 0 && iter_type != 0)
    {
      /* ITERATIONS */
      iter_cnt ++;
      *resids = (double *) realloc(*resids, iter_cnt * RESIDS_M * len * sizeof(double));
      switch(iter_type)
        {
        case 0:
          break;
        case 1:
          S = inlidxs(errs[4], len, TC*th, inliers);
          u2h(u, inliers, S.I, h, buffer);
          d = errs[0];
          HDS1(Z, u, h, d, len);

          S.I = 0;
          S.J = 0;
          for (j = 0; j < len; j++) {
              if (d[j] <= th) S.I++;
              S.J += truncQuad(d[j], th);
            }
          break;
        case 2:
          S = exp_iterHcustom(u, len, inliers, th, TC*th, 4, h, Z, errs, buffer, ++iterID, inlLimit, *resids + 2*len + (iter_cnt-1)*RESIDS_M*len,HDS1);
          break;
        case 3:
          d = errs[0];
          S = inlidxs(errs[4], len, TC*th, inliers);
          u2h(u, inliers, S.I, h, buffer);
          HDS1(Z, u, h, d, len);

          S = inlidxs(d, len, th, inliers);

          //I = inHran (u, len, inliers, I, th, Z, errs, buffer, h, RAN_REP);
          break;
        case 4:
          memcpy(*resids + RESIDS_M*(iter_cnt - 1)*len, errs[4], len*sizeof(double));
#ifdef __LSQ_BEFORE_LO__
          d = errs[0];
#ifdef __LSBL_MCE__
	  hMCEscustom(Z, u, d, bestsamidx, len, errs[4], TC*th*MWM,HDSi1);
	  S = inlidxs(d, len, TC*th*MWM, inliers);
#else /* __LSBL_MCE__ */
	  S = inlidxs(errs[4], len, TC*th*MWM, inliers);
#endif /* __LSBL_MCE__ */
	  u2h(u, inliers, S.I, h, buffer);
	  HDS1(Z, u, h, d, len);

#ifdef __IB_MCE__
          hMCEscustom(Z, u, d, bestsamidx, len, d, th,HDSi1);
#endif /* __IB_MCE__ */
          S = inlidxs(d, len, th, inliers);
          memcpy(*resids + RESIDS_M*(iter_cnt - 1)*len + len, d, len*sizeof(double));
#else /* __LSQ_BEFORE_LO__ */
          S = inlidxs(errs[4], len, th, inliers);
#endif /* __LSQ_BEFORE_LO__ */
          S = exp_inHranicustom(u, len, inliers, S.I, th, Z, errs, buffer, h, RAN_REP, &iterID, inlLimit, *resids + 2*len + (iter_cnt-1)*RESIDS_M*len,HDS1);
          break;
        }

      tol = h[8];
      if (tol == 0) {
          for (i = 0; i < 9; ++i) { /* Frobenius norm */
              tol += h[i]*h[i];
            }
          tol = sqrt(tol);
          tol *= 0.001; /* typical ratio H(3,3)/||H||_F */
        }
      tol = tol*tol*tol;
      //
      // printf("Check = %f\n", fabs(det3(h)/tol));
      //
      if(scoreLess(maxS, S) && (fabs(det3(h)/tol) > 10e-2))
        {
          ///
          if (doSymCheck) //Mishkin. Check by symmetrical distance
            {
              //              bad_model = 0;
              //              double Hinvtemp[9];
              //              for (rr=0; rr<9; rr++)
              //                Hinvtemp[rr] = h[rr];
              //              minv(Hinvtemp,3);

              //              double tol1 = Hinvtemp[8];
              //              if (tol1 == 0) {
              //                  for (rr = 0; rr < 9; ++rr) { /* Frobenius norm */
              //                      tol1 += Hinvtemp[rr]*Hinvtemp[rr];
              //                    }
              //                  tol1 = sqrt(tol1);
              //                  tol1 *= 0.001; /* typical ratio H(3,3)/||H||_F */
              //                }
              //              tol1 = tol1*tol1*tol1;
              //              if  (fabs(det3(Hinvtemp)/tol1) <= 10e-2)
              //                bad_model = 1;

              HDsSym(Z, u, h, d_check, len);
              Scheck.I = 0;
              bad_model = 0;
              double th_check =  CHECK_COEF*th;
              for (j = 0; j < len; j++)
                if (d_check[j] <= th_check) Scheck.I++;
              if (Scheck.I <= MIN_GOOD_SYM_PTS)
                bad_model = 1;
            }
          if (!bad_model) {
              d = errs[0];
              errs[0] = errs[3];
              errs[3] = d;
              maxS = S;
              memcpy(H,h,9*sizeof(double));
            }
        }
    }

  d = errs[3];

#ifdef __FINAL_LSQ__
  S = inlidxs(d, len, th, inliers); //Final LSq
  u2h(u, inliers, S.I, H, buffer);
  HDS1(Z, u, H, d, len);
#endif

  for (j = 0; j < len; j++) {
      if (d[j] <= th) {
          inl[j] = 1;
        }
      else inl[j] = 0;
    }
  //   printf("Orientation: %.2f, det: %f\n", 100.0 *no_rej / no_sam,fabs(det3(H)/H[8]));

  /* deallocations */

#ifdef __HASHING__
  htClear(&HASH_TABLE);
#endif // __HASHING__

  free(pool);
  free(Z);
  free(buffer);
  free(err);
  free(inliers);
  free(d_check);

  *data_out = no_sam;
  data_out[1] = iter_cnt;
  if (iter_type == 0) {
      data_out[1] = 0;
    }
  data_out[2] = no_rej;

  return maxS;
}

void hMCEscustom(double *Z, double *u, double *d, int *samidx, int len, double * errs, double thr,HDsiPtr HDSi1) {
  int i, j;
  double C[9*9], V[9*9], *Cc = V;
  double A1[3], A2[3];
  double A[10*9];
  double h[3*3], e[4];

  normu (u, samidx, 4, A1, A2); /* Constrained optimization according to Hartley&Zisserman, A5.4 */
  lin_hgN(u, A, samidx, 4, A1, A2);

  for (i = 0; i < len; ++i) {
      if (errs[i] <= thr) {
          d[i] = errs[i];
          continue;
        }
#ifndef __MCE_SOFT__
      lin_hgN(u, C, &i, 1, A1, A2);
      for (j = 2*9; j < 9*9; ++j) {
          C[j] = 0.0;
        }
      trnm(C, 9); /* LAPACK needs C stored column-wise */
      nullsize = 7;
      /*		nullsize = nullspace(C, Cc, 9, nullspace_buff);*/
      lap_SVD (ACc, C, CcT, 9, V, 9); /* V <- V^T column-wise, ignore ACc & CcT */
      trnm(V, 9); /* V column-wise */
      Cc = V + 9*(9-nullsize); /* C complement column-wise */
      mattr(CcT, Cc, nullsize, 9); /* C complement row-wise */
      rmmult(ACc, A, CcT, 8, 9, nullsize);
      cov_mat(C, ACc, 8, nullsize);
      lap_eig(C, Cc, nullsize); /* Least squares, x' in C */
      rmmult(h, CcT, C, 9, nullsize, 1); /* x = Cc . x' */
#else /* __MCE_SOFT__ */
      lin_hgN(u, A + 8*9, &i, 1, A1, A2); /* Add one pair of equations */
      scalmul(A + 8*9, MCE_SOFT_WEIGHT, 2*9, 1); /* Weight the equations */
      cov_mat(C, A, 10, 9);
      lap_eig(C, Cc, 9); /* Solve for h */
      memcpy(h, C, 3*3*sizeof(double));
#endif /* __MCE_SOFT__ */
      denormH(h, A1, A2);
      HDSi1(Z, u, h, e, len, samidx, 4);

      d[i] = 0;
      for (j = 0; j < 4; ++j) {
          d[i] += e[j];
        }
      d[i] /= 4;
    }
}

