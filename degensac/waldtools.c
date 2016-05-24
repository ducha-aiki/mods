#include <stdlib.h>
#include <math.h>
#include <matutl.h>
#include "rtools.h"
#include "waldtools.h"

double wlad_getA (double ep, double del, double *la_in, double *la_out,
                  double C_tM, double C_ms)
{
  int i;
  double A, A2, C, K;

  *la_in = del / ep;
  *la_out = (1-del) / (1 - ep);
  C = 1/((1-del)*log(*la_out) + del * log(*la_in));
  K = C_tM/(C_ms*C)+1;

  A2 = K + log(K);

  for (i = 0; i < 10; i++)
    {
      A = K + log(A2);
      if (A - A2 < 1.5e-8) break;
      A2 = A;
    }

  return A;
}  

double wald_geth(double nep, double ep, double del)
{
  double al, be, x0, x1, v0, v1, h;

  al = log(del/ep);
  be = log((1-del)/(1-ep));

  x0 = log(1/(1-nep))/be;
  v0 = nep * exp(x0 *al);
  x1 = log((1-2*v0)/(1-nep))/be;
  v1 = nep * exp(x1 * al) + (1-nep) * exp(x1 * be);
  h = x0 - (x0 - x1)/(1+v0 - v1)*v0;
  return (h);
}

int wald_nsamples(int ninl, int ptNum, int samsiz, double conf, 
                  double A, struct samhist * sh)
{
  double a = 1, b = 1;
  int i;
  double h, k = 0, nep = ninl/ptNum, prolong, leta = 0;

  for (i = 0; i < samsiz; i++)
    {
      a *= ninl-i;
      b *= ptNum -i;
    }
  a = a/b;
  if (a < eps)
    return MAX_SAMPLES;
  if (1-a < eps)
    return 1;

  while (sh != NULL)
    {
      k+= sh->k;
      h = wald_geth(nep,sh->ep,sh->del);
      prolong = 1 - 1/(exp(h* log(sh->A)));
      leta += (double) sh->k * log(1-a*prolong);
      sh = sh->next;
    }

   b = k + (log(1-conf)-leta) / log(1-a*(1-(1/A)));
   if (b > MAX_SAMPLES)
   return MAX_SAMPLES; else
   return (int) ceil(b);
}

struct samhist * wald_addsamhist(double ep, double del, double A,
				 struct samhist * sh, int no_sam,
				 int * lastAchg)
{
  struct samhist * newsh;
  
  newsh = (struct samhist*) malloc(sizeof(struct samhist));
  newsh->ep = ep;
  newsh->del = del;
  newsh->A = A;
  newsh->k = no_sam - *lastAchg;
  newsh->next = sh;
  *lastAchg = no_sam;
  return newsh;
}
