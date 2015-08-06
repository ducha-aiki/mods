/**
  *    DaLI: Deformation and Light Invariant Descriptor
  *    Edgar Simo-Serra, Carme Torras, Francesc Moreno-Noguer
  *    International Journal of Computer Vision (IJCV), 2015
  *
  * Copyright (C) <2011-2015>  <Francesc Moreno-Noguer, Edgar Simo-Serra>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the version 3 of the GNU General Public License
  * as published by the Free Software Foundation.
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  * General Public License for more details.      
  * You should have received a copy of the GNU General Public License
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  *
  * Edgar Simo-Serra, Institut de Robotica i Informatica Industrial (CSIC/UPC)
  * esimo@iri.upc.edu, http://www-iri.upc.es/people/esimo/
 **/


#include "dali.h"

#include <math.h>
#include <stdio.h>
#include <assert.h>


#define MIN(a,b)  (((a)<(b))?(a):(b))
#define POW2(x)   ((x)*(x))


static void dali_updateGauss( dali_t *desc, double sigma )
{
   int i;
   double s;

   if (fabs(sigma - *desc->sigma) < 1e-10)
      return;

   s = 2.*POW2( (double)desc->sz * sigma );
   for (i=0; i<desc->len; i++)
      desc->sgauss[ i ] = exp( desc->gauss[ i ] / s );
   *desc->sigma = sigma;
}


double dali_distance( dali_t *desc1, dali_t *desc2, double sigma )
{
   int i,w;
   int ws;
   double d1, d2, dd, d, dr;
   double g, gs;

   /* Set up sizes. */
   assert( desc1->sz == desc2->sz );
   assert( desc1->len == desc2->len );

   /* Update gaussians if necessary. */
   dali_updateGauss( desc1, sigma );

   /* Set up offsets to prepare comparison. */
   ws = MIN( desc1->wlen, desc2->wlen );

   /* Calculate distance for the subset we've found. */
   gs = 0.;
   d  = 0.;
   for (i=0; i<desc1->len; i++) {
      /* Check if in mask. */
      if (desc1->mask[ i ] == 0)
         continue;
      if (desc2->mask[ i ] == 0)
         continue;

      /* Calculate Gaussian at the given position. */
      g   = desc1->sgauss[ i ] ;

      /* Increment Gaussian size. */
      gs += g;

      /* Inner loop in frequency domain has same Gaussian value. */
      dr = 0.;
      for (w=0; w<ws; w++) {
         d1  = desc1->desc[ i*desc1->wlen + w ];
         d2  = desc2->desc[ i*desc2->wlen + w ];
         dd  = d1-d2;
         dr += dd*dd;
      }
      /* We modulate the square by the Gaussian so it's a weighted sum. */
      d  += g * dr;
   }
   if (gs == 0.)
      d = NAN;
   else
      d /= gs;

   return d;
}


double dali_distance_transform( dali_t *desc1, dali_t *desc2,
      double sigma, double theta, double scale )
{
   int u,v,w;
   int ue,ve;
   int urot, vrot;
   int ws;
   double ud, vd;
   double d1, d2, dd, d, dr;
   double g, gs;
   double cR, sR;
   double w2, h2;
   int id1, id2;

   /* Set up sizes. */
   assert( desc1->sz == desc2->sz );
   assert( desc1->len == desc2->len );

   /* Update gaussians if necessary. */
   dali_updateGauss( desc1, sigma );

   /* Set up offsets to prepare comparison. */
   ue  = desc1->ulen;
   ve  = desc1->vlen;
   ws  = MIN( desc1->wlen, desc2->wlen );

   /* Prepare rotation. */
   cR = cos( theta );
   sR = sin( theta );
   w2 = (double)ue/2.;
   h2 = (double)ve/2.;

   /* Calculate distance for the subset we've found. */
   gs = 0.;
   d  = 0.;
   for (u=0; u<ue; u++) {
      for (v=0; v<ve; v++) {
         /* Get first ID and check to see if it's in mask. */
         id1 = desc1->shape[ u*ve + v ];
         if (id1 < 0)
            continue;
         if (desc1->mask[ id1 ] == 0)
            continue;

         /* We'll rotate around for the second one. */
         ud   = (double)u;
         vd   = (double)v;
         urot = (int)round( (w2 + (ud - w2)*cR + (vd - h2)*sR)/scale );
         vrot = (int)round( (h2 - (ud - w2)*sR + (vd - h2)*cR)/scale );

         /* Make sure it's in the mask of the second one. */
         if ((urot < 0) || (urot >= desc2->ulen))
            continue;
         if ((vrot < 0) || (vrot >= desc2->vlen))
            continue;
         id2 = desc2->shape[ urot*ve + vrot ];
         if (id2 < 0)
            continue;
         if (desc2->mask[ id2 ] == 0)
            continue;

         /* Calculate Gaussian at the given position. */
         g   = desc1->sgauss[ id1 ];

         /* Increment Gaussian size. */
         gs += g;

         /* Inner loop in frequency domain has same Gaussian value. */
         dr = 0.;
         for (w=0; w<ws; w++) {
            d1  = desc1->desc[ id1*desc1->wlen + w ];
            d2  = desc2->desc[ id2*desc2->wlen + w ];
            dd  = d1-d2;
            dr += dd*dd;
         }
         /* We modulate the square by the Gaussian so it's a weighted sum. */
         d  += g * dr;
      }
   }
   if (gs == 0.)
      d = NAN;
   else
      d /= gs;

   return d;
}


double dali_distance_transform_lin( dali_t *desc1, dali_t *desc2,
      double sigma, double theta, double scale )
{
   int u,v,w;
   int ue,ve;
   double urot, vrot;
   double ua,va, u1a,v1a, uva,u1va,uv1a,u1v1a;
   int url,uru, vrl,vru;
   int ws;
   double ud, vd;
   double d1, d2, dd, d, dr;
   double g, gs;
   double cR, sR;
   double w2, h2;
   int id1, id2ll, id2lu, id2uu, id2ul;

   /* Set up sizes. */
   assert( desc1->sz == desc2->sz );
   assert( desc1->len == desc2->len );

   /* Set up offsets to prepare comparison. */
   ue  = desc1->ulen;
   ve  = desc1->vlen;
   ws  = MIN( desc1->wlen, desc2->wlen );

   /* Prepare rotation. */
   cR = cos( theta );
   sR = sin( theta );
   w2 = (double)ue/2.;
   h2 = (double)ve/2.;

   /* Update gaussians if necessary. */
   dali_updateGauss( desc1, sigma );

   /* Calculate distance for the subset we've found. */
   gs = 0.;
   d  = 0.;
   for (u=0; u<ue; u++) {
      for (v=0; v<ve; v++) {
         /* Get first ID and check to see if it's in mask. */
         id1 = desc1->shape[ u*ve + v ];
         if (id1 < 0)
            continue;
         if (desc1->mask[ id1 ] == 0)
            continue;

         /* We'll rotate around for the second one. */
         ud   = (double)u;
         vd   = (double)v;
         urot = (w2 + (ud - w2)*cR + (vd - h2)*sR) / scale;
         vrot = (h2 - (ud - w2)*sR + (vd - h2)*cR) / scale;
         url  = floor( urot );
         uru  = ceil(  urot );
         vrl  = floor( vrot );
         vru  = ceil(  vrot );

         /* Make sure it's in the maks of the second one. */
         if ((url < 0) || (uru >= desc2->ulen))
            continue;
         if ((vrl < 0) || (vru >= desc2->vlen))
            continue;
         id2ll = desc2->shape[ url*ve + vrl ];
         id2lu = desc2->shape[ url*ve + vru ];
         id2uu = desc2->shape[ uru*ve + vru ];
         id2ul = desc2->shape[ uru*ve + vrl ];
         if ((id2ll < 0) || (id2lu < 0) || (id2uu < 0) || (id2ul < 0))
            continue;
         if (desc2->mask[ id2ll ] == 0)
            continue;
         if (desc2->mask[ id2lu ] == 0)
            continue;
         if (desc2->mask[ id2uu ] == 0)
            continue;
         if (desc2->mask[ id2ul ] == 0)
            continue;

         /* Calculate Gaussian at the given position. */
         g   = desc1->sgauss[ id1 ];

         /* Increment Gaussian size. */
         gs += g;

         /* Some minor optimizations when calculating pixel weights. */
         ua    = urot - (double)url;
         va    = vrot - (double)vrl;
         u1a   = 1. - ua;
         v1a   = 1. - va;
         uva   = ua*va;
         u1va  = ua*v1a;
         uv1a  = u1a*va;
         u1v1a = u1a*v1a;

         /* Inner loop in frequency domain has same Gaussian value. */
         dr = 0.;
         for (w=0; w<ws; w++) {
            d1  = desc1->desc[ id1*desc1->wlen    + w ];
            /* We must weigh by the transformation. */
            d2  = uva   * desc2->desc[ id2ll*desc2->wlen + w ] +
                  u1va  * desc2->desc[ id2ul*desc2->wlen + w ] +
                  uv1a  * desc2->desc[ id2lu*desc2->wlen + w ] +
                  u1v1a * desc2->desc[ id2uu*desc2->wlen + w ];
            /* Calculate differences. */
            dd  = d1-d2;
            dr += dd*dd;
         }
         /* We modulate the square by the Gaussian so it's a weighted sum. */
         d  += g * dr;
      }
   }
   if (gs == 0.)
      d = NAN;
   else
      d /= gs;

   return d;
}


double dali_distance_pure( dali_t *desc1, dali_t *desc2 )
{
   int i,w;
   int ws;
   double d1, d2, dd, d, dr;

   /* Set up sizes. */
   assert( desc1->sz == desc2->sz );
   assert( desc1->len == desc2->len );

   /* Set up offsets to prepare comparison. */
   ws = MIN( desc1->wlen, desc2->wlen );

   /* Calculate distance for the subset we've found. */
   d  = 0.;
   for (i=0; i<desc1->len; i++) {
      /* Inner loop in frequency domain has same Gaussian value. */
      dr = 0.;
      for (w=0; w<ws; w++) {
         d1  = desc1->desc[ i*desc1->wlen + w ];
         d2  = desc2->desc[ i*desc2->wlen + w ];
         dd  = d1-d2;
         dr += dd*dd;
      }
      /* We modulate the square by the Gaussian so it's a weighted sum. */
      d += dr;
   }

   return d;
}

