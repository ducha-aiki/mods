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


#ifndef DALI_INTERNAL_H
#  define DALI_INTERNAL_H


#include <complex.h> /* Must be before fftw3.h */
#include <fftw3.h>
#include <suitesparse/cs.h>

#include "dali.h"


#define MIN(a,b)  (((a)<(b))?(a):(b))
#define MAX(a,b)  (((a)>(b))?(a):(b))

#define POW2(x)   ((x)*(x))
#define ABS(x)    (((x) >= 0) ? (x) : -(x))


/**
 * @brief Small image wrapper to make it more comfortable to manipulate.
 */
typedef struct dali_img_s {
   const double *data; /**< Image data itself. */
   int w;         /**< Image width. */
   int h;         /**< Image height. */
} dali_img_t;


/**
 * @brief Internal working structure to store all the temporary buffers and
 * information necessary to compute the descriptors.
 */
typedef struct dali_mesh_s {
   /* Timing information .*/
   double time_mesh;    /**< Meshing time. */
   double time_lb;      /**< LB time. */
   double time_eig;     /**< ceigs time. */
   double time_hks;     /**< hks time. */
   double time_hks_si;  /**< hks-si time. */

   /* Information and center. */
   int u;      /**< Point of interest U coordinate. */
   int v;      /**< Point of interest V coordinate. */
   int sz;     /**< Spread or half-width of patch. */

   /* Patch information (takes into account image limits). */
   int ui;     /**< Patch U start coordinate. */
   int ue;     /**< Patch V end coordinate. */
   int vi;     /**< Patch V start coordinate. */
   int ve;     /**< Patch V end coordinate. */

   /* Vertex information. */
   double *V;  /**< [ X, Y, I ] */
   double *oV; /**< [ X, Y ], used only with Gaussian version. */
   int n;      /**< Number of values in the mesh. */
   int nr;     /**< Non-artificially added components in the mesh. */

   /* Face information. */
   int *F;     /**< Faces are 3 ints per, indicating how to form the face. */
   int nF;     /**< Number of faces. */

   /* Temporary buffers associated with meshing. */
   int len;    /**< Real descriptor length. */
   int *Mt;    /**< Temporary mesh identifiers. */
   char *cMt;  /**< Internal mesh properties. */
   double *gauss; /**< Temporary gauss calculations. */

   /* Laplace-Beltrami information. */
   cs *L;      /**< Size of Laplace-Beltrami information. */
   double *eigd; /**< Eigenvalues of the Laplace-Beltrami operator. */
   double *eigv; /**< Eigenvectors of the Laplace-Beltrami operator. */

   /* Vertex area information. */
   double *ai; /**< Vector containing area of associated with each vertex. */
   cs *A;      /**< Matrix form of the ai vector. */

   /* HKS information. */
   double *hks_buf; /**< Temporary buffer for storing rows during HKS computation. */
   double *logt;  /**< Temporary buffer to contain logarithm of time slices. */
   double *t;     /**< Expanded time slice values. */
   double *expd;  /**< Temporary buffer to calculate HKS. */

   /* HKS + FFTW */
   double *HKS;   /**< Actual HKS information, already run through log(). */
   fftw_complex *HKS_fft; /**< HKS processed by FFT. */
   fftw_plan fftp; /**< The plan used by FFTW. */
} dali_mesh_t;


void dali_meshComputeCircleGaussianTree( dali_mesh_t *mesh, const dali_params_t *params );
/**
 * @brief Computes the mesh using a Gaussian distribution.
 */
void dali_meshComputeCircleGaussian( dali_t *desc, dali_mesh_t *mesh,
      const dali_img_t *im, const dali_params_t *params );


#endif /* DALI_INTERNAL_H */

