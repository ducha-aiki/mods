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

#include <ceigs.h>

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#ifdef DEBUG
#include <fenv.h>
#endif /* DEBUG */

/* To remove. */
#include <stdio.h>

#include "dali_internal.h"


/*
#define FOUT
*/


/**
 * @brief Does the dot product between two vectors.
 */
static double vec3_dot( const double u[3], const double v[3] )
{
   return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}
/**
 * @brief Does the cross product between two vectors.
 */
static void vec3_cross( double o[3], const double u[3], const double v[3] )
{
   double t[3];
   t[0] =  u[1]*v[2] - u[2]*v[1];
   t[1] = -u[0]*v[2] + u[2]*v[0];
   t[2] =  u[0]*v[1] - u[1]*v[0];
   memcpy( o, t, sizeof(double)*3 );
}
/**
 * @brief Subtracts a vector from another.
 */
static void vec3_sub( double o[3], const double u[3], const double v[3] )
{
   int i;
   for (i=0; i<3; i++)
      o[i] = u[i] - v[i];
}
/**
 * @brief Gets the norm of a vector.
 */
static double vec3_norm( const double v[3] )
{
   return sqrt( vec3_dot( v, v ) );
}
/**
 * @brief Normalizes a vector.
 */
static void vec3_normalize( double v[3] )
{
   double n = vec3_norm( v );
   v[0] /= n;
   v[1] /= n;
   v[2] /= n;
}


#if HAVE_PTHREADS
#include <pthread.h>
/**
 * @brief Provides data for threads.
 */
typedef struct dali_thread_s {
   /* Thread data. */
   pthread_t         thread;  /**< Thread id. */
   pthread_mutex_t*  lock;    /**< Lock protecting ->processed. */
   /* Index data. */
   int*              processed; /**< Current value being processed. */
   int               num;     /**< Total number to process. */
   /* Descriptor data. */
   dali_t*           desc;    /**< Descriptors to calculate. */
   dali_mesh_t       mesh;    /**< Mesh buffer for the thread. */
   const dali_img_t* img;     /**< Image to use to construct descriptors. */
   const int*        u;       /**< U coordinates of points of interest. */
   const int*        v;       /**< V coordinates of points of interest. */
   const dali_params_t* params; /**< Parameters to use for the descriptor. */
} dali_thread_t;
#endif /* HAVE_PTHREADS */


/*
 * Prototypes.
 */
/* High level. */
static void dali_compute_single( dali_t *desc, dali_mesh_t *mesh,
      const dali_img_t *img, int uc, int vc,
      const dali_params_t *params );
#if HAVE_PTHREADS
static void* dali_compute_thread( void *data );
#endif /* HAVE_PTHREADS */
/* Mesh. */
static int dali_meshInit( dali_mesh_t *mesh, const dali_params_t *params );
static void dali_meshCleanup( dali_mesh_t *mesh );
static void dali_meshComputeSquare( dali_t *desc, dali_mesh_t *mesh,
      const dali_img_t *im, const dali_params_t *params );
static void dali_meshComputeCircle( dali_t *desc, dali_mesh_t *mesh,
      const dali_img_t *im, const dali_params_t *params );
static void dali_meshComputeCircleVariable( dali_t *desc, dali_mesh_t *mesh,
      const dali_img_t *im, const dali_params_t *params );
static void dali_meshCompute( dali_t *desc,  dali_mesh_t *mesh,
      const dali_img_t *im, const dali_params_t *params );
static void dali_meshComputeAreas( dali_mesh_t *mesh );
/* Laplace-Betrami. */
static int dali_computeLaplaceBeltrami( dali_mesh_t *mesh );
/* Eigenvector voodoo. */
static int dali_computeEigs( dali_mesh_t *mesh, const dali_params_t *params );
/* Calculate the Heat Kernel Signature. */
static int dali_computeHKS( dali_mesh_t *mesh, const dali_params_t *params );
static int dali_computeHKS_SI( dali_t *desc, dali_mesh_t *mesh, const dali_params_t *params );


void dali_optsDefault( dali_params_t *params )
{
   memset( params, 0, sizeof(dali_params_t) );
   /* Descriptor. */
   params->mtype  = DALI_MESH_TYPE_CIRCLE_VARIABLE;
   params->Sz     = 30;
   params->Sz_coarse = 10;
   params->beta   = 2000.;
   params->wmax   = 20;
   params->ncomp  = 200;
   params->ntime  = 100;
   /* Meshing. */
   params->mesh_K       = 2500.;
   params->mesh_sigma   = 0.2;
   /* Implementation. */
   params->threads = 1;
   params->lanczos = 0; /* Use ceigs default. */
   params->eigs_iter = 300;
   params->use_si  = 1;
   params->verbose = 0;
}


/**
 * @brief Extracts the execution information from the mesh.
 */
static void dali_compute_info( dali_info_t *info, const dali_mesh_t *mesh, int n )
{
   double num = n;

   /* Timing information. */
   info->time_meshing   = mesh->time_mesh / num;
   info->time_laplacebeltrami = mesh->time_lb / num;
   info->time_eigenvectors = mesh->time_eig / num;
   info->time_hks       = mesh->time_hks / num;
   info->time_hks_si    = mesh->time_hks_si / num;
   info->time_elapsed   = info->time_meshing +
                          info->time_laplacebeltrami +
                          info->time_eigenvectors +
                          info->time_hks +
                          info->time_hks_si;

   /* Graph information. */
   info->nodes_real  = mesh->nr;
   info->nodes_total = mesh->n;
   info->faces       = mesh->nF;
}


static double dali_timediff( clock_t *t1, clock_t *t2 )
{
   return ((double)((*t2)-(*t1))) / CLOCKS_PER_SEC;
}


/**
 * @brief Computes a single descriptor.
 *
 * This writes to desc and reads/writes from mesh.
 *
 *    @param[out] desc Descriptor to write to.
 *    @param[in,out] mesh Mesh to use to perform the calculations.
 *    @param[in] uc U coordinate of point to calculate descriptor of.
 *    @param[in] uv V coordinate of point to calculate the descriptor of.
 *    @param[in] params Parameters to use when calculating the descriptor.
 */
static void dali_compute_single( dali_t *desc, dali_mesh_t *mesh,
      const dali_img_t *img, int uc, int vc,
      const dali_params_t *params )
{
   int i, j;
   clock_t t1, t2;

   /* Comfort variables. */
   mesh->u   = uc;
   mesh->v   = vc;
   mesh->sz  = params->Sz;
   if (params->verbose)
      printf("Processing point %d x %d...\n", uc, vc );

   /* Get square patch and assosciated triangle mesh. */
   if (params->verbose)
      printf("   Computing mesh...\n");
   t1 = clock();
   dali_meshCompute( desc, mesh, img, params );
   t2 = clock();
   mesh->time_mesh += dali_timediff( &t1, &t2 );

   /* Laplace-Beltrani calculations. */
   if (params->verbose)
      printf("   Computing Laplace-Beltrami...\n");
   dali_computeLaplaceBeltrami( mesh );
   t1 = clock();
   mesh->time_lb += dali_timediff( &t2, &t1 );

   /* Obtain a subset of eigenvectors and eigenvalues of the Laplace-Beltrani map. */
   if (params->verbose)
      printf("   Computing EigenVectors...\n");
   dali_computeEigs( mesh, params );
   t2 = clock();
   mesh->time_eig += dali_timediff( &t1, &t2 );

   /* Create actual descriptor. */
   if (params->verbose)
      printf("   Computing HKS...\n");
   dali_computeHKS( mesh, params );
   t1 = clock();
   mesh->time_hks += dali_timediff( &t2, &t1 );

   /* Make descriptor scale invariant. */
   if (params->use_si) {
      if (params->verbose)
         printf("   Computing HKS-SI...\n");
      dali_computeHKS_SI( desc, mesh, params );
      t2 = clock();
      mesh->time_hks_si += dali_timediff( &t1, &t2 );
   }
   else {
      if (params->verbose)
         printf("   Skipping HKS-SI...\n");
      if (params->wmax != params->ntime)
         fprintf( stderr, "Warning! When using HKS instead of HKS-SI 'ntime' (%d) parameter should be set to the same value as 'wmax' (%d)!\n", params->ntime, params->wmax );
      desc->desc = malloc( mesh->nr*params->wmax*sizeof(double) );
      for (i=0; i<mesh->nr; i++)
         for (j=0; j<params->wmax; j++)
            desc->desc[ i*params->wmax + j ] = mesh->HKS[ i*params->ntime + j ];
      t2 = clock();
      mesh->time_hks_si += dali_timediff( &t1, &t2 );
   }

   /* Done. */
   if (params->verbose)
      printf("   Done!\n");
}


#if HAVE_PTHREADS
static void* dali_compute_thread( void *data )
{
   int i;
   dali_thread_t *thdata;

   /* Get data. */
   thdata = (dali_thread_t*) data;

   while (1) {
      /* Here we get the one to process. */
      pthread_mutex_lock( thdata->lock );
      if (*thdata->processed >= thdata->num)
         break;
      i = *thdata->processed;
      (thdata->processed)++;
      pthread_mutex_unlock( thdata->lock );

      /* Actually process now. */
      dali_compute_single( &thdata->desc[i], &thdata->mesh,
            thdata->img, thdata->u[i], thdata->v[i], thdata->params );
   }

   /* Clean up and die. */
   pthread_mutex_unlock( thdata->lock );
   pthread_exit( NULL );
}
#endif /* HAVE_PTHREADS */


dali_t* dali_compute( const double *im, int w, int h,
      const int *u, const int *v, int n,
      const dali_params_t *params, dali_info_t *info )
{
   int i, th;
   dali_img_t img;
   dali_mesh_t mesh;
   dali_t *desc;
   const dali_params_t *params_use;
   dali_params_t params_default;

#ifdef DEBUG
   fenv_t fpenv;
   fegetenv( &fpenv );
   feenableexcept( FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW );
#endif /* DEBUG */

   /* Set up. */
   img.data = im;
   img.w    = w;
   img.h    = h;

   /* Choose what parameters to use. */
   if (params == NULL) {
      dali_optsDefault( &params_default );
      params_use = &params_default;
   }
   else
      params_use = params;
   if (params_use->verbose > 1) {
      printf( "mesh_type: %d\n", params_use->mtype );
      printf( "Sz: %d\n", params_use->Sz );
      printf( "Sz_coarse: %d\n", params_use->Sz_coarse );
      printf( "beta: %f\n", params_use->beta );
      printf( "ncomp: %d\n", params_use->ncomp );
      printf( "ntime: %d\n", params_use->ntime );
      printf( "wmax: %d\n", params_use->wmax );
      printf( "mesh_K: %f\n", params_use->mesh_K );
      printf( "mesh_sigma: %f\n", params_use->mesh_sigma );
      printf( "threads: %d\n", params_use->threads );
      printf( "lanczos: %d\n", params_use->lanczos );
      printf( "eigs_iter: %d\n", params_use->eigs_iter );
      printf( "use_si: %d\n", params_use->use_si );
      printf( "verbose: %d\n", params_use->verbose );
   }

   /* Memory allocation. */
   desc     = calloc( n, sizeof(dali_t) );
   assert( desc != NULL );
   desc->n  = n; /* Only first descriptor has n set. */

   /* Non-threaded approach. */
   th = params_use->threads;
#if HAVE_PTHREADS
   if (th <= 1) {
#else /* HAVE_PTHREADS */
      if (th > 1)
         fprintf( stderr, "Warning: DaLI compiled without pthread support, unable to thread.\n" );
#endif /* HAVE_PTHREADS */
      dali_meshInit( &mesh, params_use );
      for (i=0; i<n; i++)
         dali_compute_single( &desc[i], &mesh, &img, u[i], v[i], params );
      /* Copy stuff over. */
      desc->len   = mesh.len;
      desc->gauss = mesh.gauss;
      desc->shape = mesh.Mt;
      if (info != NULL)
         dali_compute_info( info, &mesh, n );
      dali_meshCleanup( &mesh );
#if HAVE_PTHREADS
   }
   else {
      int process;
      dali_thread_t *thdata, *thd;
      pthread_mutex_t lock;
      thdata = calloc( th, sizeof(dali_thread_t) );

      /* Must initialize the meshes, they can not be threaded due to FFTW3. */
      for (i=0; i<th; i++)
         dali_meshInit( &thdata[i].mesh, params_use );

      /* Launch the threads. */
      process = 0;
      pthread_mutex_init( &lock, NULL );
      for (i=0; i<th; i++) {
         thd            = &thdata[i];
         thd->lock      = &lock;
         thd->processed = &process;
         thd->num       = n;
         thd->desc      = desc;
         thd->img       = &img;
         thd->u         = u;
         thd->v         = v;
         thd->params    = params_use;
         pthread_create( &thd->thread, NULL, dali_compute_thread, (void*) thd );
      }

      /* Wait for the threads. */
      for (i=0; i<th; i++)
         pthread_join( thdata[i].thread, NULL );
      pthread_mutex_destroy( &lock );

      /* Copy stuff over. */
      desc->len   = thdata[0].mesh.len;
      desc->gauss = thdata[0].mesh.gauss;
      desc->shape = thdata[0].mesh.Mt;

      /* Clean up. */
      for (i=0; i<th; i++)
         dali_meshCleanup( &thdata[i].mesh );
      free( thdata );
   }
#endif /* HAVE_PTHREADS */

   desc->sigma  = malloc( sizeof(double) );
   *desc->sigma = INFINITY;
   desc->sgauss = malloc( desc->len * sizeof(double) );
   for (i=1; i<n; i++) {
      desc[i].len    = desc[0].len;
      desc[i].sigma  = desc[0].sigma;
      desc[i].gauss  = desc[0].gauss;
      desc[i].shape  = desc[0].shape;
      desc[i].sgauss = desc[0].sgauss;
   }

#ifdef DEBUG
   fesetenv( &fpenv );
#endif /* DEBUG */

   return desc;
}


void dali_free( dali_t *dali )
{
   int i;

   /* Free per-descriptor information. */
   for (i=0; i<dali->n; i++) {
      free( dali[i].desc );
      free( dali[i].mask );
   }

   /* Free global information. */
   free( dali->shape );
   free( dali->gauss );
   free( dali->sgauss );
   free( dali->sigma );

   /* Free descriptors. */
   free( dali );
}


static int dali_meshInit( dali_mesh_t *mesh, const dali_params_t *params )
{
   int i, nr, nc, tmp, howmany;
   int sz, ncomp, ntime;

   /* Comfort. */
   sz    = params->Sz;
   ncomp = params->ncomp;
   ntime = params->ntime;

   /* Just in case. */
   memset( mesh, 0, sizeof(dali_mesh_t) );

   /* Max limits. */
   nr = 2*sz+1;
   nc = 2*sz+1;

   /* Lengths. */
   switch (params->mtype) {
      case DALI_MESH_TYPE_CIRCLE_VARIABLE:
         mesh->cMt  = malloc( nr*nc * sizeof(char) );
         assert( mesh->cMt  != NULL );
      case DALI_MESH_TYPE_SQUARE_DENSE:
      case DALI_MESH_TYPE_CIRCLE_DENSE:
         mesh->n  = nr*nc + (nr-1)*(nc-1);
         mesh->nF = 4*(nr-1)*(nc-1);
         mesh->V  = malloc( mesh->n * 3*sizeof(double) );
         mesh->F  = malloc( mesh->nF * 3*sizeof(int) ); /* 3 ids per face. */
         howmany  = nr*nc;
         break;
      case DALI_MESH_TYPE_CIRCLE_GAUSS:
         dali_meshComputeCircleGaussianTree( mesh, params );
         howmany  = mesh->n;
         break;
   }

   /* Vertex and face information buffers. */
   mesh->ai   = malloc( mesh->n * sizeof(double) );

   /* Laplace-Beltrami temporary buffers. */
   mesh->A    = cs_spalloc( mesh->n, mesh->n, mesh->n,   1, 1 );
   mesh->L    = cs_spalloc( mesh->n, mesh->n, 3*mesh->n, 1, 1 );
   mesh->eigv = malloc( ncomp*mesh->n * sizeof(double) );
   mesh->eigd = malloc( ncomp * sizeof(double) );

   /* Heat Kernel Signature temporary buffers. */
   mesh->hks_buf = malloc( (ncomp-1) * sizeof(double) );
   mesh->logt    = malloc( ntime * sizeof(double) );
   mesh->t       = malloc( ntime * sizeof(double) );
   for (i=0; i<ntime; i++) {
      mesh->logt[i]  = -10. + (30.) * ((double)i) / ((double)(ntime-1));
      mesh->t[i]     = pow( 2., mesh->logt[i] );
   }
   mesh->expd    = malloc( ntime*(ncomp-1) * sizeof(double) );

   /* For Heat Kernel Signature and it's fast-fourier transform. */
   mesh->HKS     = fftw_malloc( sizeof(double) * howmany * ntime );
   mesh->HKS_fft = fftw_malloc( sizeof(fftw_complex) * howmany * ntime ); /* used to be (ntime-1). */
                   /* Not sure what the issue is, but was getting OOB reads with the
                    * Gaussian mesh so I increased the usage by a bit. */
   tmp           = ntime-1;
   mesh->fftp    = fftw_plan_many_dft_r2c(
         1, &tmp, /* Rank and n. */
         howmany, /* howmany. */
         mesh->HKS,     0, 1, ntime, /* in, inembed, istride, idist */
         mesh->HKS_fft, 0, 1, ntime-1, /* out, onembed, ostride, odist */
         FFTW_ESTIMATE ); /* flags */

   /* Memory checks. */
   assert( mesh->V    != NULL );
   assert( mesh->F    != NULL );
   assert( mesh->ai   != NULL );
   assert( mesh->A    != NULL );
   assert( mesh->L    != NULL );
   assert( mesh->eigv != NULL );
   assert( mesh->eigd != NULL );
   assert( mesh->hks_buf != NULL );
   assert( mesh->logt != NULL );
   assert( mesh->t    != NULL );
   assert( mesh->expd != NULL );
   assert( mesh->HKS  != NULL );
   assert( mesh->HKS_fft != NULL );
   assert( mesh->fftp != NULL );

   return 0;
}


static void dali_meshCleanup( dali_mesh_t *mesh )
{
   free( mesh->V );
   free( mesh->oV );
   free( mesh->F );
   free( mesh->cMt );

   free( mesh->ai );
   cs_spfree( mesh->A );
   cs_spfree( mesh->L );
   free( mesh->eigv );
   free( mesh->eigd );

   free( mesh->hks_buf );
   free( mesh->logt );
   free( mesh->t );
   free( mesh->expd );

   fftw_free( mesh->HKS );
   fftw_free( mesh->HKS_fft );
   fftw_destroy_plan( mesh->fftp );
   fftw_cleanup();
}


#define SETF( F, id, a, b, c )   \
(F)[ (id)*3+0  ] = a; \
(F)[ (id)*3+1  ] = b; \
(F)[ (id)*3+2  ] = c
#define MEAN4( v, i1, i2, i3, i4 ) \
((1./4.) * ((v)[i1] + (v)[i2] + (v)[i3] + (v)[i4]))
#define MEAN3( v, i1, i2, i3 ) \
((1./3.) * ((v)[i1] + (v)[i2] + (v)[i3] ))
/**
 * @brief Creates the actual faces and adds virtual mesh points if necessary.
 */
static void dali_meshComputeSquare( dali_t *desc, dali_mesh_t *mesh,
                              const dali_img_t *im, const dali_params_t *params )
{
   int i, j, nF;
   int ii, jj;
   int u, v;
   int w, sz, len;
   int id, id1, id2, id3, id4, ide;
   int us,vs, ui,vi, ue,ve;
   int uoff,voff, ueff,veff;

   /* Decriptor length. */
   u        = mesh->u;
   v        = mesh->v;
   sz       = params->Sz;
   len      = 2*sz+1;

   /* Calculate patch. */
   ui = MAX( 0,       u-sz   );
   vi = MAX( 0,       v-sz   );
   ue = MIN( im->w, u+sz+1 );
   ve = MIN( im->h, v+sz+1 );
   mesh->ui = ui;
   mesh->ue = ue;
   mesh->vi = vi;
   mesh->ve = ve;

   /* Comfort. */
   w  = im->w;
   us = ue-ui;
   vs = ve-vi;

   /* Descriptor output information. */
   uoff = ui - (u-sz);
   voff = vi - (v-sz);
   ueff = us;
   veff = vs;
   desc->ulen = len;
   desc->vlen = len;
   desc->wlen = params->wmax;
   desc->sz   = sz;

   /* Allocate shape mesh size. */
   if (mesh->Mt == NULL) {
      mesh->Mt    = malloc( len*len * sizeof(int) );
      mesh->gauss = malloc( len*len * sizeof(double) );
      assert( mesh->Mt != NULL );
      assert( mesh->gauss != NULL );

      id = 0;
      for (i=0; i<len; i++) {
         for (j=0; j<len; j++) {
            /* Set up shape and gaussian. */
            mesh->Mt[id]      = id;
            mesh->gauss[ id ] = -(double)(POW2(i-sz) + POW2(j-sz));
            id++;
         }
      }
      mesh->len = len*len;
   }

   /* Create patch mask. */
   desc->mask = calloc( len*len, sizeof(char) );
   for (i=uoff; i<uoff+ueff; i++)
      for (j=voff; j<voff+veff; j++)
         desc->mask[ i*desc->vlen + j ] = 1;

   /* First load up the original mesh. */
   id = 0;
   for (i=0; i<len; i++) {
      for (j=0; j<len; j++) {
         /* Set up mesh. */
         mesh->V[ 3*id+0 ] = j;
         mesh->V[ 3*id+1 ] = i;
         if (desc->mask[ id ] != 0)
            mesh->V[ 3*id+2 ] = im->data[ w*(vi+j-voff) + (ui+i-uoff) ];
         else {
            /* Out of bounds, so here what we do is mirror the data. This is to create
             * a similar heat flow to in a sense compensate the missing values. */
#if 0
            mesh->V[ 3*id+2 ] = 0.;
#else
            ii = i;
            jj = j;
            if (ii < uoff)
               ii = 2*uoff - ii;
            else if (ii >= uoff + ueff)
               ii = 2*(uoff+ueff) - ii - 1;
            if (jj < voff)
               jj = 2*voff - jj;
            else if (jj >= voff + veff)
               jj = 2*(voff+veff) - jj - 1;
            mesh->V[ 3*id+2 ] = im->data[ w*(vi+jj-voff) + (ui+ii-uoff) ];
#endif
         }
         id++;
      }
   }
   mesh->nr  = len*len; /* Store number of real vertex components. */

#ifdef FOUT
   /* Print patch. */
   FILE *fpatch = fopen( "data_patch.dat", "w" );
   for (j=0; j<len; j++) {
      for (i=0; i<len; i++)
         fprintf( fpatch, "%f ", mesh->V[ 3*(i*len+j)+2 ] );
      fprintf( fpatch, "\n" );
   }
   fclose( fpatch );
#endif

   /* Now we actually create the mesh and set up the faces and extra vertices. */
   nF  = 0;
   ide = mesh->nr;
   for (i=0; i<len-1; i++) {
      for (j=0; j<len-1; j++) {
         /* Calculate indices. */
         id1 = i*len     + j;
         id2 = i*len     + j+1;
         id3 = (i+1)*len + j+1;
         id4 = (i+1)*len + j;

         /* Average the meshes for the extra point. */
         mesh->V[ 3*ide+0 ] = MEAN4( mesh->V, 3*id1+0, 3*id2+0, 3*id3+0, 3*id4+0 );
         mesh->V[ 3*ide+1 ] = MEAN4( mesh->V, 3*id1+1, 3*id2+1, 3*id3+1, 3*id4+1 );
         mesh->V[ 3*ide+2 ] = MEAN4( mesh->V, 3*id1+2, 3*id2+2, 3*id3+2, 3*id4+2 );

         /* Set up the faces. */
         SETF( mesh->F, 4*nF+0, id1, id2, ide );
         SETF( mesh->F, 4*nF+1, id2, id3, ide );
         SETF( mesh->F, 4*nF+2, id3, id4, ide );
         SETF( mesh->F, 4*nF+3, id1, id4, ide );

         /* Increment counters. */
         nF++;
         ide++;
      }
   }

   /* We must set new values, as we are setting a subset sometimes. */
   mesh->n  = ide;
   mesh->nF = 4*nF;
}


/**
 * @brief Creates the actual faces and adds virtual mesh points if necessary.
 */
static void dali_meshComputeCircle( dali_t *desc, dali_mesh_t *mesh,
                              const dali_img_t *im, const dali_params_t *params )
{
   int i, j, nF;
   int ii, jj;
   int u, v;
   int w, sz, len;
   int id, id0, id1, id2, id3, id4, ide;
   int us,vs, ui,vi, ue,ve;
   int uoff,voff, ueff,veff;
   double r2;

   /* Decriptor length. */
   u        = mesh->u;
   v        = mesh->v;
   sz       = params->Sz;
   len      = 2*sz+1;

   /* Calculate patch. */
   ui = MAX( 0,       u-sz   );
   vi = MAX( 0,       v-sz   );
   ue = MIN( im->w, u+sz+1 );
   ve = MIN( im->h, v+sz+1 );
   mesh->ui = ui;
   mesh->ue = ue;
   mesh->vi = vi;
   mesh->ve = ve;

   /* Comfort. */
   w  = im->w;
   us = ue-ui;
   vs = ve-vi;

   /* Descriptor output information. */
   uoff = ui - (u-sz);
   voff = vi - (v-sz);
   ueff = us;
   veff = vs;
   desc->wlen = params->wmax;
   desc->sz   = sz;

   /* We store additional stuff in the first descriptor. */
   if (mesh->Mt == NULL) {
      /* Allocate shape mesh size. */
      mesh->Mt    = malloc( len*len * sizeof(int) );
      mesh->gauss = malloc( len*len * sizeof(double) );
      assert( mesh->Mt != NULL );
      assert( mesh->gauss != NULL );
      /* Calculate size and form mesh. */
      id = 0;
      r2 = (double)(sz*sz);
      for (i=0; i<len; i++) {
         for (j=0; j<len; j++) {
            /* Must be inside circle. */
            if (POW2(((double)ABS(i-sz))-0.5) + POW2(((double)ABS(j-sz))-0.5) > r2) {
               mesh->Mt[ len*i + j ] = -1;
               continue;
            }
            mesh->Mt[ len*i + j ] = id;
            mesh->gauss[ id ]     = -(double)(POW2(i-sz) + POW2(j-sz));
            id++;
         }
      }
      mesh->len = id;

      /* Now with the exact size we can allocate various things. */
      desc->mask  = calloc( mesh->len, sizeof(int) );
   }
   else {
      desc->mask  = calloc( mesh->len, sizeof(int) );
   }

   /* First load up the original mesh. */
   for (i=0; i<len; i++) {
      for (j=0; j<len; j++) {

         /* Must be inside circle. */
         id = mesh->Mt[ len*i + j ];
         if (id < 0)
            continue;

         /* Set mesh ids. */
         mesh->V[ 3*id+0 ] = j;
         mesh->V[ 3*id+1 ] = i;

         /* Add mask if in bounds. */
         if ((i >= uoff) && (i < uoff+ueff) &&
               (j >= voff) && (j < voff+veff)) {
            desc->mask[ id ]  = 1;
            mesh->V[ 3*id+2 ] = im->data[ w*(vi+j-voff) + (ui+i-uoff) ];
         }
         else {
            /* Out of bounds, so here what we do is mirror the data. This is to create
             * a similar heat flow to in a sense compensate the missing values. */
            ii = i;
            jj = j;
            if (ii < uoff)
               ii = 2*uoff - ii;
            else if (ii >= uoff + ueff)
               ii = 2*(uoff+ueff) - ii - 1;
            if (jj < voff)
               jj = 2*voff - jj;
            else if (jj >= voff + veff)
               jj = 2*(voff+veff) - jj - 1;
            mesh->V[ 3*id+2 ] = im->data[ w*(vi+jj-voff) + (ui+ii-uoff) ];
         }
      }
   }
   mesh->nr = mesh->len; /* Store number of real vertex components. */

   /* Not realistic size. */
   desc->ulen = len;
   desc->vlen = len;

   /* Now we actually create the mesh and set up the faces and extra vertices. */
   nF  = 0;
   ide = mesh->nr;
   for (i=0; i<len-1; i++) {
      for (j=0; j<len-1; j++) {

         /* Only interested in points of the mesh that matter. */
         id0 = mesh->Mt[ len*i + j ];
         if (id0 < 0)
            continue;

         /* Check top-right. */
         if (i > 0) {
            /* Must be nothing ontop and two nodes on right and top-right. */
            id1 = mesh->Mt[ len*(i-1) + j   ];
            id2 = mesh->Mt[ len*(i-1) + j+1 ];
            id3 = mesh->Mt[ len*i     + j+1 ];
            if ((id1 < 0) && (id2 >= 0) && (id3 >= 0)) {
               id4 = 3*ide;
               /* Add virtual point. */
               mesh->V[ id4+0 ] = mesh->V[ 3*id0+0 ] + 0.5;
               mesh->V[ id4+1 ] = mesh->V[ 3*id0+1 ] - 0.5;
               mesh->V[ id4+2 ] = MEAN3( mesh->V, 3*id0+2, 3*id2+2, 3*id3+2 );
               /* Add the two faces. */
               SETF( mesh->F, nF+0, id0, ide, id3 );
               SETF( mesh->F, nF+1, id2, ide, id3 );
               nF += 2;
               ide++;
            }
         }

         /* Check bottom-right. This is the main sector of interest. */
         id1 = mesh->Mt[ len*(i+1) + j+1 ];
         if (id1 >= 0) {
            id2 = mesh->Mt[ len*i     + j+1 ];
            id3 = mesh->Mt[ len*(i+1) + j   ];
            id4 = 3*ide;
            /* Add virtual point. */
            mesh->V[ id4+0 ] = mesh->V[ 3*id0+0 ] + 0.5;
            mesh->V[ id4+1 ] = mesh->V[ 3*id0+1 ] + 0.5;
            if ((id2 >= 0) && (id3 >= 0))
               mesh->V[ id4+2 ] = MEAN4( mesh->V, 3*id0+2, 3*id1+2, 3*id2+2, 3*id3+2 );
            else if (id2 >= 0)
               mesh->V[ id4+2 ] = MEAN3( mesh->V, 3*id0+2, 3*id1+2, 3*id2+2 );
            else if (id3 >= 0)
               mesh->V[ id4+2 ] = MEAN3( mesh->V, 3*id0+2, 3*id1+2, 3*id3+2 );
            /* Add right faces. */
            if (id2 >= 0) {
               SETF( mesh->F, nF+0, id0, ide, id2 );
               SETF( mesh->F, nF+1, id1, ide, id2 );
               nF += 2;
            }
            /* Add bottom faces. */
            if (id3 >= 0) {
               SETF( mesh->F, nF+0, id0, ide, id3 );
               SETF( mesh->F, nF+1, id1, ide, id3 );
               nF += 2;
            }
            ide++;
         }
         /* Corner case #1 => Nothing on bottom-right, while elements on the right and bottom. */
         else {
            id2 = mesh->Mt[ len*i     + j+1 ];
            id3 = mesh->Mt[ len*(i+1) + j   ];
            if ((id2 >= 0) && (id3 >= 0)) {
               id4 = 3*ide;
               /* Add virtual point. */
               mesh->V[ id4+0 ] = mesh->V[ 3*id0+0 ] + 0.5;
               mesh->V[ id4+1 ] = mesh->V[ 3*id0+1 ] + 0.5;
               mesh->V[ id4+2 ] = MEAN3( mesh->V, 3*id0+2, 3*id2+2, 3*id3+2 );
               /* Add faces. */
               SETF( mesh->F, nF+0, id0, ide, id2 );
               SETF( mesh->F, nF+1, id0, ide, id3 );
               nF += 2;
               ide++;
            }
         }
      }
   }

   /* We must set new values, as we are setting a subset sometimes. */
   mesh->n  = ide;
   mesh->nF = nF;
}


/**
 * @brief Creates the actual faces and adds virtual mesh points if necessary.
 */
static void dali_meshComputeCircleVariable( dali_t *desc, dali_mesh_t *mesh,
                              const dali_img_t *im, const dali_params_t *params )
{
   int i, j, nF;
   int ii, jj;
   int u, v;
   int w, sz, csz, len;
   int id, id0, id1, id2, id3, id4, ide;
   int us,vs, ui,vi, ue,ve;
   int uoff,voff, ueff,veff;
   double dist, r2, cr2;

   /* Decriptor length. */
   u        = mesh->u;
   v        = mesh->v;
   sz       = params->Sz;
   csz      = params->Sz_coarse;
   len      = 2*sz+1;

   /* Calculate patch. */
   ui = MAX( 0,       u-sz   );
   vi = MAX( 0,       v-sz   );
   ue = MIN( im->w, u+sz+1 );
   ve = MIN( im->h, v+sz+1 );
   mesh->ui = ui;
   mesh->ue = ue;
   mesh->vi = vi;
   mesh->ve = ve;

   /* Comfort. */
   w  = im->w;
   us = ue-ui;
   vs = ve-vi;

   /* Descriptor output information. */
   desc->wlen = params->wmax;
   desc->sz   = sz;

   /* We store additional stuff in the first descriptor. */
   if (mesh->Mt == NULL) {
      /* Allocate shape mesh size. */
      mesh->Mt    = malloc( len*len * sizeof(int) );
      mesh->gauss = malloc( len*len * sizeof(double) );
      assert( mesh->Mt != NULL );
      assert( mesh->gauss != NULL );
      /* Calculate size and form mesh. */
      id  = 0;
      r2  = (double)(sz*sz);
      cr2 = (double)(csz*csz);
      for (i=0; i<len; i++) {
         for (j=0; j<len; j++) {
            dist = POW2(((double)ABS(i-sz))-0.5) + POW2(((double)ABS(j-sz))-0.5);
            /* Must be inside circle. */
            if (dist > r2) {
               mesh->Mt[ len*i + j ]  = -1;
               mesh->cMt[ len*i + j ] = 0; /* 0 means it's outside. */
               continue;
            }
            /* Check inner circle. */
            if (dist > cr2)
               mesh->cMt[ len*i + j ] = 2; /* 1 means it's the outter circle. */
            else
               mesh->cMt[ len*i + j ] = 1; /* 2 means it's the inner circle. */
            /* Normal mesh. */
            mesh->Mt[ len*i + j ] = id;
            mesh->gauss[ id ]     = -(double)(POW2(i-sz) + POW2(j-sz));
            id++;
         }
      }
      mesh->len = id;

      /* Now with the exact size we can allocate various things. */
      desc->mask  = calloc( mesh->len, sizeof(int) );
   }
   else {
      desc->mask  = calloc( mesh->len, sizeof(int) );
   }

   /* Limits to create the mask. */
   uoff = ui - (u-sz);
   voff = vi - (v-sz);
   ueff = us;
   veff = vs;

   /* First load up the original mesh. */
   for (i=0; i<len; i++) {
      for (j=0; j<len; j++) {

         /* Must be inside circle. */
         id = mesh->Mt[ len*i + j ];
         if (id < 0)
            continue;

         /* Set mesh ids. */
         mesh->V[ 3*id+0 ] = j;
         mesh->V[ 3*id+1 ] = i;

         /* Add mask if in bounds. */
         if ((i >= uoff) && (i < uoff+ueff) &&
               (j >= voff) && (j < voff+veff)) {
            desc->mask[ id ]  = 1;
            mesh->V[ 3*id+2 ] = im->data[ w*(vi+j-voff) + (ui+i-uoff) ];
         }
         else {
            /* Out of bounds, so here what we do is mirror the data. This is to create
             * a similar heat flow to in a sense compensate the missing values. */
            ii = i;
            jj = j;
            if (ii < uoff)
               ii = 2*uoff - ii;
            else if (ii >= uoff + ueff)
               ii = 2*(uoff+ueff) - ii - 1;
            if (jj < voff)
               jj = 2*voff - jj;
            else if (jj >= voff + veff)
               jj = 2*(voff+veff) - jj - 1;
            mesh->V[ 3*id+2 ] = im->data[ w*(vi+jj-voff) + (ui+ii-uoff) ];
         }
      }
   }
   mesh->nr = mesh->len; /* Store number of real vertex components. */

   /* Not realistic size. */
   desc->ulen = len;
   desc->vlen = len;

   /* Now we actually create the mesh and set up the faces and extra vertices. */
   nF  = 0;
   ide = mesh->nr;
   for (i=0; i<len-1; i++) {
      for (j=0; j<len-1; j++) {
         int c1, c2, c3, c4;

         /* Only interested in points of the mesh that matter. */
         id0 = mesh->Mt[ len*i + j ];
         if (id0 < 0)
            continue;

         /* Check values. */
         c1 = mesh->cMt[ len*i     + j     ];
         c2 = mesh->cMt[ len*(i+1) + j     ];
         c3 = mesh->cMt[ len*(i+1) + (j+1) ];
         c4 = mesh->cMt[ len*i     + (j+1) ];

         /* All in inner circle, we can do virtual point meshing. */
         if ((c1==1) && (c2==1) && (c3==1) && (c4==1)) {
            id1 = mesh->Mt[ len*(i+1) + j+1 ];
            id2 = mesh->Mt[ len*i     + j+1 ];
            id3 = mesh->Mt[ len*(i+1) + j   ];
            id4 = 3*ide;
            /* Add virtual point. */
            mesh->V[ id4+0 ] = mesh->V[ 3*id0+0 ] + 0.5;
            mesh->V[ id4+1 ] = mesh->V[ 3*id0+1 ] + 0.5;
            mesh->V[ id4+2 ] = MEAN4( mesh->V, 3*id0+2, 3*id1+2, 3*id2+2, 3*id3+2 );
            /* Add right faces. */
            SETF( mesh->F, nF+0, id0, ide, id2 );
            SETF( mesh->F, nF+1, id1, ide, id2 );
            /* Add bottom faces. */
            SETF( mesh->F, nF+2, id0, ide, id3 );
            SETF( mesh->F, nF+3, id1, ide, id3 );
            nF += 4;
            ide++;
            continue;
         }

         /* Here we mesh normally course without additional nodes. */
         /* Check top-right. */
         if (i > 0) {
            /* Must be nothing ontop and two nodes on right and top-right. */
            id1 = mesh->Mt[ len*(i-1) + j   ];
            id2 = mesh->Mt[ len*(i-1) + j+1 ];
            id3 = mesh->Mt[ len*i     + j+1 ];
            if ((id1 < 0) && (id2 >= 0) && (id3 >= 0)) {
               /* Add the two faces. */
               SETF( mesh->F, nF+0, id0, id3, id2 );
               nF += 1;
            }
         }

         /* Check bottom-right. This is the main sector of interest. */
         id1 = mesh->Mt[ len*(i+1) + j+1 ];
         if (id1 >= 0) {
            double ioff, joff;
            ioff = i-sz+0.5;
            joff = j-sz+0.5;
            id2  = mesh->Mt[ len*i     + j+1 ];
            id3  = mesh->Mt[ len*(i+1) + j   ];
            if (ioff*joff < 0.) {
               /* Add right faces. */
               if (id2 >= 0) {
                  SETF( mesh->F, nF+0, id0, id1, id2 );
                  nF += 1;
               }
               /* Add bottom faces. */
               if (id3 >= 0) {
                  SETF( mesh->F, nF+0, id0, id1, id3 );
                  nF += 1;
               }
            }
            else {
               /* Add right faces. */
               SETF( mesh->F, nF+0, id0, id3, id2 );
               /* Add bottom faces. */
               SETF( mesh->F, nF+1, id2, id1, id3 );
               nF += 2;
            }
         }
         /* Corner case #1 => Nothing on bottom-right, while elements on the right and bottom. */
         else {
            id2 = mesh->Mt[ len*i     + j+1 ];
            id3 = mesh->Mt[ len*(i+1) + j   ];
            if ((id2 >= 0) && (id3 >= 0)) {
               /* Add faces. */
               SETF( mesh->F, nF+0, id0, id3, id2 );
               nF += 1;
            }
         }
      }
   }

   /* We must set new values, as we are setting a subset sometimes. */
   mesh->n  = ide;
   mesh->nF = nF;
}


/**
 * @brief Computes the mesh for the interest point.
 */
static void dali_meshCompute( dali_t *desc, dali_mesh_t *mesh,
                              const dali_img_t *im, const dali_params_t *params )
{
   int i;

   /* Compute mesh. */
   switch (params->mtype) {
      case DALI_MESH_TYPE_SQUARE_DENSE:
         dali_meshComputeSquare( desc, mesh, im, params );
         break;
      case DALI_MESH_TYPE_CIRCLE_DENSE:
         dali_meshComputeCircle( desc, mesh, im, params );
         break;
      case DALI_MESH_TYPE_CIRCLE_VARIABLE:
         dali_meshComputeCircleVariable( desc, mesh, im, params );
         break;
      case DALI_MESH_TYPE_CIRCLE_GAUSS:
         dali_meshComputeCircleGaussian( desc, mesh, im, params );
         break;
   }

   /* Apply beta. */
   for (i=0; i<mesh->n; i++)
      mesh->V[ 3*i+2 ] *= params->beta;

#if 0
   int len = 2*params->Sz+1;
   int j;
   for (i=0; i<len; i++) {
      for (j=0; j<len; j++) {
         printf( "% 4d ", mesh->Mt[ len*i + j ]+1 );
         //printf( "% 4.0f ", mesh->gauss[ len*i + j ] );
      }
      printf( "\n" );
   }
#endif

   /* Display mesh. */
#if 0
   FILE *fdot = fopen( "mesh.py", "w" );
   fprintf( fdot,
      "import pydot\n"
      "graph = pydot.Dot( 'Mesh', graph_type='graph' )\n" );
   for (i=0; i<mesh->n; i++) {
      fprintf( fdot,
         "node_%d = pydot.Node( '%%d' %% %d, shape='circle', pos='%.1f,%.1f)', label='\\\"\\\"', fillcolor='%s', style='filled' )\n"
         "graph.add_node( node_%d )\n",
         i, i, 100.*mesh->V[ 3*i+0 ], -100.*mesh->V[ 3*i+1 ],
         (i<mesh->nr) ? "grey" : "white", i );
   }
   for (i=0; i<mesh->nF; i++) {
      fprintf( fdot,
         "graph.add_edge( pydot.Edge( node_%d, node_%d ) )\n"
         "graph.add_edge( pydot.Edge( node_%d, node_%d ) )\n"
         "graph.add_edge( pydot.Edge( node_%d, node_%d ) )\n",
         mesh->F[ 3*i+0 ], mesh->F[ 3*i+1 ],
         mesh->F[ 3*i+1 ], mesh->F[ 3*i+2 ],
         mesh->F[ 3*i+2 ], mesh->F[ 3*i+0 ] );
   }
   fprintf( fdot,
      "graph.set_simplify( True )\n"
      "graph.write_png( 'mesh.png', prog=['neato','-n'] )\n" );
   fclose( fdot );
#endif

#ifdef FOUT
   FILE *fout = fopen( "mesh_V.dat", "w" );
   for (i=0; i<mesh->n; i++)
      fprintf( fout, "%.18f %.18f %.18f\n", mesh->V[ 3*i+0 ]+1, mesh->V[ 3*i+1 ]+1, mesh->V[ 3*i+2 ] );
   fclose( fout );
   fout = fopen( "mesh_F.dat", "w" );
   for (i=0; i<mesh->nF; i++)
      fprintf( fout, "%d %d %d\n", mesh->F[ 3*i+0 ]+1, mesh->F[ 3*i+1 ]+1, mesh->F[ 3*i+2 ]+1 );
   fclose( fout );
#endif

   /* Compute areas. */
   dali_meshComputeAreas( mesh );
}
#undef SETF
#undef MEAN4


/**
 * @brief Computes and calculates the mesh area matrix.
 *
 * The matrix is diagonal and thus sparse.
 */
static void dali_meshComputeAreas( dali_mesh_t *mesh )
{
   int i;
   int fid1, fid2, fid3;
   double a[3], b[3], c[3];
   double area;

   /* Clear the data. */
   memset( mesh->ai, 0, mesh->n*sizeof(double) );

   /* Calculate all areas in form of a vector. */
   for (i=0; i<mesh->nF; i++) {
      fid1 = mesh->F[ 3*i + 0 ];
      fid2 = mesh->F[ 3*i + 1 ];
      fid3 = mesh->F[ 3*i + 2 ];

      /* We calculate the area of the triangle here. */
      vec3_sub( a, &mesh->V[ 3*fid2 ], &mesh->V[ 3*fid1 ] );
      vec3_sub( b, &mesh->V[ 3*fid3 ], &mesh->V[ 3*fid1 ] );
      vec3_cross( c, a, b );
      area = vec3_norm( c ) / 2.;

      /* Give all the vertices the area. */
      mesh->ai[ fid1 ] += area;
      mesh->ai[ fid2 ] += area;
      mesh->ai[ fid3 ] += area;
   }

   /* Create the A matrix from the vector. */
   mesh->A->nz = 0; /* Reset matrix without affecting values .*/
   mesh->A->n  = mesh->n;
   mesh->A->m  = mesh->n;
   for (i=0; i<mesh->n; i++)
      cs_entry( mesh->A, i, i, mesh->ai[i] );
}


#define MOD(x,y)  (((x) < 0) ? ((x)%(y)+(y)) : (x)%(y))
/**
 * @brief Computes the Laplace-Beltrami operator for a given triangular mesh.
 */
static int dali_computeLaplaceBeltrami( dali_mesh_t *mesh )
{
   int i, j, i1, i2, i3;
   int fid1, fid2, fid3;
   double pp[3], qq[3], t;
   int n = mesh->n;

   /* Reset the matrix, this avoids more memory allocations. */
   mesh->L->nz = 0;
   mesh->L->m  = mesh->n;
   mesh->L->n  = mesh->n;

   /* To cheat we can store temporarily the sum of the diagonal in eigv. */
   memset( mesh->eigv, 0, mesh->n*sizeof(double) );

   /* Calculate the non-diagonal terms of the Laplace-Beltrami operator. */
   for (i=0; i<3; i++) {
      i1 = MOD( i-1, 3 );
      i2 = MOD( i,   3 );
      i3 = MOD( i+1, 3 );

      /* For all faces. */
      for (j=0; j<mesh->nF; j++) {
         /* ID shortcuts. */
         fid1 = mesh->F[ 3*j + i1 ];
         fid2 = mesh->F[ 3*j + i2 ];
         fid3 = mesh->F[ 3*j + i3 ];

         /* pp vector. */
         vec3_sub( pp, &mesh->V[ 3*fid2 ], &mesh->V[ 3*fid1 ] );
         vec3_normalize( pp );

         /* qq vector. */
         vec3_sub( qq, &mesh->V[ 3*fid3 ], &mesh->V[ 3*fid1 ] );
         vec3_normalize( qq );

         /* Set W. */
         t = 1.0 / tan( acos( vec3_dot( pp, qq ) ) );
         /* Set both symmetrical entries. */
         cs_entry( mesh->L, fid2, fid3, -t );
         cs_entry( mesh->L, fid3, fid2, -t );

         /* Add sum to diagonal. */
         mesh->eigv[ fid2 ] += t;
         mesh->eigv[ fid3 ] += t;
      }
   }

   /* We must now put the sum into the diagonal. */
   for (i=0; i<n; i++)
      cs_entry( mesh->L, i, i, mesh->eigv[i] );

   return 0;
}
#undef MOD


/**
 * @brief Computes a subset of eigenvectors of the Laplace-Beltrami operator.
 */
static int dali_computeEigs( dali_mesh_t *mesh, const dali_params_t *params )
{
   int ret;
   cs *Li, *Aim;
   EigsOpts_t opts;

   /* CSparse forces us to compress the matrices to be able to perform operations on them. */
   Li    = cs_compress( mesh->L );
   Aim   = cs_compress( mesh->A );

#if DEBUG
   int i, fnan, finf;
   fnan = 0;
   finf = 0;
   for (i=0; i<Li->nzmax; i++) {
      if (isnan(Li->x[i]))
         fnan = 1;
      if (isinf(Li->x[i]))
         finf = 1;
   }
   if (fnan)
      fprintf( stderr, "L matrix has NaN values!\n" );
   if (finf)
      fprintf( stderr, "L matrix has NaN values!\n" );
   fnan = 0;
   finf = 0;
   for (i=0; i<Aim->nzmax; i++) {
      if (isnan(Aim->x[i]))
         fnan = 1;
      if (isinf(Aim->x[i]))
         finf = 1;
   }
   if (fnan)
      fprintf( stderr, "A matrix has NaN values!\n" );
   if (finf)
      fprintf( stderr, "A matrix has NaN values!\n" );
#endif /* DEBUG */

   /* Set options. */
   eigs_optsDefault( &opts );
   opts.sigma = -1e-5; /* To avoid singularity issues. */
   opts.ncv   = params->lanczos; /* Set the number of lanczos vectors. */
   opts.iters = params->eigs_iter; /* Set iterations. */

#ifdef FOUT
   int j;
   double *A;
   cs *LL = mesh->L;
   FILE *foutL = fopen( "data_Lsparse.dat", "w" );
   FILE *foutA = fopen( "data_Asparse.dat", "w" ); 
   A = calloc( mesh->n*mesh->n, sizeof(double) );
   for (i=0; i<LL->nz; i++)
      A[ LL->p[i]*mesh->n+LL->i[i] ] = LL->x[i];
   for (i=0; i<mesh->n; i++) {
      fprintf( foutA, "%.18f ", mesh->ai[i] );
      for (j=0; j<mesh->n; j++)
         fprintf( foutL, "%.18f ", A[ i*mesh->n+j ] );
      fprintf( foutL, "\n" );
   }
   fprintf( foutA, "\n" );
   fclose( foutL );
   fclose( foutA );
   free( A );
#endif /* FOUT */

   /* We must now calculate the eigenvectors. */
   ret = eigs( mesh->n, params->ncomp, mesh->eigd, mesh->eigv, Li, Aim,
         EIGS_ORDER_LM, EIGS_MODE_G_SHIFTINVERT, NULL, &opts );
   if (ret) {
      fprintf( stderr, "eigs failed to run\n" );
      memset( mesh->eigv, 0, params->ncomp*mesh->n*sizeof(double) );
      memset( mesh->eigd, 0, params->ncomp*sizeof(double) );
      return ret;
   }

#ifdef FOUT
   FILE *foutd = fopen( "data_eigd.dat", "w" );
   FILE *foutv = fopen( "data_eigv.dat", "w" );
   for (i=0; i<params->ncomp; i++) {
      fprintf( foutd, "%.18f ", mesh->eigd[i] );
      for (j=0; j<mesh->n; j++)
         fprintf( foutv, "%.18f ", mesh->eigv[ i*mesh->n + j ] );
      fprintf( foutv, "\n" );
   }
   fprintf( foutd, "\n" );
   fclose(foutd);
   fclose(foutv);
#endif

   /* Clean up. */
   cs_spfree( Li );
   cs_spfree( Aim );

   return 0;
}


/**
 * @brief Compute the Heat Kernel Signature.
 *
 * This is done by \f$ HKS(p) = \sum_i e^{-\lambda_i t} \phi^2_i(p) \f$ with
 * \f$ \lambda_i \f$ the i-th eigenvalue of eigenvector \f$ \phi_i \f$.
 */
static int dali_computeHKS( dali_mesh_t *mesh, const dali_params_t *params )
{
   int i, j, k, n;
   int ncomp, ntime;
   double t;

   /* Comfort. */
   ncomp = params->ncomp;
   ntime = params->ntime;

   /* To minimize memory needed, we'll operate with static right hand operator.
    * Here we store basically the expression \f$ e^{-\lammbda_i t} \f$. */
   n = ncomp-1;
   for (j=0; j<ntime; j++)
      for (i=0; i<n; i++)
         mesh->expd[ j*n + i ] = exp( -fabs(mesh->eigd[ 1+i ]) * mesh->t[ j ] );

   /* Now we can do the operation on a line to line basis.i
    * Note that we are operating on the REAL mesh coordinates and not the full mesh
    * coordinates that include possible virtual nodes (depending on how it's meshed. */
   for (k=0; k<mesh->nr; k++) {

      /* Create temporary row of the eigenvector matrix squared, as it's iterated over
       * for each new different time value. */
      for (i=0; i<n; i++)
         mesh->hks_buf[ i ] = POW2( mesh->eigv[ (1+i)*mesh->n + k ] );

      /* Now we compute all the operations on the row of the buffer. */
      for (j=0; j<ntime; j++) {
         /* here we perform the actual sum for the specific pixel and time slice. */
         t = 0.;
         for (i=0; i<n; i++)
            t += mesh->hks_buf[i] * mesh->expd[ j*n + i ];
         /* The log here is not actually part of the HKS, but of the HKS-SI.
          * However, this avoids us having to calculate it later with more overhead. */
         mesh->HKS[ k*ntime + j ] = log( t );
      }
   }

#ifdef FOUT
   FILE *fout = fopen( "mesh_hksi.dat", "w" );
   for (i=0; i<mesh->nr; i++) {
      for (j=0; j<ntime; j++)
         fprintf( fout, "%.18f ", exp( mesh->HKS[ i*ntime + j ] ) );
      fprintf( fout, "\n" );
   }
   fclose( fout );
#endif

   return 0;
}


/**
 * @brief Compute the Heat Kernel Signature - Scale Invariant.
 */
static int dali_computeHKS_SI( dali_t *desc, dali_mesh_t *mesh, const dali_params_t *params )
{
   int i, j;
   int ntime;
   int nr, nc, pitch;

   /* Comfort parameters. */
   ntime = params->ntime;
   nr    = mesh->nr;
   nc    = ntime-1;
   pitch = ntime;

   /* Calculate the derivative of the Heat Kernel Signature. */
   for (i=0; i<nr; i++)
      for (j=0; j<nc; j++)
         mesh->HKS[ i*pitch + j ] = mesh->HKS[ i*pitch+j+1 ] - mesh->HKS[ i*pitch+j ];

#ifdef FOUT
   FILE *fout = fopen( "dkhs.dat", "w" );
   for (j=0; j<nr; j++) {
      for (i=0; i<nc; i++)
         fprintf( fout, "%.18f ", mesh->HKS[ j*pitch + i ] );
      fprintf( fout, "\n" );
   }
   fclose( fout );
#endif

   /* Compute the Fast Fourier Transform. */
   fftw_execute_dft_r2c( mesh->fftp, mesh->HKS, mesh->HKS_fft );

   /** @TODO there is no reason to do the mirroring, in fact this is just more memory usage
    * and slows us down so this should be removed. */
   /* Complete the thingy. */
   for (i=0; i<nr; i++)
      for (j=nc/2+1; j<nc; j++)
         mesh->HKS_fft[ i*nc + j ] = conj( mesh->HKS_fft[ i*nc + (nc-j) ] );

   /* Absolute value. */
   desc->desc = malloc( nr * params->wmax * sizeof(double) );
   assert( desc->desc != NULL );
   for (j=0; j<nr; j++)
      for (i=0; i<params->wmax; i++)
         desc->desc[ j*params->wmax+i ] = cabs( mesh->HKS_fft[ j*nc+i ] );

#ifdef FOUT
   fout = fopen( "descriptor.dat", "w" );
   for (j=0; j<nr; j++) {
      for (i=0; i<params->wmax; i++)
         fprintf( fout, "%.18f ", desc->desc[ j*params->wmax+i ] );
      fprintf( fout, "\n" );
   }
   fclose(fout);
   fout = fopen( "mask.dat", "w" );
   for (j=0; j<nr; j++) {
      fprintf( fout, "%d ", desc->mask[ j ] );
   }
   fclose(fout);
#endif

   return 0;
}



void dali_print( const dali_t *desc )
{
   int i, j, id;
   double v;
#if 0
   int m;

   for (i=0; i<desc->ulen; i++) {
      for (j=0; j<desc->vlen; j++) {
         id = desc->shape[ i*desc->vlen + j ];
         if (id < 0)
            v = 0.;
         else
            v = desc->desc[ id*desc->wlen + 1 ];
         printf( "% 4.2f ", v );
      }
      printf( "\n" );
   }
   printf( "\n" );
   for (i=0; i<desc->ulen; i++) {
      for (j=0; j<desc->vlen; j++) {
         id = desc->shape[ i*desc->vlen + j ];
         if (id < 0)
            m = 0;
         else
            m = desc->mask[ id ];
         printf( "% 3d ", m );
      }
      printf( "\n" );
   }
   printf( "\n" );
#endif
   for (i=0; i<desc->ulen; i++) {
      for (j=0; j<desc->vlen; j++) {
         id = desc->shape[ i*desc->vlen + j ];
         if (id < 0)
            v = 0.;
         else
            v = desc->gauss[ id ];
         printf( "% 3.0f ", v );
      }
      printf( "\n" );
   }
   printf( "\n" );
}


void dali_fprintInfo( FILE *stream, const dali_info_t *info )
{
   fprintf( stream,
         "Meshing dimensions:\n"
         "   Real Nodes:  %d\n"
         "   Total Nodes: %d\n"
         "   Total Faces: %d\n"
         "Timing results:\n"
         "   Meshing:          %.3f s\n"
         "   Laplace-Beltrami: %.3f s\n"
         "   Eigenvectors:     %.3f s\n"
         "   Heat Kernel Sig.: %.3f s\n"
         "   Scale-Invariance: %.3f s\n"
         "   TOTAL:            %.3f s\n",
         info->nodes_real, info->nodes_total, info->faces,
         info->time_meshing,
         info->time_laplacebeltrami,
         info->time_eigenvectors,
         info->time_hks,
         info->time_hks_si,
         info->time_elapsed );
}





