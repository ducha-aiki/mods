#ifndef DALI_H
#define DALI_H
#include <stdio.h>


/**
 * @mainpage DaLI Descriptor Library
 * @author Edgar Simo-Serra <esimo@iri.upc.edu>
 * @version 1.0
 * @date January 2015
 *
 * @section License
 *
 @verbatim
 Copyright (C) <2011-2015>  <Francesc Moreno-Noguer, Edgar Simo-Serra>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 of the GNU General Public License
 as published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.

 Edgar Simo-Serra, Institut de Robotica i Informatica Industrial (CSIC/UPC)
 esimo@iri.upc.edu, http://www-iri.upc.es/people/esimo/
 @endverbatim
 *
 * @section Overview
 *    This library implements the Deformation and Illumination Invariant
 * Feature Point Descriptor or DaLI for short. The object is to provide a
 * robust and fast feature point descriptor that has both deformation and
 * illumination invariant properties. This descriptor is meant to be used in
 * computer vision problems dealing with deformable objects.
 *
 * @section Dependencies
 *
 * - ceigs
 *   - ARPACK
 *   - LAPACK
 *   - BLAS
 *   - gfortran
 *   - CXSparse (suitesparse)
 *   - UMFPACK
 * - FFTW3
 *
 * @section Changelog
 * - Version 0.1 (internal release), January 2012
 *   - Initial implementation as per original paper.
 *
 * @section References
 * - Edgar Simo-Serra, Carme Torras, Francesc Moreno-Noguer.
 *   DaLI: Deformation and Light Invariant Descriptor
 *   International Journal of Computer Vision (IJCV), 2015.
 * - F. Moreno-Noguer.
 *   Deformation and Illumination Invariant Feature Point Descriptor.
 *   Conference in Computer Vision and Pattern Recognition (CVPR), 2011.
 */


/**
 * @brief Controls the type of the mesh being used.
 */
typedef enum dali_mesh_type_e {
   DALI_MESH_TYPE_SQUARE_DENSE, /**< Dense with extra additional virtual nodes evenly distributed. */
   DALI_MESH_TYPE_CIRCLE_DENSE, /**< Dense with extra additional virtual nodes evenly distributed. */
   DALI_MESH_TYPE_CIRCLE_VARIABLE, /**< Variable meshing using additional virtual nodes only on the center.
                                        This inner mesh size is controlled by the Sz_coarse parameter. */
   DALI_MESH_TYPE_CIRCLE_GAUSS, /**< Variable weighting using two parameters following the density of the Gaussian. */
} dali_mesh_type_t;


/**
 * @brief The descriptor parameters.
 *
 * Should be generally set by dali_optsDefault.
 *
 * @sa dali_optsDefault
 */
typedef struct dali_params_s {
   /* Descriptor parameters. */
   dali_mesh_type_t mtype; /**< Type of the mesh. Defaults to DALI_MESH_TYPE_CIRCLE_VARIABLE. */
   int Sz;        /**< Half of the patch size, generally 20<=S<=30. Defaults to 30. */
   int Sz_coarse; /**< Radius to use for denser meshing when using a variable type mesh.
                       Defaults to 15. */
   double beta;   /**< Magnitude of the embedding, generally 1000<=beta<=2000.
                       This represents the value to scale the intensity by.
                       Defaults to 2000. */
   int ncomp;     /**< Number of eigenvalues/vectors to compute for the
                       Laplace-Beltrami operator. This is currently the slowest
                       part of the algorithm and has the greatest impact on performance.
                       Defaults to 200. */
   int ntime;     /**< Number of time slices to use. This is used when
                       performing the DFT. It must be less than or equal to ncomp.
                       Defaults to 100. */
   int wmax;      /**< Dimension in the frequency domain, generally 20. This is
                       only to save storage space. It does not actually speed up
                       calculations. It must be equal or less to ntime.
                       Defaults to 20. */
   /* Meshing parameters. */
   double mesh_K;
   double mesh_sigma;
   /* Implementation parameters. */
   int threads;   /**< Number of threads to use. This will not work without a
                       thread-safe version of ARPACK.
                       Defaults to 1. */
   int lanczos;   /**< Number of Lanczos vectors to use in the Arnoldi iteration algorithm.
                       More vectors generally mean more memory usage and faster convergence.
                       A value of 0 attempts to set a good number of Lanczos vectors based
                       on the value of ncomp. The value should be at least ncomp+1.
                       Usually the value range is between 2*ncomp to 4*ncomp.
                       Defaults to 0. */
   int eigs_iter; /**< Number of iterations to perform maximum for the ARNOLDI iteration
                       algorithm used to calculate the Heat Kernel Signature.
                       Defaults to 300. */
   int use_si;    /**< Whether or not to perform the final scale invariance stage (i.e. calculate HKS instead of HKS-SI
                       Defaults to 1 */
   int verbose;   /**< Verbosity level of the algorithm. 0 is completely
                       silent, while other values will display progress through stdout.
                       Defaults to 0. */
} dali_params_t;


/**
 * @brief Gets information from the execution time.
 */
typedef struct dali_info_s {
   int nodes_real;            /**< Number of real nodes (pixels). */
   int nodes_total;           /**< Total number of nodes for calculations. */
   int faces;                 /**< Number of faces for calculations. */
   double time_meshing;       /**< Time spent on meshing. */
   double time_laplacebeltrami; /**< Time spent on calculating Laplace-Beltrami. */
   double time_eigenvectors;  /**< Time spent calculating eigenvectors. */
   double time_hks;           /**< Time spent calculating the Heat Kernel Signature. */
   double time_hks_si;        /**< Time spent making scale-invariant. */
   double time_elapsed;       /**< Sum of all elapsed time. */
} dali_info_t;


/**
 * @brief The actual DaLI descriptor itself.
 *
 * To access the descriptor the following code can be used:
 * @code
 * // Accesses slice at position (u,v,w) within the descriptor
 * // Only valid for u in [desc->uoff, desc->uoff+desc->ueff)
 * // Only valid for v in [desc->voff, desc->voff+desc->veff)
 * // Only valid for w in [0,desc->wlen)
 * double d = desc->desc[ ((u-desc->uoff)*desc->vlen + (v-desc->vlen) * desc->wlen + w ];
 * @endcode
 *
 * @sa dali_compute
 */
typedef struct dali_s {
   int n;         /**< Number of descriptors. This is used internally for freeing and shouldn't be modified. */
   /* Descriptor information. */
   int nodes;     /**< Number of nodes. */
   int ulen;      /**< The pitch of the width of the descriptor. */
   int vlen;      /**< The pitch of the height of the descriptor. */
   int wlen;      /**< Frequency information of the descriptor. */
   int sz;        /**< Original half patch size. */
   /* The descriptor information and mask aren't necessarily square and can
    * adopt a myriad of forms. */
   int len;       /**< Descriptor length. */
   double *desc;  /**< Descriptor data of length len*wlen. */
   char *mask;    /**< Mask data of length len. */
   int *shape;    /**< Buffer that keeps track of the patch shape. Shared by all descriptors. */
   /* Comparison information to speed up comparison. */
   double *gauss; /**< Gaussian information (only stores x^2+y^2) of length len.
                       Shared by all descriptors. */
   double *sigma;  /**< Sigma used to calculate sgauss. */
   double *sgauss; /**< Actual Gaussian information of length len. Shared by all descriptors. */
} dali_t;


/**
 * @brief Gets an element from a descriptor.
 *
 *    @param desc Descriptor to get element from.
 *    @param u U coordinate to get element from.
 *    @param v V coordinate to get element from.
 *    @param w Frequency coordinate to get element from.
 *    @return The value of the descriptor at that coordinate.
 */
/*
static double dali_elem( const dali_t *desc, int u, int v, int w )
{
   int urel, vrel;
   urel = u - desc->uoff;
   vrel = v - desc->voff;
   if ((urel < 0) || (urel >= desc->ueff))
      return NAN;
   if ((vrel < 0) || (vrel >= desc->veff))
      return NAN;
   if ((w < 0) || (w >= desc->wlen))
      return NAN;
   return desc->desc[ (urel * desc->vlen + vrel) * desc->wlen + w ];
}
*/


/**
 * @brief Sets the options to be the default for the DaLI descriptor.
 *
 *    @param[out] params Parameters to set.
 */
#ifdef __cplusplus
extern "C"
#endif
void dali_optsDefault( dali_params_t *params );


/**
 * @brief Computes the DaLI descriptor for a point of interest in an image.
 *
 *    @param[in] im Image to compute descriptor on.
 *    @param[in] w Width of the image.
 *    @param[in] h Height of the image.
 *    @param[in] u U coordinates of points of interest.
 *    @param[in] v V coordinates of points of interest.
 *    @param[in] n Number of points of interest.
 *    @param[in] params Parameters to use when computing the DaLI descriptor.
 *    @return The descriptors calculated.
 * @sa dali_free
 * @sa dali_distance
 */
#ifdef __cplusplus
extern "C"
#endif
dali_t* dali_compute( const double *im, int w, int h,
                      const int *u, const int *v, int n,
                      const dali_params_t *params,
                      dali_info_t *info );


/**
 * @brief Calculates the euclidean distance between two DaLI descriptors.
 *
 *    @param[in] desc1 First descriptor to compare.
 *    @param[in] desc2 Second descriptor to compare.
 *    @param[in] sigma Standard deviation of the Gaussian smoohting. Value is in proportion
 *               of descriptor size. A good value is generally 0.5 or 0.25.
 *    @return The euclidean distance between both descriptors or NaN if masks don't overlap.
 * @sa dali_distance_transform
 */
#ifdef __cplusplus
extern "C"
#endif
double dali_distance( dali_t *desc1, dali_t *desc2, double sigma );


/**
 * @brief Calculates the euclidean distance between two DaLI descriptors with rotation and scaling.
 *
 * The transformation formed by both rotation and scaling is applied to the
 * coordinates of the second descriptor. It is implemented using nearest neighbours.
 *
 *    @param[in] desc1 First descriptor to compare.
 *    @param[in] desc2 Second descriptor to compare.
 *    @param[in] sigma Standard deviation of the Gaussian smoohting. Value is in proportion
 *               of descriptor size. A good value is generally 0.5 or 0.25.
 *    @param[in] theta Amount to rotate second descriptor by (radians).
 *    @param[in] scale Amount of scale to apply to the second descriptor (1.0 is no scaling).
 *    @return The euclidean distance between both descriptors or NaN if masks don't overlap.
 * @sa dali_distance
 */
#ifdef __cplusplus
extern "C"
#endif
double dali_distance_transform( dali_t *desc1, dali_t *desc2,
                                double sigma, double theta, double scale );


/**
 * @brief Calculates the euclidean distance between two DaLI descriptors with rotation and scaling.
 *
 * The transformation formed by both rotation and scaling is applied to the
 * coordinates of the second descriptor. It is implemented using linear interpolation.
 *
 *    @param[in] desc1 First descriptor to compare.
 *    @param[in] desc2 Second descriptor to compare.
 *    @param[in] sigma Standard deviation of the Gaussian smoohting. Value is in proportion
 *               of descriptor size. A good value is generally 0.5 or 0.25.
 *    @param[in] theta Amount to rotate second descriptor by (radians).
 *    @param[in] scale Amount of scale to apply to the second descriptor (1.0 is no scaling).
 *    @return The euclidean distance between both descriptors or NaN if masks don't overlap.
 * @sa dali_distance
 */
#ifdef __cplusplus
extern "C"
#endif
double dali_distance_transform_lin( dali_t *desc1, dali_t *desc2,
                                    double sigma, double theta, double scale );

#ifdef __cplusplus
extern "C"
#endif
double dali_distance_pure( dali_t *desc1, dali_t *desc2 );


/**
 * @brief Cleans up a dali_t struct returned by dali_compute.
 *
 * The pointer passed should be the same pointer as returned by dali_compute.
 *
 *    @param[in] dali Descriptor to free.
 * @sa dali_compute
 */
#ifdef __cplusplus
extern "C"
#endif
void dali_free( dali_t *dali );


/**
 * @brief Prints the dali descriptor.
 *
 *    @param desc Descriptor to print.
 */
#ifdef __cplusplus
extern "C"
#endif
void dali_print( const dali_t *desc );


/**
 * @brief Prints the information.
 */
#ifdef __cplusplus
extern "C"
#endif
void dali_fprintInfo( FILE *stream, const dali_info_t *info );


#endif /* DALI_H */



