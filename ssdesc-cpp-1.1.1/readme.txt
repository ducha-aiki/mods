------------------------------------------------------
* C++ Implementation of the Self-Similarity Descriptor
* Version 1.1.1
* 
* Author: Ken Chatfield, March 2009
* ken@robots.ox.ac.uk
* http://www.robots.ox.ac.uk/~ken
------------------------------------------------------

Copyright (c) 2009, Ken Chatfield
All rights reserved. Licensed under the MIT license.

Table of Contents
~~~~~~~~~~~~~~~~~

1. Introduction
2. Description of Files
3. Direct Usage of C++ Implementation
4. Usage of MEX Wrapper
5. Self-Similarity Descriptor Parameters
6. Note on Allowed Range for Descriptors
7. Other Notes
8. Revision History


1. Introduction
~~~~~~~~~~~~~~~

This is a C++ implementation, with an optional Matlab MEX wrapper, of the self-similarity
descriptor first introduced in:

  Shechtman E., Irani M., "Matching Local Self-Similarities across Images and Videos" CVPR '07

With the extensions proposed in:

  Chatfield K., Philbin J., Zisserman A., "Efficient Retrieval of Deformable Shape Classes using
    Local Self-Similarities" ICCV Workshop on Non-rigid Shape Analysis and Deformable Image
    Alignment (NORDIA '09)

If you use this code in your work, please cite the above paper.

The interface of this implementation is designed to be compatible with a previously developed
Matlab implementation of the descriptor also available on the package webpage:

  http://www.robots.ox.ac.uk/~vgg/software/SelfSimilarity/

2. Description of Files
~~~~~~~~~~~~~~~~~~~~~~~

1. ssdesc.h/ssdesc.cc     C++ implementation of descriptor
2. mexCalcSsdescs.cc      Optional Matlab MEX wrapper

3. Direct Usage of C++ Implementation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There are two main functions which are used in turn to extract and process the self-similarity
descriptors from an image:

* calc_ssdescs/calc_ssdescs_alt
    -> calculates descriptors densely for an image/region or single point
    -> both calc_ssdescs and calc_ssdescs_alt are functionally equivalent, but the 'alt'
       variant uses a sliding histogram approach to speed up computation so should be
       preferred in practice
* prune_normalise_ssdescs
    -> takes the descriptors calculated by the above function and sparsifies, removing
       salient/homogeneous/non-informative descriptors, and then normalises the
       remaining descriptors over the range [0-1]

Note that the descriptors returned from calc_ssdescs/calc_ssdescs_alt are *not* normalised, and
so if using this function alone normalisation must either be performed manually, or
prune_normalise_ssdescs called anyway for that purpose. Both functions use templates to specify
the type of the source image and the precision of internal calculations (either float or double).

a. Usage of calc_ssdescs/calc_ssdescs_alt
-----------------------------------------

calc_ssdescs/calc_ssdescs_alt<fnumtype>(image, image_width, image_height, image_channels, parms,
  ssdescs, calc_rect);

Inputs:
  image	             column-major array of fnumtype (float or double) containing image data
                      in form [col row chan]
  image_width        width of input image
  image_heght        height of input image
  image_channels     number of color channels in image
  parms              self-similarity descriptor parameters (refer to Section 5 below)
  calc_rect          (optional) struct which specifies the region within which to compute
                      descriptors. If not specified, by default descriptors are computed for the
                      whole image, otherwise is a struct of type rectRegion which has fields
                      xfrom, xto, yfrom, yto which allow specification of a rectangular ROI
                      See section 6 below for a note on allowed ranges
Outputs:
  ssdescs            output containing array of self-similarity descriptors for all pixels in the
                      image in column-major form [(width-cor_size) (height-cor_size) nrad nang]
                      See section 6 below for details on the region which is returned

b. Usage of prune_normalise_ssdescs
-----------------------------------

prune_normalise_ssdescs(ssdescs, image_width, image_height, parms, resp, draw_coords,
  salient_coords, homogeneous_coords, snn_coords, calc_rect);

Inputs:
  ssdescs             descriptors as returned from calc_ssdescs/calc_ssdescs_alt
  image_width         width of input image
  image_height        height of input image
  parms               same self-similarity descriptor parameters as used with
                       calc_ssdescs/calc_ssdescs_alt
  calc_rect           (optional) same region data as passed to calc_ssdescs/calc_ssdescs_alt
Outputs:
  resp                array of normalised descriptors which are neither homogeneous/salient or
                       fail the 2NN threshold in the form [desc_size nPoints]
  draw_coords         array containing the coordinates of the descriptors returned in resp
  salient_coords      array containing the coordinates of the descriptors which were deemed to be
                       salient
  homogeneous_coords  array containing the coordinates of the descriptors which were deemed to be
                       homogeneous
  snn_coords          array containing the coordinates of the descriptors which failed the 2NN test

4. Usage of MEX Wrapper
~~~~~~~~~~~~~~~~~~~~~~~

First the MEX file needs to be compiled at the Matlab command prompt as follows:

>> mex mexCalcSsdescs.cc ssdesc.cc

The function can then be called from Matlab as follows:

[resp, draw_coords, salient_coords, homogeneous_coords, snn_coords] = mexCalcSsdescs(img, parms,
  calc_rect);

Inputs:
  img                 greyscale image of type double [from MATLAB just use double(imread(*))]
  parms               self-similarity descriptor parameters
  calc_rect           (optional) if unspecified descriptors are calculated densely for the entire
                       image, if a two vector is specified the descriptor of a single point [x y]
                       is returned, if a four vector is specified the descriptors within the region
                       [x_left y_top x_right y_bottom] are returned
                       See section 6 below for a note on allowed ranges
Outputs:
  Same as the outputs of the prune_normalise_ssdescs C++ function, described in Section 3b. above

5. Self-Similarity Descriptor Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A description of the accepted self-similarity parameters is provided below. For further details,
refer to the two papers cited in Section 1.

                    DEFAULT DESCRIPTION

patch_size          5       The size of the internal patch used for calculation off the ssd
                             surface, which will be swept across the correlation window.
                             Must be odd.
desc_rad            40       The radius of the self-similarity descriptor.
nrad                3        Number of intervals into which to quantise the radial bins.
nang                12       Number of intervals into which to quantise the angular bins.
var_noise           300000   Constant corresponding to acceptable photometric variations
                              (in colour, illumination or due to noise) which will be
                              suppressed when calculating the bin values. The default value
                              is a good starting point, and used for all experiments in
                              Chatfield et al., but this parameter should be set empirically.
                              A good guideline might be:
                               parms.patch_size^2*(image_channel_count)*(estimated_variance)
saliency_thresh     0.7      Used for salient descriptor detection. If all bins in the
                              non-normalised descriptor have a ssd value of this threshold
                              or more when compared to the central patch, then the
                              descriptor is marked as salient. A value of 1.0 disables
                              salient descriptor detection.
homogeneity_thresh  0.7      Used for homogeneous descriptor detection.
                              If all bins in the non-normalised descriptor have a similarity
                              (1-ssd) of this threshold or more when compared to the central
                              patch, then the descriptor is marked as homogeneous. A value
                              of 1.0 disables homogeneous descriptor detection.
snn_thresh          0.85     Used for elimination of descriptors based upon a second-nearest
                              neighbour constraint. For each descriptor, takes the
                              two most similar matching descriptors from across the image,
                              then eliminates those descriptors whose euclidean distance
                              ratio to these two descriptors d1/d2 > snn_thresh (where d2 >
                              d1). A value of 1.0 disables the second-nearest neighbour test.

6. Note on Allowed Range for Descriptors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There is a margin of:
  margin = desc_rad + (patch_size-1)/2
around the image within which descriptors cannot be calculated. This is because this is the
size of the neighborhood around a point which is used when computing the self-similarity
descriptor. This means that:

1. When specifying a sub-region of an image within which to calculate descriptors using the
   calc_rect parameter, x and y must lie within the closed interval:
     x_range = [margin, w-margin-1]
     y_range = [margin, h-margin-1]
   where w and h are the width and height of the image respectively and using 0-based
   indexing (note this range will be shifted +1 when using Matlab style 1-based indexing)
2. The descriptors returned from all functions will also lie within this range

7. Other Notes
~~~~~~~~~~~~~~

* The code can optionally use the `fastann` library to speed up computation, available from:
       http://www.robots.ox.ac.uk/~vgg/software/fastann/
   If this code is available, define the USE_APPROXNN flag in ssdesc.h prior to compilation
   to use it.
* The self-similarity descriptor is by its nature expensive to compute. Despite the improvement in
   performance of this C++ implementation when compared to the previously available Matlab one,
   calculating descriptors densely across an image can still take some time. For example, on a
   Pentium 2.4 GHz it can take up to 10 minutes to compute all the descriptors for a VGA (640x480)
   image when using exact nearest neighbors for thresholding.

8. Revision History
~~~~~~~~~~~~~~~~~~~
1.1.1   January 2013    - Updated README file and added licence for public release
1.1     April 2010      - Tidied up code and replaced all C-style dynamic arrays with STL containers
                        - Allowed for approximate nearest neighbours (using external library) to be
                          enabled/disabled using USE_APPROXNN define
1.0     March 2009      - Initial release

