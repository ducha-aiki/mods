/*
 * ssdesc.cc
 *
 *  Ken Chatfield
 *  Engineering Department
 *  University of Oxford
 *
 *  March 2009
 *  Updated March 2010
 */

#include "ssdesc.h"
//#include <iostream>
/**
 * Computes a self similarity descriptor for every pixel in the image.
 * This is the slower version.
 *
 * See header file for detailed description
 */
template<class fnumtype> void ssdesc::calc_ssdescs(const fnumtype* image, const int image_width, const int image_height, const int image_channels,
                                                   const ssdesc::ssdesc_parms<fnumtype> &parms, vector<fnumtype>* ssdescs, const ssdesc::rectRegion calc_rect)
{
  // Allocate ssdescs so it is the right size to store the dense descriptors calculated from across the entire image
  vector<fnumtype>((image_height - parms.cor_size + 1)*(image_width - parms.cor_size + 1)*parms.nrad*parms.nang).swap(*ssdescs);

  int xfrom, xto, yfrom, yto;
  if ((calc_rect.xfrom == -1) && (calc_rect.xto == -1) && (calc_rect.yfrom == -1) && (calc_rect.yto == -1))
    {
      // By default, compute the descriptors for the whole image
      xfrom = 0;
      xto = image_width - parms.cor_size + 1;
      yfrom = 0;
      yto = image_height - parms.cor_size + 1;
    } else {
      // Otherwise, use calc_rect if specified, converting from the
      // coordinates of the central point to the top left corner of
      // the correlation window
      xfrom = calc_rect.xfrom - (parms.cor_size-1)/2;
      xto = calc_rect.xto - (parms.cor_size-1)/2 + 1;
      yfrom = calc_rect.yfrom - (parms.cor_size-1)/2;
      yto = calc_rect.yto - (parms.cor_size-1)/2 + 1;
      // check region is within valid bounds
      assert((xfrom > 0) && (xto < (image_width - parms.cor_size + 2)) && (xto >= xfrom) &&
             (yfrom > 0) && (yto < (image_height - parms.cor_size + 2)) && (yto >= yfrom));
    }

  // This will be the size of a single ssd square patch
  int ssd_sz = (parms.cor_size - parms.window_size + 1);
  vector<fnumtype> ssd_temp;
  vector<int> imask;

  ssdesc::details::ssdesc_imask(ssd_sz, parms.nrad, parms.nang, &imask);

  // (x,y) point to the top left corner of the correlation window
  for (int y=yfrom; y<yto; ++y)
    {
      for (int x=xfrom; x<xto; ++x)
        {
          // (xp,yp) point to the top left corner of the inner patch
          int xp = x + parms.cor_size/2 - parms.window_size/2;
          int yp = y + parms.cor_size/2 - parms.window_size/2;

          ssdesc::details::ssd_compute<fnumtype>(image, image_width, image_height, image_channels, x, x + ssd_sz, y, y + ssd_sz,
                                                 xp, yp, ssd_sz, parms.window_size, &ssd_temp);

          ssdesc::details::ssdesc_descriptor(ssd_temp, imask, ssd_sz, parms.nrad, parms.nang, parms.var_noise,
                                             ssdescs, (y*(image_width - parms.cor_size + 1) + x)*(parms.nrad*parms.nang));
        }
    }
}

/**
 * Computes a self similarity descriptor for every pixel in the image.
 * This is the faster version which uses a sliding histogram approach for
 * speed.
 *
 * See header file for detailed description
 */
template<class fnumtype> void ssdesc::calc_ssdescs_alt(const fnumtype* image, const int image_width, const int image_height, const int image_channels,
                                                       const ssdesc::ssdesc_parms<fnumtype> &parms, vector<fnumtype>* ssdescs, const ssdesc::rectRegion calc_rect)
{
  // Allocate ssdescs so it is the right size to store the dense descriptors calculated from across the entire image
  size_t dim = (image_height - parms.cor_size + 1)*(image_width - parms.cor_size + 1)*parms.nrad*parms.nang;
 // std::cerr << "dim= " << dim << std::endl;
  vector<fnumtype>(dim).swap(*ssdescs);
//std::cerr << "1st alloc" << std::endl;
  int xfrom, xto, yfrom, yto;
  if ((calc_rect.xfrom == -1) && (calc_rect.xto == -1) && (calc_rect.yfrom == -1) && (calc_rect.yto == -1))
    {
      // By default, compute the descriptors for the whole image
      xfrom = 0;
      xto = image_width - parms.cor_size + 1;
      yfrom = 0;
      yto = image_height - parms.cor_size + 1;
    } else {
      // Otherwise, use calc_rect if specified, converting from the
      // coordinates of the central point to the top left corner of
      // the correlation window
      xfrom = calc_rect.xfrom - (parms.cor_size-1)/2;
      xto = calc_rect.xto - (parms.cor_size-1)/2 + 1;
      yfrom = calc_rect.yfrom - (parms.cor_size-1)/2;
      yto = calc_rect.yto - (parms.cor_size-1)/2 + 1;
      // check region is within valid bounds
      assert((xfrom >= 0) && (xto < (image_width - parms.cor_size + 2)) && (xto >= xfrom) &&
             (yfrom >= 0) && (yto < (image_height - parms.cor_size + 2)) && (yto >= yfrom));
    }

  // This will be the size of a single ssd square patch
  int ssd_sz = (parms.cor_size - parms.window_size + 1);
  vector<fnumtype> ssd_temp;
  vector<int> imask;

  ssdesc::details::ssdesc_imask(ssd_sz, parms.nrad, parms.nang, &imask);

  // Compute each row first
  // (x,y) point to the top left corner of the correlation window
  for (int y=yfrom; y<yto; ++y)
    {
      // (xp,yp) point to the top left corner of the inner patch
      // we will only compute the first ssd patch in each row fully
      int xp = xfrom + parms.cor_size/2 - parms.window_size/2;
      int yp = y + parms.cor_size/2 - parms.window_size/2;

      ssdesc::details::ssd_compute<fnumtype>(image, image_width, image_height, image_channels, xfrom, xfrom + ssd_sz,
                                             y, y + ssd_sz, xp, yp, ssd_sz, parms.window_size, &ssd_temp);

      ssdesc::details::ssdesc_descriptor(ssd_temp, imask, ssd_sz, parms.nrad, parms.nang, parms.var_noise,
                                         ssdescs, (y*(image_width - parms.cor_size + 1) + xfrom)*(parms.nrad*parms.nang));

      // For each row, incrementally compute the columns
      for (int x=(xfrom+1); x<xto; ++x)
        {
          // (xp,yp) point to the top left corner of the inner patch
          xp = x + parms.cor_size/2 - parms.window_size/2;
          yp = y + parms.cor_size/2 - parms.window_size/2;

          ssdesc::details::ssd_compute_irow<fnumtype>(image, image_width, image_height, image_channels, x, x + ssd_sz, y, y + ssd_sz,
                                                      xp, yp, parms.window_size, ssd_temp, &ssd_temp);

          ssdesc::details::ssdesc_descriptor(ssd_temp, imask, ssd_sz, parms.nrad, parms.nang, parms.var_noise,
                                             ssdescs, (y*(image_width - parms.cor_size + 1) + x)*(parms.nrad*parms.nang));
        }
    }
}

/**
 * Prunes the array of self-similarity descriptors contained in ssdescs
 * by moving valid descriptors into resp, then storing the coordinates
 * of points which fall below/above the saliency, homogeneity and snn
 * thresholds in seperate arrays
 *
 * See header file for detailed description
 *
 * N.B. ssdescs is not defined as const despite being read only since ANN
 * routine requires a non-const array
 */
template<class fnumtype> void ssdesc::prune_normalise_ssdescs(vector<fnumtype>& ssdescs, const int image_width, const int image_height,
							      const ssdesc::ssdesc_parms<fnumtype> &parms, vector<fnumtype>* resp,
							      vector<ssdesc::coordElem>* draw_coords, vector<ssdesc::coordElem>* salient_coords,
							      vector<ssdesc::coordElem>* homogeneous_coords, vector<ssdesc::coordElem>* snn_coords,
							      const ssdesc::rectRegion calc_rect)
{
  int xfrom, xto, yfrom, yto;
  if ((calc_rect.xfrom == -1) && (calc_rect.xto == -1) && (calc_rect.yfrom == -1) && (calc_rect.yto == -1))
    {
      // By default, compute the descriptors for the whole image
      xfrom = 0;
      xto = image_width - parms.cor_size + 1;
      yfrom = 0;
      yto = image_height - parms.cor_size + 1;
    } else {
      // Otherwise, use calc_rect if specified, converting from the
      // coordinates of the central point to the top left corner of
      // the correlation window
      xfrom = calc_rect.xfrom - (parms.cor_size-1)/2;
      xto = calc_rect.xto - (parms.cor_size-1)/2 + 1;
      yfrom = calc_rect.yfrom - (parms.cor_size-1)/2;
      yto = calc_rect.yto - (parms.cor_size-1)/2 + 1;
    }

#ifdef USE_APPROXNN /* if using the library approximate nearest neighbor routine, preconstruct the tree */
  int ndescs = (image_height - parms.cor_size + 1)*(image_width - parms.cor_size + 1);
  jp_nn_kdtree<fnumtype> kdt(&ssdescs[0], ndescs, (parms.nrad*parms.nang), 8);	  //Build the trees.
#else /* else precalculate the bound multiplier for exact NN calculation (see below) */
  const fnumtype boundMultiplier = sqrt((fnumtype)(parms.nrad*parms.nang));
#endif

  // (x,y) point to the top left corner of the correlation window
  for (int y=yfrom; y<yto; ++y)
    {
      for (int x=xfrom; x<xto; ++x)
        {
          // (xc,yc) point to centre of the correlation window
          // (used just for output - top left coordinates used internally)
          int xc = x + (parms.cor_size-1)/2;
          int yc = y + (parms.cor_size-1)/2;
          // retrieve the current descriptor from the array of ssdescs
          fnumtype *ssdesc = &ssdescs[(y*(image_width - parms.cor_size + 1) + x)*(parms.nrad*parms.nang)];

          // find min/max for purposes of salient/homogeneous patch detection
          fnumtype min_ssd = std::numeric_limits<fnumtype>::max();
          fnumtype max_ssd = std::numeric_limits<fnumtype>::min();
          for (int i=0; i<parms.nrad*parms.nang; ++i)
            {
              min_ssd = (ssdesc[i] < min_ssd) ? ssdesc[i] : min_ssd;
              max_ssd = (ssdesc[i] > max_ssd) ? ssdesc[i] : max_ssd;
            }

          /* Perform salient/homogeneous descriptor preening */
          bool salientOrHomogeneous = false;
          if (max_ssd < (1 - parms.saliency_thresh))
            {
              salientOrHomogeneous = true;
              salient_coords->push_back(ssdesc::coordElem(xc,yc));
            }
          if (min_ssd > parms.homogeneity_thresh)
            {
              salientOrHomogeneous = true;
              homogeneous_coords->push_back(ssdesc::coordElem(xc,yc));
            }

          /* Only continue if the descriptor hasn't already been categorised */
          if (salientOrHomogeneous == false)
            {
              /* Perform second nearest neighbour preening if necessary,
                                otherwise categorise as valid descriptor immediately */
              if (parms.snn_thresh < 1)
                {

#ifndef USE_APPROXNN  /*if not using approximate nearest-neighbors, calculate exact NN using own routine */
		  list<double> desc_sims;
		  // min_ssds is used to store the minimum 2 ssd bounds (minssd/sqrt(2))
		  // during the current iteration, so that the ssd is computed
		  // the minimum number of times
		  fnumtype ssd_min_bound1 = std::numeric_limits<double>::max();
		  fnumtype ssd_min_bound2 = std::numeric_limits<double>::max();

		  // 1. Iterate through all other descriptors, calculating and storing similarity
		  for (int y2=0; y2<(image_height - parms.cor_size + 1); ++y2)
		    {
		      for (int x2=0; x2<(image_width - parms.cor_size + 1); ++x2)
			{
			  fnumtype *ssdesc2 = &ssdescs[(y2*(image_width - parms.cor_size + 1) + x2)*(parms.nrad*parms.nang)];
			  // skip comparison to self
			  if (ssdesc == ssdesc2) continue;
			  // calculate the ssd if within the range of the two smallest ssds calculated so far
			  double ssd = ssdesc::details::calc_ssd_ssdesc_min2<fnumtype>(ssdesc, ssdesc2, parms.nrad*parms.nang, ssd_min_bound1, ssd_min_bound2, boundMultiplier);
			  // only store the ssd if within that range
			  if (ssd != -1)
			    {
			      desc_sims.push_back(ssd);
			    }
			}
		    }
		  // 2. Sort the similarities in ascending order
		  desc_sims.sort();
		  // 3. Calculate SNN ratio for current descriptor
		  list<double>::iterator it;
		  it = desc_sims.begin();

		  double snn1 = *it;
		  ++it;
		  double snn2 = *it;

		  double snn = snn1/snn2;

		  list<double>().swap(desc_sims);

#else /* USE_APPROXNN */ /* else calculate fast approximate NN using KD-trees and library routine */
		  //Calculate 2nd Nearest Neighbour's using Approximate Nearest Neighbours Function

		  pair<size_t, fnumtype> nns[4];                         //Must be num_nns+1 big.
		  kdt.search(ssdesc, 3, nns, 512);                 //Search the trees, saving the results in nns.
		  fnumtype snn = sqrt(nns[1].second/nns[2].second);
#endif /* USE_APPROXNN */

		  if (snn > parms.snn_thresh)
		    {
		      snn_coords->push_back(ssdesc::coordElem(xc,yc));
		    } else {
		      draw_coords->push_back(ssdesc::coordElem(xc,yc));
		      for (int i=0; i<parms.nrad*parms.nang; ++i)
			{
			  resp->push_back(ssdesc[i]);
			}
		    }

		} else {
		  draw_coords->push_back(ssdesc::coordElem(xc,yc));
		  for (int i=0; i<parms.nrad*parms.nang; ++i)
		    {
		      resp->push_back(ssdesc[i]);
		    }
		}
	    }
	}
    }

  /* Finally, normalise descriptors remaining in the resp array between 0..1 */
  for (int i=0; i<(int)draw_coords->size(); ++i)
    {
      fnumtype min_bin = std::numeric_limits<fnumtype>::max();
      fnumtype max_bin = std::numeric_limits<fnumtype>::min();

      int descOffset = i*parms.nrad*parms.nang;
      for (int binOffset=0; binOffset<parms.nrad*parms.nang; ++binOffset)
        {
          min_bin = ((*resp)[descOffset + binOffset] < min_bin) ? (*resp)[descOffset + binOffset] : min_bin;
          max_bin = ((*resp)[descOffset + binOffset] > max_bin) ? (*resp)[descOffset + binOffset] : max_bin;
        }
      for (int binOffset=0; binOffset<parms.nrad*parms.nang; ++binOffset)
        {
          (*resp)[descOffset + binOffset] = ((*resp)[descOffset + binOffset] - min_bin)/(max_bin - min_bin);
        }
    }
}

/**
 * Creates an indice mask of the same size as the correlation patch mapping each ssd pixel into the appropriate
 * descriptor bin using a log-polar grid
 *
 *
 * Params
 * ------
 * INPUTS:
 * ssd_sz : int
 *    The radius of the self-similarity descriptor to produce (must be half the width of the
 *    correlation patch minus (correlation_patch_size-1)
 * nrad : int
 *    Number of radial bins in the descriptor to produce
 * nang : int
 *    Number of angular bins in the descriptor to produce
 * OUTPUTS:
 * imask : vector<int>*
 *    an array of integers of size ssd_sz^2 in row-major form indicating bin indices within the
 *    correlation patch. Bins are marked from 0-(nrad*nang-1) with the bins ordered in radial
 *    layers starting from the outermost layer and incrementing clockwise from the negative y-axis.
 *    If a pixel lies outside of the log-polar grid, it is given an index of -1
 */
void ssdesc::details::ssdesc_imask(const int ssd_sz, const int nrad, const int nang, vector<int>* imask)
{
  vector<int>(ssd_sz*ssd_sz).swap(*imask);

  // centre of patch, zero indexed
  int cx = (ssd_sz-1)/2;
  int cy = (ssd_sz-1)/2;

  int rad = (ssd_sz-1)/2;

  // Calculate the radial bin boundaries based on a variable base for log radial bins
  double lpbase = pow(10,log10((double)nrad)/nrad);
  vector<double> radiiQuants(nrad);
  for (int i=0; i<nrad; ++i)
    {
      radiiQuants[i] = (pow(lpbase,i+1)-1)/(nrad - 1)*rad;
    }

  for (int x = 0; x < ssd_sz; ++x)
    {
      for (int y = 0; y < ssd_sz; ++y)
        {

          // If the central point, then skip (don't allocate to any bin)
          if ((x == cx) && (y == cy))
            {
              (*imask)[y*ssd_sz + x] = -1;
              continue;
            }

          double r = sqrt((double)((cx - x)*(cx - x) + (cy - y)*(cy - y)));
          //angle such that ang=0 is the -ve y-axis (for backward compatibility with old code)
          //double ang = atan2((double)(cx - x), (double)(y - cy)) + M_PI;  //(0,2PI]
          double ang = atan2((double)(cx - x), (double)(cy - y)) + M_PI;  //(0,2PI]

          // Calculate the radial bin of the current point
          /* int rind = (int)((r*nrad)/rad); // linear version */
          // use log-polar bins
          // http://students.ee.sun.ac.za/~riaanvdd/Tools%20-%20Fourier-Mellin%20Transform.doc

          // Use radiiQuants to allocate a bin index, with outermost bin first (for backward compatibility with old code)
          int rind;
          bool binAssigned;

          if (r > radiiQuants[nrad-1])
            {
              binAssigned = false;
            } else {
              binAssigned = true;
              for (rind = 0; rind < nrad; ++rind)
                {
                  if (r <= radiiQuants[rind]) break;
                }
              //bin index needs to be reversed, so outermost bin is first (for backward compatibility with old code)
              rind = (nrad-1) - rind;
            }

          // Calculate the angular bin of the current point
          // taking mod of nang as atan2 returns in range (-PI,PI] so ang is in range (0,2PI]
          int aind = (int)((ang*nang)/(2*M_PI)) % nang;

          // Trying to fit a square peg in a round hole...
          if (binAssigned == false) {
              (*imask)[y*ssd_sz + x] = -1;
            } else {
              (*imask)[y*ssd_sz + x] = aind*nrad + rind;
            }
        }
    }
}

/**
 * Amazingly, some 50% of the time is spent doing expf.
 * We can probably vectorize this away somehow.
 */
template<class fnumtype> inline fnumtype ssdesc::details::fast_exp(const fnumtype x)
{
  return (fnumtype)exp(x);
}

template<> inline float ssdesc::details::fast_exp(const float x)
{
  return expf(x);
}

/**
 * Takes the sum-of-square differences surface and accumulates them
 * into a self-similarity descriptor.
 */
template<class fnumtype> void ssdesc::details::ssdesc_descriptor(const vector<fnumtype>& ssd, const vector<int>& imask, const int ssd_sz, const int nrad, const int nang,
                                                                 const fnumtype var_noise, vector<fnumtype>* ssdescs, const int offset)
{
  // get a pointer to the current descriptor in ssdescs
  fnumtype* ssdesc = &(*ssdescs)[offset];

  std::fill(ssdesc, ssdesc + (nrad*nang-1), std::numeric_limits<fnumtype>::min());

  // Iterate through all points in the ssd surface
  for (int i = 0; i < ssd_sz*ssd_sz; ++i)
    {
      if (imask[i]==-1) continue; // Skip this point if it is not part of the descriptor
      // I'm guessing the var_autoq can somehow be got from ssd.
      // It's a constant for now.
      fnumtype var_autoq = fnumtype(0.0);
      // 50% of the runtime is spent doing the exp. I can't really see
      // that it's necessary - maybe 1/(1+k*x) is fine?
      fnumtype val = ssdesc::details::fast_exp( - ssd[i] / std::max(var_noise, var_autoq));

      // Take the maximum ssd in each bin for the descriptor
      ssdesc[imask[i]] = std::max(ssdesc[imask[i]], val);
    }
}

/**
 * Computes the sum-of-square differences surface for a corelation
 * window and patch location.
 *
 * Params
 * ------
 * INPUTS:
 * img : fnumtype [width*height]
 *    The image
 * width : int
 *    Image width
 * height : int
 *    Image height
 * xl,yl : int
 *    Top left corner of the corelation window
 * xr,yr : int
 *    Bottom right corner of the corelation window *minus* the
 *    patch size.
 * xp,yp : int
 *    Top left corner of the patch
 * ssd_sz : int
 *    Size (width/height) of the correlation patch to compute
 * sz : int
 *    Internal correlation patch size
 * OUTPUTS:
 * ssd_surf : vector<fnumtype>* [(co_sz - sz + 1)^2]
 *    Output
 */
template<class fnumtype> void ssdesc::details::ssd_compute(const fnumtype* img, const int width, const int height, const int nchan,const int xl, const int xr,
                                                           const int yl, const int yr, const int xp, const int yp, const int ssd_sz, const int sz, vector<fnumtype>* ssd_surf)
{
  assert((xr-xl) == ssd_sz);
  assert((yr-yl) == ssd_sz);

  vector<fnumtype>(ssd_sz*ssd_sz).swap(*ssd_surf);

  typename vector<fnumtype>::iterator ssd_surf_it = ssd_surf->begin();

  // x and y iterate across the correlation window
  for (int y = yl; y < yr; ++y)
    {
      for (int x = xl; x < xr; ++x)
        {
          fnumtype ssd = fnumtype(0.0);
          // xc and yc give offset within the inner patch
          for (int xc = 0; xc < sz; ++xc)
            {
              for (int yc = 0; yc < sz; ++yc)
                {
                  for (int n = 0; n < nchan; ++n)
                    {
                      ssd += (img[(y  + yc) + ((x  + xc) + n*width)*height] -
                          img[(yp + yc) + ((xp + xc) + n*width)*height])*
                          (img[(y  + yc) + ((x  + xc) + n*width)*height] -
                          img[(yp + yc) + ((xp + xc) + n*width)*height]);
                    }
                }
            }
          // ssd_surf contains ssd's in x-dir first, then y
          (*ssd_surf_it) = ssd;
          ++ssd_surf_it;
        }
    }
}

/**
 * Computes a sum-of-square differences surface incrementally
 * from the previous one calculated one pixel to the left.
 */
template<class fnumtype> inline void ssdesc::details::ssd_compute_irow(const fnumtype* img, const int width, const int height, const int nchan,
								       const int xl, const int xr, const int yl, const int yr, const int xp, const int yp, const int sz,
								       const vector<fnumtype>& ssd_surf_in, vector<fnumtype>* ssd_surf_out)
{
  assert(ssd_surf_in.size() == (unsigned int)(yr-yl)*(xr-xl));
  assert(ssd_surf_out->size() == (unsigned int)(yr-yl)*(xr-xl));

  typename vector<fnumtype>::const_iterator ssd_surf_in_it = ssd_surf_in.begin();
  typename vector<fnumtype>::iterator ssd_surf_out_it = ssd_surf_out->begin();

  // x and y iterate across the correlation window
  for (int y = yl; y < yr; ++y)
    {
      for (int x = xl; x < xr; ++x)
        {
          // start with the input ssd patch (from one pixel to the left)
          (*ssd_surf_out_it) = (*ssd_surf_in_it);
          // yc gives vertical offset within the inner patch
          for (int yc = 0; yc < sz; ++yc)
            {
              for (int n = 0; n < nchan; ++n)
                {
                  // Subtract the previous column (xc is a constant -1)
                  (*ssd_surf_out_it) -= (img[(y  + yc) + ((x  - 1) + n*width)*height] -
                      img[(yp + yc) + ((xp - 1) + n*width)*height])*
                      (img[(y  + yc) + ((x  - 1) + n*width)*height] -
                      img[(yp + yc) + ((xp - 1) + n*width)*height]);
                }
            }
          for (int yc = 0; yc < sz; ++yc)
            {
              for (int n = 0; n < nchan; ++n)
                {
                  // Add the new column (xc is a constant sz-1)
                  (*ssd_surf_out_it) += (img[(y  + yc) + ((x  + sz - 1) + n*width)*height] -
                      img[(yp + yc) + ((xp + sz - 1) + n*width)*height])*
                      (img[(y  + yc) + ((x  + sz - 1) + n*width)*height] -
                      img[(yp + yc) + ((xp + sz - 1) + n*width)*height]);
                }
            }
          ++ssd_surf_in_it;
          ++ssd_surf_out_it;
        }
    }
}

/**
 * Calculates the ssd between two self similarity descriptors
 * using Euclidean Distance.
 */
template<class fnumtype> fnumtype ssdesc::details::calc_ssd_ssdesc(const fnumtype* ssdesc1, const fnumtype* ssdesc2, const int ssdesc_size)
{
  fnumtype result = 0;
  for (int i=0; i<ssdesc_size; ++i)
    {
      result = result + (ssdesc1[i] - ssdesc2[i])*(ssdesc1[i] - ssdesc2[i]);
    }
  return result;
}

/**
 * Calculates the ssd between two self similarity descriptors
 * using Euclidean Distance if within the two smallest ssd,
 * else returns -1
 *
 * N.B. boundMultiplier is used to calculate the bound from the
 * smallest poss value of 2-norm for a given 1-norm, and should
 * be equal to sqrt(ssdesc_size). By default, this is calculated
 * within the routine, but this can be precomputed and passed
 * to the routine to speed things up if it is going to be called
 * multiple times
 */
template<class fnumtype> double ssdesc::details::calc_ssd_ssdesc_min2(const fnumtype* ssdesc1, const fnumtype* ssdesc2, const int ssdesc_size,
                                                                      fnumtype &ssd_min_bound1, fnumtype &ssd_min_bound2, const fnumtype boundMultiplier)
{
  fnumtype bm;

  if (boundMultiplier == -1)
    {
      bm = sqrt((fnumtype)ssdesc_size);
    } else
    {
      bm = boundMultiplier;
    }

  double one_norm = 0;
  for (int i=0; i<ssdesc_size; ++i)
    {
      one_norm = one_norm + abs((double)(ssdesc1[i] - ssdesc2[i]));
    }
  // only calculate the 2-norm if the 1-norm is such it will be within the minimum 2
  if (one_norm <= ssd_min_bound2)
    {
      double two_norm = 0;
      for (int i=0; i<ssdesc_size; ++i)
        {
          two_norm = two_norm + (double)(ssdesc1[i] - ssdesc2[i])*(ssdesc1[i] - ssdesc2[i]);
        }
      two_norm = sqrt(two_norm);
      //calculate the bound for the calculated 2-norm
      fnumtype two_norm_bound = two_norm*bm;
      //update the minimum two bounds
      if ((two_norm_bound < ssd_min_bound2) && (two_norm_bound >= ssd_min_bound1))
        {
          ssd_min_bound2 = two_norm_bound;
        }
      if (two_norm_bound < ssd_min_bound1)
        {
          ssd_min_bound2 = ssd_min_bound1;
          ssd_min_bound1 = two_norm_bound;
        }

      //NOTE: will return even if the norm is EQUAL to the second minimum value
      return two_norm;
    } else {
      return -1;
    }
}

//declare used template types for interface functions#
template void ssdesc::calc_ssdescs<double>(const double* image, const int image_width,
const int image_height, const int image_channels, const ssdesc::ssdesc_parms<double> &parms,
vector<double>* ssdescs, const ssdesc::rectRegion calc_rect);

template void ssdesc::calc_ssdescs_alt<double>(const double* image, const int image_width,
const int image_height, const int image_channels, const ssdesc::ssdesc_parms<double> &parms,
vector<double>* ssdescs, const ssdesc::rectRegion calc_rect);

template void ssdesc::prune_normalise_ssdescs<double>(vector<double>& ssdescs,
const int image_width, const int image_height, const ssdesc::ssdesc_parms<double> &parms,
vector<double>* resp, vector<ssdesc::coordElem>* draw_coords, vector<ssdesc::coordElem>* salient_coords,
vector<ssdesc::coordElem>* homogeneous_coords, vector<ssdesc::coordElem>* snn_coords, const ssdesc::rectRegion calc_rect);


template void ssdesc::calc_ssdescs<float>(const float* image, const int image_width,
const int image_height, const int image_channels, const ssdesc::ssdesc_parms<float> &parms,
vector<float>* ssdescs, const ssdesc::rectRegion calc_rect);

template void ssdesc::calc_ssdescs_alt<float>(const float* image, const int image_width,
const int image_height, const int image_channels, const ssdesc::ssdesc_parms<float> &parms,
vector<float>* ssdescs, const ssdesc::rectRegion calc_rect);

template void ssdesc::prune_normalise_ssdescs<float>(vector<float>& ssdescs,
const int image_width, const int image_height, const ssdesc::ssdesc_parms<float> &parms,
vector<float>* resp, vector<ssdesc::coordElem>* draw_coords, vector<ssdesc::coordElem>* salient_coords,
vector<ssdesc::coordElem>* homogeneous_coords, vector<ssdesc::coordElem>* snn_coords, const ssdesc::rectRegion calc_rect);
