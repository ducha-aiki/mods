/*
 *	SELF-SIMILARITY DESCRIPTOR
 *	MATLAB INTERFACE
 *	v 1.1
 *
 *	Ken Chatfield
 *	Engineering Department
 *	University of Oxford
 *
 *	Date: March 2009
 *	Updated March 2010
 *
 *	MATLAB MEX wrapper for the C++ implementation of the self-similarity descriptor
 *	defined in ssdesc.h
 *
 *	For usage instructions see readme.txt
 *
 */


#define _USE_MATH_DEFINES

#include "mex.h"

#include <algorithm>
#include <limits>
#include <vector>
#include <list>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

//helper functions
#include "ssdesc.h"

using namespace std;

//------------------------------------------------------------------------------------
// MATLAB Function
//------------------------------------------------------------------------------------

void usagerr(char *msg)
{
	mexPrintf("%s",msg);
	mexErrMsgTxt("Usage: [resp,drawCoords,salientCoords,homogeneousCoords,snnCoords]=mexCalcSsdescs(img,parms);\n");
	mexErrMsgTxt("    or [resp,drawCoords,salientCoords,homogeneousCoords,snnCoords]=mexCalcSsdescs(img,parms,calcRect);\n");
	mexErrMsgTxt("    or [resp,drawCoords,salientCoords,homogeneousCoords,snnCoords]=mexCalcSsdescs(img,parms,point);\n");
}

void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{

	if ((nrhs != 2) && (nrhs != 3)) usagerr("Either two or three input arguments are required\n");
	if (mxGetClassID(prhs[0])!= mxDOUBLE_CLASS) usagerr("Image must be of type double. Create using double(imread('image name'))\n");

	double *img, *rect;
	int img_width, img_height, img_nchan, rect_m, rect_n;
	mxArray *loader;
	ssdesc::ssdesc_parms<double> parms;

	/*** RETRIEVE AND CHECK DATA ***/

	/* Load the image */
	img = mxGetPr(prhs[0]);

	/* Get the number of channels in the image */
	int img_dimcount;
	img_dimcount = mxGetNumberOfDimensions(prhs[0]);

	if ((img_dimcount < 2) || (img_dimcount > 3))
	{
		usagerr("Field 'img' must be a greyscale (2D) or colour (3D) matrix\n");
	}

	if (img_dimcount == 2)
	{
		img_height = mxGetM(prhs[0]);
		img_width = mxGetN(prhs[0]);
		img_nchan = 1;
	} else {
		const int* img_dimsizes = mxGetDimensions(prhs[0]);
		img_height = img_dimsizes[0];
		img_width = img_dimsizes[1];
		img_nchan = img_dimsizes[2];
	}

	/* Load individual parameters of parms structure */
	loader = mxGetField(prhs[1],0,"patch_size");
	if (loader == NULL) usagerr("Field 'patch_size' could not be found in parms array\n");
	parms.patch_size = (unsigned short int)mxGetScalar(loader);
	if ((parms.patch_size%2)!=1) usagerr("Field 'patch_size' must be odd\n");

	loader = mxGetField(prhs[1],0,"desc_rad");
	if (loader == NULL) usagerr("Field 'desc_rad' could not be found in parms array\n");
	unsigned short int desc_rad = (unsigned short int)mxGetScalar(loader);
	/* Convert the descriptor radius to the size of the correlation window
	  NOTE 1: must leave a margin of (parms.patch_size-1)/2 around the outside
	  of the correlation patch for when the inner patch is placed at the
	  correlation patch boundary
	  NOTE 2: internally, desc_rad is not used in the computation of the
	  ssdesc at all, instead with all values being derived from cor_size */
	parms.cor_size = (unsigned short int)(desc_rad*2 + parms.patch_size);

	loader = mxGetField(prhs[1],0,"nrad");
	if (loader == NULL) usagerr("Field 'nrad' could not be found in parms array\n");
	parms.nrad = (unsigned short int)mxGetScalar(loader);

	loader = mxGetField(prhs[1],0,"nang");
	if (loader == NULL) usagerr("Field 'nang' could not be found in parms array\n");
	parms.nang = (unsigned short int)mxGetScalar(loader);

	loader = mxGetField(prhs[1],0,"var_noise");
	if (loader == NULL) usagerr("Field 'var_noise' could not be found in parms array\n");
	parms.var_noise = mxGetScalar(loader);

	loader = mxGetField(prhs[1],0,"saliency_thresh");
	if (loader == NULL) usagerr("Field 'saliency_thresh' could not be found in parms array\n");
	parms.saliency_thresh = mxGetScalar(loader);
	if ((parms.saliency_thresh > 1) || (parms.saliency_thresh < 0)) usagerr("'parms.saliency_thresh' must be between 0 and 1\n");

	loader = mxGetField(prhs[1],0,"homogeneity_thresh");
	if (loader == NULL) usagerr("Field 'homogeneity_thresh' could not be found in parms array\n");
	parms.homogeneity_thresh = mxGetScalar(loader);
	if ((parms.homogeneity_thresh > 1) || (parms.homogeneity_thresh < 0)) usagerr("'parms.homogeneity_thresh' must be between 0 and 1\n");

	loader = mxGetField(prhs[1],0,"snn_thresh");
	if (loader == NULL) usagerr("Field 'snn_thresh' could not be found in parms array\n");
	parms.snn_thresh = mxGetScalar(loader);
	if ((parms.snn_thresh > 1) || (parms.snn_thresh < 0)) usagerr("'parms.snn_thresh' must be between 0 and 1\n");

	/* Load rect/coordinates if specified */
	ssdesc::rectRegion calcRect;

	if (nrhs == 3)
	{
		rect = mxGetPr(prhs[2]);
		rect_m = mxGetM(prhs[2]);
		rect_n = mxGetN(prhs[2]);

		int max_dim = max(rect_m, rect_n);
		int min_dim = min(rect_m, rect_n);

		if (min_dim != 1) usagerr("Field 'rect'/'point' must be a 1D vector\n");
		if ((max_dim == 2) || (max_dim == 4))
		{
			// check ranges
			if (max_dim == 2)
			{
				// remember to convert from MATLAB style 1-indexing to C style 0-indexing
				calcRect.xfrom = (int)rect[0] - 1;
				calcRect.yfrom = (int)rect[1] - 1;
				calcRect.xto = (int)rect[0] - 1;
				calcRect.yto = (int)rect[1] - 1;

				if ((calcRect.xfrom < ((parms.cor_size-1)/2)) || (calcRect.yfrom < ((parms.cor_size-1)/2)) || (calcRect.xfrom > (img_width - 1 - (parms.cor_size - 1)/2)) || (calcRect.yfrom > (img_height - 1 - (parms.cor_size - 1)/2)))
					usagerr("Field 'point' is outside of allowable margin of (parms.patch_size-1)/2 + parms.desc_rad around the image\n");
			}
			if (max_dim == 4)
			{
				calcRect.xfrom = (int)rect[0] - 1;
				calcRect.yfrom = (int)rect[1] - 1;
				calcRect.xto = (int)rect[2] - 1;
				calcRect.yto = (int)rect[3] - 1;

				if ((calcRect.xfrom < ((parms.cor_size-1)/2)) || (calcRect.yfrom < ((parms.cor_size-1)/2)) || (calcRect.xto > (img_width - 1 - (parms.cor_size - 1)/2)) || (calcRect.yto > (img_height - 1 - (parms.cor_size - 1)/2)))
					usagerr("Field 'rect' specifies a region outside of allowable margin of (parms.patch_size-1)/2 + parms.desc_rad around the image\n");
			}
		} else {
			usagerr("Field 'rect'/'point' must be a vector of length either 4 or 2 respectively\n");
		}

	}

	/*** CALCULATE THE SELF-SIMILARITY DESCRIPTORS ***/

	vector<double> ssdescs;
	vector<double> resp;
	vector<ssdesc::coordElem> drawCoords, salientCoords, homogeneousCoords, snnCoords;

	ssdesc::calc_ssdescs_alt<double>(img, img_width, img_height, img_nchan, parms, &ssdescs, calcRect);
	ssdesc::prune_normalise_ssdescs<double>(ssdescs, img_width, img_height, parms, &resp, &drawCoords, &salientCoords, &homogeneousCoords, &snnCoords, calcRect);

	vector<double>().swap(ssdescs);

	/*** CONVERT C++ VECTORS TO MATLAB MATRICES FOR OUTPUT ***/
	double *resp_out, *drawCoords_out, *salientCoords_out, *homogeneousCoords_out, *snnCoords_out;
	plhs[0] = mxCreateDoubleMatrix(parms.nrad*parms.nang, (int)resp.size()/(parms.nrad*parms.nang), mxREAL); //mxReal is our data-type
	resp_out = mxGetPr(plhs[0]);

	plhs[1] = mxCreateDoubleMatrix(2, drawCoords.size(), mxREAL);
	drawCoords_out = mxGetPr(plhs[1]);

	plhs[2] = mxCreateDoubleMatrix(2, salientCoords.size(), mxREAL);
	salientCoords_out = mxGetPr(plhs[2]);

	plhs[3] = mxCreateDoubleMatrix(2, homogeneousCoords.size(), mxREAL);
	homogeneousCoords_out = mxGetPr(plhs[3]);

	plhs[4] = mxCreateDoubleMatrix(2, snnCoords.size(), mxREAL);
	snnCoords_out = mxGetPr(plhs[4]);

	for (int i=0; i<(int)resp.size(); ++i)
	{
		resp_out[i] = resp[i];
	}
	vector<double>().swap(resp);
	for (int i=0; i<(int)drawCoords.size(); ++i)
	{
		// remember to add offset of 1 to convert from C 0-indexing to MATLAB 1-indexing
		drawCoords_out[i*2] = drawCoords[i].x + 1;
		drawCoords_out[i*2+1] = drawCoords[i].y + 1;
	}
	vector<ssdesc::coordElem>().swap(drawCoords);
	for (int i=0; i<(int)salientCoords.size(); ++i)
	{
		salientCoords_out[i*2] = salientCoords[i].x + 1;
		salientCoords_out[i*2+1] = salientCoords[i].y + 1;
	}
	vector<ssdesc::coordElem>().swap(salientCoords);
	for (int i=0; i<(int)homogeneousCoords.size(); ++i)
	{
		homogeneousCoords_out[i*2] = homogeneousCoords[i].x + 1;
		homogeneousCoords_out[i*2+1] = homogeneousCoords[i].y + 1;
	}
	vector<ssdesc::coordElem>().swap(homogeneousCoords);
	for (int i=0; i<(int)snnCoords.size(); ++i)
	{
		snnCoords_out[i*2] = snnCoords[i].x + 1;
		snnCoords_out[i*2+1] = snnCoords[i].y + 1;
	}
	vector<ssdesc::coordElem>().swap(snnCoords);
}
