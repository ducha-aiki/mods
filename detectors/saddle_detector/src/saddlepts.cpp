/*
 * saddlept.cpp
 * Detect SADDLE point in images
 *
 *  Created on: Sep 23, 2015
 *      Author: Javier Aldana-Iuit
 */

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "anyoption.h"
#include "nms.cpp"
#include "sorb.h"


using namespace cv;
void parse_opt( int argc, char* argv[], AnyOption * opt );
void readme();
int txt_from_feats(vector<cmp::SadKeyPoint> kpts, Mat dcts, char *outpath);
int deltas_to_txt(vector<cmp::SadKeyPoint> kpts, char *outpath);
float getScale(int level, int firstLevel, double scaleFactor);
void image_pyramid_building(Mat image, vector<Mat>& pyrAdjacent, vector<Mat>& pyrOrigin, int levelsNum, int firstLevel, double scaleFactor, int edgeThreshold, int patchSize );
void substract_images( Mat A, Mat B, Mat& imDif );


int main( int argc, char** argv )
{
	AnyOption *opt = new AnyOption();
	char *imgpath, *outpath;
	int epsilon, nlevels, edgeThreshold, doNMS, descSize, nfeatures, subPixPrecision, innerTstType;
	bool showVis, savefile, allC1feats, strictMaximum, gravityCenter;
	double responseThr, scaleFactor;
	uchar deltaThr;


	bool saveDeltas = true;
	char deltaoutpath[] = "./outputs/deltas.txt";
	char* ptrdeltapath = deltaoutpath;

	parse_opt( argc, argv, opt );


	if( opt->getValue( 'i' ) != NULL  || opt->getValue( "image" ) != NULL  )
		imgpath = opt->getValue( 'i' );
	else
	{
		cout << "No input image path" << endl;
		return -1;
	}
	if( opt->getValue( 'o' ) != NULL  || opt->getValue( "output" ) != NULL  )
	{
		outpath = opt->getValue( 'o' );
		savefile = true;
	}
	else
	{
		cout << "Warning: Path for output file was not provided" << endl;
		savefile = false;
	}
	if( opt->getValue( 't' ) != NULL  || opt->getValue( "thres" ) != NULL  )
		responseThr = atof(opt->getValue( 't' ));
	else
		responseThr = 0.0;

	if( opt->getValue( 'e' ) != NULL  || opt->getValue( "epsilon" ) != NULL  )
		epsilon = atoi(opt->getValue( 'e' ));
	else
		epsilon = 1;

	if( opt->getValue( 'l' ) != NULL  || opt->getValue( "levels" ) != NULL  )
		nlevels = atoi(opt->getValue( 'l' ));
	else
		nlevels = 8;

	if( opt->getValue( 'g' ) != NULL  || opt->getValue( "gab" ) != NULL  )
		edgeThreshold = atoi(opt->getValue( 'g' ));
	else
		edgeThreshold = 3;//31

	if( opt->getValue( 's' ) != NULL  || opt->getValue( "scalefac" ) != NULL  )
		scaleFactor = atof(opt->getValue( 's' ));
	else
		scaleFactor = 1.3;

	if( opt->getValue( 'n' ) || opt->getValue( "nms" ) )
		doNMS = atoi(opt->getValue( 'n' ));
	else
		doNMS = 2;

	if( opt->getValue( 'w' ) || opt->getValue( "word" ) )
		descSize = atoi(opt->getValue( 'w' ));
	else
		descSize = 32;

	if( opt->getValue( 'd' ) || opt->getValue( "delta" ) )
		deltaThr = atoi(opt->getValue( 'd' ));
	else
		deltaThr = 0;

	if( opt->getValue( 'f' ) || opt->getValue( "feats" ) )
			nfeatures = atoi(opt->getValue( 'f' ));
		else
			nfeatures = 5000;

	if( opt->getFlag( 'v' ) || opt->getFlag( "visu" ) )
		showVis = opt->getFlag( 'v' );
	else
		showVis = false;

	if( opt->getFlag( 'c' ) || opt->getFlag( "c1feat" ) )
		allC1feats = opt->getFlag( 'c' );
	else
		allC1feats = false;

	if( opt->getFlag( 'm' ) || opt->getFlag( "maxstrict" ) )
		strictMaximum = opt->getFlag( 'm' );
	else
		strictMaximum = false;

	if( opt->getFlag( 'r' ) || opt->getFlag( "gravity" ) )
		gravityCenter = opt->getFlag( 'r' );
	else
		gravityCenter = false;

	if( opt->getValue( 'p' ) || opt->getValue( "subpix" ) )
		subPixPrecision = atoi(opt->getValue( 'p' ));
	else
		subPixPrecision = 0;

	if( opt->getValue( 'x' ) || opt->getValue( "innertype" ) )
		innerTstType = atoi(opt->getValue( 'x' ));
	else
		innerTstType = 0;

	delete opt;


	// Loading image
	Mat img = imread( imgpath , IMREAD_GRAYSCALE );
	if (img.empty())
	{
		readme();
		return -1;
	}

	printf("SADDLE detector parameters: \n   nLevels: %d, scaleFactor: %.1f, epsilon: %d, responseThr: %.2f, borderGab: %d, doNMS: %d, deltaThr: %d, nFeats: %d, allC1features: %d, strictMaxNMS: %d, subpixelMethod: %d, C1C2gravityCenter: %d \n",
			nlevels, scaleFactor, epsilon, responseThr, edgeThreshold, doNMS, deltaThr, nfeatures, allC1feats, strictMaximum, subPixPrecision, gravityCenter);

	cmp::SORB detector(responseThr, scaleFactor, nlevels, edgeThreshold, epsilon, 2, cmp::SORB::DELTA_SCORE , 31,
						doNMS, descSize, deltaThr, nfeatures, allC1feats, strictMaximum, subPixPrecision, gravityCenter, innerTstType);

	vector<cmp::SadKeyPoint> kpts;
	Mat dcts, mask;

	printf("Detecting SADDLE points... \n");
	detector( img, mask, kpts, dcts, false );

	int num_kpts = (int)kpts.size();
	printf("Total number of features %d\n\n", num_kpts);

	if (savefile)
		if (!txt_from_feats( kpts, dcts, outpath))
			return -1;

	if (saveDeltas)
		deltas_to_txt( kpts, ptrdeltapath );

	// ------------------------- Test with pyramids ----------------------------- //
#if false
	vector<Mat> pyrAdjacent(nlevels), pyrOrigin(nlevels);
	image_pyramid_building(img, pyrAdjacent, pyrOrigin, nlevels, 0, scaleFactor, edgeThreshold, 31 );

	for (int i=0; i<nlevels; i++)
	{
		ostringstream ss;
		ss << i;
		String mytitle = ("Scale " + ss.str() );

		Mat dif, normdif;
		substract_images( pyrAdjacent[i], pyrOrigin[i], dif );
		normalize(dif, normdif, 0, 255, NORM_MINMAX);

		namedWindow(mytitle, cv::WINDOW_NORMAL);
		imshow( mytitle, normdif );
	}
#endif
	// -------------------------------------------------------------------------- //


	if (showVis)
	{
		Mat img_feats;
		vector<cv::KeyPoint> kptsShow;
		kptsShow.resize(kpts.size());
		for( size_t i = 0; i < kpts.size(); i++ )
			kptsShow[i] = kpts[i];
		drawKeypoints(img, kptsShow, img_feats, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		namedWindow("Saddle features", cv::WINDOW_NORMAL);
		imshow( "Saddle features", img_feats );
		waitKey(0);
	}
	return 0;
}

void parse_opt( int argc, char* argv[], AnyOption * opt )
{
	/* USAGE/HELP   */
	opt->addUsage( "" );
	opt->addUsage( "Usage: " );
	opt->addUsage( "" );
	opt->addUsage( " -h  --help  		Displays usage of the function " );
	opt->addUsage( " -i  --image  		Input image to be processed " );
	opt->addUsage( " -o  --output 		Full path of the text file with the features " );
	opt->addUsage( " -t  --thres		Hessian response threshold not squared (Default 0.0) ");
	opt->addUsage( " -e  --epsilon  	Intensity difference between darker and brighter pixels (Default 1) " );
	opt->addUsage( " -l  --levels  		Number of levels in the image pyramid (Default 8) " );
	opt->addUsage( " -g  --gab  		Gab in the border of the image for computing feats and descriptors (Default 3) " );
	opt->addUsage( " -s  --scalefac		Scale factor from one level to the next one (Default 1.3) " );
	opt->addUsage( " -n  --nms  		Option for non maximum suppression (0) without, (1) levelwise only, (2) 3D nms (Default 2) " );
	opt->addUsage( " -f  --feats  		Maximum number of features (if NMS=0 then all detections will be passed out) " );
	opt->addUsage( " -d  --delta  		Threshold for minimum delta allowed in the regions " );
	opt->addUsage( " -w  --word  		Length of the descriptor in bytes, i.e. number of dimension of BRIEF descriptor " );
	opt->addUsage( " -p  --subpix  		Subpixel precision. (0) Integer precision, (1) weighted average 3x3 neighbors, (2) quadratic interp." );
	opt->addUsage( " -x  --innertype  	Inner circle test type. (0) baseline, (1) extension positions only, (2) extension sum, (3) extension avg., (4) extension sqrt 2." );
	opt->addUsage( " -v  --visu  		Flag for visualizing the image features " );
	opt->addUsage( " -c  --c1feat		Flag to pass all features that fulfill inner circle condition. Feature score is contrast (delta)" );
	opt->addUsage( " -m  --maxstrict  	Flag keeping response extremas only, without it the tides are allowed and passed all." );
	opt->addUsage( " -r  --gravity  	Flag for features that fulfill inner and outer circle condition, 8-conn components are single feat. its pos. is gravity center." );
	opt->addUsage( "" );

    /* OPTIONS  */
	opt->setFlag(  	 "help",	'h' );
	opt->setOption(  "image",	'i' );
	opt->setOption(  "output",	'o' );
	opt->setOption(  "thres",	't' );
	opt->setOption(  "epsilon", 'e' );
	opt->setOption(  "levels",  'l' );
	opt->setOption(  "gab", 	'g' );
	opt->setOption(  "scalefac",'s' );
	opt->setOption(  "nms", 	'n' );
	opt->setOption(  "feats", 	'f' );
	opt->setOption(  "delta", 	'd' );
	opt->setOption(  "word", 	'w' );
	opt->setOption(  "subpix", 	'p' );
	opt->setOption(  "innertype", 	'x' );
	opt->setFlag(    "visu", 	'v' );
	opt->setFlag(    "c1feat", 	'c' );
	opt->setFlag(    "maxstrict", 	'm' );
	opt->setFlag(    "gravity", 	'r' );

    /* go through the command line and get the options  */
    opt->processCommandArgs( argc, argv );

	if( ! opt->hasOptions())
	{ /* print usage if no options */
		opt->printUsage();
	    delete opt;
		return;
	}

	/* GET THE VALUES */
	if( opt->getFlag( "help" ) || opt->getFlag( 'h' ) )
		opt->printUsage();

}

void readme()
{
	std::cout << " Image not found" << std::endl;
}

int txt_from_feats(vector<cmp::SadKeyPoint> kpts, Mat dcts, char *outpath)
{

	FILE * ftfile;
	ftfile = fopen (outpath, "w+");
	if (!ftfile)
	{
		cout << "Could not create the file: " << outpath << endl;
		return -1;
   	}

	// Header and number of features
	int num_feats = kpts.size(), iDesc = 0;
	float coef;
	uchar* p;
	fprintf(ftfile, "1.0\n%d\n", num_feats);

	for (vector<cmp::SadKeyPoint>::iterator kp = kpts.begin(), kpEnd = kpts.end(); kp != kpEnd; ++kp, iDesc++)
	{
		coef = 1/(5.1962*kp->size);
		coef *= coef;
		fprintf(ftfile, "%.2f %.3f %f 0.0 %f ", kp->pt.x, kp->pt.y, coef, coef);
		p = dcts.ptr<uchar>(iDesc);

		for (int iCol=0; iCol<dcts.cols; iCol++ )
			fprintf(ftfile, "%d ", p[iCol]);
		fprintf(ftfile, "\n");
	}

	fclose(ftfile);
	return 1;
}

int deltas_to_txt(vector<cmp::SadKeyPoint> kpts, char *outpath)
{

	FILE * ftfile;
	ftfile = fopen (outpath, "w+");
	if (!ftfile)
	{
		cout << "Could not create the file: " << outpath << endl;
		return -1;
   	}

	// Header and number of features
	int num_feats = kpts.size(), iDesc = 0;

	fprintf(ftfile, "%d\n", num_feats);

	for (vector<cmp::SadKeyPoint>::iterator kp = kpts.begin(), kpEnd = kpts.end(); kp != kpEnd; ++kp, iDesc++)
	{
		fprintf(ftfile, "%d\n", kp->delta );
	}

	fclose(ftfile);
	return 1;
}


float getScale(int level, int firstLevel, double scaleFactor)
{
    return (float)std::pow(scaleFactor, (double)(level - firstLevel));
}

void image_pyramid_building(Mat image, vector<Mat>& pyrAdjacent, vector<Mat>& pyrOrigin, int levelsNum, int firstLevel, double scaleFactor, int edgeThreshold, int patchSize )
{

	const int HARRIS_BLOCK_SIZE = 9;
	int halfPatchSize = patchSize / 2;
	int border = std::max(edgeThreshold, std::max(halfPatchSize, HARRIS_BLOCK_SIZE/2))+1;

	// Pre-compute the scale pyramids
	    for (int level = 0; level < levelsNum; ++level)
	    {
	        float scale = 1/getScale(level, firstLevel, scaleFactor);

	        Size sz(cvRound(image.cols*scale), cvRound(image.rows*scale));
	        Size wholeSize(sz.width + border*2, sz.height + border*2);
	        Mat temp(wholeSize, image.type());

	        pyrAdjacent[level] = temp(Rect(border, border, sz.width, sz.height));

	        resize(image, pyrOrigin[level], sz, 0, 0, INTER_LINEAR);
	        copyMakeBorder(pyrOrigin[level], pyrOrigin[level], border, border, border, border, BORDER_REFLECT_101+BORDER_ISOLATED);
	        pyrOrigin[level] = pyrOrigin[level](Rect(border, border, sz.width, sz.height));

	        // Compute the resized image
	        if( level != firstLevel )
	        {
	            if( level < firstLevel )
	            {
	                resize(image, pyrAdjacent[level], sz, 0, 0, INTER_LINEAR);
	            }
	            else
	            {
	                resize(pyrAdjacent[level-1], pyrAdjacent[level], sz, 0, 0, INTER_LINEAR);
	            }

	            copyMakeBorder(pyrAdjacent[level], temp, border, border, border, border, BORDER_REFLECT_101+BORDER_ISOLATED);
	        }
	        else
	        {
	            copyMakeBorder(image, temp, border, border, border, border, BORDER_REFLECT_101);
	        }
	    }

}

void substract_images( Mat A, Mat B, Mat& imDif )
{

	printf("Image sizes: (%d,%d) (%d,%d)\n", A.rows, A.cols, B.rows, B.cols);

	if (A.cols==B.cols && A.rows==B.rows)
	{
		cv::absdiff(A, B, imDif);
		Scalar errorSum = cv::sum(imDif);
		std::cout << "Error pixel-wise: " << errorSum.val[0] << std::endl;
	}
	else
		std::cout << "Dimension mismatch between images" << std::endl;

}
