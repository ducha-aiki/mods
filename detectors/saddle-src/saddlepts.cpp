/*
 * saddlepts.cpp
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

//int main( int argc, char** argv )
//{
//	AnyOption *opt = new AnyOption();
//	char *imgpath, *outpath;
//	int epsilon, nlevels, edgeThreshold, doNMS, descSize, nfeatures;
//	bool showVis, savefile;
//	double responseThr, scaleFactor;
//	uchar deltaThr;

//	bool saveDeltas = true;
//	char deltaoutpath[] = "./outputs/deltas.txt";
//	char* ptrdeltapath = deltaoutpath;

//	parse_opt( argc, argv, opt );


//	if( opt->getValue( 'i' ) != NULL  || opt->getValue( "image" ) != NULL  )
//		imgpath = opt->getValue( 'i' );
//	else
//	{
//		cout << "No input image path" << endl;
//		return -1;
//	}
//	if( opt->getValue( 'o' ) != NULL  || opt->getValue( "output" ) != NULL  )
//	{
//		outpath = opt->getValue( 'o' );
//		savefile = true;
//	}
//	else
//	{
//		cout << "Warning: Path for output file was not provided" << endl;
//		savefile = false;
//	}
//	if( opt->getValue( 't' ) != NULL  || opt->getValue( "thres" ) != NULL  )
//		responseThr = atof(opt->getValue( 't' ));
//	else
//		responseThr = 0.0;

//	if( opt->getValue( 'e' ) != NULL  || opt->getValue( "epsilon" ) != NULL  )
//		epsilon = atoi(opt->getValue( 'e' ));
//	else
//		epsilon = 1;

//	if( opt->getValue( 'l' ) != NULL  || opt->getValue( "levels" ) != NULL  )
//		nlevels = atoi(opt->getValue( 'l' ));
//	else
//		nlevels = 8;

//	if( opt->getValue( 'g' ) != NULL  || opt->getValue( "gab" ) != NULL  )
//		edgeThreshold = atoi(opt->getValue( 'g' ));
//	else
//		edgeThreshold = 3;//31

//	if( opt->getValue( 's' ) != NULL  || opt->getValue( "scalefac" ) != NULL  )
//		scaleFactor = atof(opt->getValue( 's' ));
//	else
//		scaleFactor = 1.3;

//	if( opt->getValue( 'n' ) || opt->getValue( "nms" ) )
//		doNMS = atoi(opt->getValue( 'n' ));
//	else
//		doNMS = 2;

//	if( opt->getValue( 'w' ) || opt->getValue( "word" ) )
//		descSize = atoi(opt->getValue( 'w' ));
//	else
//		descSize = 32;

//	if( opt->getValue( 'd' ) || opt->getValue( "delta" ) )
//		deltaThr = atoi(opt->getValue( 'd' ));
//	else
//		deltaThr = 0;

//	if( opt->getValue( 'f' ) || opt->getValue( "feats" ) )
//			nfeatures = atoi(opt->getValue( 'f' ));
//		else
//			nfeatures = 5000;

//	if( opt->getFlag( 'v' ) || opt->getFlag( "visu" ) )
//		showVis = opt->getFlag( 'v' );
//	else
//		showVis = false;
//	delete opt;


//	// Loading image
//	Mat img = imread( imgpath , IMREAD_GRAYSCALE );
//	if (img.empty())
//	{
//		readme();
//		return -1;
//	}

//	printf("SADDLE detector parameters: \n   nLevels: %d, scaleFactor: %.1f, epsilon: %d, responseThr: %.2f, borderGab: %d, doNMS: %d, deltaThr: %d, nFeats: %d\n",
//			nlevels, scaleFactor, epsilon, responseThr, edgeThreshold, doNMS, deltaThr, nfeatures);
//	cmp::SORB detector(responseThr, scaleFactor, nlevels, edgeThreshold, epsilon, 2, cmp::SORB::SUMOFABS_SCORE , 31, doNMS, descSize, deltaThr, nfeatures);

//	vector<cmp::SadKeyPoint> kpts;
//	Mat dcts, mask;

//	printf("Detecting SADDLE points... \n");
//	detector( img, mask, kpts, dcts, false );
//	printf("Total number of features %d\n\n", (int)kpts.size());

//	if (savefile)
//		if (!txt_from_feats( kpts, dcts, outpath))
//			return -1;

//	if (saveDeltas)
//		deltas_to_txt( kpts, ptrdeltapath );


//	if (showVis)
//	{
//		Mat img_feats;
//		vector<cv::KeyPoint> kptsShow;
//		kptsShow.resize(kpts.size());
//		for( size_t i = 0; i < kpts.size(); i++ )
//			kptsShow[i] = kpts[i];
//		drawKeypoints(img, kptsShow, img_feats, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//		namedWindow("Saddle features", cv::WINDOW_NORMAL);
//		imshow( "Saddle features", img_feats );
//		waitKey(0);
//	}

//	return 0;
//}

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
	opt->addUsage( " -v  --visu  		Flag for visualizing the image features " );
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
	opt->setFlag(    "visu", 	'v' );

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
