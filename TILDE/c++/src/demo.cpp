// test.cpp --- 
// 
// Filename: test.cpp
// Description: 
// Author: Yannick Verdie, Kwang Moo Yi, Alberto Crivella
// Maintainer: Yannick Verdie
// Created: Tue Mar  3 17:47:28 2015 (+0100)
// Version: 0.5a
// Package-Requires: ()
// Last-Updated: Thu May 28 13:04:33 2015 (+0200)
//           By: Kwang
//     Update #: 25
// URL: 
// Doc URL: 
// Keywords: 
// Compatibility: 
// 
// 

// Commentary: 
// 
// 
// 
// 

// Change Log:
// 
// 
// 
// 
// Copyright (C), EPFL Computer Vision Lab.
// 
// 

// Code:

#include "src/libTILDE.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
//#include <utility>      // std::pair

vector<KeyPoint> testAndDump(const Mat &I,const string &pathFilter, const int &nbTest = 1, const char* ext = NULL, Mat* score = NULL)
{
	using namespace std::chrono;
	using namespace cv;
 	high_resolution_clock::time_point t1, t2;
 	std::vector<KeyPoint> kps;
 	double time_spent = 0;

	// Use appoximated filters if told to do so
 	bool useApprox = false;
 	if (ext != NULL)
 		useApprox = true;

	// Run multiple times to measure average runtime
	for (int i =0;i<nbTest;i++)
	{
		t1 = high_resolution_clock::now();
		// Run TILDE
	    kps = getTILDEKeyPoints(I, pathFilter, useApprox,true,false,score);
		t2 = high_resolution_clock::now();

		time_spent += duration_cast<duration<double>>(t2 - t1).count();
	}
	// Display execution time
	cout<<"Time all: "<<time_spent/nbTest<<" s"<<endl;


	std::vector<KeyPoint> res;
	//keep only the 100 best
	std::copy(kps.begin(),kps.begin()+min<int>(kps.size(),500),back_inserter(res));

	// Display the score image
	{
		char buf[100];
		sprintf(buf,"binary_res.png");
		if (ext != NULL)
			sprintf(buf,"binary_res_%s.png",ext);

		double minVal, maxVal;
		minMaxLoc(*score, &minVal, &maxVal);
		double range = maxVal;
		*score = (*score) / range;
		cv::imwrite(buf,*score*255);
	}	

	return res;	
}

vector<KeyPoint> test_fast(const Mat &I,const string &pathFilter, const int &nbTest = 1, Mat* score = NULL)
{
	using namespace std::chrono;
	using namespace cv;
 	high_resolution_clock::time_point t1, t2;
 	std::vector<KeyPoint> kps;
 	double time_spent = 0;



	// Run multiple times to measure average runtime
	for (int i =0;i<nbTest;i++)
	{
		t1 = high_resolution_clock::now();
		// Run TILDE
	    kps = getTILDEKeyPoints_fast(I, pathFilter,true,false,score);
		t2 = high_resolution_clock::now();

		time_spent += duration_cast<duration<double>>(t2 - t1).count();
	}
	// Display execution time
	cout<<"Time all: "<<time_spent/nbTest<<" s"<<endl;


	std::vector<KeyPoint> res;
	//keep only the 100 best
	std::copy(kps.begin(),kps.begin()+min<int>(kps.size(),500),back_inserter(res));	

	return res;	
}

int main(int argc,char** argv)
{
	using namespace std::chrono;
	using namespace cv;
	string pathFilter;


	try
	{
		// Load test image
		Mat I = imread("../../data/testImage.png");
		if (I.data == 0) throw std::runtime_error("Image not found !");

	
		cout<<"Process image without approximation (Mexico filter):"<<endl;
		// Path to the TILDE filter
			// Initialize the score image
		pathFilter = "../filters/Mexico.txt";
 		Mat score1 = Mat::zeros(I.rows,I.cols,CV_32F);
		vector<KeyPoint> kps1 = testAndDump(I,pathFilter,1,NULL, &score1);
		Mat ImgKps1;
		drawKeypoints(I, kps1, ImgKps1);
		cv::imshow("keypoints without approximation",ImgKps1);
		cv::imshow("score without approximation",score1);



		cout<<"Process Image with approximation (Mexico filter):"<<endl;
		// Path to the TILDE approx filter
		pathFilter = "../filters/Mexico24.txt";
		Mat score2 = Mat::zeros(I.rows,I.cols,CV_32F);
		vector<KeyPoint> kps2 = testAndDump(I,pathFilter,1,"n_approx", &score2);
		Mat ImgKps2;
		drawKeypoints(I, kps2, ImgKps2);
		cv::imshow("keypoints with approximation",ImgKps2);
		cv::imshow("image with approximation",normalizeScore(score2));




		cout<<"Process Image with approximation (Mexico filter) fast:"<<endl;
		// Path to the TILDE approx filter
		pathFilter = "../filters/Mexico24.txt";
		Mat score3 = Mat::zeros(I.rows,I.cols,CV_32F);
		vector<KeyPoint> kps3 = test_fast(I,pathFilter,1, &score3);
		Mat ImgKps3;
		drawKeypoints(I, kps3, ImgKps3);
		cv::imshow("keypoints with approximation fast",ImgKps3);
		cv::imshow("image with approximation fast",normalizeScore(score3));


		cout<<"press a key to exit"<<endl;
		cv::waitKey(0);
	}
	catch (std::exception &e) {
		cout<<"ERROR: "<<e.what()<<"\n";
	}

	return 0;
}
// 
// test.cpp ends here
