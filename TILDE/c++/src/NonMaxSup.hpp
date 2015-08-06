// NonMaxSup.hpp --- 
// 
// Filename: NonMaxSup.hpp
// Description: 
// Author: Yannick Verdie, Kwang Moo Yi, Alberto Crivella
// Maintainer: Yannick Verdie, Kwang Moo Yi
// Created: Tue Mar  3 17:51:23 2015 (+0100)
// Version: 0.5a
// Package-Requires: ()
// Last-Updated: Thu May 28 12:53:23 2015 (+0200)
//           By: Kwang
//     Update #: 8
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


#ifndef _NON_MAX_SUP_HPP_
#define _NON_MAX_SUP_HPP_


#include <iostream>
#include <fstream>
#include <string>

 #include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

vector<Point3f> NonMaxSup(const Mat & response);
vector<KeyPoint> NonMaxSup_resize_format(const Mat &response, const float& resizeRatio, const float &scaleKeypoint, const float & orientationKeypoint);



#endif // _NON_MAX_SUP_HPP_

// 
// NonMaxSup.hpp ends here
