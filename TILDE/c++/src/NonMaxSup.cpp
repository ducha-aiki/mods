// NonMaxSup.cpp --- 
// 
// Filename: NonMaxSup.cpp
// Description: 
// Author: Yannick Verdie, Kwang Moo Yi, Alberto Crivella
// Maintainer: Yannick Verdie, Kwang Moo Yi
// Created: Tue Mar  3 17:48:14 2015 (+0100)
// Version: 0.5a
// Package-Requires: ()
// Last-Updated: Thu May 28 12:53:40 2015 (+0200)
//           By: Kwang
//     Update #: 5
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
#include "NonMaxSup.hpp"

vector<Point3f> 
NonMaxSup(const Mat & response)
{
    // stupid non-max suppression without any fancy tricks
    vector<Point3f> res;
    for(int i=1; i<response.rows-1; ++i){
        for(int j=1; j<response.cols-1; ++j)
        {
            bool bMax = true;

            for(int ii=-1; ii <= +1; ++ii)
            for(int jj=-1; jj <= +1; ++jj){
                if(ii==0 && jj==0)
                    continue;
                bMax &= response.at<float>(i,j) > response.at<float>(i+ii,j+jj);
            }

            if (bMax)
            {
                res.push_back(Point3f(j,i,response.at<float>(i,j)));
                //cout<<i<<" "<<j<<endl;
            }

        }            
    }

    return res;
}

vector<KeyPoint> NonMaxSup_resize_format(const Mat &response, const float& resizeRatio, const float &scaleKeypoint, const float & orientationKeypoint)
{
    // stupid non-max suppression without any fancy tricks
    vector<KeyPoint> res;
    for(int i=1; i<response.rows-1; ++i){
        for(int j=1; j<response.cols-1; ++j)
        {
            bool bMax = true;
            const float val = response.at<float>(i,j);
            for(int ii=-1; ii <= +1; ++ii)
            for(int jj=-1; jj <= +1; ++jj){
                if(ii==0 && jj==0)
                    continue;
                bMax &= val > response.at<float>(i+ii,j+jj);
            }

            if (bMax)
            {
                res.push_back(KeyPoint(Point2f(j * resizeRatio, i * resizeRatio), scaleKeypoint,orientationKeypoint,val));
            }

        }            
    }

    return res;
}



// NonMaxSup.cpp ends here
