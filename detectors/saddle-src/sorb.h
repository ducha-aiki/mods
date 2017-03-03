/*
 * sorb.h
 *
 *  Created on: Oct 13, 2015
 *      Author: aldanjav
 */

#ifndef SRC_SORB_H_
#define SRC_SORB_H_

#include <opencv2/features2d/features2d.hpp>

using namespace cv;

namespace cmp{

class SadKeyPoint;

class CV_EXPORTS_W SORB
{
public:

    // the size of the signature in bytes
    enum { K_BYTES = 32, ZERO_SCORE = 0, DELTA_SCORE = 1, SUMOFABS_SCORE = 2, AVGOFABS_SCORE = 3, NORM_SCORE = 4, HESS_SCORE = 5};

    CV_WRAP explicit SORB(double responseThr = 0.0, float scaleFactor = 1.2f, int nlevels = 8, int edgeThreshold = 31,
                int epsilon = 1, int WTA_K=2, int scoreType=SUMOFABS_SCORE, int patchSize=31, int doNMS=2, int descSize=K_BYTES, uchar deltaThr=0, int nfeatures = 5000 );

    // returns the descriptor size in bytes
    int descriptorSize() const;
    // Set the descriptor size in bytes
    void setDescriptorSize(int dsize);
    // returns the descriptor type
    int descriptorType() const;

    // Compute the ORB features and descriptors on an image
    void operator()(InputArray image, InputArray mask, vector<SadKeyPoint>& keypoints) const;

    // Compute the ORB features and descriptors on an image
    void operator()( InputArray image, InputArray mask, vector<SadKeyPoint>& keypoints,
                     OutputArray descriptors, bool useProvidedKeypoints=false ) const;

    double getPyramidTime();
    double getDetectTime();
    double getDescribeTime();
    double getInnerTime();
    double getOutterTime();

    int getNumInner();
    int getNumInnerFul();
    int getNumOutterFul();


protected:

    void computeImpl( const Mat& image, vector<SadKeyPoint>& keypoints, Mat& descriptors ) const;
    void detectImpl( const Mat& image, vector<SadKeyPoint>& keypoints, const Mat& mask=Mat() ) const;


    CV_PROP_RW int nfeatures;
    CV_PROP_RW double responseThr;
    CV_PROP_RW double scaleFactor;
    CV_PROP_RW int nlevels;
    CV_PROP_RW int edgeThreshold;
    CV_PROP_RW int epsilon;
    CV_PROP_RW int WTA_K;
    CV_PROP_RW int scoreType;
    CV_PROP_RW int patchSize;
    CV_PROP_RW int doNMS;
    CV_PROP_RW int descSize;
    CV_PROP_RW uchar deltaThr;

};

class CV_EXPORTS_W_SIMPLE SadKeyPoint : public cv::KeyPoint
{
public:


CV_WRAP SadKeyPoint() : cv::KeyPoint(), intensityCenter(0), delta(0) {}
//! the full constructor
CV_WRAP SadKeyPoint(cv::Point2f _pt, float _size, float _angle=-1,
float _response=0, int _octave=0, int _class_id=-1, double intensityCenter=0, uchar delta=0): cv::KeyPoint(_pt, _size, _angle, _response, _octave, _class_id), intensityCenter(intensityCenter), delta(delta) {}

CV_WRAP SadKeyPoint(float x, float y, float _size, float _angle=-1,
float _response=0, int _octave=0, int _class_id=-1, double intensityCenter=0, uchar delta=0): cv::KeyPoint(x, y, _size, _angle, _response, _octave, _class_id), intensityCenter(intensityCenter), delta(delta) {}

double intensityCenter;
uchar labels[16];
uchar intensityPixels[16];
uchar delta;

};


class CV_EXPORTS_W_SIMPLE FastKeyPoint : public cv::KeyPoint
{
public:

CV_WRAP FastKeyPoint() : cv::KeyPoint(), count(0), isMerged(false) {}
//! the full constructor
CV_WRAP FastKeyPoint(cv::Point2f _pt, float _size, float _angle=-1,
float _response=0, int _octave=0, int _class_id=-1, uchar count = 0, bool isMerged = false) : cv::KeyPoint(_pt, _size, _angle, _response, _octave, _class_id), count(count), isMerged(isMerged) {}

CV_WRAP FastKeyPoint(float x, float y, float _size, float _angle=-1,
float _response=0, int _octave=0, int _class_id=-1, uchar count = 0, bool isMerged = false): cv::KeyPoint(x, y, _size, _angle, _response, _octave, _class_id), count(count), isMerged(isMerged) {}

cv::Point2f intensityIn;
cv::Point2f intensityOut;
uchar count;
bool isMerged;
uchar type = 0;
uchar channel = 0;
uchar maxima = 0;
};



}//end namespace cmp

#endif /* SRC_SORB_H_ */
