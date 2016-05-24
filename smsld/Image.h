// Image.h -- Declaration of the class Image.


#ifndef IMAGE_H
#define IMAGE_H

/*
#include <cstdlib>
#include <iostream>
#include <cassert>

using namespace std;
*/
#include "SiftUtil.h"


class Image
{
public:
	Image(void);
	Image(int xDim, int yDim);
	Image(const Image &other);
	~Image(void);
	Image & operator =(const Image &other);

	void ReAllocate(int xDim, int yDim);
	void Allocate(int xDim, int yDim);
	void DeAllocate(void);
	
	double & operator ()(int x, int y);
	int GetXDim(void) const;
	int GetYDim(void) const;

	void Normalize(void);
	Image HalfScale(void);
	Image DoubleScale(void);
	//friend Image operator -(Image &img1, Image &img2);

private:
	int m_xDim; // width of the image.
	int m_yDim; // height of the image.
	double **m_pixels; // pixel values of the image.
};


Image operator -(Image &img1, Image &img2);


#endif