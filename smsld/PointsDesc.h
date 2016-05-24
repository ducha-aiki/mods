// PointsDesc.h -- Declaration file.
// This class includes computing the orientations and 
// SIFT descriptors for given points in the image.
#pragma once

#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "Image.h"

typedef unsigned char uchar;

class SingleDesc
{
	friend class PointsDesc;

public:
	SingleDesc(void);
	~SingleDesc(void);

	void SetDesc(int dim, uchar *desc);

private:
	SingleDesc(const SingleDesc &other);
	SingleDesc & operator=(const SingleDesc &other);

public:
	float	m_x;
	float	m_y;
	uchar	*m_desc;

	double	m_orien;
	int		m_descDim;
};


class PointsDesc
{
public:
	PointsDesc(void);
	PointsDesc(Image &img);
	~PointsDesc(void);
	void	ComputeShapeDes(double* pLinePts,int iLineCounts,int szPtsCounts[]);
	void	GenerateDesc(int scale, double scaleFactor, bool isOrien);
	void	WriteDescFile(char *fileName);
	int		ComputeShapeDesFromPoint(double*& pShapeDes);

private:
	PointsDesc(const PointsDesc &other);
	PointsDesc & operator=(const PointsDesc &other);

	void	CalGradImgs(void);
	double	AssignOrien(int xPos, int yPos, int binsNum, double scaleFactor);
	bool	IsInCircle(double x, double y, double radius);
	bool	ParabolaInter(double &peakPos, double &peakVal,double left, double middle, double right);
	void	CreateDescriptor(int iCount, int gridDim, int dirNum,int gridSpace, double illuThresh, int scale, double scaleFactor);
	double	BlinearInter(double x, double y, Image &img);
	void	ThreshNorm(double *desc, double illuThresh, int dim);

public:
	Image		m_img;
	Image		m_gradMagni; // magnitudes of gradients.
	Image		m_gradOrien; // orientations of gradients.

	//单个形状的信息
	int			m_pointsNum;
	float*		m_xPoints;
	float*		m_yPoints;
	SingleDesc*	m_descs;
	int*		m_nPointValidFlag;	 //每一个形状内各点的

	//每个形状的信息
	float*		m_siftDes;			 //形状描述子
	uchar*		m_byValidFlag;		 //每一个形状描述子计算成功标记
};