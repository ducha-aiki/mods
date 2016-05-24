// PointsDesc.cpp -- Defination file.
// This class includes computing the orientations and 
// SIFT descriptors for given points in the image.

//#include "stdafx.h"
#include "PointsDesc.h"
#include <math.h>
#include <fstream>
#include "wzhlib.h"
#include "Image.h"
#include "wzhlib.h"


using namespace std;


/*-----------------------------Single descriptor-----------------------------*/

SingleDesc::SingleDesc(void)
{
	m_desc = NULL;
}

SingleDesc::~SingleDesc(void)
{
	if (m_desc != NULL)
	{
		delete []m_desc;
	}
}

void SingleDesc::SetDesc(int dim, uchar *desc)
{
	assert(dim > 0 && desc != NULL);

	m_descDim = dim;

	if (m_desc != NULL)
	{
		delete []m_desc;
	}

	m_desc = new uchar[m_descDim];
	if (m_desc == NULL)
	{
		exit(1);
	}

	for (int i=0; i<m_descDim; ++i)
	{
		m_desc[i] = desc[i];
	}
}


/*-----------------------------Points descriptors----------------------------*/

PointsDesc::PointsDesc(void)
{
	m_descs = NULL;
}

PointsDesc::PointsDesc(Image &img)
: m_img(img)
{	
	this->CalGradImgs();
}

void	PointsDesc::ComputeShapeDes(double* pLinePts,int iLineCounts,int szPtsCounts[])
{
	//初始化描述子
	m_siftDes = new float[nDesDim*iLineCounts];
	for(int i=0; i<nDesDim*iLineCounts; i++)
	{
		m_siftDes[i] = 0;
	}
	m_byValidFlag = new uchar[iLineCounts];
	for(int i=0; i<iLineCounts; i++)
	{
		m_byValidFlag[i] = 1;
	}

	//循环计算每形状的描述子
	double* pShapeDes = new double[nDesDim];
	int nPointerCurrentPos = 0;
	for(int nNo=0; nNo<iLineCounts; nNo++)
	{
		//初始化
		m_pointsNum = szPtsCounts[nNo];
		float* xPoints = new float[m_pointsNum];
		float* yPoints = new float[m_pointsNum];
		for(int k=0; k<m_pointsNum; k++)
		{
			xPoints[k] = float(pLinePts[nPointerCurrentPos+2*k+1]);
			yPoints[k] = float(pLinePts[nPointerCurrentPos+2*k]);
		}
		m_xPoints = xPoints;
		m_yPoints = yPoints;
		m_descs = new SingleDesc[m_pointsNum];
		m_nPointValidFlag = new int[m_pointsNum];

		//计算点描述子
		int		scale		= 16;
		double	scaleFactor = 1.3;
		bool	isOrien		= false;
		GenerateDesc(scale, scaleFactor, isOrien);

		//计算最终形状描述子并保存
		if(ComputeShapeDesFromPoint(pShapeDes) != 1)
		{
			m_byValidFlag[nNo] = 0;
			continue;
		}

		for(int nTemp=0; nTemp<nDesDim; nTemp++)
		{
			m_siftDes[nNo*nDesDim+nTemp] = (float)pShapeDes[nTemp];
		}
		nPointerCurrentPos = nPointerCurrentPos + 2*szPtsCounts[nNo];

		//释放内存
		if (m_descs != NULL)
		{
			delete []m_descs;
			m_descs = NULL;
		}
		wzhFreePointer(m_nPointValidFlag);
		wzhFreePointer(xPoints);
		wzhFreePointer(yPoints);
	}

	wzhFreePointer(pShapeDes);
}

int	PointsDesc::ComputeShapeDesFromPoint(double*& pShapeDes)
{
	wzhSet(pShapeDes,0,nDesDim);

	//验证有效性
	int nValid = 0;
	for(int j=0; j<m_pointsNum; j++)
	{
		if(m_nPointValidFlag[j] == 1)
			nValid++;
	}
	if(nValid < m_pointsNum*0.6)
	{
		return 0;
	}

	//计算均值
	double* pMean = new double[nDesDim/2];
	wzhSet(pMean,0,nDesDim/2);
	for(int i=0; i<nDesDim/2; i++)
		for(int j=0; j<m_pointsNum; j++)
		{
			if(m_nPointValidFlag[j] == 1)
			{
				uchar cTemp = m_descs[j].m_desc[i];
				pMean[i] = pMean[i] + (double)cTemp;
			}
		}
	for(int i=0; i<nDesDim/2; i++)
	{
		pMean[i] = pMean[i]/nValid;
	}

	//计算标准差
	double* pStd = new double[nDesDim/2];
	wzhSet(pStd,0,nDesDim/2);
	for(int i=0; i<nDesDim/2; i++)
		for(int j=0; j<m_pointsNum; j++)
		{
			if(m_nPointValidFlag[j] == 1)
			{
				double dTemp = pMean[i];
				double dCur = (double)m_descs[j].m_desc[i];
				pStd[i] = pStd[i] + (dCur-dTemp)*(dCur-dTemp);
			}
		}
	for(int i=0; i<nDesDim/2; i++)
	{
		pStd[i] = sqrt(pStd[i]/nValid);
	}

	//存入描述子
	wzhNormorlizeNorm(pMean,nDesDim/2);
	wzhNormorlizeNorm(pStd,nDesDim/2);
	for(int i=0; i<nDesDim/2; i++)
	{
		pShapeDes[i] = pMean[i];
		pShapeDes[i+nDesDim/2] = pStd[i];
	}
	wzhNormorlizeNorm(pShapeDes,nDesDim);

	wzhFreePointer(pMean);
	wzhFreePointer(pStd);
	
	return 1;
}

PointsDesc::~PointsDesc(void)
{
	if (m_descs != NULL)
	{
		delete []m_descs;
	}
	wzhFreePointer(m_siftDes);
	wzhFreePointer(m_byValidFlag);
}

/********************************************************************
   'scale' -- scale of the neighbour region for computing descriptor.
   'scaleFactor' -- the scale factor, used for comparing.
   'isOrien' -- determine whether to calculate orientation.
********************************************************************/
void PointsDesc::GenerateDesc(int scale, double scaleFactor, bool isOrien)
{
	int binsNum = 36;
	int gridDim = 4;
	int dirNum = 8;
	int gridSpace = 4;
	double illuThresh = 0.2;

	for (int i = 0; i < m_pointsNum; i++)
	{
		m_descs[i].m_x = m_xPoints[i];
		m_descs[i].m_y = m_yPoints[i];

		int xPos = (int)(m_xPoints[i]);
		int yPos = (int)(m_yPoints[i]);
		
		bool bFlag = xPos >= SCRadius && xPos < m_img.GetXDim()-SCRadius && 
					 yPos >= SCRadius && yPos < m_img.GetYDim()-SCRadius;
		if(!bFlag)
		{
			m_nPointValidFlag[i] = 0;
			continue;
		}

		m_descs[i].m_orien = 0.0;
		if (isOrien)
		{
			m_descs[i].m_orien = this->AssignOrien(xPos, yPos, binsNum, scaleFactor);
		}

		this->CreateDescriptor(i, gridDim, dirNum, gridSpace, illuThresh, scale, scaleFactor);
		
		//成功标记
		m_nPointValidFlag[i] = 1;
	}

}

/********************************************************************
   Calculate gradient magnitudes and orientations of Gaussian images.
********************************************************************/
void PointsDesc::CalGradImgs(void)
{
	int xDim = m_img.GetXDim();
	int yDim = m_img.GetYDim();
	Image *magnitude = new Image(xDim, yDim);
	if (magnitude == NULL)
	{
		//cerr << "Allocating memory fails!" << endl;
		exit(1);
	}
	Image *direction = new Image(xDim, yDim);
	if (direction == NULL)
	{
		//cerr << "Allocating memory fails!" << endl;
		exit(1);
	}
	
	for (int y=1; y<(yDim-1); ++y)
	{
		for (int x=1; x<(xDim-1); ++x)
		{
			(*magnitude)(x, y) = sqrt(
				pow(m_img(x + 1, y) - m_img(x - 1, y), 2) + 
				pow(m_img(x, y + 1) - m_img(x, y - 1), 2));

			(*direction)(x, y) = atan2(
				(m_img(x, y + 1) - m_img(x, y - 1)), 
				(m_img(x + 1, y) - m_img(x - 1, y)));
			if (fabs((*direction)(x, y) - PI) < pow(double(10.0), -7))
			{
				(*direction)(x, y) = -PI;
			}
		}
	}

	m_gradMagni = *magnitude;
	m_gradOrien = *direction;

	delete magnitude;
	delete direction;
}

/********************************************************************
   'binsNum' -- bins number of the orientation histogram.
   This function assigns orientations (between [-PI, PI)) for one 
   keypoint and returns assigned orientation number.
********************************************************************/
double PointsDesc::AssignOrien(int xPos, int yPos, int binsNum, double scaleFactor)
{
	Image &magnitude = m_gradMagni;
	Image &direction = m_gradOrien;

	// Build orientation histogram.

	double keyScale = 1.0 * scaleFactor;
	double sigma = 1.5 * keyScale;
	int radius = (int)(3.0 * sigma + 0.5);

	int xMin = Max(xPos - radius, 1);
	int xMax = Min(xPos + radius, magnitude.GetXDim() - 1);
	int yMin = Max(yPos - radius, 1);
	int yMax = Min(yPos + radius, magnitude.GetYDim() - 1);

	double *bins = new double[binsNum];
	for (int i=0; i<binsNum; ++i)
	{
		bins[i] = 0.0;
	}

	for (int y=yMin; y<yMax; ++y)
	{
		for (int x=xMin; x<xMax; ++x)
		{
			int relX = x - xPos;
			int relY = y - yPos;

			if (!IsInCircle(relX, relY, radius))
			{
				continue;
			}

			double gWeight = exp(- ((relX * relX + relY * relY) / 
				                   (2.0 * sigma * sigma)));
			//gWeight = gWeight / (2.0 * PI * sigma * sigma);

			double dir = direction(x, y);
			if (dir < -PI)
			{
				dir += 2.0 * PI;
			}
			if (dir >= PI)
			{
				dir -= 2.0 * PI;
			}

			// Calculate weight for orientation.
			double idxDir = (dir + PI) * binsNum / (2.0 * PI);
			int binIdxL = (int)idxDir;
			int binIdxR = (binIdxL + 1) % binsNum;
			double dirWeightL = 1.0 - (idxDir - binIdxL);
			double dirWeightR = idxDir - binIdxL;

			bins[binIdxL] += magnitude(x, y) * gWeight * dirWeightL;
			bins[binIdxR] += magnitude(x, y) * gWeight * dirWeightR;
		}
	}

	// Average orientation bins.
	//AverageWeakBins(bins, binsNum);

	// Detect the highest peak.
	double maxValue = 0.0;
	int maxBin = 0;

	for (int i=0; i<binsNum; ++i)
	{
		if (bins[i] > maxValue)
		{
			maxValue = bins[i];
			maxBin = i;
		}
	}

	// Interpolate the peak position.
	double peakPos = 0.0;
	double peakVal = maxValue;

	this->ParabolaInter(peakPos, peakVal, 
		bins[(maxBin == 0) ? (binsNum - 1) : (maxBin - 1)], 
		bins[maxBin], bins[(maxBin + 1) % binsNum]);

	assert(peakPos >= -0.5 && peakPos <= 0.5);

	double binLen = 2 * PI / binsNum;
	double orien = (maxBin + peakPos) * binLen - PI;
	if (orien < -PI)
	{
		orien += 2.0 * PI;
	}
	else if (orien >= PI)
	{
		orien -= 2.0 * PI;
	}

	delete []bins;

	return orien;
}

/********************************************************************
   Determine if point ('x', 'y') is within a circle of 'radius', 
   assuming the circle center is (0, 0).
********************************************************************/
inline bool PointsDesc::IsInCircle(double x, double y, double radius)
{
	if ((x * x + y * y) <= (radius * radius))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/********************************************************************
   Fit a parabola to the three points (-1.0; left), (0.0; middle) 
   and (1.0; right).
   Formula : f(x) = a (x - c)^2 + b.
   where c is the peak offset, b is the peak value.
   If the parabola interpolating is successed, return true, 
   otherwise return false.
********************************************************************/
bool PointsDesc::ParabolaInter(double &peakPos, double &peakVal, 
							   double left, double middle, double right)
{
	double a = ((left + right) - 2.0 * middle) / 2.0;

	// not a parabola, a horizontal line.
	if (a == 0.0)
	{
		return false;
	}

	double c = (((left - middle) / a) - 1.0) / 2.0;
	double b = middle - c * c * a;

	// 'middle' is not a peak.
	if (c < -0.5 || c > 0.5)
	{
		return false;
	}

	peakPos = c;
	peakVal = b;

	return true;
}

/********************************************************************
   'gridDim' -- grid dimension of the descriptor, 
                the recommended value is 4.
   'dirNum' -- count of discretized direction, 
               the recommended value is 8.
   'gridSpace' -- grid spacing of the descriptor, 
                  the recommended value is 4.
   'illuThresh' -- threshold to avoid illumination change, 
                   the recommended value is 0.2.
   'scale' -- scale of the neighbour region for computing descriptor.
   'scaleFactor' -- the scale factor, used for comparing.
   This function create local image descriptor for the keypoint.
   We build orientation histograms with the gradient samples around 
   the keypoint, then create descriptor with these histogram bins value.
********************************************************************/
void PointsDesc::CreateDescriptor(int iCount, 
								  int gridDim, int dirNum, int gridSpace, 
								  double illuThresh, 
								  int scale, double scaleFactor)
{
	int xPos = (int)(m_descs[iCount].m_x + 0.5);
	int yPos = (int)(m_descs[iCount].m_y + 0.5);
	double angle = m_descs[iCount].m_orien;

	int dim = gridDim * gridDim * dirNum;
	double *desc = new double[dim];
	for (int i=0; i<dim; ++i)
	{
		desc[i] = 0.0;
	}

	Image &magnitude = m_gradMagni;
	Image &direction = m_gradOrien;
	int xDim = magnitude.GetXDim();
	int yDim = magnitude.GetYDim();

	double dirSpace = 2.0 * PI / dirNum;
	int descWindow = gridDim * gridSpace;
	int radius = descWindow / 2;

	double factor = (double)scale / descWindow;

	// Gaussian weight sigma and radius.
	double gSigma = descWindow / 2;
	int gRadius = (int)(3.0 * gSigma + 0.5);

	// Search all sample points around the keypoint to create descriptor.
	for (int x=-radius; x<radius; ++x)
	{
		for (int y=-radius; y<radius; ++y)
		{
			// The keypoint is center (0, 0).
			double xS = x + 0.5;
			double yS = y + 0.5;

			// Calculate Gaussian weight.
			if (!IsInCircle(xS, yS, gRadius))
			{
				continue;
			}

			double gWeight = exp(- (xS * xS + yS * yS) / (2.0 * gSigma * gSigma));
			gWeight /= (2.0 * PI * gSigma * gSigma);

			// The coordinates are rotated by 'angle'.
			double xSR = xS * cos(angle) - yS * sin(angle);
			double ySR = xS * sin(angle) + yS * cos(angle);

			// Interpolate magnitude and direction pixel value using bilinear.
			double curX = xPos + xSR * factor * scaleFactor;
			double curY = yPos + ySR * factor * scaleFactor;
			if (curX <= 1 || curX >= (xDim - 2) ||
				curY <= 1 || curY >= (yDim - 2))
			{
				continue;
			}
			double mag = this->BlinearInter(curX, curY, magnitude);
			double ori = this->BlinearInter(curX, curY, direction);
			
			double magW = mag * gWeight;

			// We distribute the value of each gradient sample into 
			// adjacent 8 histogram bins.
			int xIdx[2];
			int yIdx[2];
			int dirIdx[2];
			double xWeight[2];
			double yWeight[2];
			double dirWeight[2];
			for (int i=0; i<2; ++i)
			{
				xIdx[i] = 0;
				yIdx[i] = 0;
				dirIdx[i] = 0;
				xWeight[i] = 0.0;
				yWeight[i] = 0.0;
				dirWeight[i] = 0.0;
			}

			// Calculate weights for x, that is (1.0 - d).
			double idxX = (xS + radius - (gridSpace / 2.0)) / gridSpace;
			if (idxX >= 0)
			{
				xIdx[0] = (int)idxX;
				xWeight[0] = 1.0 - (idxX - xIdx[0]);
			}
			if (idxX < (gridDim - 1))
			{
				xIdx[1] = (int)(idxX + 1.0);
				xWeight[1] = 1.0 - (xIdx[1] - idxX);
			}

			// Calculate weights for y, that is (1.0 - d).
			double idxY = (yS + radius - (gridSpace / 2.0)) / gridSpace;
			if (idxY >= 0)
			{
				yIdx[0] = (int)idxY;
				yWeight[0] = 1.0 - (idxY - yIdx[0]);
			}
			if (idxY < (gridDim - 1))
			{
				yIdx[1] = (int)(idxY + 1.0);
				yWeight[1] = 1.0 - (yIdx[1] - idxY);
			}

			// The direction is rotated by 'angle'.	
			double dir = ori - angle;
			if (dir < -PI)
			{
				dir += 2.0 * PI;
			}
			if (dir >= PI)
			{
				dir -= 2.0 * PI;
			}

			// Calculate weight for orientation, that is (1.0 - d).
			double idxDir = (dir + PI) * dirNum / (2.0 * PI);
			if ((int)idxDir == dirNum)
			{
				idxDir -= dirNum;
			}
			dirIdx[0] = (int)idxDir;
			dirIdx[1] = (dirIdx[0] + 1) % dirNum;
			dirWeight[0] = 1.0 - (idxDir - dirIdx[0]);
			dirWeight[1] = idxDir - dirIdx[0];

			// Build orientation histogram, and create descriptor.
			for (int iy = 0 ; iy < 2 ; ++iy)
			{
				for (int ix = 0 ; ix < 2 ; ++ix)
				{
					for (int id = 0 ; id < 2 ; ++id)
					{
						int idx = (xIdx[ix] * gridDim * dirNum) + 
							(yIdx[iy] * dirNum) + dirIdx[id];
						assert(idx >= 0 && idx < 128);

						desc[idx] += magW * xWeight[ix] * yWeight[iy] * dirWeight[id];
					}
				}
			} // end of for
		}
	}

	// Avoid illumination change.
	ThreshNorm(desc, illuThresh, dim);

	// Convert float descriptor values to uchar format.
	uchar *descUchar = new uchar[dim];
	for (int i=0; i<dim; ++i)
	{
		int val = (int)(desc[i] * 255.0 + 0.5);
		assert(val >= 0 && val <= 255);

		descUchar[i] = (uchar)val;
	}

	m_descs[iCount].SetDesc(dim, descUchar);

	delete []desc;
	delete []descUchar;
}

/********************************************************************
   Calculate bilinear interpolation pixel value in the image.
********************************************************************/
double PointsDesc::BlinearInter(double x, double y, Image &img)
{
	int x1 = (int)x;
	int y1 = (int)y;
	int x2 = x1 + 1;
	int y2 = y1 + 1;

	assert(x1 >= 0 && x2 >= 0);
	assert(x2 < img.GetXDim() && y2 < img.GetYDim());

	double val = 
		(x2 - x) * (y2 - y) * img(x1, y1) + 
		(x - x1) * (y2 - y) * img(x2, y1) + 
		(x2 - x) * (y - y1) * img(x1, y2) + 
		(x - x1) * (y - y1) * img(x2, y2);

	return val;
}

/********************************************************************
   To avoid linear illumination change, we normalize the descriptor.
   To avoid non-linear illumination change, we threshold the value 
   of each descriptor element to 'illuThresh', then normalize again.
********************************************************************/
void PointsDesc::ThreshNorm(double *desc, double illuThresh, int dim)
{
	// Normalize the descriptor, and threshold 
	// value of each element to 'illuThresh'.

	double norm = 0.0;
	
	for (int i=0; i<dim; ++i)
	{
		norm += desc[i] * desc[i];
	}

	norm = sqrt(norm);
	if(norm == 0)
	{
		return;
	}

	for (int i=0; i<dim; ++i)
	{
		desc[i] /= norm;

		if (desc[i] > illuThresh)
		{
			desc[i] = illuThresh;
		}
	}

	// Normalize again.

	norm = 0.0;

	for (int i=0; i<dim; ++i)
	{
		norm += desc[i] * desc[i];
	}

	norm = sqrt(norm);
	assert(norm != 0);

	for (int i=0; i<dim; ++i)
	{
		desc[i] /= norm;
	}
}

/*---------------------------Write descriptors file--------------------------*/
void PointsDesc::WriteDescFile(char *fileName)
{
	FILE *fp;
	fp = fopen(fileName, "wb");
	if (fp == NULL)
	{
		exit(1);
	}

	int num = m_pointsNum;
	int dim = m_descs[0].m_descDim;
	assert(dim > 0);
	fprintf(fp, "%d %d\n", num, dim);

	float scale = 1.0;
	for (int n=0; n<num; ++n)
	{
		// row col scale orientation
		fprintf(fp, "%.2f %.2f %.2f %.3f",m_descs[n].m_y, m_descs[n].m_x, scale, (float)m_descs[n].m_orien);

		uchar *desc = m_descs[n].m_desc;
		assert(desc != NULL);

		for (int i = 0; i < dim; ++i)
		{
			// Write 20 descriptor values per line.
			if((i % 20) == 0)
				fprintf(fp, "\n");
			
			fprintf(fp, "%d ", desc[i]);
		}
		fprintf(fp, "\n");
	}

	fclose(fp);
}