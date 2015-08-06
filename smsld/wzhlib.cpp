#include "wzhlib.h"
#include "math.h"
#include "matlib.h"
#include "stdafx.h"

#include <fstream>
using namespace std;

/********************************************************************************

						格式转化

********************************************************************************/
void Trans2
	(IplImage* pimage,char* pImageData, int nWidth,int nHeight)
{
	memcpy(pimage->imageData,pImageData,nWidth*nHeight);
	pimage->width = nWidth;
	pimage->height = nHeight;
	pimage->imageSize = nWidth*nHeight;
	pimage->nChannels = 1;
}

void Trans2IplImage(IplImage* pimage,double* pImageData, int nWidth,int nHeight)
{
	int nSize = nWidth * nHeight;
	wzhAbs(pImageData,nSize);
	wzhNormorlize(pImageData,nSize,255.0f);

	char* pTemp = new char[nWidth*nHeight];
	Float2Byte(pTemp,pImageData,nWidth*nHeight);
	memcpy(pimage->imageData,pTemp,nWidth*nHeight);
	pimage->width = nWidth;
	pimage->height = nHeight;
	pimage->imageSize = nWidth*nHeight;
	pimage->nChannels = 1;
	delete pTemp;
}

void	GetIplImageData(double* pImageData,IplImage* pimage)
{
	int nWidth	= pimage->width;
	int nHeight = pimage->height;
	int nLineWidth = pimage->widthStep;
	for(int i = 0; i < nHeight; i++)
		for(int j =0; j < nWidth; j++)
		{
			int k1 = i*nLineWidth + j;
			char byTemp = (char)pimage->imageData[k1];
			int k2 = i*nWidth + j;
			pImageData[k2] = (double)byTemp;
		}
}

/*
Image	CVImage2WanImage(IplImage* pGrayimage)
{
	int nWidth = pGrayimage->width;
	int nHeight = pGrayimage->height;
	Image image(nWidth,nHeight);
	for (int r = 0; r < nHeight; r++)
		for (int c = 0; c < nWidth; c++)
		{
			int pos = r*nWidth+c;
			image(c,r) = ((double) pGrayimage->imageData[pos]) / 255.0;
		}
		return image;
}
*/
Image	CreatWanImage(double* pGrayimage,int nWidth,int nHeight)
{
	Image image(nWidth,nHeight);
	for (int r = 0; r < nHeight; r++)
		for (int c = 0; c < nWidth; c++)
		{
			int pos = r*nWidth+c;
			image(c, r) = ((double) pGrayimage[pos]) / 255.0;
		}
	return image;
}

/********************************************************************************

						CV 显示相关
	
********************************************************************************/
/*void wzhShowData(double* pData,int nWidth,int nHeight,char* name)
{
	double* pTemp = new double[nWidth*nHeight];
	memcpy(pTemp,pData,sizeof(double)*nWidth*nHeight);
	IplImage* pImageTemp = cvCreateImage(cvSize(nWidth,nHeight),IPL_DEPTH_8U,1);
	Trans2IplImage(pImageTemp,pData,nWidth,nHeight);
	cvNamedWindow(name,2);
	cvShowImage(name,pImageTemp);
	cvWaitKey(0);
	cvDestroyWindow(name);
	cvReleaseImage(&pImageTemp);
	wzhFreePointer(pTemp);
}

void wzhShowData(char* pData,int nWidth,int nHeight,char* name)
{
	char* pTemp = new char[nWidth*nHeight];
	memcpy(pTemp,pData,sizeof(char)*nWidth*nHeight);
	IplImage* pImageTemp = cvCreateImage(cvSize(nWidth,nHeight),IPL_DEPTH_8U,1);
	Trans2IplImage(pImageTemp,pData,nWidth,nHeight);
	cvNamedWindow(name,2);
	cvShowImage(name,pImageTemp);
	cvWaitKey(0);
	cvDestroyWindow(name);
	cvReleaseImage(&pImageTemp);
	wzhFreePointer(pTemp);
}*/

/********************************************************************************
函数名：
MarkCornerImage
描述：
该函数显示角点图像
结果：
********************************************************************************/
void	wzhShowPointsOfImage(IplImage* pImage,int* pPoints,int nCount)
{
	CvScalar color = cvScalar(0, 0, 255);
	for(int i = 0; i < nCount; i++)
	{
		int rr = pPoints[2*i];
		int cc = pPoints[2*i+1];

		CvPoint pt1, pt2;
		pt1.x = cc-2; 
		pt1.y = rr-2;
		pt2.x = cc+2; 
		pt2.y = rr+2;
		cvRectangle(pImage, pt1, pt2, color);
	}

	char* name = "KeyPoints Detected";
	cvNamedWindow(name,2);
	cvShowImage(name,pImage);
	cvWaitKey(0);
	cvDestroyWindow(name);
}

bool wzhLoadImage(double*& pImageData, int& nWidth,int& nHeight,char* filename)
{
	//读入图像 
	IplImage*	cvGrayimg	= cvLoadImage(filename,0);
	if(cvGrayimg == NULL)
	{
		return FALSE;
	}
	nWidth = cvGrayimg->width;
	nHeight	= cvGrayimg->height;
	pImageData	= new double[nWidth*nHeight];
	GetIplImageData(pImageData,cvGrayimg);
	cvReleaseImage(&cvGrayimg);

	return TRUE;
}

/********************************************************************************

							文件相关

********************************************************************************/
bool wzhOut(char* filename,double* pData,int nWidth, int nHeight)
{
	ofstream outfile;
	outfile.open(filename);

	//寻找保存
	for(int i = 0; i < nHeight; i++)
	{
		for(int j = 0; j < nWidth; j++)
		{
			int  k = i * nWidth + j;
			outfile << pData[k] << "	";
		}
		outfile << endl;
	}
	outfile.close();

	return true;
}

bool wzhOut(char* filename,float* pData,int nWidth, int nHeight)
{
	ofstream outfile;
	outfile.open(filename);

	//寻找保存
	for(int i = 0; i < nHeight; i++)
	{
		for(int j = 0; j < nWidth; j++)
		{
			int  k = i * nWidth + j;
			outfile << (double)pData[k] << "	";
		}
		outfile << endl;
	}
	outfile.close();

	return true;
}


bool	LoadCornerTxt(double*& pfCorners,int& iCornersCount,char* cornerPath)
{

	FILE * fp1;
	fp1 = fopen(cornerPath,"r");
	if (!fp1) 
	{
		return false;
	}

	//读取第一行的角点数
	fscanf(fp1,"%i",&iCornersCount);

	wzhFreePointer(pfCorners);
	pfCorners = new double[iCornersCount*2];

	//读取角点信息
	for (int i = 0; i < iCornersCount; i++) 
	{
		float rr;
		float cc;
		fscanf(fp1,"%f",&rr);
		fscanf(fp1,"%f",&cc);
		pfCorners[2*i]		= (double)rr;
		pfCorners[2*i+1]	= (double)cc;
	}

	fclose(fp1);
	return true;
}

bool	LoadMatchCornerTxt(double*& pfCorners1,double*& pfCorners2,int& iCornersCount,char* cornerPath)
{
	FILE * fp1;
	fp1 = fopen(cornerPath,"r");
	if (!fp1) 
	{
		return false;
	}

	//读取第一行的角点数
	fscanf(fp1,"%i",&iCornersCount);
	pfCorners1 = new double[iCornersCount*3];
	pfCorners2 = new double[iCornersCount*3];

	//读取角点信息
	for (int i = 0; i < iCornersCount; i++) 
	{
		double rr;
		double cc;
		fscanf(fp1,"%f",&rr);
		fscanf(fp1,"%f",&cc);
		pfCorners1[3*i]		= rr;
		pfCorners1[3*i+1]	= cc;
		pfCorners1[3*i+2]	= 1.0f;
		fscanf(fp1,"%f",&rr);
		fscanf(fp1,"%f",&cc);
		pfCorners2[3*i]		= rr;
		pfCorners2[3*i+1]		= cc;
		pfCorners2[3*i+2]	= 1.0f;
	}

	fclose(fp1);
	return true;
}

bool	LoadLineTxt(double*& pLinePts,int& nLineCount,int nCountForEachLine[], float scaleForEachLine[], float angleForEachLine[], char* txtFilename)
{
	//打开文件
	FILE * fp1;
	fp1 = fopen(txtFilename,"r");
	if (!fp1) 
	{
		return false;
	}

	//读入直线总数
	// Get number of found line segments
	fscanf(fp1,"%i",&nLineCount);
	
	//读取各直线上点的个数
	// Calculate total length of all points on lines.
	int nTotol = 0;
	for(int i = 0; i < nLineCount; i++)
	{
		int nTemp = 0;
		fscanf(fp1,"%i",&nTemp);
		nCountForEachLine[i] = nTemp;
		nTotol = nTotol + nTemp;
	}

	// My addition
	// Get scales of each line
	for(int i = 0; i < 2*nLineCount; i++) {
			float tmp_scale;
			fscanf(fp1,"%f",&tmp_scale);
			scaleForEachLine[i] = tmp_scale;
	}

	// Get angles of scale lines.
	for(int i = 0; i < 2*nLineCount; i++) {
		float tmp_angle;
		fscanf(fp1,"%f",&tmp_angle);
		angleForEachLine[i] = tmp_angle;
	}
	// End addition

	//读取点坐标
	// Get all points
	wzhFreePointer(pLinePts);
	pLinePts = new double[nTotol*2];
	int nCount = 0;
	while(!feof(fp1))
	{
		float rr;
		float cc;
		fscanf(fp1,"%f",&rr);
		fscanf(fp1,"%f",&cc);
		pLinePts[2*nCount]   = (double)(rr-1);
		pLinePts[2*nCount+1] = (double)(cc-1);
		nCount ++;
		if(nCount > nTotol-1)
		{
			break;
		}
	}


	fclose(fp1);
	return true;
}

bool	LoadFlagTxt(int& nNumber1,int& nNumber2,char* txtFileFlag)
{
	//打开文件
	FILE * fp1;
	fp1 = fopen(txtFileFlag,"r");
	if (!fp1) 
	{
		return false;
	}

	//读入直线总数
	fscanf(fp1,"%i",&nNumber1);
	fscanf(fp1,"%i",&nNumber2);

	fclose(fp1);
	return true;
}

/********************************************************************************

							矩阵操作

********************************************************************************/
void	RGB2gray(char* pDataGray,const char* pDataRGB,int nWidth,int nHeight)
{  
	for(int i = 0;i < nHeight; i++)
		for(int j = 0; j < nWidth; j++)
		{
			int  k = i*nWidth + j;
			pDataGray[k] = (char)(pDataRGB[3*k+2]*0.2989f) + (char)(pDataRGB[3*k+1]*0.5870f) + (char)(pDataRGB[3*k]*0.1140f);
		} 
}

void	Float2Byte(char* pByteData,double* pFloatData,int nSize)
{
	for(int i = 0; i < nSize; i++)
	{
		double tt = (char)pFloatData[i];
		char by = (char)tt;
		pByteData[i] = (char)pFloatData[i];
	}
}

double	wzhMax(const double* pData, int nSize)
{
	double fMax = pData[0];
	for(int  i = 0; i < nSize; i++)
	{
		if(pData[i] > fMax)
			fMax = pData[i];
	}
	return fMax;
}

void wzhMax(double& maxV,int& maxPos,const double* pData, int nSize)
{
	maxV = pData[0];
	for(int  i = 0; i < nSize; i++)
	{
		if(pData[i] > maxV)
		{
			maxV = pData[i];
			maxPos = i;
		}
	}
}
double	wzhMin(const double* pData, int nSize)
{
	double fMax = 100000;
	for(int  i = 0; i < nSize; i++)
	{
		if(pData[i] < fMax)
			fMax = pData[i];
	}
	return fMax;
}

double	wzhMean(const double* pData, int nSize)
{
	double fSumV = wzhSum(pData,nSize);
	return fSumV/(double)nSize;
}

double	wzhSum(const double* pData, int nSize)
{
	double fReult = 0;
	for(int i = 1; i < nSize; i++)
	{
		fReult = fReult + pData[i];
	}
	return fReult;
}

void	wzhAbs(double* pData,int nSize)
{
	for(int i = 0; i < nSize; i ++)
	{
		pData[i] = abs(pData[i]);
	}
}
void	wzhSqare(double* pData,int nSize)
{
	for(int i = 0; i < nSize; i ++)
	{
		pData[i] = pData[i] * pData[i];
	}
}

void	wzhNormorlize(double* pData,int nSize,double fV)
{
	double maxV = wzhMax(pData,nSize);
	for(int i = 0; i < nSize; i++)
	{
		pData[i] = pData[i]*fV/maxV;
	}
}

void	wzhNormorlizeNorm(double* pData,int nSize)
{
	double fNorm = 0.0f;
	for(int i = 0; i < nSize; i++)
	{
		double fTemp  = pData[i];
		fNorm = fNorm + fTemp*fTemp;
	}
	fNorm = (double)sqrt((double)fNorm);
	if(fNorm < EPS_CONST)
	{
		return;
	}
	for(int i = 0; i < nSize; i++)
	{
		pData[i] = pData[i]/fNorm;
	}
}

void	wzhMulMatrix(double* pResult,const double* pMatrix1,const double* pMatrix2,int nDataLength)
{
	for(int i = 1; i < nDataLength; i++)
	{
		pResult[i] = pMatrix1[i]*pMatrix2[i];
	}
}

void	wzhMulMatrix(char* pResult,const char* pMatrix1,const char* pMatrix2,int nDataLength)
{
	for(int i = 1; i < nDataLength; i++)
	{
		pResult[i] = pMatrix1[i]*pMatrix2[i];
	}
}

void	wzhFindMaximum(char* pResult,int& nCount,double* pData,int nWidth,int nHeight,int nR)
{
	nCount = 0;
	memset(pResult,0,sizeof(char)*nWidth*nHeight);
	for(int i = nR; i < nHeight-nR-1; i++)
		for(int j = nR; j < nWidth-nR-1; j++)
		{
			int k = i*nWidth+j;
			bool flag = true;
			for(int k1 = 0; k1 < 2*nR+1; k1++)
			{
				for(int k2 = 0; k2 < 2*nR+1; k2++)
				{
					int ii = i - nR + k1;
					int jj = j - nR + k2;
					int kk = ii*nWidth + jj;
					if(pData[k] < pData[kk])
					{
						flag = false;
						break;
					}
				}
				if(!flag)
				{
					break;
				}
			}
			if(flag)
			{
				pResult[k] = 1;
				nCount++;
			}
		}
}

void	wzhFindNonZeros(double*& pPs,int& nCount,char* pData,int nWidth,int nHeight)
{
	double* pTemp = new double[nWidth*nHeight*2];
	nCount = 0;
	for(int i = 0; i < nHeight; i++)
		for(int j = 0; j < nWidth; j++)
		{
			int k = i*nWidth + j;
			if(pData[k] > 0)
			{
				pTemp[2*nCount] = (double)i;
				pTemp[2*nCount+1] = (double)j;
				nCount++;
			}
		}
	pPs = new double[nCount*2];
	memcpy(pPs,pTemp,sizeof(double)*nCount*2);
	wzhFreePointer(pTemp);
}

void	wzhThreshold(char* pResult,double* pData,int nWidth,int nHeight,double fT)
{
	memset(pResult,0,sizeof(char)*nWidth*nHeight);
	for(int i = 0; i < nHeight; i++)
		for(int j = 0; j < nWidth; j++)
		{
			int k = i * nWidth + j;
			if(pData[k] > fT)
			{
				pResult[k] = 1;
			}
		}
}

/*****************************************************************
	描述： 
		该函数计算卷积
*****************************************************************/
void	wzhConvol(double* pImageDataResult, const double* pImageData,int nWidth,int nHeight,double* pfTempalte,int nR)
{
	double* pTemp = new double[nWidth*nHeight];
	memset(pTemp,0,sizeof(double)*nWidth*nHeight);

	int nN = 2*nR + 1;
	for(int i = 0; i < nHeight; i++)
		for(int j = 0; j < nWidth; j++)
		{
			int k = i*nWidth+j;
			if(i < nR || j < nR || i >= nHeight-nR || j >= nWidth-nR)
			{
				continue;
			}
			double fAdd = 0.0f;
			for(int k1 = 0; k1 < nN; k1++)
				for(int k2 = 0; k2 < nN; k2++)
				{
					int ii = (i+k1-nR);
					int jj = (j+k2-nR);
					int kk = ii*nWidth + jj;
					int pos = k1*nN + k2;
					fAdd = fAdd + pImageData[kk]*pfTempalte[pos];
				}
			pTemp[k] = fAdd;
		}
	memcpy(pImageDataResult,pTemp,sizeof(double)*nWidth*nHeight);
	wzhFreePointer(pTemp);

}

double	wzhDot(double* pData1,double* pData2,int nDim)
{
	double fReturn = 0.0f;
	for(int i = 0; i < nDim; i++)
	{
		fReturn = pData1[i]*pData2[i];
	}
	return fReturn;
}
float	wzhDistance(float* pData1,float* pData2,int nDim)
{
	float fReturn = 0.0f;
	for(int i = 0; i < nDim; i++)
	{
		float fTemp = pData1[i]-pData2[i];
		fReturn = fReturn + fTemp*fTemp;
	}
	return (float)sqrt((double)fReturn);
}

void	wzhSet(double* pData,double fV, int nSize)
{
	for(int i = 0; i < nSize; i++)
	{
		pData[i] = fV;
	}
}

int	wzhRound(double dData)
{
	int iReturn = (int)dData;
	double error = abs(iReturn-dData);
	if(dData >= 0 && error > 0.5)
	{
		iReturn = iReturn + 1;
	}
	else if(dData < 0 && error > 0.5)
	{
		iReturn = iReturn - 1;
	}
	return iReturn;
}

int	wzhRange(int nP,int nMin,int nMax)
{
	int nReturn = nP;
	if(nP < nMin)
		nReturn = nMin;
	if(nP > nMax)
		nReturn = nMax;
	return nReturn;
}

/****************************************************************************

		
							计算梯度信息


****************************************************************************/
void ConputeGaussianGrad(double* pResult,double* pOri,int nWidth,int nHeight,double fSigma,int nType)
{
	double * pTemp = new double[nWidth*nHeight];

	//计算窗口大小
	int nTemR = (int)(3*fSigma);
	if(abs(3*fSigma - nTemR) > 0.5)
	{
		nTemR++;
	}
	int nN = 2*nTemR + 1;

	//分配内存
	double*	fTemD2	= new double[nN*nN];
	
	//计算模板和梯度图像
	ComputeGaussianTepalte(fTemD2,nTemR,fSigma,nType);
	wzhConvol(pTemp, pOri,nWidth,nHeight,fTemD2,nTemR);

	//复制结果
	memcpy(pResult,pTemp,nWidth*nHeight*sizeof(double));

	//释放内存
	if(fTemD2)
	{
		delete fTemD2;
		fTemD2 = NULL;
	}
	if(pTemp)
	{
		delete pTemp;
		pTemp = NULL;
	}

}

void	ComputeMag(double* fMag,double* pOri,int nWidth,int nHeight,double fSigma)
{
	double * pTemp = new double[nWidth*nHeight];

	//分配内存
	double* pImageDx = new double[nWidth*nHeight];
	double* pImageDy = new double[nWidth*nHeight];

	//计算DX DY
	ConputeGaussianGrad(pImageDx,pOri,nWidth,nHeight,fSigma,11);
	ConputeGaussianGrad(pImageDy,pOri,nWidth,nHeight,fSigma,12);

	//计算模
	ComputeMag(pTemp,pImageDx,pImageDy,nWidth*nHeight);

	//复制结果
	memcpy(fMag,pTemp,nWidth*nHeight*sizeof(double));

	//释放内存
	if(pImageDx != NULL)
	{
		delete pImageDx;
		pImageDx = NULL;
	}
	if(pImageDy != NULL)
	{
		delete pImageDy;
		pImageDy = NULL;
	}

	if(pTemp)
	{
		delete pTemp;
		pTemp = NULL;
	}
}

void	ComputeMag(double* fMag,const double* fGx,const double* fGy,int nSize)
{
	for(int k = 0; k < nSize; k++)
	{
		fMag[k] = sqrt(fGx[k]*fGx[k] + fGy[k]*fGy[k]);
	}
}

/*****************************************************************
描述： 
	该函数计算高斯模板
	type 0  ——0阶		11——1阶x		12 —— 1阶y	
		21 ——2阶xx	22——2阶yy		212—— 2阶xy
*****************************************************************/
void	ComputeGaussianTepalte(double* pTempalte,int nR,double fSigma,int type)
{
	int nN = 2*nR + 1;
	double fSigma_2 = fSigma*fSigma;
	double fSigma_4 = fSigma_2*fSigma_2;
	double fSigma_6 = fSigma_4*fSigma_2;
	for(int i = 0; i< nN; i++)
		for(int j = 0; j< nN; j++)
		{
			double	y = i - (double)nR;
			double	x = j - (double)nR;
			double	dis_2 = x*x + y*y;
			// Check whether distance is out of scope of nR x nR block
			if(dis_2 > nR*nR)
			{
				pTempalte[i*nN+j] = 0;
				continue;
			}
			double   temp = exp(-dis_2/(2*fSigma_2));

			//0阶
			if(type == 0)
				pTempalte[i*nN+j] = (double)(temp / (2*M_PI*fSigma_2));

			//1阶
			else if(type == 11)
				pTempalte[i*nN+j] = (double)(-x*temp / (2*M_PI*fSigma_4));
			else if(type == 12)
				pTempalte[i*nN+j] = (double)(-y*temp / (2*M_PI*fSigma_4));

			//2阶
			else if(type == 21)
				pTempalte[i*nN+j] = (double)((x*x-fSigma_2)*temp / (2*M_PI*fSigma_6));
			else if(type == 22)
				pTempalte[i*nN+j] = (double)((y*y-fSigma_2)*temp / (2*M_PI*fSigma_6));
			else if(type == 212)
				pTempalte[i*nN+j] = (double)(x*y*temp / (2*M_PI*fSigma_6));
		}
}


/*****************************************************************

						其他 

*****************************************************************/
int		ComputeAngle(double fR, double fC)
{
	double xx = fC;
	double yy = fR;
	double arc = atan2((double)xx,(double)yy);
	if(arc < 0)
	{
		arc = arc + 2*M_PI;
	}
	int nAngle = (int)(arc*180/M_PI);
	return nAngle;
}

void	ComputeHarrisCurvature(double* pResult,double* pOri,int nWidth,int nHeight,double fSigma)
{
	//模板
	double	fTemDX[9]	=	{-1,0,1,	-1,0,1, -1,0,1};
	double	fTemDY[9]	=	{-1,-1,-1,	0,0,0,	1,1,1 };
	
	//分配内存
	double* pDx = new double[nWidth*nHeight];
	double* pDy = new double[nWidth*nHeight];
	double* pDxy = new double[nWidth*nHeight];

	//梯度
	wzhConvol(pDx, pOri,nWidth,nHeight,&fTemDX[0],1);
	wzhConvol(pDy, pOri,nWidth,nHeight,&fTemDY[0],1);

	//2
	wzhMulMatrix(pDxy,pDx,pDy,nWidth*nHeight);
	wzhSqare(pDx,nWidth*nHeight);
	wzhSqare(pDy,nWidth*nHeight);

	//Blur
	ConputeGaussianGrad(pDx,pDx,nWidth,nHeight,fSigma,0);
	ConputeGaussianGrad(pDy,pDy,nWidth,nHeight,fSigma,0);
	ConputeGaussianGrad(pDxy,pDxy,nWidth,nHeight,fSigma,0);

	memset(pResult,0,sizeof(double)*nWidth*nHeight); 
	double fEeps = 0.0001f;
	for(int i = 0; i < nHeight; i++)
		for(int j = 0; j < nWidth; j++)
		{
			int k = i*nWidth + j;
			double Ix2 = pDx[k];
			double Iy2 = pDy[k];
			double Ixy = pDxy[k];
			pResult[k] = abs((Ix2*Iy2 - Ixy*Ixy)/(Ix2 + Iy2 + fEeps)); 
		}

	//释放内存
	if(pDx != NULL)
	{
		delete pDx;
		pDx = NULL;
	}
	if(pDy != NULL)
	{
		delete pDy;
		pDy = NULL;
	}
	if(pDxy != NULL)
	{
		delete pDxy;
		pDxy = NULL;
	}
}


void	ComputeLogEnergy(double* pResult,double* pOri,int nWidth,int nHeight,double fSigma)
{
	int nR = (int)(fSigma*3.0f);
	int nN = 2*nR + 1;

	//梯度
	double* pTemXX = new double[nN*nN];
	double* pTemYY = new double[nN*nN];
	double* pTem_Log = new double[nN*nN];
	ComputeGaussianTepalte(pTemXX,nR,fSigma,21);
	ComputeGaussianTepalte(pTemYY,nR,fSigma,22);
	for(int i=0; i < nN*nN; i++)
	{
		pTem_Log[i] = pTemXX[i] + pTemYY[i];
	}

	wzhConvol(pResult, pOri,nWidth,nHeight,pTem_Log,nR);
	wzhAbs(pResult,nWidth*nHeight);
	wzhFreePointer(pTemXX);
	wzhFreePointer(pTemYY);
	wzhFreePointer(pTem_Log);
}

void	wzhFreePointer(short* pP)
{
	if(pP != NULL)
	{
		delete pP;
		pP = NULL;
	}
}

void	wzhFreePointer(float* pP)
{
	if(pP != NULL)
	{
		delete pP;
		pP = NULL;
	}
}

void	wzhFreePointer(double* pP)
{
	if(pP != NULL)
	{
		delete pP;
		pP = NULL;
	}
}
void	wzhFreePointer(char* pP)
{
	if(pP != NULL)
	{
		delete pP;
		pP = NULL;
	}
}
void	wzhFreePointer(int* pP)
{
	if(pP != NULL)
	{
		delete pP;
		pP = NULL;
	}
}

void	NormalizePs2D(double* pNewPs,double T[3][3],double* pPs,int nCount)
{		
	double* pTemp = new double[nCount*3];
	memcpy(pTemp,pPs,sizeof(double)*nCount*3);

	//计算质心
	double rrC = 0.0f;
	double ccC = 0.0f;
	for(int i = 0; i < nCount; i++)
	{
		rrC		= rrC + pPs[3*i];
		ccC		= ccC + pPs[3*i+1];
	}
	rrC = rrC / nCount;
	ccC = ccC / nCount;

	//归一化到质心
	for(int i = 0; i < nCount; i++)
	{
		pTemp[3*i]		= pPs[3*i]		- rrC;
		pTemp[3*i+1]	= pPs[3*i+1]	- ccC;
	}

	//计算方差
	double meandist = 0.0f;
	for(int i = 0; i < nCount; i++)
	{
		meandist = meandist + sqrt(pTemp[3*i] * pTemp[3*i] + pTemp[3*i+1] * pTemp[3*i+1]);
	}
	meandist = meandist/nCount;

	//计算变换
	double scale = (double)sqrt((double)2)/meandist;
	double T_t[3][3] = {	{scale,	0,		-scale*rrC},
						{0,		scale,	-scale*ccC},
						{0,		0,		1}};
	memcpy(&T[0][0],T_t,sizeof(double)*9);

	//计算新坐标	newpts = T*pts;
	for(int i = 0; i < nCount; i++)
	{
		pNewPs[3*i]		= T[0][0]*pPs[3*i] + T[0][1]*pPs[3*i+1] + T[0][2]*pPs[3*i+2];
		pNewPs[3*i+1]	= T[1][0]*pPs[3*i] + T[1][1]*pPs[3*i+1] + T[1][2]*pPs[3*i+2];
		pNewPs[3*i+2]	= T[2][0]*pPs[3*i] + T[2][1]*pPs[3*i+1] + T[2][2]*pPs[3*i+2];
	}

	//释放内存
	wzhFreePointer(pTemp);
}

//*************************************************************************
//
//	该函数 通过直接线性变换求基本矩阵
//	pP1 齐次坐标
//	pP2	齐次坐标
//
//*************************************************************************
void	ComputeFundamental(double F[3][3],double* pP1,double* pP2,int nCount)
{
	//转化为齐次坐标
	double* pNewP1	= new double[nCount*3];
	double* pNewP2	= new double[nCount*3];

	//归一化
	double T1[3][3] = {0};
	double T2[3][3] = {0};
	NormalizePs2D(pNewP1,T1,pP1,nCount);
	NormalizePs2D(pNewP2,T2,pP2,nCount);

	//计算矩阵
	double* AA = new double[nCount*9];
	for(int g = 0; g < nCount; g++)
	{

		AA[9*g+0] = pNewP2[3*g+0]*pNewP1[3*g+0];
		AA[9*g+1] = pNewP2[3*g+0]*pNewP1[3*g+1];
		AA[9*g+2] = pNewP2[3*g+0];

		AA[9*g+3] = pNewP2[3*g+1]*pNewP1[3*g+0];
		AA[9*g+4] = pNewP2[3*g+1]*pNewP1[3*g+1];
		AA[9*g+5] = pNewP2[3*g+1];

		AA[9*g+6] = pNewP1[3*g+0];
		AA[9*g+7] = pNewP1[3*g+1];
		AA[9*g+8] = 1.0f;
	} 

	//奇异值分解
	initM(MATCOM_VERSION);
	Mm mMatrix = zeros(nCount,9);
	for(int g1 = 0; g1 < nCount; g1++)
		for(int g2 = 0; g2 < 9; g2++)
		{
			mMatrix.r(g1+1,g2+1) = AA[g1*9+g2];
		}

	//奇异值分解获得精确位置
	Mm u,s,v;
	i_o_t i_o = {0,0};
	svd(mMatrix,i_o,u,s,v);
	
	//使F的行列式为0
	Mm Fm = zeros(3,3);
	for(int i = 1; i < 4; i++)
		for(int j = 1; j < 4; j++)
		{
			int k = (i-1)*3 + j;
			Fm.r(i,j) = v.r(k,9);
		}
	svd(Fm,i_o,u,s,v);
	Mm mTemp = zeros(3,3);
	mTemp.r(1,1)  = s.r(1,1);
	mTemp.r(2,2)  = s.r(2,2);
	Fm = u*mTemp*ctranspose(v);

	//转化为矩阵形式
	double F_Temp[3][3] = {0};
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
		{
			int k = 3*i+j;
			F_Temp[i][j] = (double)Fm.r(i+1,j+1);
		}
		
	//退出
	exitM();

	//矩阵变换 T2'*F_temp*T1';
	double FF[3][3];
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
		{
			FF[i][j] =  T2[0][i]*F_Temp[0][j] + T2[1][i]*F_Temp[1][j] + T2[2][i]*F_Temp[2][j];
		}
	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
		{
			F[i][j] =  FF[i][0]*T1[0][j] + FF[i][1]*T1[1][j] + FF[i][2]*T1[2][j];
		}
	
	//释放内存
	wzhFreePointer(pNewP1);
	wzhFreePointer(pNewP2);
	wzhFreePointer(AA);
}

void GetSmallRegion(double* pSmallImage,double* m_pImage,int nWidth,int m_nHeight,int nCornerR,int nCornerC,int nRadius)
{
	int nCount = 0;
	for(int i = nCornerR-nRadius; i <= nCornerR+nRadius; i++)
		for(int j = nCornerC-nRadius; j <= nCornerC+nRadius; j++)
		{
			int k = i*nWidth + j;
			pSmallImage[nCount++] = m_pImage[k];
		}
}

/********************************************************************
Determine if point ('x', 'y') is within a circle of 'radius', 
assuming the circle center is (0, 0).
********************************************************************/
bool IsInCircle(double x, double y, double radius)
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
bool ParabolaInter(double &peakPos, double &peakVal, double left, double middle, double right)
{
	double a = ((left + right) - 2.0f * middle) / 2.0f;

	// not a parabola, a horizontal line.
	if (a == 0.0)
	{
		return false;
	}

	double c = (((left - middle) / a) - 1.0f) / 2.0f;
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

void	FFT1(double pResult[],double* pData, int nN)
{
	double r0 = 0;
	double i0 = 0;
	double r1 = 0;
	double i1 = 0;
	double r2 = 0;
	double i2 = 0;
	double r3 = 0;
	double i3 = 0;
	for(int i = 0; i < nN; i++)
	{
		r0 = r0 + pData[i]*cos(2*M_PI*0*i/nN);
		i0 = i0 + pData[i]*sin(2*M_PI*0*i/nN);
		r1 = r1 + pData[i]*cos(2*M_PI*1*i/nN);
		i1 = i1 + pData[i]*sin(2*M_PI*1*i/nN);
		r2 = r2 + pData[i]*cos(2*M_PI*2*i/nN);
		i2 = i2 + pData[i]*sin(2*M_PI*2*i/nN);
		r3 = r3 + pData[i]*cos(2*M_PI*3*i/nN);
		i3 = i3 + pData[i]*sin(2*M_PI*3*i/nN);
	}

	pResult[0] = sqrt(r0*r0 + i0*i0);
	pResult[1] = sqrt(r1*r1 + i1*i1);
	pResult[2] = sqrt(r2*r2 + i2*i2);
	pResult[3] = sqrt(r3*r3 + i3*i3);
}

void	ComputerJu(double pResult[],double* pData, int nN)
{
	//计算均值
	double dAvg = 0;
	for(int i = 0; i < nN; i++)
	{
		dAvg = dAvg + pData[i];
	}
	dAvg = dAvg/nN;

	//计算距
	double Ju1 = 0;
	double Ju2 = 0;
	double Ju3 = 0;
	for(int i = 0; i < nN; i++)
	{
		double error = abs(pData[i]-dAvg);
		Ju1 = Ju1 + error;
		Ju2 = Ju2 + error*error;
		Ju3 = Ju3 + error*error*error;
	}

	pResult[0] = dAvg;
	pResult[1] = (Ju1/nN);
	pResult[2] = (Ju2/nN);
	pResult[3] = (Ju3/nN);
}

void	ComputeAvgAndStd(double& dAvg,double& dStd,double* pData,int nN)
{
	//计算均值
	dAvg = 0;
	for(int i = 0; i < nN; i++)
	{
		dAvg = dAvg + pData[i];
	}
	dAvg = dAvg/nN; 

	//计算标准差 
	dStd = 0;
	for(int i = 0; i < nN; i++)
	{
		double error = (pData[i]-dAvg);
		dStd = dStd + error*error;
	}
	dStd = sqrt(dStd/nN);
}

double  LimitArc(double dArc)
{
	if(dArc < 0)
		dArc = dArc + 2*M_PI;
	else if(dArc > 2*M_PI)
		dArc = dArc - 2*M_PI;

	return dArc;
}

double	ArcDis(double dArc1,double dArc2)
{
	double error = abs(dArc1-dArc2);
	if(error > M_PI)
		error = 2*M_PI - error;

	return error;
}