//wzhlib
#pragma once
#include "cv.h"
#include "highgui.h"
#include "Image.h"

//#include <windows.h>

//cv相关
void	Trans2IplImage(IplImage* pimage,char* pImageData, int nWidth,int nHeight);
void	Trans2IplImage(IplImage* pimage,double* pImageData, int nWidth,int nHeight);
/*void	wzhShowData(char* pData,int nWidth,int nHeight,char* name);
void	wzhShowData(double* pData,int nWidth,int nHeight,char* name);*/
void	GetIplImageData(double* pImageData,IplImage* pimage);
void	wzhShowPointsOfImage(IplImage* pImage,int* pPoints,int nCount);
bool	wzhLoadImage(double*& pImageData, int& nWidth,int& nHeight,char* filename);

Image	CVImage2WanImage(IplImage* pimage);
Image	CreatWanImage(double* pGrayimage,int nWidth,int nHeight);

bool	LoadCornerTxt(double*& pfCorners,int& iCornersCount,char* cornerPath);
bool	wzhOut(char* filename,double* pData,int nWidth, int nHeight);
bool	wzhOut(char* filename,float* pData,int nWidth, int nHeight);
bool	LoadMatchCornerTxt(double*& pfCorners1,double*& pfCorners2,int& iCornersCount,char* cornerPath);
bool	LoadLineTxt(double*& pLinePts,int& nLineCount,int nCountForEachLine[], float scaleForEachLine[], float angleForEachLine[], char* txtFilename);
bool	LoadFlagTxt(int& nNumber1,int& nNumber2,char* txtFileFlag);

//矩阵操作
void	wzhConvol(double* pImageDataResult, const double* pImageData,int nWidth,int nHeight,double* pfTempalte,int nR);
void	wzhMulMatrix(double* pResult,const double* pMatrix1,const double* pMatrix2,int nDataLength);
void	wzhMulMatrix(char* pResult,const char* pMatrix1,const char* pMatrix2,int nDataLength);
void	RGB2gray(char* pDataGray,const char* pDataRGB,int nWidth,int nHeight);
void	Float2Byte(char* pByteData,double* pFloatData,int nSize);

double	wzhMax(const double* pData, int nSize);
void	wzhMax(double& maxV,int& maxPos,const double* pData, int nSize);
double	wzhMin(const double* pData, int nSize);
double	wzhMean(const double* pData, int nSize);
double	wzhSum(const double* pData, int nSize);
void	wzhAbs(double* pData,int nSize);
void	wzhSqare(double* pData,int nSize);
void	wzhNormorlize(double* pData,int nSize,double fV);
void	wzhNormorlizeNorm(double* pData,int nSize);
void	wzhFindMaximum(char* pResult,int& nCount,double* pData,int nWidth,int nHeight,int nR);
void	wzhFindNonZeros(double*& pPs,int& nCount,char* pData,int nWidth,int nHeight);
void	wzhThreshold(char* pResult,double* pData,int nWidth,int nHeight,double fT);
double	wzhDot(double* pData1,double* pData2,int nDim);
float	wzhDistance(float* pData1,float* pData2,int nDim);
void	wzhSet(double* pData,double fV, int nSize);
int		wzhRound(double dData);
int		wzhRange(int nP,int nMin,int nMax);

//计算图像高斯梯度图
void	ComputeGaussianTepalte(double* pTempalte,int nR,double sigma,int type);
void	ConputeGaussianGrad(double* pResult,double* pOri,int nWidth,int nHeight,double fSigma,int nType);
void	ComputeMag(double* fMag,const double* fGx,const double* fGy,int nSize);
void	ComputeMag(double* fMag,double* pOri,int nWidth,int nHeight,double fSigma);

//其他
int		ComputeAngle(double yy, double xx);
void	ComputeHarrisCurvature(double* pResult,double* pOri,int nWidth,int nHeight,double sigma);
void	ComputeLogEnergy(double* pResult,double* pOri,int nWidth,int nHeight,double fSigma);
void	wzhFreePointer(double* pP);
void	wzhFreePointer(char* pP);
void	wzhFreePointer(int* pP);
void	wzhFreePointer(short* pP);
void	wzhFreePointer(float* pP);

//		归一化点
void	NormalizePs2D(double* pNewPs,double T[3][3],double* pPs,int nCount);
void	ComputeFundamental(double F[3][3],double* pP1,double* pP2,int nCount);

void	GetSmallRegion(double* pSmallImage,double* m_pImage,int nWidth,int m_nHeight,int nCornerR,int nCornerC,int nRadius);

//是否在园内
bool	IsInCircle(double x, double y, double radius);
bool	ParabolaInter(double &peakPos, double &peakVal, double left, double middle, double right);

//计算傅立叶变换
void	FFT1(double pResult[],double* pData, int nN);
void	ComputerJu(double pResult[],double* pData, int nN);
void	ComputeAvgAndStd(double& dAvg,double& dStd,double* pData, int nN);

double  LimitArc(double dArc);
double	ArcDis(double dArc1,double dArc2);
