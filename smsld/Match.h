#pragma once
#include "wzhlib.h"

void matchDes(	double* matchPs,int& nMatchCount,int nDim,double fDistinctive,
			  float* pDes1,int nCount1,char* pValidFlag1,int szCountForEachLine1[],
			  float* pDes2,int nCount2,char* pValidFlag2,int szCountForEachLine2[]);

void ComputeDes(float*& pDes,char*& pByValidFlag,
				double* pImageData,int nWidth,int nHeight,
				double* pLinePts,int nLineCount,int szLinePtsCounts[],float scalesForEachLine[],float angleForEachLine[]);

bool ValidFrelation(float L1[4],float L2[4]);
float P2LDis(float P[2],float L[3]);
