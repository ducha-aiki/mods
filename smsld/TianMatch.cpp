// TianMatch.cpp : Defines the entry point for the console application.
//
//#include "stdafx.h"

#include "cv.h"
#include "highgui.h"

#include "wzhlib.h"
#include "descriptor.h"
#include "Match.h"

#include <string.h>

#pragma comment(lib, "v4500v.lib")

int MatchLineBySC(char* imageFilename1,char* imageFilename2,
				  char* txtFilename1,char* txtFilename2,
				  char* txtSaveFile,double fDistinctive);

int _tmain(int argc, char* argv[])
{
	char * imageFilename1;
	char * imageFilename2;
	char * txtFilename1;
	char * txtFilename2;
	char * txtSaveFileSc;

	if(argc >= 6) {
		imageFilename1 = argv[1];
		imageFilename2 = argv[2];
		txtFilename1 = argv[3];
		txtFilename2 = argv[4];
		txtSaveFileSc = argv[5];
	} else {
		printf("Usage: TianMatch.exe <path_to_image1> <path_to_image2> <path_to_txt_image1> <path_to_txt_image2> <path_to_results_file>");
		return 1;
	}
	double fDistinctive = 0.8;
	MatchLineBySC(imageFilename1,imageFilename2,txtFilename1,txtFilename2,txtSaveFileSc,fDistinctive);
	return 1;
}

int MatchLineBySC(char* imageFilename1,char* imageFilename2,
					 char* txtFilename1,char* txtFilename2,
					 char* txtSaveFile,double fDistinctive)
{
	double* pImageData1	= NULL;
	int nWidth1			= 0;
	int nHeight1		= 0;
	if(!wzhLoadImage(pImageData1,nWidth1,nHeight1,imageFilename1))
	{
		return 0;
	}

	//Get points and describe lines.
	int nLineCount1		= 0;
	int szCountForEachLine1[nMaxLineCount];
	float scalesForEachLine1[2*nMaxLineCount];
	float angleForEachLine1[2*nMaxLineCount];
	double* pLinePts1	= NULL;
	if(!LoadLineTxt(pLinePts1,nLineCount1,szCountForEachLine1,scalesForEachLine1,angleForEachLine1,txtFilename1))
	{
		return 0;
	}

	float* pDes1 = NULL;
	byte*  pByValidFlag1 = new byte[nLineCount1];
	pDes1 = new float[nDesDim*nLineCount1];
	ComputeDes(	pDes1,pByValidFlag1,
				pImageData1,nWidth1,nHeight1,
				pLinePts1,nLineCount1,szCountForEachLine1,scalesForEachLine1,angleForEachLine1);

	double* pImageData2	= NULL;
	int nWidth2			= 0;
	int nHeight2		= 0;
	if(!wzhLoadImage(pImageData2,nWidth2,nHeight2,imageFilename2))
	{
		return 0;
	}

	int nLineCount2		= 0;
	int szCountForEachLine2[nMaxLineCount];
	float scalesForEachLine2[2*nMaxLineCount];
	float angleForEachLine2[2*nMaxLineCount];
	double* pLinePts2	= NULL;
	if(!LoadLineTxt(pLinePts2,nLineCount2,szCountForEachLine2,scalesForEachLine2,angleForEachLine2,txtFilename2))
	{
		return 0;
	}
	
	float* pDes2 = NULL;
	byte*  pByValidFlag2 = new byte[nLineCount2];
	pDes2 = new float[nDesDim*nLineCount2];
	ComputeDes(	pDes2,pByValidFlag2,
				pImageData2,nWidth2,nHeight2,
				pLinePts2,nLineCount2,szCountForEachLine2,scalesForEachLine2,angleForEachLine2);
	descriptorFreeMemory();

	/******************************************************************************
						Matching
	******************************************************************************/
	//printf("%s","matching...\n");
	int nMaxMatchNum = max(nLineCount1,nLineCount2);
	double* pMatches  = new double[nMaxMatchNum*2];
	int nMacthCount = 0;
	matchDes(pMatches,nMacthCount,nDesDim,fDistinctive,
			 pDes1,nLineCount1,pByValidFlag1,szCountForEachLine1,
			 pDes2,nLineCount2,pByValidFlag2,szCountForEachLine2);

	/******************************************************************************

						Free memory

	******************************************************************************/
	wzhOut(txtSaveFile,pMatches,2,nMacthCount);
	wzhFreePointer(pImageData1);
	wzhFreePointer(pImageData2);
	wzhFreePointer(pLinePts1);
	wzhFreePointer(pLinePts2);
	wzhFreePointer(pDes1);
	wzhFreePointer(pDes2);
	wzhFreePointer(pByValidFlag1);
	wzhFreePointer(pByValidFlag2);
	wzhFreePointer(pMatches);

	return nMacthCount;
}