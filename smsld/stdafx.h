// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once
#define _CRT_SECURE_NO_DEPRECATE
#include <iostream>
//#include <tchar.h>
#include "matlib.h"


#define		matchType			 1						//1直线 2//曲线 3//区域 


#define		PI					3.1415926
#define		EPS_CONST			0.000001
#define		nMaxLineCount		6100
#define		nMaxPtsForLine		1000

#define		nMaxRegionNum		9
#define		nW					5
#define		nH					nW
#define		nEachRPixes			(nW*nH)

#define		nWScale				5
#define		nHScale				nWScale
#define		MAXSCALE			27*2
#define		MAXBLOCKWIDTH		(MAXSCALE+1)*nWScale


//MSLD
#define		SCRadius		(nMaxRegionNum / 2 * nH + 2)
#define		nDesDim			(nMaxRegionNum*8)		
