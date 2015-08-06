//descriptor.cpp
#pragma  once
#include "stdafx.h"
#include "descriptor.h"
#include "wzhlib.h"
#include <math.h>


SCNo	LUTSubRegion[360][nMaxRegionNum*nEachRPixes];	//各子区内的点
SCPos	LUTBiPos[nMaxRegionNum*nEachRPixes];
double	LUTWeight[nMaxRegionNum*nEachRPixes];

SCNo * LUTSubRegionScale = new SCNo[nMaxRegionNum*MAXBLOCKWIDTH*MAXBLOCKWIDTH];	//各子区内的点
SCNo * LUTSubRegionScaleZero = new SCNo[nMaxRegionNum*MAXBLOCKWIDTH*MAXBLOCKWIDTH];	
SCPos *	LUTBiPosScale = new SCPos[nMaxRegionNum*MAXBLOCKWIDTH*MAXBLOCKWIDTH];
double * LUTWeightScale = new double[nMaxRegionNum*MAXBLOCKWIDTH*MAXBLOCKWIDTH];

void descriptorFreeMemory() {
	delete[] LUTSubRegionScale;
	delete[] LUTSubRegionScaleZero;
	delete[] LUTBiPosScale;
	wzhFreePointer(LUTWeightScale);
}

/********************************************************************************
构造	函数
析构	函数
********************************************************************************/
CDescriptor::CDescriptor(double* pGrayData,int nWidth,int nHegiht,double* pLinePts,int inLineCounts,int szPtsCounts[],float scalesForEachLine[],float angleForEachLine[])
{
	m_nLineCount = inLineCounts;

	//计算总个数
	m_nTotolPts = 0;
	for(int i = 0; i < inLineCounts; i++)
	{
		m_nTotolPts = m_nTotolPts + szPtsCounts[i];
		m_szPtsCounts[i] = szPtsCounts[i];

		for(int j = 0; j <= 1; j++) {
			m_scalesForEachLine[2*i+j] = scalesForEachLine[2*i+j];
		}
		for(int j = 0; j <= 1; j++) {
			m_angleForEachLine[2*i+j] = angleForEachLine[2*i+j];
		}
	}

	//图像
	m_pImageData = NULL;
	if(pGrayData != NULL)
	{
		m_nWidth		= nWidth;
		m_nHeight		= nHegiht;
		m_nTotolPixels	= m_nWidth*m_nHeight;
		m_pImageData	= new double[m_nTotolPixels];
		memcpy(m_pImageData, pGrayData, sizeof(double)*m_nTotolPixels);
		m_pLinePts		= new double[m_nTotolPts*2];
		memcpy(m_pLinePts,pLinePts,sizeof(double)*m_nTotolPts*2);		
		m_pMainArc		= new double[m_nLineCount];
		wzhSet(m_pMainArc,0.0f,m_nLineCount);
	}

	//梯度图像
	m_pDxImage	 = new double[m_nTotolPixels];
	m_pDyImage	 = new double[m_nTotolPixels];
	m_pMagImage	 = new double[m_nTotolPixels];

	//参数
	m_fSigma	= 1.2f;
	m_scDes		= NULL;

	//描述子相关
	m_nDesDim				= nDesDim;
	m_scDes					= new float[m_nDesDim*m_nLineCount];
	m_pByValidFlag			= new char[inLineCounts];
	memset(m_pByValidFlag,0,sizeof(char)*inLineCounts);

	//初始化查找表
	InitializeLUT();
}

CDescriptor::~CDescriptor()
{
	//图像
	wzhFreePointer(m_pImageData);

	//直线点
	wzhFreePointer(m_pLinePts);
	wzhFreePointer(m_pByValidFlag);

	//梯度图像
	wzhFreePointer(m_pDxImage);
	wzhFreePointer(m_pDyImage);
	wzhFreePointer(m_pMagImage);

	//描述子
	wzhFreePointer(m_scDes);
	wzhFreePointer(m_pMainArc);
}

/********************************************************************************
初始化查找表
********************************************************************************/
void  CDescriptor::InitializeLUT()
{
	//先计算0度角的点
	// Find middle of regions in the orthogonal direction. Subtract it from the length to get the line segment in the middle of the 9 region-lines.
	int nC = nMaxRegionNum*nH/2;
	for(int i = 0;i<nMaxRegionNum; i++)
		for(int j = 0; j < nH; j++)
			for(int k=0; k < nW; k++)
			{
				// Find index of the subregion
				int temp = nW*nH*i + j*nW +k;
				// Shift (nW-1)/2 to the left to position the centers correctly
				LUTSubRegion[0][temp].nNo1 = k-(nW-1)/2;
				// Search height of center and shift it to position it with respect to the line segment in the middle (=> -nC)
				LUTSubRegion[0][temp].nNo2 = i*nH + j - nC;
			}

			//计算各旋转位置
			// Project all the center points of each region to each possible angle with the line_segment with a resolution of 1 degree.
			for(int i= 1;i < 360; i++)
			{	
				double dArc = -(double)(i*PI/180);
				for(int j= 0; j < nMaxRegionNum*nEachRPixes; j++)
				{
					int xx = LUTSubRegion[0][j].nNo1;
					int yy = LUTSubRegion[0][j].nNo2;
					LUTSubRegion[i][j].nNo1 = wzhRound(xx*cos(dArc) - yy*sin(dArc));
					LUTSubRegion[i][j].nNo2 = wzhRound(xx*sin(dArc) + yy*cos(dArc));
				}
			}

			//
			double dSigma = 22.0;
			int nR = (nH-1)/2;
			for(int i=0; i<nMaxRegionNum; i++)
				for(int j=0; j<nH; j++)
					for(int k=0; k<nW;k++)
					{
						int P = i*nEachRPixes + j*nW + k;
						int nNo1 = 0;
						int nNo2 = 0;
						// Distance from line segment (=in height direction)
						double dCoe1 = 0;
						if(j < nR)
						{
							nNo1 = i-1;
							nNo2 = nNo1 + 1;
							dCoe1 = double(nR-j)/nH;
						}
						else if(j == nR)
						{
							nNo1 = i;
							nNo2 = i;
							dCoe1 = 1;
						}
						else
						{
							nNo1 = i;
							nNo2 = nNo1 + 1;
							dCoe1 = 1 - double(j-nR)/nH;
						}

						//特殊
						// Check if we are outside the boundaries
						if(nNo1 == -1)
						{
							nNo1 = 0;
							dCoe1 = 1;
						}
						if(nNo2 == nMaxRegionNum)
						{
							nNo2	= nMaxRegionNum-1;
							dCoe1	= 0;
						}

						LUTBiPos[P].nNo1	= nNo1;
						LUTBiPos[P].nNo2	= nNo2;
						LUTBiPos[P].dCoe1	= dCoe1;
						LUTBiPos[P].dCoe2	= 1-dCoe1;

						int nC		 = (nH*nMaxRegionNum-1)/2;
						// Weight that consideres the distance to the next block
						double	d	 = (double)abs(i*nH+j-nC);
						LUTWeight[P] = exp(-d*d/(2*dSigma*dSigma));
					}
}

// Modification of initializeLUT for scaled line segments
void  CDescriptor::getScaledSubRegionPointsZeroAngle(int block_width, int block_height)
{
	// Build box that has angle 0 => much easier
	// Find middle of regions in the orthogonal direction. Subtract it from the length to get the line segment in the middle of the 9 region-lines.
	int nC = nMaxRegionNum*block_height/2;
	for(int i = 0;i<nMaxRegionNum; i++) {
		for(int j = 0; j < block_height; j++) {
			for(int k=0; k < block_width; k++)
			{
				// Find index of the subregion
				int temp = block_width*block_height*i + j*block_width +k;
				// Shift (nW-1)/2 to the left to position the centers correctly
				LUTSubRegionScaleZero[temp].nNo1 = k-(block_width-1)/2;
				// Search height of center and shift it to position it with respect to the line segment in the middle (=> -nC)
				LUTSubRegionScaleZero[temp].nNo2 = i*block_height + j - nC;
			}
		}
	}
}

void  CDescriptor::getScaledSubRegionPoints(int block_width, int block_height, double dArc)
{
	// Now rotate box
	for(int j= 0; j < nMaxRegionNum*block_width*block_height; j++)
	{
		int xx = LUTSubRegionScaleZero[j].nNo1;
		int yy = LUTSubRegionScaleZero[j].nNo2;
		LUTSubRegionScale[j].nNo1 = wzhRound(xx*cos(dArc) - yy*sin(dArc));
		LUTSubRegionScale[j].nNo2 = wzhRound(xx*sin(dArc) + yy*cos(dArc));
	}
}
void CDescriptor::getWeightingTable(int block_width, int block_height, double scale) {
	//
	double dSigma = 22.0*(scale+1);
	int nR = (block_height-1)/2;
	for(int i=0; i<nMaxRegionNum; i++)
		for(int j=0; j<block_height; j++)
			for(int k=0; k<block_width;k++)
			{
				int P = i*block_height*block_width+ j*block_width + k;
				int nNo1 = 0;
				int nNo2 = 0;
				// Distance from line segment (=in height direction)
				double dCoe1 = 0;
				if(j < nR)
				{
					nNo1 = i-1;
					nNo2 = nNo1 + 1;
					dCoe1 = double(nR-j)/block_height;
				}
				else if(j == nR)
				{
					nNo1 = i;
					nNo2 = i;
					dCoe1 = 1;
				}
				else
				{
					nNo1 = i;
					nNo2 = nNo1 + 1;
					dCoe1 = 1 - double(j-nR)/block_height;
				}

				//特殊
				// Check if we are outside the boundaries
				if(nNo1 == -1)
				{
					nNo1 = 0;
					dCoe1 = 1;
				}
				if(nNo2 == nMaxRegionNum)
				{
					nNo2	= nMaxRegionNum-1;
					dCoe1	= 0;
				}

				LUTBiPosScale[P].nNo1	= nNo1;
				LUTBiPosScale[P].nNo2	= nNo2;
				LUTBiPosScale[P].dCoe1	= dCoe1;
				LUTBiPosScale[P].dCoe2	= 1-dCoe1;

				int nC		 = (block_height*nMaxRegionNum-1)/2;
				// Weight that consideres the distance to the next block
				double	d	 = (double)abs(i*block_height+j-nC);
				LUTWeightScale[P] = exp(-d*d/(2*dSigma*dSigma));
			}
}

/********************************************************************************
直线描述子
********************************************************************************/
void  CDescriptor::ComputeLineDescriptor()
{
	//计算梯度图像
	// Compute gradients of the image
	ConputeGaussianGrad(m_pDxImage,m_pImageData,m_nWidth,m_nHeight,m_fSigma,11);
	ConputeGaussianGrad(m_pDyImage,m_pImageData,m_nWidth,m_nHeight,m_fSigma,12);
	// Compute magnitude of the gradient
	ComputeMag(m_pMagImage,m_pDxImage,m_pDyImage,m_nTotolPixels);

	//计算每一条直线的描述子
	int nPtsPos = 0;
	double nPtsPos_double;
	double* pSubDesLineDes	= new double[nMaxRegionNum*8];
	for(int nNo = 0; nNo < m_nLineCount;nNo++)
	{
		//printf("Describing line %d\n",nNo);
		// Find scale for this line. Scales in file are actually half scales.
		double scale = 2*m_scalesForEachLine[2*nNo];
		if(scale > MAXSCALE) {
			scale = MAXSCALE;
		}

		double scale_rico;
		if(2*m_scalesForEachLine[nNo+1] > MAXSCALE) {
			scale_rico = (MAXSCALE-scale)/(m_szPtsCounts[nNo]-1);
		} else {
			scale_rico = (2*m_scalesForEachLine[2*nNo+1]-scale)/(m_szPtsCounts[nNo]-1);
		}

		double scRadius;
		if(scale == 0)
			scRadius = (double) SCRadius;
		else
			scRadius = ((double) nMaxRegionNum / 2 * nHScale*(scale+1)) + 2.0;

		//*************************************************
		//		1 如果直线上无效的点超过一半,则无效
		//*************************************************
		int nPtsPos_bak = nPtsPos;
		int nValid	= 0;
		int nInValid	= 0;
		double dDxAvg	= 0;
		double dDyAvg	= 0;
		nPtsPos_double = (double) nPtsPos;
		double scale_loop = scale;
		double nT1 = 0;
		while(nT1<=m_szPtsCounts[nNo]-1)
		{
			//获得当前点的位置信息
			
			int	nCenterR	= (int)m_pLinePts[2*nPtsPos];
			int	nCenterC	= (int)m_pLinePts[2*nPtsPos+1];
			int nCenterP	= nCenterR*m_nWidth + nCenterC;
			if(scale_loop == 0) {
				dDxAvg			= dDxAvg + m_pDxImage[nCenterP];
				dDyAvg			= dDyAvg + m_pDyImage[nCenterP];
			}

			nPtsPos_double += scale_loop + 1;
			nPtsPos = (int) nPtsPos_double;
			//nPtsPos++;

			//判断是否越界
			if(	nCenterR < scRadius+1 || nCenterR > m_nHeight-scRadius-1 || 
				nCenterC < scRadius+1 || nCenterC > m_nWidth-scRadius-1)
			{
				nInValid++;
			} else {
				nValid++;
			}
			nT1+=scale_loop+1;
			scale_loop += (scale_loop+1)*scale_rico;
		}

		// Fix for memory issues.
		nPtsPos = nPtsPos_bak + m_szPtsCounts[nNo];
		//int nValid = m_szPtsCounts[nNo] - nInValid;
		if(nInValid > nValid)
		{
			m_pByValidFlag[nNo] = 0;
			continue;
		}
		else
		{
			m_pByValidFlag[nNo] = 1;
		}

		//***************************************************
		//		2	计算直线的主方向
		//		Calculate arc for line segment if we are in MSLD mode.
		//***************************************************
		double dMainArc = 0;
		float angle1;
		float angle2;
		if(matchType == 1 && scale == 0) {
			dMainArc = ComputeLineDir(&m_pLinePts[2*nPtsPos_bak],m_szPtsCounts[nNo],dDxAvg,dDyAvg);
		} else {
			angle1 = (float) LimitArc(m_angleForEachLine[2*nNo]);
			angle2 = (float) LimitArc(m_angleForEachLine[2*nNo+1]);
		}

		//***************************************************
		//		3	计算子区域描述子矩阵
		//***************************************************

		int nReCount = 0;
		nPtsPos = nPtsPos_bak;
		// Double variant of nPtsPos for better accuracy.
		nPtsPos_double = (double) nPtsPos;
		double* pSubDesMatrix  = new double[nMaxRegionNum*4*nValid];
		wzhSet(pSubDesMatrix,0,nMaxRegionNum*4*nValid);
		nT1 = 0;
		while(nT1<=m_szPtsCounts[nNo]-1)
		{

			//获得当前点的位置信息
			int	nCenterR	= (int)m_pLinePts[2*nPtsPos];
			int	nCenterC	= (int)m_pLinePts[2*nPtsPos+1];
			int nCenterP	= nCenterR*m_nWidth + nCenterC;
			nPtsPos_double += scale + 1;
			nPtsPos = (int) nPtsPos_double;
			//nPtsPos += (int) scale + 1;

			//判断是否越界
			if(	nCenterR < scRadius+1 || nCenterR > m_nHeight-scRadius-1 || 
				nCenterC < scRadius+1 || nCenterC > m_nWidth-scRadius-1)
			{
				nT1+=scale+1;
				scale += (scale+1)*scale_rico;
				continue;
			}

			//计算直线点描述子矩阵表
			double pSingleSubDes[nMaxRegionNum*4];
			if(scale == 0) {
				ComputeSubRegionProjectionZeroScale(pSingleSubDes,dMainArc,nCenterR,nCenterC);
			} else {
				ComputeSubRegionProjection(pSingleSubDes,angle1,angle2,nCenterR,nCenterC,scale);
			}
			memcpy(&pSubDesMatrix[nMaxRegionNum*4*nReCount],pSingleSubDes,sizeof(double)*nMaxRegionNum*4);
			nReCount++;
			
			// Adapt scale for next calculation. We take steps of scale+1
			nT1+=scale+1;
			scale += (scale+1)*scale_rico;
		}
		nPtsPos = nPtsPos_bak + m_szPtsCounts[nNo];

		//***************************************************
		//		4	计算描述子并存入描述子
		//***************************************************
		ComputeDescriptorByMatrix(pSubDesLineDes,pSubDesMatrix,nMaxRegionNum*4,nValid);
		for(int g = 0; g < nMaxRegionNum*8; g++)
		{
			m_scDes[nNo*m_nDesDim+g] = (float)pSubDesLineDes[g];
		}

		//***************************************************
		//		5	释放内存
		//***************************************************
		wzhFreePointer(pSubDesMatrix);
	}
	wzhFreePointer(pSubDesLineDes);
}

void  CDescriptor::ComputeSubRegionProjectionZeroScale(double* pSubRegionDes,double dMainArc,int nCenterR,int nCenterC)
{
	//取出9类小区域内的的梯度
	int nMainAngle = (int)(dMainArc*180/PI);
	double* pDataDx = new double[nMaxRegionNum*nEachRPixes];
	double* pDataDy = new double[nMaxRegionNum*nEachRPixes];
	for(int i=0; i<nMaxRegionNum; i++)
		for(int j=0; j<nEachRPixes; j++)
		{
			int k = i*nEachRPixes + j;
			int rr = LUTSubRegion[nMainAngle][k].nNo1 + nCenterR;
			int cc = LUTSubRegion[nMainAngle][k].nNo2 + nCenterC;
			int kk = rr*m_nWidth+cc;

			if(kk < 0 || kk > m_nTotolPixels-1)
			{
				continue;
			}
			pDataDx[k] = m_pDxImage[kk];
			pDataDy[k] = m_pDyImage[kk];
		}

		//主方向
		double dLineVx = cos(dMainArc);
		double dLineVy = sin(dMainArc);

		//计算每一类的四个分量
		for(int i=0; i< 4*nMaxRegionNum; i++)
		{
			pSubRegionDes[i] = 0;
		}
		for(int i=0; i<nMaxRegionNum*nEachRPixes; i++)
		{
			//梯度加权
			double dx = pDataDx[i]*LUTWeight[i];
			double dy = pDataDy[i]*LUTWeight[i];
			double IP = dx*dLineVx + dy*dLineVy;
			double EP = dx*dLineVy - dy*dLineVx;

			//查表获得最接近的2区域和相应的权值
			int nNo1 = LUTBiPos[i].nNo1;
			int nNo2 = LUTBiPos[i].nNo2;
			double dCoe1 = LUTBiPos[i].dCoe1;
			double dCoe2 = LUTBiPos[i].dCoe2;

			//累加到区域1上
			if(IP > 0)
			{
				pSubRegionDes[4*nNo1]	 = pSubRegionDes[4*nNo1] + IP*dCoe1;
			}
			else
			{
				pSubRegionDes[4*nNo1+2]	 = pSubRegionDes[4*nNo1+2] + abs(IP*dCoe1);
			}
			if(EP > 0)
			{
				pSubRegionDes[4*nNo1+1]	 = pSubRegionDes[4*nNo1+1] + EP*dCoe1;
			}
			else
			{
				pSubRegionDes[4*nNo1+3]	 = pSubRegionDes[4*nNo1+3] + abs(EP*dCoe1);
			}

			//累加到区域2上
			if(IP > 0)
			{
				pSubRegionDes[4*nNo2]	 = pSubRegionDes[4*nNo2] + IP*dCoe2;
			}
			else
			{
				pSubRegionDes[4*nNo2+2]	 = pSubRegionDes[4*nNo2+2] + abs(IP*dCoe2);
			}
			if(EP > 0)
			{
				pSubRegionDes[4*nNo2+1]	 = pSubRegionDes[4*nNo2+1] + EP*dCoe2;
			}
			else
			{
				pSubRegionDes[4*nNo2+3]	 = pSubRegionDes[4*nNo2+3] + abs(EP*dCoe2);
			}

		}
		/***********************************************************************/
		//释放内存
		wzhFreePointer(pDataDx);
		wzhFreePointer(pDataDy);
}

/********************************************************************************
计算直线点描述子
********************************************************************************/
void  CDescriptor::ComputeSubRegionProjection(double* pSubRegionDes,float angle1,float angle2,int nCenterR,int nCenterC,double scale)
{
	// Initialize subregions
	for(int i=0; i< 4*nMaxRegionNum; i++)
	{
		pSubRegionDes[i] = 0;
	}
	
	int rounded_scale = wzhRound(scale);
	int block_width = nHScale*(rounded_scale+1);
	int block_height = block_width;
	// Calculate things that are general for all three subparts.
	getScaledSubRegionPointsZeroAngle(block_width, block_height);
	getWeightingTable(block_width, block_height,scale);


	int lowerLineR = wzhRound((double) nCenterR - scale/2 * cos(angle1));
	int lowerLineC = wzhRound((double) nCenterC - scale/2 * sin(angle1));

	// Project point to the two actual lines.
	int upperLineR = wzhRound(nCenterR - scale/2 * cos(angle2));
	int upperLineC = wzhRound(nCenterC - scale/2 * sin(angle2));

	// Only lowerLineR <= upperLineR && lowerLineC <= upperLineC or lowerLineR > upperLineR && lowerLineC > upperLineC occur, not the others
	if(lowerLineR >= upperLineR && lowerLineC >= upperLineC) {
		// Change everything
		int tmp_R = lowerLineR;
		int tmp_C = lowerLineC;
		float tmp_angle = angle1;

		lowerLineR = upperLineR;
		lowerLineC = upperLineC;
		angle1 = angle2;

		upperLineR = tmp_R;
		upperLineC = tmp_C;
		angle2 = tmp_angle;
	}

	// Find angle of the line
	float avg_angle = (angle1 + angle2)/2;

	ComputeSubRegionProjectionLowerHalf(pSubRegionDes,angle1,lowerLineR, lowerLineC,scale);

	ComputeSubRegionProjectionUpperHalf(pSubRegionDes,angle2,upperLineR, upperLineC,scale);

	ComputeSubRegionProjectionMiddle(pSubRegionDes,avg_angle, nCenterR, nCenterC,scale);
}

void CDescriptor::ComputeSubRegionProjectionMiddle(double* pSubRegionDes, float angle, int nCenterR, int nCenterC, double scale) {
	int rounded_scale = wzhRound(scale);
	int block_width = nHScale*(rounded_scale+1);
	int block_height = block_width;

	// Get LUT for variable subregion blockwidths
	getScaledSubRegionPoints(block_width, block_height,angle);

	double * pDataDx = new double[block_width*block_height];
	double * pDataDy = new double[block_width*block_height];
	
	// Get middle lines
	for(int j=0; j<block_width*block_height; j++)
	{
		int real_k = j + ((nMaxRegionNum-1)/2)*block_width*block_height;
		int rr = LUTSubRegionScale[real_k].nNo1 + nCenterR;
		int cc = LUTSubRegionScale[real_k].nNo2 + nCenterC;
		int kk = rr*m_nWidth+cc;

		if(kk < 0 || kk > m_nTotolPixels-1)
		{
			continue;
		}
		pDataDx[j] = m_pDxImage[kk];
		pDataDy[j] = m_pDyImage[kk];
	}

	double dLineVx = cos(angle);
	double dLineVy = sin(angle);

	// Do soft assignment?
	for(int i=0; i<block_width*block_height; i++)
	{
		int real_i = i + ((nMaxRegionNum-1)/2)*block_width*block_height;
		double dx = pDataDx[i]*LUTWeightScale[real_i];
		double dy = pDataDy[i]*LUTWeightScale[real_i];
		double IP = dx*dLineVx + dy*dLineVy;
		double EP = dx*dLineVy - dy*dLineVx;

		int nNo1 = LUTBiPosScale[real_i].nNo1;
		int nNo2 = LUTBiPosScale[real_i].nNo2;
		double dCoe1 = LUTBiPosScale[real_i].dCoe1;
		double dCoe2 = LUTBiPosScale[real_i].dCoe2;

		if(IP > 0)
		{
			pSubRegionDes[4*nNo1]	 = pSubRegionDes[4*nNo1] + IP*dCoe1;
		}
		else
		{
			pSubRegionDes[4*nNo1+2]	 = pSubRegionDes[4*nNo1+2] + abs(IP*dCoe1);
		}
		if(EP > 0)
		{
			pSubRegionDes[4*nNo1+1]	 = pSubRegionDes[4*nNo1+1] + EP*dCoe1;
		}
		else
		{
			pSubRegionDes[4*nNo1+3]	 = pSubRegionDes[4*nNo1+3] + abs(EP*dCoe1);
		}

		if(IP > 0)
		{
			pSubRegionDes[4*nNo2]	 = pSubRegionDes[4*nNo2] + IP*dCoe2;
		}
		else
		{
			pSubRegionDes[4*nNo2+2]	 = pSubRegionDes[4*nNo2+2] + abs(IP*dCoe2);
		}
		if(EP > 0)
		{
			pSubRegionDes[4*nNo2+1]	 = pSubRegionDes[4*nNo2+1] + EP*dCoe2;
		}
		else
		{
			pSubRegionDes[4*nNo2+3]	 = pSubRegionDes[4*nNo2+3] + abs(EP*dCoe2);
		}

	}
	delete[] pDataDx;
	delete[] pDataDy;
}

void CDescriptor::ComputeSubRegionProjectionUpperHalf(double* pSubRegionDes, float angle, int nCenterR, int nCenterC, double scale) {
	int rounded_scale = wzhRound(scale);
	int block_width = nHScale*(rounded_scale+1);
	int block_height = block_width;

	// Get LUT for variable subregion blockwidths
	getScaledSubRegionPoints(block_width, block_height,angle+PI/2);

	//取出9类小区域内的的梯度
	double * pDataDx = new double[(nMaxRegionNum-1)/2*block_width*block_height];
	double * pDataDy = new double[(nMaxRegionNum-1)/2*block_width*block_height];

	// Get upper lines
	for(int i=0; i<(nMaxRegionNum-1)/2; i++) {
		for(int j=0; j<block_width*block_height; j++)
		{
			int k = i*block_width*block_height + j;
			int real_k = k + ((nMaxRegionNum-1)/2+1)*block_width*block_height;
			int rr = LUTSubRegionScale[real_k].nNo1 + nCenterR;
			int cc = LUTSubRegionScale[real_k].nNo2 + nCenterC;
			int kk = rr*m_nWidth+cc;

			if(kk < 0 || kk > m_nTotolPixels-1)
			{
				continue;
			}
			pDataDx[k] = m_pDxImage[kk];
			pDataDy[k] = m_pDyImage[kk];
		}
	}

	//主方向
	double dLineVx = cos(angle+PI/2);
	double dLineVy = sin(angle+PI/2);

	// Do soft assignment?
	for(int i=0; i<(nMaxRegionNum-1)/2*block_width*block_height; i++)
	{
		//梯度加权
		int real_i = i + ((nMaxRegionNum-1)/2+1)*block_width*block_height;
		double dx = pDataDx[i]*LUTWeightScale[real_i];
		double dy = pDataDy[i]*LUTWeightScale[real_i];
		double IP = dx*dLineVx + dy*dLineVy;
		double EP = dx*dLineVy - dy*dLineVx;

		//查表获得最接近的2区域和相应的权值
		int nNo1 = LUTBiPosScale[real_i].nNo1;
		int nNo2 = LUTBiPosScale[real_i].nNo2;
		double dCoe1 = LUTBiPosScale[real_i].dCoe1;
		double dCoe2 = LUTBiPosScale[real_i].dCoe2;

		//累加到区域1上
		if(IP > 0)
		{
			pSubRegionDes[4*nNo1]	 = pSubRegionDes[4*nNo1] + IP*dCoe1;
		}
		else
		{
			pSubRegionDes[4*nNo1+2]	 = pSubRegionDes[4*nNo1+2] + abs(IP*dCoe1);
		}
		if(EP > 0)
		{
			pSubRegionDes[4*nNo1+1]	 = pSubRegionDes[4*nNo1+1] + EP*dCoe1;
		}
		else
		{
			pSubRegionDes[4*nNo1+3]	 = pSubRegionDes[4*nNo1+3] + abs(EP*dCoe1);
		}

		//累加到区域2上
		if(IP > 0)
		{
			pSubRegionDes[4*nNo2]	 = pSubRegionDes[4*nNo2] + IP*dCoe2;
		}
		else
		{
			pSubRegionDes[4*nNo2+2]	 = pSubRegionDes[4*nNo2+2] + abs(IP*dCoe2);
		}
		if(EP > 0)
		{
			pSubRegionDes[4*nNo2+1]	 = pSubRegionDes[4*nNo2+1] + EP*dCoe2;
		}
		else
		{
			pSubRegionDes[4*nNo2+3]	 = pSubRegionDes[4*nNo2+3] + abs(EP*dCoe2);
		}

	}
	/***********************************************************************/
	delete[] pDataDx;
	delete[] pDataDy;
}

void CDescriptor::ComputeSubRegionProjectionLowerHalf(double* pSubRegionDes, float angle, int nCenterR, int nCenterC, double scale) {
	int rounded_scale = wzhRound(scale);
	int block_width = nHScale*(rounded_scale+1);
	int block_height = block_width;

	// Get LUT for variable subregion blockwidths
	getScaledSubRegionPoints(block_width, block_height,angle-PI/2);

	//取出9类小区域内的的梯度
	double * pDataDx = new double[(nMaxRegionNum-1)/2*block_width*block_height];
	double * pDataDy = new double[(nMaxRegionNum-1)/2*block_width*block_height];

	// Get lower lines
	for(int i=0; i<(nMaxRegionNum-1)/2; i++) {
		for(int j=0; j<block_width*block_height; j++)
		{
			int k = i*block_width*block_height + j;
			int rr = LUTSubRegionScale[k].nNo1 + nCenterR;
			int cc = LUTSubRegionScale[k].nNo2 + nCenterC;
			int kk = rr*m_nWidth+cc;

			if(kk < 0 || kk > m_nTotolPixels-1)
			{
				continue;
			}
			pDataDx[k] = m_pDxImage[kk];
			pDataDy[k] = m_pDyImage[kk];
		}	
	}

	//主方向
	double dLineVx = cos(angle-PI/2);
	double dLineVy = sin(angle-PI/2);

	// Do soft assignment?
	for(int i=0; i<(nMaxRegionNum-1)/2*block_width*block_height; i++)
	{
		//梯度加权
		double dx = pDataDx[i]*LUTWeightScale[i];
		double dy = pDataDy[i]*LUTWeightScale[i];
		double IP = dx*dLineVx + dy*dLineVy;
		double EP = dx*dLineVy - dy*dLineVx;

		//查表获得最接近的2区域和相应的权值
		int nNo1 = LUTBiPosScale[i].nNo1;
		int nNo2 = LUTBiPosScale[i].nNo2;
		double dCoe1 = LUTBiPosScale[i].dCoe1;
		double dCoe2 = LUTBiPosScale[i].dCoe2;

		//累加到区域1上
		if(IP > 0)
		{
			pSubRegionDes[4*nNo1]	 = pSubRegionDes[4*nNo1] + IP*dCoe1;
		}
		else
		{
			pSubRegionDes[4*nNo1+2]	 = pSubRegionDes[4*nNo1+2] + abs(IP*dCoe1);
		}
		if(EP > 0)
		{
			pSubRegionDes[4*nNo1+1]	 = pSubRegionDes[4*nNo1+1] + EP*dCoe1;
		}
		else
		{
			pSubRegionDes[4*nNo1+3]	 = pSubRegionDes[4*nNo1+3] + abs(EP*dCoe1);
		}

		//累加到区域2上
		if(IP > 0)
		{
			pSubRegionDes[4*nNo2]	 = pSubRegionDes[4*nNo2] + IP*dCoe2;
		}
		else
		{
			pSubRegionDes[4*nNo2+2]	 = pSubRegionDes[4*nNo2+2] + abs(IP*dCoe2);
		}
		if(EP > 0)
		{
			pSubRegionDes[4*nNo2+1]	 = pSubRegionDes[4*nNo2+1] + EP*dCoe2;
		}
		else
		{
			pSubRegionDes[4*nNo2+3]	 = pSubRegionDes[4*nNo2+3] + abs(EP*dCoe2);
		}

	}
	/***********************************************************************/
	//释放内存
	/*wzhFreePointer(pDataDx);
	wzhFreePointer(pDataDy);*/
	delete[] pDataDx;
	delete[] pDataDy;
}

/********************************************************************************
根据描述子矩阵计算描述子
********************************************************************************/
void  CDescriptor::ComputeDescriptorByMatrix(double* pLineDes,double* pMatrix,int nD,int nValid)
{
	//计算均值
	// Calculate average of the described subregions (= M in MSLD)
	// M(GDM(L))//||M(GDM(L))||
	double* pAvg = new double[nD];
	wzhSet(pAvg,0,nD);
	for(int i = 0; i < nD; i++)
	{
		for(int j = 0; j < nValid; j++)
		{
			int k = j*nD + i;
			pAvg[i] = pAvg[i] + pMatrix[k];
		}
	}
	for(int i = 0; i < nD; i++)
	{
		pAvg[i] = pAvg[i]/nValid;
	}

	//计算标准差
	// Calculate the standard deviation of the subregions
	// S(GDM(L))//||S(GDM(L))||
	double* pStd = new double[nD];
	wzhSet(pStd,0,nD);
	for(int i = 0; i < nD; i++)
	{
		for(int j = 0; j < nValid; j++)
		{
			int k = j*nD + i;
			double dVar = (pMatrix[k]-pAvg[i])*(pMatrix[k]-pAvg[i]);
			pStd[i] = pStd[i] + dVar;
		}
	}
	for(int i = 0; i < nD; i++)
	{
		pStd[i] = sqrt(pStd[i]/nValid);
	}

	//分别归一化
	wzhNormorlizeNorm(pAvg,nD);
	wzhNormorlizeNorm(pStd,nD);

	//描述子
	// Limit values of unit feature vector to max. 0.4. This reduces the influence of non-linear illumination.
	for(int i = 0; i < nD; i++)
	{
		if(pAvg[i] < 0.4)
			pLineDes[i]		= pAvg[i];
		else
			pLineDes[i]		= 0.4;
		if(pStd[i] < 0.4)
			pLineDes[nD+i]	= pStd[i];
		else
			pLineDes[nD+i]	= 0.4;
	}

	wzhNormorlizeNorm(pLineDes,2*nD);

	//释放内存
	wzhFreePointer(pStd);
	wzhFreePointer(pAvg);
}

double  CDescriptor::ComputeLineDir(double* pLinePts,int nCount,double dDxAvg, double dDyAvg)
{
	//利用最小二乘技术求方向
	initM(MATCOM_VERSION);
	Mm mMatrix = zeros(nCount,3);
	for(int g1 = 0; g1 < nCount; g1++)
	{
		mMatrix.r(g1+1,1) = pLinePts[2*g1];
		mMatrix.r(g1+1,2) = pLinePts[2*g1+1];
		mMatrix.r(g1+1,3) = 1;
	}

	//奇异值分解获得精确位置
	Mm u,s,v;
	i_o_t i_o = {0,0};
	svd(mMatrix,i_o,u,s,v);

	//计算方向
	double a = v.r(1,3);
	double b = v.r(2,3);
	double dMainArc = atan2(-b,a);
	dMainArc = LimitArc(dMainArc);

	//退出
	exitM();

	//判定方向
	double dMainArc1 = dMainArc - PI/2;
	dMainArc1 = LimitArc(dMainArc1);
	double dMainArc2 = dMainArc + PI/2;
	dMainArc2 = LimitArc(dMainArc2);
	double dAvgArc = atan2(-dDyAvg,-dDxAvg);
	dAvgArc = LimitArc(dAvgArc);

	double error1 = ArcDis(dMainArc1,dAvgArc);
	double error2 = ArcDis(dMainArc2,dAvgArc);

	//返回最终方向
	double nArcReturn = dMainArc1;
	if(error1 > error2)
		nArcReturn = dMainArc2;

	return nArcReturn;
}