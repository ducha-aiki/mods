//Match.cpp
//#include "stdafx.h"
#include "Match.h"
#include "descriptor.h"
#include "PointsDesc.h"

void matchDes(	double* matchPs,int& nMatchCount,int nDim,double fDistinctive,
				float* pDes1,int nCount1,char* pValidFlag1,int szCountForEachLine1[], 
				float* pDes2,int nCount2,char* pValidFlag2,int szCountForEachLine2[])
{
	nMatchCount = 0;

	//计算两两距离
	float* pScore = new float[nCount1*nCount2];
	for(int i = 0; i < nCount1; i++)
		for(int j = 0;j < nCount2; j++)
		{
			int k = i*nCount2 + j;

			//验证点数
			double dL1 = szCountForEachLine1[i];
			double dL2 = szCountForEachLine2[j];
			double dLMax = max(dL1,dL2);
			double dLMin = min(dL1,dL2);
			double dError = (dLMax-dLMin)/dLMax;
			if(dError>0.7 || dLMin < 20)
			{
				pScore[k] = 10000.0f;
				continue;
			}
			
			//验证描述子有效性
			if( pValidFlag1[i] == 0 || pValidFlag2[j] == 0)
			{
				pScore[k] = 10000.0f;
				continue;
			}

			//计算相似性
			pScore[k] = wzhDistance(&pDes1[i*nDim],&pDes2[j*nDim],nDim);
		}

		//************************************************************
		int nDebug = 0;
		if(nDebug == 1)
		{
			wzhOut("d:\\pDes1.txt",pDes1,nDesDim,nCount1);
			wzhOut("d:\\pDes2.txt",pDes2,nDesDim,nCount2);
			wzhOut("d:\\pScore.txt",pScore,nCount1,nCount2);
			float * pTemp1 = new float[nCount2];
			for(int nTemp = 0;nTemp<nCount2;nTemp++)
			{
				pTemp1[nTemp] = pValidFlag2[nTemp];
			}
			wzhOut("d:\\pValidFlag1.txt",pTemp1,1,nCount2);
			delete pTemp1;
		}
		//************************************************************

#if 1
	//通过最小值比次小值进行匹配
	for(int i = 0; i < nCount1; i++)
	{
			if(pValidFlag1[i] == 0)
			{
				continue;
			}

			//求出最小得分与次小得分
			float score1 = 100000.0f;
			float score2 = 100000.0f;
			int nNo1	 = -1;
			int nNo2	 = -1;
			for(int j = 0; j < nCount2; j++)
			{
				if(pValidFlag2[j] == 0)
				{
					continue;
				}
				int k = (i*nCount2 + j);
				float score = pScore[k];
				if(score < score1)
				{
					score2 = score1;
					nNo2 = nNo1;
					score1 = score;
					nNo1 = j;		
				}
				else if(score < score2)
				{
					score2 = score;
					nNo2 = j;
				}
			}
			
			if(score1 > 1 || score2 < 0.000001)
			{
				continue;
			}

			//如果小于阈值,则匹配成功
			if(score1/score2 <  fDistinctive && score1 < 0.5)
			{
				matchPs[2*nMatchCount] = i;
				matchPs[2*nMatchCount+1] = nNo1;
				nMatchCount ++;
			}
	}
#else
		for(int i = 0; i < nCount1; i++)
		{
			if(pValidFlag1[i] == 0)
			{
				continue;
			}

			//求出最小匹配
			float score1 = 100000.0f;
			int nNo1	 = -1;
			for(int j = 0; j < nCount2; j++)
			{
				if(pValidFlag2[j] == 0)
				{
					continue;
				}
				int k = (i*nCount2 + j);
				float score = pScore[k];
				if(score < score1)
				{
					score1 = score;
					nNo1 = j;
				}
			}

			//反向求最小匹配
			bool bFlag = true;
			for(int iii = 0; iii < nCount1; iii++)
			{
				int k = (iii*nCount2 + nNo1);
				float score = pScore[k];
				if(score < score1)
				{
					bFlag = false;
					break;
				}
			}

			//
			if(bFlag && score1 < 0.5)
			{
				matchPs[2*nMatchCount] = i;
				matchPs[2*nMatchCount+1] = nNo1;
				nMatchCount ++;
			}
		}
#endif
	//释放内存
	wzhFreePointer(pScore);
}

void ComputeDes(float*& pDes,char*& pByValidFlag,
				double* pImageData,int nWidth,int nHeight,
				double* pLinePts,int nLineCount,int szLinePtsCounts[],float scalesForEachLine[],float angleForEachLine[])
{
	CDescriptor des(pImageData,nWidth,nHeight,pLinePts,nLineCount,szLinePtsCounts,scalesForEachLine,angleForEachLine);
	des.ComputeLineDescriptor();
	int nDim = des.m_nDesDim;
	memcpy(pDes,des.m_scDes,sizeof(float)*nDim*nLineCount);
	memcpy(pByValidFlag,des.m_pByValidFlag,sizeof(char)*nLineCount);	
}

bool ValidFrelation(float L1[4],float L2[4])
{
	float F[9] = {0};
	//计算两条极线
	float e_L1[3];
	float e_L2[3];
	e_L1[0] = float(L2[0]*F[0]+ L2[1]*F[1] + F[2]);
	e_L1[1] = float(L2[0]*F[3]+ L2[1]*F[4] + F[5]);
	e_L1[2] = float(L2[0]*F[6]+ L2[1]*F[7] + F[8]);
	e_L2[0] = float(L2[2]*F[0]+ L2[3]*F[1] + F[2]);
	e_L2[1] = float(L2[2]*F[3]+ L2[3]*F[4] + F[5]);
	e_L2[2] = float(L2[2]*F[6]+ L2[3]*F[7] + F[8]);

	//端点
	float P1[2];
	float P2[2];
	P1[0] = L2[0];
	P1[1] = L2[1];
	P2[0] = L2[2];
	P2[1] = L2[3];

	float e1_d1 = P2LDis(P1,e_L1);
	float e1_d2 = P2LDis(P2,e_L1);
	float e2_d1 = P2LDis(P1,e_L2);
	float e2_d2 = P2LDis(P2,e_L2);

	float min1 = min(e1_d1,e1_d2);
	float min2 = min(e2_d1,e2_d2);

	if(min1 > 20 || min2 > 20)
	{
		return FALSE;
	}
	return TRUE;
}

float P2LDis(float P[2],float L[3])
{
	float norm_L = (float)sqrt(L[0]*L[0] + L[1]*L[1]) + (float)0.00001;
	float score = abs(P[0]*L[0]+P[1]*L[1]+L[2])/norm_L;
	return score;
}