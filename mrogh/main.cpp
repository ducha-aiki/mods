/*
An implementation of MROGH descriptor

For more information, refer to:

Bin Fan, Fuchao Wu and Zhanyi Hu, Aggregating Gradient Distributions into Intensity Orders: A Novel Local Image Descriptor,
<EM>CVPR 2011</EM>,pp.2377-2384.

Copyright (C) 2011 Bin Fan <bfan@nlpr.ia.ac.cn> 
All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
at your option any later version.
See the GNU General Public License for more details.

*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "mrogh.h"
#include <ctime>


int main(int argc, char** argv)
{
	char *im_file = 0;
	char *feat_file = 0;
	char *out_file= 0;
	int nDir = 8, nOrder = 6, nMultiRegion = 4;
	int m_Dim = nDir * nOrder * nMultiRegion;

	int counter = 1;
	while( counter < argc )
	{
		if( !strcmp("-i", argv[counter] ))
		{
			im_file = argv[++counter];
			counter++;
			continue;
		}
		if( !strcmp("-f", argv[counter] ))
		{
			feat_file = argv[++counter];
			counter++;
			continue;
		}
		if( !strcmp("-o", argv[counter] ))
		{
			out_file = argv[++counter];
			counter++;
			continue;
		}
		if( !strcmp("-Dir", argv[counter] ) )
		{
			nDir = atoi(argv[++counter]);
			counter++;
			continue;
		}
		if( !strcmp("-Order", argv[counter] ) )
		{
			nOrder = atoi(argv[++counter]);
			counter++;
			continue;
		}
		if( !strcmp("-R", argv[counter] ) )
		{
			nMultiRegion = atoi(argv[++counter]);
			counter++;
			continue;
		}
		exit(1);
	}

	/* do the job */

	m_Dim = nDir * nOrder * nMultiRegion;

	clock_t start,final;
	start = clock();

	int m_nKeys = 0;
	OxKey *m_pKeys = ReadKeyFile(feat_file,m_nKeys);

	CalcuTrans(m_pKeys,m_nKeys);

	IplImage* m_pImg = cvLoadImage(im_file,CV_LOAD_IMAGE_GRAYSCALE);
	cvSmooth(m_pImg,m_pImg,CV_GAUSSIAN,5,5,1);

	FILE *fid = fopen(out_file,"wt");
	fprintf(fid,"%d\n%d\n",m_Dim,m_nKeys);
	int i;
	for (i = 0;i < m_nKeys;i++)
	{
		int *desc = 0;
		desc = Extract_MROGH(m_pKeys[i],m_pImg,nDir,nOrder,nMultiRegion);
		if ( !desc )	continue;
		fprintf(fid,"%f %f %f %f %f",m_pKeys[i].x,m_pKeys[i].y,m_pKeys[i].a,m_pKeys[i].b,m_pKeys[i].c);
		for (int j = 0;j < m_Dim;j++)
		{
			fprintf(fid," %d",desc[j]);
		}
		fprintf(fid,"\n");
		delete [] desc;
	}
	fclose(fid);

	final = clock();
	printf("\nUsed %lf seconds\n", (double)(final - start) / CLOCKS_PER_SEC);

	cvReleaseImage(&m_pImg);

	delete [] m_pKeys;

	return 0;
}