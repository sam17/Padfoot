#include "stdafx.h"
#include "MAT.h"


MAT::MAT(void): weight(64,1.0)
{
	lambda = LAMBDA_VALUE;
	alpha = ALPHA_VALUE;
	distFactor = DIST_FACTOR;
	pd = Patch();
}
/**
Method to get patches from boundary box in frame
@param img: MAT object of frame
@param boundaryBox: Rectangle object of a pedestrian bounding box from which patches are extracted
**/
vector<Patch> MAT::getPatches(Mat img,Rect boundaryBox)
{
	vector<Patch> lpatches;
	Mat p;
	Rect r;
	Point2d jumpSize = (-boundaryBox.tl()+boundaryBox.br());
	jumpSize.x = jumpSize.x / PATCH_FACTOR;
	jumpSize.y = jumpSize.y / PATCH_FACTOR;

	for(k=0;k<8;k++)
	{
		for(int l=0;l<8;l++)
		{
			r = Rect(boundaryBox.tl().x+l*jumpSize.x,boundaryBox.tl().y+k*jumpSize.y,8,8);
			p = img(r);
			Patch pd(p,r);		
			lpatches.push_back(pd);
		}
	}
	return lpatches;
}
/**
Method to find histogram intersection distance between to patches
@param a,b: Patches whose histogram is calculated and then histogram intersection distance is calculated
**/
double MAT::findHistogramIntersection(Patch a,Patch b)
{
	double intersection=0.0;
	intersection = compareHist(a.patchHistogram,b.patchHistogram,CV_COMP_INTERSECT);
	return intersection;
}
/**
Method to find the area of intersection between two rectangles
@param box1,box2: Rectangle objects whose intersection is to be found
**/
double MAT::findIntersectionArea(Rect box1,Rect box2)
{
	double area = MAX(0, MIN(box1.br().x, box2.br().x)-MAX(box1.tl().x, box2.tl().x)) * MAX(0, -MAX(box1.tl().y, box2.tl().y) + MIN(box1.br().y, box2.br().y));
	return area;
}
/**
Method to find union area between two rectangles
@param box1,box2: Rectangle objetcs whose union is to be found
**/
double MAT::findUnionArea(Rect box1,Rect box2)
{
	double area = box1.area() + box2.area() - findIntersectionArea(box1,box2);
	return area;
}
/**
Method to calculate cost between a tracker and candidate detection
@param img: MAT object of the current frame
@param i,j: i is tracker pedestrians and j is candidate detection with which cost is calculated
@param globalV: global velocity of tracker pedestrian 
**/
double MAT::calculateCost(Mat img,pedestrian& i,pedestrian& j,Point2f globalV)
{
	
	iPatches = getPatches(img,i.rect);
	jPatches = getPatches(img,j.rect);


	vector<double> histIntersection;
	double minInteresction = -LDBL_MAX,value,sumG=0.0,sumH=0.0;
	int min_i,min_j;
	for(size_t x =0;x<iPatches.size();x++)
	{
		for(size_t y=0;y<jPatches.size();y++)
		{
			if(x==y)
				histIntersection.push_back(findHistogramIntersection(iPatches[x],jPatches[y]));
			value = findHistogramIntersection(iPatches[x],jPatches[y]);
			if(value>=minInteresction)
			{
				minInteresction = value;
				min_i = x;
				min_j = y;
			}
		}
		iPatches[x].setPatchVelocity((iPatches[min_i].center-jPatches[min_j].center) * VELOCITY_FACTOR);
		iPatches[x].setTheta(globalV);
		iPatches[x].calculateG();
		sumG+=iPatches[x].levelG;
	}
	for(size_t x =0;x<iPatches.size();x++)
	{
		weight[x] = ((alpha * weight[x]) + iPatches[x].levelG)/(alpha + sumG);
		sumH+=histIntersection[x];
	}
	alpha = (alpha + sumG)*0.5;
	
	double val;
	
	double intersection = findIntersectionArea(i.rect,j.rect);
	double uni = findUnionArea(i.rect,j.rect);
	val = 1- (intersection/uni);

	double latterTerm = 0.0;

	for (k=0; k<iPatches.size(); k++)
	{
		//double histTerm = findHistogramIntersection(
		if(sumH!=0.0) latterTerm += weight[k] * ( 1 -  (histIntersection[k]/sumH));
	}

	val = val + lambda * latterTerm;
	
	Point2d diff=(i.center-j.center);
	val+=distFactor*cvRound(sqrt(diff.x*diff.x+diff.y*diff.y));

	return val;
}
MAT::~MAT(void)
{
}
