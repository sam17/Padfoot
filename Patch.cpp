#include "stdafx.h"
#include "Patch.h"

/**
Method to create histogram from patch
**/
void Patch::createHistogram()
{
	Mat p = this->patch;
	cvtColor(p,p,CV_BGR2HSV);
	Mat hist;
	int histSize = 64;
	float range[] = { 0, 256 } ; //the upper boundary is exclusive
	int dimRange[] = {0,1,2};
	const float* histRange = { range };
	const int* dimensions = { dimRange };
	calcHist(&p,1,dimensions,Mat(),hist,1,&histSize,&histRange,true,false);
	normalize(hist, hist, 0, 255, NORM_MINMAX, -1, Mat());
	this->patchHistogram = hist;
}

Patch::Patch():patch(),center()
{
	weight  = DEFAULTWEIGHT;
	levelG = 1;
	theta = 0.0;
}

Patch::Patch(Mat &p,Rect &r):patch(p),center((r.tl() + r.br()) * 0.5)
{
	//weight  = DEFAULTWEIGHT;
	levelG = 1;
	theta = 0.0;
	createHistogram();
}
/**
Method to calculate g value of a patch
**/
void Patch::calculateG()
{
	if(theta < PI/4)
		levelG = 2;
	else levelG = 0;
}
/**
Method to store the local velocity of a patch
@param v: velocity of patch
**/
void Patch::setPatchVelocity(Point2f v)
{
	patchVelocity = v;
}
/**
Method to set the thea value of a patch from global velocity
@param globalV: Global Velocity of pedestrian from Kalman
**/
void Patch::setTheta(Point2f globalV)
{
	theta = acos(patchVelocity.ddot(globalV)/(sqrt(globalV.x*globalV.x+globalV.y*globalV.y)*sqrt(patchVelocity.x*patchVelocity.x+patchVelocity.y*patchVelocity.y)));
}
Patch::~Patch(void)
{
}





