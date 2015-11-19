#pragma once
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include "Drawing.h"


#define DEFAULTWEIGHT 1.0
#define PI atan(1.0)*4

using namespace cv;
using namespace std; 

class Patch
{

		//Patch data
		Mat patch;
		/**
		Method to create histogram from patch
		**/
		void createHistogram();

	public:

		//Velocity of Patch
		Point2f patchVelocity;
		//Histogram
		Mat patchHistogram;
		//center of patch
		Point2d center;
		//weight of patch
		double weight;
		//Levelg value
		int levelG;
		//theta value
		double theta;

		Patch(Mat &p,Rect &r);
		Patch();
		
		void setPatchVelocity(Point2f v);
		/**
		Method to set the thea value of a patch from global velocity
		@param globalV: Global Velocity of pedestrian from Kalman
		**/
		void setTheta(Point2f globalV);
		/**
		Method to calculate g value of a patch
		**/
		void calculateG(void);
				
		~Patch();
};




