#pragma once

#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "pedestrian.hpp"

using namespace cv;
using namespace std;

class kalmanTracker
{
public:
	KalmanFilter* kalman; 
	double deltatime; 
	Point2f LastResult;
	
	kalmanTracker(Point2f p,float dt=0.2,float Accel_noise_mag=0.5);
	~kalmanTracker();
	
	/**
	Method to update the predicted data by kalman and store in the Kalman object
	**/
	Point2f GetPrediction();
	/**
	Method to get velocity of center of the pedestrian for MAT
	**/
	Point2d GetCenterVelocity();
	/**
	Method that updates kalman filter data from measured data
	@param p: Point that is measured
	@param DataCorrect: Boolean value to tell function if reading of p is from current measurement or not
	**/
	Point2f Update(Point2f p, bool DataCorrect);
};
