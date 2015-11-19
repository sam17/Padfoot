#include "stdafx.h"
#include "kalmanTracker.h"


kalmanTracker::kalmanTracker(Point2f pd,float dt,float Accel_noise_mag)
{
	deltatime = dt;
	kalman = new KalmanFilter( 6, 2, 0 );
	
	kalman->transitionMatrix = *(Mat_<float>(6, 6) << 1,0,1,0,0.5,0,   0,1,0,1,0,0.5,  0,0,1,0,1,0,  0,0,0,1,0,1,  0,0,0,0,1,0,  0,0,0,0,0,1 );
	//initialisations
	LastResult = pd;

	kalman->statePre.at<float>(0) = pd.x; // x
	kalman->statePre.at<float>(1) = pd.y; // y

	kalman->statePre.at<float>(2) = 0;
	kalman->statePre.at<float>(3) = 0;

	kalman->statePre.at<float>(4) = 0;
	kalman->statePre.at<float>(5) = 0;

	kalman->statePost.at<float>(0)=pd.x;
	kalman->statePost.at<float>(1)=pd.y;

	setIdentity(kalman->measurementMatrix);	
		
	setIdentity(kalman->processNoiseCov, Scalar::all(1e-5));

	kalman->processNoiseCov*=Accel_noise_mag;

	setIdentity(kalman->measurementNoiseCov, Scalar::all(0.1));
	 
	setIdentity(kalman->errorCovPost, Scalar::all(.1));
}
kalmanTracker::~kalmanTracker(void)
{
	delete kalman;
}

/**
Method to update the predicted data by kalman and store in the Kalman object
**/
Point2f kalmanTracker::GetPrediction()
{
	Mat prediction = kalman->predict();
	LastResult = Point2f(prediction.at<float>(0),prediction.at<float>(1));
	return LastResult;
}
/**
Method to get velocity of center of the pedestrian for MAT
**/
Point2d kalmanTracker::GetCenterVelocity()
{
	Point2d v;	
	v.x = kalman->statePost.at<float>(4);
	v.y = kalman->statePost.at<float>(5);
	return v;	
}

/**
Method that updates kalman filter data from measured data
@param p: Point that is measured
@param DataCorrect: Boolean value to tell function if reading of p is from current measurement or not
**/
Point2f kalmanTracker::Update(Point2f p, bool DataCorrect)
{
	Mat measurement(2,1,CV_32FC1);
	if(!DataCorrect)
	{
		measurement.at<float>(0) = LastResult.x;
		measurement.at<float>(1) = LastResult.y;
	}
	else
	{
		measurement.at<float>(0) = p.x;
		measurement.at<float>(1) = p.y;
	}
	
	Mat estimated = kalman->correct(measurement);
	LastResult = Point2f(estimated.at<float>(0),estimated.at<float>(1));

	return LastResult;	
}
