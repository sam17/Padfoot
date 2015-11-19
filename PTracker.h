#pragma once

#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "kalmanTracker.h"
#include "AssignmentProblemSolver.h"
#include "Detector.h"
#include "pedestrian.hpp"
#include "MAT.h"

using namespace cv;
using namespace std;

//Class that maintains all the trackers of pedestrians
class PTrack


{
public:

	vector<Point2f> trace; //Vector of points that stores the trajectory of tracker
	static size_t NextTrackID; 
	size_t track_id; //Counter for TrackID of trackers
	size_t skipped_frames; //Count of the number of frames where track not found 
	pedestrian predictedPedestrian; //Predicted pedestrian data
	kalmanTracker* KF; 
	PTrack(Point2f p, float dt, float Accel_noise_mag);
	~PTrack();
};
//Class that maintains the methods to update the tracks
class PTracker
{
public:
	//Paramaeters for tracking
	float dt; 
	float Accel_noise_mag;
	double dist_thres;
	int maximum_allowed_skipped_frames;
	int max_trace_length;
	//Object for MAT
	MAT *agreementTracking;
	//Vector for all tracks
	vector<PTrack*> tracks;
	/**
	Method that updates all teh trackers from detection in every frame
	@param img: MAT object of the current frame
	@param detectedPedestrians: Vector containing the pedestrians from detector
	**/
	void Update(const Mat img,vector<pedestrian>& detectedPedestrians);
	
	PTracker(float _dt, float _Accel_noise_mag, double _dist_thres=60, int _maximum_allowed_skipped_frames=10,int _max_trace_length=10);
	~PTracker(void);
};

