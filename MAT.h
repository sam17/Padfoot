#pragma once

#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include "Patch.h"

using namespace cv;
using namespace std; 

#define VELOCITY_FACTOR 1.0
#define LAMBDA_VALUE 1.0
#define ALPHA_VALUE 1.0
#define PATCH_FACTOR 8
#define DIST_FACTOR 0.5
 
class MAT
{

	size_t k;	//Variable to be used as counter in loops
	//Parameters used in calculating cost
	double alpha;
	double lambda;
	double distFactor;
	//Vectors to store all patches of the candidate detection and tracker
	vector<Patch> iPatches;
	vector<Patch> jPatches;
	Patch pd;
	//Vector to store weight of every patch
	vector<double> weight;
	 
public:
	
	MAT(void);
	~MAT(void);

	/**
	Method to find histogram intersection distance between to patches
	@param a,b: Patches whose histogram is calculated and then histogram intersection distance is calculated
	**/
	double findHistogramIntersection(Patch a,Patch b);
	/**
	Method to get patches from boundary box in frame
	@param img: MAT object of frame
	@param boundaryBox: Rectangle object of a pedestrian bounding box from which patches are extracted
	**/
	vector<Patch> getPatches(Mat img,Rect boundaryBox);
	/**
	Method to find the area of intersection between two rectangles
	@param box1,box2: Rectangle objects whose intersection is to be found
	**/
	double findIntersectionArea(Rect box1,Rect box2);
	/**
	Method to find union area between two rectangles
	@param box1,box2: Rectangle objetcs whose union is to be found
	**/
	double findUnionArea(   Rect box1,Rect box2);
	/**
	Method to calculate cost between a tracker and candidate detection
	@param img: MAT object of the current frame
	@param i,j: i is tracker pedestrians and j is candidate detection with which cost is calculated
	@param globalV: global velocity of tracker pedestrian 
	**/
	double calculateCost(Mat img,pedestrian &i,pedestrian &j,Point2f globalV);
};

