#pragma once

#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include "pedestrian.hpp"
#include "Drawing.h"

using namespace cv;
using namespace std;

class PDetector
{
private:
	vector<pedestrian> pedestrians;
	Drawing *paint;
	cv::HOGDescriptor hog;
	size_t i,j;
	
	/**
	Method used to 
	@param img: MAT object of the frame where pedestrians are detected
	@param pedestrians: Vector in which all detected pedestrians are stored 
	**/
	void DetectPedestrians(Mat& img, vector<pedestrian> &pedestrians);

public:
	
	PDetector(Drawing *painter);
	~PDetector(void);

	/**
	Method used to call Detect pedestrians to return list of pedestrians in frame
	@param img: image in which pedestrians need to be detected
	**/
	vector<pedestrian> Detect(Mat& img);
};
