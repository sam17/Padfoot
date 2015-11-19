#pragma once

#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include <iostream>
#include <vector>
#include <math.h>

//Structure that stores a pedestrian data including it's boundary box and center
struct pedestrian
{
	cv::Rect rect;
	cv::Point2f center;
	
	pedestrian() { }
	pedestrian(cv::Point2f p,cv::Rect r): rect(r),center(p) { }
};



