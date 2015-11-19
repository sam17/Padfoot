#pragma once

#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include "pedestrian.hpp"

using namespace cv;
using namespace std;

//Class that consists of all the methods to draw on the frame
class Drawing
{
	size_t i;	//Variable to use as counter in loops
public:

	Drawing(void);
	~Drawing(void);
	/**
	Method to draw cross of pedestrian center
	@param img: MAT object of frame to be drawn on
	@param center: Point object of the center where cross has to eb drawn
	@param color: Scalar object to set color of cross
	@param d: Parameter to determine size of cross
	**/
	void drawCross(Mat &img, cv::Point center, CvScalar color, int d);
	/**
	Method to draw bounding boxes of all detections
	@param img: MAT object of frame to be drawn on
	@param pedestrians: Vector of pedestrians that consists of all the bounding box coordinates of detected pedestrians
	**/
	void drawRectanglefromDetection(Mat &img,vector<pedestrian> &pedestrians);
	/**
	Method to label the trackers
	@param im:  MAT object of frame to be drawn on
	@param r: Rectangle that needs to be labelled
	@param label: String to be written on label
	**/
	void set_label(cv::Mat& im, cv::Rect r, const std::string label);
	/**
	Method to draw bounding boxes of all trackers
	@param img: MAT object of frame to be drawn on
	@param r: Rectangle object to be drawn
	**/
	void drawRectanglefromTracking(Mat &img,Rect r);

};

