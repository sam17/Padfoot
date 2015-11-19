#include "stdafx.h"
#include "Drawing.h"

Drawing::Drawing()
{

}
/**
Method to draw cross of pedestrian center
@param img: MAT object of frame to be drawn on
@param center: Point object of the center where cross has to eb drawn
@param color: Scalar object to set color of cross
@param d: Parameter to determine size of cross
**/
void Drawing::drawCross(Mat &img, cv::Point center, CvScalar color, int d )
{
	cv::line( img, cv::Point( center.x - d, center.y - d ),cv::Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); 
	cv::line( img, cv::Point( center.x + d, center.y - d ),cv::Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 );
}
/**
Method to draw bounding boxes of all detections
@param img: MAT object of frame to be drawn on
@param pedestrians: Vector of pedestrians that consists of all the bounding box coordinates of detected pedestrians
**/
void Drawing::drawRectanglefromDetection(Mat &img,vector<pedestrian> &pedestrians)
{
		for (i=0; i<pedestrians.size(); i++)
		{
			cv::Rect r = pedestrians[i].rect;
			r.x += cvRound(r.width*0.1);
			r.width = cvRound(r.width*0.8);
			r.y += cvRound(r.height*0.06);
			r.height = cvRound(r.height*0.9);
			//Drawing Class Add
			rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
		}
}
/**
Method to draw bounding boxes of all trackers
@param img: MAT object of frame to be drawn on
@param r: Rectangle object to be drawn
**/
void Drawing::drawRectanglefromTracking(Mat &img,Rect r)
{
	rectangle(img,r.tl(),r.br(),Scalar(0,0,255),2);
}
/**
Method to label the trackers
@param im:  MAT object of frame to be drawn on
@param r: Rectangle that needs to be labelled
@param label: String to be written on label
**/
void Drawing::set_label(cv::Mat& im, cv::Rect r, const std::string label)
{
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.5;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::Point pt(r.x + (r.width-text.width)/2, r.y + (r.height+text.height)/2);

    cv::rectangle(
        im, 
        pt + cv::Point(0, baseline), 
        pt + cv::Point(text.width, -text.height), 
        CV_RGB(255,0,0), CV_FILLED
    );

    cv::putText(im, label, pt, fontface, scale, CV_RGB(255,255,0), thickness, 8);
}
Drawing::~Drawing(void)
{
}
