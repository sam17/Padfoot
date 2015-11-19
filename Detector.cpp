#include "stdafx.h"
#include "Detector.h"

using namespace cv;
using namespace std;

PDetector::PDetector(Drawing *painter)
{
	paint = painter;
	//Detect Pedestrians using HOG using Daimler detector of OpenCV
	//Parameters set are default tuning for Daimler detector
	hog = HOGDescriptor(cv::Size(48, 96), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9, 1,-1, cv::HOGDescriptor::L2Hys, 0.2, true, cv::HOGDescriptor::DEFAULT_NLEVELS);
	hog.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());
	vector<Rect> rects;
	vector<Point2d> centers;

}
/**
Method used to 
@param img: MAT object of the frame where pedestrians are detected
@param pedestrians: Vector in which all detected pedestrians are stored 
**/
void PDetector::DetectPedestrians(Mat& img,vector<pedestrian>& pedestrians)
{
	pedestrians.clear();
	pedestrian p;
	vector<Rect> found;
	hog.detectMultiScale(img, found, 1.5, cv::Size(8,8), cv::Size(0,0), 1.05, 2);
		
	for (i=0; i<found.size(); i++)
	{
		cv::Rect r = found[i];
		
		p.rect = r;
		p.center = (r.tl()+r.br())*0.5;

		for (j=0; j<found.size(); j++)
			if (j!=i && (r & found[j])==r)
				break;	
		if (j==found.size())	
		{
			p.center = (r.br()+r.tl())*0.5;
			pedestrians.push_back(p);
		}
	}

	paint->drawRectanglefromDetection(img,pedestrians);
}
/**
Method used to call Detect pedestrians to return list of pedestrians in frame
@param img: image in which pedestrians need to be detected
**/
vector<pedestrian> PDetector::Detect(Mat& img)
{
		DetectPedestrians(img,pedestrians);
		return pedestrians;
}

PDetector::~PDetector(void)
{

}
