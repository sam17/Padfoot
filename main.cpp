// KalmanTracking.cpp : Defines the entry point for the console application.
//

/************************************************/
// Name: Soumyadeep Mukherjee
// Project Topic: Pedestrian Tracking
// Supervisor: Wu Meiqing
// Dates: 5 May to 17 July
// Contact: soumyadeep9@gmail.com
/************************************************/

//BUGS: Zero Tracks causes assertion failed.   FIXED

#include "stdafx.h"

#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <string>
#include "Detector.h"
#include "Drawing.h"
#include "AssignmentProblemSolver.h"
#include "kalmanTracker.h"
#include "PTracker.h"
#include "pedestrian.hpp"
#include "InterframeRegister.h"

using namespace cv;
using namespace std;


#define IMAGES

int main()
{
	
	cv::namedWindow("Input Image");

	cv::Mat img,temp,oldImg;
	Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127)};
	Drawing* painter = new Drawing();
	PDetector* detector=new PDetector(painter);

	PTracker tracker(0.2,10.0,90.0,20,10);

	vector<pedestrian> pedestrians;

	InterframeRegister reg;
	Mat pCurrFrame,pHistFrame,pHistMat,pAccumIFMat,pResultMat;


	for(int no=2;no<=5700;no++)
	{
		//File Traversing in Folder
		char* fileName_pre = "c1/A (";
		//char *fileName_pre = "lp1-left/image_0000";
		char filename[1024]; 
		sprintf(filename,"%s%d).pgm",fileName_pre,no);
		img = cv::imread(filename);
		sprintf(filename,"%s%d).pgm",fileName_pre,no-1);
		oldImg=imread(filename);

		/***************************************/
		// EGO-MOTION CALCULATION IN CONSECUTIVE FRAMES
		/***************************************/
		cvtColor(img,pCurrFrame,CV_BGR2GRAY);
		cvtColor(oldImg,pHistFrame,CV_BGR2GRAY);
		bool bIsInterframeBad = true;
		Mat pEstMat = reg.Register(pCurrFrame, pHistFrame, Mat(), bIsInterframeBad);

		/***************************************/
		// DETECION AND TRACKING
		/***************************************/
		
		temp = img.clone();
		//Detect Pedestrians
		pedestrians = detector->Detect(img);
		
	
		for(size_t i=0;i<pedestrians.size();i++)
		{
			pedestrians[i].center = Point2f(pEstMat.at<double>(0,0)*pedestrians[i].center.x+pEstMat.at<double>(0,1)*pedestrians[i].center.y+pEstMat.at<double>(0,2),pEstMat.at<double>(1,0)*pedestrians[i].center.x+pEstMat.at<double>(1,1)*pedestrians[i].center.y+pEstMat.at<double>(1,2));
			painter->drawCross(img,pedestrians[i].center,Scalar(0,0,255),3);
		}	
		
		//Update Kalman Tracker
		tracker.Update(temp,pedestrians);
		
		cout<<"Trackers: "<<tracker.tracks.size()<<" ";
		
		//Draw Lines of Traces of Tracks
		for(size_t i=0;i<tracker.tracks.size();i++)
		{
			if(tracker.tracks[i]->trace.size()>1)
			{
				for(size_t j=0;j<tracker.tracks[i]->trace.size()-1;j++)
				{
					line(img,tracker.tracks[i]->trace[j],tracker.tracks[i]->trace[j+1],Colors[tracker.tracks[i]->track_id%9],5,CV_AA);
				}		
			}
		}
		

		//Log the Output
		char *namePre = "output";
		char name[1024];
		sprintf(name,"%s%d.jpg",namePre,no);
		imwrite(name,img);
		cv::imshow("Input Image",img);
		no++;
		cv::waitKey(33);
	}

	delete detector;
	return 0;
}

