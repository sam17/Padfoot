#include "stdafx.h"
#include "PTracker.h"

size_t PTrack::NextTrackID=0;

PTrack::PTrack(Point2f p, float dt, float Accel_noise_mag)
{
	track_id=NextTrackID;

	NextTrackID++;
	KF = new kalmanTracker(p,dt,Accel_noise_mag);
	predictedPedestrian.center=p;
	skipped_frames=0;
}

PTrack::~PTrack()
{
	delete KF;
}

PTracker::PTracker(float _dt, float _Accel_noise_mag, double _dist_thres, int _maximum_allowed_skipped_frames,int _max_trace_length)
{
	dt=_dt;
	Accel_noise_mag=_Accel_noise_mag;
	dist_thres=_dist_thres;
	maximum_allowed_skipped_frames=_maximum_allowed_skipped_frames;
	max_trace_length=_max_trace_length;
	agreementTracking = new MAT();
}

/**
Method that updates all teh trackers from detection in every frame
@param img: MAT object of the current frame
@param detectedPedestrians: Vector containing the pedestrians from detector
**/
void PTracker::Update(const Mat img,vector<pedestrian>& detectedPedestrians)
{
	if(tracks.size()==0)		
	{
		for(size_t i=0;i<detectedPedestrians.size();i++)
		{
			PTrack* tr=new PTrack(detectedPedestrians[i].center,dt,Accel_noise_mag);
			tracks.push_back(tr);
		}	
	}

	int N=tracks.size();	
	int M=detectedPedestrians.size();	

	vector< vector<double> > Cost(N,vector<double>(M));
	vector<int> assignment; 

	double dist;
	Point2f v;
	

	for(size_t i=0;i<tracks.size();i++)
	{	
		v = tracks[i]->KF->GetCenterVelocity();
		for(size_t j=0;j<detectedPedestrians.size();j++)
		{
			dist = agreementTracking->calculateCost(img,tracks[i]->predictedPedestrian,detectedPedestrians[j],v);
			//cout<<"Dist: "<<dist;
			Cost[i][j]=dist;
		}
		//cout<<"\n";
	}
	AssignmentProblemSolver APS;
	APS.Solve(Cost,assignment,AssignmentProblemSolver::optimal);

	vector<int> not_assigned_tracks;

	for(size_t i=0;i<assignment.size();i++)
	{
		if(assignment[i]!=-1)
		{
			if(Cost[i][assignment[i]]>dist_thres)
			{
				assignment[i]=-1;
				not_assigned_tracks.push_back(i);
			}
		}
		else
		{			
			tracks[i]->skipped_frames++;
		}

	}

	for(size_t i=0;i<tracks.size();i++)
	{
		if(tracks[i]->skipped_frames>(unsigned)maximum_allowed_skipped_frames)
		{
			delete tracks[i];
			tracks.erase(tracks.begin()+i);
			assignment.erase(assignment.begin()+i);
			i--;
		}
	}

	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for(size_t i=0;i<detectedPedestrians.size();i++)
	{
		it=find(assignment.begin(), assignment.end(), i);
		if(it==assignment.end())
		{
			not_assigned_detections.push_back(i);
		}
	}

	if(not_assigned_detections.size()!=0)
	{
		for(size_t i=0;i<not_assigned_detections.size();i++)
		{
			PTrack* tr=new PTrack(detectedPedestrians[not_assigned_detections[i]].center,dt,Accel_noise_mag);
			tracks.push_back(tr);
		}	
	}

	for(size_t i=0;i<assignment.size();i++)
	{
		
		tracks[i]->KF->GetPrediction();

		if(assignment[i]!=-1) 
		{
			tracks[i]->skipped_frames=0;
			tracks[i]->predictedPedestrian.center=tracks[i]->KF->Update(detectedPedestrians[assignment[i]].center,1);
		}
		else				  
		{
			tracks[i]->predictedPedestrian.center=tracks[i]->KF->Update(Point2f(0,0),0);	
		}
		
		if(tracks[i]->trace.size()>(unsigned)max_trace_length)
		{
			tracks[i]->trace.erase(tracks[i]->trace.begin(),tracks[i]->trace.end()-max_trace_length);
		}

		tracks[i]->trace.push_back(tracks[i]->predictedPedestrian.center);
		tracks[i]->KF->LastResult=tracks[i]->predictedPedestrian.center;
	}

}


PTracker::~PTracker(void)
{
	for(size_t i=0;i<tracks.size();i++)
	{
		delete tracks[i];
	}
	tracks.clear();
}
