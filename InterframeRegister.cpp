#include "stdafx.h"
#include "InterframeRegister.h"

/* set constants */
const CvTermCriteria InterframeRegister::m_critTerm = cvTermCriteria(CV_TERMCRIT_ITER, 10, 1e-16);
int InterframeRegister::MAX_KLT_POINTS = 500;
int InterframeRegister::INLIER_THRESH = 30;
int InterframeRegister::RANSAC_ITER = 50;
int InterframeRegister::RANSAC_SAMPLE_SIZE = 10;
double InterframeRegister::FEATURE_QUALITY = 0.01;
int InterframeRegister::MIN_FEATURE_DIST = 5;
double InterframeRegister::MATCH_ERR_THRESH = 4.0;
CvSize InterframeRegister::KLT_SEARCH_WIN = cvSize(7, 7);
InterframeRegister::MODEL_TYPE InterframeRegister::MODEL = InterframeRegister::AFFINE;

InterframeRegister::InterframeRegister(void)
{
}

/**
Method to register two images using the simple pairwise algorithm without allocating memory
@param srcIm: Source Image 
@param dstIm: Destination image 
@param Covar: CvMat Not used 
@param pInterframePoorFlag: Detection Flag 
@param pA: Not Used 
@param pB: Not Used
@return void
*/
void InterframeRegister::Register( Mat srcIm, Mat dstIm, Mat &pTransform, Mat pCovar, bool pInterframePoorFlag)
{

	/* init transform to identity */
	setIdentity(pTransform);
	
	/* image params */
	const Size szCurr = srcIm.size();
	
	/* registration parameters */
	double bound[2][2] = {0.0};
	double transform[3][3] = {0.0};

	vector <Point2f> ptSrc (InterframeRegister::MAX_KLT_POINTS);
	
	/* Contrast stretch */
	Mat srcStretch = srcIm.clone();
	Mat dstStretch = dstIm.clone();
	cvutStdDevStretch( srcStretch , srcStretch );
	cvutStdDevStretch( dstStretch , dstStretch );

	////////////////////////////////////////////////////////
	/* KLT ONLY. NO SWITCHING TO INTENSITY-BASED */
	int cornerCount = InterframeRegister::InitFeatures(srcStretch, ptSrc);

	vector <Point2f> ptDst (cornerCount);

	/* sanity check for sufficiency */
	if(cornerCount < InterframeRegister::INLIER_THRESH)
	{
		if(pInterframePoorFlag != NULL) pInterframePoorFlag = true;
		return;
	}
	
	/* find matching points */
	cornerCount = TrackFeatures(srcStretch, dstStretch, ptSrc, ptDst, cornerCount);

	/* Robust estimation of motion parameters */
	vector <Point2d> optInliersL (cornerCount); /* dst */
	vector <Point2d> optInliersR (cornerCount); /* src */

	pTransform = RunRansac(ptDst, ptSrc, cornerCount, optInliersL, optInliersR, srcIm, dstIm);
}

/**
Method to register two images using the simple pairwise algorithm
@param srcIm: Source Image 
@param dstIm: Destination image 
@param Covar: CvMat Not used 
@param pInterframePoorFlag: Detection Flag 
@param pA: Not Used 
@param pB: Not Used
@return Newly allocated memory for interframe matrix
*/
Mat InterframeRegister::Register( Mat srcIm, Mat dstIm, Mat pCovar,bool pInterframePoorFlag)
{
	Mat HCurr(2,3,CV_64FC1);
	setIdentity(HCurr);
	Register(srcIm, dstIm, HCurr, pCovar, pInterframePoorFlag);
	return HCurr;
}


int InterframeRegister::InitFeatures(Mat fc, vector<Point2f> &arrFeaturePts)
{
	Mat eigImage,tempImage;
	int cornerCount = InterframeRegister::MAX_KLT_POINTS;
	goodFeaturesToTrack(fc, arrFeaturePts, cornerCount, FEATURE_QUALITY, MIN_FEATURE_DIST);
	return arrFeaturePts.size();
}

/**
Tracks features using KLT optical flow
*/
int InterframeRegister::TrackFeatures(Mat fp, Mat fc,vector<Point2f> &pPrev, vector<Point2f> &pCurr, int cornerCount)
{
	// allocate memory
	vector <uchar> status;
	status.reserve(cornerCount);
	Mat err;
	err.reserve(cornerCount);
	vector<Point2f> tempPrev = pPrev;
	vector<Point2f> tempCurr = pCurr;

	// calculate optical flow
	calcOpticalFlowPyrLK(fp, fc,tempPrev, tempCurr, status,err, KLT_SEARCH_WIN,4, m_critTerm, OPTFLOW_LK_GET_MIN_EIGENVALS);

	// copy matched points to array
	int nMatchedPts = 0;
	for (int ind = 0; ind < cornerCount; ind++)
	{
		if (status[ind] != 0x00)
		{
			pPrev[nMatchedPts] = tempPrev[ind];
			pCurr[nMatchedPts] = tempCurr[ind];
			++nMatchedPts;
		}
	}
	return nMatchedPts;
}

/**
calculates Affine transformation between two point sets
*/
void InterframeRegister::getAffine(vector <Point2d> ptL, vector <Point2d> ptR, Mat &H, int num)
{
	// Make matrices
	Mat A (num, 3, CV_64F); // transposed M
	Mat B (num, 2, CV_64F); // transposed U
	Mat hVec (3, 2, CV_64F); // transposed A
	for(int i = 0; i < num; i++)
	{
		A.at<double>(i,0) = ptL[i].x;
		A.at<double>(i,1) = ptL[i].y;
		A.at<double>(i,2) = 1.0;
	
		B.at<double>(i,0) = ptR[i].x;
		B.at<double>(i,1) = ptR[i].y;		
	}	
	int result = solve( A, B, hVec, DECOMP_SVD );

	// due to transposition, have to change stuff

	H.at<double>(0,0) = hVec.at<double>(0,0);
	H.at<double>(0,1) = hVec.at<double>(1,0);

	H.at<double>(0,2) = hVec.at<double>(2,0);

	H.at<double>(1,0) = hVec.at<double>(0,1);
	H.at<double>(1,1) = hVec.at<double>(1,1);
	
	H.at<double>(1,2) = hVec.at<double>(2,1);
	
}

/**
Transform determination. Usage of RANSAC to robustly estimate transformation between two point set ys
@param initMatchL: Input CvPoints for Destination Image 
@param initMatchR: Input CvPoints for Source Image 
@param noMatch: Number of Points
@param inliersL: Robust estimation of motion parameters for Destination Image
@param inliersR: Robust estimation of motion parameters for Source Image
@param srcIm: Source Image
@param dstIm: Destination Image
@return noInliers values
*/
Mat InterframeRegister::RunRansac(vector <Point2f> initMatchL, vector <Point2f> initMatchR, int noMatch,vector <Point2d> inliersL, vector <Point2d> inliersR, Mat srcIm, Mat dstIm)
{
	// number of test samples
	if(noMatch <= InterframeRegister::RANSAC_SAMPLE_SIZE) return Mat(); // sanity check

	// array of random indices {0,1,2 ... noMatch}
	static vector<int> randNumbs(RANSAC_SAMPLE_SIZE);
    vector<int> blacklist(noMatch);
	Mat thisH (2,3,CV_64FC1);
	
	double minErr = noMatch*MATCH_ERR_THRESH;
    double maxdist = 0;
	int noInliers = 0; // number of inliers
	vector<int> matchInd (noMatch);
    int blcnt = 0;

	// randomizer
	int stime = (unsigned) time(NULL)/2;
	srand(stime);

    for (int j = 0; j < noMatch; j++) 
	{
        blacklist[j] = -1;
    }

	// iterate over the number of samples
	vector <Point2d> selMatchL(RANSAC_SAMPLE_SIZE);
	vector <Point2d> selMatchR(RANSAC_SAMPLE_SIZE);
	
	for (int numSamples = 0; numSamples < RANSAC_ITER; numSamples++)
	{
		// make a random subset of entries from 0 to noMatch-1
		randNumbs = GetRandomSubset(noMatch, RANSAC_SAMPLE_SIZE, blacklist, blcnt);
		
		double sumErr = 0.0;
		for(int i = 0; i < RANSAC_SAMPLE_SIZE; i++)
		{
			selMatchL[i] = initMatchL[randNumbs[i]];
			selMatchR[i] = initMatchR[randNumbs[i]];
        }

		// get the affine matrix that best matches this
		switch(MODEL)
		{
		case AFFINE: getAffine(selMatchL,selMatchR,thisH,RANSAC_SAMPLE_SIZE);
					 break;

		default: throw; 
				 break;					
		}	
	}
	return thisH;
}

/**
Fills the array with random subset from 0 (inclusive) to iMaxNum (exclusive)
*/
vector<int> InterframeRegister::GetRandomSubset(int iMaxNum,int nArrSize, vector <int> blacklist, int blcnt)
{
	vector<int> iRandomSubsetArr(nArrSize);
	// need more array elements than number of indices
	if(nArrSize > iMaxNum) throw; 

	// iterate across array
	for(int i = 0; i < nArrSize; i++)
	{
		int newnum = (rand()%iMaxNum); // potential random number
		bool bProceedFlag = true;	   // flag to check whether to proceed
		
		// check the number is already present in the array
		for(int j = 0; j < i; j++)
		{
			if(iRandomSubsetArr[j] == newnum)
			{
				bProceedFlag = false;
				break;
			}
		}

        for (int j = 0; j < blcnt; j++) 
        {
            if (blacklist[j] == newnum)
            {
                bProceedFlag = false;
                break;
            }
        }

		// final check
		if(bProceedFlag) iRandomSubsetArr[i] = newnum; // all good, proceed
		else i--; // not good, try again
	}
	return iRandomSubsetArr;
}

/**
Calculates backprojection error and returns number of points within error tolerance
@param pL: Number of indices 
@param pR: Storage of the indices
@param numMatch: Number of Matches
@param H: Storage matrix used by RunRansac function
@param ind: array to store good matches
@param totErr: array to store error matches 
@param srcIm: Source Image 
@param dstIm: Destination Image
@return void
*/
int InterframeRegister::CalcErr(vector<Point2f> pL, vector<Point2f> pR, int numMatch, Mat H, vector<int>& ind, double *totErr) 
{
	//float err;
	Mat HInv(3,3,CV_64FC1);

	// check for singularity
	double matDet = invert(H,HInv,DECOMP_LU);
	if (abs(matDet)<0.00001)
	{
		*totErr = numMatch*MATCH_ERR_THRESH;
		return 0;
	}
	
	// compute error and number of matches
	int nGoodMatches = 0;
	Mat matLeft(3, 1, CV_64FC1);
	Mat matRight (3, 1, CV_64FC1);
	Mat matLeftEst (3, 1, CV_64FC1);
	Mat matRightEst (3, 1, CV_64FC1);
	Mat matLeftDiff (3, 1, CV_64FC1);
	Mat matRightDiff (3, 1, CV_64FC1);
	
	for (int cnt = 0; cnt < numMatch; cnt++)
	{
		matLeft = (Mat_<double>(3,1) << pL[cnt].x,pL[cnt].y,1.0);
		matRightEst = H * matLeft;
		
		matRight = (Mat_<double>(3,1) << pR[cnt].x,pR[cnt].y,1.0);

		matLeftEst = HInv * matRight;
		
		for(int i = 0; i < 3; i++)
		{
			matLeftEst.data[i] /= matLeftEst.data[2];
			matRightEst.data[i] /= matRightEst.data[2];
		}
		
		matLeftDiff = matLeftEst-matLeft;
		matRightDiff = matRightEst -matRight;

		double dLeftErr = norm(matLeftDiff);
		double dRightErr = norm(matRightDiff);
		double dErr = (dLeftErr * dLeftErr) + (dRightErr * dRightErr);
		

		if (dErr<MATCH_ERR_THRESH)
		{
			*totErr = *totErr+dErr;
			ind[nGoodMatches++] = cnt;			
		}
		else *totErr = *totErr+MATCH_ERR_THRESH;
	}
	
	return nGoodMatches;
}

int InterframeRegister::cvutStdDevStretch( Mat src, Mat dst)
{
	// Contrast Stretching:
	// Stretching by Standard deviation
	// A two standard deviation linear contrast stretch is applied

	Scalar mean;
	Scalar std_deviation;
	double min, max, scale, shift;

	// Computes mean and std dev of image src

	meanStdDev( src, mean, std_deviation);

	// Computes the stretch min and max
	min = mean.val[0] - 2.0 * std_deviation.val[0];
	max = mean.val[0] + 2.0 * std_deviation.val[0];

	if ( min != max )
	{
		scale = 255.0/(max - min);
		shift = -min * scale;
	}
	else
	{
		scale = 1.0;
		shift = 0.0;
	}

	// Computes:
	// image = ((image-mind)/(maxd-mind))*255;
	convertScaleAbs( src, dst, scale, shift );
	return true;
}


InterframeRegister::~InterframeRegister(void)
{
}
