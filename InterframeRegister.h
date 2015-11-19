#pragma once

#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include <iostream>
#include <vector>
#include <math.h>

using namespace cv;
using namespace std;

class InterframeRegister
{

public:

	InterframeRegister(void);
	~InterframeRegister(void);

	/** Constants used in the interframe algorithm. Can be changed. Defaults in CPP file */
	static int MAX_KLT_POINTS;									/**< Maximum number of corner matches to use */
	static int INLIER_THRESH;									/**< Min number of inlier matches for good interframe*/
	static int RANSAC_ITER;										/**< Number of ransac iterations to perform */
	static int RANSAC_SAMPLE_SIZE;								/**< Number of samples used in each ransac iteration */
	static double FEATURE_QUALITY;								/**< Quality of features used for opencv function*/
	static int MIN_FEATURE_DIST;								/**< Minimum distance between two features */
	static double MATCH_ERR_THRESH;								/**< Error threshold for matching */
    static CvSize KLT_SEARCH_WIN;								/**< KLT Search size */
	static const CvTermCriteria m_critTerm;		/**< Termination criteria for KLT */
	
	typedef enum MODEL_TYPE {TRANSLATION, AFFINE, HOMOGRAPHY};  /**< Matching model */
	static MODEL_TYPE MODEL;									/**< Typecast Model */

	
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
	static void Register( Mat srcIm, Mat dstIm, Mat &pTransform, Mat pCovar, bool pInterframePoorFlag);
	
	/**
	Method to register two images using the simple pairwise algorithm
	@param srcIm: Source Image 
	@param dstIm: Destination image 
	@param Covar: CvMat Not u sed 
	@param pInterframePoorFlag: Detection Flag 
	@param pA: Not Used 
	@param pB: Not Used
	@return Newly allocated memory for interframe matrix
	*/
	Mat Register( Mat srcIm, Mat dstIm, Mat pCovar,bool pInterframePoorFlag);
	
	/**
	calculates Affine transformation between two point sets
	@param ptL: Input CvPoints for Destination Image 
	@param ptR: Input CvPoints for Source Image 
	@param H: CvMat storage for Affine Matrix
    @param num: Number of Points
	@param ppA: Not Used
	@param ppB: Not Used
    @param pCovarMat: Not Used
	@return noInliers values
	*/
	/// @brief calculates Affine transformation between two point sets. 
	static inline void getAffine(vector <Point2d> ptL, vector <Point2d> ptR, Mat &H, int num);

protected:
	/**
	Feature detection and tracking. Initializes features based on min eigenvalues ???
	@param fc: Source Image 
	@param pCurr: CvPoints for image 
	@return cornerCount values
	*/
	static int InitFeatures(Mat fc, vector<Point2f> &arrFeaturePts);

	/**
	Tracks features using KLT optical flow
	@param fp: Source Image 
	@param fc: Detsination Image
	@param pPrev: CvPoints for source image
    @param pCurr: CvPoints for destination image
	@return nMatchedPts values
	*/
	static int TrackFeatures(Mat fp, Mat fc,vector<Point2f> &pPrev, vector<Point2f> &pCurr, int cornerCount);
	
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
	static Mat RunRansac(vector <Point2f> initMatchL, vector <Point2f> initMatchR, int noMatch,vector <Point2d> inliersL, vector <Point2d> inliersR, Mat srcIm, Mat dstIm);

	/**
	Fills the array with random subset from 0 (inclusive) to iMaxNum (exclusive)
	@param iMaxNum: Number of indices 
	@param iRandomSubsetArr: Storage of the indices
	@param nArrsize: Array size
    @param blacklist: Storage of the ranadom indices
	@param blcnt: counter for backlist
	@return void
	*/
	static vector<int> GetRandomSubset(int iMaxNum, int nArrSize, vector <int> blacklist, int blcnt);
	
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
	static inline int CalcErr(vector<Point2f> pL, vector<Point2f> pR, int numMatch, Mat H, vector<int> &ind, double *totErr);

	/**
	Contrast Stretching. Stretching by Standard deviation. A two standard deviation linear contrast stretch is applied
	@param src: Source Image 
	@param dst: Destination image 
	@param mask: Masking method.
	@return true when finish executing
	*/
	static int cvutStdDevStretch( Mat src, Mat dst);

};

