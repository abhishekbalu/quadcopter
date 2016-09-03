/*
 * PoseEstimation.cpp
 *
 *  Created on: May 6, 2013
 *      Author: ivan
 */

#include "PoseEstimation.h"

/*
 * Class constructor
 */
PoseEstimation::PoseEstimation(vector<Point3f> objectPoints, Mat distCoeffs, Mat cameraMatrix) {

	this->objectPoints = objectPoints;
	this->distCoeffs = distCoeffs;
	this->cameraMatrix = cameraMatrix;
}

PoseEstimation::~PoseEstimation() {
	// TODO Auto-generated destructor stub
}

void PoseEstimation::setObjectPoints()
{
	vector<Point3f> objectPoints;
	// TODO: static hardcoded points, change to dynamic / read from file
	objectPoints.push_back(Point3f(1,0,0));
	objectPoints.push_back(Point3f(0,1,0));
	objectPoints.push_back(Point3f(-1,0,0));
	objectPoints.push_back(Point3f(-1,0,0));
	this->objectPoints = objectPoints;
}

/*
 * Estimates the position of the camera by solving the
 * P4P problem using the Levenbergh-Marquandt algorithm
 */

Mat PoseEstimation::estimate(vector<Point2f> imagePoints, Mat& rvec, Mat& tvec, int flags, int log)
{
	Mat imgP(imagePoints);
	Mat objP(objectPoints);
	bool completed = solvePnP(objP, imgP, cameraMatrix,
				distCoeffs, rvec, tvec, false, flags); 

	//cout << "Translation vector " << tvec << endl;

	/* get rotation matrix from rotation vector */
	Mat R;
	Rodrigues(rvec,R);

	/* (u,v,1) point -> use floats if possible*/
	Mat uvpoint(3,1,DataType<double>::type);
	// get XYZ world coordinates of the camera -> these need to be advertised as a ROS topic
	uvpoint.at<double>(0,0) = imagePoints.at(0).x;
	uvpoint.at<double>(1,0) = imagePoints.at(0).y;
	uvpoint.at<double>(2,0) = 1;
	//Mat P = R.inv() * (cameraMatrix.inv() * uvpoint - tvec);
	Mat P = -R.t()*tvec;	

	if(log)
	{
		fstream log;
		log.open("posest_log", fstream::in | fstream::out | fstream::app);
		log << "P = " << R.inv() * (cameraMatrix.inv() * uvpoint - tvec) << endl;
		log.close();
	}
	return P;
}


