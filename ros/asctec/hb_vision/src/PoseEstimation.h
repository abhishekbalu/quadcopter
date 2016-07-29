/*
 * PoseEstimation.h
 *
 *  Created on: May 6, 2013
 *      Author: ivan
 */

#ifndef POSEESTIMATION_H_
#define POSEESTIMATION_H_

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

class PoseEstimation {
public:
	PoseEstimation(vector<Point3f> objectPoints, Mat distCoeffs, Mat cameraMatrix);
	void setCameraMatrix();
	void setObjectPoints();
	virtual ~PoseEstimation();
	Mat estimate(vector<Point2f> imagePoints, Mat& rvec, Mat& tvec, int flags, int log);
private:
	vector<Point3f> objectPoints;
	Mat distCoeffs;
	Mat cameraMatrix;
};

#endif /* POSEESTIMATION_H_ */
