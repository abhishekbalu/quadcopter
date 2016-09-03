/*
 * BlobId.h
 *
 *  Created on: Apr 15, 2013
 *      Author: ivan
 */

#ifndef BLOB_H_
#define BLOB_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

class Blob {
public:
	Blob(std::vector<cv::Point> contour, int id, cv::Point2f c, int radius, cv::Rect bRect);
	virtual ~Blob();
	vector<Blob> distanceFilter(vector<Blob> &ids, int thresh);

	std::vector<cv::Point> contour;
	int id;
	cv::Point center;
	int radius;
	cv::Rect bRect;
};

/* helper function declarations */
Blob contourToBlob(vector<Point>& contour);
vector<Blob> distanceFilter(vector<Blob> &ids, int thresh);
float sqdistance(Point2f a, Point2f b);
vector<Point> getPath(Blob& blob);
vector<Point2f> getPointVector(vector<Blob> &ids);


#endif /* BLOB_H_ */
