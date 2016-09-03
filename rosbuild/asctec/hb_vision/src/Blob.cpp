/*
 * BlobId.cpp
 *
 *  Created on: Apr 15, 2013
 *      Author: ivan
 */

#include "Blob.h"

Blob::Blob(std::vector<cv::Point> contour, int id, cv::Point2f c, int radius, cv::Rect bRect)
{
	this->contour = contour;
	this->id = id;
	this->center = c;
	this->radius = radius;
	this->bRect = bRect;
}

Blob::~Blob()
{
	// TODO Auto-generated destructor stub
}

/* helper functions */

float sqdistance(Point2f a, Point2f b)
{
	float retval = (a.y-b.y)*(a.y-b.y) + (a.x-b.x)*(a.x-b.x);
	return retval;
}

vector<Blob> distanceFilter(vector<Blob> &ids, int thresh)
{
	vector<Blob> out;
	for(size_t i = 1; i<ids.size(); i++)
	{
		float dist = sqrt(sqdistance(ids[i].center,ids[i-1].center));
		if(dist > thresh)
		{
			out.push_back(ids[i]);
		}
	}
	return out;
}

Blob contourToBlob(vector<Point>& contour)
{
	Rect bRect = boundingRect( Mat(contour) );
	Point2f center = Point(bRect.tl().x + (bRect.width / 2), bRect.tl().y + (bRect.height/2));
	vector<float> dists;
	for (size_t pointIdx = 0; pointIdx < contour.size(); pointIdx++)
	{
		Point2f pt = contour[pointIdx];
		dists.push_back(norm(center - pt));
	}
	sort(dists.begin(), dists.end());
	int radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;

	// returns blob with default id = 0
	return Blob(contour,0,center,radius,bRect);
}

vector<Point> getPath(Blob& blob)
{
	vector<Point> path;
	path.push_back(blob.center);
	return path;
}

vector<Point2f> getPointVector(vector<Blob> &ids)
{
	vector<Point2f> pv;
	for(int i = 0; i<ids.size(); i++)
	{
		pv.push_back(ids.at(i).center);
	}
	return pv;
}



