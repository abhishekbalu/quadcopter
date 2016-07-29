/*
 * OpticalFlow.h
 *
 *  Created on: May 6, 2013
 *      Author: ivan
 */

#ifndef OPTICALFLOW_H_
#define OPTICALFLOW_H_

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

struct SFParams
{
	 int layers;
	 int averaging_block_size;
	 int max_flow;
	 double sigma_dist;
	 double sigma_color;
	 int postprocess_window;
	 double sigma_dist_fix;
	 double sigma_color_fix;
	 double occ_thr;
	 int upscale_averaging_radius;
	 double upscale_sigma_dist;
	 double upscale_sigma_color;
	 double speed_up_thr;
};

// Lucas-Kanade method for optical flow
struct LKParams
{
	Size winsize;
	int maxlevel;
	TermCriteria crit;
	int flags;
	double minEigThreshold;
};

class OpticalFlow {
public:
	OpticalFlow();
	OpticalFlow(LKParams lparams);
	OpticalFlow(SFParams sparams);
	Point2f calcFlow(Mat& frame1, Mat& frame2,vector<Point2f> markers);
	virtual ~OpticalFlow();

private:
	SFParams sparams;
	LKParams lparams;
	int flag; // LK or SF 
};



#endif /* OPTICALFLOW_H_ */
