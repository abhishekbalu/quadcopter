/*
 * OpticalFlow.cpp
 *
 *  Created on: May 6, 2013
 *      Author: ivan
 */

#include "OpticalFlow.h"

OpticalFlow::OpticalFlow()
{
	this->flag = 0;
}

OpticalFlow::OpticalFlow(SFParams sparams)
{
	this->sparams = sparams;
	this->flag = 1;
}

OpticalFlow::OpticalFlow(LKParams lparams)
{
	this->lparams = lparams;
	this->flag = 0;
}

OpticalFlow::~OpticalFlow()
{
	// default destructor
}

Point2f OpticalFlow::calcFlow(Mat& frame1, Mat& frame2,vector<Point2f> markers)
{
	Point2f flow_est;
	Mat flow; // output flow
	if(flag)
	{
		calcOpticalFlowSF(frame1,frame2,flow,sparams.layers,
				sparams.averaging_block_size, sparams.max_flow, sparams.sigma_dist, sparams.sigma_color,
				sparams.postprocess_window, sparams.sigma_dist_fix,sparams.sigma_color_fix, sparams.occ_thr,
				sparams.upscale_averaging_radius, sparams.upscale_sigma_dist, sparams.upscale_sigma_color,
				sparams.speed_up_thr);
	}
	else
	{
		Mat status, err; // status and error matrices
		// Use Lucas-Kanade iterative pyramid method
		calcOpticalFlowPyrLK(frame1, frame2, markers, markers, status, err);
	}
	return flow_est;
}
