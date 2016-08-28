/*
 * HB_Vision.h
 *
 * Created on: May 6, 2013
 *      Author: ivan
 */

#ifndef HB_VISION_H_
#define HB_VISION_H_

/* ==== System includes ==== */

// ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

// OpenCV includes

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

// Other includes

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>


const int log_on = 1;

/* YELLOW HSV FILTER */

//int hmin = 13;
//int smin = 70;
//int vmin = 160;
//int hmax = 33;
//int smax = 240;
//int vmax = 255;
int dist = 50;

/* BLUE HSV FILTER */

int hmin = 67;
int smin = 10;
int vmin = 20;
int hmax = 124;
int smax = 236;
int vmax = 255;

/* Aspect Ratio constants */

float bhigh = 1.7;
float blow = 0.3;

int minarea = 50;

/* Calibration parameters of the Caspa camera*/

Mat camMatrix = (Mat_<double>(3,3) << 2.3217604383275972e+02, 0., 1.7494982178310201e+02, 0.,
       2.7188629999974950e+02, 1.5031655590051398e+02, 0., 0., 1.);

Mat distCoeffs = (Mat_<double>(1,5) << -3.2552846361509252e-01, 1.3306214736208402e-01, 1.9789073961702522e-03, -1.1763870567679031e-02, 1.1287075868541123e-01); 

// old camera parameters -> laptop webcam

/*Mat camMatrix = (Mat_<double>(3,3) << 2.7701078684563563e+03, 0., 3.4861602280341123e+02, 
		  0.,2.9110423363124887e+03, 2.4738217602358861e+02, 0., 0., 1.);

Mat distCoeffs = (Mat_<double>(1,5) << -4.3406888327405815e+00, 1.4922729753038408e+02,
       7.3887417806545688e-02, -2.5244159653189735e-03, 3.1293043023906129e+00); 
*/

#endif /* HB_VISION_H_ */
