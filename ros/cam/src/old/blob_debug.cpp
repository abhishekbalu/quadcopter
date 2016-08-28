//C++ Libraries
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
//ROS Libraries
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <geometry_msgs/Point.h>
//Personal Libraries
#include "cam/blob_detection.h"
#include "cam/detections.h"
#include "cam/blob.h"
#include "cam/usb_cam.h"
#include "cam/debug.h" //Comment or uncomment this for verbose

ros::Publisher debug_image_pub;
ros::Publisher detections_pub;

static std::vector<Blob*> blob_vector; 
cam::detections blobs;

BlobParams blob_params;
bool publish_debug = true;

void getDetections(const sensor_msgs::ImageConstPtr& msg){
	//Detect blobs
	sensor_msgs::Image img = *msg;
	#ifdef VERBOSE
		ROS_INFO("Got an image with step &d...\n", img.step);
    #endif
    detect_blobs(img, blobs, blob_params, debug_image_pub, publish_debug);
	//Publish the blob detections
	detections_pub.publish(blobs);
}


int main(int argc, char** argv){
	ROS_INFO("Started blob debugger...\n");
	initialize_blob_params(blob_params, 
		60, //min_hue
		130, //max_hue
		50, //min_sat
		120, //max_sat
		90, //min_val
		180, //max_val
		3 //min_size
		);


	ros::init(argc, argv, "blob_detector");
	ros::NodeHandle nh;
	ros::Subscriber image_sub=nh.subscribe("usb_cam/image_raw", 1, getDetections); // only 1 in buffer size to drop other images if processing is not finished
	debug_image_pub=nh.advertise<sensor_msgs::Image>("blob/debug", 1);
	detections_pub=nh.advertise<cam::detections>("blob/detections_by_pc",1);
	

	ros::Rate loop_rate(10);

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
