//C++ Libraries
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
//ROS libraries
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <tf/transform_datatypes.h>
//user libraries
#include "cam/detections.h"
#include "cam/blob_detection.h"
#include "cam/marker_detection.h"
#include "cam/debug.h" //Comment or uncomment this for verbose
#include "yaml-cpp/yaml.h"

#include "cam/QuadPose.h"
#include "cam/QuadPoseList.h"

using namespace std;
using namespace YAML;
using namespace Eigen;

vector<marker>* detected_quads; //Remember, marker struct = one quad
MatrixXd* objects;
unsigned char* bufb,* buf;
blob* blp;
int nblobs,nobjects;

int vl; //min val 
int vh; //max val 

int hl; //min hue 
int hh; //max hue 

int sl; //min sat 
int sh; //max sat 
int width;
int height;
static ros::Publisher rgb_image_pub;
static ros::Publisher bin_image_pub;
static ros::Publisher detections_pub;
static ros::Publisher markers_pub;
std::string blob_settings = "blue_blob_settings.yaml";
std::string image_settings = "image_settings.yaml";
void image_reception_callback(const sensor_msgs::ImageConstPtr& msg){

	//get image
	const sensor_msgs::Image img = *msg;

	//Copy image to the buffer
	unsigned long int cnt=0;
    for(unsigned int l=0;l<img.height;l++){
        for(unsigned int k=0;k<img.width;k++){
            buf[cnt]=img.data[cnt];
            buf[cnt+1]=img.data[cnt+1];
            buf[cnt+2]=img.data[cnt+2];
            cnt+=3;
            
        }
    }
    int xi = 0;
    int xf = img.width;
    int yi = 0;
    int yf = img.height;

    //publishing usefull information
    sensor_msgs::Image rgb_img = *msg;
    fillImage(rgb_img,"rgb8",img.height,img.width,img.step,buf);
    rgb_image_pub.publish(rgb_img);

    for(unsigned int l=0;l<img.height;l++)
        for(unsigned int k=0;k<img.width;k++)
        	if(bufb[l*img.width + k] > 0) 
                bufb[l*img.width + k] = 255;
    sensor_msgs::Image bin_img = *msg;
    bufb = get_binary_image();
    detected_quads = get_markers();
    fillImage(bin_img,"mono8",img.height,img.width,img.step/3,bufb);
    bin_image_pub.publish(bin_img);

	//blob detection and publish binary image
    track_markers(buf,3, vl, vh, hl, hh, sl, sh, xi, xf, yi, yf, img.width, img.height);
	
	detected_quads = get_markers();
	cam::QuadPoseList quad_poses_msg;
	quad_poses_msg.header.stamp = ros::Time::now();
	Matrix3d rotation;
	for(vector<marker>::iterator it = detected_quads->begin(); it != detected_quads->end(); ++it){
		cam::QuadPose quad_pose;
		quad_pose.name = it->name;
		quad_pose.position.x = it->T.col(3)[0];
		quad_pose.position.y = it->T.col(3)[1];
		quad_pose.position.z = it->T.col(3)[2];
		rotation.col(0) = it->T.col(0);
		rotation.col(1) = it->T.col(1);
		rotation.col(2) = it->T.col(2);
		Vector3d ea = rotation.eulerAngles(2, 1, 0); //the order of the angles matter
		double roll = ea[2];
		double pitch = ea[1];
		double yaw = ea[0];
		quad_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
		if(it->failures == 0)
		    quad_pose.pose_updated = 1;
		else
		    quad_pose.pose_updated = 0;
		quad_poses_msg.poses.push_back(quad_pose);
	}
	markers_pub.publish(quad_poses_msg);

    return;
}

void readBlobParams(std::string file){
	YAML::Node root = YAML::LoadFile(file);
	try{
		vl = root["vl"].as<int>();
		vh = root["vh"].as<int>();
		hl = root["hl"].as<int>();
		hh = root["hh"].as<int>();
		sl = root["sl"].as<int>();
		sh = root["sh"].as<int>();
		#ifdef VERBOSE
		 	std::cout << root["vl"].as<int>() << "\n";
		 	std::cout << root["vh"].as<int>() << "\n";
		 	std::cout << root["hl"].as<int>() << "\n";
		 	std::cout << root["hh"].as<int>() << "\n";
		 	std::cout << root["sl"].as<int>() << "\n";
		 	std::cout << root["sh"].as<int>() << "\n";
	 	#endif
	}catch(const BadConversion& e){}
}

void readImageParams(std::string file){
	YAML::Node root = YAML::LoadFile(file);
	try{
		width = root["width"].as<int>();
		height = root["height"].as<int>();
		#ifdef VERBOSE
			std::cout << root["width"].as<int>() << "\n";
	 		std::cout << root["height"].as<int>() << "\n";
	 	#endif
	}catch(const BadConversion& e){}
}

int main(int argc, char** argv){
	if(argc==6){
		vl = atoi(argv[1]);
		vh = atoi(argv[2]);
		hl = atoi(argv[3]);
		hh = atoi(argv[3]);
		sl = atoi(argv[4]);
		sh = atoi(argv[5]);
	}
	readImageParams(image_settings);
	readBlobParams(blob_settings);

	//Initialize the sensor
	init_binary_img(width, height);
	buf = (unsigned char*)malloc(width*height*3*sizeof(unsigned char));
    bufb = get_binary_image();
    blp = get_blobs();
    initialize_markers();

    //intialize ROS
    ros::init(argc, argv,"node");
    ros::NodeHandle nh;

    //subscribers
    ros::Subscriber image_sub=nh.subscribe("usb_cam/image_raw", 1, image_reception_callback); // only 1 in buffer size to drop other images if processing is not finished

    //publishers
    rgb_image_pub=nh.advertise<sensor_msgs::Image>("node/rgb_image",1);
    bin_image_pub=nh.advertise<sensor_msgs::Image>("node/binary_image",1);
    detections_pub=nh.advertise<cam::detections>("node/detections",1);
    markers_pub = nh.advertise<cam::QuadPoseList>("node/markers",1);





    ros::Rate loop_rate(30);
   
    //main loop
    while(ros::ok()){
        loop_rate.sleep();
		ros::spinOnce();
    }
    return EXIT_SUCCESS;
}