//C++ Libraries
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
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
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;
using namespace YAML;
using namespace Eigen;

#define MAX_DEVIATION 0.2
const int RATE = 50;
std_msgs::Float32MultiArray val_arr;
vector<marker>* detected_quads; //Remember, marker struct = one quad
MatrixXd* objects;
unsigned char* bufb,* buf;
blob* blp;
int nblobs,nobjects;

int datapoints = 0;
ofstream outputfile;


double value = 0.0;
double last_value = 0.0;

ros::Time t;
int width;
int height;
static ros::Publisher rgb_image_pub;
static ros::Publisher bin_image_pub;
static ros::Publisher detections_pub;
static ros::Publisher markers_pub;
static ros::Publisher frame_pub;
std::string image_settings = "yamls/image_settings.yaml";
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
    track_markers(buf, 3, img.width, img.height);
	
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
		
		if(quad_pose.name == "frame0" || quad_pose.name == "unknown"){
			printf("ROLL: %f PITCH: %f  YAW: %f\n", roll, pitch, yaw);
			//Reject outliers and when the frame is lost
			value = quad_pose.position.x;
			if((std::abs(last_value-value) < MAX_DEVIATION || last_value == 0) && quad_pose.position.x !=NULL){
				std::stringstream ss;
				t = ros::Time::now();
				ss << t << "," << quad_pose.position.x << "," << quad_pose.position.y << "," << quad_pose.position.z << "," << roll << "," << pitch << "," << yaw  << "," << datapoints;
				std::string s = ss.str();
				outputfile << s << endl;
				cout << s << endl;
				datapoints++;
				last_value = value;
				if(value <= 17.5){
					value = 17.5;
				}
				val_arr.data.clear();
				val_arr.data.push_back(last_value);
			}
			
		}
		quad_poses_msg.poses.push_back(quad_pose);
	}
	markers_pub.publish(quad_poses_msg);
	frame_pub.publish(val_arr);
    return;
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
	readImageParams(image_settings);

	//Initialize the sensor
	init_binary_img(width, height);
	buf = (unsigned char*)malloc(width*height*3*sizeof(unsigned char));
    bufb = get_binary_image();
    blp = get_blobs();
    initialize_markers();
    outputfile.open("frame_data.csv");
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
    frame_pub = nh.advertise<std_msgs::Float32MultiArray>("node/frame", 1);

    ros::Rate loop_rate(RATE);
   
    //main loop
    while(ros::ok()){
        loop_rate.sleep();
		ros::spinOnce();
    }
    return EXIT_SUCCESS;
}