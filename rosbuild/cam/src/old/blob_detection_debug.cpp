//C++ Libraries
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
//ROS libraries
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
//user libraries
#include "cam/detections.h"
#include "cam/blob_detection.h"
#include "cam/debug.h" //Comment or uncomment this for verbose
#include "yaml-cpp/yaml.h"
using namespace std;
using namespace YAML;
unsigned char* bufb,* buf;
blob* blp;
int nblobs,nobjects;


//The following are for a red marker
int vl; //min val 50 (bright daylight)
int vh; //max val 100 (bright daylight)

int hl; //min hue 340 (bright daylight)
int hh; //max hue 370 (bright daylight)

int sl; //min sat 50 (bright daylight)
int sh; //max sat 100 (bright daylight)
int width;
int height;
static ros::Publisher rgb_image_pub;
static ros::Publisher bin_image_pub;
static ros::Publisher detections_pub;
std::string blob_settings = "blob_settings.yaml";
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

	//blob detection and publish binary image
    nblobs=detect_blobs(buf,3, vl, vh, hl, hh, sl, sh, 
    	xi, xf, yi, yf, img.width, img.height);

    for(unsigned int l=0;l<img.height;l++)
        for(unsigned int k=0;k<img.width;k++)
        	if(bufb[l*img.width + k] > 0) 
                bufb[l*img.width + k] = 255;


    //publishing usefull information
    sensor_msgs::Image rgb_img = *msg;
    fillImage(rgb_img,"rgb8",img.height,img.width,img.step,buf);
    rgb_image_pub.publish(rgb_img);

    sensor_msgs::Image bin_img = *msg;
    fillImage(bin_img,"mono8",img.height,img.width,img.step/3,bufb);
    bin_image_pub.publish(bin_img);

    cam::detections det_msg;
    det_msg.header = img.header;
    for(int k=0;k<nblobs;k++){
	    if(blp[k].valid){
	        det_msg.pos_x.push_back(blp[k].x);
	        det_msg.pos_y.push_back(blp[k].y);
	        det_msg.size.push_back(blp[k].size);
	    }
    }
    det_msg.blob_count = nblobs;
    detections_pub.publish(det_msg);
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

    //intialize ROS
    ros::init(argc, argv,"blob_detection_debug");
    ros::NodeHandle nh;

    //subscribers
    ros::Subscriber image_sub=nh.subscribe("usb_cam/image_raw", 1, image_reception_callback); // only 1 in buffer size to drop other images if processing is not finished

    //publishers
    rgb_image_pub=nh.advertise<sensor_msgs::Image>("cam/rgb_image",1);
    bin_image_pub=nh.advertise<sensor_msgs::Image>("cam/binary_image",1);
    detections_pub=nh.advertise<cam::detections>("cam/detections",1);

    ros::Rate loop_rate(1);
   

    //main loop
    while(ros::ok()){
        loop_rate.sleep();
		ros::spinOnce();
    }
    return EXIT_SUCCESS;
}