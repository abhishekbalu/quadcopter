//C++ Libraries
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
//ROS libraries
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/Int8.h>
//user libraries
#include "cam/detections.h"
#include "cam/blob_detection.h"
//#include "cam/debug.h" //Comment or uncomment this for verbose
#include "yaml-cpp/yaml.h"

#include "qtfile.h"

using namespace std;
using namespace YAML;

#define GUI 1
#define RATE 50
#define INIT_SLEEP_TIME 2
const int BLUE = 1;
int fd;

unsigned char* bufb,* buf;
blob* blp;
int nblobs,nobjects;

int vl; //min val 
int vh; //max val 

int hl; //min hue 
int hh; //max hue 

int sl; //min sat 
int sh; //max sat 

int toggle = 0;
int line_num = 0;
int width;
int height;
static ros::Publisher rgb_image_pub;
static ros::Publisher bin_image_pub;
static ros::Publisher detections_pub;
static ros::Publisher valid_blobs_pub;
std::string blob_settings = "params/blue_blob_settings.yaml";
std::string image_settings = "params/image_settings.yaml";
std_msgs::Int8 valid_msg;

void image_reception_callback(const sensor_msgs::ImageConstPtr& msg){

	//get image
	const sensor_msgs::Image img = *msg;

	//Copy image to the buffer and convert
  //If the format of the image is yuyv (it should be) the driver will convert to rgb
  unsigned int cnt = 0;
    for(unsigned int l=0;l<img.height;l++){
        for(unsigned int k=0;k<img.width;k++){
          //CONVERSION YUYV to RGB -- https://en.wikipedia.org/wiki/YUV
          buf[cnt] = img.data[cnt];
          buf[cnt+1] = img.data[cnt +1];
          buf[cnt+2] = img.data[cnt + 2];
          //printf("RED: %d, GREEN: %d, BLUE: %d\n", red, green, blue);
          cnt+=3;
        }
    }

    int xi = 0;
    int xf = img.width;
    int yi = 0;
    int yf = img.height;

	//blob detection and publish binary image
    nblobs = detect_blobs(buf,3, vl, vh, hl, hh, sl, sh, xi, xf, yi, yf, img.width, img.height, 1, toggle);
    #ifdef VERBOSE
        printf("Number of valid blobs: %d\n", get_valid());
        valid_msg.data = get_valid();
    #endif

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


void read_file(){
    string line;
    ifstream myfile ("pipe.txt");
    if(line_num == 7)
      line_num = 0;
    if (myfile.is_open()){
      while(myfile.good()){
        getline(myfile, line);
        switch(line_num){
          case 0:
            sl = atoi(line.c_str());
            break;
          case 1:
            sh = atoi(line.c_str());
            break;
          case 2:
            vl = atoi(line.c_str());
            break;
          case 3:
            vh = atoi(line.c_str());
            break;
          case 4:
            hl = atoi(line.c_str());
            break;
          case 5:
            hh = atoi(line.c_str());
            break;
          case 6:
            toggle = atoi(line.c_str());
            break;
          default:
            break;
        }
        line_num++;
      }
      #ifdef VERBOSE
        printf("sl:%d sh:%d vl:%d vh:%d hl:%d hh:%d toggle:%d \n", sl, sh, vl, vh, hl, hh, toggle);
      #endif
      myfile.close();  
    }
    
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

void qt(int argc, char** argv){ //Apparently only works in windows and linux
    #ifdef VERBOSE
      printf("New thread!\n");
    #endif
    QApplication app(argc, argv); //<---------------------------------------------------------
    QFile qss("stylesheet.qss");
    qss.open(QFile::ReadOnly);
    app.setStyleSheet(qss.readAll());
    qss.close();
    MainWindow mainWindow; //<---------------------------------------------------------
    mainWindow.show(); //<---------------------------------------------------------
    app.exec();
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
    
  std::thread thr(qt, argc, argv);
    
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
  valid_blobs_pub=nh.advertise<std_msgs::Int8>("/valid_blobs", 1);
  ros::Rate loop_rate(RATE);
   

  sleep(INIT_SLEEP_TIME);
  //main loop
  while(ros::ok()){
      
    if(toggle == 0)
        read_file();
	  valid_blobs_pub.publish(valid_msg);
    loop_rate.sleep();
    ros::spinOnce();
  }

  thr.join();
  close(fd);
  free(buf);
  free(bufb);
  free(blp);
  return EXIT_SUCCESS;
}