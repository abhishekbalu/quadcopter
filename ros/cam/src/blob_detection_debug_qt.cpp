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
#include "cam/debug.h" //Comment or uncomment this for verbose
#include "yaml-cpp/yaml.h"

#include "qtfile.h"

using namespace std;
using namespace YAML;

#define GUI 1
#define RATE 50
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

int width;
int height;
static ros::Publisher rgb_image_pub;
static ros::Publisher bin_image_pub;
static ros::Publisher detections_pub;
static ros::Publisher valid_blobs_pub;
std::string blob_settings = "yamls/blue_blob_settings.yaml";
std::string image_settings = "yamls/image_settings.yaml";
std_msgs::Int8 valid_msg;


const int uchar_clipping_table[] = {
  0,0,0,0,0,0,0,0, // -128 - -121
  0,0,0,0,0,0,0,0, // -120 - -113
  0,0,0,0,0,0,0,0, // -112 - -105
  0,0,0,0,0,0,0,0, // -104 -  -97
  0,0,0,0,0,0,0,0, //  -96 -  -89
  0,0,0,0,0,0,0,0, //  -88 -  -81
  0,0,0,0,0,0,0,0, //  -80 -  -73
  0,0,0,0,0,0,0,0, //  -72 -  -65
  0,0,0,0,0,0,0,0, //  -64 -  -57
  0,0,0,0,0,0,0,0, //  -56 -  -49
  0,0,0,0,0,0,0,0, //  -48 -  -41
  0,0,0,0,0,0,0,0, //  -40 -  -33
  0,0,0,0,0,0,0,0, //  -32 -  -25
  0,0,0,0,0,0,0,0, //  -24 -  -17
  0,0,0,0,0,0,0,0, //  -16 -   -9
  0,0,0,0,0,0,0,0, //   -8 -   -1
  0,1,2,3,4,5,6,7,
  8,9,10,11,12,13,14,15,
  16,17,18,19,20,21,22,23,
  24,25,26,27,28,29,30,31,
  32,33,34,35,36,37,38,39,
  40,41,42,43,44,45,46,47,
  48,49,50,51,52,53,54,55,
  56,57,58,59,60,61,62,63,
  64,65,66,67,68,69,70,71,
  72,73,74,75,76,77,78,79,
  80,81,82,83,84,85,86,87,
  88,89,90,91,92,93,94,95,
  96,97,98,99,100,101,102,103,
  104,105,106,107,108,109,110,111,
  112,113,114,115,116,117,118,119,
  120,121,122,123,124,125,126,127,
  128,129,130,131,132,133,134,135,
  136,137,138,139,140,141,142,143,
  144,145,146,147,148,149,150,151,
  152,153,154,155,156,157,158,159,
  160,161,162,163,164,165,166,167,
  168,169,170,171,172,173,174,175,
  176,177,178,179,180,181,182,183,
  184,185,186,187,188,189,190,191,
  192,193,194,195,196,197,198,199,
  200,201,202,203,204,205,206,207,
  208,209,210,211,212,213,214,215,
  216,217,218,219,220,221,222,223,
  224,225,226,227,228,229,230,231,
  232,233,234,235,236,237,238,239,
  240,241,242,243,244,245,246,247,
  248,249,250,251,252,253,254,255,
  255,255,255,255,255,255,255,255, // 256-263
  255,255,255,255,255,255,255,255, // 264-271
  255,255,255,255,255,255,255,255, // 272-279
  255,255,255,255,255,255,255,255, // 280-287
  255,255,255,255,255,255,255,255, // 288-295
  255,255,255,255,255,255,255,255, // 296-303
  255,255,255,255,255,255,255,255, // 304-311
  255,255,255,255,255,255,255,255, // 312-319
  255,255,255,255,255,255,255,255, // 320-327
  255,255,255,255,255,255,255,255, // 328-335
  255,255,255,255,255,255,255,255, // 336-343
  255,255,255,255,255,255,255,255, // 344-351
  255,255,255,255,255,255,255,255, // 352-359
  255,255,255,255,255,255,255,255, // 360-367
  255,255,255,255,255,255,255,255, // 368-375
  255,255,255,255,255,255,255,255, // 376-383
};
const int clipping_table_offset = 128;

/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<val<383.
 */
static int CLIPVALUE(int val){
  return uchar_clipping_table[val+clipping_table_offset];
}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
static void YUV2RGB(const int y, const int u, const int v, int* r, int* g, int* b){
    const int y2=(int)y;
    const int u2=(int)u-128;
    const int v2=(int)v-128;
    //std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;


    // This is the normal YUV conversion, but
    // appears to be incorrect for the firewire cameras
    //   int r2 = y2 + ( (v2*91947) >> 16);
    //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
    //   int b2 = y2 + ( (u2*115999) >> 16);
    // This is an adjusted version (UV spread out a bit)
    int r2 = y2 + ( (v2*37221) >> 15);
    int g2 = y2 - ( ((u2*12975) + (v2*18949)) >> 15 );
    int b2 = y2 + ( (u2*66883) >> 15);
    //std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;


    // Cap the values.
    *r=CLIPVALUE(r2);
    *g=CLIPVALUE(g2);
    *b=CLIPVALUE(b2);
}

void image_reception_callback(const sensor_msgs::ImageConstPtr& msg){

	//get image
	const sensor_msgs::Image img = *msg;

	//Copy image to the buffer and convert
	unsigned long int cnt=0;
    int y0, u, y1, v, r, g, b;
    
    for(unsigned int l=0;l<img.height;l++){
        for(unsigned int k=0;k<img.width;k++){
          //CONVERT RGB TO YUYV -- https://en.wikipedia.org/wiki/YUV
          int red = img.data[cnt];
          int green = img.data[cnt +1];
          int blue = img.data[cnt + 2];
          //printf("RED: %d, GREEN: %d, BLUE: %d\n", red, green, blue);
          /*
          int y = (0.299 * red) + (0.587 * green) + (0.114 * blue);
          int u = (0.436 * blue) - (0.147 * red) - (0.289 * green);
          int v = (0.615 * red) - (0.515 * green)  - (0.100 * blue);

          int y = (0.257 * red) + (0.504 * green) + (0.098 * blue);
          int u = (0.439 * blue) - (0.148 * red) - (0.291 * green);
          int v = (0.439 * red) - (0.368 * green)  - (0.071 * blue);*/
          //printf("Y: %d, U: %d, V: %d\n", y, u, v);
          buf[cnt] = red;
          buf[cnt+1] = green;
          buf[cnt+2] = blue;
          cnt+=3;
        }
    }
    /*
    for (int i = 0, cnt = 0; i < ((int)img.width*img.height << 1); i += 4, cnt += 6){
        y0 = img.data[i + 0];
        u = img.data[i + 1];
        y1 = img.data[i + 2];
        v = img.data[i + 3];
        YUV2RGB (y0, u, v, &r, &g, &b);
        buf[cnt + 0] = r;
        buf[cnt + 1] = g;
        buf[cnt + 2] = b;
        YUV2RGB (y1, u, v, &r, &g, &b);
        buf[cnt + 3] = r;
        buf[cnt + 4] = g;
        buf[cnt + 5] = b;
    }*/
    //The entire image has been converted to yuv
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

std::string getLastLine(std::ifstream& in)
{
    std::string line;
    while (in >> std::ws && std::getline(in, line)) // skip empty lines
        ;

    return line;
}

void read_file(){
    string line;

    ifstream myfile ("pipe.txt");
    if (myfile.is_open()){
        line = getLastLine(myfile);
        sl = atoi((line.substr(0,2)).c_str());
        sh = atoi((line.substr(3,3)).c_str());
        vl = atoi((line.substr(7,2)).c_str());
        vh = atoi((line.substr(10,3)).c_str());
        hl = atoi((line.substr(14,3)).c_str());
        hh = atoi((line.substr(18,3)).c_str());
        toggle = atoi((line.substr(22,1)).c_str());
        printf("sl:%d sh:%d vl:%d vh:%d hl:%d hh:%d toggle:%d \n", sl, sh, vl, vh, hl, hh, toggle);
        myfile.close();
    }
    printf("and here2\n");
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
    printf("New thread!\n");
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
  printf("here\n");
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
    ros::Time t;
    printf("and here\n");
    sleep(2);
    //main loop
    while(ros::ok()){
        t = ros::Time::now();
        cout << t << endl;
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
    return 0;
}