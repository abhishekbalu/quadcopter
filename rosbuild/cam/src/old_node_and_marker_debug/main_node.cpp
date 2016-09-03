//C++ Libraries
#include <stdio.h>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
//ROS Libraries
#include "ros/ros.h"
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <tf/transform_datatypes.h>

//User Libraries
#include "cam/blob_detection.h"
#include "cam/marker_detection.h"
#include "cam/usb_cam.h"
#include "cam/detections.h"

#include "cam/QuadPose.h"
#include "cam/QuadPoseList.h"

using namespace Eigen;
using namespace std;
MatrixXd* objects;
blob* blp;
vector<marker>* detected_quads; //Remember, marker struct = one quad
static ros::Publisher detections_pub;
static ros::Publisher markers_pub;
int nblobs, nobjects;
int colour_img;

class USBCamNode{
	public:
		//Variables
		ros::NodeHandle n;
		sensor_msgs::Image img_;
		std::string video_device_name_;
		std::string io_method_name_;
		std::string pixel_format_name_;
		int image_width_, image_height_;
		bool autofocus_;

		sensor_msgs::CameraInfo info_;
		ros::Time now_time;
		ros::Time next_time_;
		int count_;
		usb_cam_camera_image_t* camera_image_;
		unsigned char* bin_image;
		image_transport::CameraPublisher image_pub_;
		//Parameters
		double fx,fy,cx,cy,kx1,kx2,ky1,ky2,theta,dt_min;
		int lochm,lochM,locsm,locsM,locvm,locvM;
		int save_blobs,noise_active;
		int width,height;
		std::string sensor_camera_topic;
		//Public Functions
		//Constructor
		USBCamNode() : n("~"){
		    //get parameters
		    std::string node_name ("/cam/");
		    n.getParam((node_name + "image_width").c_str(),width);
		    n.getParam((node_name + "image_height").c_str(),height);
		    n.getParam((node_name + "fx").c_str(),fx);
		    n.getParam((node_name + "fy").c_str(),fy);
		    n.getParam((node_name + "cx").c_str(),cx);
		    n.getParam((node_name + "cy").c_str(),cy);
		    n.getParam((node_name + "kx1").c_str(),kx1);
		    n.getParam((node_name + "kx2").c_str(),kx2);
		    n.getParam((node_name + "ky1").c_str(),ky1);
		    n.getParam((node_name + "ky2").c_str(),ky2);
		    n.getParam((node_name + "theta").c_str(),theta);
		    n.getParam((node_name + "lochm").c_str(),lochm);
		    n.getParam((node_name + "lochM").c_str(),lochM);
		    n.getParam((node_name + "locsm").c_str(),locsm);
		    n.getParam((node_name + "locsM").c_str(),locsM);
		    n.getParam((node_name + "locvm").c_str(),locvm);
		    n.getParam((node_name + "locvM").c_str(),locvM);
		    n.getParam((node_name + "save_blobs").c_str(),save_blobs);
		    n.getParam((node_name + "noise_active").c_str(),noise_active);
		    n.getParam((node_name + "colour_img").c_str(),colour_img);
		    n.getParam((node_name + "dt_min").c_str(),dt_min);
		    n.getParam((node_name + "sensor_camera_topic").c_str(),sensor_camera_topic);
		    //publishers
		    image_transport::ImageTransport it(n);
		    image_pub_ = it.advertiseCamera("image_raw", 1);
		    detections_pub = n.advertise<cam::detections>("sensor/detections",1);
		    markers_pub = n.advertise<cam::QuadPoseList>(sensor_camera_topic,1);
		    //More parameters
		    blp=get_blobs();
		    n.param("video_device", video_device_name_, std::string("/dev/video0"));
		    n.param("io_method", io_method_name_, std::string("mmap")); // possible values: mmap, read, userptr
		    image_width_ = width;//n.param("image_width", image_width_, 320);
		    image_height_ = height; //n.param("image_height", image_height_, 240);
		    n.param("pixel_format", pixel_format_name_, std::string("yuyv")); // possible values: yuyv, uyvy, mjpeg
		    n.param("autofocus", autofocus_, false); // enable/disable autofocus
		    //Get camera parameters
		    XmlRpc::XmlRpcValue double_list;
      		info_.height = height; //image_height_;
      		info_.width = width; //image_width_;

      		n.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
	      	info_.header.frame_id = img_.header.frame_id;

	      	n.getParam("K", double_list);
			if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 9)) {
				for (int i=0; i<9; i++) {
				  ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				  info_.K[i] = double_list[i];
				}
			}

	      	n.getParam("D", double_list);
			if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray)) {
				info_.D.resize(double_list.size());
				for (int i=0; i<double_list.size(); i++) {
				  ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				  info_.D[i] = double_list[i];
				}
			}

			n.getParam("R", double_list);
			if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 9)) {
				for (int i=0; i<9; i++) {
				  ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				  info_.R[i] = double_list[i];
				}
			}

	        n.getParam("P", double_list);
			if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) && (double_list.size() == 12)) {
				for (int i=0; i<12; i++) {
				  ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				  info_.P[i] = double_list[i];
				}
			}
			usb_cam_io_method io_method;
		    if(io_method_name_ == "mmap")
		    	io_method = IO_METHOD_MMAP;
		    else if(io_method_name_ == "read")
		    	io_method = IO_METHOD_READ;
		    else if(io_method_name_ == "userptr")
		    	io_method = IO_METHOD_USERPTR;
		    else{
		    	ROS_FATAL("Unknown io method.");
		    	n.shutdown();
		    	return;
		    }
		    usb_cam_pixel_format pixel_format;
		    if(pixel_format_name_ == "yuyv")
		    	pixel_format = PIXEL_FORMAT_YUYV;
		    else if(pixel_format_name_ == "uyvy")
		    	pixel_format = PIXEL_FORMAT_UYVY;
		    else if(pixel_format_name_ == "mjpeg") {
		    	pixel_format = PIXEL_FORMAT_MJPEG;
		    }else{
		    	ROS_FATAL("Unknown pixel format.");
		    	n.shutdown();
		    	return;
		    }
		    camera_image_ = usb_cam_camera_start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		    image_height_);

		    if(autofocus_)
		      usb_cam_camera_set_auto_focus(1);
		    next_time_ = ros::Time::now();
		    count_ = 0;

		    printf("Loaded all parameters...\n");
		}

		virtual ~USBCamNode(){
			usb_cam_camera_shutdown();
		}
		bool take_and_send_image(){
			usb_cam_camera_grab_image(camera_image_);

		    if(colour_img>=0){ // only publish image if parameter non-negative
				if(colour_img==1) // publish colour image
				    fillImage(img_, "rgb8", image_height_, image_width_, 3 * image_width_, camera_image_->image);
				else if(colour_img==0){ // publish binary image
				    bin_image = get_binary_image();
				    for(unsigned int l=0; l<image_height_; l++)
						for(unsigned int k=0; k<image_width_; k++)
						    if(bin_image[l*image_width_ + k]>0)
								bin_image[l*image_width_ + k]=255;
					detected_quads = get_markers();
					fillImage(img_, "mono8", image_height_, image_width_, image_width_, bin_image);
				}
				img_.header.stamp = ros::Time::now();
				info_.header.stamp = img_.header.stamp;
				image_pub_.publish(img_, info_);
			}

			track_markers((unsigned char*)camera_image_->image, 3, locvm, locvM, lochm, lochM, locsm, locsM, 0, width, 0, height, width, height);
			
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
			return true;

		}
		bool spin(){
			ros::Rate loop_rate(30);
			while(n.ok()){
				if(take_and_send_image()){
					count_++;
					ros::Time now_time = ros::Time::now();
					if(now_time > next_time_){
						count_ = 0.0;
						next_time_ = next_time_ + ros::Duration(1.0);
					}
				}else{
					ROS_ERROR("couldn't take image.");
        			sleep(1);
				}
				loop_rate.sleep();
			}
    		return true;
		}

};

int main(int argc, char **argv){
	ros::init(argc, argv, "usb_cam");
	USBCamNode c;
	printf("-----------------------------------------\n");
	c.spin();
	return 0;
}