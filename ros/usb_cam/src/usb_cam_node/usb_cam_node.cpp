/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

//ros libraries
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <usb_cam/usb_cam.h>
#include <self_test/self_test.h>
#include <image_transport/image_transport.h>
#include <usb_cam/detections.h>
#include <usb_cam/QuadPose.h>
#include <usb_cam/QuadPoseList.h>
#include <tf/transform_datatypes.h>

//std libraries
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;

//local libraries
#include "sensor.hpp"

MatrixXd* objects;
blob* blp;
vector<marker>* detected_quads;
static ros::Publisher detections_pub;
static ros::Publisher markers_pub;
int nblobs,nobjects;
int colour_img;

class UsbCamNode
{
public:
  ros::NodeHandle node_;
  sensor_msgs::Image img_;

  std::string video_device_name_;
  std::string io_method_name_;
  int image_width_,image_height_;
  std::string pixel_format_name_;
  bool autofocus_;

  sensor_msgs::CameraInfo info_;

  ros::Time next_time_;
  int count_;

  usb_cam_camera_image_t* camera_image_;
  unsigned char* bin_image;

  image_transport::CameraPublisher image_pub_;

  UsbCamNode() :
      node_("~")
  {
   
    int lochm=180,lochM=250,locsm=50,locsM=110,locvm=80,locvM=110;
    int save_blobs=0,noise_active=0;
    std::string sensor_camera_topic;

    double fx=470,fy=490; //focal distance guess?
    double cx=320,cy=240; //center x, center y should be 320 and 240
    double kx1=0.0000055,kx2=2e-10,ky1=0.0000020,ky2=0.0;
    double theta = 0.00;
   
    int width = 640;
    int height = 480;
    double dt_min = 0.1;

    //get parameters
    std::string full_name;
    full_name = "/usb_cam_node/image_width";
    node_.getParam(full_name.c_str(),width);
    full_name = "/usb_cam_node/image_height";
    node_.getParam(full_name.c_str(),height);
    full_name = "/usb_cam_node/fx";
    node_.getParam(full_name.c_str(),fx);
    full_name = "/usb_cam_node/fy";
    node_.getParam(full_name.c_str(),fy);
    full_name = "/usb_cam_node/cx";
    node_.getParam(full_name.c_str(),cx);
    full_name = "/usb_cam_node/cy";
    node_.getParam(full_name.c_str(),cy);
    full_name = "/usb_cam_node/kx1";
    node_.getParam(full_name.c_str(),kx1);
    full_name = "/usb_cam_node/kx2";
    node_.getParam(full_name.c_str(),kx2);
    full_name = "/usb_cam_node/ky1";
    node_.getParam(full_name.c_str(),ky1);
    full_name = "/usb_cam_node/ky2";
    node_.getParam(full_name.c_str(),ky2);
    full_name = "/usb_cam_node/theta";
    node_.getParam(full_name.c_str(),theta);
    full_name = "/usb_cam_node/lochm";
    node_.getParam(full_name.c_str(),lochm);
    full_name = "/usb_cam_node/lochM";
    node_.getParam(full_name.c_str(),lochM);
    full_name = "/usb_cam_node/locsm";
    node_.getParam(full_name.c_str(),locsm);
    full_name = "/usb_cam_node/locsM";
    node_.getParam(full_name.c_str(),locsM);
    full_name = "/usb_cam_node/locvm";
    node_.getParam(full_name.c_str(),locvm);
    full_name = "/usb_cam_node/locvM";
    node_.getParam(full_name.c_str(),locvM);
    full_name = "/usb_cam_node/save_blobs";
    node_.getParam(full_name.c_str(),save_blobs);
    full_name = "/usb_cam_node/noise_active";
    node_.getParam(full_name.c_str(),noise_active);
    full_name = "/usb_cam_node/colour_img";
    node_.getParam(full_name.c_str(),colour_img);
    full_name = "/usb_cam_node/dt_min";
    node_.getParam(full_name.c_str(),dt_min);
    full_name = "/usb_cam_node/sensor_camera_topic";
    node_.getParam(full_name.c_str(),sensor_camera_topic);

    //publishers
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);
    detections_pub=node_.advertise<usb_cam::detections>("sensor/detections",1);
    markers_pub=node_.advertise<usb_cam::QuadPoseList>("sensor/markers",1);
    //markers_pub = node_.advertise<usb_cam::QuadPoseList>(sensor_camera_topic,1);

    //initialize sensor
    objects=sensor_init(width,height,fx,fy,cx,cy,kx1,kx2,ky1,ky2,theta,dt_min,lochm,lochM,locsm,locsM,locvm,locvM,save_blobs,noise_active);
    blp=get_blobs();
    cout<<width<<" "<<height<<endl;

    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("io_method", io_method_name_, std::string("mmap")); // possible values: mmap, read, userptr
    image_width_ = width;//node_.param("image_width", image_width_, 320);
    image_height_ = height; //node_.param("image_height", image_height_, 240);
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg")); // possible values: yuyv, uyvy, mjpeg
    node_.param("autofocus", autofocus_, false); // enable/disable autofocus



    
      XmlRpc::XmlRpcValue double_list;
      info_.height = height; //image_height_;
      info_.width = width; //image_width_;

      node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
      info_.header.frame_id = img_.header.frame_id;

      node_.getParam("K", double_list);
      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          info_.K[i] = double_list[i];
        }
      }

      node_.getParam("D", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray)) {
        info_.D.resize(double_list.size());
        for (int i=0; i<double_list.size(); i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          info_.D[i] = double_list[i];
        }
      }

      node_.getParam("R", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 9)) {
        for (int i=0; i<9; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          info_.R[i] = double_list[i];
        }
      }

      node_.getParam("P", double_list);

      if ((double_list.getType() == XmlRpc::XmlRpcValue::TypeArray) &&
          (double_list.size() == 12)) {
        for (int i=0; i<12; i++) {
          ROS_ASSERT(double_list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
          info_.P[i] = double_list[i];
        }
      }
    

    printf("usb_cam video_device set to [%s]\n", video_device_name_.c_str());
    printf("usb_cam io_method set to [%s]\n", io_method_name_.c_str());
    printf("usb_cam image_width set to [%d]\n", image_width_);
    printf("usb_cam image_height set to [%d]\n", image_height_);
    printf("usb_cam pixel_format set to [%s]\n", pixel_format_name_.c_str());
    printf("usb_cam auto_focus set to [%d]\n", autofocus_);

    usb_cam_io_method io_method;
    if(io_method_name_ == "mmap")
      io_method = IO_METHOD_MMAP;
    else if(io_method_name_ == "read")
      io_method = IO_METHOD_READ;
    else if(io_method_name_ == "userptr")
      io_method = IO_METHOD_USERPTR;
    else {
      ROS_FATAL("Unknown io method.");
      node_.shutdown();
      return;
    }
    usb_cam_pixel_format pixel_format;
    if(pixel_format_name_ == "yuyv")
      pixel_format = PIXEL_FORMAT_YUYV;
    else if(pixel_format_name_ == "uyvy")
      pixel_format = PIXEL_FORMAT_UYVY;
    else if(pixel_format_name_ == "mjpeg") {
      pixel_format = PIXEL_FORMAT_MJPEG;
    }
    else {
      ROS_FATAL("Unknown pixel format.");
      node_.shutdown();
      return;
    }
    camera_image_ = usb_cam_camera_start(video_device_name_.c_str(),
        io_method,
        pixel_format,
        image_width_,
        image_height_);

    if(autofocus_) {
      usb_cam_camera_set_auto_focus(1);
    }
    next_time_ = ros::Time::now();
    count_ = 0;
  }

  virtual ~UsbCamNode()
  {

    usb_cam_camera_shutdown();
  }

  bool take_and_send_image()
  {
    //count time
    //ros::Time t= ros::Time::now();

    //get the image
    usb_cam_camera_grab_image(camera_image_);

    if(colour_img>=0){ // only publish image if parameter non-negative
	if(colour_img==1) // publish colour image
	    fillImage(img_, "rgb8", image_height_, image_width_, 3 * image_width_, camera_image_->image);
	}else if(colour_img==0){ // publish binary image
	    bin_image = get_binary_image();
	    for(unsigned int l=0; l<image_height_; l++)
		for(unsigned int k=0; k<image_width_; k++)
		    if(bin_image[l*image_width_ + k]>0)
			bin_image[l*image_width_ + k]=255;
	   
	    //debug images (inclusion of the detected areas)
	    detected_quads = get_markers();	    
	    /*
	    for(vector<marker>::iterator it = detected_quads->begin(); it != detected_quads->end(); ++it){
		for(int k=it->imx0,l=it->imy0;k<it->imxf;k++) bin_image[l*image_width_  + k]=255;
		for(int k=it->imx0,l=it->imyf;k<it->imxf;k++) bin_image[l*image_width_ + k]=255;
		for(int k=it->imx0,l=it->imy0;l<it->imyf;l++) bin_image[l*image_width_  + k]=255;
		for(int k=it->imxf,l=it->imy0;l<it->imyf;l++) bin_image[l*image_width_ + k]=255;

		//if(strcmp(it->name,"unknown")){
		    for(int l=it->imy0_LED;l<it->imyf_LED;l++)
			for(int k=it->imx0_LED;k<it->imxf_LED;k++)
			    if(bin_image[l*image_width_+ k]!=0)
				bin_image[l*image_width_ + k]=255;

		    for(int k=it->imx0_LED,l=it->imy0_LED;k<it->imxf_LED;k++) bin_image[l*image_width_ + k]=255;
		    for(int k=it->imx0_LED,l=it->imyf_LED;k<it->imxf_LED;k++) bin_image[l*image_width_ + k]=255;
		    for(int k=it->imx0_LED,l=it->imy0_LED;l<it->imyf_LED;l++) bin_image[l*image_width_ + k]=255;
		    for(int k=it->imxf_LED,l=it->imy0_LED;l<it->imyf_LED;l++) bin_image[l*image_width_ + k]=255;
		    
		//}
	    }*/

	    fillImage(img_, "mono8", image_height_, image_width_, image_width_, bin_image);

	}

	img_.header.stamp = ros::Time::now();
	info_.header.stamp = img_.header.stamp;
	image_pub_.publish(img_, info_);


    
    
    /*
    //perform blob detection
    int vl, vh, hl, hh, sl, sh, xi, xf, yi,yf;
    vl = 40; vh = 100; // range of value to be detected
    hl = 170; hh = 230; // range of hue to be detected
    sl = 40; sh = 100; // range of saturation to be detected
    xi = 0; xf = 320; yi = 0; yf = 240;
    nblobs = detect_blobs((unsigned char*)camera_image_->image,3,vl,vh,hl,hh,sl,sh,xi,xf,yi,yf);
    cout << nblobs << " blobs, " << endl;
    */
    //detect markers
    //nobjects = detect_markers();
    //print time after the process is over
    //std::cout<<"finished in "<<ros::Time::now()-t<<std::endl;

    track_markers((unsigned char*)camera_image_->image,3);

    //publishing blob information
    /*usb_cam::detections det_msg;
    det_msg.header.stamp = ros::Time::now();
    for(int k=0;k<nblobs;k++)
        if(blp[k].valid){
            det_msg.pos_x.push_back(blp[k].x);
            det_msg.pos_y.push_back(blp[k].y);
            det_msg.size.push_back(blp[k].sz);
        }
    detections_pub.publish(det_msg);*/

    // publish information about detected quads on /sensor/poses
    detected_quads = get_markers();
    
    usb_cam::QuadPoseList quad_poses_msg;
    quad_poses_msg.header.stamp = ros::Time::now();
    Matrix3d rotation;
    for(vector<marker>::iterator it = detected_quads->begin(); it != detected_quads->end(); ++it){
    	usb_cam::QuadPose quad_pose;
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

    /*
    //publishing marker information
    usb_cam::QuadPoseList quad_poses_msg;
    quad_poses_msg.header.stamp = ros::Time::now();
    Matrix3d rotation;
    //mcs_pose_msg.header.frame_id - not important
    for(int k=0;k<nobjects;k++)
    {
      usb_cam::QuadPose quad_pose;
      quad_pose.name="quad98";
      quad_pose.position.x=objects[k].col(3)[0];
      quad_pose.position.y=objects[k].col(3)[1];
      quad_pose.position.z=objects[k].col(3)[2];
      rotation.col(0)=objects[k].col(0);
      rotation.col(1)=objects[k].col(1);
      rotation.col(2)=objects[k].col(2);
      Vector3d ea = rotation.eulerAngles(2, 1, 0); //the order of the angles matter
      double roll = ea[2];
      double pitch = ea[1];
      double yaw = ea[0];
      quad_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
      quad_pose.pose_updated=1;
      quad_poses_msg.poses.push_back(quad_pose);
    }
    markers_pub.publish(quad_poses_msg);
    */
    return true;
  }


  bool spin()
  {
    ros::Rate loop_rate(30);
    while (node_.ok())
    {
      if (take_and_send_image())
      {
        count_++;
        ros::Time now_time = ros::Time::now();
        if (now_time > next_time_) {
	    std::cout << count_ << " frames/sec at " << now_time << std::endl;
	    count_ = 0;
	    next_time_ = next_time_ + ros::Duration(1.0);
	}
      } else {
        ROS_ERROR("couldn't take image.");
        usleep(1000000);
      }
      loop_rate.sleep();
    }
    
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  UsbCamNode a;
  cout << "delay flag" << endl;
  a.spin();
  return 0;
}
