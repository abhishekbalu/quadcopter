//ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <usb_cam/detections.h>
#include <usb_cam/QuadPose.h>
#include <usb_cam/QuadPoseList.h>
#include <tf/transform_datatypes.h>

//std libraries
#include <iostream>
using namespace std;

//local libraries
#include "sensor.hpp"

unsigned char* bufb,* buf;
blob* blp;
int nblobs,nobjects;
Eigen::MatrixXd* objects;

void image_reception_callback(const sensor_msgs::ImageConstPtr& msg);
void initialize_sensor(std::string param_name);

static ros::Publisher rgb_image_pub;
static ros::Publisher bin_image_pub;
static ros::Publisher detections_pub;
static ros::Publisher markers_pub;

int main(int argc, char** argv)
{
    //intialize ROS
    ros::init(argc, argv,"usb_cam");
    ros::NodeHandle nh;

    //load coeficients
    initialize_sensor("/blob_detection_tools/");

    //publishers
    rgb_image_pub=nh.advertise<sensor_msgs::Image>("usb_cam_node/sensor/rgb_image",1);
    bin_image_pub=nh.advertise<sensor_msgs::Image>("usb_cam_node/sensor/binary_image",1);
    detections_pub=nh.advertise<usb_cam::detections>("usb_cam_node/sensor/detections",1);
    markers_pub=nh.advertise<usb_cam::QuadPoseList>("usb_cam_node/sensor/markers",1);

    //subscribers
    ros::Subscriber image_sub=nh.subscribe("usb_cam/image_reduced",1,image_reception_callback); // only 1 in buffer size to drop other images if processing is not finished

    //main loop
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }
}

void image_reception_callback(const sensor_msgs::ImageConstPtr& msg)
{
    //extract image
    const sensor_msgs::Image img = *msg;

    //copy image
    unsigned long int cnt=0;
    for(unsigned int l=0;l<img.height;l++)
        for(unsigned int k=0;k<img.width;k++){
            buf[cnt]=img.data[cnt];
            buf[cnt+1]=img.data[cnt+1];
            buf[cnt+2]=img.data[cnt+2];
            cnt+=3;
        }

    //blob detection and publish binary image
    nblobs=detect_blobs(buf,3);
    for(unsigned int l=0;l<img.height;l++)
        for(unsigned int k=0;k<img.width;k++)
            if(bufb[l*img.width + k]>0)
                bufb[l*img.width + k]=255;

    //publishing usefull information
    sensor_msgs::Image rgb_img = *msg;
    fillImage(rgb_img,"rgb8",img.height,img.width,img.step,buf);
    rgb_image_pub.publish(rgb_img);
    sensor_msgs::Image bin_img = *msg;
    fillImage(bin_img,"mono8",img.height,img.width,img.step/3,bufb);
    bin_image_pub.publish(bin_img);
    usb_cam::detections det_msg;
    det_msg.header = img.header;
    for(int k=0;k<nblobs;k++)
        if(blp[k].valid){
            det_msg.pos_x.push_back(blp[k].x);
            det_msg.pos_y.push_back(blp[k].y);
            det_msg.size.push_back(blp[k].sz);
        }
    detections_pub.publish(det_msg);







    //detect markers and publish them to topic
    nobjects=detect_markers();
    usb_cam::QuadPoseList quad_poses_msg;
    quad_poses_msg.header.stamp = ros::Time::now();
    Eigen::Matrix3d rotation;
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
      Eigen::Vector3d ea = rotation.eulerAngles(2, 1, 0); //the order of the angles matter
      double roll = ea[2];
      double pitch = ea[1];
      double yaw = ea[0];
      quad_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
      quad_pose.pose_updated=1;
      quad_poses_msg.poses.push_back(quad_pose);
    }
    markers_pub.publish(quad_poses_msg);
    return;
}

//sensor parameters
void initialize_sensor(std::string param_name)
{
    double fx,fy,cx,cy,kx1,kx2,ky1,ky2;
    //double roll,pitch,yaw,roll_cam,pitch_cam,yaw_cam,x_cam,y_cam,z_cam;
    int width,height;

    //get parameters
    ros::NodeHandle nh; // is the same handle as in the main, seems ros is global
    std::string full_name;
    full_name = param_name + "image_width";
    nh.getParam(full_name.c_str(),width);
    full_name = param_name + "image_height";
     nh.getParam(full_name.c_str(),height);
    full_name = param_name + "fx";
    nh.getParam(full_name.c_str(),fx);
    full_name = param_name + "fy";
    nh.getParam(full_name.c_str(),fy);
    full_name = param_name + "cx";
    nh.getParam(full_name.c_str(),cx);
    full_name = param_name + "cy";
    nh.getParam(full_name.c_str(),cy);
    full_name = param_name + "kx1";
    nh.getParam(full_name.c_str(),kx1);
    full_name = param_name + "kx2";
    nh.getParam(full_name.c_str(),kx2);
    full_name = param_name + "ky1";
    nh.getParam(full_name.c_str(),ky1);
    full_name = param_name + "ky2";
    nh.getParam(full_name.c_str(),ky2);

    //initialize sensor
    objects=sensor_init(width,height,fx,fy,cx,cy,kx1,kx2,ky1,ky2);
    buf=(unsigned char*)malloc(width*height*3*sizeof(unsigned char));
    bufb=get_binary_image();
    blp=get_blobs();
}