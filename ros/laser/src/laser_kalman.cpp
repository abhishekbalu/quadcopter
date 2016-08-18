//C++ Libraries
#include <math.h>
#include <iostream>
//ROS Libraries
#include "tf/transform_datatypes.h"
#include <px_comm/OpticalFlow.h>
#include "ros/ros.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
//User Libraries
#include "laser/state.h"
#include "laser/debug.h" //Comment or uncomment this for verbose
#include "kalman/ekfilter.hpp" 

using namespace Eigen;

#define PI 3.14159265

const int RATE = 50;


int main(int argc, char** argv){
	ROS_INFO("Started kalman xy_pose...\n");
	ros::init(argc, argv, "kalman_xy_pose");
    ros::NodeHandle n, nh("~");



    //Subscribers
    /*
    ros::Subscriber opt = n.subscribe("/px4flow/opt_flow", 1, getOptFlow);
	ros::Subscriber imu = n.subscribe("/mavros/imu/data", 10, getImu);
	ros::Subscriber laser = nh.subscribe("/slam_out_pose", 1, getLaser);
	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/difference", 10);
	*/
	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		//pub.publish(array);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}
