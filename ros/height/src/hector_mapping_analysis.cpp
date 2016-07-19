//C++ Libraries
#include <iostream>     
#include <cmath>
//ROS Libraries
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
//User Libraries
#include "height/debug.h" //Comment or uncomment this for verbose
using namespace std;

#define RATE 100
geometry_msgs::Quaternion *imu = NULL;
geometry_msgs::Pose *slam = NULL;
geometry_msgs::Pose *gnd_truth = NULL;

geometry_msgs::Pose error;

void getSlam(const geometry_msgs::PoseStamped::ConstPtr& data){
	if(slam == NULL){
		slam = new geometry_msgs::Pose;
	}
	slam->position.x = data->pose.position.x;
	slam->position.y = data->pose.position.y;
	slam->position.z = data->pose.position.z;
}

void getGndTruth(const nav_msgs::Odometry::ConstPtr& data){
	if(gnd_truth == NULL){
		gnd_truth = new geometry_msgs::Pose;
	}
	gnd_truth->position.x = data->pose.pose.position.x;
	gnd_truth->position.y = data->pose.pose.position.y;
	gnd_truth->position.z = data->pose.pose.position.z;
}

void getImu(const sensor_msgs::Imu::ConstPtr& data){
	if(imu == NULL){
		imu = new geometry_msgs::Quaternion;
	}
	imu->x = data->orientation.x;
	imu->y = data->orientation.y;
	imu->z = data->orientation.z;
	imu->w = data->orientation.w;
}

void calculate_errors(){
	error.position.x = abs(slam->position.x - gnd_truth->position.x);
	printf("Error X: %f\n", error.position.x);
	error.position.y = abs(slam->position.y - gnd_truth->position.y);
	error.position.z = abs(slam->position.z - gnd_truth->position.z); 

}

int main(int argc, char **argv){
	ROS_INFO("Started Hector Analysis...\n");
	ros::init(argc, argv, "hector_analysis");
	ros::NodeHandle n;
	
	ros::Subscriber gnd_sub = n.subscribe("/gnd_truth/state", 10, getGndTruth); 
	ros::Subscriber slam_sub = n.subscribe("/slam_out_pose", 10, getSlam);
	ros::Subscriber imu_sub = n.subscribe("/raw_imu", 10, getImu);
	ros::Publisher pub = n.advertise<geometry_msgs::Pose>("/slam_error",10);
	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		calculate_errors();
		pub.publish(error);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}