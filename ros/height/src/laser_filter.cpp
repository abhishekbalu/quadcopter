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

geometry_msgs::Pose slam;
geometry_msgs::Pose error;

void getSlam(const geometry_msgs::PoseStamped::ConstPtr& data){
	slam.position.x = data->pose.position.x;
	printf("%d", data->pose.position.x);
	slam.position.y = data->pose.position.y;
	slam.position.z = data->pose.position.z;
}

void getGndTruth(const nav_msgs::Odometry::ConstPtr& data){
	float testx = abs(abs(slam.position.x) - abs(data->pose.pose.position.x));
	float testy = abs(abs(slam.position.y) - abs(data->pose.pose.position.y));
	float testz = abs(abs(slam.position.z) - abs(data->pose.pose.position.z));

	error.position.x = abs(abs(slam.position.x) - abs(data->pose.pose.position.x));
	error.position.y = abs(abs(slam.position.y) - abs(data->pose.pose.position.y));
	error.position.z = abs(abs(slam.position.z) - abs(data->pose.pose.position.z));
	cout << testx << endl;
	cout << testy << endl;
	cout << testz << endl;
}


int main(int argc, char **argv){
	ROS_INFO("Started Hector Analysis...\n");
	ros::init(argc, argv, "hector_analysis");
	ros::NodeHandle n;
	
	ros::Subscriber gnd_sub = n.subscribe("/gnd_truth/state", 1, getGndTruth); 
	ros::Subscriber slam_sub = n.subscribe("/slam_out_pose", 1, getSlam);

	ros::Publisher pub = n.advertise<geometry_msgs::Pose>("/slam_error",1);
	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		pub.publish(error);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}