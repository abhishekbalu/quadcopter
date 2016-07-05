//C++ Libraries
#include <list>
#include <math.h>
#include <sstream>
#include <typeinfo>
//ROS Libraries
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <px_comm/OpticalFlow.h>
#include <nav_msgs/Odometry.h>
//User Libraries
#include "height/debug.h" //Comment or uncomment this for verbose output

#define RATE 20
px_comm::OpticalFlow *OF = NULL;


void getGndTruth(const nav_msgs::Odometry::ConstPtr& data){
	if(OF == NULL){
		OF = new px_comm::OpticalFlow;
		OF->header.seq=0;
	}
	OF->header.seq++;
	OF->ground_distance = data->pose.pose.position.z;
	OF->velocity_x = data->twist.twist.linear.x;
	OF->velocity_y = data->twist.twist.linear.y;
	OF->quality = 255;
	#ifdef VERBOSE
		ROS_INFO("Got linear velocities: velocity_x: [%f], velocity_y: [%f] with quality [%d]", OF->velocity_x, OF->velocity_y, OF->quality);
	#endif
}


int main(int argc, char **argv){
	ROS_INFO("Started simulated_OptFlow...\n");
	ros::init(argc, argv, "simulated_OptFlow");
	ros::NodeHandle n;
	

	ros::Subscriber sonar = n.subscribe("/ground_truth/state", 10, getGndTruth); 
	ros::Publisher pub = n.advertise<px_comm::OpticalFlow>("/simOptFlow",10);
	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		pub.publish(*OF);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}