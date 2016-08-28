//C++ Libraries
//ROS LIbraries
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#define RATE 20

geometry_msgs::TwistStamped msg;


void pubVel(ros::Publisher pub){
	msg.header.seq++;
	msg.header.stamp = ros::Time::now();
	pub.publish(msg);
}

void getVel(const geometry_msgs::Twist::ConstPtr& data){
	msg.twist = *data;
}

int main(int argc, char **argv){
	msg.header.seq = 0;
	msg.header.frame_id = "base_link";
	ros::init(argc, argv, "repeater_node");
	ros::NodeHandle n;

	ros::Subscriber vel = n.subscribe("/cmd_vel", 10, getVel); 
	ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		pubVel(pub);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}