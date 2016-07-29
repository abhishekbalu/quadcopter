// terminal.cpp
// reads from command line and sends commands to user_control node, to be parsed and sent to the quadrotor(s)

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <std_msgs/String.h>

#define LOOP_RATE 20

using namespace std;

int main(int argc, char** argv){
	// initialize ROS
	ros::init(argc,argv,"terminal");
	ros::NodeHandle nh;
	ros::Rate loop_rate(LOOP_RATE);

	// initialize publisher
	ros::Publisher term_pub = nh.advertise<std_msgs::String>("terminal/command_out",5);

	string command;
	std_msgs::String msg;

	while( ros::ok() ){
		// read user input from the command line
		// getline blocks until input in received
		getline(cin,command);

		msg.data = command;

		// publish the command
		term_pub.publish(msg);

		loop_rate.sleep();
	}

	return 0;
}