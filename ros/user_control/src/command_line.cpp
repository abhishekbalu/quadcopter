//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
using namespace std;

int main(int argc, char** argv){

	//initialize ros node
	ros::init(argc, argv,"user_control");
	ros::NodeHandle nh;

	//variables for command stroage
	string command;
	std_msgs::String msg;

	//initialize publisher
	ros::Publisher user_pub=nh.advertise<std_msgs::String>("user/command_out",5);

	//main loop
	while(ros::ok()){

		//read the command
		getline(cin,command);

		//send it to the controller
		msg.data = command;
		user_pub.publish(msg);
	}

	return 0;
}