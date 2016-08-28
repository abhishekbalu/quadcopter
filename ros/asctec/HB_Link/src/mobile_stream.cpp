#include "mobile_stream.h"

using namespace std;
using namespace ros;

/*Global variables for communication with fix PC*/
//Udp_client *client_stream;
/*
void feedbackCallback(const asctec_hl_comm::WaypointFeedbackConstPtr & msg){
	std::string output;
	char output_c[MAX_INPUT_SIZE];
	const geometry_msgs::Point32 & wp = msg->current_pos;
	ROS_INFO("I heard: [%f] [%f] [%f] [%f]", wp.x, wp.y, wp.z, msg->current_yaw*180/M_PI);
	
	create_message_waypointfeedback(msg, output);
	reset_char_array(output_c, MAX_INPUT_SIZE);
	fill_char_array(output_c, output);
	
	if(output.size()!=0){
		client_stream->send_msg(output_c);
	}
}*/

void currentposeCallback(const geometry_msgs::PoseStampedConstPtr & msg){
	
	std::string output;
	char output_c[MAX_INPUT_SIZE];
	
	ROS_INFO("I heard: [%f] [%f] [%f]",msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	
	create_message_currentpose(msg, output);
	reset_char_array(output_c, MAX_INPUT_SIZE);
	fill_char_array(output_c, output);

	/*if(output.size()!=0){
		client_stream->send_msg(output_c);
	}*/
}

int main(int argc, char ** argv){
	
	ros::init(argc,argv,"mobile_stream"); 
	ros::NodeHandle n;
	ros::Rate loop_rate(LOOP_RATE);
	//ros::Subscriber feedback_sub;
	ros::Subscriber current_pose_sub;
	
	//client_stream = new Udp_client(argv[1], argv[2]);

	//feedback_sub = n.subscribe("info/feedback", 1000, feedbackCallback);
	current_pose_sub = n.subscribe("fcu/current_pose", 1000, currentposeCallback);
	
	ros::spin();
	
	//delete client_stream;
	
	return 0;
}
