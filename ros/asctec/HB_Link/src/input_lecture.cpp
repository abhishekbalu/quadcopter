#include "input_lecture.h"

using namespace std;

void reset_char_array(char* array, int size){
	int i;
	for(i=0;i<size;i++){
		array[i]=0;
	}
}

void fill_char_array(char* array, std::string &string){
	int i;
	for(i=0;i<(int)string.size();i++){
		array[i]=string[i];
	}
}	
	/*
void create_message_waypointfeedback(const asctec_hl_comm::WaypointFeedbackConstPtr & msg, std::string &output){
	stringstream ss;

	const geometry_msgs::Point32 & wp = msg->current_pos;
	ss<<"WaypointFeedback"<<"/"<<msg->header.seq<<"/"<<msg->header.stamp<<"/"<<wp.x<<"/"<<wp.y<<"/"<<wp.z<<"/"<<msg->current_yaw;
	
	output.clear();
	output=ss.str();
}*/

void create_message_currentpose(const geometry_msgs::PoseStampedConstPtr & msg, std::string &output){

	stringstream ss;

	ss<<"CurrentPose"<<"/"<<msg->header.seq<<"/"<<msg->header.stamp<<"/"<<msg->pose.position.x<<"/"<<msg->pose.position.y<<"/"<<msg->pose.position.z;
	
	output.clear();
	output=ss.str();

}


