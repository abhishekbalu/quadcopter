#include "fix_interaction.h"

using namespace std;
using namespace ros;

/*Global variables for ROS feedback functions*/
std::string command;

void commandCallback(const std_msgs::String::ConstPtr& msg){
	/* Info given to global variable*/
	command = msg->data.data();
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char ** argv){
	
	ros::init(argc,argv,"fix_interaction"); 
	ros::NodeHandle n;
	ros::Rate loop_rate(LOOP_RATE);
	ros::Subscriber command_sub;
	
	Udp_client *client_interaction=new Udp_client(argv[1],argv[2]);
	Udp_server *server_interaction=new Udp_server(argv[3]);
	
	int ret_client=0, ret_server=0;
	char command_c[COMMAND_LENGTH];
	std::string input;
	
	while(ros::ok()){
	
	//command entry 
		command_sub = n.subscribe("command", 1000, commandCallback);
		reset_char_array(command_c,COMMAND_LENGTH);
		fill_char_array(command_c,command);
	//send message to quadrotor
		if(command.size()!=0){
			if((ret_client=client_interaction->send_msg(command_c))){
				ROS_INFO("FIX_INTERACTION: Message sent to quadrotor.");
			}else{
				ROS_INFO("FIX_INTERACTION: NO message to send to quadrotor.");
			}	
			command.clear();
		}
	//receiving messages from quad
		if((ret_server=server_interaction->msg_box(input))){
		//print answer from quadrotor
			cout<<input<<endl;
		}else if(ret_server==2){
			ROS_INFO("FIX_INTERACTION: Server socket timeout.");
		}else if(ret_server==0){
			ROS_INFO("FIX_INTERACTION: Server socket error.");
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	delete client_interaction;
	delete server_interaction;
	
	return 0;
}
