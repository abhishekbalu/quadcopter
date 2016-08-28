#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cmath>
#include <sstream>
#include <std_msgs/String.h>

#include <quad_msgs/Estimate.h>
#include <quad_msgs/EstimateSingle.h>
#include <main_control/string_stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

using namespace std;

#include "udp_server.h"
#include "udp_client.h"

#define TIMER_TH 0.5

std::string input;
std::string command;
int ret_server=0;
int asctec = 0;

double fcu_timer;
int fcu_OK;
void asctec_input_callback(const geometry_msgs::PoseStamped::ConstPtr& new_state);
void px4_input_callback(const sensor_msgs::Imu::ConstPtr& new_state);
double mcs_timer;
int mcs_OK;
void motion_analysis_input_callback(const geometry_msgs::PoseStamped::ConstPtr& new_state);
double estimator_timer;
int estimator_OK;
void estimator_input_callback(const quad_msgs::EstimateSingle::ConstPtr& new_state);
double main_control_timer;
volatile int main_control_OK;
string main_control_state;
void main_control_input_callback(const main_control::string_stamped::ConstPtr& msg);

int main(int argc, char** argv){

	//initialize ros node
	string node_name = "user_control";
	ros::init(argc, argv,node_name.c_str());
	ros::NodeHandle nh;

	//get fcu mode
	string full_name = node_name + "asctec";
	nh.getParam(node_name.c_str(), asctec);

	
	//initialize UDP server
	Udp_server *server_interaction = new Udp_server(argv[1]);

	//initialize publisher
	ros::Publisher user_pub=nh.advertise<std_msgs::String>("user/command_out",5);

	//initialize subscribers
	ros::Subscriber fcuinput;
	if(asctec == 1)
		fcuinput=nh.subscribe("fcu/current_pose", 1, asctec_input_callback);
	else
		fcuinput=nh.subscribe("mavros/imu/data", 1, px4_input_callback);
	ros::Subscriber motion_capture_input=nh.subscribe("macortex_bridge/mcs_pose", 1, motion_analysis_input_callback);

	ros::Subscriber estimator_input=nh.subscribe("estimator/estimate_self", 1, estimator_input_callback);
	ros::Subscriber main_control_input=nh.subscribe("main_control/state", 1, main_control_input_callback);
	
	//initializations
	std_msgs::String msg;
	fcu_timer=0.0; fcu_OK=1;
	mcs_timer=0.0; mcs_OK=1;
	estimator_timer=0.0; estimator_OK=1;
	
	//run program
	while(ros::ok())
	{

		//get xbox command
		ret_server=server_interaction->msg_box(input);
		
		if(ret_server==1)
		{
			printf("%s\n",input.c_str());

			if(!strcmp(input.c_str(),"status"))
			{
				fcu_OK=((ros::Time::now().toSec()-fcu_timer>TIMER_TH)?0:1);
				mcs_OK=((ros::Time::now().toSec()-mcs_timer>TIMER_TH)?0:1);
				estimator_OK=((ros::Time::now().toSec()-estimator_timer>TIMER_TH)?0:1);
				msg.data = "state";
				user_pub.publish(msg);
				main_control_OK=0; main_control_timer=ros::Time::now().toSec();
				while((ros::Time::now().toSec()-main_control_timer<TIMER_TH) && (main_control_OK == 0)) ros::spinOnce();

				//send message back to the caller
				stringstream ss;
				ss << "FCU_" << (fcu_OK?"OK":"DOWN") << endl;
				ss << "MCS_" << (mcs_OK?"OK":"DOWN") << endl;
				ss << "ESTIMATOR_" << (estimator_OK?"OK":"DOWN") << endl;
				ss << "MAIN_CONTROL_" << (main_control_OK?"OK-main_control_status " + main_control_state:"DOWN") << endl;
				command=ss.str();
				string client_name(server_interaction->get_addr_name());
				Udp_client* client_interaction = new Udp_client((char*)client_name.c_str(),argv[1]);
				//cout << command << endl;
				client_interaction->send_msg(command.c_str());
			}

			//send it to the controller
			msg.data = input;
			user_pub.publish(msg);
			
		}
		
		
		//run ros
		ros::spinOnce();
		
	}

	return 0;
	
}

void asctec_input_callback(const geometry_msgs::PoseStamped::ConstPtr& new_state)
{
	fcu_timer = ros::Time::now().toSec();
}
void px4_input_callback(const sensor_msgs::Imu::ConstPtr& new_state)
{
	fcu_timer = ros::Time::now().toSec();
}
void motion_analysis_input_callback(const geometry_msgs::PoseStamped::ConstPtr& new_state)
{
	mcs_timer = ros::Time::now().toSec();
}
void estimator_input_callback(const quad_msgs::EstimateSingle::ConstPtr& new_state)
{
	estimator_timer = ros::Time::now().toSec();
}
void main_control_input_callback(const main_control::string_stamped::ConstPtr& msg)
{
	main_control_state=msg->data;
	main_control_OK=1;
}
