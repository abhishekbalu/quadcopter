#include <list>
#include "ros/ros.h"
#include <math.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <sstream>
#include <typeinfo>

sensor_msgs::Range *config=NULL;
sensor_msgs::Range *msg;
struct State{ 
	float u;
	float std_o;
	float o;
	float valid_z;
	float last_valid;
	std::list<float> reads;
	State():u(0.0), std_o(0.2), o(0.2), valid_z(0.0), last_valid(0.0){}
}state;


void filterSonar(ros::Publisher pub){
	float t_res = 0.01;
	//Last value
	float z = state.reads.back();
	//Mean
	float sum = 0;
	for(std::list<float>::iterator it= state.reads.begin(); it != state.reads.end(); ++it){
		sum = sum + *it;
	}
	float mean=sum/state.reads.size();
	//Variance
	float var = 0.0;
	for(std::list<float>::iterator it= state.reads.begin(); it != state.reads.end(); ++it){
		var+=pow((*it-mean),2);
	}
	var = var / (state.reads.size()-1);
	//If variance is below t_res, then last value is valid
	if(var < t_res){
		state.valid_z = z;
		state.last_valid = z;
		//New value is valid if it's within last valid and the margin given by sqrt(t_res)
	}else if (abs(state.last_valid - z) < sqrt(t_res)){
		state.valid_z = z;
	}
	//Publish
	config->header.seq += 1;
	msg = new sensor_msgs::Range;
	msg->header = config->header;
	msg->radiation_type = config->radiation_type;
	msg->field_of_view = config->field_of_view;
	msg->min_range = config->min_range;
	msg->max_range = config->max_range;


	msg->header.stamp=ros::Time::now();
	msg->range = state.valid_z;

	//std::cout << typeid(*msg).name() << '\n';
	ROS_INFO("Filtered Seq: [%d]", msg->header.seq);
	ROS_INFO("Filtered Range: [%f]", msg->range);
	pub.publish(*msg);
}

void getSonar(const sensor_msgs::Range::ConstPtr& data){
	ROS_INFO("Received Seq [%d]", data->header.seq);
	ROS_INFO("Received Range [%f]", data->range);
	//Do a deep copy
	if (config == NULL){
		config = new sensor_msgs::Range;
		config->header = data->header;
		config->radiation_type = data->radiation_type;
		config->field_of_view = data->field_of_view;
		config->min_range = data->min_range;
		config->max_range = data->max_range;
		config->range = data->range;
		config->header.seq = 0;
	}
	float aux = data->range;
	state.reads.push_back(aux);
	if (state.reads.size() > 4){
		if(state.reads.size() == 6){
			state.reads.pop_front();
		}
	}

}

int main(int argc, char **argv){

	ros::init(argc, argv, "filter_sonar");
	ros::NodeHandle n;

	ros::Subscriber sonar = n.subscribe("/mavros/px4flow/ground_distance", 10, getSonar); //May use the simulation sonar here
	ros::Publisher pub = n.advertise<sensor_msgs::Range>("/sonar_filtered",10);
	ros::Rate loop_rate(5);

	while(ros::ok()){
		if(state.reads.size() > 4){
			filterSonar(pub);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}