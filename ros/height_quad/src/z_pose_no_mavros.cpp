//C++ Libraries
#include <list>
#include <math.h>
//ROS Libraries
#include "ros/ros.h"
#include <sensor_msgs/Range.h>

#include <px_comm/OpticalFlow.h>
//User Libraries
#include "height_quad/Kalman_1D.h"
#include "height_quad/debug.h"

#define CENTER_OF_MASS_FILTER_OFFSET 0.06
#define NUMB_VALUES_TO_DISCARD 4
#define RATE 10
#define MAX_DISTANCE 3.0	
#define MIN_DISTANCE 0.2

Kalman_1D kalman(0.2, 32, 1, 0.14); //Q, R, P, z initial sensor position(height of the sensor)

px_comm::OpticalFlow *config=NULL;
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
	for(std::list<float>::iterator it=state.reads.begin(); it != state.reads.end(); ++it){
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
	//Copy everything
	config->header.seq += 1;
	msg = new sensor_msgs::Range;
	msg->header = config->header;
	//msg->radiation_type = config->radiation_type;
	//msg->field_of_view = config->field_of_view;
	msg->min_range = MAX_DISTANCE;
	msg->max_range = MIN_DISTANCE;

	msg->header.stamp=ros::Time::now();
	msg->range = state.valid_z;

	//std::cout << typeid(*msg).name() << '\n';
	ROS_INFO("Filtered Seq: [%d]", msg->header.seq);
	ROS_INFO("Filtered Range: [%f]", msg->range);
	pub.publish(*msg);
}

void getSonar(const px_comm::OpticalFlow::ConstPtr& data){
	#ifdef VERBOSE
		ROS_INFO("Received Seq [%d]", data->header.seq);
		ROS_INFO("Received Range [%f]", data->ground_distance);
	#endif 
	//Do a deep copy
	//For more about deepcopies https://www.cs.utexas.edu/~scottm/cs307/handouts/deepCopying.htm
	if (config == NULL){
		config = new px_comm::OpticalFlow;
		//Deepcopy of config
		config->header = data->header;
		config->velocity_x = data->velocity_x;
		config->velocity_y = data->velocity_y;
		config->flow_x = data->flow_x;
		config->flow_y = data->flow_y;
		config->ground_distance = data->ground_distance;
		config->header.seq = 0;

	}
	//Use the ground_distance parameter and reject outliers
	if(data->ground_distance < MAX_DISTANCE && data->ground_distance > MIN_DISTANCE){
		float aux = kalman.KalmanFilter(data->ground_distance + CENTER_OF_MASS_FILTER_OFFSET);
		state.reads.push_back(aux);
		if(state.reads.size() > 4){
			if(state.reads.size() == 6){
				state.reads.pop_front();
			}
		}
	}
}

int main(int argc, char **argv){
	ROS_INFO("Started z_pose...\n");
	ros::init(argc, argv, "z_pose");
	ros::NodeHandle n;

	ros::Subscriber sonar = n.subscribe("/px4flow/opt_flow", 10, getSonar); //May use the simulation sonar here
	ros::Publisher pub = n.advertise<sensor_msgs::Range>("/z_pose",10);
	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		if(state.reads.size() > NUMB_VALUES_TO_DISCARD){
			filterSonar(pub);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}
