//Averaging Filter
#include <list>
#include "ros/ros.h"
#include <math.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <sstream>
#include <typeinfo>

#include "height/debug.h" //Comment or uncomment this for verbose
#include <height/Kalman_1D.h>

//This offset is due to the sensor not being on the exact center of mass of quad
#define CENTER_OF_MASS_FILTER_OFFSET 0

//Global variables -- this should be fixed
std::list<float> reads;
sensor_msgs::Range *config=NULL;
sensor_msgs::Range *msg;
float value;
int created=0;
Kalman_1D kalman(0.2, 32, 1, 0.14); //Q, R, P, z initial sensor position(height of the sensor)

void readings(ros::Publisher pub){
	if (reads.size() != 0){
		float sum = 0;
		/*The filtering is just averaging for now*/
		for(std::list<float>::iterator it= reads.begin(); it != reads.end(); ++it){
			sum = sum + *it;
		}
		value=sum/reads.size();

	
	}else{
		value=0;
	}
	if (config != NULL){
		config->header.seq += 1;
		msg = new sensor_msgs::Range;
		msg->header = config->header;
		msg->radiation_type = config->radiation_type;
		msg->field_of_view = config->field_of_view;
		msg->min_range = config->min_range;
		msg->max_range = config->max_range;
		msg->range = config->range;

		msg->header.stamp=ros::Time::now();
		msg->range=value;
		//std::cout << typeid(*msg).name() << '\n';
		#ifdef VERBOSE
			ROS_INFO("Filtered Seq: [%d]", msg->header.seq);
			ROS_INFO("Filtered Range: [%f]", msg->range);
		#endif
		pub.publish(*msg);

	}
}

void getSonar(const sensor_msgs::Range::ConstPtr& data){
	#ifdef VERBOSE
		ROS_INFO("Received Seq [%d]", data->header.seq);
		ROS_INFO("Received Range [%f]", data->range);
	#endif
	//Do a deep copy
	if (config == NULL){
		config = new sensor_msgs::Range;
		config->header = data->header;
		config->radiation_type = data->radiation_type;
		config->field_of_view = data->field_of_view;
		config->min_range = data->min_range;
		config->max_range = data->max_range;
		config->range = data->range;
	}
	//Append to the list
	//float aux = data->range + CENTER_OF_MASS_FILTER_OFFSET;
	float aux = kalman.KalmanFilter(data->range + CENTER_OF_MASS_FILTER_OFFSET);
	reads.push_back(aux);
	if(reads.size()>5){
		reads.pop_front();
	}
}

int main(int argc, char **argv){
	ROS_INFO("Started z_pose...\n");
	ros::init(argc, argv, "z_pose");
	ros::NodeHandle n;

	ros::Subscriber sonar = n.subscribe("/sonar_bottom", 10, getSonar); //May use the simulation sonar here
	ros::Publisher pub = n.advertise<sensor_msgs::Range>("/z_pose",10);
	ros::Rate loop_rate(5);

	while(ros::ok()){
		readings(pub);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}