//C++ Libraries
#include <list>
#include <math.h>
#include <sstream>
#include <typeinfo>
//ROS Libraries
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
//User Libraries
#include "height_quad/debug.h"

#define QUALITY_THRESHOLD 5
/*	From: http://docs.ros.org/api/mavros_msgs/html/msg/OpticalFlowRad.html
OPTICAL_FLOW_RAD message data

std_msgs/Header header

uint32 integration_time_us
float32 integrated_x
float32 integrated_y
float32 integrated_xgyro
float32 integrated_ygyro
float32 integrated_zgyro
int16 temperature
uint8 quality
uint32 time_delta_distance_us
float32 distance 
*/

/* From: http://docs.ros.org/jade/api/geometry_msgs/html/msg/TwistWithCovariance.html
		 http://docs.ros.org/jade/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html
# This expresses velocity in free space with uncertainty.

Twist twist

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance
# This represents an estimated twist with reference coordinate frame and timestamp.
Header header
TwistWithCovariance twist
*/

/*This should not only publish the position but also filter and publish the velocity*/
geometry_msgs::TwistWithCovarianceStamped msg;

void getOdom(const mavros_msgs::OpticalFlowRad::ConstPtr& data){

		if(data->quality<=QUALITY_THRESHOLD){
			ROS_INFO("Quad is standing still...");
			msg.twist.twist.linear.x = 0;
			msg.twist.twist.linear.y = 0;
			return;
		}
		if(data->integration_time_us==0)
			return;
		if(data->distance==0.0)
			return;
	ROS_INFO("READING @ Temperature: [%d]", data->temperature); //Log temperature

	//Changes in variable assignment and sign to acomodate referencial changes in angular velocity

	//Velocity is position over time (keep in mind, integraded x is adimensional)
	msg.twist.twist.linear.y = pow(10, 6) * (data->integrated_y * data->distance) / data->integration_time_us;
	msg.twist.twist.linear.x = - pow(10, 6) * (data->integrated_x * data->distance) / data->integration_time_us;
	msg.twist.twist.linear.z = 0; //Optical flow only knows of the linear x and y (sonar handles the zx 

	msg.twist.twist.angular.y = pow(10, 6) * (data->integrated_ygyro) / data->integration_time_us;
	msg.twist.twist.angular.x = pow(10, 6) * (data->integrated_xgyro) / data->integration_time_us;
	msg.twist.twist.angular.z = pow(10, 6) * (data->integrated_zgyro) / data->integration_time_us;


	//Maybe we should filter this data?

	float quantization_uncertainty = pow(10, -((float)data->quality / ((255.0)/(6.0)) )); //Uncertainty because the quality is an 8 bit value
	/*Covariance Matrix (Notice that only x and y for linear velocity)
	[0, 0, 0, X, 0, 0,
	 0, 0, 0, 0, X, 0,
	 0, 0, 0, 0, 0, X,
	 0, 0, 0, 0, 0, 0,
	 X, 0, 0, 0, 0, 0,
	 0, X, 0, 0, 0, 0]
	*/
	for(int i = 0; i<36; i++){
		if(i==3||i==10||i==17||i==24||i==31){
			msg.twist.covariance[i] = quantization_uncertainty;
		}else{
			msg.twist.covariance[i] = 0;	
		}
	}

	ROS_INFO("Linear Velocity(x): [%f] Linear Velocity(y): [%f]", msg.twist.twist.linear.x, msg.twist.twist.linear.y);
	ROS_INFO("Angular Velocity(x): [%f] Angular Velocity(y): [%f] Angular Velocity(z): [%f]", msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
	return;
}

int main(int argc, char** argv){

	ROS_INFO("Started odometry node...\n");
	ros::init(argc, argv, "velocity_xy");
	ros::NodeHandle n;
	ros::Subscriber optflow = n.subscribe("/mavros/px4flow/raw/optical_flow_rad", 10, getOdom); 
	ros::Publisher pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/velocity_xy", 10);
	ros::Rate loop_rate(20);

	msg.header.seq = 0;
	msg.header.frame_id = "sonar_link";


	while(ros::ok()){
		msg.header.seq++;
		msg.header.stamp = ros::Time::now();
		pub.publish(msg);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;
}
