//C++ Libraries
#include <math.h>
//ROS Libraries
#include "ros/ros.h"
#include <mavros_msgs/OpticalFlowRad.h>
#include <geometry_msgs/TwistStamped.h>
//User Libraries
#include "height_quad/debug.h"

#define QUALITY_THRESHOLD 7
#define RATE 20
/*This should not only publish the position but also filter and publish the velocity*/
geometry_msgs::TwistStamped msg;

void getOdom(const mavros_msgs::OpticalFlowRad::ConstPtr& data){

	if(data->quality<=QUALITY_THRESHOLD){
		#ifdef VERBOSE
			ROS_INFO("Quad is standing still...");
		#endif
		msg.twist.linear.x = 0;
		msg.twist.linear.y = 0;
		return;
	}
	if(data->integration_time_us==0)
		return;
	if(data->distance==0.0)
		return;
	#ifdef VERBOSE
		ROS_INFO("READING @ Temperature: [%d]", data->temperature); //Log temperature
	#endif
	//Changes in variable assignment and sign to acomodate referencial changes in angular velocity

	//Velocity is position over time (keep in mind, integraded x is adimensional)
	msg.twist.linear.y = - pow(10, 6) * ((data->integrated_y / data->integration_time_us) / data->distance);
	msg.twist.linear.x = pow(10, 6) * ((data->integrated_x / data->integration_time_us) / data->distance);
	msg.twist.linear.z = 0.0; //Optical flow only knows of the linear x and y (sonar handles the zx 

	msg.twist.angular.y = pow(10, 6) * (data->integrated_ygyro) / data->integration_time_us;
	msg.twist.angular.x = pow(10, 6) * (data->integrated_xgyro) / data->integration_time_us;
	msg.twist.angular.z = pow(10, 6) * (data->integrated_zgyro) / data->integration_time_us;

	#ifdef VERBOSE
		ROS_INFO("Linear Velocity(x): [%f] Linear Velocity(y): [%f]", msg.twist.linear.x, msg.twist.linear.y);
		ROS_INFO("Angular Velocity(x): [%f] Angular Velocity(y): [%f] Angular Velocity(z): [%f]", msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
	#endif	
	return;
}

int main(int argc, char** argv){
	ROS_INFO("Started odometry node...\n");
	ros::init(argc, argv, "velocity_xy");
	ros::NodeHandle n;
	ros::Subscriber optflow = n.subscribe("/mavros/px4flow/raw/optical_flow_rad", 10, getOdom); 
	ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>("/velocity_xy", 10);
	ros::Rate loop_rate(RATE);

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
