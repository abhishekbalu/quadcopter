#include <list>
#include "ros/ros.h"
#include <math.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>

#include <ros/serialization.h>
#include <sstream>
#include <typeinfo>


//Global variables
sensor_msgs::Range height;
geometry_msgs::Quaternion *imu = NULL;

//Create local deepcopies of received data
void getHeight(const sensor_msgs::Range::ConstPtr& data){

	height.header = data->header;
	height.radiation_type = data->radiation_type;
	height.field_of_view = data->field_of_view;
	height.min_range = data->min_range;
	height.max_range = data->max_range;
	height.range = data->range;
}

void getImu(const sensor_msgs::Imu::ConstPtr& data){
	if(imu == NULL){
		imu = new geometry_msgs::Quaternion;
	}
	imu = data->orientation;

}
void tf_dyn(tf::TransformBroadcaster br){
	tf::Transform t;
	t.setOrigin(tf::Vector3(0.0, 0.0, height));
	tf::Quaternion q(imu->x, imu->y, imu->z, imu->w);
	t.setRotation(q);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "base_footprint"));
}

void tf_st(tf::TransformBroadcaster br){
	tf::Transform t;

	t.setOrigin(tf::Vector3(0.0, 0.0, 0.118));
	tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
	t.setRotation(q);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "laser_link", "base_link"));

	t.setOrigin(tf::Vector3(0.1, 0.096, 0.0));
	q.setEuler(0.0, 0.0, 3*M_PI/4)
	t.setRotation(q);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "sonar_link", "base_link"));

}
//THIS FILE TRANSFORMS THE WORLD COORDINATES TO QUAD COORDINATES VIA TRANSFORMS
int main(int argc, char **argv){


	ros::init(argc, argv, "position");
	ros::NodeHandle n;
	static tf::TransformBroadcaster br;
	ros::Subscriber sonar = n.subscribe("/sonar_filtered", 10, getHeight);
	ros::Subscriber imu = n.subscribe("/raw_imu", 10, getImu);


	ros::Rate loop_rate(100);
	bool x = TRUE;
	while(ros::ok()){
		if(imu != NULL){
			tf_dyn(br);
			if(x)
				tf_st(br);
			x = !x;
		}

		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}