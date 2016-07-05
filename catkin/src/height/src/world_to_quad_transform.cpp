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
/*
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 
*/

/*
# Single range reading from an active ranger that emits energy and reports
# one range reading that is valid along an arc at the distance measured. 
# This message is  not appropriate for laser scanners. See the LaserScan
# message if you are working with a laser scanner.

# This message also can represent a fixed-distance (binary) ranger.  This
# sensor will have min_range===max_range===distance of detection.
# These sensors follow REP 117 and will output -Inf if the object is detected
# and +Inf if the object is outside of the detection range.

Header header           # timestamp in the header is the time the ranger
                        # returned the distance reading

# Radiation type enums
# If you want a value added to this list, send an email to the ros-users list
uint8 ULTRASOUND=0
uint8 INFRARED=1

uint8 radiation_type    # the type of radiation used by the sensor
                        # (sound, IR, etc) [enum]

float32 field_of_view   # the size of the arc that the distance reading is
                        # valid for [rad]
                        # the object causing the range reading may have
                        # been anywhere within -field_of_view/2 and
                        # field_of_view/2 at the measured range. 
                        # 0 angle corresponds to the x-axis of the sensor.

float32 min_range       # minimum range value [m]
float32 max_range       # maximum range value [m]
                        # Fixed distance rangers require min_range==max_range

float32 range           # range data [m]
                        # (Note: values < range_min or > range_max
                        # should be discarded)
                        # Fixed distance rangers only output -Inf or +Inf.
                        # -Inf represents a detection within fixed distance.
                        # (Detection too close to the sensor to quantify)
                        # +Inf represents no detection within the fixed distance.
                        # (Object out of range)
*/

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