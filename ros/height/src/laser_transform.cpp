#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>

using namespace std;

#define PI 3.1415926535897
#define RATE 100

sensor_msgs::Range *height=NULL;
geometry_msgs::Quaternion *imu = NULL;

bool toggle = true;
static tf::TransformBroadcaster br;

void getImu(const sensor_msgs::Imu::ConstPtr& data){
	if(imu == NULL){
		imu = new geometry_msgs::Quaternion;
	}
	imu->x = data->orientation.x;
	imu->y = data->orientation.y;
	imu->z = data->orientation.z;
	imu->w = data->orientation.w;
}

void getHeight(const sensor_msgs::Range::ConstPtr& data){
	if(height == NULL){
		height = new sensor_msgs::Range;
	}
	height->header = data->header;
	height->radiation_type = data->radiation_type;
	height->field_of_view = data->field_of_view;
	height->min_range = data->min_range;
	height->max_range = data->max_range;
	height->range = data->range;
}

void tf_dyn(){
	tf::Transform t;
	t.setOrigin(tf::Vector3(0.0, 0.0, height->range));
	tf::Quaternion q(imu->x, imu->y, imu->z, imu->w);
	t.setRotation(q);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "base_link", "base_footprint"));
}

void tf_st(){
	tf::Transform t;

	t.setOrigin(tf::Vector3(0.0, 0.0, 0.118));
	tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
	t.setRotation(q);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "laser_link", "base_link"));

	t.setOrigin(tf::Vector3(0.1, 0.096, 0.0));
	q.setEuler(0.0, 0.0, 3*PI/4);
	t.setRotation(q);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "sonar_link", "base_link"));
}

int main(int argc, char** argv){
	ROS_INFO("Started frame transformations...\n");
	ros::init(argc, argv, "robot_publisher");
	ros::NodeHandle n;

	ros::Subscriber imu_sub = n.subscribe("/raw_imu", 10, getImu); 
	ros::Subscriber z_pose_sub = n.subscribe("/z_pose", 10, getHeight);
	ros::Rate loop_rate(RATE);


	while(ros::ok()){

		if(imu != NULL){
			tf_dyn();
			if(toggle){
				tf_st();
			}
			toggle = !toggle;
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;
}