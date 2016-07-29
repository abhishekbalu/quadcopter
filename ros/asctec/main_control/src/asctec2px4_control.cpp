#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>

using namespace Eigen;

Eigen::Quaternion<double> q_roll, q_pitch, q_yaw, q_quad, q_control;
double x, y, z, w;

ros::Publisher pub;
mavros_msgs::AttitudeTarget new_target;

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
	//extract imu orientation
	x = imu->orientation.x;
	y = imu->orientation.y;
	z = imu->orientation.z;
	w = imu->orientation.w;
}

//void control_callback(const mavros_msgs::AttitudeTarget::ConstPtr& target)
void control_callback(const geometry_msgs::Quaternion::ConstPtr& target)
{

	//extract control information
	double roll = target->x;
	double pitch = target->y;
	double yaw = target->z;
	double thrust = target->w;

	//get control quaternion from roll pitch and yaw
	q_roll = Eigen::Quaternion<double>(cos(roll/2), sin(roll/2), 0, 0);
	q_pitch = Eigen::Quaternion<double>(cos(pitch/2), 0, sin(pitch/2), 0);
	q_yaw = Eigen::Quaternion<double>(cos(yaw/2), 0, 0, sin(yaw/2));
	q_control = q_yaw * q_pitch * q_roll;

	//compute the yaw sensed by the quadrotor (using the XYZ rotation order)
	yaw = atan2( 2*(w*z + x*y),(1 - 2*(y*y + z*z)) );

	//compute quaternion based only on the yaw (using the XYZ rotation order)
	q_yaw = Eigen::Quaternion<double>(cos(yaw/2), 0, 0, sin(yaw/2));

	//convert the control quaternion to the global frame (compensate non-zero yaw)
	q_control = q_yaw * q_control;

	//publish the results
	new_target.orientation.w = q_control.w();
	new_target.orientation.x = q_control.x();
	new_target.orientation.y = q_control.y();
	new_target.orientation.z = q_control.z();
	new_target.thrust = thrust;
	pub.publish(new_target);

}

int main(int argc, char** argv){

	ros::init(argc, argv,"asctec2px4_control");

	ros::NodeHandle nh;

	//setup publisher
	pub=nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",1);

	//setup subscribers
	ros::Subscriber imu_input=nh.subscribe("/mavros/imu/data", 1, imu_callback);
	ros::Subscriber imu_input1=nh.subscribe("/mavros/setpoint_raw/attitude_local", 1, control_callback);

	//loop forever
	ros::spin();

	return 0;
}
