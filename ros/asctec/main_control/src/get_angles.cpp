#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <main_control/imu.h>
#include <main_control/imu_acc.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher angle_pub;
ros::Publisher angle_pub1;
ros::Publisher angle_pub2;
ros::Publisher angle_pub3;
double r;
double p;
double y;
double accx;
double accy;
double accz;

void asctec_imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
	//extract imu orientation
	double x=imu->orientation.x;
	double y=imu->orientation.y;
	double z=imu->orientation.z;
	double w=imu->orientation.w;
	accx=imu->linear_acceleration.x;
	accy=imu->linear_acceleration.y;
	accz=imu->linear_acceleration.z;

	//convert to angles
	double roll=atan2(2*(w*x+z*y),(1-2*(x*x+y*y))); r=roll;
	double pitch=asin(2*(w*y-x*z)); p=pitch;
	double yaw=atan2(2*(w*z+x*y),(1-2*(y*y+z*z))); y=yaw;

	//publish
	main_control::imu data;
	data.roll=roll;
	data.pitch=pitch;
	data.yaw=yaw;
	angle_pub.publish(data);
}

void asctec_imu_callback1(const geometry_msgs::PoseStamped::ConstPtr& imu1)
{
	//extract imu orientation
	double x=imu1->pose.orientation.x;
	double y=imu1->pose.orientation.y;
	double z=imu1->pose.orientation.z;
	double w=imu1->pose.orientation.w;

	//convert to angles
	double roll=atan2(2*(w*x+z*y),(1-2*(x*x+y*y)));
	double pitch=asin(2*(w*y-x*z));
	double yaw=atan2(2*(w*z+x*y),(1-2*(y*y+z*z)));

	//publish
	main_control::imu data;
	data.roll=roll;
	data.pitch=pitch;
	data.yaw=yaw;
	angle_pub1.publish(data);
}

void macortex_callback(const geometry_msgs::PoseStamped::ConstPtr& macortex)
{
	//extract imu orientation
	double x=macortex->pose.orientation.x;
	double y=macortex->pose.orientation.y;
	double z=macortex->pose.orientation.z;
	double w=macortex->pose.orientation.w;

	//convert to angles
	double roll=atan2(2*(w*x+z*y),(1-2*(x*x+y*y)));
	double pitch=asin(2*(w*y-x*z));
	double yaw=atan2(2*(w*z+x*y),(1-2*(y*y+z*z)));

	//publish
	main_control::imu data;
	data.roll=roll;
	data.pitch=pitch;
	data.yaw=yaw;
	angle_pub2.publish(data);
}

void timer_callback(const ros::TimerEvent& event)
{
	//publish
	main_control::imu_acc data;
	data.roll=r;
	data.pitch=p;
	data.yaw=y;
	data.accx=accx;
	data.accy=accy;
	data.accz=accz;
	angle_pub3.publish(data);
}

int main(int argc, char** argv){

	ros::init(argc, argv,"main_controller");

	ros::NodeHandle nh;

	//setup publisher
	angle_pub=nh.advertise<main_control::imu>("main_control/imu",1);
	angle_pub1=nh.advertise<main_control::imu>("main_control/imu1",1);
	angle_pub2=nh.advertise<main_control::imu>("main_control/macortex",1);
	angle_pub3=nh.advertise<main_control::imu_acc>("main_control/imu_sampled",1);

	//setup subscribers
	ros::Subscriber imu_input=nh.subscribe("fcu/imu", 1, asctec_imu_callback);
	ros::Subscriber imu_input1=nh.subscribe("fcu/current_pose", 1, asctec_imu_callback1);
	ros::Subscriber macortex_input=nh.subscribe("macortex_bridge/mcs_pose", 1, macortex_callback);

	//initialize timers
	ros::Timer timer = nh.createTimer(ros::Duration(0.05), timer_callback);

	//loop forever
	ros::spin();

	return 0;
}

