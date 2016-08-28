//C++ Libraries
#include <list>
#include <math.h>
#include <sstream>
#include <typeinfo>
#include <unistd.h>
#include <string>
//ROS Libraries
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <sensor_msgs/Imu.h>
//User Libraries
#include <height/pid.h>
#include "height/state.h"
#include "height/debug.h" //Comment or uncomment this for verbose
#include "height/full_pose.h"
#include <sensor_msgs/Imu.h>

#define NUM_OFFSET_VALUES 5 //These first values from the sonar are averaged
//to extract their mean which is used as an offset
#define RATE 10


float z = 0.0; //Height
int num = 0;
int count = 0;
float offset;
int toggle = -1; //This is the initial value -- it's just stopped, neither taking off nor landing
std::list<float> first_sonar_readings;
geometry_msgs::Twist msg;
PID pid(0.3, 0.0, 3.0, 0.0, 0.0, 200.0, -200.0); //0.25, 0.0, 4.0 works 
float tolerance = 0.01;
float value=0.0;

float x=0.0;
float y=0.0;
float x_value=0.0;
float y_value=0.0;
PID xpid(0.37, 0.001, 3.7, 0.0, 0.0, 200.0, -200.0); //0.25, 0.0, 4.0 works 
PID ypid(0.37, 0.001, 4.0, 0.0, 0.0, 200.0, -200.0); //0.25, 0.0, 4.0 works 

//default setPoint values
float z_setPoint = 0.4;
float x_setPoint = 1.0;
float y_setPoint = 1.0;

height::full_pose pose;

void getHeight(const sensor_msgs::Range::ConstPtr& data){
	
	if(num <= NUM_OFFSET_VALUES){ //Append to the list
		first_sonar_readings.push_back(data->range);
		if(first_sonar_readings.size() == NUM_OFFSET_VALUES){ 
			float sum = 0;
			//Average to find the offset
			for(std::list<float>::iterator it= first_sonar_readings.begin(); it != first_sonar_readings.end(); ++it){
				sum = sum + *it;
			}
			offset = sum/(first_sonar_readings.size());
		}
		num++;
	}
	
	#ifdef VERBOSE
		ROS_INFO("Offset Value:  [%f] Heigth from the Quad CoM: [%f]", offset, z);
	#endif
	
	z = data->range - offset;
	pose.z = z;
	
}
void getOptFlow(const height::state::ConstPtr& data)
{
	x=data->x;
	y=data->y;

	pose.x = x;
	pose.y = y;
	pose.quality = data->quality;
	pose.vx = data->vx;
	pose.vy = data->vy;
	pose.attitude = data->attitude;
}
void getVelocity(const geometry_msgs::Twist::ConstPtr& data){
	//Copy the data to a local variable
	msg.linear = data->linear;
	msg.angular = data->angular;
	if(x_value < x_setPoint+0.1 && x_value >x_setPoint-0.1){
		msg.linear.x = 0.0;
	}
	if(y_value < y_setPoint+0.1 && y_value >y_setPoint-0.1){
		msg.linear.y = 0.0;
	}
}

void getCommand(const std_msgs::String::ConstPtr& data){
	#ifdef VERBOSE
		ROS_INFO("Received command [%s]", data->data.c_str());
	#endif
	if (!strcmp(data->data.c_str(),"takeoff")){
		pid.setPoint(z_setPoint); //initialize it by going up
		xpid.setPoint(x_setPoint);
		ypid.setPoint(y_setPoint);
		toggle = 1;
	}else if(!strcmp(data->data.c_str(),"land")){
		pid.setPoint(0.0); //The real quad could never reach this
		toggle = 0;
	}
}

void getImu(const sensor_msgs::Imu::ConstPtr& data){
	pose.ang_vel = data->angular_velocity;
	pose.lin_acc = data->linear_acceleration;
}

int main(int argc, char **argv){

	if(argc==4){
		z_setPoint = atof(argv[1]);
		x_setPoint = atof(argv[2]);
		y_setPoint = atof(argv[3]);
	}


	ROS_INFO("Started controller...\n");
	ros::init(argc, argv, "controller");
	ros::NodeHandle n;


	sleep(1.5); //wait for the quad to start

	//Subscribe to IMU
	

	ros::Subscriber sonar = n.subscribe("/z_pose", 10, getHeight); //May use the simulation sonar here
	ros::Subscriber optFlow = n.subscribe("xy_pose", 10, getOptFlow);
	ros::Subscriber velocity = n.subscribe("/cmd_vel", 10, getVelocity);
	ros::Subscriber syscommand = n.subscribe("/syscommand", 10, getCommand);
	ros::Subscriber imu = n.subscribe("/raw_imu", 10, getImu);
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	
	ros::Publisher full_pose_pub = n.advertise<height::full_pose>("/full_pose", 10);

	ros::Rate loop_rate(RATE);
	while(ros::ok()){
		if(toggle == 1){
			if((z != 0.0) && (count < 10)){ //If height isn't 0 and we haven't had 10 tolerance errors
				value = pid.update(z);
				x_value = xpid.update(x);
				y_value = ypid.update(y);
				if(value <= tolerance){
					count++;
					value=0.0;
					x_value = 0.0;
					y_value = 0.0;
				}else{
					count=0;
				}
				if(value < -1.0)
					value = -1.0;
				if(value > 1.0)
					value = 1.0;
				#ifdef VERBOSE
					ROS_INFO("PID ZValue [%f]", value);
					ROS_INFO("PID XValue [%f]", x_value);
					ROS_INFO("PID YValue [%f]", y_value);
				#endif
				msg.linear.z = value;
				msg.linear.x = x_value;
				msg.linear.y = y_value;
			}else{
				msg.linear.x = 0;
				msg.linear.y = 0;
			}
			
		}else if(toggle == 0){
			value = pid.update(z);
			msg.linear.z = value;
			msg.linear.x = 0;
			msg.linear.y = 0;
		}
		pub.publish(msg);
		full_pose_pub.publish(pose);
		loop_rate.sleep();
		ros::spinOnce();

	}

	ros::spin();
	return 0;

}