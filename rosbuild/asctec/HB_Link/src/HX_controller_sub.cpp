#include "ros/ros.h"
#include "math.h"
#include <cmath>
#include <string.h>
#include <stdlib.h>
#include <SDL/SDL.h>
#include "std_msgs/String.h"
#include <sstream>

#define PI = 3.1415;

std::string controller;

void controllerCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  controller=msg->data.c_str();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
 
  ros::Publisher pub = n.advertise<std_msgs::String>("command",10);
  ros::Subscriber sub = n.subscribe("controller", 10, controllerCallback);
  ros::Rate loop_rate(25);
  float thrust=0;
  float roll=0;
  float pitch=0;
  float yaw=0; 

  while(ros::ok()){
  std_msgs::String msg;
  std::stringstream ss;
  msg.data=ss.str();
  unsigned int i;
  std::string t;
  std::stringstream st;
  //int T=0;
  if(!controller.empty()){
  	if(controller.at(0)=='b'){
		switch(controller.at(3)){
			case('0'):
				ss << "motor 1";
  				msg.data=ss.str();
  				break;

			case('1'):
				ss<< "motor 0";
				msg.data=ss.str();
				break;			
			default: ss<<"motor 0";
				 msg.data=ss.str();
				
		}
	pub.publish(msg);

	} 	
	else{ int T0,T1,T2,T3;
		switch(controller.at(3)){
			case('0'):
				t="";
				for(i=5;i<controller.length();i++){
					t=t+controller.at(i);
				}
		       		
				st << t;
				st >> T3;
				
				yaw=(float) T3/32768;

				if(fabs(yaw)<0.1)
					yaw=0;

				break;


			case('2'):
				t="";
				for(i=5;i<controller.length();i++){
					t=t+controller.at(i);
				}
		       		
				st << t;
				st >> T0;
				
				thrust=(float) (T0+32768)/65536;
					
				if(fabs(thrust)<0.1)
					thrust=0;
				break;

			case('3'):
				t="";
				for(i=5;i<controller.length();i++){
					t=t+controller.at(i);
				}
		       		
				st << t;
				st >> T1;
				
				roll=(float) T1/32768;

				if(fabs(roll)<0.1)
					roll=0;

				break;
	
			case('4'):
				t="";
				int T2=0;
				for(i=5;i<controller.length();i++){
					t=t+controller.at(i);
				}
		       		
				st << t;
				st >> T2;
								
				pitch=(float) T2/32768;
				
				if(fabs(pitch)<0.1)
					pitch=0;
				
				break;
		}
	
		

	ss << "ctrl " << thrust <<" " << roll << " " << pitch << " " << yaw;
        msg.data=ss.str();
	pub.publish(msg);
	}
  }
  ros::spinOnce();
  loop_rate.sleep();
  }
  
  ros::spinOnce();  
return 0;
}
