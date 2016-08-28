#include "ros/ros.h"
#include <stdlib.h>
#include <SDL/SDL.h>
#include "std_msgs/String.h"
#include <sstream>


void controllerCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("controller", 1000, controller);

  ros::spin();

  return 0;
}
