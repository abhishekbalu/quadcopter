#include <ros/ros.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
std_msgs::Float32MultiArray array;
ros::Publisher pub;

void cb(sensor_msgs::Imu data) {
    
      tf::Quaternion q(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
     
      array.data.clear();
      roll = roll  * 180.0 / M_PI;
      pitch = pitch * 180.0 / M_PI;
      yaw = yaw * 180/M_PI;

      array.data.push_back(roll);
      array.data.push_back(pitch);
      array.data.push_back(yaw);
      ROS_INFO("roll, pitch, yaw=%f  %f  %f", roll, pitch, yaw);
      // roll  --> rotate around vertical axis
      // pitch --> rotate around horizontal axis
      // yaw   --> rotate around depth axis
      pub.publish(array);
}

int main(int argc, char **argv) {
  std::string origin_topic = "raw_imu";// mavros/imu/data
  if(argv[0] != NULL)
    origin_topic = argv[0];
  ros::init(argc, argv, "rpy_IMU");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe(origin_topic, 1, cb);
  pub = nh.advertise<std_msgs::Float32MultiArray>("/rpyimu", 1);
  ros::spin();
  return 0;

}
