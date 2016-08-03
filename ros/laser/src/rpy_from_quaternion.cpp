#include <ros/ros.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
std_msgs::Float32MultiArray array;
ros::Publisher pub;

void cb(geometry_msgs::PoseStamped data) {
    
      tf::Quaternion q(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w);
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
      ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
      // roll  --> rotate around vertical axis
      // pitch --> rotate around horizontal axis
      // yaw   --> rotate around depth axis
      pub.publish(array);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rpy_quat");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/slam_out_pose", 1, cb);
  pub = nh.advertise<std_msgs::Float32MultiArray>("/rpy", 1);
  ros::spin();
  return 0;

}
