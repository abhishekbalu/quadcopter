//C++ Libraries
#include <iostream>
#include <math.h>
//ROS Libraries
#include <px_comm/OpticalFlow.h>
#include "ros/ros.h"

const int RATE = 1;
const int TEST_TIME = 100;
ros::Time begin;
int quality_sum = 0;

int last_quality = -1;
int outliers=1;
double koppa;
double quality_avg=0;

double max_height = -1;
double min_height = 1000;
void getOptFlow(const px_comm::OpticalFlow::ConstPtr& data){

  if(data->ground_distance < min_height)
    min_height = data->ground_distance;
  if(data->ground_distance > max_height)
    max_height = data->ground_distance;
  if(abs(data->quality-last_quality)>120 && last_quality!=-1)
    outliers++;
  last_quality = data->quality;
  quality_sum+=data->quality; 
  double seconds_elapsed = ros::Time::now().toSec() - begin.toSec();
  quality_avg = (double)quality_sum/seconds_elapsed;
  koppa = quality_avg/outliers;
  printf("%f\n", data->ground_distance);
  printf("SECONDS_ELAPSED: %f Q: %d Q_SUM: %d Q/sec: %f OUTLIERS: %d KOPPA: %f, @ Max_H: %f Min_H: %f\n",  seconds_elapsed, data->quality, quality_sum, quality_avg, outliers, koppa, max_height, min_height);
  if((int)seconds_elapsed == TEST_TIME)
    exit(0);
}

int main(int argc, char** argv){
  ROS_INFO("Started floor_checker...\n");
  ros::init(argc, argv, "floor_checker");
  ros::NodeHandle n, nh("~");
  begin = ros::Time::now();
  //Subscribers
  ros::Subscriber opt = n.subscribe("/px4flow/opt_flow", 1, getOptFlow);
  
  ros::Rate loop_rate(RATE);

  while(ros::ok()){




    loop_rate.sleep();
    ros::spinOnce();
  }
  ros::spin();
  return 0;

}