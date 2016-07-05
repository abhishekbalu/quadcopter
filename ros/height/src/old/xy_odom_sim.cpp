#include <list>
#include "ros/ros.h"
#include <math.h>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <sstream>
#include <typeinfo>


#include <geometry_msgs/Vector3Stamped.h>
#include <px_comm/OpticalFlow.h>

#include "quad_can_driver/Attitude.h"
#include <geometry_msgs/TwistStamped.h>

double K;
quad_can_driver::Attitude attitude;
px_comm::OpticalFlow *OF = NULL;
//Variables for the filtering
ros::Time last_time;
ros::Time current_time;
double last_velocity_x;
double last_velocity_y;
double current_velocity_x;
double current_velocity_y;
double theta; //Pitch
double phi;//Roll
//Constants
double Ts = 0.01;
double tau = 0.2;
double alpha = Ts/tau;
double focal_length;
double fov;

double computeFOV(){
    double d = 30*60*pow(10,-6);
    double aux = 2*atan2(d, 2*focal_length*pow(10,-3));
    float field_of_view = aux * 1000; 
    ROS_INFO("Field of View: [%f]", field_of_view);
    return aux;
}


void getOdom(const px_comm::OpticalFlow::ConstPtr& data){
    if(OF == NULL){
        //initialize OF
        OF = new px_comm::OpticalFlow;
        last_velocity_x = data->velocity_x;
        last_velocity_y = data->velocity_y;
        current_velocity_x = last_velocity_y;
        current_velocity_y = last_velocity_y;

    }else{
        last_velocity_x = current_velocity_x;
        last_velocity_y = current_velocity_y;

        //Simple Low pass digital with an RC constant of alpha filter
        current_velocity_x = last_velocity_x + alpha * (data->velocity_x -last_velocity_x);
        current_velocity_y = last_velocity_y + alpha * (data->velocity_y - last_velocity_y);




        OF->header.stamp = ros::Time::now();
        OF->velocity_x = current_velocity_x;
        OF->velocity_y = current_velocity_y;
        OF->quality = data->quality;
        theta = K * OF->velocity_x;
        phi = K * OF->velocity_y;
        attitude.pitch = theta;
        attitude.roll = phi;
        fov = computeFOV();
        float delta = abs(data->velocity_x - OF->velocity_x);
        ROS_INFO("Delta to received [%f]", delta);
        ROS_INFO("velocity_x: [%f] , velocity_y: [%f]", OF->velocity_x, OF->velocity_y);
        ROS_INFO("Phi(Roll): [%f] , Theta(Pitch): [%f]", phi, theta);

    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "quad_position");
    ros::NodeHandle n, nh("~");

    //Read Parameters
    nh.param("P_gain", K, 0.16);
    nh.param("Focal_length", focal_length, 12.0); //in mm
    //Subscribing
    //ros::Subscriber sub1 = n.subscribe("/quad_OF/O_Flow", 10, OFCallback);
    ros::Subscriber sub2 = n.subscribe("/simOptFlow", 1, getOdom);

    //Publishing
    ros::Publisher attitude_pub = n.advertise<quad_can_driver::Attitude>("AttitudeCtrl", 1);
    ros::Publisher of_pub = n.advertise<px_comm::OpticalFlow>("/opt_flow_filtered", 1);
    ros::Rate loop_rate(50);

    while(ros::ok()){
        attitude_pub.publish(attitude);
        of_pub.publish(*OF);
        
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
