//C++ Includes
#include <iostream>
#include <sstream>
#include <string.h>
#include <math.h>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

//ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <px_comm/OpticalFlow.h>

//Local Includes
#include <quad_msgs/RaB3DInfo.h>
#include <quad_msgs/RaB3DValues.h>
#include <quad_msgs/OpticalFlow.h>
#include <quad_msgs/Estimate.h>
#include <quad_msgs/EstimateSingle.h>
#include <quad_msgs/EstimateMulti.h>

#include "egomotion_2.h"
//Namespaces
using namespace std;
using namespace boost::interprocess;
//Macros
#define PI 3.14159

const int MAX_INIT_TIME = 4;

bool info_initialized = 0; //variable indicating if the previous receiver structure is initialized
vector<int> codes_self; //variable that contains the codes of the emitters that belong to this robot
int self_robot_index; //variable that indicates the entry of the robot estimation list that corresponds to the robot running this node
double node_rate; //variable that defines the sensor output rate
string robot_self_name; //the name of this robot
void* values_msg_shared; //memory shared address for the values message


void initialize_estimator(std::string param_name){
	//auxiliar variables
	double w_th, w_roll, w_pitch, w_change, w_yaw, v_Z, v_OF;
	double lag;
	double mass, calib_slope, calib_bias;

	//get parameters
	ros::NodeHandle nh; // is the same handle as in the main, seems ros is global
	std::string full_name;
	full_name = param_name + "node_rate"; //rate that estimation messages are published by this node
	nh.getParam(full_name.c_str(),node_rate);
	full_name = param_name + "lag"; //with how much lag are the RaB3D measurements received (default is zero)
	nh.getParam(full_name.c_str(),lag);
	full_name = param_name + "w_th"; //thrust egomotion noise (way smaller then neighbors measurement noise) - std
	nh.getParam(full_name.c_str(),w_th);
	full_name = param_name + "w_roll"; //error in roll imu estimations - std
	nh.getParam(full_name.c_str(),w_roll);
	full_name = param_name + "w_pitch"; //error in pitch imu estimations - std
	nh.getParam(full_name.c_str(),w_pitch);
	full_name = param_name + "w_change"; //how compensation factors can change
	nh.getParam(full_name.c_str(),w_change);
	full_name = param_name + "w_yaw"; //angular velocity egomotion noise - std
	nh.getParam(full_name.c_str(),w_yaw);
	full_name = param_name + "v_Z"; //noise of the altimeter - std
	nh.getParam(full_name.c_str(),v_Z);
	full_name = param_name + "v_OF"; //noise of the optical flow - std
	nh.getParam(full_name.c_str(),v_OF);
	full_name = param_name + "mass"; //robot mass
	nh.getParam(full_name.c_str(),mass);

	//select thrust calibration curve according to the selected autopilot
	//px4
	full_name = param_name + "px4_calib_slope";
	nh.getParam(full_name.c_str(),calib_slope);
	full_name = param_name + "px4_calib_bias";
	nh.getParam(full_name.c_str(),calib_bias);
	

	//initialize egomotion estimation variables
	w_th *= w_th; 
	w_yaw *= w_yaw; 
	v_Z *= v_Z; 
	v_OF *= v_OF; //change to variances
	//egomotion_init(1.0/node_rate, 10.0/node_rate, w_th, w_yaw, v_Z, v_OF, lag, mass);
	egomotion_init(1.0/node_rate, 10.0/node_rate, w_th, w_roll, w_pitch, w_change, w_yaw, v_Z, v_OF, lag, mass, calib_slope, calib_bias);

	node_rate += 5; //assume that the rate is high enough, just add a bit more so the loop wont get behind schedule

}

void OpticalFlow_callback_px4_full(const px_comm::OpticalFlow::ConstPtr& optical_flow_msg){
	//update vertical egomtion acording to the measured ground distance
	egomotion_update_Z_self(optical_flow_msg->ground_distance);

	//cout << optical_flow_msg->ground_distance << endl;

	//only consider good optical flow measures

	double vx1 = 0;
        double vy1 = 0;
	if( optical_flow_msg->quality > 100 ){
		//compensate from opticalflow frame to quadrotor frame
		double yaw = -PI/4;
		double vx =-optical_flow_msg->velocity_x;
		double vy =optical_flow_msg->velocity_y;
		vx1 = cos(yaw)*vx - sin(yaw)*vy;
		vy1 = sin(yaw)*vx + cos(yaw)*vy;
	}
		//cout << vx1 << " " << vy1 << endl;

	//update horizontal egomtion acording to the optical flow measurements
	egomotion_update_OF_self(vx1, vy1);
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "RaB3D");
	ros::NodeHandle nh;

	//initialize
	initialize_estimator("rab3D/");

	//publisher where to send position/pose estimations
	ros::Publisher est_multi_pub=nh.advertise<quad_msgs::EstimateMulti>("estimator/estimate_neighbors",1);
	ros::Publisher est_self_pub=nh.advertise<quad_msgs::EstimateSingle>("estimator/estimate_self",1); //where to send egomotion estimations

	//egomotion subscribers

	ros::Subscriber angle_input = nh.subscribe("mavros/imu/data", 1, angle_callback);
	ros::Subscriber control_input = nh.subscribe("mavros/setpoint_raw/attitude",1,control_callback_px4);
	ros::Subscriber OpticalFlow_sub = nh.subscribe("/px4flow/opt_flow", 1, OpticalFlow_callback_px4_full);
	
	

	//initialize loop rate
	ros::Rate loop_rate(node_rate);

	//run algorithm
	while(ros::ok()){ //stop if Ctrl-C is pressed

		//variables to be used to form the estimated pose messages to send
		quad_msgs::Estimate sl;
		

		//predict first the egomotion of the current robot
		egomotion_predict_self();

		//publish egomotion state
		est_self_pub.publish(*egomotion_get_ros_message());


		//do a single ros loop (for the updates) - the update is done after the current predict
		ros::spinOnce();

		//sleep untill end of the prediction periode
		loop_rate.sleep();

	}

}

//debug egomotion -> for now the problem might be the lag
// switch the angle control ON or OFF to see if it makes a difference

//introduce camera and IR sensor noise to emulate problems that happen in reality
//    I should add distortion to the image (understand if I can include the unified camera model)
//    In IR I should emulate IR reflection problems (to see how the quadrotors react)
