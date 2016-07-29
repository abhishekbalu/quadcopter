/* === System includes === */
#include <iostream>
#include <sstream>
#include <string.h>
#include <math.h>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

/* === ROS includes === */
#include <ros/ros.h>
#include <quad_msgs/RaB3DInfo.h>
#include <quad_msgs/RaB3DValues.h>
#include <quad_msgs/OpticalFlow.h>
#include <quad_msgs/Estimate.h>
#include <quad_msgs/EstimateSingle.h>
#include <quad_msgs/EstimateMulti.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <px_comm/OpticalFlow.h>

/* === Local includes === */
#include "RaB3D.h"
#include "estimator_robot.h"
#include "egomotion_2.h"

using namespace std;
using namespace boost::interprocess;

rcv_data* prcv = NULL; //structure that contains all receiver calibration and values data
robot_list* probotlist = NULL; //structure containing all the estimated robots
bool info_initialized = 0; //variable indicating if the previous receiver structure is initialized
vector<int> codes_self; //variable that contains the codes of the emitters that belong to this robot
int self_robot_index; //variable that indicates the entry of the robot estimation list that corresponds to the robot running this node
double node_rate; //variable that defines the sensor output rate
string robot_self_name; //the name of this robot
void* values_msg_shared; //memory shared address for the values message
int asctec; //decides the platform to which send commands to

void initialize_estimator(std::string param_name)
{
	//auxiliar variables
	vector<int> robot_codes, robot_codes_single, robot_codes_n;
	vector<double> marker_pos, marker_pos_single;
	vector<string> robot_names_separated;
	string robot_names, robot_self;
	double w_th, w_roll, w_pitch, w_change, w_yaw, v_Z, v_OF;
	double lag;
	double mass, calib_slope, calib_bias;

	//get parameters
	ros::NodeHandle nh; // is the same handle as in the main, seems ros is global
	std::string full_name;
	full_name = param_name + "node_rate"; //rate that estimation messages are published by this node
	nh.getParam(full_name.c_str(),node_rate);
	full_name = param_name + "codes_self"; //codes that belong to the this robot
	nh.getParam(full_name.c_str(),codes_self);
	full_name = param_name + "robot_names"; //robot names
	nh.getParam(full_name.c_str(),robot_names);
	full_name = param_name + "this_quad"; //self robot name
	nh.getParam(full_name.c_str(),robot_self);
	full_name = param_name + "robot_codes_n"; //number of codes that each robot has
	nh.getParam(full_name.c_str(),robot_codes_n);
	full_name = param_name + "robot_codes"; //all the codes for all the robots
	nh.getParam(full_name.c_str(),robot_codes);
	full_name = param_name + "marker_pos"; //all the marker positions with respect to the robot frame
	nh.getParam(full_name.c_str(),marker_pos);
	full_name = param_name + "marker_pos"; //all the marker positions with respect to the robot frame
	nh.getParam(full_name.c_str(),marker_pos);
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
	full_name = param_name + "asctec"; //selected auto-pilot
	nh.getParam(full_name.c_str(),asctec);

	//select thrust calibration curve according to the selected autopilot
	if(asctec == 0){ //px4
		full_name = param_name + "px4_calib_slope";
		nh.getParam(full_name.c_str(),calib_slope);
		full_name = param_name + "px4_calib_bias";
		nh.getParam(full_name.c_str(),calib_bias);
	}
	else { //asctec
		full_name = param_name + "asctec_calib_slope";
		nh.getParam(full_name.c_str(),calib_slope);
		full_name = param_name + "asctec_calib_bias";
		nh.getParam(full_name.c_str(),calib_bias);
	}

	//initialize egomotion estimation variables
	w_th *= w_th; w_yaw *= w_yaw; v_Z *= v_Z; v_OF *= v_OF; //change to variances
	//egomotion_init(1.0/node_rate, 10.0/node_rate, w_th, w_yaw, v_Z, v_OF, lag, mass);
	egomotion_init(1.0/node_rate, 10.0/node_rate, w_th, w_roll, w_pitch, w_change, w_yaw, v_Z, v_OF, lag, mass, calib_slope, calib_bias);

	//save the name of this robot
	robot_self_name = robot_self;

	//initialize estimator list with at most MAX_ROBOTS robot estimation entries (define value in RaB3D.h)
	probotlist = estimator_init(MAX_ROBOTS, 1.0/node_rate);
	node_rate += 5; //assume that the rate is high enough, just add a bit more so the loop wont get behind schedule

	//seperating the string with the robot names into individual robot names
	string single_name;
	stringstream ss; ss << robot_names;
	while(ss >> single_name)
		robot_names_separated.push_back(single_name);

	//find if any of the robots is the one doing these computations
	self_robot_index = -1;
	for(int k = 0; k < robot_names_separated.size(); k++)
		if(!strcmp(robot_names_separated[k].c_str(), robot_self.c_str())) //case the names are the same
			{self_robot_index = k; break;} //robot names are assumed to be unique

	//go through the name list, get the codes that are associated to it, and initialize a robot entry in the estimation list
	for(int k = 0; k < robot_names_separated.size(); k++){

		//fill the code vector for this robot (NOTE: robot_codes_n is of the same size as robot_names_separated)
		robot_codes_single.clear(); //start with zero codes
		for(int l = 0; l < robot_codes_n[k]; l++) //the codes for a single robot are consecutive
			robot_codes_single.push_back(robot_codes[l]);
		robot_codes.erase (robot_codes.begin(),robot_codes.begin()+robot_codes_n[k]); //eliminate the current robot codes, so we can always read from the first element

		//register the robot in the estimation list
		int slot = estimator_add_robot(probotlist, robot_names_separated[k], robot_codes_single, marker_pos);

		//if this is the self robot, save this slot
		if(k == self_robot_index) self_robot_index = slot; //self robot bumber becomes the list entry index

	}

}

void RaB3DInfo_callback(const quad_msgs::RaB3DInfo::ConstPtr& info_msg)
{
	//initialize receiver calibration data
	prcv = RaB3D_initialize_from_message(info_msg);

	//this node is initialized - it can now run normally
	info_initialized = 1;
}

void OpticalFlow_callback(const quad_msgs::OpticalFlow::ConstPtr& optical_flow_msg)
{
	//update vertical egomtion acording to the measured ground distance
	egomotion_update_Z_self(optical_flow_msg->ground_distance);

	//get yaw and compensate x, y rotation (from velocity in the world to velocity in the quadrotor frame)
	double yaw = get_self_yaw();
	double vx =optical_flow_msg->velocity_x;
	double vy =optical_flow_msg->velocity_y;
	double vx1 = cos(yaw)*vx + sin(yaw)*vy;
	double vy1 = -sin(yaw)*vx + cos(yaw)*vy;

	//update horizontal egomtion acording to the optical flow measurements
	egomotion_update_OF_self(vx1, vy1);
}
void OpticalFlow_callback_px4(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& optical_flow_msg)
{
	//get yaw and compensate x, y rotation
	//double yaw = get_self_yaw();
	//double vx = optical_flow_msg->linear.x;
	//double vy = optical_flow_msg->linear.y;
	double vx1 = -optical_flow_msg->twist.twist.linear.x; //cos(yaw)*vx - sin(yaw)*vy;
	double vy1 = -optical_flow_msg->twist.twist.linear.y;//sin(yaw)*vx + cos(yaw)*vy;

	//update horizontal egomtion acording to the optical flow measurements
	egomotion_update_OF_self(vx1, vy1);
}
void Z_callback_px4(const sensor_msgs::Range::ConstPtr& range_msg)
{
	//update vertical egomtion acording to the measured ground distance
	egomotion_update_Z_self(range_msg->range);
}
void OpticalFlow_callback_px4_full(const px_comm::OpticalFlow::ConstPtr& optical_flow_msg)
{
	//update vertical egomtion acording to the measured ground distance
	egomotion_update_Z_self(optical_flow_msg->ground_distance);

	//cout << optical_flow_msg->ground_distance << endl;

	//only consider good optical flow measures

	double vx1 = 0;
        double vy1 = 0;
	if( optical_flow_msg->quality > 100 ){
		//compensate from optical frow frame to quadrotor frame
		double yaw = -3.14159/4;
		double vx =-optical_flow_msg->velocity_x;
		double vy =optical_flow_msg->velocity_y;
		vx1 = cos(yaw)*vx - sin(yaw)*vy;
		vy1 = sin(yaw)*vx + cos(yaw)*vy;
	}
		//cout << vx1 << " " << vy1 << endl;

	//update horizontal egomtion acording to the optical flow measurements
	egomotion_update_OF_self(vx1, vy1);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "RaB3D");
	ros::NodeHandle nh;

	//initialize
	initialize_estimator("rab3D/");

	//subscribe to RaB3DInfo topic
	ros::Subscriber RaB3DInfo_sub=nh.subscribe("sensor/RaB3D/info", 1, RaB3DInfo_callback);

	//initializes the algorithms by first reading a RaB3DInfo message (the program does not start before this)
	double local_time = ros::Time::now().toSec();
	while(ros::ok()){ //stop if Ctrl-C is pressed
		ros::spinOnce(); //do a single ros loop
		if(info_initialized) break;
		if( (ros::Time::now().toSec() - local_time) > 4) break; //break also if it took to mutch time to initialize the sensor
	}

	//shutdown RaB3DInfo topic
	RaB3DInfo_sub.shutdown();

	//there is no problem if RaB3DInfo is not initialized (the right flags will prevent anything from happening
	if(info_initialized == 0)
		cout << "RaB3D is not initialized." << endl;

	//initialize shared memory to receive data from IR receivers (emulating the hardware data reception)
	shared_memory_object shm;
	mapped_region region;
	
	if(info_initialized){
		int memory_error = 0;
		try {
			shm = shared_memory_object(open_only, robot_self_name.c_str(), read_only); //getting region
		}
		catch(interprocess_exception &ex){
			cout << "Shared memory problem:" << endl;
			cout << ex.what() << endl;
			memory_error = 1;
		} 
		if(memory_error) //if shared memory could not be found abort
			return 0;
		region = mapped_region(shm, read_only); //map region
		values_msg_shared = region.get_address(); //get shared memory address
		cout << "Shared memory process over for " << robot_self_name.c_str() << "." << endl;
	}

	//publisher where to send position/pose estimations
	ros::Publisher est_multi_pub=nh.advertise<quad_msgs::EstimateMulti>("estimator/estimate_neighbors",1);
	ros::Publisher est_self_pub=nh.advertise<quad_msgs::EstimateSingle>("estimator/estimate_self",1); //where to send egomotion estimations

	//egomotion subscribers
	ros::Subscriber angle_input;
	if(asctec)
		angle_input=nh.subscribe("fcu/imu", 1, angle_callback);
	else
		angle_input=nh.subscribe("mavros/imu/data", 1, angle_callback);

	ros::Subscriber control_input;
	if(asctec)
		control_input=nh.subscribe("fcu/control",1,control_callback);
	else
		control_input=nh.subscribe("mavros/setpoint_raw/attitude",1,control_callback_px4);

	ros::Subscriber OpticalFlow_sub;
	ros::Subscriber Z_sub;
	if(asctec)
		OpticalFlow_sub=nh.subscribe("sensor/optical_flow", 1, OpticalFlow_callback);
	else {
		
		OpticalFlow_sub=nh.subscribe("/px4flow/opt_flow", 1, OpticalFlow_callback_px4_full);
		//OpticalFlow_sub=nh.subscribe("visual_odom", 1, OpticalFlow_callback_px4);
		//Z_sub=nh.subscribe("mavros/px4flow/ground_distance", 1, Z_callback_px4);
	}

	//initialize loop rate
	ros::Rate loop_rate(node_rate);

	//run algorithm
	while(ros::ok()){ //stop if Ctrl-C is pressed

		//variables to be used to form the estimated pose messages to send
		quad_msgs::Estimate sl;
		quad_msgs::EstimateMulti s_multi;

		//predict first the egomotion of the current robot
		egomotion_predict_self();

		//go through the robot list for predictions and message sending
		for(int k=0, kmax = probotlist->size; k<kmax; k++)
		{

			//if the emitter slot is being tracked
			if(probotlist->probot[k].initialized > 0){

				//prediction (ego-motion compensation)
				estimator_predict_robot(probotlist, k);

				//fill estimate message with robot pose estimates
				//x1 is the estimate variable that contains the pose and velocity of the tracked robot
				sl.name.data = probotlist->probot[k].name; //name of the robot
				sl.position.x=probotlist->probot[k].x1(0); sl.position.y=probotlist->probot[k].x1(1); sl.position.z=probotlist->probot[k].x1(2); //position
				sl.velocity.x=probotlist->probot[k].x1(4); sl.velocity.y=probotlist->probot[k].x1(5); sl.velocity.z=probotlist->probot[k].x1(6); //velocity
				Matrix3f Rot; //obtain yaw rotation matrix
				double yaw = probotlist->probot[k].x1(3);
				double cy = cos(yaw), sy = sin(yaw);
				Rot << cy,-sy,0,sy,cy,0,0,0,1;
				Quaternionf quatFromRot(Rot); //get quaternion from rotation matrix
				sl.orientation.x = quatFromRot.x();
				sl.orientation.y = quatFromRot.y();
				sl.orientation.z = quatFromRot.z();
				sl.orientation.w = quatFromRot.w();
				sl.covariance.clear(); //clear the vector first
				for(int l1 = 0; l1 < 7; l1++) //covariance (position, yaw, and velocity)
					for(int l2 = 0; l2 < 7; l2++)
						//P1 is the estimate covaraince for the pose and velocity of the tracked robot
						sl.covariance.push_back(probotlist->probot[k].P1(l1,l2));
				sl.updated = probotlist->probot[k].updated; //tells if estimation was updated since the last message
				if(sl.updated)
					sl.sensors.data = "RaB3D:"; //and by which sensors (more sensors can update this estimate, and this string can grow)
				else
					sl.sensors.data = "";
				
				//insert estimate on the estimation list to publish
				s_multi.estimates.push_back(sl);
				
				//reset estimation update status
				probotlist->probot[k].updated = 0;

			}

		}

		//publish egomotion state
		est_self_pub.publish(*egomotion_get_ros_message());

		//the tracked relative poses are always in the robots reference frame
		s_multi.relative_on=1;

		//insert header with time stamp
		s_multi.header.stamp = ros::Time::now();
		
		//publish tracked robots
		est_multi_pub.publish(s_multi);

		//update if possible
		if(info_initialized){
			if(RaB3D_set_values_from_shared_memory(prcv, values_msg_shared) == 1){
				for(int k = 0; k < probotlist->size; k++)
					if(k != self_robot_index) {//self estimations are not updated using this algorithm
						estimator_update_robot(probotlist, k, prcv);
					}
			}
		}

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
