/* === System includes === */
#include <iostream>
#include <string.h>
#include <math.h>

/* === ROS includes === */
#include <ros/ros.h>
#include <quad_msgs/RaB3DInfo.h>
#include <quad_msgs/RaB3DValues.h>
#include <quad_msgs/Estimate.h>
#include <quad_msgs/EstimateMulti.h>

/* === Local includes === */
#include "RaB3D.h"
#include "estimator_emit.h"

using namespace std;

rcv_data* prcv = NULL; //structure that contains all receiver calibration and values data
emt_list* pemitlist = NULL; //structure containing all the estimated emitters
bool info_initialized = 0; //variable indicating if the previous receiver structure is initialized
vector<int> codes_self; //variable that contains the codes of the emitters that belong to this robot
double node_rate; 

void initialize_estimator(std::string param_name)
{
	//get parameters
	ros::NodeHandle nh; // is the same handle as in the main, seems ros is global
	std::string full_name;
	full_name = param_name + "codes_self";
	nh.getParam(full_name.c_str(),codes_self);
	full_name = param_name + "node_rate";
	nh.getParam(full_name.c_str(),node_rate);
}

void RaB3DInfo_callback(const quad_msgs::RaB3DInfo::ConstPtr& info_msg)
{
	//initialize receiver calibration data
	prcv = RaB3D_initialize_from_message(info_msg);

	//initialize estimator list with at most MAX_EMITTERS emitter estimations (define value in RaB3D.h)
	pemitlist = estimator_init(MAX_EMITTERS, 1.0/node_rate);

	//this node is initialized - it can now run normally
	info_initialized = 1;
}

void RaB3DValues_callback(const quad_msgs::RaB3DValues::ConstPtr& values_msg)
{
	//update receiver values and status variables on the receiver structure
	RaB3D_set_values_from_message(prcv, values_msg);

	//go through the emitters detected in the values_msg
	for(int k = 0; k < prcv->emitn; k++){

		//robot emitters are not considered for estimation (their relative positions are already known)
		int code_matched = 0;
		for(int l = 0; l < codes_self.size(); l++){
			if(prcv->code[k] == codes_self[l])
				{code_matched = 1; break;}
		}
		if(code_matched)
			continue;

		//try to find the emitter on the estimation list
		int slot = estimator_find_emitter(pemitlist, prcv->code[k]);
		if(slot >= 0){ //if the emitter is on the list perform predictions and updates (they are sequencial)

			//update
			estimator_update_single_marker(pemitlist, slot, prcv);

		}
		else {//add the emitter if it is not on the list
			slot = estimator_add_emitter(pemitlist, prcv->code[k]);
			if(slot < 0)
				cout << "WARNING: emitter list full" << endl;
			else
				cout << "Added emitter " << prcv->code[k] <<" in slot " << slot << endl;
		}

	}
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
	while(ros::ok()){ //stop if Ctrl-C is pressed
		ros::spinOnce(); //do a single ros loop
		if(info_initialized) break;
	}

	//shutdown RaB3DInfo topic
	RaB3DInfo_sub.shutdown();

	//subscribe to RaB3DValues topic
	ros::Subscriber RaB3DValues_sub=nh.subscribe("sensor/RaB3D/values", 1, RaB3DValues_callback);

	//declare publisher where to send position/pose estimations
	ros::Publisher est_multi_pub=nh.advertise<quad_msgs::EstimateMulti>("estimator/estimate_neighbors",1);

	//initialize loop rate
	ros::Rate loop_rate(node_rate);

	//run algorithm
	while(ros::ok()){ //stop if Ctrl-C is pressed

		//variables to be used to form the estimated position/pose messages to send
		quad_msgs::Estimate sl;
		quad_msgs::EstimateMulti s_multi;

		//go through the emitter list for predictions and message sending
		for(int k=0, kmax = pemitlist->size; k<kmax; k++)
		{
			//if the emitter slot is being tracked
			if(pemitlist->pemit[k].initialized > 0){

				//prediction (ego-motion compensation)
				estimator_predict_single_marker(pemitlist, k);

				//fill estimate message with emitter/robot position/pose estimates
				//x1 is the estimate variable that contains the position/pose and velocity of the tracked emitter/robot
				ostringstream convert; //generate emitter name to publish in the message
				convert << pemitlist->pemit[k].code;
				sl.name.data = "emitter" + convert.str(); //emitter name (with its code attached)
				sl.position.x=pemitlist->pemit[k].x1(0); sl.position.y=pemitlist->pemit[k].x1(1); sl.position.z=pemitlist->pemit[k].x1(2); //position
				sl.velocity.x=pemitlist->pemit[k].x1(3); sl.velocity.y=pemitlist->pemit[k].x1(4); sl.velocity.z=pemitlist->pemit[k].x1(5); //velocity
				//yaw should go in the orientation component of the estimate
				sl.covariance.clear(); //clear the vector first
				for(int l1 = 0; l1 < 6; l1++) //covariance (for now of position and velocity)
					for(int l2 = 0; l2 < 6; l2++)
						//P1 is the estimate covaraince for the position/pose and velocity of the tracked emitter/robot
						sl.covariance.push_back(pemitlist->pemit[k].P1(l1,l2));
				sl.updated = pemitlist->pemit[k].updated; //tells if estimation was updated since the last message
				sl.sensors.data = "RaB3D:"; //and by which sensors (more sensors can update this estimate, and this string can grow)
				
				//insert estimate on the estimation list to publish
				s_multi.estimates.push_back(sl);
				
				//reset estimation update status
				pemitlist->pemit[k].updated = 0;

			}

		}

		//the tracked relative positions/poses are always in the robots reference frame
		s_multi.relative_on=1;

		//insert header with time stamp
		s_multi.header.stamp = ros::Time::now();
		
		//publish tracked emitters/robots
		est_multi_pub.publish(s_multi);

		//do a single ros loop (for the updates) - the update is done after the current predict
		ros::spinOnce();

		//sleep untill end of the prediction periode
		loop_rate.sleep();

	}

}

//constant predicts use less memory, it gives a constant output rate, but makes the update unsynchronized with prediction, and the code more confusing
//predict data can come from IMU/control; update data only from RaB3D (local velocity estimation might be required for egomotion compensation)


//create a function for estimating the quadrotor (should be only one with adjustments in the Jacobian)
//move the quadrotor in the same trajectories and try to detect it also (maybe the yaw should also change)
//finally perform the formation control algorithms


//I need to convert RaB3D in webots on two files (try to merge the two libraries)
//I still have the noise to introduce in the camera sensor and the IR sensor to emulate the problems that I have in reality
