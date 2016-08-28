#ifndef ESTIMATOR_ROBOT
#define ESTIMATOR_ROBOT

/* === System includes === */
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>

/* === Local includes === */
#include "RaB3D.h"
#include "egomotion_2.h"

using namespace Eigen;
using namespace std;

//structures where robot estimations will be saved
//(a)the robot list should be initialized according to a list of robot names with n associated codes each
//(b)the emitters can be placed at any position with respect to the robot frame, but only consider the two emitters with strongest intensity
//(c)only the robot relative (x,y,z,yaw) is estimated
//data for each slot
typedef struct t_robot_data{
	string  name; //name of the quadrotor
	vector<int> codes; //vector with all the codes associated to this robot
	vector<double> x_markers; //vector with all the marker positions with respect to the robot frame
	int initialized; //if initialized = -1 the slot is available; if = 0 then the slot is filled but in an un-initialized mode; if > 0 then the slot is filled and in an initialized mode
	int updated; //if there is any measurement that updated this estimate between the time of two consecutive ros message, then there is new information to be sent on the next message
	VectorXd x, x1; //estimated robot 3D pose (it is only valid when initialized > 0)
	MatrixXd P, P1; //estimation covariance (it is only valid when initialized > 0)
	egomotion* ego; //a list with lag parameters
} robot_data;

//the entire list manager
typedef struct t_robot_list{
	int size; //the amount of slots that the list contains
	robot_data* probot; //where the emitter slots will be stored
	double dt; //the time-step of the estimation
	MatrixXd A, A1; //dyanamic matrices for the motion model
	MatrixXd Q, Q1; //noise matrices for the motion model
	double R; //receiver noise (it is considered the same to all receivers)
} robot_list;

//initializes a list of robots to be tracked
robot_list* estimator_init(int robotn, double dt);

//function for robot registration (the code positions in the vector are inportant for later maps of codes with the same number to actual code positions)
int estimator_add_robot(robot_list* probotlist, string name, vector<int>& codes, vector<double>& marker_pos);

//function that finds the slot of an robot that contain an emitter with the given code
int estimator_find_robot(robot_list* probotlist, int code);

//function for robot removal
int estimator_remove_robot(robot_list* probotlist, int slot);

//propagate robot pose based on dynamics
void estimator_predict_robot(robot_list* probotlist, int slot);

//update robot pose and velocity given the measurements present in the receiver structure
void estimator_update_robot(robot_list* probotlist, int slot, rcv_data* prcv);

//perform marker association of the observed markers to the actual robot markers
//NOTE: this is a mapping function, and it depends on each robot implementation (the introduction of robot types would originate in different mappings)
int estimator_marker_association(robot_list* probotlist, int slot, rcv_data* prcv, int first_emitter, int* association);

#endif
//ESTIMATOR_ROBOT
