#ifndef ESTIMATOR
#define ESTIMATOR

/* === System includes === */
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>

/* === Local includes === */
#include "RaB3D.h"

using namespace Eigen;

//structures where the emitter data will be saved
//data for each slot
typedef struct t_emt_data{
	int code; //emitter code (identifies each emitter)
	int initialized; //if initialized = -1 the slot is available; if = 0 then the slot is filled but in an un-initialized mode; if > 0 then the slot is filled and in an initialized mode
	int updated; //if there is any measurement that updated this estimate between the time of two consecutive ros message, then there is new information to be sent on the next message
	VectorXd x, x1; //estimated emitter 3D position in the environment (it is only valid when initialized > 0)
	MatrixXd P, P1; //estimation covariance (it is only valid when initialized > 0)
} emt_data;

//the entire list manager
typedef struct t_emt_list{
	int size; //the amount of slots that the list contains
	emt_data* pemit; //where the emitter slots will be stored
	double dt; //the time-step of the estimation
	MatrixXd A, A1; //dyanamic matrices for the motion model
	MatrixXd Q, Q1; //noise matrices for the motion model
	double R; //receiver noise (it is considered the same to all receivers)
} emt_list;

//initializes a list of emitters to be tracked
emt_list* estimator_init(int emitn, double dt);

//function for emitter registration
int estimator_add_emitter(emt_list* pemitlist, int code);

//function for emitter removal
int estimator_remove_emitter(emt_list* pemitlist, int code);

//function that finds the slot of an emitter that matches the given code
int estimator_find_emitter(emt_list* pemitlist, int code);

//propagate emitter position based on dynamics
void estimator_predict_single_marker(emt_list* pemitlist, int slot);

//update emitter position and velocity given the measurements present in the receiver structure
void estimator_update_single_marker(emt_list* pemitlist, int slot, rcv_data* prcv);

#endif
//ESTIMATOR
