/* === System includes === */
#include <iostream>
#include <string.h>
#include <math.h>

/* === Local includes === */
#include "estimator_emit.h"
#include "RaB3D.h"

using namespace std;

//auxiliar vector that indicates the receivers to be considered in the update function
char rcv_mask[MAX_RECEIVERS];

//auxiliar vectors that contains the respective receiver values and Jacobians
double rcv_predictions[MAX_RECEIVERS];
double J[3*MAX_RECEIVERS];

//error vector for the EKF
double errors[MAX_RECEIVERS];

//auxiliar vector for 3D positions in space 
double vaux3D[3];

//Kalman Filter pre-allocated vectors
MatrixXd H1(3,6);
MatrixXd K1(6,3);
VectorXd e1(6,1);
MatrixXd eye3 = MatrixXd::Identity(3,3);
MatrixXd eye6 = MatrixXd::Identity(6,6);

//initializes a list of emitters to be tracked
//NOTE: the noises and dynamic matrices are all hardcoded
emt_list* estimator_init(int emitn, double dt){

	//alocating memory for the list
	emt_list* pemitlist = new emt_list; //memory for the list
	pemitlist->size = emitn; //insert list size
	pemitlist->pemit = new emt_data[emitn]; //alocate the actual memory for the list data

	//initialize estimator parameters
	pemitlist->dt = dt; //the time step of the estimation
	pemitlist->Q=MatrixXd(3,3);
	pemitlist->Q << 1,0,0,0,1,0,0,0,1; //motion noise for the raw emitter
	pemitlist->Q = pemitlist->Q*(0.05*0.05); //same
	double w1 = 0.02*0.02, w2 = 0.2*0.2;
	pemitlist->Q1=MatrixXd(6,6); //motion dynamics for the emitter which includes velocity
	pemitlist->Q1 << w1 ,  0 ,  0 ,  0 ,  0 ,  0,
	                  0 , w1 ,  0 ,  0 ,  0 ,  0,
	                  0 ,  0 , w1 ,  0 ,  0 ,  0,
	                  0 ,  0 ,  0 , w2 ,  0 ,  0,
	                  0 ,  0 ,  0 ,  0 , w2 ,  0,
	                  0 ,  0 ,  0 ,  0 ,  0 , w2;
	pemitlist->A=MatrixXd(3,3); //motion dynamcis for the raw emitter
	pemitlist->A << 1,0,0,0,1,0,0,0,1;
	pemitlist->A1=MatrixXd(6,6); //motion dynamics for the emitter which includes velocity
	pemitlist->A1 << 1 , 0 , 0 , dt ,  0 ,  0,
	                 0 , 1 , 0 ,  0 , dt ,  0,
	                 0 , 0 , 1 ,  0 ,  0 , dt,
	                 0 , 0 , 0 ,  1 ,  0 ,  0,
	                 0 , 0 , 0 ,  0 ,  1 ,  0,
	                 0 , 0 , 0 ,  0 ,  0 ,  1;
	pemitlist->R = 2*2; //the noise of the receiver RSS

	//initialize what can be initialized on the Kalman Filter matrices
	H1 << 1 , 0 , 0 , 0 , 0 , 0 ,
	      0 , 1 , 0 , 0 , 0 , 0 ,
	      0 , 0 , 1 , 0 , 0 , 0 ;

	//un-register the emitters by default
	for(int k = 0; k < emitn; k++){
		pemitlist->pemit[k].initialized = -1; //un-registered emitter
		pemitlist->pemit[k].updated = 0; //no new information is available on the estimate
		pemitlist->pemit[k].code = 0; //reseting its code
		//not necessary to initialize x or P of the emitter (these will be initialized at the first update)
	}

	//reseting receiver mask (the above global auxiliar variable)
	memset(rcv_mask,0,MAX_RECEIVERS*sizeof(char));

	//return the generated list
	return pemitlist;
}

//function for emitter registration
int estimator_add_emitter(emt_list* pemitlist, int code){

	//find a free emitter spot
	int free_slot = -1; //if this value remains -1 then there was no free spot
	for(int k = 0, n = pemitlist->size; k < n; k++)
		if(pemitlist->pemit[k].initialized == -1){ //if there is a free spot
			free_slot = k; //make note of the slot position
			break; //do not need to search more
		}

	//if no free position then return imediately
	if(free_slot >= 0){

		//fill emitter structure
		pemitlist->pemit[free_slot].initialized = 0; //the emitter is still not initialized
		pemitlist->pemit[free_slot].updated = 0; //no new information is available on the estimate
		pemitlist->pemit[free_slot].code = code; //insert emitter code
		//not necessary to initialize x or P of the emitter (these will be initialized at the first update)

	}

	//return the slot where the emitter was saved. A value of -1 will be issued in case of no free spot found
	return free_slot;

}

//function that finds the slot of an emitter that matches the given code
int estimator_find_emitter(emt_list* pemitlist, int code){

	int slot = -1; //if this value remains -1 then there was no emitter with the specified code
	for(int k = 0, n = pemitlist->size; k < n; k++)
		if(pemitlist->pemit[k].initialized >= 0) //if the slot is filled
			if(pemitlist->pemit[k].code == code) //if the codes matches
				slot = k; //make note of the slot position

	return slot;

}

//function for emitter removal
int estimator_remove_emitter(emt_list* pemitlist, int slot){

	//make sure the slot is valid - the removal can be done
	if( (slot >= 0) && (slot < pemitlist->size) ){
		pemitlist->pemit[slot].initialized = -1; //un-registered emitter
		pemitlist->pemit[slot].updated = 0; //no new information is available on the estimate
		pemitlist->pemit[slot].code = 0; //reseting its code
	}

	//return the slot where the emitter was removed. A value of -1 will be issued in case of no emitter with the specified code was found
	return slot;
}

//propagate emitter position based on dynamics
void estimator_predict_single_marker(emt_list* pemitlist, int slot){

	//make sure the slot is valid
	if( (slot >= 0) && (slot < pemitlist->size) ){

		//propagate states acording to the dynamic models
		//only propagate if the states were already initialized
		if(pemitlist->pemit[slot].initialized > 0){
			pemitlist->pemit[slot].x = pemitlist->A * pemitlist->pemit[slot].x; //propagate the emitter state
			pemitlist->pemit[slot].x1 = pemitlist->A1 * pemitlist->pemit[slot].x1; //propagate the emitter state acording to the estimated velocity
			pemitlist->pemit[slot].P = pemitlist->A * pemitlist->pemit[slot].P * pemitlist->A.transpose() + pemitlist->Q; //propagate state covariance
			pemitlist->pemit[slot].P1 = pemitlist->A1 * pemitlist->pemit[slot].P1 * pemitlist->A1.transpose() + pemitlist->Q1; //propagate state covariance with velocity
		}

		//WARNING: we are missing here the actuation from the motors (vyaw, roll, pitch, and thrust, so we can predict ego-motion)
		//can we by moving detect the reflections? (here would be the syntetic radar)

	}

}

//update emitter position and velocity given the measurements present in the receiver structure
void estimator_update_single_marker(emt_list* pemitlist, int slot, rcv_data* prcv){

	//make sure the slot is valid
	if(slot >= 0){

		//find the emitter number on the RaB3D values list
		int emit_ind = RaB3D_find_emitter(prcv, pemitlist->pemit[slot].code);

		//use a simple initialization protocol if this is the first update
		if(pemitlist->pemit[slot].initialized == 0){

			//select the receiever measuring the highest RSS
			short int max_ind = 0, max_value = EMITTER_RSS(prcv, emit_ind, 0);
			for(int k = 1; k < prcv->rcvn; k++)
				if( EMITTER_RSS(prcv, emit_ind, k) > max_value ){
					max_ind = k;
					max_value = EMITTER_RSS(prcv, emit_ind, k);
				}

			//initialize the bearing of the emitter
			double bearing[3];
			bearing[0] = prcv->r[3*max_ind + 0];
			bearing[1] = prcv->r[3*max_ind + 1];
			bearing[2] = prcv->r[3*max_ind + 2];

			//initialize range of the emitter
			//it detects if the receievr is non ON and if the value is big enough to be considered to estimate distance
			double d = RaB3D_find_range_from_RSS(prcv, max_ind, max_value);

			//initialize emitter only if the used receiver value was considered to be valid
			if( d >= 0 ){

				//initialize emitter estimates
				pemitlist->pemit[slot].x = VectorXd(3,1); //only 3D position
				pemitlist->pemit[slot].x << d*bearing[0], d*bearing[1], d*bearing[2];
				pemitlist->pemit[slot].x1 = VectorXd(6,1); //3D position and velocity
				pemitlist->pemit[slot].x1 << d*bearing[0], d*bearing[1], d*bearing[2], 0.0, 0.0, 0.0; //velocity is assumed zero at the beginning

				//initialize the covariance matrices
				pemitlist->pemit[slot].P = MatrixXd(3,3); //just for the emitter position
				pemitlist->pemit[slot].P << 0.5*0.5, 0, 0, 0, 0.5*0.5, 0, 0, 0, 0.5*0.5;
				pemitlist->pemit[slot].P1 = MatrixXd(6,6); //emitter position with velocity
				pemitlist->pemit[slot].P1 << 0.5*0.5 ,    0    ,    0    ,    0    ,    0    ,    0   ,
				                                0    , 0.5*0.5 ,    0    ,    0    ,    0    ,    0   ,
				                                0    ,    0    , 0.5*0.5 ,    0    ,    0    ,    0   ,
				                                0    ,    0    ,    0    , 0.5*0.5 ,    0    ,    0   ,
				                                0    ,    0    ,    0    ,    0    , 0.5*0.5 ,    0   ,
				                                0    ,    0    ,    0    ,    0    ,    0    , 0.5*0.5;

				//the emitter is now initialized
				pemitlist->pemit[slot].initialized = 1;

			}

		}
		
		//if this is not the first update for this slot 
		else {

			//predict RSS values from the receivers, when detecting this emitter at the supposed position
			vaux3D[0] = pemitlist->pemit[slot].x[0]; //use a double* to store the predicted emitter position
			vaux3D[1] = pemitlist->pemit[slot].x[1];
			vaux3D[2] = pemitlist->pemit[slot].x[2];
			RaB3D_generate_RSS_multi(prcv, vaux3D, rcv_predictions, J, rcv_mask, 0.15);

			//select the receivers to apply on the update (compute measurement errors)
			for(int k = 0, n = prcv->rcvn, m = prcv->emitn; k < n; k++)
				
				rcv_mask[k] = rcv_mask[k]*(char)(EMITTER_RSS(prcv, emit_ind, k) >= 10);

			//create errors and jacobian for the selected receievers
			for(int k = 0, n = prcv->rcvn, m = prcv->emitn; k < n; k++)
				errors[k] = EMITTER_RSS(prcv, emit_ind, k) - rcv_predictions[k];

			//place errors and jaconians into errors and matrices (ignore unselected receievers)
			int select_count = 0; //count the number of selected receivers
			for(int k = 0, n = prcv->rcvn; k < n; k++) select_count+=rcv_mask[k];
			MatrixXd H(select_count, 3); //generate matrix memory
			MatrixXd K;
			MatrixXd R = pemitlist->R * MatrixXd::Identity(select_count,select_count);
			VectorXd e(select_count);
			for(int k = 0, n = prcv->rcvn, l = 0; k < n; k++) //filling matrices
				if(rcv_mask[k] == 1){ //only fill with selected receivers
					e(l) = errors[k]; //placing errors
					H(l  ,0) = J[3*k + 0]; //placing Jacobians
					H(l  ,1) = J[3*k + 1];
					H(l++,2) = J[3*k + 2];
				}

			//apply the EKF update equations using the errors and jacobians of the selected receivers
			//NOTE: if there are less than three receievers the estimator might diverge fast
			K = pemitlist->pemit[slot].P*H.transpose() * ( H*pemitlist->pemit[slot].P*H.transpose() + R).inverse();
			pemitlist->pemit[slot].x = pemitlist->pemit[slot].x + K*e;
			pemitlist->pemit[slot].P = (eye3 - K*H)*pemitlist->pemit[slot].P;

			//apply the EKF update for the estimated emitter that includes the velocity component (e1, K1, and H1 are already allocated and H1 is fixed)
			//NOTE: the used measure uncertainty 10 * the raw estimation uncertainty (P), as the raw estimator underestimates the uncertainty
			e1(0) = pemitlist->pemit[slot].x(0) - pemitlist->pemit[slot].x1(0); //computing measurement error
			e1(1) = pemitlist->pemit[slot].x(1) - pemitlist->pemit[slot].x1(1);
			e1(2) = pemitlist->pemit[slot].x(2) - pemitlist->pemit[slot].x1(2);
			K1 = pemitlist->pemit[slot].P1*H1.transpose() * (H1*pemitlist->pemit[slot].P1*H1.transpose() + 10*pemitlist->pemit[slot].P).inverse();
			pemitlist->pemit[slot].x1 = pemitlist->pemit[slot].x1 + K1*e1;
			pemitlist->pemit[slot].P1 = (eye6 - K1*H1)*pemitlist->pemit[slot].P1;

			//new information is now available for this estimate
			pemitlist->pemit[slot].updated = 1;

		}
	}

}

//a quadrotor is composed of several emitters
//we have to have a quadrotor associated to several emitter objects
//when the emitter message comes, we have all emitters ready with their measurements
//we should have a 