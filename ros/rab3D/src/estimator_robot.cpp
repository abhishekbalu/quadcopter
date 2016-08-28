/* === System includes === */
#include <iostream>
#include <string.h>
#include <math.h>

/* === Local includes === */
#include "estimator_robot.h"
#include "RaB3D.h"

using namespace std;

//defines
#define PI 3.14159265

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
MatrixXd H1(4,7);
MatrixXd K1(7,4);
VectorXd e1(7,1);
MatrixXd eye4 = MatrixXd::Identity(4,4);
MatrixXd eye7 = MatrixXd::Identity(7,7);

//initializes a list of robots to be tracked
//NOTE: the noises and dynamic matrices are all hardcoded
robot_list* estimator_init(int robotn, double dt){

	//alocating memory for the list
	robot_list* probotlist = new robot_list; //memory for the list
	probotlist->size = robotn; //insert list size
	probotlist->probot = new robot_data[robotn]; //alocate the actual memory for the list data

	//initialize estimator parameters
	probotlist->dt = dt; //the time step of the estimation
	probotlist->Q = MatrixXd::Identity(4,4); //motion noise for the raw quadrotor
	probotlist->Q = probotlist->Q*(0.05*0.05);
	double w1 = 0.025*0.025, w2 = 0.2*0.2, w3 = 0.2;
	probotlist->Q1=MatrixXd(7,7); //motion dynamics for the robot which includes velocity
	probotlist->Q1 << w1 , 0 ,  0 ,  0 ,  0 ,  0 ,  0,
	                  0 , w1 ,  0 ,  0 ,  0 ,  0 ,  0,
	                  0 ,  0 , w1 ,  0 ,  0 ,  0 ,  0,
	                  0 ,  0 ,  0 , w3 ,  0 ,  0 ,  0,
	                  0 ,  0 ,  0 ,  0 , w2 ,  0 ,  0,
	                  0 ,  0 ,  0 ,  0 ,  0 , w2 ,  0,
	                  0 ,  0 ,  0 ,  0 ,  0 ,  0 , w2;
	probotlist->A = MatrixXd::Identity(4,4); //motion dynamcis for the raw robot
	probotlist->A1 = MatrixXd(7,7); //motion dynamics for the robot which includes velocity
	probotlist->A1 << 1 , 0 , 0 , 0 , dt ,  0 ,  0,
	                  0 , 1 , 0 , 0 ,  0 , dt ,  0,
	                  0 , 0 , 1 , 0 ,  0 ,  0 , dt,
	                  0 , 0 , 0 , 1 ,  0 ,  0 ,  0,
	                  0 , 0 , 0 , 0 ,  1 ,  0 ,  0,
	                  0 , 0 , 0 , 0 ,  0 ,  1 ,  0,
	                  0 , 0 , 0 , 0 ,  0 ,  0 ,  1;
	probotlist->R = 2*2; //the noise of the receiver RSS

	//initialize what can be initialized on the Kalman Filter matrices
	H1 << 1 , 0 , 0 , 0 , 0 , 0 , 0,
	      0 , 1 , 0 , 0 , 0 , 0 , 0,
	      0 , 0 , 1 , 0 , 0 , 0 , 0,
	      0 , 0 , 0 , 1 , 0 , 0 , 0;

	//un-register the robots by default
	for(int k = 0; k < robotn; k++){
		probotlist->probot[k].name = ""; //reset robot name
		probotlist->probot[k].initialized = -1; //un-registered robot
		probotlist->probot[k].updated = 0; //no new information is available on the estimate
		probotlist->probot[k].codes.clear(); //reseting its codes
		probotlist->probot[k].x_markers.clear(); //reseting marker positions
		probotlist->probot[k].ego = NULL; //reseting lag information
		//not necessary to initialize x or P of the robot (these will be initialized at the first update)
	}

	//reseting receiver mask (the above global auxiliar variable)
	memset(rcv_mask,0,MAX_RECEIVERS*sizeof(char));

	//return the generated list
	return probotlist;
}

//function for robot registration (the code positions in the vector are inportant for later maps of codes with the same number to actual code positions)
int estimator_add_robot(robot_list* probotlist, string name, vector<int>& codes, vector<double>& marker_pos){

	//find a free spot in the estimation list
	int free_slot = -1; //if this value remains -1 then there was no free spot
	for(int k = 0, n = probotlist->size; k < n; k++)
		if(probotlist->probot[k].initialized == -1){ //if there is a free spot
			free_slot = k; //make note of the slot position
			break; //do not need to search more
		}

	//if no free position then return imediately
	if(free_slot >= 0){

		//fill robot structure
		probotlist->probot[free_slot].name = name; //insert robot name
		probotlist->probot[free_slot].initialized = 0; //the robot is still not initialized
		probotlist->probot[free_slot].updated = 0; //no new information is available on the estimate
		probotlist->probot[free_slot].codes.insert(probotlist->probot[free_slot].codes.begin(),codes.begin(),codes.end()); //insert robot codes
		probotlist->probot[free_slot].x_markers.insert(probotlist->probot[free_slot].x_markers.begin(), marker_pos.begin(), marker_pos.end()); //insert marker positions
		probotlist->probot[free_slot].ego = egomotion_init_structure();
		//not necessary to initialize x or P of the robot (these will be initialized at the first update)
	}

	//return the slot where the robot was saved. A value of -1 will be issued in case of no free spot found
	return free_slot;

}

//function that finds the slot of an robot that matches the given code
int estimator_find_robot(robot_list* probotlist, int code){

	for(int k = 0, n = probotlist->size; k < n; k++)
		if(probotlist->probot[k].initialized >= 0) //if the slot is filled
			for(int l = 0, m = probotlist->probot[k].codes.size(); l < m; l++)
				if(probotlist->probot[k].codes[l] == code) //if the codes matches
					return k; //return slot position (no more robots should have this code)

	//no robot was found
	return -1;

}

//function for robot removal
int estimator_remove_robot(robot_list* probotlist, int slot){

	//make sure the slot is valid - the removal can be done
	if( (slot >= 0) && (slot < probotlist->size) ){
		probotlist->probot[slot].name = ""; //reset robot name
		probotlist->probot[slot].initialized = -1; //un-registered robot
		probotlist->probot[slot].updated = 0; //no new information is available on the estimate
		probotlist->probot[slot].codes.clear(); //reseting its code
		probotlist->probot[slot].x_markers.clear(); //reseting marker positions
		delete probotlist->probot[slot].ego->Ps; //resetinglag information
		delete probotlist->probot[slot].ego; 
	}

	//return the slot where the robot was removed. A value of -1 will be issued in case of no emitter with the specified code was found
	return slot;
}

//propagate robot pose based on dynamics
void estimator_predict_robot(robot_list* probotlist, int slot){

	//make sure the slot is valid
	if( (slot >= 0) && (slot < probotlist->size) ){

		//propagate states acording to the dynamic models
		//only propagate if the states were already initialized
		if(probotlist->probot[slot].initialized > 0){

			//possible motion of the tracked robot
			probotlist->probot[slot].x = probotlist->A * probotlist->probot[slot].x; //propagate the robot state
			probotlist->probot[slot].x1 = probotlist->A1 * probotlist->probot[slot].x1; //propagate the robot state acording to the estimated velocity
			probotlist->probot[slot].P = probotlist->A * probotlist->probot[slot].P * probotlist->A.transpose() + probotlist->Q; //propagate state covariance
			probotlist->probot[slot].P1 = probotlist->A1 * probotlist->probot[slot].P1 * probotlist->A1.transpose() + probotlist->Q1; //propagate state covariance with velocity
		
			//add egomotion to the previous tracked robot motion (added required information to the lag lists)
			egomotion_predict(probotlist->probot[slot].ego, probotlist->probot[slot].x1, probotlist->probot[slot].P1);
		}

	}

}

//update robot pose and velocity given the measurements present in the receiver structure
void estimator_update_robot(robot_list* probotlist, int slot, rcv_data* prcv){

	//ignore updates if robot is not initialized
	if(probotlist->probot[slot].initialized < 0) return;

	//go through the emitters and find the first code that belongs to the robot that is being estimated (-1 is given if no code belongs to the robot)
	int first_emitter = -1;
	for(int k = 0, n = prcv->emitn; k < n; k++)
		if( estimator_find_robot(probotlist, prcv->code[k]) == slot )
			{first_emitter = k; break;}

	//make sure there are codes belonging to the robot
	if(first_emitter >= 0){

		//perform marker association (if the number of associated markers is less than the total, the remaining vector elements will be -1)
		//if the function returns -1, then the marker association has failled (for example, there are not enough markers)
		int marker_association[10];
		int nmarkers = estimator_marker_association(probotlist, slot, prcv, first_emitter, marker_association);

		//cout << nmarkers << " " << marker_association[0] << " " << marker_association[1] << " " << marker_association[2] << " " << marker_association[3] << endl;

		//only proceed if more than two markers were associated
		if(nmarkers >= 2){

			//go through the emitters that were associated and extract the maximum RSS values obtained for each one
			double max_rsss[10];
			int max_inds[10];
			int max_inds_e[10];
			int max_associations[10];
			
			
			for(int k = 0, nr = prcv->rcvn; k < nmarkers; k++){ 
				
				//take the maximum RSS value obtained for this emitter
				double max_rss = EMITTER_RSS(prcv, first_emitter + k, 0); 
				int max_ind = 0;
				for(int l = 1; l < nr; l++) 
					if( EMITTER_RSS(prcv, first_emitter + k, l) > max_rss)
						{max_rss = EMITTER_RSS(prcv, first_emitter + k, l); max_ind = l;}

				//take note of the strength and index in a list (these list will be ordered later), and the respective marker association
				max_rsss[k] = max_rss;
				max_inds[k] = max_ind;
				max_associations[k] = marker_association[k]; //association between code number and code position in quadrotor
				max_inds_e[k] = k; //to convert the associations to the actual positions in the emitter vector, just add first emitter
			}
			
			//order the markers by intensity of their highest receiver value
			for(int k = 0; k < nmarkers; k++){
				for(int l = k; l > 0; l--)
					if(max_rsss[l] > max_rsss[l - 1]){ //by descending order
						double auxd = max_rsss[l - 1]; //swap rss
						max_rsss[l -1] = max_rsss[l];
						max_rsss[l] = auxd;
						int auxi = max_inds[l -1]; //swap index
						max_inds[l -1] = max_inds[l];
						max_inds[l] = auxi;
						auxi = max_associations[l -1]; //swap associations
						max_associations[l - 1] = max_associations[l];
						max_associations[l] = auxi;
						auxi = max_inds_e[l -1]; //swap associations
						max_inds_e[l - 1] = max_inds_e[l];
						max_inds_e[l] = auxi;
					}
			} //the two markers with highest RSS values will be chosen
			//here the existance of two markers to use in the update is always assured
/*
			cout << "----------------" << endl;
			cout << "Quadrotor " << slot << endl;
			cout << first_emitter << endl;
			for(int k = 0; k < nmarkers; k++){
				cout << k << ": " << max_associations[k] << " " << max_inds[k] << " " << max_rsss[k] << endl;
			}
*/
			//use a simple initialization protocol (based on the marker with the highest RSS) if this is the first update
			if(probotlist->probot[slot].initialized == 0){

				//initialize robot bearing
				double bearing[3];
				bearing[0] = prcv->r[3*max_inds[0] + 0];
				bearing[1] = prcv->r[3*max_inds[0] + 1];
				bearing[2] = prcv->r[3*max_inds[0] + 2];

				//initialize robot heading
				double delta_psi = atan2(probotlist->probot[slot].x_markers[max_associations[0]*3 + 1], probotlist->probot[slot].x_markers[max_associations[0]*3 + 0]);
				double heading = PI + atan2(bearing[1], bearing[0]) - delta_psi;

				//initialize range of the emitter
				//it detects if the receiver is non ON and if the value is big enough to be considered to estimate distance
				double d = RaB3D_find_range_from_RSS(prcv, max_inds[0], max_rsss[0]);

				//initialize robot only if the used receiver value was considered to be valid
				if( d >= 0 ){

					//initialize robot estimates
					probotlist->probot[slot].x = VectorXd(4,1); //only 3D pose
					probotlist->probot[slot].x << d*bearing[0], d*bearing[1], d*bearing[2], heading;
					probotlist->probot[slot].x1 = VectorXd(7,1); //3D pose and velocity
					probotlist->probot[slot].x1 << d*bearing[0], d*bearing[1], d*bearing[2], heading, 0.0, 0.0, 0.0; //velocity is assumed zero at the beginning

					//initialize the covariance matrices
					probotlist->probot[slot].P = MatrixXd::Identity(4,4)*(0.5*0.5); //just for the robot pose
					probotlist->probot[slot].P1 = MatrixXd::Identity(7,7)*(0.5*0.5); //robot pose with velocity

					//the emitter is now initialized
					probotlist->probot[slot].initialized = 1;

				}

			}
			
			//if initialization was done properly, or already in prevous updates, normally update the estimate
			double* x_mrk = (double*)probotlist->probot[slot].x_markers.data(); //auxiliar variables
			if(probotlist->probot[slot].initialized == 1){
				
				//go through the two emitters with highest detected intensities
				for(int kemit = 0; kemit < 2; kemit++){

					double cy = cos(probotlist->probot[slot].x[3]), sy = sin(probotlist->probot[slot].x[3]); //auxiliar variables

					//generate predicted emitter position acording to estimated robot pose
					vaux3D[0] = probotlist->probot[slot].x[0] + x_mrk[max_associations[kemit]*3 + 0]*cy - x_mrk[max_associations[kemit]*3 + 1]*sy; //use a double* to store the predicted emitter position
					vaux3D[1] = probotlist->probot[slot].x[1] + x_mrk[max_associations[kemit]*3 + 0]*sy + x_mrk[max_associations[kemit]*3 + 1]*cy;
					vaux3D[2] = probotlist->probot[slot].x[2] + x_mrk[max_associations[kemit]*3 + 2];
					
					//predict RSS values from the receivers, when detecting this emitter at the supposed position
					RaB3D_generate_RSS_multi(prcv, vaux3D, rcv_predictions, J, rcv_mask, 0.3);

					//select the receivers to apply on the update (compute measurement errors)
					for(int k = 0, n = prcv->rcvn; k < n; k++)
						rcv_mask[k] = rcv_mask[k]*(char)(EMITTER_RSS(prcv, first_emitter + max_inds_e[kemit], k) >= 10);

					//create errors and jacobian for the selected receievers
					for(int k = 0, n = prcv->rcvn; k < n; k++)
						errors[k] = EMITTER_RSS(prcv, first_emitter + max_inds_e[kemit], k) - rcv_predictions[k];

					//place errors and jaconians into errors and matrices (ignore unselected receievers)
					int select_count = 0; //count the number of selected receivers
					for(int k = 0, n = prcv->rcvn; k < n; k++) select_count+=rcv_mask[k];
					MatrixXd H(select_count, 4); //generate matrix memory
					MatrixXd K;
					MatrixXd R = probotlist->R * MatrixXd::Identity(select_count,select_count);
					VectorXd e(select_count);
					for(int k = 0, n = prcv->rcvn, l = 0; k < n; k++) //filling matrices
						if(rcv_mask[k] == 1){ //only fill with selected receivers
							e(l) = errors[k]; //placing errors
							H(l  ,0) = J[3*k + 0]; //placing Jacobians
							H(l  ,1) = J[3*k + 1];
							H(l  ,2) = J[3*k + 2];
							H(l++,3) = J[3*k + 0]*(-x_mrk[max_associations[kemit]*3 + 0]*sy - x_mrk[max_associations[kemit]*3 + 1]*cy) + \
							           J[3*k + 1]*(+x_mrk[max_associations[kemit]*3 + 0]*cy - x_mrk[max_associations[kemit]*3 + 1]*sy);
						}

					//apply the EKF update equations using the errors and jacobians of the selected receivers
					//NOTE: if there are less than three receievers the estimator might diverge fast
						
					K = probotlist->probot[slot].P*H.transpose() * ( H*probotlist->probot[slot].P*H.transpose() + R).inverse();
					probotlist->probot[slot].x = probotlist->probot[slot].x + K*e;
					probotlist->probot[slot].P = (eye4 - K*H)*probotlist->probot[slot].P;				

				}

				//convert sensor measurement to the flying frame
				VectorXd measure = probotlist->probot[slot].x;
				MatrixXd R = probotlist->probot[slot].P;
				egomotion_convert_to_flying_frame(measure, R);

				//apply the EKF update for the estimated robot that includes the velocity component (e1, K1, and H1 are already allocated and H1 is fixed)
				//NOTE: the used measure uncertainty 10 * the raw estimation uncertainty (P), as the raw estimator underestimates the uncertainty
				e1(0) = measure(0) - probotlist->probot[slot].x1(0); //computing measurement error
				e1(1) = measure(1) - probotlist->probot[slot].x1(1);
				e1(2) = measure(2) - probotlist->probot[slot].x1(2);
				e1(3) = measure(3) - probotlist->probot[slot].x1(3);
				K1 = probotlist->probot[slot].P1*H1.transpose() * (H1*probotlist->probot[slot].P1*H1.transpose() + 10*R).inverse();
				probotlist->probot[slot].x1 = probotlist->probot[slot].x1 + K1*e1;
				probotlist->probot[slot].P1 = (eye7 - K1*H1)*probotlist->probot[slot].P1;

				//new information is now available for this estimate
				probotlist->probot[slot].updated = 1;

			}

		}

	}

}

//perform marker association of the observed markers to the actual robot markers
//NOTE: this is a mapping function, and it depends on each robot implementation (the introduction of robot types would originate in different mappings)
//WARNING: this assumes that two consecutive elements are visible at all times (it might generate some bug)
int estimator_marker_association(robot_list* probotlist, int slot, rcv_data* prcv, int first_emitter, int* association){

	//count the number of emitters that belong to the robot
	int emit_count = 1; //if this function is called, 1 emitter already belongs to the quadrotor
	for(int k = first_emitter + 1, n = prcv->emitn; k < n; k++){ //the remaining are contiguous
		if( estimator_find_robot(probotlist, prcv->code[k]) != slot ) //stop if the emitters nolonger belong to the robot
			break;
		emit_count++;
	}
/*
	for(int k = 0; k < emit_count; k++)
		cout << prcv->code[first_emitter + k] << " ";
	cout << endl;
	*/

	//associate the markers according to a mapping function (this is a quadrotor mapping function)
	if(emit_count > 4) return -1; //a quadrotor can only have 4 markers max
	else if(emit_count == 4)
		{association[0] = 0; association[1] = 1; association[2] = 2; association[3] = 3; return emit_count;}
	else if(emit_count == 3){
		association[3] = -1;
		if(prcv->code[first_emitter] == probotlist->probot[slot].codes[0]){
			if( (prcv->slot[first_emitter + 1] - prcv->slot[first_emitter + 0]) <= 2){
				if(prcv->code[first_emitter + 2] == probotlist->probot[slot].codes[0])
					{association[0] = 0; association[1] = 1; association[2] = 3;}
				else
					{association[0] = 0; association[1] = 1; association[2] = 2;}
			}
			else
				{association[0] = 0; association[1] = 2; association[2] = 3;}
		}
		else
			{association[0] = 1; association[1] = 2; association[2] = 3;}
		return emit_count;
	}
	else if(emit_count == 2){
		association[2] = association[3] = -1;
		if(prcv->code[first_emitter] == probotlist->probot[slot].codes[0]){
			if(prcv->code[first_emitter + 1] == probotlist->probot[slot].codes[1]){
				if( (prcv->slot[first_emitter + 1] - prcv->slot[first_emitter + 0]) <= 2)
					{association[0] = 0; association[1] = 1;}
				else
					{association[0] = 0; association[1] = 2;}
			}
			else
				{association[0] = 0; association[1] = 3;}
		}
		else {
			if(prcv->code[first_emitter + 1] == probotlist->probot[slot].codes[0]){
				if( (prcv->slot[first_emitter + 1] - prcv->slot[first_emitter + 0]) <= 2)
					{association[0] = 2; association[1] = 3;}
				else
					{association[0] = 1; association[1] = 3;}
			}
			else
				{association[0] = 1; association[1] = 2;}
		}
		return emit_count;
	}
	else if(emit_count <= 1) return -1; //less than two markers does not allow for marker association

	return -1; //should not reach this ending (but just in case)

}
