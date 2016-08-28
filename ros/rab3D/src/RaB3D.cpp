//system includes
#include <iostream>
#include <string.h>
#include <math.h>

//local includes
#include "RaB3D.h"

//this structure will be used to store messages with mapped memory
//it has the same structure as RaB3DValues message except for the stamp
typedef struct t_RaB3DValuesMem{
	char stamp;
	char rn;
	char ON[MAX_RECEIVERS];
	char emitn;
	char detected[MAX_EMITTERS];
	short code[MAX_EMITTERS];
	short slot[MAX_EMITTERS];
	short values[MAX_RECEIVERS*MAX_EMITTERS];
} RaB3DValuesMem;

using namespace std;

//initializes the entire receiver structure from a received info meassage
rcv_data* RaB3D_initialize_from_message(const quad_msgs::RaB3DInfo::ConstPtr& info_msg){

	//initialize one rcv_data structure
	rcv_data* prcv = new rcv_data;

	//initialize info_msg structure
	int bncount = 0; //auxiliar variable to count amplification stage model branches (see explanations below)
	prcv->info_msg.rn = info_msg->rn; //number of receivers
	for(int k = 0; k < info_msg->rn; k++){ //go through the receivers

		//initialize receiver positions
		prcv->info_msg.x.push_back(info_msg->x[k*3 + 0]);
		prcv->info_msg.x.push_back(info_msg->x[k*3 + 1]);
		prcv->info_msg.x.push_back(info_msg->x[k*3 + 2]);

		//initialize receiver headings
		prcv->info_msg.r.push_back(info_msg->r[k*3 + 0]);
		prcv->info_msg.r.push_back(info_msg->r[k*3 + 1]);
		prcv->info_msg.r.push_back(info_msg->r[k*3 + 2]);

		//initialize receiver RSS model parameters
		//absorption coefficient model
		prcv->info_msg.f.push_back(info_msg->f[k*2 + 0]);
		prcv->info_msg.f.push_back(info_msg->f[k*2 + 1]);

		//amplification stage model
		prcv->info_msg.bn.push_back(info_msg->bn[k]); //the number of branches of the amplification stage model
		double s_i; //auxiliar variable (see below)
		for(int l = 0; l < prcv->info_msg.bn[k] + 1; l++){ //go through the branches

			//the branch transition positions
			prcv->info_msg.s_it.push_back(info_msg->s_it[(bncount + k) + l]); //pay attention to + k, since the transition is the number of branches + 1
			s_i = info_msg->s_it[(bncount + k) + l]; //compute the output respective to the transitions

			//amplification stage model parameters
			//do not do the last one, since it just matters for the transitions
			if(l < prcv->info_msg.bn[k]){
				prcv->info_msg.g.push_back(info_msg->g[3*(bncount + l) + 0]);
				prcv->info_msg.g.push_back(info_msg->g[3*(bncount + l) + 1]);
				prcv->info_msg.g.push_back(info_msg->g[3*(bncount + l) + 2]);
				prcv->info_msg.s_ot.push_back(  info_msg->g[3*(bncount + l) + 0] + info_msg->g[3*(bncount + l) + 1]*s_i + info_msg->g[3*(bncount + l) + 2]*s_i*s_i );
			}
			else
				prcv->info_msg.s_ot.push_back(  info_msg->g[3*(bncount + (l-1)) + 0] + info_msg->g[3*(bncount + (l-1)) + 1]*s_i + info_msg->g[3*(bncount + (l-1)) + 2]*s_i*s_i );

		}

		//update the number of amplification stage model that were counted in total
		//important so correct positioning in s_it and g vectors can be achieved for each receiver
		bncount += prcv->info_msg.bn[k];

	}

	//initialize the rest of the rcv_data structure
	prcv->rcvn = prcv->info_msg.rn;
	prcv->x = (double*)prcv->info_msg.x.data();
	prcv->r = (double*)prcv->info_msg.r.data();
	prcv->f = (double*)prcv->info_msg.f.data();
	prcv->bn = (char*)prcv->info_msg.bn.data();
	prcv->s_it = (double*)prcv->info_msg.s_it.data();
	prcv->s_ot = (double*)prcv->info_msg.s_ot.data();
	prcv->g = (double*)prcv->info_msg.g.data();
	prcv->values_msg.values.resize(prcv->rcvn*MAX_EMITTERS); //reserving memory and the size inside the vectors
	prcv->values_msg.ON.resize(prcv->rcvn*MAX_EMITTERS);
	prcv->values_msg.code.resize(MAX_EMITTERS);
	prcv->values_msg.slot.resize(MAX_EMITTERS);
	prcv->s_o = (short int*)prcv->values_msg.values.data(); //get pointer to the first element of the lists
	prcv->ON = (char*)prcv->values_msg.ON.data();
	prcv->code = (short int*)prcv->values_msg.code.data();
	prcv->slot = (short int*)prcv->values_msg.slot.data();
	prcv->emitn = 0; //for now no values message was processed, so no emitter information is available

	//initialize shared memory option
	prcv->stamp_old = 0;

	//debug prcv initialization
	/*
	cout << "results of initialization by info message: " << endl;
	for(int k = 0; k < prcv->rcvn; k++){
		cout << "    receiver " << k << ": " << endl;
		//cout << "        ON = " << prcv->ON[k] << endl;
		cout << "        position (" << prcv->x[k*3 + 0] << "," << prcv->x[k*3 + 1] << "," << prcv->x[k*3 + 2] << ")" << endl;
		cout << "        orientation_x (" << prcv->r[k*3 + 0] << "," << prcv->r[k*3 + 1] << "," << prcv->r[k*3 + 2] << ")" << endl;
		if(k == 1){
		cout << "        showing receiver " << k << " model:" << endl;
		cout << "        f = " << prcv->f[k*2 + 0] << "," << prcv->f[k*2 + 1] << endl;
		bncount = 0;
		for(int l = 0; l < k; l++) bncount += prcv->bn[l];
		cout << "        s_it = "; for(int l = 0; l < prcv->bn[k]+1; l++) cout << prcv->s_it[(bncount + k) + l] << ","; cout << endl;
		cout << "        s_ot = "; for(int l = 0; l < prcv->bn[k]+1; l++) cout << prcv->s_ot[(bncount + k) + l] << ","; cout << endl;
		for(int l = 0; l < prcv->bn[k]; l++)
		cout << "        branch " << l << ": g = " << prcv->g[3*(bncount + l) + 0] << "," << prcv->g[3*(bncount + l) + 1] << "," << prcv->g[3*(bncount + l) + 2] << endl;
		}
	}
	*/

	//give configured rcv_data structure to user
	return prcv;

}

//I can try to change

//set receiver values and status from a received values message
void RaB3D_set_values_from_message(rcv_data* prcv, const quad_msgs::RaB3DValues::ConstPtr& values_msg){

	//check number of receivers
	if( prcv->rcvn != values_msg->rn)
		{cout << "ERROR: Number of receivers in values message different than the given configuration. Shuting down. . ." << endl; exit(1);}

	//number of emitter tracked in the RaB3DValues message
	prcv->emitn = values_msg->emitn;

	//get emitter codes (in this case this is ficticious)
	int rn = prcv->rcvn; //number of receivers of the robot
	int en = values_msg->emitn; //number of emitters in the RaB3DValues message
	int en_detected = 0; //number of detected emitters
	unsigned short* p_s_o = (unsigned short*)values_msg->values.data(); //data with the detected emitter values
	for(int k = 0; k < en; k++) //go through the RaB3DValues message emitters

		//determine the ones that are detected
		if(values_msg->detected[k]){

			//get their codes and slots
			prcv->code[en_detected] = values_msg->code[k];
			prcv->slot[en_detected] = values_msg->slot[k];

			//get their values for each receiver
			for(int l = 0; l < rn; l++)
				prcv->s_o[rn*en_detected + l] = p_s_o[en*l + k];

			//increase the number of detected emitters
			en_detected++;
		}

	//number of detected emitters
	prcv->emitn = en_detected;

	//copy memory to the s_o and ON vectors (the vector size is always the same, prcv->rcvn*MAX_EMITTERS)
	memcpy(prcv->ON, values_msg->ON.data(), (prcv->rcvn)*sizeof(char)); //the receiver status

/*
	//debug values message reception
	cout << "-------- RaB3DValues msg --------" << endl;
	for(int k = 0;k < prcv->rcvn; k++){
		cout << "receiver " << k << ": ";
		for(int l = 0; l < prcv->emitn; l++)
			cout << prcv->s_o[prcv->rcvn*l + k] << " ";
		cout << endl;
		cout << "ON = " << (int)prcv->ON[k] << endl;
	}
	cout << "---------------------------------" << endl;
*/

}

//set receiver values and status from a shared memory chunck
int RaB3D_set_values_from_shared_memory(rcv_data* prcv, void* shm_pointer){

	//values_msg in shared memory
	RaB3DValuesMem* values_msg = static_cast<RaB3DValuesMem*>(shm_pointer);

	//check if a new message is ready
	if(values_msg->stamp == prcv->stamp_old) return 0;
	prcv->stamp_old = values_msg->stamp; //save stamp of the message that is being processed for future comparisons

	//check number of receivers
	if( prcv->rcvn != values_msg->rn)
		{cout << "ERROR: Number of receivers in values message different than the given configuration. Shuting down. . ." << endl; exit(1);}

	//number of emitter tracked in the RaB3DValues message
	prcv->emitn = values_msg->emitn;

	//get emitter codes (in this case this is ficticious)
	int rn = prcv->rcvn; //number of receivers of the robot
	int en = values_msg->emitn; //number of emitters in the RaB3DValues message
	int en_detected = 0; //number of detected emitters
	unsigned short* p_s_o = (unsigned short*)values_msg->values; //data with the detected emitter values
	for(int k = 0; k < en; k++) //go through the RaB3DValues message emitters

		//determine the ones that are detected
		if(values_msg->detected[k]){

			//get their codes and slots
			prcv->code[en_detected] = values_msg->code[k];
			prcv->slot[en_detected] = values_msg->slot[k];

			//get their values for each receiver
			for(int l = 0; l < rn; l++)
				prcv->s_o[rn*en_detected + l] = p_s_o[en*l + k];

			//increase the number of detected emitters
			en_detected++;
		}

	//number of detected emitters
	prcv->emitn = en_detected;

	//copy memory to the s_o and ON vectors (the vector size is always the same, prcv->rcvn*MAX_EMITTERS)
	memcpy(prcv->ON, values_msg->ON, (prcv->rcvn)*sizeof(char)); //the receiver status

/*
	//debug values message reception
	cout << "-------- RaB3DValues msg --------" << endl;
	cout << "emitters: ";
	for(int k = 0;k < prcv->emitn; k++)
		cout << prcv->code[k] << " ";
	cout << endl;
	for(int k = 0;k < prcv->rcvn; k++){
		cout << "receiver " << k << ": ";
		for(int l = 0; l < prcv->emitn; l++)
			cout << prcv->s_o[prcv->rcvn*l + k] << " ";
		cout << endl;
		cout << "ON = " << (int)prcv->ON[k] << endl;
	}
	cout << "---------------------------------" << endl;
*/

	return 1;

}

//finds the position in the receiver structure of the emitter matching the given code (-1 is returned if no such code is found on the list)
int RaB3D_find_emitter(rcv_data* prcv, int code){

	int slot = -1; //if this value remains -1 then there was no emitter with the specified code
	for(int k = 0, n = prcv->emitn; k < n; k++)
			if(prcv->code[k] == code) //if the codes matches
				slot = k; //make note of the slot position

	return slot;
}

//function that computes the receiver value (for receiver k)
double RaB3D_generate_RSS(rcv_data* prcv, double* xe, double* J, int k, double ct_min){

	//check if receiver is ON
	if(prcv->ON[k] == 1){

		//get receiver position and orientation
		double* xr = &prcv->x[3*k];
		double* r = &prcv->r[3*k];

		//compute distances and incidence angles
		double dx[3]; //compute displacement vector between receiver and emitter
		dx[0] = xe[0] - xr[0];
		dx[1] = xe[1] - xr[1];
		dx[2] = xe[2] - xr[2];
		double d2 = dx[0]*dx[0] + dx[1]*dx[1] + dx[2]*dx[2]; //compute distance squared between receiver and emitter
		double d = sqrt(d2);
		double ct = dx[0]*r[0] + dx[1]*r[1] + dx[2]*r[2]; //compute cosine of the incidence angle between the emitter light ray and the receiver surface
		ct = ct/d; //take out the norm of the relative distance between receiver and emitter
		if(ct < ct_min) //do not consider orientations that are bigger than a certain threshold defined by the cosine of that angle
			return -1; //this measurement should not be considered

		//compute absorption coefficient
		double f = prcv->f[2*k +0] * ct + prcv->f[2*k + 1] * sqrt(ct);

		//compute predicted input light
		double s_i = f/d2;
		//if s_i is too big then the measurement will be invalid. s_i can only be smaller than zero if the incidence angle is bigger than 90 degrees. But this is covered in other functions

		//compute predicted output RSS resulting of passing the predicted input light to the amplification circuit
		double s_o; //the variable where the RSS will be stored
		int bncount = 0; //compute all branches that has passed for all the receievers until this one
		for(int m = 0; m < k; m++) bncount += prcv->bn[m];
		int l = 0,lMax = prcv->bn[k] + 1;
		for(; l < lMax; l++)
			if(s_i < prcv->s_it[(bncount + k) + l]) break; //chose amplification stage branch to use (pay attention to + k, since the transition is the number of branches + 1)
		l = l - 1; //if s_i is smaller than the curent s_it then the measurement belogs to the previous stage
		if((l >= 0) && (l < (lMax-1))) //if s_i is not smaller than zero, or if it is not too large
			s_o = prcv->g[3*(bncount + l) + 0] + prcv->g[3*(bncount + l) + 1]*s_i + prcv->g[3*(bncount + l) + 2]*s_i*s_i; //use the correct amplification stage brnch model to get the output RSS
		else
			s_o = -1; //in case s_i is invalid (see coment above on the if) s_o is marked as -1 and it should not be used

		//compute Jacobian if relevant (indexes of J singals which )
		if(s_o >= 0){
			double dfdct = prcv->f[2*k +0] + prcv->f[2*k +1]/sqrt(ct); //absorption coefficient derivatives
			double dgdsi = prcv->g[3*(bncount + l) + 1] + 2*prcv->g[3*(bncount + l) + 2]*s_i; //amplification stage model derivatives
			double G = dgdsi/(d*d*d);
			J[k*3 + 0] = G * ( r[0]*dfdct - (dx[0]/d)*(dfdct*ct + 2*f) ); //Jacobian equations for the three dimensions
			J[k*3 + 1] = G * ( r[1]*dfdct - (dx[1]/d)*(dfdct*ct + 2*f) );
			J[k*3 + 2] = G * ( r[2]*dfdct - (dx[2]/d)*(dfdct*ct + 2*f) );
		}

		//return the computed RSS
		return s_o;
	}
	else
		return -1; //return a negative RSS, signaling that this value is not to be used for estimation 

}

//function that computes the value + Jacobian for all receivers
void RaB3D_generate_RSS_multi(rcv_data* prcv, double* xe, double* rcv_predictions, double* J, char* rcv_mask, double ct_min){

	//go through the receivers
	for(int k = 0; k < prcv->rcvn; k++){

		//compute receiver k value (it already checks if receiver is ON, the value is big enough, and if the incidence angle is small enough)
		rcv_predictions[k] = RaB3D_generate_RSS(prcv, xe, J, k, ct_min);

		//define mask for receiver based on the previous result
		rcv_mask[k] = ((rcv_predictions[k] >= 0)?1:0);

	}
}

//function that computes the distance from the receiver k RSS value (assuming the incidence angle is prependicular to the receiver surface)
double RaB3D_find_range_from_RSS(rcv_data* prcv, int k, int s_o){

	//check if the receiver is ON
	if(prcv->ON[k] == 1){

		//if the signal power is too low, it is not well characterized by the RSS polynomial
		if( s_o > 10 ){

			//select polynomial branch where s_o fits, for sensor k
			int bncount = 0; //compute all branches that has passed for all the receievers until this receiver
			for(int m = 0; m < k; m++) bncount += prcv->bn[m];
			int l = 0,lMax = prcv->bn[k] + 1;
			for(; l < lMax; l++)
				if(s_o < prcv->s_ot[(bncount + k) + l]) break; //chose amplification stage branch to use (pay attention to + k, since the transition is the number of branches + 1)
			l = l - 1; //if s_i is smaller than the curent s_it then the measurement belogs to the previous stage
			
			//if s_o is not smaller than zero, or if it is not too large
			if((l >= 0) && (l < (lMax-1))){

				//optimized search method to find s_i matching s_o
				double low = prcv->s_it[l];
				double high = prcv->s_it[l + 1];
				double s_i = (low + high)/2;
				double s_ol;
				while(1){
					s_ol = prcv->g[3*(bncount + l) + 0] + prcv->g[3*(bncount + l) + 1]*s_i + prcv->g[3*(bncount + l) + 2]*s_i*s_i; //use the correct amplification stage brnch model to get the output RSS
					if( ABS(s_o - s_ol) < 0.5) break; //if the predicted value is close enough to the given value stop the search
					if( s_o > s_ol ){ //if selected s_i is smaller than the required
						low = s_i; //restrict search to the upper half of the previous interval
						s_i = (low + high)/2;
					}
					else{ //if selected s_i is greater than the required
						high = s_i; //restrict search to the lower half of the previous interval
						s_i = (low + high)/2;
					}
				}

				//return distance
				return 1/sqrt(s_i); //incidence angle is assumed perpendicular to the emitter suface (absorption coefficient is 1)

			}
			else
				return -1; //the value was too low or too high and the distance could not be computed (warn the user)
		}
		else
			return -2; //the value was too low and the distance could not be computed (warn the user)
	}
	else
		return -3; //the receiver was not ON (warn the user) - this error is issued because usually the receievr is OFF when its models are not initialized or its giving bad valus
}
