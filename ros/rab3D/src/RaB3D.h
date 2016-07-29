#ifndef RAB3D
#define RAB3D

/* === ROS includes === */
#include <quad_msgs/RaB3DInfo.h>
#include <quad_msgs/RaB3DValues.h>

//defines
#define MAX_ROBOTS 50
#define MAX_RECEIVERS 100
#define MAX_EMITTERS 20

//a is the structure, b is the emitter slot, and c is the the receiver ID
#define EMITTER_RSS(a, b, c) ((a)->s_o[(b)*((a)->rcvn) + (c)])

#define ABS(x) (((x)>0)?(x):-(x))

//structure that will contain the receivers for each robot
//each receiver will have emitn values, one for each emitter in the world. These values are ordered according to an emitter list
typedef struct t_rcv_data {
	quad_msgs::RaB3DInfo info_msg; //ros variable that stores the receiver calibration data
	quad_msgs::RaB3DValues values_msg; //ros variable that stores the receiver values for each emitter
	int rcvn; //number of receivers
	double* f, * g, * s_it, * s_ot, * x, * r; //parameters of the used models for each receiver
	char* bn; //parameters of the used models for each receiver
	int emitn; //number of detected emitters
	short int* code; //the detected emitter codes
	short int* slot; //time slot where emitters were detected
	short int* s_o; //receiver intensities.
	char* ON; //wether the receiver is ON
	char stamp_old; //stamp of the shared memory message
	//NOTE: vectors are fetched from the pointer at the respective values_msg and info_msgs. These messages need to be initialized with the right amount of data
} rcv_data;

//initializes the entire receiver structure from a received info meassage
rcv_data* RaB3D_initialize_from_message(const quad_msgs::RaB3DInfo::ConstPtr& info_msg);

//set receiver values and status from a received values message
void RaB3D_set_values_from_message(rcv_data* prcv, const quad_msgs::RaB3DValues::ConstPtr& values_msg);

//set receiver values and status from a shared memory chunck
int RaB3D_set_values_from_shared_memory(rcv_data* prcv, void* shm_pointer);

//finds the position in the receiver structure of the emitter matching the given code (-1 is returned if no such code is found on the list)
int RaB3D_find_emitter(rcv_data* prcv, int code);

//function that computes the receiver value + Jacobian (for receiver k)
double RaB3D_generate_RSS(rcv_data* prcv, double* xe, double* J, int k, double ct_mint);

//function that computes the value + Jacobian for all receivers
void RaB3D_generate_RSS_multi(rcv_data* prcv, double* xe, double* rcv_predictions, double* J, char* rcv_mask, double ct_min);

//function that computes the distance from the receiver k RSS value (assuming the incidence angle is prependicular to the receiver surface)
double RaB3D_find_range_from_RSS(rcv_data* prcv, int k, int s_o);

#endif
//RAB3D