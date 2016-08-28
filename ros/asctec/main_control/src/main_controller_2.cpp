//**** This node implements a controller for the Asctec Hummingbird ****//

#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Geometry>


// messages

#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <main_control/full_state.h>
#include <main_control/state_command.h>
#include <main_control/string_stamped.h>
#include <quad_msgs/Estimate.h>
#include <quad_msgs/EstimateMulti.h>
#include <quad_msgs/EstimateSingle.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <asctec_hl_comm/mav_status.h>
#include <mavros_msgs/AttitudeTarget.h>

// defines

#define GRAVITY 9.81
#define PI 3.14159
#define LOOP_RATE 20 // frequency of the main loop in Hz, should be > 10Hz
#define CORTEX_RATE 20 //frequency of cortex pose messages
#define ALPHA 0.8

#define BAT_CRIT_LEVEL 8.5
#define LOSE_THRESH_POS 1

// define types of operations
#define DOWN 0
#define TAKEOFF 1
#define HOME 2
#define NORMAL_OPERATION 3
#define BOUNDARY_REACHED 4
#define LANDING 5
#define WAIT_USER 6

#define BUF 2


using namespace std;
using namespace Eigen;

//**** define some structures for storing values ****//

struct Control_Coefs{
// coefficients for the control
	double Kp; 		// position
	double Kv; 		// velocity
	double Ka; 		// acceleration
	double Kyaw;	// yaw
	double offset_yaw;
	double KIz;		// integral on height
	double KIv;		// integral on velocity (vertical velocity is always zero)
	double Kg;		// gravity compensation. Can be set lower to 1 to go down
	double mass;

// limits of the errors
// -1 means no limit
	double Lph; 	// limits on horizontal position
	double Lpv; 	// limits on vertical position
	double Lv; 		// limits on velocity
	double La; 		// limits on acceleration
	double LIz; 	// limits on integral of height
	double Lyaw; 	// limits on yaw

// safety parameters
	double target_error;
	double takeoff_height;
	double safety_distance;
	double landing_height;
	double landing_speed;
	double turnoff_speed;

};

typedef struct Control_Coefs Control_Coefs;

// define a structure containing the current state of the quadrotor
// can also be used to define target

//quad state
typedef struct tState{
	Eigen::Vector3d pos;
	Eigen::Vector3d old_pos;
	double old_timer;
	Eigen::Vector3d speed;
	Eigen::Vector3d accel;
	Eigen::Vector3d angles;  // roll pitch yaw
	Eigen::Vector3d angles_rate;
	bool first_time;
	int pos_on, yaw_rate_on,relative_on;
} State;

//control state
typedef struct tController{
	Eigen::Vector3d ep;
	double eIz;
	Eigen::Vector3d ev;
	Eigen::Vector3d eIv;
	Eigen::Vector3d a_d;
	double eyaw;
	double eIyaw;
	double start_landing;
	int emergency_landing;
	double landing_time;
	double last_time;
	int action;
	int pos_on, yaw_rate_on,relative_on;
	int user_needed;
	double bias_thrust;
	double bias_roll;
	double bias_pitch;
	double bias_vyaw;
} Controller;

//boundary variables
typedef struct tBoundary{
	Eigen::Vector3d center;
	double x, y, z;
} Boundary;


// define some static variables that will be used over the whole file 
// this structure will contain all parameters for the control
static Control_Coefs coefs;
static Control_Coefs safe_coefs; // will contain safe coefs hard encoded in this node 

//vehicle and target states: position, velocity and acceleration
static State state;
static State target;
static State takeoff_target;
static State home_target;
static State safe_target;
static State wait_target;
static State landing_target;
static State aux_target;

//controller structure: PID errors plus desired acceleration
static Controller controller;
static Boundary boundary;
int issued_commands;
int measurements_received;
int tracking;
double timer_zof = 0.0, timer_mcs = 0.0;

//state machine variable
string current_state;

// timing
//static double last_time;
//static double elapsed;

//other variables
double cortex_rate;
double alpha;
bool verbose_mode;
int asctec;

//failure flag
bool lose_self_sim_9999 = 0;
bool lose_self_sim_value = 0;
bool bat_fail_sim = 0;

Eigen::Vector3d force;
double yaw_torque;
double relative;
double calib_slope, calib_bias;

string formation_list,this_quad;
char local_name[20];
int current_quad;

Eigen::Vector3d vel_ffwd(0,0,0);

//**** Define function names ****//
// compute virtual force using current state and target
void compute_force(Eigen::Vector3d & force, double &yaw_torque);

// compute command from virtual force
void force2command(asctec_hl_comm::mav_ctrl & command, mavros_msgs::AttitudeTarget& px4_command, Eigen::Vector3d & force, double yaw_torque);

// several callbacks for all subcribed topics
// treated state information from the motion capture system
void estimator_input_callback(const quad_msgs::EstimateSingle::ConstPtr& estimates_msg);

// input from the user including emergency stop, does not work to set waypoints
void user_input_callback(const std_msgs::String& new_message);

// command input from any navigation algorithm but can also be used to set a waypoint
void algorithm_input_callback(const main_control::state_command::ConstPtr& new_target);

// gather all control coefficients from rosparam
void initialize_controller(Control_Coefs & new_coefs, std::string param_name);

// input from fcu
void fcu_input_callback(const asctec_hl_comm::mav_status::ConstPtr& status);

//control bias to test main control when the applied force is zero (quadrotor down)
void bias_callback(const std_msgs::Float32MultiArray::ConstPtr& new_bias);

// for sending state messages
ros::Publisher state_pub;
void send_state(ros::Publisher& state_pub,string command);
void send_current_state(ros::Publisher& state_pub);
ros::Publisher notification_pub;


//for giving feedback of the desired control force
ros::Publisher feedback_pub;
void send_feedback(ros::Publisher& feedback_pub,Eigen::Vector3d & force,double integrator_force);

int main(int argc, char** argv){

	ros::init(argc, argv,"main_controller");

	ros::NodeHandle nh;
	ros::Rate loop_rate(LOOP_RATE);

	//load controller coeficients
	initialize_controller(coefs, "main_control/");

	//**** declare some variables ****//
	// declare the command message that will be sent to the fcu/control topic
	asctec_hl_comm::mav_ctrl command;
	mavros_msgs::AttitudeTarget px4_command;


	//setup publisher
	ros::Publisher control_pub;
	if(asctec)
		control_pub=nh.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1);
	else
		control_pub=nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",1);
	state_pub=nh.advertise<main_control::string_stamped>("main_control/state",1);
	feedback_pub=nh.advertise<main_control::state_command>("main_control/feedback",1);
	notification_pub=nh.advertise<std_msgs::String>("notification/channel",1);

	//setup subscribers
	ros::Subscriber motion_capture_input;
	ros::Subscriber estimator_input;
	ros::Subscriber fcu_input=nh.subscribe("fcu/status", 1, fcu_input_callback);
	estimator_input = nh.subscribe("estimator/estimate_self", 1, estimator_input_callback);
	ros::Subscriber user_input = nh.subscribe("user/command_out", 2, user_input_callback);
	ros::Subscriber algorithm_input = nh.subscribe("algorithm/command_out", 1, algorithm_input_callback);
	ros::Subscriber bias_input = nh.subscribe("user/bias_change", 1, bias_callback);

	//safe coefs
	//TODO verify that they are safe
	safe_coefs.Kp = 0.2;
	safe_coefs.Kv = 0.1;
	safe_coefs.Ka = 0.0;
	safe_coefs.Kyaw = 0.005;	
	safe_coefs.KIz = 0.0005;
	safe_coefs.Kg = 1.0;
	safe_coefs.Lph = 1;
	safe_coefs.Lpv = 0.5;
	safe_coefs.Lv = -1;
	safe_coefs.La = 0.1;
	safe_coefs.Lyaw = 0.1;	
	safe_coefs.LIz = 10;

	//run controller
	send_state(state_pub,"down");
	while(ros::ok())
	{
		//compute 3D force from desired position, speed, and acceleration
		compute_force(force, yaw_torque);

		//convert desired force to roll, pitch, thrust commands
		force2command(command, px4_command, force, yaw_torque);

		//send to the quadcopter
		if(asctec)
			control_pub.publish(command);
		else
			control_pub.publish(px4_command);

		//next cycle
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;

}

#define SGN(x) ((x)>0 ?1:-1)
#define NORM(x) (sqrt((x).dot(x)))
#define LIMIT_SCALAR(a,b) (a *= ( (abs(a)>b) ? (b/abs(a)):(1) ))

Eigen::VectorXd limit_vector(Eigen::VectorXd v, double l)
{
	if(v.dot(v) > l*l)
	{
		// normalize if too big
		double norm = sqrt(v.dot(v));
		v *= l/norm;
	}

	return v;
}

void compute_force(Eigen::Vector3d & force, double &yaw_torque)
{
	//update controller time step
	double elapsed = ros::Time::now().toSec() - controller.last_time;
	controller.last_time = controller.last_time + elapsed;

	//check which controller mode should be on
	int mcs_mode = ( (ros::Time::now().toSec() - timer_mcs) <= 0.5);
   	int zof_mode = ( (ros::Time::now().toSec() - timer_zof) <= 0.5);

	//check if receiving self localization data (absolute localization in the environment)
	//if not self localized, quadrotor performs emergency landing (except if it is already turned off)
	if( (mcs_mode == 0) && (zof_mode == 0) )
	{
		//tracking is considered stopped
		tracking = 0;

		//activate emergency landing, and turn off if not already turned off
		if(controller.action!=DOWN){
			controller.action = DOWN; //set quadrotor to down state (turned off)
			send_state(state_pub, "down"); //notify state change
			controller.emergency_landing = 1; //emergency landing activated

			//notifications to user
			std_msgs::String message;
			message.data = "lose_self detected";
			notification_pub.publish(message);

			//notify through console
			if(verbose_mode)
				cout << "losing self position: emergency landing" << endl;
		}
	}

	//issue a warning to the user if taking ore that usual
	if( (ros::Time::now().toSec() - timer_mcs > 0.2) && (ros::Time::now().toSec() - timer_zof > 0.2) ){
		cout << "main_control: measurement update is taking an unusual time to be performed." << endl;
	}

	//check if environment boundaries reached
	Eigen::Vector3d d = state.pos - boundary.center;
	int bcount = 0;
	aux_target = takeoff_target; //just to put zeros on the aux_target variables
	aux_target.pos = state.pos; //insert current position
	aux_target.accel << 0,0,0;
	if( (abs(d(0)) > boundary.x) && (mcs_mode) ) //test if inside x-axis limits
		{aux_target.pos(0) = boundary.center(0)+SGN(d(0))*(boundary.x-coefs.safety_distance); bcount++;} //create safety target
	if( (abs(d(1)) > boundary.y) && (mcs_mode) ) //test if inside y-axis limits
		{aux_target.pos(1) = boundary.center(1)+SGN(d(1))*(boundary.y-coefs.safety_distance); bcount++;} //create safety target
	if( (abs(d(2)) > boundary.z) && ((mcs_mode)||(zof_mode)) ) //test if inside z-axis limits
		{aux_target.pos(2) = boundary.center(2)+SGN(d(2))*(boundary.z-coefs.safety_distance);  bcount++;} //create safety target
	if(bcount>0) //if any of the axis limits were breached, activite boundery_reached
		{if(controller.action != BOUNDARY_REACHED) send_state(state_pub,"hit");
		 safe_target.pos=aux_target.pos; controller.action = BOUNDARY_REACHED; controller.user_needed=1;}

	//each control action means different control strategies
	//yaw control is decoupled from the 3D acceleration based controller
	//these commands do not depend on the control mode (they probably can be partailly done if the controller reached this stage)
	switch(controller.action)
	{

		case DOWN: {
			//when the vehicle goes to the down state, the controller sends it to zero roll/pitch, and the thrust gently decreases
			//this is how the emergency landing is performed
			if(verbose_mode) cout << "down!!!" <<endl;
			force(0) = 0;
			force(1) = 0;
			force(2) -= elapsed*coefs.turnoff_speed;
			force(2) = ((force(2)<0.0)?0.0:force(2));
			return;
		} break;

		case TAKEOFF: {

			//choose target to follow
			aux_target = takeoff_target;
			aux_target.pos(0) = state.pos(0);
			aux_target.pos(1) = state.pos(1);
			if(verbose_mode) cout << "taking off!!!" << endl;

			//check if end of takeoff conditions are met
			if(abs(state.pos(2) - aux_target.pos(2)) < coefs.target_error)
				{controller.action = HOME; send_state(state_pub,"home");}

		} break;

		case HOME: {

			//choose target to follow
			aux_target = home_target;
			if(mcs_mode == 0){ //if no mcs is being used, ivalidate horizontal positions
				aux_target.pos(0) = state.pos(0);
				aux_target.pos(1) = state.pos(1);
			}
			if(verbose_mode) cout << "going home!!!" <<endl;

			//if home has been achieved
			if(NORM(state.pos - aux_target.pos) < coefs.target_error)
			{
				if(controller.user_needed)
					{controller.action = WAIT_USER; send_state(state_pub,"wait");} //wait untill user gives order
				else
					{controller.action = NORMAL_OPERATION; send_state(state_pub,"operating");} //start or resume
				wait_target.pos=state.pos;
				issued_commands=0;
			}

		} break;

		case WAIT_USER: {

			//choose target to follow
			aux_target=wait_target;
			if(mcs_mode == 0){ //if no mcs is being used, ivalidate horizontal positions
				aux_target.pos(0) = state.pos(0);
				aux_target.pos(1) = state.pos(1);
			}
			aux_target.pos_on=1; //BUG QUAD FALLS AFTER FORMATION IS STOPPED
			if(verbose_mode) cout << "wating for user!!!" << aux_target.pos << endl;

			//if user has authorised normal operations
			if(!controller.user_needed)
				{controller.action=NORMAL_OPERATION; send_state(state_pub,"operating");}

		} break;

		case NORMAL_OPERATION: {

			//wait until commands were issued
			if(issued_commands)
				aux_target=target;
			else
				aux_target=wait_target;
			if(verbose_mode) 
			{				
				cout << "running normally!!!" <<endl;
				cout << "target: " << endl << aux_target.pos << endl << aux_target.speed << endl;
			}

		} break;

		//TODO
		case BOUNDARY_REACHED: {

			//choose target to follow
			aux_target=safe_target;
			if(mcs_mode == 0){ //if no mcs is being used, ivalidate horizontal positions
				aux_target.pos(0) = state.pos(0);
				aux_target.pos(1) = state.pos(1);
			}
			if(verbose_mode) cout << "boundary reached!!!" <<endl;

			//check if end of boundary reach conditions are met
			if(mcs_mode) //only in mcs_mode this can be done (which is OK because when we reach here we probably are in mcs)
				if(NORM(state.pos-aux_target.pos) < coefs.target_error)
					{controller.action = HOME; send_state(state_pub,"home");}

		} break;

		case LANDING: {

			//initialize landing
			if(controller.start_landing)
			{
				controller.landing_time = controller.last_time;
				controller.start_landing=0;
				landing_target.pos=state.pos;
			}

			//control landing trajectory
			else if(landing_target.pos(2) > coefs.landing_height)
				landing_target.pos(2)-=elapsed*coefs.landing_speed;
			else {
				landing_target.pos(2)=0;
				{controller.action = DOWN; send_state(state_pub,"down");}
			}

			//choose target to follow
			aux_target=landing_target;
			if(verbose_mode) cout << "landing!!!" <<endl;

		} break;

		default: {
			
			//no force should be applied
			return;

		}
	}
	//cout << "state: " << endl << state.pos << endl << "target: " << endl << aux_target.pos << endl << aux_target.speed << endl << aux_target.speed << endl;

	Eigen::Matrix3d R;
	double angle = state.angles(2);
    R <<  cos(angle),   -sin(angle),  	0,
          sin(angle),  	cos(angle),  	0, 
          0,            0,             	1;    	

	// predict the velocity caused by the formation control acceleration command
	// integrate force and divide by mass
	vel_ffwd += 1.0*coefs.Ka*R*controller.a_d*elapsed;
	vel_ffwd(2) = 0;

	//compute errors
	controller.ep = state.pos - aux_target.pos;
	// velocity error = (actual velocity - desired velocity)
	// note that this error is multiplied by (-) later
	// actual group velocity = actual velocity - fedforward formation velocity
	// commanded velocity in body frame is rotated into absolute frame
	//controller.ev = ((state.speed - 0.0*vel_ffwd) - R*aux_target.speed);
	controller.ev = ((state.speed - 0.0*vel_ffwd) - aux_target.speed);
	controller.a_d = aux_target.accel;
	controller.pos_on = aux_target.pos_on;
	controller.yaw_rate_on = aux_target.yaw_rate_on;
	controller.relative_on = aux_target.relative_on;
	if(mcs_mode == 0) //when mcs is not on, then there is no absolute position, then we need to rely on relative measurements
		controller.relative_on = 1;

	//update integrators
	if(controller.pos_on == 1) controller.eIz += 0*(controller.ep(2)) * elapsed; //integrator is not ON (multiplication by zero)
	//if(controller.pos_on > 1){
		controller.eIv += controller.ev * elapsed; //integratior only working when in formation control
		controller.eIv(2) = 0; //there is no height integrator
	//}

	//compute error between current and desired yaw
	controller.eyaw = state.angles(2) - (aux_target.angles(2) + coefs.offset_yaw);


	if(verbose_mode)
	{
		cout << "state_angle: " << state.angles(2) << " and target_angle: " << aux_target.angles(2) <<endl;
		cout << "eyaw: " << controller.eyaw << endl;
	}

	while(controller.eyaw > PI)
		controller.eyaw -= 2*PI;
	while(controller.eyaw < -PI)
		controller.eyaw += 2*PI;

	controller.eIyaw += (controller.eyaw)*elapsed;

	if(verbose_mode) 
	{
		cout << "eyaw2: " << controller.eyaw << endl;
		cout << "debug time!!\n" << controller.pos_on << endl;

		cout << "position error\n" << controller.ep << "speed error\n" << controller.ev << "eIz\n" << controller.eIz << "a_d\n" << controller.a_d << endl;
		cout << "position \n" << state.pos << "speed \n" << state.speed << endl;
		cout << "position target \n" << target.pos << "speed target\n" << target.speed << endl;
	}

	//yaw
	LIMIT_SCALAR(controller.eyaw,coefs.Lyaw);

	//adjust the controller to respect its mode
	if(mcs_mode == 0) //there is no horizontal position if there is no mcs_mode
		{controller.ep(0) = 0; controller.ep(1) = 0;}

	//reset force
	Eigen::Vector3d z; z << 0, 0, 1; // Z axis
	force << 0,0,0;

	//desired force depending on the chosen method
	//control based on position and velocity error
	if(controller.pos_on == 1){
		force = -coefs.Kp*controller.ep - coefs.Kv*controller.ev - coefs.KIz*controller.eIz*z;
		force += -coefs.KIv*controller.eIv; //NOTE: the height integratior should be turned off
		//cout << -coefs.KIv*controller.eIv << endl;
	}
	//control based on velocity error (followers in teleoperation case)
	else if(controller.pos_on == 2){
		force = -0.5*controller.ev;
		force += -coefs.KIv*controller.eIv;
	}
	//control based on velocity error, but absolute z height also controlled (leader in teleoperation case, otherwise leader falls down)
	else if(controller.pos_on == 3) {
		force = -0.5*controller.ev;
		force += -coefs.KIv*controller.eIv;
		force(2) += -coefs.Kp*controller.ep(2) - coefs.KIz*controller.eIz*z(2);
	}

	//add feedforward terms or forces that were computed in external nodes
	force *= coefs.mass;
	force += coefs.mass*coefs.Ka*controller.a_d;

	//adding gravity and limiting the force amplitude
	force += coefs.mass*GRAVITY*z;
	Eigen::Vector2d forceh;
	forceh << force(0),force(1);
	forceh = limit_vector(forceh,coefs.Lph*coefs.Kp);
	force(0) = forceh(0);
	force(1) = forceh(1);
	//LIMIT_SCALAR(force(2),(coefs.mass*GRAVITY+coefs.La));

	//yaw controller (NOTE: px4 commands give yaw bias to achieve - this should imply a change in yaw gains)
	if(controller.yaw_rate_on)
		yaw_torque = target.angles_rate(2);
	else
		yaw_torque = -coefs.Kyaw*controller.eyaw;

	//are the computed forces to be interpreted in the body frame or in the absolute frame?
	relative = controller.relative_on;

	//send feedack
	send_feedback(feedback_pub,force,coefs.KIz*controller.eIz);
}

void force2command(asctec_hl_comm::mav_ctrl & command, mavros_msgs::AttitudeTarget& px4_command, Eigen::Vector3d & force, double yaw_torque)
{
	//transform desired acceleration from "world" to "body" frame. Uses the yaw in the "world" frame
	Eigen::Matrix3d R;
	double angle = state.angles(2);
	R <<  cos(angle),  sin(angle),  0,
             -sin(angle),  cos(angle),  0,
                  0,            0,      1;

	Eigen::Vector3d body_force;
	if(!relative) //only transform if the desired quantities are not already in the "body" frame
		body_force = R*force;
	else
		body_force = force;

	cout << "---------------------" << endl;
        cout << force << endl;

	//generate asctec command if asctec controller is on
	if(asctec == 1){

		//convert acceleration in the "body" frame into roll, pitch, thrust command + consider vyaw command
		double thrust = (body_force(2)/(cos(state.angles(0) + state.accel(0))*cos(state.angles(1) + state.accel(1)))) - state.accel(2)*coefs.mass + controller.bias_thrust;
		double roll = (body_force.dot(body_force) != 0 ? -atan2(body_force(1), thrust):0 ) - state.accel(0) + controller.bias_roll;
		double pitch = (body_force.dot(body_force) != 0 ? atan2(body_force(0), thrust):0 ) - state.accel(1) + controller.bias_pitch;
		yaw_torque += controller.bias_vyaw;

		//send the commnad to the auto-pilot
		command.type = asctec_hl_comm::mav_ctrl::acceleration;
		command.x = (-1)*pitch; //pitch will probably have to change
		command.y = (-1)*roll; //roll as computed before should be negative here to make sense
		command.z = calib_slope*thrust + calib_bias;
		command.yaw = yaw_torque;
	}
	//generate asctec command if px4 controller is on
	else{
		//convert yaw control into a ficticius bias yaw to follow (yaw control gains probably need to change here)
		//convert desired acceleration into the "new body" frame including this yaw bias
		//body_force(0) += controller.bias_pitch;
		//body_force(1) -= controller.bias_roll;
		double d_yaw = yaw_torque + controller.bias_vyaw;
		R <<  cos(d_yaw),  sin(d_yaw),  0,
             	     -sin(d_yaw),  cos(d_yaw),  0,
                          0,            0,      1;
		body_force = R*body_force; //this desired acceleration is now in the "new body" frame

		//convert acceleration in the "new body" frame into roll, pitch, yaw, thrust command
                double thrust = (body_force(2)/(cos(state.angles(0) + state.accel(0))*cos(state.angles(1) + state.accel(1)))) - state.accel(2)*coefs.mass + controller.bias_thrust;
		double roll = (body_force.dot(body_force) != 0 ? -atan2(body_force(1), thrust):0 ) - state.accel(0) + controller.bias_roll;
		double pitch = (body_force.dot(body_force) != 0 ? atan2(body_force(0), thrust):0 ) - state.accel(1) + controller.bias_pitch;

		//convert attitude command to a quaternion command
		Eigen::Quaternion<double> q_roll = Eigen::Quaternion<double>(cos(roll/2), sin(roll/2), 0, 0);
		Eigen::Quaternion<double> q_pitch = Eigen::Quaternion<double>(cos(pitch/2), 0, sin(pitch/2), 0);
		Eigen::Quaternion<double> q_control = q_pitch * q_roll; //yaw is always zero

		//compute attitude command to the "earth" ENU frame (using the XYZ rotation order)
		Eigen::Quaternion<double> q_yaw = Eigen::Quaternion<double>(cos( (d_yaw + state.angles(2))/2 ), 0, 0, sin( (d_yaw + state.angles(2))/2 ));
		q_control = q_yaw * q_control;

		//send command to auto-pilot
		px4_command.orientation.w = q_control.w();
		px4_command.orientation.x = q_control.x();
		px4_command.orientation.y = q_control.y();
		px4_command.orientation.z = q_control.z();
		px4_command.thrust = calib_slope*thrust + calib_bias; //convert predicted thrust to px4 commands
		if(px4_command.thrust < 0) px4_command.thrust = 0; //do not accept negative thrust
	}

}

void estimator_input_callback(const quad_msgs::EstimateSingle::ConstPtr& estimates_msg)
{

	//seperate strings to see which sensors were used to update
	stringstream stream(estimates_msg->estimate.sensors.data);
	vector<string> words;
	string word;
    while( getline(stream, word, ':') )
    	words.push_back(word);

    //activate control modes based on the sensors that updated the self state
    int zof_flags = 0; //since optic flow and height are different sensors needed for the same control mode, both their presence are required to activate the mode
   	int mcs_mode = 0;
   	int zof_mode = 0;
   	for(int k = 0; k < words.size(); k++){
   		if( strcmp(words[k].c_str(), "mcs") == 0 ) //activate mcs_mode
        	mcs_mode = 1;
        if( strcmp(words[k].c_str(), "z") == 0 )
        	zof_flags |= 1; //one sensor is present for the zof_mode
       	if( strcmp(words[k].c_str(), "of") == 0 )
       		zof_flags |= 2; //another sensor is present for the zof_mode
       	if( zof_flags == 3 ) //check if both sensors are present for the zof_mode
       		zof_mode = 1;
   	}

	//TODO: convert asctec commands to px4 commands

	//activating control modes (if possible)
	if( mcs_mode == 1 )
		{timer_mcs = ros::Time::now().toSec(); measurements_received=1;}
	if( zof_mode == 1 )
		{timer_zof = ros::Time::now().toSec(); measurements_received=1;}

	//extract estimation quantities considering the activated control modes and the sensors used to updated the estimates

	//extracting position
	if( mcs_mode ){
		state.pos(0) = estimates_msg->estimate.position.x;
		state.pos(1) = estimates_msg->estimate.position.y;
	}
	if( (mcs_mode) || (zof_mode) )
		state.pos(2) = estimates_msg->estimate.position.z;

	//extracting velocity
	if( (mcs_mode) || (zof_mode) ){
		state.speed(0) = estimates_msg->estimate.velocity.x;
		state.speed(1) = estimates_msg->estimate.velocity.y;
		state.speed(2) = estimates_msg->estimate.velocity.z;
	}

	//extracting perturbation
	if( (mcs_mode) || (zof_mode) ){
		state.accel(0) = 0*estimates_msg->estimate.perturbation.x;
		state.accel(1) = 0*estimates_msg->estimate.perturbation.y;
		state.accel(2) = estimates_msg->estimate.perturbation.z;
	}

	//extracting object orientation (important just if the vehicle is not on relative coordinates)
	double x = estimates_msg->estimate.orientation.x;
	double y = estimates_msg->estimate.orientation.y;
	double z = estimates_msg->estimate.orientation.z;
	double w = estimates_msg->estimate.orientation.w;

	//quaternions to angles
	state.angles(0) = atan2(2*(w*x + z*y),(1 - 2*(x*x + y*y)));
	state.angles(1) = asin(2*(w*y - x*z));
	state.angles(2) = atan2(2*(w*z + x*y),(1 - 2*(y*y + z*z)));

}

void user_input_callback(const std_msgs::String& obj)
{
	if(!measurements_received)
		{cout << "cortex is down" << endl; controller.action=DOWN; return;}

	stringstream ss; ss << obj.data;
	string command_name; ss >> command_name;

	//state machine management
	if(!strcmp(command_name.c_str(),"takeoff"))
	{
		switch(controller.action){
			case DOWN: {controller.action=TAKEOFF; send_state(state_pub,"takeoff"); controller.user_needed=1;} break;
		}
	}
	else if(!strcmp(command_name.c_str(),"operate"))
	{
		switch(controller.action){
			case DOWN: {controller.action=TAKEOFF; send_state(state_pub,"operate"); controller.user_needed=0;} break;
			case WAIT_USER: controller.user_needed=0; break;
		}
	}
	else if(!strcmp(command_name.c_str(),"home"))
	{
		switch(controller.action){
			case NORMAL_OPERATION:
			case WAIT_USER: {controller.action=HOME; send_state(state_pub,"goinghome"); controller.user_needed=1;} break;
		}
	}
	else if(!strcmp(command_name.c_str(),"stop"))
	{
		switch(controller.action){
			case WAIT_USER: 
			case NORMAL_OPERATION: {controller.action=WAIT_USER; send_state(state_pub,"stop"); controller.user_needed=1;
			                        wait_target.pos=state.pos; target = wait_target; } break; //last command is to reset the target for subsquent operate messages
		}
	}
	else if(!strcmp(command_name.c_str(),"land"))
	{
		switch(controller.action){
			case WAIT_USER: {controller.action=LANDING; send_state(state_pub,"land"); controller.start_landing=1;} break;
		}
	}
	else if(!strcmp(command_name.c_str(),"state"))
	{
		send_current_state(state_pub); 
	}
	else if(!strcmp(command_name.c_str(),"reset"))
	{
		controller.action = DOWN; //change quarotor state
		controller.eIz = 0; //reset internal control integrator if it is being used
		force<<0,0,0; //no force is being issued to the quadrotor
		send_state(state_pub,"down"); //notify current quadrotor state

	}

	//quadrotor movement orders
	else if(!strcmp(command_name.c_str(),"goto"))
	{

		//check if quadrotor is on the right state
		switch(controller.action){

			//NORMAL_OPERATION is the only state where this command can be triggered
			case NORMAL_OPERATION:
			{
				//zero the velocity target
				memset(&target,0,sizeof(State));

				//set desired velocities (no position, acceleration, or angles/angle rates given)
				ss >> target.pos(0);
				ss >> target.pos(1);
				ss >> target.pos(2);

				//status information
				target.pos_on = 1; //what type of controller (check function compute_force) -> 1 = position/velocity controller
				target.relative_on = 0; //is the target pos/velocity/acceleration expressed in the quadrotor frame or not (absolute reference frame)
				target.yaw_rate_on = 0; //should yaw rate be controlled or not (no control of yaw rate)
				
				//the controller has a command to follow (in this case from the user)
				issued_commands = 1;
	
				break;
			}
		}
	}

	//quadrotor velocity orders
	else if(!strcmp(command_name.c_str(),"vgoto")){

		//check if quadrotor is on the right state
		switch(controller.action){

			//NORMAL_OPERATION is the only state where this command can be triggered
			case NORMAL_OPERATION:
			{

				//zero the velocity target
				memset(&target,0,sizeof(State));

				//set desired velocities (no position, acceleration, or angles/angle rates given)
				ss >> target.speed(0);
				ss >> target.speed(1);
				ss >> target.speed(2);

				//height target is the same has the initial state
				target.pos(2) = state.pos(2);

				//status information
				target.pos_on = 1; //what type of controller (check function compute_force) -> 1 = position/velocity controller
				target.relative_on = 1; //is the target pos/velocity/acceleration expressed in the quadrotor frame or not (absolute reference frame)
				target.yaw_rate_on = 0; //should yaw rate be controlled or not (no control of yaw rate)

				//the controller has a command to follow (in this case from the user)
				issued_commands = 1;
	
				break;
			}
		}
	}

	else if(!strcmp(command_name.c_str(),"set_yaw"))
	{
		//read yaw value to insert
		double new_yaw; ss >> new_yaw;

		//only change this value if quadrotor is running
		if(controller.action == NORMAL_OPERATION)
		{
			target.angles(2) = PI*new_yaw/180; //convert angle to radians (it is issued in degrees)
			issued_commands = 1;
		}

		printf("new target yaw %f \n", target.angles(2));
	}
	
	//configuration parameters
	else if(!strcmp(command_name.c_str(),"set_home"))
	{
		//get new home position
		double x; ss >> x;
		double y; ss >> y;
		double z; ss >> z;

		//update home position target if quadrotor is not executing the maneuver
		if(controller.action != HOME)
			home_target.pos << x, y, z;
	}
	else if(!strcmp(command_name.c_str(),"bat_fail_sim"))
	{
		bat_fail_sim=1; //simulating battery failure
		std_msgs::String message; //notify user
		message.data="battery_low begin";
		notification_pub.publish(message);
		cout<<"bat_sim_started" << endl; //consol message
	}
	else if(!strcmp(command_name.c_str(),"bat_fail_stop"))
	{
		bat_fail_sim=0; //stop battery simulation failure
	}
	else {return;}
}

void fcu_input_callback(const asctec_hl_comm::mav_status::ConstPtr& status)
{
    double bat_level=0;
    
    //take battery level from fcu
	bat_level=status->battery_voltage; 

    //generate fail if asked to simulate it
    if(bat_fail_sim==1)
        bat_level=2;
    
    //test
    if(bat_level <= 8.5 && (controller.action!=LANDING) && (controller.action!=DOWN))
	{ 
		controller.action=LANDING;
		send_state(state_pub,"land");
		controller.start_landing=1;

		//notifications
		std_msgs::String message;
		message.data="battery_low detected";
		notification_pub.publish(message);

		//console
		cout<<"not enough battery: landing" <<endl;
	}
}

void algorithm_input_callback(const main_control::state_command::ConstPtr& new_target)
{
	// TODO this function should also check that the target is in the boundaries
	// if not it should sent a warning message

	//store message in target structure
	//position commands
	target.pos(0) = new_target->position.x;
	target.pos(1) = new_target->position.y;
	target.pos(2) = new_target->position.z;

	//velocity commands
	target.speed(0) = new_target->velocity.x;
	target.speed(1) = new_target->velocity.y;
	target.speed(2) = new_target->velocity.z;

	//acceleration commands
	target.accel(0) = new_target->acceleration.x;
	target.accel(1) = new_target->acceleration.y;
	target.accel(2) = new_target->acceleration.z;

	//define target yaw and yaw rate
	//NOTE: yaw can only be controlled if it is being given by the estimayion layer
	target.angles(2) = new_target->yaw;
	target.angles_rate(2) = new_target->yaw_rate;

	//status information
	target.pos_on = new_target->pos_on; //what type of controller (check function compute_force)
	target.relative_on = new_target->relative_on; //is the target pos/velocity/acceleration expressed in the quadrotor frame or not
	target.yaw_rate_on = new_target->yaw_rate_on; //should yaw rate be controlled or not
	
	//the controller has a command to follow (in this case from the algorithm layer)
	issued_commands = 1;
}

//control bias to test main control when the applied force is zero (quadrotor down)
void bias_callback(const std_msgs::Float32MultiArray::ConstPtr& new_bias){
	controller.bias_thrust = new_bias->data[0];
	controller.bias_roll   = new_bias->data[1];
	controller.bias_pitch  = new_bias->data[2];
	controller.bias_vyaw   = new_bias->data[3];
}

void initialize_controller(Control_Coefs & new_coefs, std::string param_name)
{
	//get ros node handle
	ros::NodeHandle nh; // is the same handle as in the main, seems ros is global
	std::string full_name;

	//start getting parameters
	full_name = param_name + "mass";
	nh.getParam(full_name.c_str(), new_coefs.mass);
	full_name = param_name + "Kp";
	nh.getParam(full_name.c_str(), new_coefs.Kp);
	printf("test2 %f \n", new_coefs.Kp);
	full_name = param_name + "Kv";
	nh.getParam(full_name.c_str(), new_coefs.Kv);
	full_name = param_name + "Ka";
	nh.getParam(full_name.c_str(), new_coefs.Ka);
	full_name = param_name + "Kyaw";
	nh.getParam(full_name.c_str(), new_coefs.Kyaw);
	full_name = param_name + "OFFSETyaw";
	nh.getParam(full_name.c_str(), new_coefs.offset_yaw);
	full_name = param_name + "KIz";
	nh.getParam(full_name.c_str(), new_coefs.KIz);
	full_name = param_name + "KIv";
	nh.getParam(full_name.c_str(), new_coefs.KIv);
	full_name = param_name + "Lph";
	nh.getParam(full_name.c_str(), new_coefs.Lph);
	full_name = param_name + "Lpv";
	nh.getParam(full_name.c_str(), new_coefs.Lpv);
	full_name = param_name + "Lv";
	nh.getParam(full_name.c_str(), new_coefs.Lv);
	full_name = param_name + "La";
	nh.getParam(full_name.c_str(), new_coefs.La);
	full_name = param_name + "Lyaw";
	nh.getParam(full_name.c_str(), new_coefs.Lyaw);
	full_name = param_name + "LIz";
	nh.getParam(full_name.c_str(), new_coefs.LIz);
	full_name = param_name + "emax";
	nh.getParam(full_name.c_str(), new_coefs.target_error);
	full_name = param_name + "OFFz";
	nh.getParam(full_name.c_str(), new_coefs.takeoff_height);
	full_name = param_name + "SAFEd";
	nh.getParam(full_name.c_str(), new_coefs.safety_distance);
	full_name = param_name + "LANDz";
	nh.getParam(full_name.c_str(), new_coefs.landing_height);
	full_name = param_name + "LANDv";
	nh.getParam(full_name.c_str(), new_coefs.landing_speed);
	full_name = param_name + "TurnOffv";
	nh.getParam(full_name.c_str(), new_coefs.turnoff_speed);
	full_name = param_name + "formation_list";
	nh.getParam(full_name.c_str(),formation_list);
	full_name = param_name + "this_quad";
	nh.getParam(full_name.c_str(),this_quad);

	//get current quadrotor
	char* pc=&formation_list[0];
	current_quad=0;
	while(*pc != 0)
	{
		sscanf(pc,"%s",local_name);
		if(!strcmp(this_quad.c_str(),local_name))
			break;
		while(*pc==' ') pc++;
		while((*pc!=' ') && (*pc != '\0')) pc++;
        current_quad++;
	}
	cout << current_quad << endl;

	//initialize takeoff target
	takeoff_target.pos << 0,0,new_coefs.takeoff_height;
	takeoff_target.old_pos << 0,0,0;
	takeoff_target.old_timer=0;
	takeoff_target.speed << 0,0,0;
	takeoff_target.accel << 0,0,0;
	takeoff_target.angles << 0,0,0;
	takeoff_target.first_time=1;
	takeoff_target.pos_on=1;
	takeoff_target.angles_rate << 0,0,0;
	takeoff_target.yaw_rate_on=0;
	takeoff_target.relative_on=0;

	//initialize home position
	double x, y, z;
	full_name = param_name + "homex";
	nh.getParam(full_name.c_str(), x);
	full_name = param_name + "homey";
	nh.getParam(full_name.c_str(), y);
	full_name = param_name + "homez";
	nh.getParam(full_name.c_str(), z);
	home_target.pos << x, y, z;
	home_target.old_pos << 0,0,0;
	home_target.old_timer=0;
	home_target.speed << 0,0,0;
	home_target.accel << 0,0,0;
	home_target.angles << 0,0,0;
	home_target.first_time=1;
	home_target.pos_on=1;
	home_target.angles_rate << 0,0,0;
	home_target.yaw_rate_on=0;
	home_target.relative_on=0;

	//initialize safe target
	safe_target.pos << 0,0,0;
	safe_target.old_pos << 0,0,0;
	safe_target.old_timer=0;
	safe_target.speed << 0,0,0;
	safe_target.accel << 0,0,0;
	safe_target.angles << 0,0,0;
	safe_target.first_time=1;
	safe_target.pos_on=1;
	safe_target.angles_rate << 0,0,0;
	safe_target.yaw_rate_on=0;
	safe_target.relative_on=0;

	//initialize stop target
	wait_target.pos << 0,0,0;
	wait_target.old_pos << 0,0,0;
	wait_target.old_timer=0;
	wait_target.speed << 0,0,0;
	wait_target.accel << 0,0,0;
	wait_target.angles << 0,0,0;
	wait_target.first_time=1;
	wait_target.pos_on=1;
	wait_target.angles_rate << 0,0,0;
	wait_target.yaw_rate_on=0;
	wait_target.relative_on=0;

	//initialize landing target
	landing_target.pos << 0,0,0;
	landing_target.old_pos << 0,0,0;
	landing_target.old_timer=0;
	landing_target.speed << 0,0,0;
	landing_target.accel << 0,0,0;
	landing_target.angles << 0,0,0;
	landing_target.first_time=1;
	landing_target.pos_on=1;
	landing_target.angles_rate << 0,0,0;
	landing_target.yaw_rate_on=0;
	landing_target.relative_on=0;

	//initialize aux target
	aux_target.pos << 0,0,0;
	aux_target.old_pos << 0,0,0;
	aux_target.old_timer = 0;
	aux_target.speed << 0,0,0;
	aux_target.accel << 0,0,0;
	aux_target.angles << 0,0,0;
	aux_target.first_time=1;
	aux_target.pos_on = 1;
	aux_target.angles_rate << 0,0,0;
	aux_target.yaw_rate_on=0;
	aux_target.relative_on=0;

	//initialize target
	target.pos << 0,0,0;
	target.old_pos << 0,0,0;
	target.old_timer=0;
	target.speed << 0,0,0;
	target.accel << 0,0,0;
	target.angles << 0,0,0;
	target.first_time=1;
	target.pos_on=1;
	target.angles_rate << 0,0,0;
	target.yaw_rate_on=0;
	target.relative_on=0;

	//initialize boundary
	full_name = param_name + "envx";
	nh.getParam(full_name.c_str(), x);
	full_name = param_name + "envy";
	nh.getParam(full_name.c_str(), y);
	full_name = param_name + "envz";
	nh.getParam(full_name.c_str(), z);
	boundary.center << x,y,z;
	full_name = param_name + "boundx";
	nh.getParam(full_name.c_str(), boundary.x);
	full_name = param_name + "boundy";
	nh.getParam(full_name.c_str(), boundary.y);
	full_name = param_name + "boundz";
	nh.getParam(full_name.c_str(), boundary.z);

	//controller initializations
	controller.action = DOWN;
	controller.last_time = ros::Time::now().toSec();
	controller.eIz = 0;
	controller.eIv << 0,0,0;
	controller.eIyaw = 0;
	controller.pos_on = 1;
	//controller.pos_on = 2;
	controller.user_needed = 0;
	controller.emergency_landing = 0;
	controller.bias_thrust = 0;
	controller.bias_roll   = 0;
	controller.bias_pitch  = 0;
	controller.bias_vyaw   = 0;
	issued_commands=0;
	measurements_received=0;
	tracking=0;

	full_name = param_name + "cortex_rate";
	nh.getParam(full_name.c_str(), cortex_rate);
	full_name = param_name + "alpha";
	nh.getParam(full_name.c_str(), alpha);

	full_name = param_name + "verbose";
	nh.getParam(full_name.c_str(), verbose_mode);

	//selecting auto-pilot
	full_name = param_name + "asctec";
	nh.getParam(full_name.c_str(), asctec);
	if(asctec == 0) //in case of a PX4 platform , the yaw control is done internally, we just give a gain close to 1 
		new_coefs.Kyaw = 0.8;

	//selecting different thrust calibration curves according to the selected auto-pilot
	if(asctec == 0){
		full_name = param_name + "px4_calib_slope";
		nh.getParam(full_name.c_str(),calib_slope);
		full_name = param_name + "px4_calib_bias";
		nh.getParam(full_name.c_str(),calib_bias);
	}
	else {
		full_name = param_name + "asctec_calib_slope";
		nh.getParam(full_name.c_str(),calib_slope);
		full_name = param_name + "asctec_calib_bias";
		nh.getParam(full_name.c_str(),calib_bias);
	}

}

void send_state(ros::Publisher& state_pub,string command)
{
	main_control::string_stamped msg;

	current_state=command;
	msg.data = command;
	printf("here %s\n",command.c_str());
	state_pub.publish(msg);
}

void send_current_state(ros::Publisher& state_pub)
{
	main_control::string_stamped msg;

	msg.data = current_state;
	printf("here %s\n",current_state.c_str());
	state_pub.publish(msg);
}

void send_feedback(ros::Publisher& feedback_pub,Eigen::Vector3d & force,double integrator_force)
{
	main_control::state_command f;

	f.header.stamp = ros::Time::now();
	f.position.x = 0;
	f.position.y = 0;
	f.position.z = integrator_force;
	f.velocity.x = 0;
	f.velocity.y = 0;
	f.velocity.z = 0;
	f.acceleration.x = force(0)/coefs.mass;
	f.acceleration.y = force(1)/coefs.mass;
	f.acceleration.z = force(2)/coefs.mass;
	f.yaw = 0;

	f.pos_on = target.pos_on;

	feedback_pub.publish(f);
}
