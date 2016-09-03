//**** This node implements a controller for the Asctec Hummingbird ****//

#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <cmath>
//#include <time.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


// messages

#include <std_msgs/String.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <ssf_core/ext_state.h>
#include <main_control/full_state.h>
#include <main_control/state_command.h>
#include <main_control/string_stamped.h>
#include <internal_ekf/Estimate.h>
#include <internal_ekf/EstimateMulti.h>
#include <internal_ekf/EstimateSingle.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <asctec_hl_comm/mav_status.h>

// defines

#define GRAVITY 9.81
#define PI 3.14159
#define THRUST_SCALE 0.07170 // Scaling between newtons calulated here and what needs to be send
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
	double Kg;		// gravity compensation. Can be set lower to 1 to go down
	double mass;
	double allowf;

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
double timer;

//state machine variable
string current_state;

// timing
//static double last_time;
//static double elapsed;

//other variables
double cortex_rate;
double alpha;
bool verbose_mode;

//failure flag
bool lose_self = 0;
bool lose_self_sim_9999 = 0;
bool lose_self_sim_value = 0;
bool bat_fail_sim = 0;

Eigen::Vector3d force;
double yaw_torque;
double relative;

string formation_list,this_quad;
char local_name[20];
int current_quad;

double last_command[3] = {0,0,0};

//**** Define function names ****//
// compute virtual force using current state and target
void compute_force(Eigen::Vector3d & force, double &yaw_torque);

// compute command from virtual force
void force2command(asctec_hl_comm::mav_ctrl & command, Eigen::Vector3d & force, double yaw_torque);

// several callbacks for all subcribed topics
// state information from the ethz sensor fusion package and should be used for GPS positionning 
void ethz_kalman_input_callback(const ssf_core::ext_state::ConstPtr& new_state);

// state information from the motion capture system
void motion_analysis_input_callback(const geometry_msgs::PoseStamped::ConstPtr& new_state);
void motion_analysis_input_callback_void(const geometry_msgs::PoseStamped::ConstPtr& new_state);

// treated state information from the motion capture system
void internal_ekf_input_callback_void(const internal_ekf::EstimateSingle::ConstPtr& estimates_msg);
void internal_ekf_input_callback(const internal_ekf::EstimateSingle::ConstPtr& estimates_msg);

// input from the user including emergency stop, does not work to set waypoints
void user_input_callback(const std_msgs::String& new_message);

// command input from any navigation algorithm but can also be used to set a waypoint
void algorithm_input_callback(const main_control::state_command::ConstPtr& new_target);

// gather all control coefficients from rosparam
void initialize_controller(Control_Coefs & new_coefs, std::string param_name);

// input from fcu
void fcu_input_callback(const asctec_hl_comm::mav_status::ConstPtr& status);

//input from formation_control, for failure
void failure_message_callback(const std_msgs::String& message);

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


	//setup publisher
	ros::Publisher control_pub=nh.advertise<asctec_hl_comm::mav_ctrl>("fcu/control",1);
	               state_pub=nh.advertise<main_control::string_stamped>("main_control/state",1);
	               feedback_pub=nh.advertise<main_control::state_command>("main_control/feedback",1);
				   notification_pub=nh.advertise<std_msgs::String>("notification/channel",1);

	//setup subscribers
	ros::Subscriber ethz_kalman_input=nh.subscribe("ssf_core/ext_state", 1, ethz_kalman_input_callback);
	ros::Subscriber motion_capture_input;
	ros::Subscriber estimator_input;
	ros::Subscriber fcu_input=nh.subscribe("fcu/status", 1, fcu_input_callback);
	ros::Subscriber failure_input=nh.subscribe("notification/channel", 1, failure_message_callback);
	if(coefs.allowf==1){
		motion_capture_input = nh.subscribe("macortex_bridge/mcs_pose", 7, motion_analysis_input_callback_void);
		//estimator_input=nh.subscribe("internal_ekf/estimate_single", 1, estimator_input_callback);
		estimator_input = nh.subscribe("internal_ekf/estimate_self", 1, internal_ekf_input_callback);
	}
	else{
		motion_capture_input = nh.subscribe("macortex_bridge/mcs_pose", 7, motion_analysis_input_callback);
		estimator_input = nh.subscribe("internal_ekf/estimate_self", 1, internal_ekf_input_callback_void);
	}
	ros::Subscriber user_input = nh.subscribe("user/command_out", 2, user_input_callback);
	ros::Subscriber algorithm_input = nh.subscribe("algorithm/command_out", 1, algorithm_input_callback);

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
	    //duration between each cycle - control integrator purposes
		//elapsed = ros::Time::now().toSec() - last_time;

		//compute 3D force from desired position, speed, and acceleration
		compute_force(force, yaw_torque);

		//convert desired force to roll, pitch, thrust commands
		force2command(command, force, yaw_torque);

		//send to the quadcopter
		control_pub.publish(command);

		//next cycle
		ros::spinOnce();
		loop_rate.sleep();

		//cout << force << endl;
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

	//check if receiving data from cortex
	if(ros::Time::now().toSec() - timer > 0.5)
	{
		tracking = 0;
		lose_self = 1;
		if(verbose_mode) cout << "losing self position" << endl;
		//controller.action = DOWN;
		//controller.emergency_landing = 1;
	}

	if(lose_self==1 && (controller.action!=DOWN))
    {
        if(verbose_mode)
        	cout << "losing self position: emergency landing" << endl;
		controller.action = DOWN;
		send_state(state_pub, "down");
		controller.emergency_landing = 1;

		//notifications
		std_msgs::String message;
		message.data = "lose_self detected";
		notification_pub.publish(message);
    }  
	lose_self = 0;

	//check if environment boundaries reached
	Eigen::Vector3d d = state.pos - boundary.center;
	int bcount = 0;
	aux_target = takeoff_target; //just to put zeros on the other sutff
	aux_target.pos = state.pos;
	aux_target.accel << 0,0,0;
	if( abs(d(0)) > boundary.x )
		{aux_target.pos(0) = boundary.center(0)+SGN(d(0))*(boundary.x-coefs.safety_distance); bcount++;}
	if( abs(d(1)) > boundary.y )
		{aux_target.pos(1) = boundary.center(1)+SGN(d(1))*(boundary.y-coefs.safety_distance); bcount++;}
	if( abs(d(2)) > boundary.z )
		{aux_target.pos(2) = boundary.center(2)+SGN(d(2))*(boundary.z-coefs.safety_distance);  bcount++;}
	if(bcount>0)
		{if(controller.action != BOUNDARY_REACHED) send_state(state_pub,"hit");
		 safe_target.pos=aux_target.pos; controller.action = BOUNDARY_REACHED; controller.user_needed=1;}

	//each control action means different control strategies
	//yaw plays no part in the force
	switch(controller.action)
	{

		case DOWN: {
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
			aux_target.pos_on=1; //BUG QUAD FALLS AFTER FORMATION IS STOPPED
			if(verbose_mode) cout << "wating for user!!!" << aux_target.pos << endl;

			//if user has authorised normal operations
			if(!controller.user_needed)
				{controller.action=NORMAL_OPERATION; send_state(state_pub,"operating");}

		} break;

		case NORMAL_OPERATION: {

			//wait untill commands were issued
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
			if(verbose_mode) cout << "boundary reached!!!" <<endl;

			//check if end of boundary reach conditions are met
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

	//compute errors
	controller.ep = state.pos - aux_target.pos;	
	controller.ev = state.speed - aux_target.speed;
	controller.a_d = aux_target.accel;
	controller.pos_on = aux_target.pos_on;
	controller.yaw_rate_on = aux_target.yaw_rate_on;
	controller.relative_on = aux_target.relative_on;
	if(controller.pos_on == 1)
		controller.eIz += (controller.ep(2))*elapsed;

	controller.eyaw = state.angles(2)+coefs.offset_yaw-aux_target.angles(2);
	

	if(verbose_mode) 
	{
		cout << "state_angle: " << state.angles(2) << " and target_angle: " << aux_target.angles(2) <<endl;
		cout << "eyaw: " << controller.eyaw << endl;
	}


	while(controller.eyaw > PI) controller.eyaw -= 2*PI;
	while(controller.eyaw < -PI) controller.eyaw += 2*PI;
	
	controller.eIyaw += (controller.eyaw)*elapsed;

	if(verbose_mode) 
	{
		cout << "eyaw2: " << controller.eyaw << endl;
		cout << "debug time!!\n" << controller.pos_on << endl;

		cout << "position error\n" << controller.ep << "speed error\n" << controller.ev << "eIz\n" << controller.eIz << "a_d\n" << controller.a_d << endl;
		cout << "position \n" << state.pos << "speed \n" << state.speed << endl;
		cout << "position target \n" << target.pos << "speed target\n" << target.speed << endl;
	}


	//satisfy limitations
	//position
	/*Eigen::Vector2d eph;
	eph << controller.ep(0),controller.ep(1);
	eph = limit_vector(eph, coefs.Lph);
	double epv = controller.ep(2); LIMIT_SCALAR(epv,coefs.Lpv);
	controller.ep << eph, epv;

	//height
	LIMIT_SCALAR(controller.eIz,coefs.LIz);

	//velocity
	controller.ev = limit_vector(controller.ev,coefs.Lv);

	//acceleration
	controller.a_d = limit_vector(controller.a_d,coefs.La);*/

	//yaw
	LIMIT_SCALAR(controller.eyaw,coefs.Lyaw);


	//compute force
	Eigen::Vector3d z; z << 0, 0, 1; // Z axis
	force << 0,0,0;
	// different methods for calculating force depending on case
	// case pos_on == 1: control based on relative(?) position and velocity error
	if(controller.pos_on == 1)
		force = -coefs.Kp*controller.ep - coefs.Kv*controller.ev - coefs.KIz*controller.eIz*z;
	// case pos_on == 2: control based on relative(?) velocity error (used for followers in teleoperation case)
	else if(controller.pos_on == 2)
		force = -coefs.Kv*controller.ev;

	// add in force caused by velocity consensus command
	force = force + coefs.mass*coefs.Ka*controller.a_d;
	force = force - 0*coefs.allowf*state.accel*coefs.mass + coefs.mass*GRAVITY*z;
	Eigen::Vector2d forceh;
	forceh << force(0),force(1);
	forceh = limit_vector(forceh,coefs.Lph*coefs.Kp);
	force(0) = forceh(0);
	force(1) = forceh(1);
	//LIMIT_SCALAR(force(2),(coefs.mass*GRAVITY+coefs.La));
	
	if(controller.yaw_rate_on) yaw_torque = target.angles_rate(2);
	else yaw_torque = -coefs.Kyaw*controller.eyaw;
	relative = controller.relative_on;

	//send feedack
	send_feedback(feedback_pub,force,coefs.KIz*controller.eIz);
}

void force2command(asctec_hl_comm::mav_ctrl & command, Eigen::Vector3d & force, double yaw_torque)
{
	//going to the "body" frame. Only uses yaw
	// TODO verify signs in this rotation matrix
	Eigen::Matrix3d R;
    R <<  cos(state.angles(2)),   sin(state.angles(2)),  0,
          -sin(state.angles(2)),  cos(state.angles(2)),  0, 
                   0,                     0,             1;
    Eigen::Vector3d body_force;
    if(!relative)
		body_force = R*force; // virtual force rotated in the "body" frame
	else
		body_force = force;

	double thrust = body_force(2)/(cos(state.angles(0))*cos(state.angles(1)));
	double roll = (body_force.dot(body_force) != 0 ? -atan2(body_force(1), thrust):0 );
	double pitch = (body_force.dot(body_force) != 0 ? atan2(body_force(0), thrust):0 );
	command.type = asctec_hl_comm::mav_ctrl::acceleration;
	command.x = (-1)*pitch; //pitch will probably have to change
	command.y = (-1)*roll; //roll as computed before should be negative here to make sense
	command.z = THRUST_SCALE*thrust;
	command.yaw = yaw_torque;
}

void ethz_kalman_input_callback(const ssf_core::ext_state::ConstPtr& new_state)
{
	///TODO SPEED
	///
	state.pos(0) = new_state->pose.position.x;
	state.pos(1) = new_state->pose.position.y;
	state.pos(2) = new_state->pose.position.z;


    //state.speed(0) = (state.pos(0)-state.old_pos(0))/(current_timer-state.old_timer);
    //state.speed(1) = (state.pos(0)-state.old_pos(0))/(current_timer-state.old_timer);
    //state.speed(2) = (state.pos(0)-state.old_pos(0))/(current_timer-state.old_timer);
	

	double x=new_state->pose.orientation.x;
	double y=new_state->pose.orientation.y;
	double z=new_state->pose.orientation.z;
	double w=new_state->pose.orientation.w;

	state.angles(0)=atan2(2*(w*x+z*y),(1-2*(x*x+y*y)));
	state.angles(1)=asin(2*(w*y-x*z));
	state.angles(2)=atan2(2*(w*z+x*y),(1-2*(y*y+z*z)));
	
}
void motion_analysis_input_callback_void(const geometry_msgs::PoseStamped::ConstPtr& new_state)
{
	//we got measurements
	if( ((new_state->pose.position.x == 9999.999) && (new_state->pose.position.y == 9999.999) && (new_state->pose.position.z == 9999.999)) || lose_self_sim_9999==1)
	{
		printf("its working\n");
		return ;
	}

	//check if new measurement is possible
    double deplacement;
    deplacement=sqrt(pow(new_state->pose.position.x-state.pos(0),2) + 
                     pow(new_state->pose.position.y-state.pos(1),2) +
                     pow(new_state->pose.position.z-state.pos(2),2));
	static int counter_display=0;
	counter_display++;
	if(counter_display==6)
	{
		//cout << deplacement << endl;
		counter_display=0;
	}
	
    if((deplacement > LOSE_THRESH_POS || lose_self_sim_value==1)&&(tracking))
	{
		static bool already_done=0;
		if(already_done==0)
		{
			cout << "lose position : impossible value" << endl;
			cout << state.pos(0) << " -> " << new_state->pose.position.x << endl;
			cout << state.pos(1) << " -> " << new_state->pose.position.y << endl;
			cout << state.pos(2) << " -> " << new_state->pose.position.z << endl;
			cout << deplacement << endl;
			already_done=1;
		}
		return ;
	}

	//if no error, update timer
	tracking=1;
	timer = ros::Time::now().toSec(); measurements_received=1;
}
void motion_analysis_input_callback(const geometry_msgs::PoseStamped::ConstPtr& new_state)
{

	//we got measurements
	if( ((new_state->pose.position.x == 9999.999) && (new_state->pose.position.y == 9999.999) && (new_state->pose.position.z == 9999.999)) || lose_self_sim_9999==1)
	{
		printf("its working\n");
		return ;
	}

	//check if new measurement is possible
    double deplacement;
	if(verbose_mode)  printf("check self position (MA) : %f\n", ros::Time::now().toSec());
    deplacement=sqrt(pow(new_state->pose.position.x-state.pos(0),2) + 
                     pow(new_state->pose.position.y-state.pos(1),2) +
                     pow(new_state->pose.position.z-state.pos(2),2));
	//cout << deplacement << endl;
    if((deplacement > LOSE_THRESH_POS || lose_self_sim_value==1)&&(tracking))
	{
		static bool already_done=0;
		if(already_done==0)
		{
			cout << "lose position : impossible value" << endl;
			cout << state.pos(0) << " -> " << new_state->pose.position.x << endl;
			cout << state.pos(0) << " -> " << new_state->pose.position.x << endl;
			cout << state.pos(0) << " -> " << new_state->pose.position.x << endl;
			cout << deplacement << endl;
			already_done=1;
		}
		return ;
	}
	tracking = 1;

	//if no error, update timer
	timer = ros::Time::now().toSec(); measurements_received=1;

    //extracting position
        state.pos(0) = new_state->pose.position.x;
	state.pos(1) = new_state->pose.position.y;
	state.pos(2) = new_state->pose.position.z;

    //extracting velocity
	if(!state.first_time)
    {
        //first time the algorithm is run, the velocity cannot be computed
        state.first_time=1;
        state.speed(0) = 0;
	    state.speed(1) = 0;
	    state.speed(2) = 0;

    }
    else
    {
        //computing velocity from old pose
        Vector3d old_speed=state.speed;
        double current_timer=ros::Time::now().toSec();
        state.speed(0) = (state.pos(0)-state.old_pos(0))*cortex_rate;
	    state.speed(1) = (state.pos(1)-state.old_pos(1))*cortex_rate;
	    state.speed(2) = (state.pos(2)-state.old_pos(2))*cortex_rate;
	    state.old_timer=current_timer;
	    
	    //filtering results
	    state.speed=state.speed*alpha + old_speed*(1-alpha);
    }

    //the current position will become the old position in next iteration for velocity computations
    state.old_pos(0)=state.pos(0);
    state.old_pos(1)=state.pos(1);
    state.old_pos(2)=state.pos(2);
	
	//extracting object orientation
	double x=new_state->pose.orientation.x;
	double y=new_state->pose.orientation.y;
	double z=new_state->pose.orientation.z;
	double w=new_state->pose.orientation.w;

    //quaternions to angles
	state.angles(0)=atan2(2*(w*x+z*y),(1-2*(x*x+y*y)));
	state.angles(1)=asin(2*(w*y-x*z));
	state.angles(2)=atan2(2*(w*z+x*y),(1-2*(y*y+z*z)));
	
	/*printf("Pos=(%f %f %f)\nSpeed=(%f %f %f)\nAngles=(%f %f %f)\n\n",state.pos(0),state.pos(1),state.pos(2),
		state.speed(0),state.speed(1),state.speed(2),
		state.angles(0),state.angles(1),state.angles(2));*/
}
void internal_ekf_input_callback_void(const internal_ekf::EstimateSingle::ConstPtr& estimates_msg){;}
void internal_ekf_input_callback(const internal_ekf::EstimateSingle::ConstPtr& estimates_msg)
{
	//extracting position
	state.pos(0) = estimates_msg->estimate.position.x;
	state.pos(1) = estimates_msg->estimate.position.y;
	state.pos(2) = estimates_msg->estimate.position.z;

	//extracting velocity
	state.speed(0) = estimates_msg->estimate.velocity.x;
	state.speed(1) = estimates_msg->estimate.velocity.y;
	state.speed(2) = estimates_msg->estimate.velocity.z;

	//extracting perturbation
	state.accel(0) = estimates_msg->estimate.perturbation.x;
	state.accel(1) = estimates_msg->estimate.perturbation.y;
	state.accel(2) = estimates_msg->estimate.perturbation.z;

	//extracting object orientation
	double x=estimates_msg->estimate.orientation.x;
	double y=estimates_msg->estimate.orientation.y;
	double z=estimates_msg->estimate.orientation.z;
	double w=estimates_msg->estimate.orientation.w;

	//quaternions to angles
	state.angles(0)=atan2(2*(w*x+z*y),(1-2*(x*x+y*y)));
	state.angles(1)=asin(2*(w*y-x*z));
	state.angles(2)=atan2(2*(w*z+x*y),(1-2*(y*y+z*z)));
}

											
void user_input_callback(const std_msgs::String& obj)
{
	if(!measurements_received)
		{cout << "cortex is down" << endl; controller.action=DOWN; return;}

	stringstream ss; ss << obj.data;
	string command_name; ss >> command_name;
	if(!strcmp(command_name.c_str(),"takeoff"))
	{
		switch(controller.action){
			case DOWN: {controller.action=TAKEOFF; send_state(state_pub,"takeoff"); controller.user_needed=1;} break;
		}
	}
	if(!strcmp(command_name.c_str(),"operate"))
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
			case NORMAL_OPERATION: {controller.action=WAIT_USER; send_state(state_pub,"stop"); controller.user_needed=1; wait_target.pos=state.pos;} break;
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
	else if(!strcmp(command_name.c_str(),"goto"))
	{
		double x; ss >> x;

		double y; ss >> y;

		double z; ss >> z;

		switch(controller.action){
			case NORMAL_OPERATION:
				target.pos(0) = x;
				target.pos(1) = y;
				target.pos(2) = z;
				target.speed(0) = 0;
				target.speed(1) = 0;
				target.speed(2) = 0;

				target.accel(0) = 0;
				target.accel(1) = 0;
				target.accel(2) = 0;

				target.angles(2) = 0;

				issued_commands = 1;
				break;

		}
	}
	else if(!strcmp(command_name.c_str(), "vgoto"))
	{
		double vx, vy, vz;
		ss >> vx;
		ss >> vy;
		ss >> vz;

		switch(controller.action){
			case NORMAL_OPERATION:
				// target.speed is also set in algorithm_input_callback() below
				// because the vgoto command is the desired velocity of the entire group ...
				// it should be added to the velocity commanded to maintain the formation
				// ACTUALLY, target.speed set in algorithm_input_callback() is always zero
				target.speed(0) += vx;
				target.speed(1) += vy;
				target.speed(2) += vz;

				// vgoto commands should not be cumulative, so last commanded velocities ...
				// are subtracted from current command
				target.speed(0) -= last_command[0];
				target.speed(1) -= last_command[1];
				target.speed(2) -= last_command[2];

				// update last command velocities
				last_command[0] = vx;
				last_command[1] = vy;
				last_command[2] = vz;

				// these should be ignored anyway, since pos_on = 2 or 3
				target.pos(0) = 0;
				target.pos(1) = 0;
				target.pos(2) = 1;
				target.accel(0) = 0;
				target.accel(1) = 0;
				target.accel(2) = 0;
				target.angles(2) = 0;

				// check if this quad is the leader
				// leader doesn't have formation_control node, so ...
				// won't have pos_on set correctly
				// UPDATE TO WORK FOR ANY LEADER
				if(!strcmp(this_quad.c_str(),"quad97")){
					target.pos_on = 3;
				}

				issued_commands = 1;
				break;
		}


	}
	else if(!strcmp(command_name.c_str(),"set_home"))
	{
		double x; ss >> x;

		double y; ss >> y;

		double z; ss >> z;
		if(controller.action != HOME)
		{
			home_target.pos << x, y, z;
		}
	}
	else if(!strcmp(command_name.c_str(),"set_yaw"))
	{
		double new_yaw; ss >> new_yaw;
		if(controller.action == NORMAL_OPERATION)
		{
			target.angles(2) = PI*new_yaw/180;
			issued_commands = 1;
		}

		printf("new target yaw %f \n", target.angles(2));
	}
	else if(!strcmp(command_name.c_str(),"reset"))
	{
		controller.action = DOWN;
		controller.eIz = 0;
		force<<0,0,0;
		send_state(state_pub,"down");

	}
	else if(!strcmp(command_name.c_str(),"bat_fail_sim"))
	{
		bat_fail_sim=1;
		//notification
		std_msgs::String message;
		message.data="battery_low begin";
		notification_pub.publish(message);
		//consol
		cout<<"bat_sim_started" << endl;
	}
	else if(!strcmp(command_name.c_str(),"bat_fail_stop"))
	{
		bat_fail_sim=0;
	}
	else if(!strcmp(command_name.c_str(),"lose_self_sim_9999"))
	{
		lose_self_sim_9999=1;
		//notification
		std_msgs::String message;
		message.data="lose_self begin";
		notification_pub.publish(message);
		//consol
		cout<<"lose_self_started(delay)" << endl;
	}
	else if(!strcmp(command_name.c_str(),"lose_self_sim_value"))
	{
		lose_self_sim_value=1;
		//notification
		std_msgs::String message;
		message.data="lose_self begin";
		notification_pub.publish(message);
		//consol
		cout<<"lose_self_started(delay)" << endl;
	}
	else if(!strcmp(command_name.c_str(),"lose_self_stop"))
	{
		lose_self_sim_9999=0;
		lose_self_sim_value=0;
	}

	else {return;}
}

void fcu_input_callback(const asctec_hl_comm::mav_status::ConstPtr& status)
{
    double bat_level=0;
    
    //take battery level from fcu
	bat_level=status->battery_voltage; 

    //generate fail if needed
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

void failure_message_callback(const std_msgs::String& message)
{
	if(!strcmp(message.data.c_str(), "losing_leader"))
	{
		if((controller.action!=LANDING) && (controller.action!=DOWN))
		{
			switch(controller.action){
				case NORMAL_OPERATION: {
					controller.action=WAIT_USER;
					send_state(state_pub,"stop");
					controller.user_needed=1;
					wait_target.pos=state.pos;}
				break;
			}
		}
		//notifications
		std_msgs::String new_msg;
		new_msg.data="lose_leader detected";
		notification_pub.publish(new_msg);
	}
}

void algorithm_input_callback(const main_control::state_command::ConstPtr& new_target)
{
	// TODO this function should also check that the target is in the boundaries
	// if not it should sent a warning message

	// store message in target structure
	target.pos(0) = new_target->position.x;
	target.pos(1) = new_target->position.y;
	target.pos(2) = new_target->position.z;

	target.speed(0) = new_target->velocity.x;
	target.speed(1) = new_target->velocity.y;
	target.speed(2) = new_target->velocity.z;

	target.accel(0) = new_target->acceleration.x;
	target.accel(1) = new_target->acceleration.y;
	target.accel(2) = new_target->acceleration.z;

	target.angles(2) = new_target->yaw;
	target.angles_rate(2) = new_target->yaw_rate;

	target.pos_on = new_target->pos_on;
	target.yaw_rate_on = new_target->yaw_rate_on;
	target.relative_on = new_target->relative_on;

	issued_commands = 1;
}

void initialize_controller(Control_Coefs & new_coefs, std::string param_name)
{
	ros::NodeHandle nh; // is the same handle as in the main, seems ros is global
	std::string full_name;
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

	//initialize landding target
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
	aux_target.old_timer=0;
	aux_target.speed << 0,0,0;
	aux_target.accel << 0,0,0;
	aux_target.angles << 0,0,0;
	aux_target.first_time=1;
	aux_target.pos_on=1;
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
	controller.eIyaw = 0;
	controller.pos_on = 2;
	//controller.pos_on = 2;
	controller.user_needed = 0;
	controller.emergency_landing = 0;
	issued_commands=0;
	measurements_received=0;
	tracking=0;

	full_name = param_name + "allowf";
	nh.getParam(full_name.c_str(), new_coefs.allowf);
	full_name = param_name + "cortex_rate";
	nh.getParam(full_name.c_str(), cortex_rate);
	full_name = param_name + "alpha";
	nh.getParam(full_name.c_str(), alpha);

	full_name = param_name + "verbose";
	nh.getParam(full_name.c_str(), verbose_mode);

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

	//f.header.stamp = ros::Time::now();
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

