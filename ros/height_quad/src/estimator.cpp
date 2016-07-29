//C++ Libraries
#include <math.h>
#include <list>
#include <iostream>
//ROS Libraries
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include <px_comm/OpticalFlow.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>


#include <height_quad/Estimate.h>
#include <height_quad/EstimateSingle.h>
#include <height_quad/EstimateMulti.h>
//User Libraries
#include "height_quad/state.h"
#include "height_quad/Kalman_2D.h"
#include "height_quad/debug.h" //Comment or uncomment this for verbose
#include "height_quad/Kalman_1D.h"
#include "height_quad/debug.h"

using namespace Eigen;
using namespace std;

#define GAIN 0.16
#define FOCAL_LENGTH 12.0
#define GRAVITY 9.81
#define ALPHA 0.0625 //double Ts = 0.01; double tau = 0.16; double alpha = Ts/tau;
#define RATE 10
#define CENTER_OF_MASS_FILTER_OFFSET 0.06
#define NUMB_VALUES_TO_DISCARD 4
#define RATE 10
#define MAX_DISTANCE 3.0	
#define MIN_DISTANCE 0.2


Kalman_1D kalman1(0.2, 32, 1, 0.14); //Q, R, P, z initial sensor position(height of the sensor)

px_comm::OpticalFlow *config=NULL;
sensor_msgs::Range *msg;

struct State{ 
	float u;
	float std_o;
	float o;
	float valid_z;
	float last_valid;
	std::list<float> reads;
	State():u(0.0), std_o(0.2), o(0.2), valid_z(0.0), last_valid(0.0){}
}st_z;


height_quad::EstimateSingle est;
height_quad::Attitude attitude;
//Custom message
height_quad::state state;  
tfScalar yaw, pitch, roll;

double theta; //Pitch
double phi;//Roll
//IMU
typedef struct{
	ros::Time last_t;
	ros::Time current_t;
	double last;
	double current;
	double t;
}imu_z_accel;
 
imu_z_accel* aZ = NULL;

//OpticalFlow
typedef struct{
	ros::Time last_t;
	ros::Time current_t;
	Vector2d current_value; 
	Vector2d last_value;
}low_pass_filter;

low_pass_filter* LPF = NULL;

//Kalman Filter
VectorXd Xhat(6);
Kalman_2D kalman2(0.1, 0.1, 1, 0, 0, 1);

MatrixXd Rroll(3,3);
MatrixXd Rpitch(3,3);
MatrixXd Ryaw(3,3);
MatrixXd RotMat(3,3);
VectorXd OF(2);
VectorXd VecG(3);
VectorXd VecDir(3);
VectorXd VecAz(3);


void filterSonar(ros::Publisher pub){
	float t_res = 0.01;
	//Last value
	float z = st_z.reads.back();
	//Mean
	float sum = 0;
	for(std::list<float>::iterator it=st_z.reads.begin(); it != st_z.reads.end(); ++it){
		sum = sum + *it;
	}
	float mean=sum/st_z.reads.size();
	//Variance
	float var = 0.0;
	for(std::list<float>::iterator it= st_z.reads.begin(); it != st_z.reads.end(); ++it){
		var+=pow((*it-mean),2);
	}
	var = var / (st_z.reads.size()-1);
	//If variance is below t_res, then last value is valid
	if(var < t_res){
		st_z.valid_z = z;
		st_z.last_valid = z;
		//New value is valid if it's within last valid and the margin given by sqrt(t_res)
	}else if (abs(st_z.last_valid - z) < sqrt(t_res)){
		st_z.valid_z = z;
	}
	//Copy everything
	config->header.seq += 1;
	msg = new sensor_msgs::Range;
	msg->header = config->header;
	//msg->radiation_type = config->radiation_type;
	//msg->field_of_view = config->field_of_view;
	msg->min_range = MAX_DISTANCE;
	msg->max_range = MIN_DISTANCE;

	msg->header.stamp=ros::Time::now();
	msg->range = st_z.valid_z;
	est.header.stamp=ros::Time::now();
	est.estimate.position.z = st_z.valid_z;
	est.estimate.updated = 1;
	std_msgs::String s1;
	string aux("z:of");
	s1.data = aux;
	
	est.estimate.sensors = s1;
	//std::cout << typeid(*msg).name() << '\n';
	ROS_INFO("Filtered Seq: [%d]", msg->header.seq);
	ROS_INFO("Filtered Range: [%f]", msg->range);
	pub.publish(est);
}

void updateRotMatrixes(){
	Rroll << 1, 0,          0,
             0, cos(roll), -sin(roll),
             0, sin(roll), cos(roll);
    Rpitch << cos(pitch), 0, sin(pitch),
              0,          1, 0,
              -sin(pitch),0, cos(pitch);
    Ryaw << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0,        0,        1;
    RotMat=Rroll*Rpitch*Ryaw;
}

double computeFOV(){
    double d = 30*60*pow(10,-6);
    double aux = 2*atan2(d, 2*FOCAL_LENGTH*pow(10,-3));
    float field_of_view = aux * 1000;
    #ifdef VERBOSE
    	ROS_INFO("Field of View: [%f]", field_of_view);
    #endif
    return aux;
}

void updateState(){ //Update our matrixes with state and environment input
	double a = pow(aZ->t,2)/2;
	double b = aZ->t;
	kalman2.A(0,2)=b;
    kalman2.A(1,3)=b;
    kalman2.A(2,4)=b;
    kalman2.A(3,5)=b;

    kalman2.A(0,4)=a;
    kalman2.A(1,5)=a;
    kalman2.B(0,0)=a;
    kalman2.B(1,1)=a;

    kalman2.B(2,0)=b;
    kalman2.B(3,1)=b;

    kalman2.W(0,0)=a;
    kalman2.W(1,1)=a;

    kalman2.W(2,0)=b;
    kalman2.W(3,1)=b;
    updateRotMatrixes();
    VecAz = aZ->current * RotMat * VecDir - VecG;
	kalman2.U(0)=VecAz(0);
	kalman2.U(1)=VecAz(1);
}
void getImu(const sensor_msgs::Imu::ConstPtr& data){ //gets quaternion of orientation and z acceleration
	if(aZ == NULL){
		aZ = new imu_z_accel;
		aZ->last_t = ros::Time::now();
		aZ->last = data->linear_acceleration.z;
		aZ->current_t = aZ->last_t;
		aZ->current = aZ->last;
		VecG(2) = data->linear_acceleration.z;
	}
	//getImu -- to get rotations
	tf::Quaternion orientation;
	est.estimate.orientation = data->orientation;
	tf::quaternionMsgToTF(data->orientation, orientation); //static void tf::quaternionMsgToTF(const geometry_msgs::Quaternion &msg, Quaternion &bt)	
	tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
	//Get Roll, Pitch and Yaw
	attitude.pitch = theta;
    attitude.roll = phi;
    attitude.yaw = yaw;
	//getzAccel
	aZ->last_t = aZ->current_t;
	aZ->last = aZ->current;
	aZ->current_t = ros::Time::now();
	aZ->current = data->linear_acceleration.z;
	//ros::Duration = aZ->current_t - aZ->last_t;
	ros::Duration time = (aZ->current_t - aZ->last_t);
	aZ->t = time.toSec();
	updateState();
	kalman2.KalmanPredict();
}
void getOptFlow(const px_comm::OpticalFlow::ConstPtr& data){
	#ifdef VERBOSE
		ROS_INFO("Received Seq [%d]", data->header.seq);
		ROS_INFO("Received Range [%f]", data->ground_distance);
	#endif 
	//Do a deep copy
	//For more about deepcopies https://www.cs.utexas.edu/~scottm/cs307/handouts/deepCopying.htm
	if (config == NULL){
		config = new px_comm::OpticalFlow;
		//Deepcopy of config
		config->header = data->header;
		config->velocity_x = data->velocity_x;
		config->velocity_y = data->velocity_y;
		config->flow_x = data->flow_x;
		config->flow_y = data->flow_y;
		config->ground_distance = data->ground_distance;
		config->header.seq = 0;
	}
	//Use the ground_distance parameter and reject outliers
	if(data->ground_distance < MAX_DISTANCE && data->ground_distance > MIN_DISTANCE){
		float aux = kalman1.KalmanFilter(data->ground_distance + CENTER_OF_MASS_FILTER_OFFSET);
		st_z.reads.push_back(aux);
		if(st_z.reads.size() > 4){
			if(st_z.reads.size() == 6){
				st_z.reads.pop_front();
			}
		}
	}



	//Simple Low pass digital with an RC constant of alpha filter
	if(LPF == NULL){
        //initialize OF
        LPF = new low_pass_filter;
        LPF->last_value(0) = data->velocity_x;
        LPF->last_value(1) = data->velocity_y;
        LPF->current_value = LPF->last_value;
    
    }else{

    	LPF->last_value = LPF->current_value;
    	
    	LPF->current_value(0) = LPF->last_value(0) + ALPHA * (data->velocity_x - LPF->last_value(0));
    	LPF->current_value(1) = LPF->last_value(1) + ALPHA * (data->velocity_y - LPF->last_value(1));

   		OF = LPF->current_value;

   		theta = GAIN * OF(0);
   		phi = GAIN * OF(1);
        //fov = computeFOV();
		
        ////Kalman Update with new values
		kalman2.KalmanUpdate(OF);
		//Output state
		state.header.stamp = ros::Time::now();
		state.attitude = attitude;
		state.x = kalman2.Xhat(0,0);
		state.y = kalman2.Xhat(1,0);
		state.vx = kalman2.Xhat(2,0);
		state.vy = kalman2.Xhat(3,0);
		est.header.stamp = ros::Time::now();
		est.estimate.position.x = kalman2.Xhat(0,0);
		est.estimate.position.y = kalman2.Xhat(1,0);
		est.estimate.velocity.x = kalman2.Xhat(0,0);
		est.estimate.velocity.y = kalman2.Xhat(0,0);
		#ifdef VERBOSE
			ROS_INFO("Phi(Roll): [%f] , Theta(Pitch): [%f], Psi(Yaw): [%f]", phi, theta, yaw);
			ROS_INFO("X: [%f] Y: [%f] vX: [%f] vY: [%f], quality: [%d]", state.x, state.y, state.vx, state.vy);
    	#endif
    }

}
int main(int argc, char** argv){
	ROS_INFO("Started estimator...\n");
	ros::init(argc, argv, "estimator");
    ros::NodeHandle n, nh("~");

    VecG << 0,0,GRAVITY;
	VecDir << 0,0,1;
	VecAz << 0,0,0;

    Xhat << 0, 0, 0, 0, 0, 0; //initial position
    kalman2.Xhat = Xhat;

    //Subscribers
    //ros::Subscriber sonar = n.subscribe("/px4flow/opt_flow", 10, getSonar); //May use the simulation sonar here
    ros::Subscriber opt = n.subscribe("/px4flow/opt_flow", 1, getOptFlow);
	ros::Subscriber imu = n.subscribe("/mavros/imu/data", 10, getImu);
	ros::Publisher pub = n.advertise<height_quad::EstimateSingle>("/estimator/estimate_self", 10);
	
	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		if(st_z.reads.size() > NUMB_VALUES_TO_DISCARD){
			filterSonar(pub);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}