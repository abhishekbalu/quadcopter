//C++ Libraries
#include <math.h>
#include <iostream>
//ROS Libraries
#include "tf/transform_datatypes.h"
#include <px_comm/OpticalFlow.h>
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
//User Libraries
#include "height_quad/state.h"
#include "height_quad/Kalman_2D.h"
#include "height_quad/debug.h" //Comment or uncomment this for verbose

using namespace Eigen;

#define GAIN 0.16
#define FOCAL_LENGTH 12.0
#define GRAVITY 9.81
#define ALPHA 0.125 //double Ts = 0.01; double tau = 0.08; double alpha = Ts/tau;
#define RATE 10

height_quad::Attitude attitude;
//Custom message
height_quad::state state;  
geometry_msgs::Twist msg;
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
Kalman_2D kalman(0.1, 0.1, 1, 0, 0, 1);

MatrixXd Rroll(3,3);
MatrixXd Rpitch(3,3);
MatrixXd Ryaw(3,3);
MatrixXd RotMat(3,3);
VectorXd OF(2);
VectorXd VecG(3);
VectorXd VecDir(3);
VectorXd VecAccel(3);

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
	kalman.A(0,2)=b;
    kalman.A(1,3)=b;
    kalman.A(2,4)=b;
    kalman.A(3,5)=b;

    kalman.A(0,4)=a;
    kalman.A(1,5)=a;

    kalman.B(0,0)=a;
    kalman.B(1,1)=a;
    kalman.B(2,0)=b;
    kalman.B(3,1)=b;

    kalman.W(0,0)=a;
    kalman.W(1,1)=a;

    kalman.W(2,0)=b;
    kalman.W(3,1)=b;
    updateRotMatrixes();
    VecAccel = aZ->current * RotMat * VecDir - VecG;
	kalman.U(0)=VecAccel(0);
	kalman.U(1)=VecAccel(1);
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
	tf::quaternionMsgToTF(data->orientation, orientation); //static void tf::quaternionMsgToTF(const geometry_msgs::Quaternion &msg, Quaternion &bt)	
	tf::Matrix3x3(orientation).getEulerYPR(yaw, pitch, roll);
	//Get Roll, Pitch and Yaw
	attitude.pitch = theta;
    attitude.roll = phi;
    attitude.yaw = yaw;
	aZ->last_t = aZ->current_t;
	aZ->last = aZ->current;
	aZ->current_t = ros::Time::now();
	aZ->current = data->linear_acceleration.z;
	//ros::Duration = aZ->current_t - aZ->last_t;
	ros::Duration time = (aZ->current_t - aZ->last_t);
	aZ->t = time.toSec();
	updateState();
	kalman.KalmanPredict();
}

void getOptFlow(const geometry_msgs::TwistStamped::ConstPtr& data){
	//Simple Low pass digital with an RC constant of alpha filter
	if(LPF == NULL){
        //initialize OF
        LPF = new low_pass_filter;
        LPF->last_value(0) = (data->twist).linear.x;
        LPF->last_value(1) = (data->twist).linear.y;
        LPF->current_value = LPF->last_value;
    }else{

    	LPF->last_value = LPF->current_value;

    	LPF->current_value(0) = LPF->last_value(0) + ALPHA * ((data->twist).linear.x - LPF->last_value(0));
    	LPF->current_value(1) = LPF->last_value(1) + ALPHA * ((data->twist).linear.y - LPF->last_value(1));

    	OF = LPF->current_value;

		theta = GAIN * OF(0);
		phi = GAIN * OF(1);
		
        //Kalman Update with new values
		kalman.KalmanUpdate(OF);
		//Output state
		state.header.stamp = ros::Time::now();
		state.attitude = attitude;
		state.x = kalman.Xhat(0,0);
		state.y = kalman.Xhat(1,0);
		state.vx = kalman.Xhat(2,0);
		state.vy = kalman.Xhat(3,0);
		#ifdef VERBOSE
			ROS_INFO("Gravity value: [%f]", VecG(2));
			ROS_INFO("Phi(Roll): [%f] , Theta(Pitch): [%f], Psi(Yaw): [%f]", phi, theta, yaw);
			ROS_INFO("X: [%f] Y: [%f] vX: [%f] vY: [%f], quality: [%d]", state.x, state.y, state.vx, state.vy);
    	#endif
    }

}
int main(int argc, char** argv){
	ROS_INFO("Started xy_pose...\n");
	ros::init(argc, argv, "xy_pose");
    ros::NodeHandle n;

    VecG << 0,0,GRAVITY;
	VecDir << 0,0,1;
	VecAccel << 0,0,0;

    Xhat << 0, 0, 0, 0, 0, 0; //initial position
    kalman.Xhat = Xhat;

    //Subscribers
    ros::Subscriber opt = n.subscribe("/velocity_xy", 1, getOptFlow);
	ros::Subscriber imu = n.subscribe("/mavros/imu/data", 10, getImu);
	ros::Publisher pub = n.advertise<height_quad::state>("/xy_pose", 10);

	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		pub.publish(msg);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}
