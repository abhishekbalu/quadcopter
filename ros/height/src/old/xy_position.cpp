#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <math.h>
#include "tf/transform_datatypes.h"
#include <px_comm/OpticalFlow.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include "height/state.h"

using namespace Eigen;

#include <iostream>

height::state state; //Custom message 
tfScalar yaw, pitch, roll;

typedef struct{
	ros::Time last_t;
	ros::Time current_t;
	double last;
	double current;
	double t;
}imu_z_accel;
 
imu_z_accel* aZ = NULL;



//Kalman filter
MatrixXd A(6,6); //6, 6
MatrixXd P(6,6); //6, 6
MatrixXd B(6,2); //6, 2
MatrixXd K(6,2); //6, 2
VectorXd Xhat(6); //6, 1
VectorXd U(2); //2, 1
VectorXd Resid(2); //2, 1
VectorXd OF(2); //2, 1
MatrixXd Q(4,4); //4,4
MatrixXd W(6,4); //6,4
MatrixXd C(2,6); //2,6
MatrixXd R(2,2); //2,2

MatrixXd Rroll(3,3);
MatrixXd Rpitch(3,3);
MatrixXd Ryaw(3,3);
MatrixXd RotMat(3,3);

VectorXd VecG(3);
VectorXd VecDir(3);
VectorXd VecAz(3);

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
void KalmanUpdate(){
	K = (P * C.transpose()) * (C * P * C.transpose() + R).inverse();
	Resid = OF - C*Xhat;
	Xhat = Xhat + K*Resid;
	P = (Eigen::MatrixXd::Identity(6, 6)-K*C)*P;
}

void KalmanPredict(){
	double a = pow(aZ->t,2)/2;
	double b = aZ->t;
	A(0,2)=b;
    A(1,3)=b;
    A(2,4)=b;
    A(3,5)=b;

    A(0,4)=a;
    A(1,5)=a;
    B(0,0)=a;
    B(1,1)=a;

    B(2,0)=b;
    B(3,1)=b;

    W(0,0)=a;
    W(1,1)=a;

    W(2,0)=b;
    W(3,1)=b;
    updateRotMatrixes();
    VecAz = aZ->current * RotMat * VecDir - VecG;
	U(0)=VecAz(0);
	U(1)=VecAz(1);
	//Predict equation
	Xhat = A * Xhat + B * U;
	P = A * P * A.transpose() + W * Q * W.transpose();

}

void getOptFlow(const px_comm::OpticalFlow::ConstPtr& data){
	OF(0) = data->velocity_x;
	OF(1) = data->velocity_y;
	KalmanUpdate();
	state.header.stamp = ros::Time::now();
	state.x = Xhat(0,0);
	state.y = Xhat(1,0);
	state.vx = Xhat(2,0);
	state.vy = Xhat(3,0);
	ROS_INFO("X: [%f] Y: [%f] vX: [%f] vY: [%f]", state.x, state.y, state.vx, state.vy);
}

void Kalman_init(){
	Xhat << 0, 0, 0, 0, 0, 0;
	P << 0,0,0,0,0,0,
	     0,0,0,0,0,0,
	     0,0,0,0,0,0,
	     0,0,0,0,0,0,
	     0,0,0,0,0,0,
	     0,0,0,0,0,0;
	A << 1,0,0,0,0,0,
	     0,1,0,0,0,0,
	     0,0,1,0,0,0,
	     0,0,0,1,0,0,
	     0,0,0,0,1,0,
	     0,0,0,0,0,1;
	B << 0,0,
	     0,0,
	     0,0,
	     0,0,
	     0,0,
	     0,0;
	W << 0,0,0,0,
	     0,0,0,0,
	     0,0,0,0,
	     0,0,0,0,
	     0,0,1,0,
	     0,0,0,1;
	Q << 0.01,  0,      0,      0,
	     0,     0.01,   0,      0,
	     0,     0,      0.01,   0,
	     0,     0,      0,      0.01;
	R << 0.1,   0,
	     0,     0.1;
	VecG << 0,0,9.81;
	VecDir << 0,0,1;
	VecAz << 0,0,0;
	C << 0,0,1,0,0,0,
	     0,0,0,1,0,0;
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

	//getzAccel
	aZ->last_t = aZ->current_t;
	aZ->last = aZ->current;
	aZ->current_t = ros::Time::now();
	aZ->current = data->linear_acceleration.z;
	//ros::Duration = aZ->current_t - aZ->last_t;
	ros::Duration time = (aZ->current_t - aZ->last_t);
	aZ->t = time.toSec();
	KalmanPredict();
}
int main(int argc, char** argv){

    Kalman_init();
	ros::init(argc, argv, "XY_POS");
	ros::NodeHandle n;

	ros::Subscriber opt = n.subscribe("/opt_flow_filtered", 10, getOptFlow);
	ros::Subscriber imu = n.subscribe("/raw_imu", 10, getImu);
	ros::Publisher pub = n.advertise<height::state>("xy_state", 10);
	ros::Rate loop_rate(5);
	while(ros::ok()){
		pub.publish(state);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}