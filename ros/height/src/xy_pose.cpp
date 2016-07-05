//C++ Libraries
#include <math.h>
#include <iostream>
//ROS Libraries
#include "tf/transform_datatypes.h"
#include <px_comm/OpticalFlow.h>
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
//User Libraries
#include "height/state.h"
#include "height/Kalman_2D.h"
#include "height/debug.h" //Comment or uncomment this for verbose
//Namespaces
using namespace Eigen;

#define GAIN 0.16
#define FOCAL_LENGTH 12.0
#define GRAVITY 9.81
#define ALPHA 0.05
#define RATE 10
//Custom messages
height::Attitude attitude;
height::state state;  

tfScalar yaw, pitch, roll;

//IMU
typedef struct{
	ros::Time last_t;
	ros::Time current_t;
	double last;
	double current;
	double t;
}imu_z_accel;
 
imu_z_accel* aZ = NULL;
double theta; //Pitch
double phi;//Roll
//OpticalFlow
typedef struct{
	ros::Time last_t;
	ros::Time current_t;
	int quality;
	Vector2d current_OF; 
	Vector2d last_OF;
}optical_flow;

optical_flow* oF = NULL;

//Kalman Filter
Kalman_2D kalman(0.01, 0.1, 1, 0, 0, 1);

MatrixXd Rroll(3,3);
MatrixXd Rpitch(3,3);
MatrixXd Ryaw(3,3);
MatrixXd RotMat(3,3);
VectorXd OF(2);
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

double computeFOV(){
    double d = 30*60*pow(10,-6);
    double aux = 2*atan2(d, 2*FOCAL_LENGTH*pow(10,-3));
    float field_of_view = aux * 1000;
    #ifdef VERBOSE
    	ROS_INFO("Field of View: [%f]", field_of_view);
    #endif
    return aux;
}
void updateState(){
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
    VecAz = aZ->current * RotMat * VecDir - VecG;
	kalman.U(0)=VecAz(0);
	kalman.U(1)=VecAz(1);
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
	//getzAccel
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

void getOptFlow(const px_comm::OpticalFlow::ConstPtr& data){
	//Simple Low pass digital with an RC constant of alpha filter
	if(oF == NULL){
        //initialize OF
        oF = new optical_flow;
        oF->last_OF(0) = data->velocity_x;
        oF->last_OF(1) = data->velocity_y;
        oF->current_OF(0) = oF->last_OF(0);
        oF->current_OF(1) = oF->last_OF(1);
    }else{

    	oF->last_OF(0) = oF->current_OF(0);
    	oF->last_OF(1) = oF->current_OF(1);

    	oF->current_OF(0) = oF->last_OF(0) + ALPHA * (data->velocity_x - oF->last_OF(0));
    	oF->current_OF(1) = oF->last_OF(1) + ALPHA * (data->velocity_y - oF->last_OF(1));

   		OF(0) = oF->current_OF(0);
   		OF(1) = oF->current_OF(1);
   		oF->quality = data->quality;
   		theta = GAIN * OF(0);
   		phi = GAIN * OF(1);

        double fov = computeFOV(); //do what you want
		
        
        ////Kalman Update with new values
		kalman.KalmanUpdate(OF);
		//Output state
		state.header.stamp = ros::Time::now();
		state.quality = oF->quality;
		state.attitude = attitude;
		state.x = kalman.Xhat(0,0);
		state.y = kalman.Xhat(1,0);
		state.vx = kalman.Xhat(2,0);
		state.vy = kalman.Xhat(3,0);
		float deltax = abs(data->velocity_x - state.vx);
		float deltay = abs(data->velocity_y - state.vy);
		#ifdef VERBOSE
			ROS_INFO("Phi(Roll): [%f] , Theta(Pitch): [%f], Psi(Yaw): [%f]", phi, theta, yaw);
			ROS_INFO("Filter_dif_x [%f], Filter_dif_y [%f]", deltax, deltay);
			ROS_INFO("X: [%f] Y: [%f] vX: [%f] vY: [%f], quality: [%d]", state.x, state.y, state.vx, state.vy, oF->quality);
    	#endif
    }

}

int main(int argc, char** argv){
	ROS_INFO("Started xy_pose...\n");
	ros::init(argc, argv, "xy_pose");
    ros::NodeHandle n, nh("~");

    VecG << 0,0,GRAVITY;
	VecDir << 0,0,1;
	VecAz << 0,0,0;

 
    kalman.Xhat << 0, 0, 0, 0, 0, 0; //initial pose

    //Subscribers
    ros::Subscriber opt = n.subscribe("/simOptFlow", 1, getOptFlow);
	ros::Subscriber imu = n.subscribe("/raw_imu", 10, getImu);
	ros::Publisher pub = n.advertise<height::state>("xy_pose", 10);

	ros::Rate loop_rate(RATE);
	while(ros::ok()){
		pub.publish(state);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}