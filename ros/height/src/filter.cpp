//C++ Libraries
#include <list>
#include <math.h>
#include <sstream>
#include <iostream>
#include <iterator>
#include <random>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
//ROS Libraries
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <px_comm/OpticalFlow.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/Imu.h>

#include "height/state.h"
#include <height/Kalman_1D.h>
#include "height/Kalman_2D.h"
#include "height/debug.h" //Comment or uncomment this for verbose output

using namespace Eigen;
using namespace std;

#define CENTER_OF_MASS_FILTER_OFFSET 0
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
	Vector2d current_value; 
	Vector2d last_value;
}low_pass_filter;

low_pass_filter* LPF = NULL;

//Kalman Filter
//0.01, 0.1, 1, 0, 0, 1
Kalman_2D kalman2(0.01, 0.1, 1, 0, 0, 1);

MatrixXd Rroll(3,3);
MatrixXd Rpitch(3,3);
MatrixXd Ryaw(3,3);
MatrixXd RotMat(3,3);
VectorXd OF(2);
VectorXd VecG(3);
VectorXd VecDir(3);
VectorXd VecAz(3);
//Global variables -- this should be fixed
std::list<float> reads;
sensor_msgs::Range *config=NULL;
sensor_msgs::Range *msg;
float value;
int created=0;
Kalman_1D kalman(0.2, 32, 1, 0.14); //Q, R, P, z initial sensor position(height of the sensor)
px_comm::OpticalFlow *OptFlow = NULL;

double white_noise(){
  // construct a trivial random generator engine from a time-based seed:
  const double mean = 0.0;
  const double stddev = 0.001;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);//About 99.7% of a population is within +/- 3 standard deviations
  return dist(generator);
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

void updateState(){
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

void readings(ros::Publisher pub){
	if (reads.size() != 0){
		float sum = 0;
		/*The filtering is just averaging for now*/
		for(std::list<float>::iterator it= reads.begin(); it != reads.end(); ++it){
			sum = sum + *it;
		}
		value=sum/reads.size();
	}else{
		value=0;
	}
	if (config != NULL){
		config->header.seq += 1;
		msg = new sensor_msgs::Range;
		msg->header = config->header;
		msg->radiation_type = config->radiation_type;
		msg->field_of_view = config->field_of_view;
		msg->min_range = config->min_range;
		msg->max_range = config->max_range;
		msg->range = config->range;

		msg->header.stamp=ros::Time::now();
		msg->range=value;
		//std::cout << typeid(*msg).name() << '\n';
		#ifdef VERBOSE
			ROS_INFO("Filtered Seq: [%d]", msg->header.seq);
			ROS_INFO("Filtered Range: [%f]", msg->range);
		#endif
		pub.publish(*msg);
	}
}

void getSonar(const sensor_msgs::Range::ConstPtr& data){
	double range = data->range + white_noise();
	#ifdef VERBOSE
		ROS_INFO("Received Seq [%d]", data->header.seq);
		ROS_INFO("Received Range [%f]", range);
	#endif
	//Do a deep copy
	if (config == NULL){
		config = new sensor_msgs::Range;
		config->header = data->header;
		config->radiation_type = data->radiation_type;
		config->field_of_view = data->field_of_view;
		config->min_range = data->min_range;
		config->max_range = data->max_range;
		config->range = range;
	}
	//Append to the list
	float aux = kalman.KalmanFilter(range +  CENTER_OF_MASS_FILTER_OFFSET);
	reads.push_back(aux);
	if(reads.size()>5){
		reads.pop_front();
	}
}

void getGndTruth(const nav_msgs::Odometry::ConstPtr& data){
	if(OptFlow == NULL){
		OptFlow = new px_comm::OpticalFlow;
		OptFlow->header.seq=0;
	}
	OptFlow->header.seq++;
	OptFlow->ground_distance = data->pose.pose.position.z;
	OptFlow->velocity_x = data->twist.twist.linear.x;
	OptFlow->velocity_y = data->twist.twist.linear.y;
	OptFlow->quality = 255;
	#ifdef VERBOSE
		ROS_INFO("Got linear velocities: velocity_x: [%f], velocity_y: [%f] with quality [%d]", OptFlow->velocity_x, OptFlow->velocity_y, OptFlow->quality);
	#endif

	double velocity_x = OptFlow->velocity_x + white_noise();
	double velocity_y = OptFlow->velocity_y + white_noise();
	//Simple Low pass digital with an RC constant of alpha filter
	if(LPF == NULL){
        //initialize OF
        LPF = new low_pass_filter;
        LPF->last_value(0) = velocity_x;
        LPF->last_value(1) = velocity_y;
        LPF->current_value = LPF->last_value;
    }else{

    	LPF->last_value = LPF->current_value;

    	LPF->current_value(0) = LPF->last_value(0) + ALPHA * (velocity_x - LPF->last_value(0));
    	LPF->current_value(1) = LPF->last_value(1) + ALPHA * (velocity_y - LPF->last_value(1));
   		OF = LPF->current_value;
   		
   		LPF->quality = OptFlow->quality;
   		theta = GAIN * OF(0);
   		phi = GAIN * OF(1);
        double fov = computeFOV(); //do what you want with this
        ////Kalman Update with new values
		kalman2.KalmanUpdate(OF);
		//Output state
		state.header.stamp = ros::Time::now();
		state.quality = LPF->quality;
		state.attitude = attitude;
		state.x = kalman2.Xhat(0,0);
		state.y = kalman2.Xhat(1,0);
		state.vx = kalman2.Xhat(2,0);
		state.vy = kalman2.Xhat(3,0);
		float deltax = abs(velocity_x - state.vx);
		float deltay = abs(velocity_y - state.vy);
		#ifdef VERBOSE
			ROS_INFO("Phi(Roll): [%f] , Theta(Pitch): [%f], Psi(Yaw): [%f]", phi, theta, yaw); //Not sure this is accurate
			ROS_INFO("Filter_dif_x [%f], Filter_dif_y [%f]", deltax, deltay);
			ROS_INFO("X: [%f] Y: [%f] vX: [%f] vY: [%f], quality: [%d]", state.x, state.y, state.vx, state.vy, LPF->quality);
    	#endif
    }


}

int main(int argc, char **argv){
	ROS_INFO("Started global_filter...\n");
	ros::init(argc, argv, "filter");
	ros::NodeHandle n;

	VecG << 0,0,GRAVITY;
	VecDir << 0,0,1;
	VecAz << 0,0,0;

 
    kalman2.Xhat << 0, 0, 0, 0, 0, 0; //initial pose

	ros::Subscriber gnd = n.subscribe("/ground_truth/state", 10, getGndTruth); 
	ros::Subscriber sonar = n.subscribe("/sonar_bottom", 10, getSonar); //May use the simulation sonar here
	ros::Subscriber imu = n.subscribe("/raw_imu", 10, getImu);
	ros::Publisher z_pub = n.advertise<sensor_msgs::Range>("/z_pose",10);
	ros::Publisher xy_pub = n.advertise<height::state>("xy_pose", 10);
	ros::Rate loop_rate(RATE);

	while(ros::ok()){
		xy_pub.publish(state);
		readings(z_pub);
		loop_rate.sleep();
		ros::spinOnce();
	}
	ros::spin();
	return 0;

}