#ifndef __EGOMOTION__
#define __EGOMOTION__

//C++ Includes
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Geometry>

//ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <quad_msgs/Estimate.h>
#include <quad_msgs/EstimateSingle.h>
#include <mavros_msgs/AttitudeTarget.h>

using namespace Eigen;

//Macros
#define MAX_STAMPS 50

//sructure containing information required to implement egomotion and lag compensation on the sensor measurements
//several time stamps are considered to alow lag compesation
typedef struct t_egomotion {
	int head; //where are we inserting prediction measurements
	int lag_count; //counter to initialize enough prediction entries in the beginning
	MatrixXd vs; //self velocity in the flying frame
	double omegas[MAX_STAMPS]; //self yaw angular velocity in the flying frame
	MatrixXd xs; //saved states and covariances in the flying frame
	MatrixXd* Ps;
} egomotion;

//initialize egomotion estimation variables
void egomotion_init(double dt, double dt_max, double w_th, double w_roll, double w_pitch, double w_change, double w_yaw, double v_Z, double v_OF, double lag, double mass, double calib_slope, double calib_bias);

//initialize an egomotion structure
egomotion* egomotion_init_structure();

//egomotion predictions
void egomotion_predict_self();

//egomotion updates
void egomotion_update_Z_self(double z);
void egomotion_update_OF_self(double vx, double vy); //NOTE: we assume these velocities to be in the flying frame

//convert to flying frame (some problems will occurr if the with bad roll and pitch)
void egomotion_convert_to_flying_frame(VectorXd& x, MatrixXd& P);

//adds the egomotion prediction (should be the last predict to be made)
void egomotion_predict(egomotion* ego, VectorXd& x1, MatrixXd& P1);

//callbacks
void angle_callback(const sensor_msgs::Imu::ConstPtr& imu);

void control_callback_px4(const mavros_msgs::AttitudeTarget::ConstPtr& command);

//generate egomotion ros message
quad_msgs::EstimateSingle* egomotion_get_ros_message();

//get roll, pitch, yaw
double get_slef_roll();
double get_self_pitch();
double get_self_yaw();

#endif
//__EGOMOTION__
