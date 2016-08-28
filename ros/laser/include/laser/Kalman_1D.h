#ifndef _COMMON_KALMAN_1D_
#define _COMMON_KALMAN_1D_
/*
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <math.h>
using namespace Eigen;
*/

//Implementation found on: https://malcommielle.wordpress.com/2015/04/29/kalman-filter/
class Kalman_1D{
	private: //Define variables here

		float Q; //Process Noise Covariance
		float R; //Measurement Noise Covariance
		float P; //Estimation Error Covariance

		float z; //Measurement
		float K; //Kalman gain

	public: //Define getter and setter methods here
		Kalman_1D(float _Q, float _R, float _P, float _z);
		float KalmanFilter(float measurement);

};
#endif