#ifndef _COMMON_KALMAN_2D_
#define _COMMON_KALMAN_2D_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
#include <math.h>
using namespace Eigen;


class Kalman_2D{

	public: //Define predict and update methods here
		VectorXd Xhat; //6, 1 Filtered value
		MatrixXd A; //6, 6
		MatrixXd P; //6, 6 Estimation Error Covariance
		MatrixXd B; //6, 2
		MatrixXd K; //6, 2 Kalman gain

		VectorXd U; //2, 1
		MatrixXd Q; //4,4 Process Noise Covariance
		MatrixXd W; //6,4
		MatrixXd C; //2,6
		MatrixXd R; //2,2 Measurement Noise Covariance

		Kalman_2D(double _Q, double _R, double _W, double _P, double _B, double _A);
		void KalmanPredict();
		VectorXd KalmanUpdate(VectorXd measurement);
		

};
#endif