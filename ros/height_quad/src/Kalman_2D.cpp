#include <height_quad/Kalman_2D.h>


Kalman_2D::Kalman_2D(double _Q, double _R, double _W, double _P, double _B, double _A){
	//Zero all
	A = MatrixXd::Zero(6,6);
	P = MatrixXd::Zero(6,6);
	B = MatrixXd::Zero(6,2);
	K = MatrixXd::Zero(6,2);
	Xhat = VectorXd::Zero(6);
	U = VectorXd(2);
	Q = MatrixXd(4,4);
	W = MatrixXd(6,4);
 	C = MatrixXd(2,6);
	R = MatrixXd(2,2);

	//Set
	P << _P,0,0,0,0,0,
	     0,_P,0,0,0,0,
	     0,0,_P,0,0,0,
	     0,0,0,_P,0,0,
	     0,0,0,0,_P,0,
	     0,0,0,0,0,_P;
	A << _A,0,0,0,0,0,
	     0,_A,0,0,0,0,
	     0,0,_A,0,0,0,
	     0,0,0,_A,0,0,
	     0,0,0,0,_A,0,
	     0,0,0,0,0,_A;
	B << _B,0,
	     0,_B,
	     _B,0,
	     0,_B,
	     0,0,
	     0,0;
	W << 0,0,0,0,
	     0,0,0,0,
	     0,0,0,0,
	     0,0,0,0,
	     0,0,_W,0,
	     0,0,0,_W;
	Q << _Q,  0,      0,      0,
	     0,     _Q,   0,      0,
	     0,     0,      _Q,   0,
	     0,     0,      0,      _Q;
	R << _R,   0,
	     0,     _R;
	C << 0,0,1,0,0,0,
	     0,0,0,1,0,0;
}


void Kalman_2D::KalmanPredict(){

	//Prediction Update (Time equation)
	Xhat = A * Xhat + B * U;
	P = A * P * A.transpose() + W * Q * W.transpose();

}

VectorXd Kalman_2D::KalmanUpdate(VectorXd measurement){

	//Measurement Update (Measurement equation)
	K = (P * C.transpose()) * (C * P * C.transpose() + R).inverse(); //Kalman Gain
	Xhat = Xhat + K*(measurement - C*Xhat); //Resid = measurement - C*Xhat; 
	P = (Eigen::MatrixXd::Identity(6, 6)-K*C)*P;
	return Xhat;

}