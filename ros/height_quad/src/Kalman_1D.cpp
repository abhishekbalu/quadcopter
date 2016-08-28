#include <height_quad/Kalman_1D.h>
//using namespace Eigen;
Kalman_1D::Kalman_1D(float _Q, float _R, float _P, float _z){
	Q = _Q;
	R = _R;
	P = _P;
	z = _z;
}


float Kalman_1D::KalmanFilter(float measurement){
	/*Pedro Resende implementation

	z = A * z;
	P = A * P * A.transpose() + Q;

	K = (P * H.transpose())/(H * P * H.transpose() + R);
	z = z + K * (measurement - H * z);
	P = (Matrix4f::Identity() - K * H) * P;*/

	//Prediction Update (Time equation)
	P = P + Q;

	//Measurement Update (Measurement equation)
	K = P / (P + R);
	z = z + K * (measurement - z);
	P = (1 - K) * P;

	return z;
}

