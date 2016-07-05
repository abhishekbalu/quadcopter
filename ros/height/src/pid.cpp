//Discrete PID
#include <height/pid.h>
PID::PID(float _Kp, float _Ki, float _Kd, float _Derivator, float _Integrator, float _Integrator_max, float _Integrator_min){
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	Derivator = _Derivator;
	Integrator = _Integrator;
	Integrator_max = _Integrator_max;
	Integrator_min = _Integrator_min;

	error = 0.0;
	set_point = 0.0;
}

float PID::update(float current_value){

	error = set_point-current_value;

	P_value = Kp * error;
	D_value = Kd * (error-Derivator); //float - int?
	Derivator = error;


	Integrator = Integrator + error;
	if(Integrator > Integrator_max){
		Integrator = Integrator_max; //Clip maximum
	}else if(Integrator < Integrator_min){
		Integrator = Integrator_min; //Clip minimum
	}

	I_value = Integrator * Ki;

	float PID = P_value + I_value + D_value;

	return PID;
}
void PID::setPoint(float _set_point){
		set_point = _set_point;
		Integrator = 0.0;
		Derivator = 0.0;
}
void PID::setIntegrator(float _Integrator){
	Integrator = _Integrator;
}
void PID::setDerivator(float _Derivator){
	Derivator = _Derivator;
}
void PID::setKp(float _Kp){
	Kp = _Kp;
}
void PID::setKi(float _Ki){
	Ki = _Ki;
}
void PID::setKd(float _Kd){
	Kd = _Kd;
}
float PID::getPoint(){
	return set_point;
}
float PID::getError(){
	return error;
}
float PID::getIntegrator(){
	return Integrator;
}
float PID::getDerivator(){
	return Derivator;
}