#ifndef _COMMON_PID_
#define _COMMON_PID_
//Discrete PID
//cnr437@gmail.com

/*Example in Python from the original*/

//p=PID(3.0,0.4,1.2)
//p.setPofloat(5.0)
//while True:
//     pid = p.update(measurement_value)


//Translated to C++ by Pedro Abreu
class PID{
	private: //Define variables here
		float Kp;
		float Ki;
		float Kd;
		float Derivator;
		float Integrator;
		float Integrator_max;
		float Integrator_min;

		float set_point;
		float error;
	public: //Define getter and setter methods here
		PID(float _Kp=2.0, float _Ki=0.0, float _Kd=1.0, float _Derivator=0.0, float _Integrator=0.0, float _Integrator_max=500.0, float _Integrator_min=-500.0);
		float update(float current_value);
		void setPoint(float _set_pofloat);
		void setIntegrator(float _Integrator);
		void setDerivator(float _Derivator);
		void setKp(float _Kp);
		void setKi(float _Ki);
		void setKd(float _Kd);
		float getPoint();
		float getError();
		float getIntegrator();
		float getDerivator();
};
#endif