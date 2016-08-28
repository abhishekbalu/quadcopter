#ifndef POSCTRL_H
#define POSCTRL_H

// C++ includes
#include <vector>
#include <stdlib.h>
#include <stdio.h>

// ROS includes
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ROS message includes
#include "std_msgs/String.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>

// ROS dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>

// Asctec Communication library
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/DoubleArrayStamped.h>
#include <asctec_hl_comm/mav_status.h>

// PosCtrl Debug Library
//#include "HbPosCtrl/ctrl_debug.h"

// Structure for returning an array from a function
struct arr3 {
    float arr[3];
};

// Regulator class for position and velocity
class Regulator
{
public:
    Regulator() : px(0), py(0), pz(0), vx(0), vy(0), vz(0), pyaw(0), vyaw(0)
    {}
    float px;
    float py;
    float pz;
    float vx;
    float vy;
    float vz;
    float ix;
    float iy;
    float iz;
    float pyaw;
    float vyaw;
};


// Setpoint class for position and velocity
class Setpoint
{
public:
    Setpoint() : x(0), y(0), z(1.5), vx(0), vy(0), vz(0), yaw(0), vyaw(0)
    {}
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float yaw;
    float vyaw;
};

// Class for reference trajectory
class Trajectory
{
public:
    Trajectory()
    {}
    float p;
    float v;
    float a;
    float f;
};

// Lowpass filter class
class LPFilter
{
public:
    LPFilter()
    {}
    float yn;
    float ylast;
    float xn;
    float xlast;

    // Filter at 1 Hz
    float filter1(float in)
    {
        xn = in;
        yn = 0.03046*xn + 0.03046*xlast + 0.9391*ylast;
        xlast = xn;
        ylast = yn;
        return yn;
    }
    // Filter at 2 Hz
    float filter2(float in)
    {
        xn = in;
        yn = 0.05912*xn + 0.05912*xlast + 0.8818*ylast;
        xlast = xn;
        ylast = yn;
        return yn;
    }
    // Filter at 5 Hz
    float filter5(float in)
    {
        xn = in;
        yn = 0.1358*xn + 0.1358*xlast + 0.7285*ylast;
        xlast = xn;
        ylast = yn;
        return yn;
    }
    // Filter at 10 Hz
    float filter10(float in)
    {
        xn = in;
        yn = 0.2391*xn + 0.2391*xlast + 0.5219*ylast;
        xlast = xn;
        ylast = yn;
        return yn;
    }
    // Filter at 15 Hz
    float filter15(float in)
    {
        xn = in;
        yn = 0.3203*xn + 0.3203*xlast + 0.3594*ylast;
        xlast = xn;
        ylast = yn;
        return yn;
    }
};


// Velocity by numeric position derivative
class PosDerivative
{
public:
    PosDerivative() : posn(0), poslast(0), diffn(0), ts(0.01)
    {}
    float posn;
    float poslast;
    float diffn;
    float ts;
    LPFilter lowpass;

    float derive(float input)
    {
        posn = input;
        diffn = (posn - poslast)/ts;
        diffn = lowpass.filter1(diffn);
        poslast = posn;
        return diffn;
    }

};

// Main class for the position control
class PosCtrl
{
public:
    PosCtrl();
    ~PosCtrl();

    // Compute the control commands
    void compute();
   // Filter position data
    void filterData();
    // Send idle commands
    void idle();
    // Compute reference trajectory
    void trajectory();

    // Input data callbacks
    void debugNewData(asctec_hl_comm::DoubleArrayStamped msg);  // position and velocity estimates in world ENU
    void statusNewData(asctec_hl_comm::mav_status msg);         // Status of the flight control unit

    // Control structure for asctec Hummingbird
    asctec_hl_comm::mav_ctrl ctrl;
    // Control debug structure
    HbPosCtrl::ctrl_debug ctrldebug;
    // Private status structure
    asctec_hl_comm::mav_status status;

private:
    // Integrator for each axis
    double integrator_last;
    double integrator_curr;
    double intx_last;
    double inty_last;
    double intx_curr;
    double inty_curr;

    // Low Pass filters
    LPFilter filtx;
    LPFilter filty;
    LPFilter filtz;

    LPFilter filtvx;
    LPFilter filtvy;
    LPFilter filtvz;

    // Position derivatives
    PosDerivative drvx;
    PosDerivative drvy;
    PosDerivative drvz;

    // Parameters from the debug variables
    geometry_msgs::Point      position;
    geometry_msgs::Quaternion orientation;
    geometry_msgs::Vector3    velocity;

    geometry_msgs::Point      posraw;
    geometry_msgs::Point      velraw;

    // Reference Trajectory
    std::vector<float> t;

    std::vector<float> x;
    std::vector<float> vx;
    std::vector<float> ax;
    std::vector<float> fx;

    std::vector<float> y;
    std::vector<float> vy;
    std::vector<float> ay;
    std::vector<float> fy;

    unsigned int refidx;

    // Regulation parameters and setpoint definition
    Regulator K;
    Setpoint Des;

    // Attitude angles
    double roll, pitch, yaw;
    double rollcmd, pitchcmd, yawcmd, thrustcmd;

    // Remap thrust
    double RemapThrust(double th);

    // Rotation Matrix (obsolete)
    float R[3][3];

    // Helper functions
    // Quaternion multiplicaiton (obsolete)
    geometry_msgs::Quaternion quaternionMultiplication(geometry_msgs::Quaternion p, geometry_msgs::Quaternion q);
    // Quaternion rotation (obsolete)
    struct arr3 quaternionRotation(geometry_msgs::Quaternion v, geometry_msgs::Quaternion R);
    //rotation Matrix from Roll Pitch Yaw (obsolete)
    void ComputeRotMat();

};

#endif // POSCTRL_H
