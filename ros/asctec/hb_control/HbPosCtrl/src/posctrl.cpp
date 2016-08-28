#include "posctrl.h"

// C++ includes
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// Always use SI units !!
#define MASS    0.70        // [kg]
#define GRAVITY 9.81        // [m/s]
//#define MAXTHRUST 8.0     // [N] of thrust for one propeller of Pelican
//#define MINTHRUST 0.25    // [N] thrust for one propeller in idle mode for Pelican
#define MAXTHRUST 3.8       // [N] of thrust for one propeller of Hummingbird
#define MINTHRUST 0.1       // [N] thrust for one propeller in idle mode for Hummingbird
#define MAXANGLE  0.35      // equal to 20 deg maximum angle
#define MAXACCELERATION 4   // [m/s^2] Maximum xy acceleration
#define MAXFORCE  2.8       // [N] force = 4*0.7
#define MAXVELOCITY 0.2     // [m/s] in xy direction

#define ZMAXACC   2       // [m/s^2]

#define SAMPLES 200         // Samples for the reference trajectory

#define PI      3.14159265

PosCtrl::PosCtrl()
{
    // Definition of the PID paramters
    K.px = 8.2;
    K.py = 8.2;
    K.pz = 7;
    K.vx = 6.4;
    K.vy = 6.4;
    K.vz = 5;
    K.ix = 75;
    K.iy = 75;
    K.iz = 50;

    // Initialise the integrator to zero
    intx_curr = 0.0;
    inty_curr = 0.0;
    intx_last = 0.0;
    inty_last = 0.0;
    integrator_last = 0.0;
    integrator_curr = 0.0;
}

PosCtrl::~PosCtrl()
{

}

void PosCtrl::compute()
{
    // Data Filtering
    filterData();

    //.........................................................//
    // Calculate the desired force and the thrust
    double Fdes[3];
    double Thrust;
    // Compute the desired vector in the shifted frame ENU (front, left, up)

    if (refidx < t.size()) {
        // Integrator calculation based on reference
        intx_curr = ((x[refidx]-position.x) / K.ix) + intx_last;
        inty_curr = ((y[refidx]-position.y) / K.iy) + inty_last;

        Fdes[0] = fx[refidx] + K.px*(x[refidx] - position.x) + K.vx*(vx[refidx] - velocity.x);
        Fdes[1] = fy[refidx] + K.py*(y[refidx] - position.y) + K.vy*(vy[refidx] - velocity.y);
    }
    else {
        // Integrator calculation based on setpoint
        intx_curr = ((Des.x-position.x) / K.ix) + intx_last;
        inty_curr = ((Des.y-position.y) / K.iy) + inty_last;

        Fdes[0] = K.px*(Des.x - position.x) + K.vx*(Des.vx - velocity.x);
        Fdes[1] = K.py*(Des.y - position.y) + K.vy*(Des.vy - velocity.y);
    }
    integrator_curr = ((Des.z-position.z) / K.iz) + integrator_last;
    Fdes[2] = K.pz*(Des.z-position.z) + K.vz*(Des.vz-velocity.z) + integrator_curr + MASS*GRAVITY;

    // Security test for height
    if ((position.z < 0.7*Des.z) || (position.z > 1.3*Des.z)) {
        Fdes[2] = MASS*GRAVITY;     // Prevents from giving full thrust if estimated height is zero
        integrator_curr = 0;
    }
    if (Fdes[2] > 15) {
        Fdes[2] = 15;
    }

    // Desired up force: Dot product between desired force and current orientation
    Thrust = sqrt(Fdes[0]*Fdes[0] + Fdes[1]*Fdes[1] + Fdes[2]*Fdes[2]);
    // Thrust in [N]
    // From the dynamic model: thrust = Fdes[3]/(cos(roll)*cos(pitch))

    // Integrator for x and y ( after thrust calculation !!!)
    Fdes[0] = Fdes[0] + intx_curr;
    Fdes[1] = Fdes[1] + inty_curr;

    //.........................................................//
    // Antireset-windup for xy Force
    if (refidx < t.size()) {
        // Recalculation based on reference model
        if (Fdes[0] > MAXFORCE) {
            Fdes[0] = MAXFORCE;
            intx_curr = MAXFORCE - K.px*(x[refidx]-position.x) - K.vx*(vx[refidx]-velocity.x);
            ROS_INFO("  ARW active for x");
        } else {
            if (Fdes[0] < -MAXFORCE) {
                Fdes[0] = -MAXFORCE;
                intx_curr = -MAXFORCE - K.px*(x[refidx]-position.x) - K.vx*(vx[refidx]-velocity.x);
                ROS_INFO("  ARW active for x");
            }
        }
        if (Fdes[1] > MAXFORCE) {
            Fdes[1] = MAXFORCE;
            inty_curr = MAXFORCE - K.py*(y[refidx]-position.y) - K.vy*(vy[refidx]-velocity.y);
            ROS_INFO("  ARW active for y");
        } else {
            if (Fdes[1] < -MAXFORCE) {
                Fdes[1] = -MAXFORCE;
                inty_curr = -MAXFORCE - K.py*(y[refidx]-position.y) - K.vy*(vy[refidx]-velocity.y);
                ROS_INFO("  ARW active for y");
            }
        }
    }
    else {
        // Recalculation based on Setpoint
        if (Fdes[0] > MAXFORCE) {
            Fdes[0] = MAXFORCE;
            intx_curr = MAXFORCE - K.px*(Des.x-position.x) - K.vx*(Des.vx-velocity.x);
            ROS_INFO("  ARW active for x");
        } else {
            if (Fdes[0] < -MAXFORCE) {
                Fdes[0] = -MAXFORCE;
                intx_curr = -MAXFORCE - K.px*(Des.x-position.x) - K.vx*(Des.vx-velocity.x);
                ROS_INFO("  ARW active for x");
            }
        }
        if (Fdes[1] > MAXFORCE) {
            Fdes[1] = MAXFORCE;
            inty_curr = MAXFORCE - K.py*(Des.y-position.y) - K.vy*(Des.vy-velocity.y);
            ROS_INFO("  ARW active for y");
        } else {
            if (Fdes[1] < -MAXFORCE) {
                Fdes[1] = -MAXFORCE;
                inty_curr = -MAXFORCE - K.py*(Des.y-position.y) - K.vy*(Des.vy-velocity.y);
                ROS_INFO("  ARW active for y");
            }
        }
    }

    //.........................................................//
    // Iterate the integrartor
    intx_last = intx_curr;
    inty_last = inty_curr;
    integrator_last = integrator_curr;
    // Iterate the Reference count parameter
    if (refidx < t.size()) {
        refidx++;
        printf("Reference index: %d\n", refidx);
    }

    //.........................................................//
    // Compute acceleration in x and y
    double accx, accy, accxb, accyb;
    accx = Fdes[0]/MASS;
    accy = Fdes[1]/MASS;

    // Rotate acceleration around yaw
    //accxb = cos(yaw)*accx + sin(yaw)*accy;
    //accyb = -sin(yaw)*accx + cos(yaw)*accy;

    // Roll and Pitch commands
    rollcmd = atan(-accy/GRAVITY);               // Roll input in [rad]
    pitchcmd = atan(accx/GRAVITY);             // Pitch input in [rad]
    thrustcmd = RemapThrust(Thrust);            // 0.0 - 1.0
    yawcmd = 0.0;                               // Yaw velocity in [rad/s]
    // TODO: Find a regulator for yaw. With yawcmd=0.0, it sabilises well around
    // the yaw he alreay is

    // Up to here, ENU convention
    //---------------------------------------------------------------

    // Security for thrust and ARW: limit to 80%
    if (thrustcmd > 0.8)
    {
        thrustcmd = 0.8;
    }

    // Assign the commands to the command structure
    ctrl.x = pitchcmd;
    //ctrl.x = 0.0;       // For testing height control
    ctrl.y = -rollcmd;
    //ctrl.x = 0.0;       // For testing height control
    ctrl.z = thrustcmd;
    ctrl.yaw = yawcmd;

    ctrl.type = 1;      // Type 1 means that the commands are interpreted as angles and thrust
                        // Type 2 would be interpreted as velocities
                        // Type 3 would be interpreted as positions

    //..........................................................//
    // Logging all the control parameters for debug reason
//    ctrldebug.epos.x = position.x; //Des.x-position.x;
//    ctrldebug.epos.y = position.y; //Des.y-position.y;
//    ctrldebug.epos.z = position.z; //Des.z-position.z;
//    ctrldebug.evel.x = velocity.x; //Des.vx-velocity.x;
//    ctrldebug.evel.y = velocity.y; //Des.vy-velocity.y;
//    ctrldebug.evel.z = velocity.z; //Des.vz-velocity.z;
//    ctrldebug.fdes.x = Fdes[0];
//    ctrldebug.fdes.y = Fdes[1];
//    ctrldebug.fdes.z = Fdes[2];
//    ctrldebug.acc.x = accx;
//    ctrldebug.acc.y = accy;
//    ctrldebug.acc.z = Fdes[2]/MASS;
//    ctrldebug.accb.x = accxb;
//    ctrldebug.accb.y = accyb;
//    ctrldebug.accb.z = Fdes[2]/MASS;
//    ctrldebug.accref.x = ax[refidx-1];
//    ctrldebug.accref.y = ay[refidx-1];
//    ctrldebug.velref.x = vx[refidx-1];
//    ctrldebug.velref.y = vy[refidx-1];
//    ctrldebug.posref.x = x[refidx-1];
//    ctrldebug.posref.y = y[refidx-1];
//    ctrldebug.angles.x = roll;
//    ctrldebug.angles.y = pitch;
//    ctrldebug.angles.z = yaw;
//    ctrldebug.integrator.x = intx_curr;
//    ctrldebug.integrator.y = inty_curr;
//    ctrldebug.integrator.z = integrator_curr;
}


// Filter data
void PosCtrl::filterData()
{
    // Filter x at 10 Hz
    position.x = filtx.filter5(posraw.x);
    position.y = filty.filter5(posraw.y);
    position.z = filtz.filter10(posraw.z);
    //position.z = posraw.z;

    // Derive position
    velocity.x = drvx.derive(position.x);
    velocity.y = drvy.derive(position.y);
    //velocity.z = drvz.derive(position.z);

    // Filter velocity
    velocity.x = filtvx.filter15(velocity.x);
    velocity.y = filtvy.filter15(velocity.y);
    velocity.z = filtvz.filter15(velraw.z);
    //velocity.z = velraw.z;
}

// Send idle commands
void PosCtrl::idle()
{
    // Hold integrators in reset state
    intx_curr = 0.0;
    intx_last = 0.0;
    inty_curr = 0.0;
    inty_last = 0.0;
    integrator_last = 0.0;
    integrator_curr = 0.0;

    ctrl.x = 0.0;   // Set angles to zero
    ctrl.y = 0.0;
    ctrl.z = 0.15;   // Set thrust to 20% for attitude stabilisation to work
    ctrl.yaw = 0.0;   // Set Yaw velocity to zero

    ctrl.type = 1;  // Send as angles and thrust
}

// Compute reference trajectory
void PosCtrl::trajectory()
{
    // Vectors
    t.clear();

    x.clear();
    vx.clear();
    ax.clear();
    fx.clear();

    y.clear();
    vy.clear();
    ay.clear();
    fy.clear();

    // Initial conditions
    x.push_back(posraw.x-Des.x);    // Take raw position from debug data
    vx.push_back(velocity.x-Des.vx);// Take velocity = 0;
    ax.push_back(0);
    fx.push_back(0);

    y.push_back(posraw.y-Des.y);    // Take raw position from debug data
    vy.push_back(velocity.y-Des.vy);// Take velocity = 0;
    ay.push_back(0);
    fy.push_back(0);

    // Parameters for the spring-mass system
    float k = 6.9;
    float b = 4.4;
    float m = 0.7;
    // Starting and ending times
    t.push_back(0);
    int t_end = 3;
    // (small) step size
    float dt = 0.01;
    int idx = 0;

    //for-next loop where the position and velocity are calculated based upon previous
    for (idx = 0; idx <t_end/dt; idx++) {
        // Iterate time
        t.push_back(t[idx] + dt);

        x.push_back(x[idx] + vx[idx]*dt);
        y.push_back(y[idx] + vy[idx]*dt);

        fx.push_back(-k*m*x[idx] -b*m*vx[idx]);
        fy.push_back(-k*m*y[idx] -b*m*vy[idx]);

        ax.push_back(fx[idx+1]/m);
        ay.push_back(fy[idx+1]/m);

        if (ax[idx+1] > MAXACCELERATION) {
            ax[idx+1] = MAXACCELERATION;
            fx[idx+1] = MAXFORCE;
        }
        else {
            if (ax[idx+1] < -MAXACCELERATION) {
                ax[idx+1] = -MAXACCELERATION;
                fx[idx+1] = -MAXFORCE;
            }
        }

        if (ay[idx+1] > MAXACCELERATION) {
            ay[idx+1] = MAXACCELERATION;
            fy[idx+1] = MAXFORCE;
        }
        else {
            if (ay[idx+1] < -MAXACCELERATION) {
                ay[idx+1] = -MAXACCELERATION;
                fy[idx+1] = -MAXFORCE;
            }
        }

        vx.push_back(vx[idx] + ax[idx+1]*dt);
        vy.push_back(vy[idx] + ay[idx+1]*dt);
    }
    refidx = 0;

    ROS_INFO("  Reference trajectory computed");

    // Initialise filter at the current position
    filtx.yn = posraw.x;
    filtx.ylast = posraw.x;
    filtx.xn = posraw.x;
    filtx.xlast =posraw.x;

    filty.yn = posraw.y;
    filty.ylast = posraw.y;
    filty.xn = posraw.y;
    filty.xlast = posraw.y;

    filtz.yn = posraw.z;
    filtz.ylast = posraw.z;
    filtz.xn = posraw.z;
    filtz.xlast = posraw.z;

    ROS_INFO("  Filter initialised at position (%lf, %lf, %lf)", posraw.x, posraw.y, posraw.z);
}


// Callback function when new debuf data arrives
void PosCtrl::debugNewData(asctec_hl_comm::DoubleArrayStamped msg)
{
    //ROS_INFO("NewDebugData");
    // Position estimation seen from the world frame
    posraw.x = msg.data[13];;
    posraw.y = -msg.data[14];
    posraw.z = -msg.data[15];
    // Velocity estimation seen form the world frame
    velraw.x = msg.data[19];
    velraw.y = -msg.data[20];
    velraw.z = -msg.data[21];
    // Data conversion from NED to ENU !!!
    // Attitude angles
    roll = -msg.data[28]; // Roll has to be inverted for ENU
    pitch = msg.data[29];
    yaw = msg.data[30];
    // !!! The yaw from the debug Data is between [0, 2pi]
    if (yaw > PI) {
        yaw = yaw - 2*PI;
    }

    yaw = -yaw;             // Yaw has to be inverted for ENU

}


// Callback function when new status data arrives
void PosCtrl::statusNewData(asctec_hl_comm::mav_status msg)
{
    // Status is important to know when serial interface is enabled
    status = msg;
}


// Quaternion multiplication (obsolete)
geometry_msgs::Quaternion PosCtrl::quaternionMultiplication(geometry_msgs::Quaternion p, geometry_msgs::Quaternion q)
{
    geometry_msgs::Quaternion ret;

    ret.w = p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z;
    ret.x = p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y;
    ret.y = p.w*q.y + p.y*q.w + p.z*q.x - p.x*q.z;
    ret.z = p.w*q.z + p.z*q.w + p.x*q.y - p.y*q.x;

    return ret;
}


// Quaternion rotation (obsolete)
struct arr3 PosCtrl::quaternionRotation(geometry_msgs::Quaternion v, geometry_msgs::Quaternion R)
{
    /*
    change of coordinate from b_frame to c_frame
    v_c = R^1(-pitch, -roll) * v_b = R(pitch, roll) * v_b
    in quaternion notation
    v_c = R*v*R^1
    */
    // Compute the inverse of R
    btQuaternion Rinv;
    geometry_msgs::Quaternion Rinv_msg;
    tf::quaternionMsgToTF(R, Rinv);
    Rinv = Rinv.inverse();
    tf::quaternionTFToMsg(Rinv, Rinv_msg);
    // R*v
    geometry_msgs::Quaternion temp;
    temp = quaternionMultiplication(R, v);
    temp = quaternionMultiplication(temp, Rinv_msg);

    struct arr3 ret;

    ret.arr[0] = temp.x;
    ret.arr[1] = temp.y;
    ret.arr[2] = temp.z;

    return ret;

}


// Compute the rotation matrix (obsolete)
void PosCtrl::ComputeRotMat()
{
    // Convert quaternion to RPY.
    btQuaternion q;
    tf::quaternionMsgToTF(orientation, q);
    btMatrix3x3(q).getRPY(roll, pitch, yaw);

    float c1 = cos(roll);
    float c2 = cos(-pitch);
    float c3 = cos(yaw);
    float s1 = sin(roll);
    float s2 = sin(-pitch);
    float s3 = sin(yaw);

    // Direct cosine matrix
    R[0][0] = c1*c3;
    R[0][1] = s1*s2 - c1*c2*s3;
    R[0][2] = c2*s1 + c1*s3*s2;
    R[1][0] = s3;
    R[1][1] = c3*c2;
    R[1][2] = -c3*s2;
    R[2][0] = -c3*s1;
    R[2][1] = c1*s2 + c2*s1*s3;
    R[2][2] = c1*c2 - s1*s3*s2;
}


// Remap Thrust to output between 0.0 - 0.1
double PosCtrl::RemapThrust(double th)
{
    double output;

    // Assume linear relation around hovering point
    output = (th - 4*MINTHRUST)/(4*MAXTHRUST);

    // Inverse quadratic relation of the form: output = sqrt(th - A)/B;
    // may be closer to reality, but linear approximation works well

    return output;
}

// END OF FILE
