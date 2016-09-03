#include "posctrl.h"

// C++ includes
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// Always use SI units !!
#define MASS    0.70        // [kg]
#define GRAVITY 9.81        // [m/s]
#define MAXTHRUST 3.8       // [N] of thrust for one propeller of Hummingbird
#define MINTHRUST 0.1       // [N] thrust for one propeller in idle mode for Hummingbird
#define MAXANGLE  0.35      // equal to 20 deg maximum angle
#define MAXACCELERATION 4   // [m/s^2] Maximum xy acceleration
#define MAXFORCE  2.8       // [N] force = 4*0.7
#define MAXVELOCITY 0.2     // [m/s] in xy direction
#define ZMAXACC   2       	// [m/s^2]
#define SAMPLES 200         // Samples for the reference trajectory

#define PI      3.14159265

PosCtrl::PosCtrl()
{
    // Definition of the PID paramters
    reg.px = 8.2;
    reg.py = 8.2;
    reg.pz = 7;
    reg.vx = 6.4;
    reg.vy = 6.4;
    reg.vz = 5;
    reg.ix = 75;
    reg.iy = 75;
    reg.iz = 50;

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

void PosCtrl::updateParameters()
{
    reg.px = params.px;
    reg.py = params.py;
    reg.pz = params.pz;
    reg.vx = params.vx;
    reg.vy = params.vy;
    reg.vz = params.vz;
    reg.ix = params.ix;
    reg.iy = params.iy;
    reg.iz = params.iz;
}

void PosCtrl::compute()
{
    // Data Filtering
    filterData();

    // Calculate the desired force and the thrust
    double Fdes[3];
    double Thrust;
    // Compute the desired vector in the shifted frame ENU (front, left, up)

    if (refidx < t.size()) {
        // Integrator calculation based on reference
        intx_curr = ((x[refidx]-position.x) / reg.ix) + intx_last;
        inty_curr = ((y[refidx]-position.y) / reg.iy) + inty_last;

        Fdes[0] = fx[refidx] + reg.px*(x[refidx] - position.x) + reg.vx*(vx[refidx] - velocity.x);
        Fdes[1] = fy[refidx] + reg.py*(y[refidx] - position.y) + reg.vy*(vy[refidx] - velocity.y);
    }
    else {
        // Integrator calculation based on setpoint
        intx_curr = ((sp.x-position.x) / reg.ix) + intx_last;
        inty_curr = ((sp.y-position.y) / reg.iy) + inty_last;

        Fdes[0] = reg.px*(sp.x - position.x) + reg.vx*(sp.vx - velocity.x);
        Fdes[1] = reg.py*(sp.y - position.y) + reg.vy*(sp.vy - velocity.y);
    }
    integrator_curr = ((sp.z-position.z) / reg.iz) + integrator_last;
    Fdes[2] = reg.pz*(sp.z-position.z) + reg.vz*(sp.vz-velocity.z) + integrator_curr + MASS*GRAVITY;

    // Security test for height
    if ((position.z < 0.7*sp.z) || (position.z > 1.3*sp.z)) {
        Fdes[2] = MASS*GRAVITY;     // Prevents from giving full thrust if estimated height is zero
        integrator_curr = 0;
    }
    if (Fdes[2] > 15) {
        Fdes[2] = 15;
    }

    // Desired up force: Dot product between desired force and current orientation
    Thrust = sqrt(Fdes[0]*Fdes[0] + Fdes[1]*Fdes[1] + Fdes[2]*Fdes[2]); // Thrust in [N]
    // From the dynamic model: thrust = Fdes[3]/(cos(roll)*cos(pitch))

    // Integrator for x and y ( after thrust calculation !!!)
    Fdes[0] = Fdes[0] + intx_curr;
    Fdes[1] = Fdes[1] + inty_curr;

    // Antireset-windup for xy Force
    if (refidx < t.size()) {
        // Recalculation based on reference model
        if (Fdes[0] > MAXFORCE) {
            Fdes[0] = MAXFORCE;
            intx_curr = MAXFORCE - reg.px*(x[refidx]-position.x) - reg.vx*(vx[refidx]-velocity.x);
            //ROS_INFO("  ARW active for x");
        }
        else if (Fdes[0] < -MAXFORCE)
        {
        	Fdes[0] = -MAXFORCE;
        	intx_curr = -MAXFORCE - reg.px*(x[refidx]-position.x) - reg.vx*(vx[refidx]-velocity.x);
        	//ROS_INFO("  ARW active for x");
        }
        if (Fdes[1] > MAXFORCE) {
            Fdes[1] = MAXFORCE;
            inty_curr = MAXFORCE - reg.py*(y[refidx]-position.y) - reg.vy*(vy[refidx]-velocity.y);
            //ROS_INFO("  ARW active for y");
        }
        else if (Fdes[1] < -MAXFORCE) {
        	Fdes[1] = -MAXFORCE;
        	inty_curr = -MAXFORCE - reg.py*(y[refidx]-position.y) - reg.vy*(vy[refidx]-velocity.y);
        	//ROS_INFO("  ARW active for y");

        }
    }
    else {
        // Recalculation based on Setpoint
        if (Fdes[0] > MAXFORCE) {
            Fdes[0] = MAXFORCE;
            intx_curr = MAXFORCE - reg.px*(sp.x-position.x) - reg.vx*(sp.vx-velocity.x);
            //ROS_INFO("  ARW active for x");
        }
        else if (Fdes[0] < -MAXFORCE)
        {
        	Fdes[0] = -MAXFORCE;
        	intx_curr = -MAXFORCE - reg.px*(sp.x-position.x) - reg.vx*(sp.vx-velocity.x);
        	//ROS_INFO("  ARW active for x");
        }
        if (Fdes[1] > MAXFORCE) {
        	Fdes[1] = MAXFORCE;
        	inty_curr = MAXFORCE - reg.py*(sp.y-position.y) - reg.vy*(sp.vy-velocity.y);
        	//ROS_INFO("  ARW active for y");
        }
        else if (Fdes[1] < -MAXFORCE) {
        	Fdes[1] = -MAXFORCE;
        	inty_curr = -MAXFORCE - reg.py*(sp.y-position.y) - reg.vy*(sp.vy-velocity.y);
        	//ROS_INFO("  ARW active for y");
        }
    }

    // Iterate the integrartor
    intx_last = intx_curr;
    inty_last = inty_curr;
    integrator_last = integrator_curr;
    // Iterate the Reference count parameter
    if (refidx < t.size()) {
        refidx++;
        printf("Reference index: %d\n", refidx);
    }

    std::cout << "PX: " << reg.px << std::endl;

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
    // TODO: Find a regulator for yaw. With yawcmd=0.0, it stabilises well around current yaw

    // Up to here, ENU convention

    // Security for thrust and ARW: limit to 55%
    if (thrustcmd > 0.55)
    {
        thrustcmd = 0.55;
    }

    // Assign the commands to the command structure
    //ctrl.x = pitchcmd;
    ctrl.x = 0.0;       // For testing height control
    //ctrl.y = -rollcmd;
    ctrl.y = 0.0;       // For testing height control
    ctrl.z = thrustcmd;
    ctrl.yaw = yawcmd;

    // Type 1 means that the commands are interpreted as angles and thrust
    // Type 2 would be interpreted as velocities
    // Type 3 would be interpreted as positions -> see mav_ctrl message documentation
    ctrl.type = 1;

    // Logging all the control parameters for debug reason
    ctrldebug.epos.x = sp.x-position.x;
    ctrldebug.epos.y = sp.y-position.y;
    ctrldebug.epos.z = sp.z-position.z;
    ctrldebug.evel.x = sp.vx-velocity.x;
    ctrldebug.evel.y = sp.vy-velocity.y;
    ctrldebug.evel.z = sp.vz-velocity.z;
    ctrldebug.fdes.x = Fdes[0];
    ctrldebug.fdes.y = Fdes[1];
    ctrldebug.fdes.z = Fdes[2];
    ctrldebug.acc.x = accx;
    ctrldebug.acc.y = accy;
    ctrldebug.acc.z = Fdes[2]/MASS;
    ctrldebug.accb.x = accxb;
    ctrldebug.accb.y = accyb;
    ctrldebug.accb.z = Fdes[2]/MASS;
    ctrldebug.accref.x = ax[refidx-1];
    ctrldebug.accref.y = ay[refidx-1];
    ctrldebug.velref.x = vx[refidx-1];
    ctrldebug.velref.y = vy[refidx-1];
    ctrldebug.posref.x = x[refidx-1];
    ctrldebug.posref.y = y[refidx-1];
    ctrldebug.angles.x = roll;
    ctrldebug.angles.y = pitch;
    ctrldebug.angles.z = yaw;
    ctrldebug.integrator.x = intx_curr;
    ctrldebug.integrator.y = inty_curr;
    ctrldebug.integrator.z = integrator_curr;
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

    // Filter velocity @15Hz
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
    // clear vectors
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
    x.push_back(posraw.x-sp.x);    // Take raw position from debug data
    vx.push_back(velocity.x-sp.vx);// Take velocity = 0;
    ax.push_back(0);
    fx.push_back(0);

    y.push_back(posraw.y-sp.y);    // Take raw position from debug data
    vy.push_back(velocity.y-sp.vy);// Take velocity = 0;
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

    ROS_INFO(" Reference trajectory computed");

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

    ROS_INFO(" Filter initialised at position (%lf, %lf, %lf)", posraw.x, posraw.y, posraw.z);
}


// Callback function when new debuf data arrives
void PosCtrl::debugNewData(asctec_hl_comm::DoubleArrayStamped msg)
{
    //ROS_INFO("NewDebugData");
    // Position estimation seen from the world frame
    posraw.x = msg.data[13];;
    posraw.y = -msg.data[14];
    posraw.z = -msg.data[15];
    // Velocity estimation seen from the world frame
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

    yaw = -yaw;  // Yaw has to be inverted for ENU

}

// Callback function when new status data arrives
void PosCtrl::statusNewData(asctec_hl_comm::mav_status msg)
{
    // Status is important to know when serial interface is enabled
    status = msg;
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
