//============================================================================
// Name        : HB_Control.cpp
// Author      : Ivan Ovinnikov
// Version     :
// Description : ROS controller package for the Hummingbird autonomous landing system
//============================================================================

// ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ROS message includes
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

// ROS dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>

// C++ includes
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

// Asctec Communication library
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/DoubleArrayStamped.h>
#include <asctec_hl_comm/mav_status.h>

#include "posctrl.h"
#include <hb_control/hb_control_paramsConfig.h>

/* =============================================== */

PosCtrl hbcontrol; // class for Hummingbird control

/* The FSM enum */
enum
{
    RESET = 0,
    WAIT  = 1,
    REFTRAJECT = 2,
    CTRL  = 3
};

// Callback functions for subscriptions
/* new observer data */
void newSSDKData(asctec_hl_comm::DoubleArrayStamped msg)
{
    hbcontrol.debugNewData(msg);
}
/* new quad status data */
void newStatusData(asctec_hl_comm::mav_status msg)
{
    hbcontrol.statusNewData(msg);
}

//  print published/subscribed topics
void printTopicInfo()
{
    std::string nodeName = ros::this_node::getName();
    ros::V_string topics;
    ros::this_node::getSubscribedTopics(topics);
    std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
    for (unsigned int i = 0; i < topics.size(); i++)
        topicsStr += ("\t\t" + topics.at(i) + "\n");

    topicsStr += "\tadvertised topics:\n";
    ros::this_node::getAdvertisedTopics(topics);
    for (unsigned int i = 0; i < topics.size(); i++)
        topicsStr += ("\t\t" + topics.at(i) + "\n");

    ROS_INFO_STREAM(""<< topicsStr);
}

void configCallback(hb_control::hb_control_paramsConfig &config, uint32_t level)
{
    ROS_INFO("DynRec callback");
    hbcontrol.params.px = config.px;
    hbcontrol.params.py = config.py;
    hbcontrol.params.pz = config.pz;
    hbcontrol.params.vx = config.vx;
    hbcontrol.params.vy = config.vy;
    hbcontrol.params.vz = config.vz;
    hbcontrol.params.ix = config.ix;
    hbcontrol.params.iy = config.iy;
    hbcontrol.params.iz = config.iz;
}

int main(int argc, char ** argv)
{
	ROS_INFO("Initialising HB Control Node");

	ros::init(argc,argv,"hb_control");
	ros::NodeHandle nh;

	/* Publish the control signal to fcu/control where it gets read by the asctec framework */
	ros::Publisher ctrl = nh.advertise<asctec_hl_comm::mav_ctrl>("fcu/control", 1000);
	/* Publish additional control debug info to HB_Control/debug where it gets read by the asctec framework */
	ros::Publisher ctrldebug = nh.advertise<HB_Control::ctrl_debug> ("hb_control/debug", 1000);
	/* Subscribe to the estimator output and the quadrotor status */
	ros::Subscriber ssdk = nh.subscribe("fcu/debug", 1000, newSSDKData);
	ros::Subscriber status = nh.subscribe("fcu/status", 1000, newStatusData);

	// Dynamic reconfigure stuff for the parameters	
	dynamic_reconfigure::Server<hb_control::hb_control_paramsConfig> drsrv;
	dynamic_reconfigure::Server<hb_control::hb_control_paramsConfig>::CallbackType cb;
	cb = boost::bind(&configCallback, _1, _2);
	drsrv.setCallback(cb);

	// expose the parameters to the interface
	printTopicInfo();

	/* Set ROS frequency and start loop */
	ros::Rate loop_rate(100);   // 100Hz loop frequency
	int count = 0;              // Loop counter
	int fsmstate = RESET;		// default FSM state
	int cycle_counter = 500;    // 500 cycles will be 5 secs
	ROS_INFO("Going to RESET");

	/* Main loop */
	while (ros::ok())
	{
		hbcontrol.updateParameters();
		if(count % 100 == 0)
		   ROS_INFO("Params debug: %f, %f, %f", hbcontrol.params.px, hbcontrol.params.py, hbcontrol.params.pz);
		switch (fsmstate) {
		case RESET :
			//
			if (hbcontrol.status.serial_interface_enabled) {
				cycle_counter = 500;
				fsmstate = WAIT;
				ROS_INFO("Going to WAIT");
			}
			else {
				fsmstate = RESET;
				hbcontrol.idle();
			}
			break;
		case WAIT :
			if (hbcontrol.status.serial_interface_enabled) {
				if (cycle_counter <= 0) {
					fsmstate = REFTRAJECT;
					ROS_INFO("Going to REFTRAJECT");
				}
				else {
					cycle_counter--;
					fsmstate = WAIT;
					hbcontrol.idle();
				}
			}
			else {
				cycle_counter = 500;
				fsmstate = RESET;
			}
			break;
		case REFTRAJECT :
			hbcontrol.trajectory();
			fsmstate = CTRL;
			ROS_INFO("Going to CTRL");
			break;
		case CTRL :
			if (hbcontrol.status.serial_interface_enabled) {
				// Calculate the control commands based on the inverse dynamic model
				hbcontrol.compute();
			}
			else {
				fsmstate = RESET;
				hbcontrol.idle();
				ROS_INFO("Going to RESET");
			}
			break;
		default:
			ROS_WARN("FSM in undefined state: resetting");
			fsmstate = RESET;
            hbcontrol.idle();
        }
    
        // Time stamp the control output and publish
        hbcontrol.ctrl.header.stamp = ros::Time::now();
        ctrl.publish(hbcontrol.ctrl);

        /* control debug message */
        hbcontrol.ctrldebug.header.stamp = ros::Time::now();
        ctrldebug.publish(hbcontrol.ctrldebug);

        ros::spinOnce();
        // Adjust loop timing and iteration counter
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
