// ROS includes
#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// ROS message includes
#include "std_msgs/String.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

// ROS dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>

// Networking includes
#include <netdb.h>
#include <strings.h>
#include <arpa/inet.h>

// C++ includes
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

// Asctec Communication library
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/DoubleArrayStamped.h>
#include <asctec_hl_comm/mav_status.h>

// PosCtrl Debug Library
#include <HbPosCtrl/ctrl_debug.h>

// Pose calculation including
#include "posctrl.h"

// Global variables
PosCtrl hbcontrol;          // Class for controlling the Hummingbird

enum{
    RESET = 0,
    WAIT  = 1,
    REFTRAJECT = 3,
    CTRL  = 4
};



// Callback functions for subscriptions
void newDebugData(asctec_hl_comm::DoubleArrayStamped msg)
{
    hbcontrol.debugNewData(msg);
}

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


// Main function
int main(int argc, char **argv)
{
    ROS_INFO("Initialising TCP node");
    //----------------------------------------------------------------
    // Init the node and give back a handle
    ros::init(argc, argv, "ctrl_node");
    ros::NodeHandle nh;

    // Advertise for the mav control
    ros::Publisher ctrl = nh.advertise<asctec_hl_comm::mav_ctrl> ("fcu/control", 1000);
    // Advertise the integrator for the mav
    ros::Publisher ctrldebug = nh.advertise<HbPosCtrl::ctrl_debug> ("ctrl/debug", 1000);

    // Subscribe to debug data from ssdk
    ros::Subscriber ssdk = nh.subscribe("fcu/debug", 1000, newDebugData);
    // Subscribe to status for resetting the integrator
    ros::Subscriber status = nh.subscribe("fcu/status", 1000, newStatusData);

    printTopicInfo();


    //-----------------------------------------------------------------
    // Set Ros frequency and start loop
    ros::Rate loop_rate(100);    // Infinite loop frequency
    int count = 0;              // Loop counter
    int fsmstate = RESET;
    int cycle_counter = 500;    // 500 cycles will be 5 secs
    ROS_INFO("Going to RESET");

    //-----------------------------------------------------------------
    // Main function
    while (ros::ok())
    {
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
        case CTRL :
            if (hbcontrol.status.serial_interface_enabled) {
                //................................................//
                // Calculate the control commands based on the invers dynamic model
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

        // Time stamp the control output
        hbcontrol.ctrl.header.stamp = ros::Time::now();
        hbcontrol.ctrldebug.header.stamp = ros::Time::now();

        //................................................//
        //Publish the control message
        ctrl.publish(hbcontrol.ctrl);
        ctrldebug.publish(hbcontrol.ctrldebug);

        ros::spinOnce();

        //printf("Controls: x=%f, y=%f, z=%f\n", hbcontrol.ctrl.x, hbcontrol.ctrl.y, hbcontrol.ctrl.z);


        //................................................//
        // Adjust loop timing and iteration counter
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

// END OF FILE
