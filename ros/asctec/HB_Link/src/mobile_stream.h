#ifndef DEF_MOBILE_STREAM
#define DEF_MOBILE_STREAM

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <pthread.h>

/*#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <asctec_hl_comm/WaypointAction.h>
*/
/*message includes*/
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
/*
#include <asctec_hl_comm/WaypointActionGoal.h>
#include <asctec_hl_comm/WaypointActionResult.h>
#include <asctec_hl_comm/WaypointActionFeedback.h>
*/
/*service includes*/
#include <asctec_hl_comm/mav_ctrl_motors.h>

#include "udp_client.h"
#include "udp_server.h"
#include "input_lecture.h"

#define LOOP_RATE 				10

//WAYPOINT SERVER
//void feedbackCallback(const asctec_hl_comm::WaypointFeedbackConstPtr & msg);
void currentposeCallback(const geometry_msgs::PoseStampedConstPtr & msg);

#endif
