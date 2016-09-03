#ifndef DEF_MOBILE_INTERACTION
#define DEF_MOBILE_INTERACTION

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <pthread.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <asctec_hl_comm/WaypointAction.h>

/*message includes*/
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <asctec_hl_comm/WaypointActionGoal.h>
#include <asctec_hl_comm/WaypointActionResult.h>
#include <asctec_hl_comm/WaypointActionFeedback.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <asctec_hl_comm/ExtState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

/*service includes*/
#include <asctec_hl_comm/mav_ctrl_motors.h>

#include "udp_client.h"
#include "udp_server.h"
#include "input_lecture.h"

#define LOOP_RATE 				100
#define MAX_INPUT_SIZE 			100

/*WaypointAction constants*/
#define MAX_SPEED				0.5
#define ACCURACY_POSITION		0.3
#define ACCURACY_ORIENTATION	0.0
#define TIMEOUT					15
#define ScaletoN	        15.599244

#endif
