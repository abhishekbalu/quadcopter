#ifndef DEF_FIX_INTERACTION
#define DEF_FIX_INTERACTION
	
	#include "ros/ros.h"
    #include "std_msgs/String.h"
    #include <sstream>
   
   /*message includes*/
	#include <geometry_msgs/PoseWithCovarianceStamped.h>
	#include <asctec_hl_comm/WaypointActionGoal.h>
	#include <asctec_hl_comm/WaypointActionResult.h>
	#include <asctec_hl_comm/WaypointActionFeedback.h>
    
    #include "udp_server.h"
    #include "udp_client.h"
	#include "input_lecture.h"

    #define LOOP_RATE 10
    #define COMMAND_LENGTH 100

#endif
