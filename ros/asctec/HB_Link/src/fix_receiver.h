#ifndef DEF_FIX_RECEIVER
#define DEF_FIX_RECEIVER
	
	#include "ros/ros.h"
    #include "std_msgs/String.h"
    #include <sstream>
   
   /*message includes*/
	#include <geometry_msgs/PoseWithCovarianceStamped.h>
	#include <asctec_hl_comm/WaypointActionGoal.h>
	#include <asctec_hl_comm/WaypointActionResult.h>
	#include <asctec_hl_comm/WaypointActionFeedback.h>
    #include <HB_Link/InfoWaypointFeedback.h> 
    #include <HB_Link/InfoCurrentPose.h>
    
    #include "udp_server.h"
    #include "udp_client.h"
	#include "input_lecture.h"

    #define LOOP_RATE 		10
    
#endif
