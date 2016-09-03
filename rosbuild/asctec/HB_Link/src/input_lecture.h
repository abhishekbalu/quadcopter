#ifndef DEF_INPUT_LECTURE
#define DEF_INPUT_LECTURE
	
	#include "ros/ros.h"
	#include "std_msgs/String.h"

	#include <sys/time.h>
	#include <sys/types.h>
	#include <unistd.h>
	#include <stdio.h>
	#include <sstream>
	
	 /*message includes*/
	#include <geometry_msgs/PoseWithCovarianceStamped.h>
	#include <geometry_msgs/PoseStamped.h>
	/*#include <asctec_hl_comm/WaypointActionGoal.h>
	#include <asctec_hl_comm/WaypointActionResult.h>
	#include <asctec_hl_comm/WaypointActionFeedback.h>*/
	
	#define MAX_INPUT_SIZE 100

	//int wait_for_stdin(int seconds);
	void reset_char_array(char* array, int size);
	void fill_char_array(char* array, std::string &string);
	//int shell_lecture(char* message_c, std::string &message);
	//void create_message_waypointfeedback(const asctec_hl_comm::WaypointFeedbackConstPtr & msg, std::string &output);
	void create_message_currentpose(const geometry_msgs::PoseStampedConstPtr & msg, std::string &output);

#endif
