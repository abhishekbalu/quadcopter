#include "mobile_interaction.h"

using namespace std;
using namespace ros;
using namespace actionlib;




int main(int argc, char ** argv){
	ros::init(argc,argv,"mobile_interaction");
	ros::NodeHandle n;
	ros::Rate loop_rate(LOOP_RATE);

	ros::Duration timeout(15);	

	Udp_client *client_interaction = new Udp_client(argv[1], argv[2]);
	Udp_server *server_interaction = new Udp_server(argv[3]);



	std::stringstream ss;
	std_msgs::String msg;
	msg.data=ss.str();
	int ret_task=0;
	int ret_goal=0;
	int ret_motor=0;
	int ret_server=0;
	int seq0;
	int motor;
	
	std::string input;
	std::string task;
	char input_c[MAX_INPUT_SIZE];
	char task_c[MAX_INPUT_SIZE];
	
	ros::Publisher ObjPub=n.advertise<std_msgs::String>("objective", 5);
	
	while(ros::ok()){
		
	        //receives command from fix_interaction node
	    if((ret_server=server_interaction->msg_box(input))){	

        	//translation from string to char*
            	reset_char_array(input_c, MAX_INPUT_SIZE);
            	reset_char_array(task_c, MAX_INPUT_SIZE);
            	fill_char_array(input_c, input);
        	//lecture of task and error control
            	ret_task=sscanf(input_c,"%15[a-zA-Z_]s",task_c);
            	if(ret_task!=1 && ret_server!=2){
                	client_interaction->send_msg("MOBILE: task reading error");
            	}

            	task.clear();
            	task=task_c;

            	//do the chosen task (actualy useless, except for motors, as the tasks are being read in the mobile_controller anyway
            	if(task=="goal"){
		    ss.str(input);
		    msg.data=ss.str();
		    ObjPub.publish(msg);
		    ROS_INFO("%s published as objective",msg.data.c_str());
 
		}else if(task=="takeoff"){
			ss.str("");
			ss << "takeoff";
			msg.data=ss.str();
			ObjPub.publish(msg);
			ROS_INFO("%s published on objective",msg.data.c_str());

		}else if(task=="land"){			
			ss.str("");
			ss << "land";
			msg.data=ss.str();
			ObjPub.publish(msg);
 			ROS_INFO("%s published on objective",msg.data.c_str());
  
	  	}else if(task=="motor"){
			ret_motor=sscanf(input_c, "%n%*[a-zA-Z_/] %d", &seq0, &motor);
			if(ret_motor!=1){
	                    if(!seq0){
			    	cout<<"Seq0 failed in motor"<<endl;
	                    }
			    cout<<"Input motor on/off error"<<endl;
			    client_interaction->send_msg("MOBILE: Input motor on/off error.");
                	}

                	ros::ServiceClient client_motor = n.serviceClient<asctec_hl_comm::mav_ctrl_motors>("fcu/motor_control");
                	asctec_hl_comm::mav_ctrl_motors srv;

                	srv.request.startMotors =(bool)motor;
                	if (client_motor.call(srv)){
                	    ROS_INFO("Motors: %d", (bool)srv.response.motorsRunning);
                	}
                	else{
                	    ROS_ERROR("Failed to call service mav_ctrl_motors");
                	    client_interaction->send_msg("MOBILE: Motor failed to contact service.");
                	}
            	}else if (ret_server!=2){
                	client_interaction->send_msg("MOBILE: Error in command.");
            	}
	    }
	
	ros::spinOnce();
	loop_rate.sleep();
	}

delete client_interaction;
delete server_interaction;

return 0;
}

