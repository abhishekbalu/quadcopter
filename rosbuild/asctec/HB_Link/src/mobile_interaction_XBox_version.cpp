#include "mobile_interaction_Xbox_version.h"

using namespace std;
using namespace ros;
using namespace actionlib;

/*Global variables for ROS used in callback functions*/
//ros::Publisher feedback_pub;

//WAYPOINT SERVER

/*void feedbackCB(const asctec_hl_comm::WaypointFeedbackConstPtr &fb){

  feedback_pub.publish(fb);
  const geometry_msgs::Point32& wp = fb->current_pos;
  ROS_INFO("got feedback: %fm %fm %fm %f° ", wp.x, wp.y, wp.z, fb->current_yaw*180/M_PI);

}

void activeCb(){
  ROS_INFO("Goal just went active");
}


void doneCb(const actionlib::SimpleClientGoalState& state, const asctec_hl_comm::WaypointResultConstPtr & result){

  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  else
    ROS_WARN("Finished in state [%s]", state.toString().c_str());

  const geometry_msgs::Point32 & wp = result->result_pos;
  ROS_INFO("Reached waypoint: %fm %fm %fm %f°",wp.x, wp.y, wp.z, result->result_yaw*180/M_PI);

}*/

int main(int argc, char ** argv){
std::cout <<"message 0"<< std::endl;
ros::init(argc,argv,"mobile_interaction");
ros::NodeHandle n;
ros::Rate loop_rate(LOOP_RATE);
int j;
//feedback_pub = n.advertise<asctec_hl_comm::WaypointFeedback>("info/feedback", 1000);

ros::Duration timeout(15);

Udp_client *client_interaction = new Udp_client(argv[1], argv[2]);
Udp_server *server_interaction = new Udp_server(argv[3]);

//WAYPOINT SERVER

//actionlib::SimpleActionClient<asctec_hl_comm::WaypointAction> ac(n, "fcu/waypoint", true);  
//asctec_hl_comm::WaypointGoal goal;


int ret_task=0,ret_goal=0, ret_motor=0, ret_server=0;
int seq0;
float thrust, pitch, roll, yaw;
int motor;
//int goal_ok=0;

//float max_speed;

std::string input;
std::string task;
char input_c[MAX_INPUT_SIZE];
char task_c[MAX_INPUT_SIZE];

ros::Publisher pub=n.advertise<asctec_hl_comm::mav_ctrl>("fcu/control", 1);
asctec_hl_comm::mav_ctrl control; 
ROS_INFO("Avant la boucle");
while(ros::ok()){
	ROS_INFO("Dans la boucle");
        //receives command from fix_interaction node
    if((ret_server=server_interaction->msg_box(input))){

        //translation from string to char*
            reset_char_array(input_c, MAX_INPUT_SIZE);
            reset_char_array(task_c, MAX_INPUT_SIZE);
            fill_char_array(input_c, input);
	ROS_INFO("J'ai recu %s", input_c);
        //lecture of task and error control
            ret_task=sscanf(input_c,"%15[a-zA-Z_]s",task_c);
            if(ret_task!=1 && ret_server!=2){
                client_interaction->send_msg("MOBILE: task reading error");
            }

            task.clear();
            task=task_c;

            //do the chosen task
	    //WAYPOINT SERVER
            if(task=="ctrl"){
		thrust=0;roll=0;pitch=0;yaw=0;
		seq0=0;
		
		ret_goal=sscanf(input_c,"%n%*[a-zA-Z_/] %f %f %f %f",
                                &seq0, &thrust, &roll, &pitch, &yaw);
		std::cout << thrust << std::endl;
	    	std::cout << roll << std::endl;
		std::cout << pitch << std::endl;
		std::cout << yaw << std::endl;

		//if(ret_goal!=4){
                  //              client_interaction->send_msg("MOBILE: Input goal error.");  
                    //    }
		//else{
		    control.x=pitch;
		    control.y=roll;
		    control.z=0.5*thrust;
		    control.yaw=yaw;
		    control.v_max_xy=-0.5;
		    control.v_max_z=-0.5;
		    control.type=asctec_hl_comm::mav_ctrl::acceleration;
		   for(j=0;j<2;j++){
		     pub.publish(control);
		     }
	//	}
		
	    
	   }else if(task=="atterrissage"){
		
		//Decceleration

		control.x=0;
		control.y=0;
		control.z=0.2;
		control.yaw=0;
		control.v_max_xy=-0.5;
		control.v_max_z=-0.5;
		control.type=asctec_hl_comm::mav_ctrl::acceleration;
		for(j=0;j<20;j++){
			pub.publish(control);
			loop_rate.sleep();
			}
		control.x=0;
		control.y=0;
		control.z=0.1;
		control.yaw=0;
		control.v_max_xy=-0.5;
		control.v_max_z=-0.5;
		control.type=asctec_hl_comm::mav_ctrl::acceleration;
		for(j=0;j<20;j++){
			pub.publish(control);
			loop_rate.sleep();
			}
		control.x=0;
		control.y=0;
		control.z=0;
		control.yaw=0;
		control.v_max_xy=-0.5;
		control.v_max_z=-0.5;
		control.type=asctec_hl_comm::mav_ctrl::acceleration;
		for(j=0;j<20;j++){
			pub.publish(control);
		        loop_rate.sleep();
			}
		
		
		//Arret des moteurs
		ros::ServiceClient client_motor = n.serviceClient<asctec_hl_comm::mav_ctrl_motors>("fcu/motor_control");
                asctec_hl_comm::mav_ctrl_motors srv;

                srv.request.startMotors =(bool) 0;
                if (client_motor.call(srv)){
                    ROS_INFO("Motors: %d", (bool)srv.response.motorsRunning);
                }
                else{
                    ROS_ERROR("Failed to call service mav_ctrl_motors");
                    client_interaction->send_msg("MOBILE: Motor failed to contact service.");
                }
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
            }else if(task=="quit_quad"||task=="exit_quad"){
//ajouter que ça quitte aussi les autres noeuds (optionel)
                client_interaction->send_msg("MOBILE: Exiting Quadrotor.");
                exit(EXIT_SUCCESS);
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


