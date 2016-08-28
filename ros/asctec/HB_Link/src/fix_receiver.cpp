#include "fix_receiver.h"

using namespace std;
using namespace ros;

int main(int argc, char ** argv){

ros::init(argc,argv,"fix_receiver"); 
ros::NodeHandle n;
ros::Rate loop_rate(LOOP_RATE);
//WAYPOINT SERVER
//ros::Publisher infofeedback_pub = n.advertise<HB_Link::InfoWaypointFeedback>("info/waypoint/feedback", 1000);
ros::Publisher infocurrentpose_pub = n.advertise<HB_Link::InfoCurrentPose>("info/current/pose", 1000);

//WAYPOINT SERVER
//HB_Link::InfoWaypointFeedback info_feedback;
HB_Link::InfoCurrentPose info_currentpose;

int ret_info = 0, ret_info_feedback=0, ret_info_currentpose=0;
int seq0=0, seq1=0, seq2=0,seq3=0, seq4=0, seq5=0, seq6=0;
char input_c[MAX_INPUT_SIZE];
char info_c[MAX_INPUT_SIZE];
std::string input;
std::string info;

Udp_server *server_receiver=new Udp_server(argv[1]);

int ret_server=0;

while(ros::ok()){

//receiving input from quad
	 if((ret_server=server_receiver->msg_box(input))==1){
		 
		cout<<"INFO RECEIVED "<<input<<endl;
	 
	//d'abord prendre def info, puis ID quad => mettre dans msg puis publier topic
	//translate received info
	    reset_char_array(input_c, MAX_INPUT_SIZE);
	    reset_char_array(info_c, MAX_INPUT_SIZE);
	    fill_char_array(input_c, input);
	     
	//lecture of task and error control
	    ret_info=sscanf(input_c,"%100[a-zA-Z_]s",info_c);   
	    if(ret_info!=1 && ret_server!=2){
	        ROS_INFO("FIX_RECEIVER: info type lecture error");
	    }
	
	    info.clear();
	    info=info_c;
	    
	    //publishing received info on topic
	    
	    //WAYPOINT SERVER
	   /* if(info=="WaypointFeedback"){
			
			seq0=0;seq1=0;seq2=0,seq3=0;seq4=0;seq5=0;seq6=0;
	    
			ret_info_feedback=sscanf(input_c,"%n%*[a-zA-Z_/] %d %n%*[/] %f %n%*[/] %f %n%*[/] %f %n%*[/] %f %n%*[/] %f ", 
				&seq0, &info_feedback.feedback.header.seq, &seq2, &info_feedback.feedback.header.stamp, &seq3, 
				&info_feedback.feedback.current_pos.x, &seq4, &info_feedback.feedback.current_pos.y, 
				&seq5, &info_feedback.feedback.current_pos.z, &seq6, &info_feedback.feedback.current_yaw);
	
			//make appropriate association with ID => here just test
			info_feedback.quad_id=50;
			
			infofeedback_pub.publish(info_feedback);
		}else */if(info=="CurrentPose"){
			seq0=0;seq1=0;seq2=0,seq3=0;seq4=0;seq5=0;seq6=0;
	    
			ret_info_currentpose=sscanf(input_c,"%n%*[a-zA-Z_/] %d %n%*[/] %f %n%*[/] %lf %n%*[/] %lf %n%*[/] %lf", 
				&seq0, &info_currentpose.pose.header.seq, &seq2, &info_currentpose.pose.header.stamp, &seq3, 
				&info_currentpose.pose.pose.position.x, &seq4, &info_currentpose.pose.pose.position.y, 
				&seq5, &info_currentpose.pose.pose.position.z);
			//make appropriate association with ID => here just test
			info_currentpose.quad_id=50;
			
			infocurrentpose_pub.publish(info_currentpose);
		}

	 }else if(ret_server==2){
			ROS_INFO("FIX_RECEIVER: Server socket timeout.");
	 }else if(ret_server==0){
			ROS_INFO("FIX_RECEIVER: Server socket error.");
	 }
	 
	 ros::spinOnce();
	 loop_rate.sleep();
}

delete server_receiver;

return 0;
}
