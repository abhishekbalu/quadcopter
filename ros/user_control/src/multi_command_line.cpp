//ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <map>

using namespace std;

int main(int argc, char** argv){

	//initialize ros node
	ros::init(argc, argv,"multi_command_line");
	ros::NodeHandle nh;

	//variables for command stroage
	string command, formation_list;
	std_msgs::String msg;
	char quad_name[20];
	map<string, ros::Publisher> pub_map;

	// get the list of existing quadrotors stored in formation_list
  string param_name("multi_command_line/formation_list");
  nh.getParam(param_name.c_str(),formation_list);

	printf("list : %s\n", formation_list.c_str());

	// parse the string containing the quadrotor names
	// then initialize a publisher and store it in a map with the quad name as key
  char* pc=&formation_list[0];
  int nquads=0;
  while(*pc != 0)
  {
		// scan string for next quad name
    sscanf(pc,"%s",quad_name);
    while(*pc==' ') pc++;
    while((*pc!=' ') && (*pc != '\0')) pc++;
    printf("%s\n",quad_name);
		// create quad topic string
		string quad_topic = string(quad_name) + string("/user/command_out"); 
		// store the publisher in a map with its key (quad name)
		pub_map.insert(pair<string, ros::Publisher>(string(quad_name),
									 nh.advertise<std_msgs::String>(quad_topic, 5)));
    nquads++;
  }

	// main loop
	while(ros::ok()){

		// read the command
		getline(cin,command);

		// get quad name, command, and arguments
		stringstream ss(command);
		string command_name, quad_name, command_args;
		ss >> command_name >> quad_name;

		// find where arguments begin
		char garbadge[100], *pc=&command[0];
		for(int i = 0; i < 2; i++)
		{
			sscanf(pc,"%s",garbadge);
			while(*pc==' ') pc++;
			while((*pc!=' ') && (*pc != '\0')) pc++;
		}
		command_args = pc;  // the arguments is everything after

		// assemble message
		msg.data = command_name+command_args;
		
		// ready to send
		if(pub_map.count(quad_name)) //send message to a quad in particular
			pub_map.find(quad_name)->second.publish(msg);
		else if(quad_name == "all") //send message to all quads
		{
			for(map<string, ros::Publisher>::iterator  it = pub_map.begin(); 
					it != pub_map.end(); ++it)
				it->second.publish(msg);
		}
	}

	return 0;
}
