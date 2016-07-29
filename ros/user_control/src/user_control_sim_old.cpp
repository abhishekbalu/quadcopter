
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
using namespace std;

#include "custom_command.h" //modified remy

#define LOOP_RATE 20
#define QUAD_MAX 5
#define TEAM_MAX 10

//quadrotor lists
typedef struct t_quadrotors
{
    char name[16];
    char client_name[32];
    //Udp_client* comm_channel; // *SIM comment
    ros::Publisher comm_channel;

} quadrotors;
quadrotors quad_list[QUAD_MAX];
int nquads;

typedef struct t_team
{
    char name[16];
    int quad_list[QUAD_MAX];
    int nquads;
} team;
team team_list[TEAM_MAX];
int nteams;

//flags
int error_flag=0;

//network internal variables
int target_quads[QUAD_MAX];
int is_team;
string command;
int ret_server, ret_client;
double timer;

size_t get_executable_path (char* buffer, size_t len)
{
  char* path_end;
  if (readlink ("/proc/self/exe", buffer, len) <= 0)
    return -1;
  path_end = strrchr (buffer, '/');
  if (path_end == NULL)
    return -1;
  ++path_end;
  *path_end = '\0';
  return (size_t) (path_end - buffer);
}



string formation_config_tuning(string file_name)
{
	char path[200];
    string parameters, line;
    get_executable_path (path,200);
    file_name = string(path)+string("../params/")+file_name;
    cout<<" file name is: "<<file_name<<endl;
    ifstream file(file_name.c_str(), ios::in);

    if(file)
	{		
		while(getline(file, line))
		    parameters+=line+" ";
		file.close();
		return parameters;
	}
	else
	{
		cout << "Error : cannot open file for formation_config" << endl;
		return "";
	}
}





int main(int argc, char** argv){

    //ROS initializations
 	ros::init(argc, argv,"user_control_sim");
    ros::NodeHandle nh;
	ros::Rate loop_rate(LOOP_RATE);

	//initialize publisher
	//ros::Publisher user_pub=nh.advertise<std_msgs::String>("user/command_out",5); // *SIM comment

	//variables for command storage
	std_msgs::String msg;

	//initialize UDP server
	//Udp_server* server_interaction = new Udp_server(argv[1]); // *SIM comment

	//initialize UDP clients
	string ip_name, quad_name, team_name;
	nquads=0;
	nteams=0;
	ifstream myReadFile;
	char file_name[200];
	get_executable_path(file_name,200);
	strcat(file_name,"/../params/servers.txt");
 	myReadFile.open(file_name);

 	if (myReadFile.is_open())
 	{
		myReadFile >> nquads;
		for(int k=0;k<nquads;k++)
		{

			myReadFile >> quad_name >> ip_name;
			quad_list[k].comm_channel = nh.advertise<std_msgs::String>(string(quad_name) + string("/user/command_out"),5);
			//quad_list[k].comm_channel = new Udp_client((char*)ip_name.c_str(),argv[1]); // *SIM comment
			strcpy(quad_list[k].name,quad_name.c_str());
			strcpy(quad_list[k].client_name,ip_name.c_str());
			cout << quad_name << " reistered with IP=" << ip_name << endl;
		}
		myReadFile >> nteams;
		for(int k=0;k<nteams;k++)
		{
			myReadFile >> team_name >> team_list[k].nquads;
			strcpy(team_list[k].name,team_name.c_str());
			for(int l=0;l<team_list[k].nquads;l++)
			{
				int found=0;
				myReadFile >> quad_name;
				for(int n=0;n<nquads;n++)
					if(!strcmp(quad_name.c_str(),quad_list[n].name))
					    {team_list[k].quad_list[l]=n; found=1;}
				if(!found)
				    cout<<"error forming team "<<team_name<<": quad "<<quad_name<<" does not exists" << endl;
			}
		}
		team_list[nteams].nquads=nquads;
		strcpy(team_list[nteams].name,"all");
		for(int k=0;k<nquads;k++) {team_list[nteams].quad_list[k]=k;}
		nteams++;
		myReadFile.close();
	}
	else {cout << "problems opening servers file" << endl; exit(1);}


	//additional variable initializations
	command = "";
	ret_server=0;
	ret_client=0;
	timer=0.0;

    //main loop
    while(ros::ok())
    {
    
		//read the command
		getline(cin,command);

		//select quadrotor, commands, and arguments
		stringstream ss; ss << command;
		string command_name, quad_name;
		ss >> command_name >> quad_name;
		
		char garbadge[100];
		char* pc=&command[0];
		int n=0;
		while(n<2)
		{
			sscanf(pc,"%s",garbadge);
			while(*pc==' ') pc++;
			while((*pc!=' ') && (*pc != '\0')) pc++;
			n++;
		}
		string command_args=pc;
		
		//isolate case where command_args must be tuned    // added by remy                               
		if (!strcmp(command_name.c_str(), "formation_config")) //file to string + add the new team
        {   
			int already_exists=0;
            //check if team already exists
			for(int k=0;k<nteams;k++)			
				if(!strcmp(team_list[k].name, quad_name.c_str()))
				{
					cout << "Error : this team already exists, impossible to create the new one" << endl;
					already_exists=1;
					error_flag=1;
				}
			if(already_exists==0)
			{
				//delete the space in the beginning of args
				command_args=command_args.substr(1, command_args.length());

				//pass file name to string with file content + send it to a stringstream
				command_args = formation_config_tuning(command_args);
		        stringstream paramstream; paramstream << command_args;
		        
				//create team
				team team_formation;
		        strcpy(team_formation.name,quad_name.c_str());

		        //define team members
		        string quadname_formation;
		        int garbage;
		        while(!paramstream.eof())                 //for each connection
		        {
		            for(int k=0; k<2; k++)                //add the two quads to team
		            {
		                paramstream >> quadname_formation;
		                for(int l=0; l<nquads; l++)
		                    if(!strcmp(quadname_formation.c_str(), quad_list[l].name))
		                    {
		                        //check if quad is already in the team
		                        int already_here=0;
		                        for(int m=0; m<team_formation.nquads;m++)
		                            if(team_formation.quad_list[m]==l)
		                            {
		                                already_here=1;
		                                break;
		                            }
		                        //add quad to team if not already here
		                        if(already_here==0)
		                        {
		                            team_formation.quad_list[team_formation.nquads]=l;
		                            team_formation.nquads++;
		                        }
		                    }
		            }
		            
		            for(int k=0; k<4; k++)                //discard the 4 int (weight and distance)
		                paramstream >> garbage;
		        }
		        
		        team_list[nteams]=team_formation;
		        nteams++;
			}
        }
        command = command_name;
		if(strlen(command_args.c_str())>0) command = command + " " + command_args;
        
		//test if error appear
		if(error_flag==1)
		{
			error_flag=0;
			continue;
		}

		//determine servers to send the message
		for(int k=0;k<nquads;k++)
	        target_quads[k]=0;
		is_team=0;
		for(int k=0;k<nteams;k++)
		{
		    if(!strcmp(quad_name.c_str(),team_list[k].name))
		    {
		        for(int l=0;l<team_list[k].nquads;l++)
		            target_quads[team_list[k].quad_list[l]]=1;
		        is_team=1;
			}}
	    if(!is_team)
		    for(int k=0;k<nquads;k++)
		        if(!strcmp(quad_name.c_str(),quad_list[k].name))
		            target_quads[k]=1;
	
		//send message to respective servers
		for(int k=0;k<nquads;k++)
			if(target_quads[k]==1)
			{
				cout << "sent message to " << quad_list[k].name << endl;
				cout << "command sent : " << command.c_str() << endl;
				//ret_client=quad_list[k].comm_channel->send_msg(command.c_str()); // *SIM comment
				msg.data=command;
				quad_list[k].comm_channel.publish(msg);

				
				//post process message if needed
				if(!strcmp(command_name.c_str(),"status") || !strcmp(command_name.c_str(),"quad_status"))
				{ // *SIM taken out since cannot copy
				  // the command should be given to the other nodes that should reply in a topic esclusively to share states
				  // all nodes should advertise and subscribe to notification/channel
				  //the user control should be connected to this channel. It sends the order in user_control channel, and then waits for the reply in this channel for each node
				}
			}

		//publish the command 
		//msg.data=command;  *SIM comments - no need to send the message to a topic when in simulation
		//user_pub.publish(msg);

		//ros loop
        ros::spinOnce();
        loop_rate.sleep();
        
    }

	return 0;
}
