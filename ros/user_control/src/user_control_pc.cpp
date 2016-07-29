#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <queue>
//#include <random>

using namespace std;

#include "udp_client.h"
#include "udp_server.h"

#define LOOP_RATE 20
#define QUAD_MAX 5
#define TEAM_MAX 10

// global command buffer
string cmd_buffer[20]={};

// struct for storing command
typedef struct t_cmd
{
	string command;
	double t_received;
};
t_cmd delayed_cmd;
double t_now, dt;


//quadrotor lists
typedef struct t_quadrotors
{
    char name[16];
    char  client_name[32];
    Udp_client* comm_channel; // *SIM comment
    // ros::Publisher comm_channel; // *PC comment
    bool break_comm; // flag indicating whether to apply lag/packet loss
    double t_lag; // amount of lag applied to communication (in s)
    double p_loss; // threshold for whether to send packet on discard it
    queue<t_cmd> lag_buffer;
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
bool local_cmd = false;

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

void command_callback(const std_msgs::String::ConstPtr& msg){
	int i;
	// find first empty command
	for(i = 0; cmd_buffer[i] != "" && i<20; i++);
	cmd_buffer[i] = msg->data;
}

int main(int argc, char** argv){

    //ROS initializations
 	ros::init(argc, argv,"user_control");
    	ros::NodeHandle nh;
	ros::Rate loop_rate(LOOP_RATE);

	// initialize listener
	// commands on terminal/command_out topic are user input commands
	// commands on teleop/command_out topic are automatic mission control commands
	ros::Subscriber term_sub = nh.subscribe("terminal/command_out", 5, command_callback);
	ros::Subscriber teleop_sub = nh.subscribe("teleop/command_out", 5, command_callback);

	//initialize publisher
	ros::Publisher user_pub=nh.advertise<std_msgs::String>("user/command_out",5); // *SIM comment

	//variables for command storage
	std_msgs::String msg;

	//initialize UDP server
	Udp_server* server_interaction = new Udp_server(argv[1]); // *SIM comment

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
		for(int k=0; k < nquads; k++)
		{
			// read the quad IP address, communication lag and packet loss probability
			myReadFile >> quad_name >> ip_name >> quad_list[k].t_lag >> quad_list[k].p_loss;
			// quad_list[k].comm_channel = nh.advertise<std_msgs::String>(string(quad_name) + string("/user/command_out"),5); // *PC comment	
			quad_list[k].comm_channel = new Udp_client((char*)ip_name.c_str(),argv[1]); // *SIM comment
			strcpy(quad_list[k].name, quad_name.c_str());
			strcpy(quad_list[k].client_name, ip_name.c_str());
			quad_list[k].break_comm = false; // ensure that initially ideal communication is used
			cout << quad_name << " registered with IP = " << ip_name << endl;
		}

		myReadFile >> nteams;
		for(int k=0; k<nteams; k++)
		{
			myReadFile >> team_name >> team_list[k].nquads;
			strcpy(team_list[k].name, team_name.c_str());
			for(int l=0; l<team_list[k].nquads; l++)
			{
				int found=0;
				myReadFile >> quad_name;
				for(int n=0; n<nquads; n++)
					if(!strcmp(quad_name.c_str(), quad_list[n].name))
					    {team_list[k].quad_list[l]=n; found=1;}
				if(!found)
				    cout << "error forming team " << team_name << ": quad " << quad_name << " does not exists" << endl;
			}
		}
		team_list[nteams].nquads=nquads;
		strcpy(team_list[nteams].name, "all");
		for(int k=0; k<nquads; k++)
			team_list[nteams].quad_list[k] = k;
		nteams++;

		myReadFile.close();
	}
	else {cout << "problems opening servers file" << endl; exit(1);}


	//additional variable initializations
	command = "";
	ret_server=0;
	ret_client=0;
	timer=0.0;

	string command_name;

    //main loop
    while(ros::ok())
    {
		//timer to delay commands
		//static unsigned int delay=0;
		//if(delay>0) delay--;

		//command reading buffer or cin
		// if(cmd_buffer[0]=="")
		// {
		//     getline(cin,command);
		// }

    	if(cmd_buffer[0] != "")
		// else /*if(delay==0)*/
		{
			command = cmd_buffer[0];
			
			//discard used command and shift the nexts
			cmd_buffer[0] = "";
			int i = 0;
			while(cmd_buffer[i+1] != "")
			{
				cmd_buffer[i] = cmd_buffer[i+1];
				cmd_buffer[i+1] = "";
				i++;
			}
		

			//cout<<command<<endl;

			//select quadrotor, commands, and arguments
			stringstream ss; ss << command;
			ss >> command_name >> quad_name;

			char garbage[100];
			char* pc = &command[0];
			int n = 0;
			while(n<2)
			{
				sscanf(pc,"%s",garbage);
				while(*pc == ' ') pc++;
				while((*pc != ' ') && (*pc != '\0')) pc++;
				n++;
			}
			string command_args = pc;
			
			//'print_team' -> display all team created with team members		
			if (!strcmp(command_name.c_str(), "print_team"))
	        { 
				for(int k=0; k<nteams; k++)
				{
					cout << team_list[k].name;
					for(int l=0; l<team_list[k].nquads; l++)
					{
						cout << "\t" << team_list[k].quad_list[l];
					}
					cout << endl;
				}

			}

			//failure file.txt -> read 'file.txt' and place all the commands in a table
			if (!strcmp(command_name.c_str(), "failure"))			
			{
				//work with command_args ('failure quadXX file.txt')
				//command_args=command_args.substr(1, command_args.length());
				//ifstream file(command_args.c_str(), ios::in);

				//work with quad_name ('failure file.txt')
				ifstream file(quad_name.c_str(), ios::in);
				if(file)
				{		
					//fill the buffer with commands
					int i=0;
					while(getline(file, cmd_buffer[i]))
						i++;
					file.close();
				}
				else
					cout << "Error : cannot open file for failure" << endl;
			}	
			

			//'formation_config formation file.txt' create the team 'formation' based on 'file.txt', read the file and give it to the 'formation' team
			if (!strcmp(command_name.c_str(), "formation_config"))
	        {   
				//delete the space in the beginning of args
				command_args=command_args.substr(1, command_args.length());

				cout << "trying to open file: " << command_args << endl;

				//pass file name to string with file content + send it to a stringstream
				command_args = formation_config_tuning(command_args);
				command_args = command_args.substr(0, command_args.length()-1);
		        stringstream paramstream; paramstream << command_args;

				//create team
				team team_formation;
				team_formation.nquads=0;
				
		        strcpy(team_formation.name,quad_name.c_str());

		        //define team members
		        string leader, quadname_formation;
		        double garbage;
				paramstream >> leader >> leader;
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

		        int team_already_here=0;
				for(int k=0;k<nteams;k++)
	                if(!strcmp(team_formation.name, team_list[k].name))
					{
						team_list[k]=team_formation;
						team_already_here=1;
					}
				if(team_already_here==0)
				{
		        	team_list[nteams]=team_formation;
		        	nteams++;
				}
	        }

	        // adds lag and packet loss to the communication
	        if(!strcmp(command_name.c_str(), "break_network"))
	        {
	        	// sets the flag for all the quads
	        	if(!strcmp(quad_name.c_str(), "all"))
	        	{
	        		for(int k = 0; k < nquads; k++)
	        			quad_list[k].break_comm = true;
	        	}
	        	else // sets the flag for specific quads
	        	{
		        	for(int k = 0; k < nquads; k++){
		        		if(!strcmp(quad_name.c_str(), quad_list[k].name))
		        			quad_list[k].break_comm = true;
		        	}
		    	}

		    	cout << "Lag and packet loss enabled." << endl;
		    	local_cmd = true;
	        }

	        // stops lag and packet loss from being added to the communication
	        if(!strcmp(command_name.c_str(), "fix_network"))
	        {
	        	// sets the flag for all the quads
	        	if(!strcmp(quad_name.c_str(), "all"))
	        	{
	        		for(int k = 0; k < nquads; k++)
	        			quad_list[k].break_comm = false;
	        	}
	        	else // sets the flag for specific quads
	        	{
		        	for(int k = 0; k < nquads; k++){
		        		if(!strcmp(quad_name.c_str(), quad_list[k].name))
		        			quad_list[k].break_comm = false;
		        	}
		    	}

				cout << "Lag and packet loss disabled." << endl;
		    	local_cmd = true;
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
			for(int k=0; k<nquads; k++)
		        target_quads[k] = 0;
			is_team = 0;
			for(int k=0; k<nteams; k++)
			{
			    if(!strcmp(quad_name.c_str(), team_list[k].name))
			    {
			        for(int l=0; l<team_list[k].nquads; l++)
			            target_quads[team_list[k].quad_list[l]] = 1;
			        is_team = 1;
				}
			}
		    if(!is_team)
			    for(int k=0; k<nquads; k++)
			        if(!strcmp(quad_name.c_str(), quad_list[k].name)){
			            target_quads[k] = 1;
			        }

			// add the command to the appropriate command buffer(s)
			for(int k = 0; k < nquads; k++)
			{
				if(target_quads[k] == true && !local_cmd) // add command to appropriate buffer, if it's not local (only modifies user_control parameters)
				{
					delayed_cmd.command = command; // assign the command string to the command struct
					delayed_cmd.t_received = ros::Time::now().toSec(); // assign the time the command was received to the command struct
					quad_list[k].lag_buffer.push(delayed_cmd); // add the command struct to the queue
				}        
			}

			local_cmd = false;
		}

		// this loop is executed every ros loop, not just when a command is received
		t_now = ros::Time::now().toSec();
		srand(ros::Time::now().toSec()*1000);
		double p;
		for(int k = 0; k < nquads; k++) // look at the buffers for each quad
		{
			if(!quad_list[k].lag_buffer.empty()) // only proceed if there is actually a command waiting to be sent
			{
				dt = t_now - quad_list[k].lag_buffer.front().t_received; // calculate time since the first (oldest) command was added to the buffer
				if(dt > quad_list[k].t_lag || !quad_list[k].break_comm) // lag not enforced if break_comm flag is false
				{
					// generate a random numer [0,1] to decide whether packet will be lost or not
					p = rand()/(double)RAND_MAX;
					cout << p << endl;
					if(p >= quad_list[k].p_loss || !quad_list[k].break_comm) // packet loss not enforced if break_comm flag is false
					{
						cout << "sent message to " << quad_list[k].name << endl; // *SIM comment
						cout << "command sent : " << quad_list[k].lag_buffer.front().command.c_str() << endl; // *SIM comment

						ret_client=quad_list[k].comm_channel->send_msg(quad_list[k].lag_buffer.front().command.c_str()); // *SIM comment
						//msg.data = quad_list[k].lag_buffer.front().command; // retrieve command string from buffer
						quad_list[k].lag_buffer.pop(); // remove command from buffer
						// quad_list[k].comm_channel.publish(msg); // publish command to appropriate quad // *PC comment
						//ret_client=quad_list[k].comm_channel->send_msg(command.c_str()); // *SIM comment

						// post process message if needed					
						if(!strcmp(command_name.c_str(),"status") || !strcmp(command_name.c_str(),"quad_status"))
						{
							//get message from quadrotor
							timer=ros::Time::now().toSec();
							ret_server=0;
							for(int tries=5;tries>0;tries--)
							{
								ret_client=quad_list[k].comm_channel->send_msg(command.c_str());
								do
								{
									ret_server=server_interaction->msg_box(command);
									ros::spinOnce();
								}
								while((ros::Time::now().toSec()-timer<0.2) && (ret_server!=1));
								if(ret_server==1) break;
							}
					
							//print message
							if(ret_server==1)
							{
								cout << command << endl;
							}
							else
								cout << "quadrotor not responding" << endl;
					
							//reset comms
							timer=ros::Time::now().toSec();
							do {;} while(ros::Time::now().toSec()-timer<0.2);
							do
							{
								ret_server=server_interaction->msg_box(command);
								ros::spinOnce();
							} while(ret_server==1);
						}
						// *SIM comment
					}
					else
						quad_list[k].lag_buffer.pop(); // discard message without sending
				}
			}
		}
		
	//ros loop
        ros::spinOnce();
        loop_rate.sleep();
        
    }
	return 0;
}
