/*=========================================================
 //
 // File: macortex_bridge.cpp
 //
 // Created by Ivan Ovinnikov, Aug-2013
 //	
 =============================================================================*/

#include <string.h>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include "cortex.h"
#include <sys/socket.h>
#include <netinet/in.h>

/* === ROS includes === */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <macortex_bridge/QuadPose.h>
#include <macortex_bridge/QuadPoseList.h>
#include <tf/transform_datatypes.h>

#define BROADCAST_PORT 8001

#define PI 3.14159265269
#define QUAD_MAX 5
#define UPDATE_RATE 40
#define CORTEX_RATE 100
#define VIRTUAL_LEADER_RATE 40

ros::Publisher mcs_pose;
ros::Publisher quad_poses;

//lists of quadrotor poses
typedef struct t_quadpose
{
    char name[8];
    double position[3];
    double orientation[3];
    unsigned long pose_updated;
} quadpose;
quadpose quad_pose_list[QUAD_MAX];
int nquads;
std::string this_quad;

//virutal leader variables
int cnt;
double t_last=0.0;

//handlers
void MyErrorMsgHandler(int iLevel, const char *szMsg)
{
  const char *szLevel = NULL;

  if (iLevel == VL_Debug) {
    szLevel = "Debug";
  } else if (iLevel == VL_Info) {
    szLevel = "Info";
  } else if (iLevel == VL_Warning) {
    szLevel = "Warning";
  } else if (iLevel == VL_Error) {
    szLevel = "Error";
  }

  printf("    %s: %s\n", szLevel, szMsg);
}

void MyDataHandler(sFrameOfData* FrameOfData)
{
  //update quadlist
  for(int i=0;i<FrameOfData->nBodies;i++)
    for(int j=0;j<nquads;j++)
      if(!strcmp(FrameOfData->BodyData[i].szName,quad_pose_list[j].name))
      {
        //just update
        quad_pose_list[j].position[0]=FrameOfData->BodyData[i].Segments[0][0]/1000;
        quad_pose_list[j].position[1]=FrameOfData->BodyData[i].Segments[0][1]/1000;
        quad_pose_list[j].position[2]=FrameOfData->BodyData[i].Segments[0][2]/1000;
        quad_pose_list[j].orientation[0]=FrameOfData->BodyData[i].Segments[0][3]/180*PI;
        quad_pose_list[j].orientation[1]=FrameOfData->BodyData[i].Segments[0][4]/180*PI;
        quad_pose_list[j].orientation[2]=FrameOfData->BodyData[i].Segments[0][5]/180*PI;
        quad_pose_list[j].pose_updated = 1;

        //send ros message
        if(!strcmp(quad_pose_list[j].name,this_quad.c_str()))
        {
            geometry_msgs::PoseStamped mcs_pose_msg;
            mcs_pose_msg.header.stamp = ros::Time::now();
            mcs_pose_msg.header.frame_id = FrameOfData->BodyData[i].szName;
            mcs_pose_msg.pose.position.x = FrameOfData->BodyData[i].Segments[0][0]/1000;
            mcs_pose_msg.pose.position.y = FrameOfData->BodyData[i].Segments[0][1]/1000;
            mcs_pose_msg.pose.position.z = FrameOfData->BodyData[i].Segments[0][2]/1000;
            double roll = FrameOfData->BodyData[i].Segments[0][3]/180*PI;
            double pitch = FrameOfData->BodyData[i].Segments[0][4]/180*PI;
            double yaw = FrameOfData->BodyData[i].Segments[0][5]/180*PI;
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
            mcs_pose_msg.pose.orientation = q;
            mcs_pose.publish(mcs_pose_msg);
        }
        break;
      }

  macortex_bridge::QuadPoseList quad_poses_msg;
  quad_poses_msg.header.stamp = ros::Time::now();
  //mcs_pose_msg.header.frame_id - not important
  for(int k=0;k<nquads;k++)
  {
    macortex_bridge::QuadPose quad_pose;
    quad_pose.name=quad_pose_list[k].name;
    quad_pose.position.x=quad_pose_list[k].position[0];
    quad_pose.position.y=quad_pose_list[k].position[1];
    quad_pose.position.z=quad_pose_list[k].position[2];
    double roll = quad_pose_list[k].orientation[0];
    double pitch = quad_pose_list[k].orientation[1];
    double yaw = quad_pose_list[k].orientation[2];
    quad_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    quad_pose.pose_updated=quad_pose_list[k].pose_updated;
    quad_poses_msg.poses.push_back(quad_pose);
    quad_pose_list[k].pose_updated=0;
  }
  quad_poses.publish(quad_poses_msg);
}

void timer_callback(const ros::TimerEvent& event)
{
  //send quadrotor list
  macortex_bridge::QuadPoseList quad_poses_msg;
  quad_poses_msg.header.stamp = ros::Time::now();
  //mcs_pose_msg.header.frame_id - not important
  for(int k=0;k<nquads;k++)
  {
    macortex_bridge::QuadPose quad_pose;
    quad_pose.name=quad_pose_list[k].name;
    quad_pose.position.x=quad_pose_list[k].position[0];
    quad_pose.position.y=quad_pose_list[k].position[1];
    quad_pose.position.z=quad_pose_list[k].position[2];
    double roll = quad_pose_list[k].orientation[0];
    double pitch = quad_pose_list[k].orientation[1];
    double yaw = quad_pose_list[k].orientation[2];
    quad_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
    quad_pose.pose_updated=quad_pose_list[k].pose_updated;
    quad_poses_msg.poses.push_back(quad_pose);
    quad_pose_list[k].pose_updated=0;
  }
  quad_poses.publish(quad_poses_msg);
  return;
}

void virtual_leader_callback(const ros::TimerEvent& event)
{
  //update virtual quadrotor in the list
  double param = 0.5;
  double tmax = 2;
  for(int j=0;j<nquads;j++)
    if(!strcmp("quad1",quad_pose_list[j].name))
    {
      double t = ros::Time::now().toSec()-t_last;
      if(cnt==0)
      {
        quad_pose_list[j].position[0]=0 + param*t;
        quad_pose_list[j].position[1]=0.5;
        quad_pose_list[j].position[2]=1;
        if(t>tmax) {t_last=ros::Time::now().toSec(); cnt=1;}
      }
      else if(cnt==1)
      {
        quad_pose_list[j].position[0]=1;
        quad_pose_list[j].position[1]=0.5 - param*t;
        quad_pose_list[j].position[2]=1;
        if(t>tmax) {t_last=ros::Time::now().toSec(); cnt=2;}
      }
      else if(cnt==2)
      {
        quad_pose_list[j].position[0]=1 - param*t;
        quad_pose_list[j].position[1]=-0.5;
        quad_pose_list[j].position[2]=1;
        if(t>tmax) {t_last=ros::Time::now().toSec(); cnt=3;}
      }
      else if(cnt==3)
      {
        quad_pose_list[j].position[0]=0;
        quad_pose_list[j].position[1]=-0.5 + param*t;
        quad_pose_list[j].position[2]=1;
        if(t>tmax) {t_last=ros::Time::now().toSec(); cnt=0;}
      }
      quad_pose_list[j].orientation[0]=0;
      quad_pose_list[j].orientation[1]=0;
      quad_pose_list[j].orientation[2]=0;
      quad_pose_list[j].pose_updated = 1;
      break;
    }
}

int main(int argc, char* argv[])
{
  ROS_INFO("Initialising Motion Analysis Cortex MCS bridge");
  ros::init(argc, argv, "macortex_bridge");
  ros::NodeHandle nh;

  //fetch names of the quadcopters (same order on the formation cotrol algorithm)
  std::string formation_list;
  std::string full_name;
  std::string param_name("/macortex_bridge/");
  full_name = param_name + "formation_list";
  nh.getParam(full_name.c_str(),formation_list);
  full_name = param_name + "this_quad";
  nh.getParam(full_name.c_str(),this_quad);
  char* pc=&formation_list[0];
  nquads=0;
  while(*pc != 0)
  {
    sscanf(pc,"%s",quad_pose_list[nquads].name);
    quad_pose_list[nquads].pose_updated=0;
    while(*pc==' ') pc++;
    while((*pc!=' ') && (*pc != '\0')) pc++;
    printf("%d - %s\n",nquads,quad_pose_list[nquads].name);
    nquads++;
  }

  //initialize publishers
  mcs_pose = nh.advertise<geometry_msgs::PoseStamped>("macortex_bridge/mcs_pose",1);
  quad_poses = nh.advertise<macortex_bridge::QuadPoseList>("macortex_bridge/quad_poses",1);

  //initialize timers
  //ros::Timer timer1 = nh.createTimer(ros::Duration(1.0/UPDATE_RATE), timer_callback);
  //ros::Timer timer2 = nh.createTimer(ros::Duration(1.0/VIRTUAL_LEADER_RATE), virtual_leader_callback);

  //initialize virtual leader variables
  t_last = ros::Time::now().toSec();
  cnt = 0;

  int retval = RC_Okay;
  unsigned char SDK_Version[4];
  sBodyDefs* pBodyDefs = NULL;

  Cortex_SetVerbosityLevel(VL_Info);

  Cortex_GetSdkVersion(SDK_Version);
  printf("Using SDK Version %d.%d.%d\n", SDK_Version[1], SDK_Version[2],
         SDK_Version[3]);

  Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
  Cortex_SetDataHandlerFunc(MyDataHandler);

  printf("****** Cortex_Initialize ******\n");
  if (argc == 1) {
    retval = Cortex_Initialize("", NULL);
  } else if (argc == 2) {
    retval = Cortex_Initialize(argv[1], NULL);
  } else if (argc == 3) {
    retval = Cortex_Initialize(argv[1], argv[2]);
  }

  if (retval != RC_Okay) {
    printf("Error: Unable to initialize ethernet communication\n");
    retval = Cortex_Exit();
    return 1;
  }

  printf("****** Cortex_GetBodyDefs ******\n");
  pBodyDefs = Cortex_GetBodyDefs();

  if (pBodyDefs == NULL) {
    printf("Failed to get body defs\n");
  } else {
    printf("Got body defs\n");
    Cortex_FreeBodyDefs(pBodyDefs);
    pBodyDefs = NULL;
  }

  void *pResponse;
  int nBytes;
  retval = Cortex_Request("GetContextFrameRate", &pResponse, &nBytes);
  if (retval != RC_Okay)
    printf("ERROR, GetContextFrameRate\n");

  float *contextFrameRate = (float*) pResponse;

  printf("ContextFrameRate = %3.1f Hz\n", *contextFrameRate);

  ros::Rate loop_rate(CORTEX_RATE);
  int loopcounter = 0;

  retval = Cortex_Request("LiveMode", &pResponse, &nBytes);

  while(ros::ok()) // until Ctrl-C is pressed
  {
    ros::spinOnce();
    loop_rate.sleep();
    loopcounter++;
    
	}
	printf("****** Cortex_Exit ******\n");
  retval = Cortex_Exit();

  return 0;
}
