#include "ros/ros.h"
#include <stdlib.h>
#include <SDL/SDL.h>
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char** argv){

	ros::init(argc,argv,"controller")

	ros::NodeHandle n;

	ros::Publisher controller_pub = n.advertise<std_msgs::String>("controller", 1000);

	ros::Rate loop_rate(10);
    static SDL_Event events;
    if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0)
        return EXIT_FAILURE;

    SDL_JoystickEventState(SDL_ENABLE);
    SDL_Joystick *joystick; // on crée le joystick
    joystick = SDL_JoystickOpen(0);
    
    while (ros::ok())
    {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;

    SDL_WaitEvent(&events):
    int num;
    int mag;
    stringstream Num;
    stringstream Mag;
    Num.str("");
    Mag.str("");

    switch(events.type){
	case SDL_JOYBUTTONDOWN:
		num=events.jbutton.button;
		Num << num;
		ss="b "+ convert.str();
		break;

	case SDL_JOYAXISMOTION:
		num=events.jaxis.axis;
		Num << num;
		mag=events.jaxis.value;
		Mag << mag;
		ss="a "+ Num.str() + " " + Mag.str();
		break;	
   }
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    controller_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

    SDL_JoystickClose(joystick);
    SDL_Quit();
    return EXIT_SUCCESS
}
