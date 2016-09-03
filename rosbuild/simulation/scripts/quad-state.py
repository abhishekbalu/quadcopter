#!/usr/bin/env python

import sys
import rospy
from mavros.srv import *
from mavros.msg import State

def get_state(data):
	global str_state

	line='\nQuadrotor state\n'
	line+='connected: '+str(data.connected)+'\n'
	line+='armed: '+str(data.armed)+'\n'
	line+='guided: '+str(data.guided)+'\n'
	line+='mode: '+str(data.mode)

	if(str_state!=line):
		if(str_state!=''):
			rospy.loginfo('\n\n\n\n')
		rospy.loginfo(line)
		str_state=line
	

#Main
str_state=''
rospy.init_node('status',anonymous=True)
rospy.Subscriber('/mavros/state',State,get_state)
rospy.spin()

