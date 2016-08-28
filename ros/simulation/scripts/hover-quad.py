#!/usr/bin/env python

import rospy
from PID import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String

def getHeight(data):
	global sonar_z,height
	sonar_z.append(data.range)
	if len(sonar_z)==5:
		height=sum(sonar_z)/5.0
		sonar_z.pop(0)

#def controlVel(data):
#	global msg
#	
#	msg=data

def command(data):
	global toogle

	cmd=data.data
	if cmd=='takeoff':
		print('takeoff')
		toogle=True
	if cmd=='land':
		print('stop?')
		toogle=False

#Main
pid=PID(0.2,0.0,0.1)
pid.setPoint(0.4)
sonar_z=list()
height=None
msg=Twist()
count=0
tolerance=0.01
toogle=False
output=0.0

rospy.Subscriber('/sonar_filtred',Range,getHeight)
rospy.Subscriber('/syscommand',String,command)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
rospy.init_node('zAxisPID',anonymous=True)
rate=rospy.Rate(10)

while not rospy.is_shutdown():
	#compute PID
	if not toogle:
		continue
	if height is not None:
		value=pid.update(height)
		output+=value
		msg.linear.z=output
		pub.publish(msg)
	rate.sleep()
