#!/usr/bin/env python

import rospy,time
from PID import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String

def getHeight(data):
	global sonar_z,height

	if type(sonar_z) is list:
		sonar_z.append(data.range)
		if len(sonar_z)==5:
			aux=sum(sonar_z)/5.0
			sonar_z=aux
	else:
		height=data.range-sonar_z

def controlVel(data):
	global msg
	
	msg=data
def command(data):
	global toogle

	cmd=data.data
	if cmd=='takeoff':
		toogle=True
	if cmd=='stop':
		toogle=False

#Main
pid=PID(0.5,0.0,0.5)
pid.setPoint(1.0)
sonar_z=list()
height=None
msg=Twist()
count=0
tolerance=0.01
#Manual Takeoff
toogle=True
#Automatic Takeoff
#waits for the initialization of the quadrotor
time.sleep(1.5)

rospy.Subscriber('/sonar_filtred',Range,getHeight)
rospy.Subscriber('/cmd_vel',Twist,controlVel)
rospy.Subscriber('/syscommand',String,command)
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
rospy.init_node('testPID',anonymous=True)
rate=rospy.Rate(10)

while not rospy.is_shutdown():
	#compute PID
	if not toogle:
		continue
	if height is not None and count<10:
		value=pid.update(height)
		if value<=tolerance:
			count+=1
			value=0.0
		else:
			count=0
		#saturate
		if value<-1.0:
			value=-1.0
		if value>1.0:
			calue=1.0
		msg.linear.z=value
		pub.publish(msg)
	rate.sleep()
