#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,TwistStamped

def pub_cmd(pub):
	global msg
	msg.header.seq+=1
	msg.header.stamp=rospy.get_rostime()
	pub.publish(msg)

def get_vel(data):
	global msg
	msg.twist=data

#Initialize ROS message
msg=TwistStamped()
msg.header.seq=0
msg.header.frame_id='base_link'

topic_name='/mavros/setpoint_velocity/cmd_vel'
pub=rospy.Publisher(topic_name,TwistStamped,queue_size=10)
rospy.Subscriber('/cmd_vel',Twist,get_vel)
#Initialize ROS Node
rospy.init_node('cmd_vel-repeater',anonymous=False)
rate=rospy.Rate(20)
while not rospy.is_shutdown():
	#publish velocity comand
	pub_cmd(pub)
	rate.sleep()

