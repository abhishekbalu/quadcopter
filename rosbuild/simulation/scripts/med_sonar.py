#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from copy import deepcopy

def displayReading(reads,config,pub):
	#Compute value
	if len(reads)!=0:
		value=sum(reads)/len(reads)
	else:
		value=0
	#Publish value
	if config is not None:
		config.header.seq+=1
		msg=deepcopy(config)
		msg.header.stamp=rospy.Time.now()
		msg.range=value
		pub.publish(msg)
		#debug
		rospy.loginfo(msg)

def sonarFilter(data):
	global reads,config

	if config is None:
		config=deepcopy(data)
		config.header.seq=1
	newRead=data.range
	reads.append(newRead)
	if len(reads)>5:
		reads.pop(0)

#Main
reads=list()
config=None
rospy.init_node('filter_odom',anonymous=True)
rospy.Subscriber('/sonar_bottom',Range,sonarFilter)
pub=rospy.Publisher('/sonar_filtred',Range,queue_size=10)

rate=rospy.Rate(5)
while not rospy.is_shutdown():
	displayReading(reads,config,pub)
	rate.sleep()
