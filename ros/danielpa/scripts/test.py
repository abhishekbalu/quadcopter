#!/usr/bin/env python

import rospy

#Main
rospy.init_node('test',anonymous=True)
rate=rospy.Rate(10)
while not rospy.is_shutdown():
	print("asd")
	rate.sleep()
