#!/usr/bin/env python
import roslib; roslib.load_manifest('laser')
import rospy, tf
from math import pi
from copy import deepcopy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from std_msgs.msg import String

def get_imu(data):
	global imu
	imu=deepcopy(data.orientation)

def get_height(data):
	global height
        #height=deepcopy(data).range+sonar_z
	height=deepcopy(data).range

def tf_dynamic(br,imu,height):
	#base footprint
    
	#base stabilized
#	pos=(0.0,0.0,height)
#	quat=(0.0,0.0,imu.z,imu.w)
#	br.sendTransform(pos,quat,rospy.get_rostime(),'base_stabilized','base_footprint')

	#base link
	pos=(0.0,0.0,height)
	quat=(imu.x,imu.y,0.0,imu.w)
#        quat=(imu.x,imu.y.imu.z.imu.w)
	br.sendTransform(pos,quat,rospy.get_rostime(),'base_link','base_footprint')

def tf_static(br):
	#laser link
	quat=(0.0,0.0,0.0,1.0)
	pos=(0.0,0.0,0.118)
	br.sendTransform(pos,quat,rospy.get_rostime(),'laser_link','base_link')
	#sonar link
	quat=tf.transformations.quaternion_from_euler(0,0,3*pi/4)
	pos=(0.1,0.096,0.0)
	br.sendTransform(pos,quat,rospy.get_rostime(),'sonar_link','base_link')

#Main
imu=None
height=0.0
sonar_z=0.07
slam_pos=None
toggle=True

rospy.init_node('robot_publisher',anonymous=True)
br_tf=tf.TransformBroadcaster()
rate=rospy.Rate(100)
rospy.Subscriber('/mavros/imu/data',Imu,get_imu)
rospy.Subscriber('/z_pose',Range,get_height)

while not rospy.is_shutdown():
	if imu is not None:
                tf_dynamic(br_tf,imu,height)
        if toggle:
                tf_static(br_tf)
        toggle=not(toggle)
	rate.sleep()
