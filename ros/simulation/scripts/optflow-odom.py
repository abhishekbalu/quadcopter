#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OpticalFlowRad
from geometry_msgs.msg import TwistWithCovarianceStamped as TwistCovStamped

def compute_odom(data):
	global msg
	if(data.quality==0):
		return
	if(data.integration_time_us==0):
		return
	if(data.distance==0.0):
		return
	#translation from optical flow, in m/s
	#changes in variable asignment and sign to acumudate referencial in angular
	msg.twist.twist.linear.y=(10**6)*(data.integrated_x/data.integration_time_us)/data.distance
	msg.twist.twist.linear.x=-(10**6)*(data.integrated_y/data.integration_time_us)/data.distance
	msg.twist.twist.linear.z=0.0
	#rotation from integrated gyro, in rad/s
	msg.twist.twist.angular.x=(10**6)*data.integrated_xgyro/data.integration_time_us
	msg.twist.twist.angular.y=(10**6)*data.integrated_ygyro/data.integration_time_us
	msg.twist.twist.angular.z=(10**6)*data.integrated_zgyro/data.integration_time_us
	#Populate covariance matrix with uncertainty values
	uncertainty=10**(-1.0*data.quality/(255.0/6.0))
	aux=[0.0]*36
	for x in range(3,36,7):
		aux[x]=uncertainty;
	#msg.twist.covariance=aux;
	for x in range(36):
		msg.twist.covariance[x]=aux[x];
        
def pub_odom(pub,msg):
	msg.header.seq+=1
	msg.header.stamp=rospy.get_rostime()
	pub.publish(msg)


topic='/mavros/px4flow/raw/optical_flow_rad'
pub=rospy.Publisher('/visual_odom',TwistCovStamped,queue_size=10)
rospy.Subscriber(topic,OpticalFlowRad,compute_odom)
msg=TwistCovStamped()
msg.header.seq=0
msg.header.frame_id='sonar_link'

#Initialize ROS Node
rospy.init_node('mavrostest',anonymous=False)
rate=rospy.Rate(20)
while not rospy.is_shutdown():
	pub_odom(pub,msg)
	rate.sleep()
