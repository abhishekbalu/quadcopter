#!/usr/bin/env python

import math
import rospy
from copy import deepcopy
from sensor_msgs.msg import Range

def gaussian(x,u,o):

	alpha=1/math.sqrt(2*math.pi*o)
	beta=(x-u)**2/(2*o)

	f=alpha*math.exp(-beta)
	return f

def filterSonar(state,config,pub):
	tres=0.01

	#compute variance
	z=state.reads[-1]
	mean=sum(state.reads)/len(state.reads)
	var=0.0
	for x in state.reads:
		var+=(x-mean)**2
	var=var/(len(state.reads)-1)
	#if variance is below tres, new read if valid
	if var<tres:
		state.valid_z=z
		state.last_valid=z
        #if new read is valid if is within last valid + margin
	else:
		aux=abs(state.last_valid-z)
		if aux<math.sqrt(tres):
			state.valid_z=z
	#publish msg
	msg=deepcopy(config)
	msg.header.seq+=1
	msg.header.stamp=rospy.Time.now()
	msg.range=state.valid_z
	pub.publish(msg)

def getSonar(data):
	global state,config,pub

	if config is None:
		config=deepcopy(data)
		config.header.seq=0

	state.reads.append(data.range)
	if len(state.reads)>4:
		if len(state.reads)==6:
			state.reads.pop(0)
		filterSonar(state,config,pub)

#Main
class state:
	u=0.0
	std_o=0.2
	o=0.2
	valid_z=0.0
	last_valid=0.0
	reads=list()
config=None

rospy.init_node('filter_sonar',anonymous=True)
rospy.Subscriber('/mavros/px4flow/ground_distance',Range,getSonar)
pub=rospy.Publisher('/sonar_filtred',Range,queue_size=10)

rospy.spin()
