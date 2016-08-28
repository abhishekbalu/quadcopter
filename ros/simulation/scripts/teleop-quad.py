#!/usr/bin/env python

import sys, rospy
import Tkinter as tk
from geometry_msgs.msg import PoseStamped

def on_closing():
	global msg

	msg.pose.position.x=0.0
	msg.pose.position.y=0.0
	msg.pose.position.z=0.0
	msg.pose.orientation.z=0.0
	msg.pose.orientation.w=1.0
	root.destroy()

def onKeyPress(event):
	global config,msg,text,pub

	if event.char==config['front']:
		aux=msg.pose.position.x+0.1
		msg.pose.position.x=round(aux,1)
	elif event.char==config['back']:
		aux=msg.pose.position.x-0.1
		msg.pose.position.x=round(aux,1)
	elif event.char==config['left']:
		aux=msg.pose.position.y+0.1
		msg.pose.position.y=round(aux,1)
	elif event.char==config['right']:
		aux=msg.pose.position.y-0.1
		msg.pose.position.y=round(aux,1)
	elif event.char==config['up']:
		aux=msg.pose.position.z+0.1
		msg.pose.position.z=round(aux,1)
	elif event.char==config['down']:
		aux=msg.pose.position.z-0.1
		msg.pose.position.z=round(aux,1)
	elif event.char==config['turnleft']:
		aux=msg.pose.orientation.z+0.05
		msg.pose.orientation.z=round(aux,2)
	elif event.char==config['turnright']:
		aux=msg.pose.orientation.z-0.05
		msg.pose.orientation.z=round(aux,2)
	elif event.char==config['stop']:
		msg.pose.position.x=0.0
		msg.pose.position.y=0.0
		#msg.pose.position.z=0.0
		msg.pose.orientation.z=0.0
	
	print_screen(text,msg)
	pub.publish(msg)	

def print_screen(text,msg):

	line='Commands sent to:\n'
	line+='/mavros/setpoint_position/local\n\n'
	line+='  x: '+str(msg.pose.position.x)+'\n'
	line+='  y: '+str(msg.pose.position.y)+'\n'
	line+='  z: '+str(msg.pose.position.z)+'\n'
	line+='  a: '+('%.2f'%msg.pose.orientation.z)
	text.delete('1.0','10.0')
	text.insert('1.0', line)

def pub_cmd(event):
	global msg,pub
	msg.header.seq+=1
	msg.header.stamp=rospy.get_rostime()
	#rospy.loginfo(cmd)
	pub.publish(msg)

#def getVel(data):
#	global msg
#
#	msg=data

#Main
setup=False
config=dict()

doc=rospy.get_param('config_file')
f=open(doc,'r')
if f is None:
	doc=rospy.get_param('default_file')
	f=open(doc,'r')

while not setup:
	aux=[0,0,0,0,0,0,0,0,0]
	f.readline()
	f.readline()
	line=f.readline()
	while line:
		split=line.rstrip().split('=')
		config.update({split[0]:split[1]})
		if split[0]=='front':
			aux[0]+=1
		elif split[0]=='back':
			aux[1]+=1
		elif split[0]=='left':
			aux[2]+=1
		elif split[0]=='right':
			aux[3]+=1
		elif split[0]=='up':
			aux[4]+=1
		elif split[0]=='down':
			aux[5]+=1
		elif split[0]=='turnleft':
			aux[6]+=1
		elif split[0]=='turnright':
			aux[7]+=1
		elif split[0]=='stop':
			aux[8]+=1
		line=f.readline()

	if aux.count(0):
		doc=rospy.get_param('default_file')
		f=open(doc,'r')
	else:
		setup=True

#Initialize ROS Node
msg=PoseStamped()
msg.header.seq=0
msg.header.frame_id='base_link'
msg.pose.orientation.w=1.0

topic='/mavros/setpoint_position/local'
pub=rospy.Publisher(topic,PoseStamped,queue_size=10)
#rospy.Subscriber(topic,PoseStamped,getVel)
rospy.init_node('keyboard_teleop',anonymous=False)
rospy.Timer(rospy.Duration(0.05),pub_cmd)

#Initialize Tkinter
global root
root = tk.Tk()
root.geometry('300x140')
root.wm_title("Keyboard Teleop")
text = tk.Text(root, background='black', foreground='white', font=('Comic Sans MS', 12))
text.pack()
root.bind('<KeyPress>', onKeyPress)
root.wm_protocol("WM_DELETE_WINDOW", on_closing)
print_screen(text,msg)

#Main Loop
root.mainloop()
