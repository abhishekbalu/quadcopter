#!/usr/bin/env python

import sys
import rospy
import Tkinter as tk
from geometry_msgs.msg import Twist

def on_closing():
	global msg

	msg.linear.x=0.0
	msg.linear.y=0.0
	msg.linear.z=0.0
	msg.angular.z=0.0
	root.destroy()

def onKeyPress(event):
	global config,msg,text,pub

	if event.char==config['front']:
		aux=msg.linear.x+0.2
		msg.linear.x=round(aux,1)
	elif event.char==config['back']:
		aux=msg.linear.x-0.2
		msg.linear.x=round(aux,1)
	elif event.char==config['left']:
		aux=msg.linear.y+0.2
		msg.linear.y=round(aux,1)
	elif event.char==config['right']:
		aux=msg.linear.y-0.2
		msg.linear.y=round(aux,1)
	elif event.char==config['up']:
		aux=msg.linear.z+0.2
		msg.linear.z=round(aux,1)
	elif event.char==config['down']:
		aux=msg.linear.z-0.2
		msg.linear.z=round(aux,1)
	elif event.char==config['turnleft']:
		aux=msg.angular.z+0.2
		msg.angular.z=round(aux,1)
	elif event.char==config['turnright']:
		aux=msg.angular.z-0.2
		msg.angular.z=round(aux,1)
	elif event.char==config['stop']:
		msg.linear.x=0.0
		msg.linear.y=0.0
		msg.linear.z=0.0
		msg.angular.z=0.0
	
	print_screen(text,msg)
	pub.publish(msg)	

def print_screen(text,msg):

	l='['+str(msg.linear.x)+','+str(msg.linear.y)+','+str(msg.linear.z)+']'
	a='['+str(msg.angular.x)+','+str(msg.angular.y)+','+str(msg.angular.z)+']'
	line='Commands sent to /cmd_vel topic\n\n'
	line+='Linear speed:\n'+l+'\nAngular speed:\n'+a+'\n'
	text.delete('1.0','10.0')
	text.insert('1.0', line)

def pub_cmd(event):
	global msg,pub
	#rospy.loginfo(cmd)
	pub.publish(msg)

def getVel(data):
	global msg

	msg=data

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
msg=Twist()
pub=rospy.Publisher('cmd_vel',Twist,queue_size=10)
rospy.Subscriber('cmd_vel',Twist,getVel)
rospy.init_node('keyboard_teleop',anonymous=False)
rospy.Timer(rospy.Duration(0.1),pub_cmd)

#Initialize Tkinter
global root
root = tk.Tk()
root.geometry('300x120')
root.wm_title("Keyboard Teleop")
text = tk.Text(root, background='black', foreground='white', font=('Comic Sans MS', 12))
text.pack()
root.bind('<KeyPress>', onKeyPress)
root.wm_protocol("WM_DELETE_WINDOW", on_closing)
print_screen(text,msg)

#Main Loop
root.mainloop()
