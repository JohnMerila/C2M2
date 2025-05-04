#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from math import pi
from roboticstoolbox import trapezoidal
from roboticstoolbox import mtraj
import numpy as np
import os
import time
'''
Here is where the final version of the robot control for a walk will be developed

'''



go = 0
startTime = 0
def callback(data):
	global go
	global startTime
	print("IT DID ITAAAA")
	go = go + 1
	startTime = time.time()

	rospy.loginfo("Received message: %s", data.data)
A0 = 0.0
A1 = 0.0
A2 = 0.0
A3 = 0.0
A4 = 0.0
A5 = 0.0
def callbackAO0(data):
	global A0 
	A0 = data.data
	#rospy.loginfo("A0: %s", data.data)
def callbackAO1(data):
	global A1 
	A1 = data.data
	#rospy.loginfo("A1: %s", data.data)
def callbackAO2(data):
	global A2 
	A2= data.data
	#rospy.loginfo("A2: %s", data.data)
def callbackAO3(data):
	global A3 
	A3= data.data
	#rospy.loginfo("A3: %s", data.data)
def callbackAO4(data):
	global A4 
	A4= data.data
	#rospy.loginfo("A4: %s", data.data)
def callbackAO5(data):
	global A5 
	A5= data.data
	#rospy.loginfo("A5: %s", data.data)		


def robo_com():
	#go = 0
	global go
	
	MoveDoneO = rospy.Publisher('MoveDone',Float64,queue_size=10)
	
	rospy.Subscriber("AO0", Float64, callbackAO0)
	rospy.Subscriber("AO1", Float64, callbackAO1)
	rospy.Subscriber("AO2", Float64, callbackAO2)
	rospy.Subscriber("AO3", Float64, callbackAO3)
	rospy.Subscriber("AO4", Float64, callbackAO4)
	rospy.Subscriber("AO5", Float64, callbackAO5)
	
	rospy.Subscriber("DataSwitch", Float64, callback)
	
	rospy.init_node('robo_com', anonymous=True)

	MoveDone = Float64(data=1) 

	#print(J0M)
	unique_filename = "MOVEACROSS_MovingConfB_Test6.csv"
	full_file = ("/home/john/catkin_ws/src/c2m2_middleware/src/fig2/"+unique_filename)
	print(full_file)
	file = open(full_file, "w")
	pubrate = 20
	rate = rospy.Rate(pubrate) # hz
	
	while (not rospy.is_shutdown()):
		curTimeRel = time.time() - startTime
		data = [curTimeRel,A0,A1,A2,A3,A4,A5]
		dataOut = list(map(str, data))
		if go == 1:
			file.write(",".join(dataOut)+"\n")
		if go == 2:
			file.close()
		
		

		
		print(go)
		rate.sleep()

	file.close()
	

if __name__ == '__main__':
	try:
		robo_com()
	except rospy.ROSInterruptException:
		pass