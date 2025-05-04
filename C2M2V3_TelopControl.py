#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from math import pi
from roboticstoolbox import trapezoidal
from roboticstoolbox import mtraj
import numpy as np
import sys
'''
Here is where the final version of the robot control for a walk will be developed

'''
go = 0
def callback(data):
	global go
	#print("IT DID ITAAAA")
	go = go + 1
	#rospy.loginfo("Received message: %s", data.data)
'''	
def callbackAO0(data):

	#rospy.loginfo("A0: %s", data.data)
def callbackAO1(data):
	#rospy.loginfo("A1: %s", data.data)
def callbackAO2(data):
	#rospy.loginfo("A2: %s", data.data)
def callbackAO3(data):
	#rospy.loginfo("A3: %s", data.data)
def callbackAO4(data):
	#rospy.loginfo("A4: %s", data.data)
def callbackAO5(data):
	#rospy.loginfo("A5: %s", data.data)		
'''

def robo_com():
	#go = 0
	global go
	J0 = rospy.Publisher('J0', Float64,queue_size=10)
	J1 = rospy.Publisher('J1', Float64,queue_size=10)
	J2 = rospy.Publisher('J2', Float64,queue_size=10)
	J3 = rospy.Publisher('J3', Float64,queue_size=10)
	J4 = rospy.Publisher('J4', Float64,queue_size=10)
	J5 = rospy.Publisher('J5', Float64,queue_size=10)

	G0 = rospy.Publisher('G0', Float64,queue_size=10)
	G1 = rospy.Publisher('G1', Float64,queue_size=10)
	MoveDoneO = rospy.Publisher('MoveDone',Float64,queue_size=10)
	'''
	rospy.Subscriber("AO0", Float64, callbackAO0)
	rospy.Subscriber("AO1", Float64, callbackAO1)
	rospy.Subscriber("AO2", Float64, callbackAO2)
	rospy.Subscriber("AO3", Float64, callbackAO3)
	rospy.Subscriber("AO4", Float64, callbackAO4)
	rospy.Subscriber("AO5", Float64, callbackAO5)
	'''
	rospy.Subscriber("MD", Float64, callback)
	
	rospy.init_node('robo_com', anonymous=True)


	#powerpub = rospy.Publisher('Power', Float32MultiArray,queue_size=10)

	#Move to 45x45
	#J1 - 50 before
	jointAngle = [[0,45,0,90,-45,0],[0,45,0,90,-45,0],[0,51,0,90,-50,0]]
	G0M = Float64(data=0)
	G1M = Float64(data=1) 
	#Move to 45x45 other way across
	
	jointAngle = [[0,40,0,90,-45,0],[135,45,0,90,-50,45],[135,45,0,90,-50,47]]
	#jointAngle = [[135,45,0,90,-50,47],[135,45,0,90,-50,47],[135,45,0,90,-50,47]]

	G0M = Float64(data=0)
	G1M = Float64(data=1)
	
	#jointAngle = [135,45,0,90,-50,45]
	#G0M = Float64(data=1)
	#G1M = Float64(data=0) 
	#Move back on that side
	
	jointAngle = [[135,45,0,90,-35,47],[0,45,0,90,-35,170],[0,45,0,90,-47,170]]
	G0M = Float64(data=1)
	G1M = Float64(data=0)

	jointAngle = np.array([[0,35,0,110,-35,0],
							[45,35,0,45,-90,45],
							[45,85,0,0,90,45]])
	# FULLY EXTENDED WITH BASE ATTACHED [45,75,0,0,-98,-45]
	jointAngle = np.array([[0,40,0,110,-45,-0]])
	#start Pose [0,40,0,110,-45,-0]
	#Fully extended with [30,75,0,30,75,150] sim
	#IN CONFB First Move[30,65,0,30,-65,150]
	#Second Step [30,40,0,30,-30,150]
	#Back most of the way [0,40,0,110,-30,0]
	#All the way back [0,40,0,110,-40,0]
	jointAngle = np.array([[30,65,0,30,-80,-60],
							[30,65,0,30,-80,-60]])
	
	jointAngle = np.array([[0,44,0,110,-30,150],
							[0,44,0,110,-30,150]])

	#CONFA_MOVE start w/ J1 @ 46
	jointAngle = np.array([[0,30,0,110,-45,-0],
							[0,30,0,110,-45,10],
							[30,50,0,30,-80,115],
							[30,65,0,30,-80,115]])



	jointAngle = np.array([[30,65,0,30,-60,115],
							[30,65,0,30,-60,115],
							[0,37,0,110,-30,0],
							[0,37,0,110,-35,0]
							])
	# jointAngle = np.array([[0,30,0,0,-45,0],
	#  						[0,46,0,110,-45,0]])
	# jointAngle = np.array([[30,65,0,30,-80,115],
	#  						[30,65,0,30,-80,115]])
	

	# jointAngle = np.array([[0,37,0,110,-35,0],
	#  						[0,35,0,110,-35,0]])
	
	
	'''
	jointAngle = [[0,45,0,90,-47,170],[0,45,0,90,-35,170],[135,45,0,90,-45,47]]
	G0M = Float64(data=1)
	G1M = Float64(data=0)
	#jointAngle = [[135,45,0,90,-45,47],[135,45,0,90,-45,47],[135,45,0,90,-45,47]]
	'''
	'''
	jointAngle = [[135,40,0,90,-45,47],[0,40,0,90,-45,0],[0,45,0,90,-50,0]]
	#jointAngle = [[135,45,0,90,-50,47],[135,45,0,90,-50,47],[135,45,0,90,-50,47]]

	G0M = Float64(data=0)
	G1M = Float64(data=1)
	'''
	#jointAngle = [0,45,0,90,-47,170]
	#G0M = Float64(data=1)
	#G1M = Float64(data=0) 
	#Move to zeroe
	#jointAngle = [0,0,0,0,0,70]
	'''
	J0M = Float64(data=jointAngle[0])
	J1M = Float64(data=jointAngle[1])
	J2M = Float64(data=jointAngle[2])
	J3M = Float64(data=jointAngle[3])
	J4M = Float64(data=jointAngle[4])
	J5M = Float64(data=jointAngle[5])
	'''
	MoveDone = Float64(data=1) 

	#print(J0M)
	
	
	pubrate = 20
	rate = rospy.Rate(pubrate) # hz
	while (not rospy.is_shutdown()):
		
		#Add in logic here to go from move to move when moveDone = 0
		if go == 0:
			G0.publish(G0M)
			G1.publish(G1M)
			print("IN 0")
		if (go >= 1):
		
			user_input = input("Please enter 6 values separated by spaces & DONE to close gripper: ")
			if (user_input == "DONE"):
				MoveDoneO.publish(MoveDone)
				return
			# Split the input string by spaces and convert to a list of floats
			values = user_input.split()

			# Ensure there are exactly 6 values
			if len(values) != 6:
				print("Error: You must enter exactly 6 values. ")
			
			else:
				# Convert each value to a float (or int if you want integers)
				try:
					values = [float(value) for value in values]
					print("The extracted values are:", values)
					J0M = Float64(data=values[0])
					J1M = Float64(data=values[1])
					J2M = Float64(data=values[2])
					J3M = Float64(data=values[3])
					J4M = Float64(data=values[4])
					J5M = Float64(data=values[5])
					J0.publish(J0M)
					J1.publish(J1M)
					J2.publish(J2M)
					J3.publish(J3M)
					J4.publish(J4M)
					J5.publish(J5M)
					print("IN LOOP")
				except ValueError:
					print("Error: All values must be numbers.")

			
	

if __name__ == '__main__':
	try:
		robo_com()
	except rospy.ROSInterruptException:
		pass