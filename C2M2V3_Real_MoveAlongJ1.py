#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from math import pi
from roboticstoolbox import trapezoidal
from roboticstoolbox import mtraj
import numpy as np
'''
Here is where the final version of the robot control for a walk will be developed

'''
go = 0
def callback(data):
	global go
	print("IT DID ITAAAA")
	go = go + 1
	rospy.loginfo("Received message: %s", data.data)
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
	#jointAngle = np.array([[0.0, 45.0, 0.0, 110.0, -45.0, 0.0], [0.0, 40.0, 0.0, 110.0, -45.0, 0.0], [0.0, 43.2, 0.0, 99.0, -51.0, 0.0], [0.0, 46.4, 0.0, 88.0, -57.0, 0.0], [0.0, 49.6, 0.0, 77.0, -63.0, 0.0], [0.0, 52.8, 0.0, 66.0, -69.0, 0.0], [0.0, 56.0, 0.0, 55.0, -75.0, 0.0], [0.0, 59.2, 0.0, 44.0, -81.0, 0.0], [0.0, 62.4, 0.0, 33.0, -87.0, 0.0], [0.0, 65.6, 0.0, 22.0, -93.0, 0.0], [0.0, 68.8, 0.0, 11.0, -99.0, 0.0], [0.0, 72.0, 0.0, 0.0, -105.0, 0.0], [0.0, 77.0, 0.0, 0.0, -105.0, 0.0]])
	jointAngle = np.array([[0.0, 77.0, 0.0, 0.0, -105.0, 0.0], [0.0, 77.0, 0.0, 0.0, -110.0, 0.0], [0.0, 73.8, 0.0, 11.0, -104.0, 0.0], [0.0, 70.6, 0.0, 22.0, -98.0, 0.0], [0.0, 67.4, 0.0, 33.0, -92.0, 0.0], [0.0, 64.2, 0.0, 44.0, -86.0, 0.0], [0.0, 61.0, 0.0, 55.0, -80.0, 0.0], [0.0, 57.8, 0.0, 66.0, -74.0, 0.0], [0.0, 54.6, 0.0, 77.0, -68.0, 0.0], [0.0, 51.4, 0.0, 88.0, -62.0, 0.0], [0.0, 48.2, 0.0, 99.0, -56.0, 0.0], [0.0, 45.0, 0.0, 110.0, -50.0, 0.0], [0.0, 45.0, 0.0, 110.0, -45.0, 0.0]])
	
	#jointAngle = np.array([[0.0, 39.0, 0.0, 105.0, -45.0, 0.0], [30.0, 29.0, 0.0, 105.0, -45.0, 0.0], [37.8, 33.2, -4.5, 104.5, -48.0, 5.17], [45.6, 37.4, -9.0, 104.0, -51.0, 10.34], [53.4, 41.6, -13.5, 103.5, -54.0, 15.51], [61.2, 45.8, -18.0, 103.0, -57.0, 20.68], [69.0, 50.0, -22.5, 102.5, -60.0, 25.85], [76.8, 54.2, -27.0, 102.0, -63.0, 31.02], [84.6, 58.4, -31.5, 101.5, -66.0, 36.19], [92.4, 62.6, -36.0, 101.0, -69.0, 41.36], [100.2, 66.8, -40.5, 100.5, -72.0, 46.53], [108.0, 71.0, -45.0, 100.0, -75.0, 51.7], [78.0, 71.0, -45.0, 100.0, -75.0, 51.7]])

	#Reach Around FRONT OF GRIPPER 12-5/8 FROM FACE OF STRUT
	#jointAngle = np.array([[0.0, 39.0, 0.0, 105.0, -45.0, 0.0], [30.0, 29.0, 0.0, 105.0, -45.0, 0.0], [37.8, 33.2, -4.5, 104.5, -48.0, 5.17], [45.6, 37.4, -9.0, 104.0, -51.0, 10.34], [53.4, 41.6, -13.5, 103.5, -54.0, 15.51], [61.2, 45.8, -18.0, 103.0, -57.0, 20.68], [69.0, 50.0, -22.5, 102.5, -60.0, 25.85], [76.8, 54.2, -27.0, 102.0, -63.0, 31.02], [84.6, 58.4, -31.5, 101.5, -66.0, 36.19], [92.4, 62.6, -36.0, 101.0, -69.0, 41.36], [100.2, 66.8, -40.5, 100.5, -72.0, 46.53], [108.0, 71.0, -45.0, 100.0, -75.0, 51.7], [78.0, 71.0, -45.0, 100.0, -75.0, 51.7]])
	jointAngle = np.array([[78.0, 71.0, -45.0, 100.0, -75.0, 51.7], [78.0, 71.0, -45.0, 100.0, -55.0, 81.7], [70.2, 67.8, -40.5, 100.5, -52.5, 74.03], [62.4, 64.6, -36.0, 101.0, -50.0, 66.36], [54.6, 61.4, -31.5, 101.5, -47.5, 58.69], [46.8, 58.2, -27.0, 102.0, -45.0, 51.02], [39.0, 55.0, -22.5, 102.5, -42.5, 43.35], [31.2, 51.8, -18.0, 103.0, -40.0, 35.68], [23.4, 48.6, -13.5, 103.5, -37.5, 28.01], [15.6, 45.4, -9.0, 104.0, -35.0, 20.34], [7.8, 42.2, -4.5, 104.5, -32.5, 12.67], [0.0, 39.0, 0.0, 105.0, -30.0, 5.0], [0.0, 39.0, 0.0, 105.0, -45.0, 10.0]])
	#
	#Reach Up
	#jointAngle = np.array([[0.0, 39.0, 0.0, 110.0, -45.0, 0.0], [0.0, -1.0, 0.0, 110.0, -45.0, 0.0], [0.0, 0.85, 0.0, 103.5, -42.75, 0.0], [0.0, 2.7, 0.0, 97.0, -40.5, 0.0], [0.0, 4.55, 0.0, 90.5, -38.25, 0.0], [0.0, 6.4, 0.0, 84.0, -36.0, 0.0], [0.0, 8.25, 0.0, 77.5, -33.75, 0.0], [0.0, 10.1, 0.0, 71.0, -31.5, 0.0], [0.0, 11.95, 0.0, 64.5, -29.25, 0.0], [0.0, 13.8, 0.0, 58.0, -27.0, 0.0], [0.0, 15.65, 0.0, 51.5, -24.75, 0.0], [0.0, 17.5, 0.0, 45.0, -22.5, 0.0], [0.0, 22.5, 0.0, 45.0, -22.5, 0.0]])
	jointAngle = np.array([[0.0, 22.5, 0.0, 45.0, -22.5, 0.0], [0.0, 22.5, 0.0, 45.0, 17.5, 0.0], [0.0, 24.15, 0.0, 51.5, 11.75, 0.0], [0.0, 25.8, 0.0, 58.0, 6.0, 0.0], [0.0, 27.45, 0.0, 64.5, 0.25, 0.0], [0.0, 29.1, 0.0, 71.0, -5.5, 0.0], [0.0, 30.75, 0.0, 77.5, -11.25, 0.0], [0.0, 32.4, 0.0, 84.0, -17.0, 0.0], [0.0, 34.05, 0.0, 90.5, -22.75, 0.0], [0.0, 35.7, 0.0, 97.0, -28.5, 0.0], [0.0, 37.35, 0.0, 103.5, -34.25, 0.0], [0.0, 39.0, 0.0, 110.0, -40.0, 0.0], [0.0, 39.0, 0.0, 110.0, -45.0, 0.0]])
	
	#Reach Across
	#jointAngle = np.array([[0.0, 37.0, 0.0, 110.0, -40.0, 0.0], [30.0, 27.0, 0.0, 110.0, -40.0, 0.0], [30.0, 29.8, 0.0, 102.0, -45.0, 11.5], [30.0, 32.6, 0.0, 94.0, -50.0, 23.0], [30.0, 35.4, 0.0, 86.0, -55.0, 34.5], [30.0, 38.2, 0.0, 78.0, -60.0, 46.0], [30.0, 41.0, 0.0, 70.0, -65.0, 57.5], [30.0, 43.8, 0.0, 62.0, -70.0, 69.0], [30.0, 46.6, 0.0, 54.0, -75.0, 80.5], [30.0, 49.4, 0.0, 46.0, -80.0, 92.0], [30.0, 52.2, 0.0, 38.0, -85.0, 103.5], [30.0, 55.0, 0.0, 30.0, -90.0, 115.0], [30.0, 65.0, 0.0, 30.0, -90.0, 115.0]])
	jointAngle = np.array([[30.0, 65.0, 0.0, 30.0, -90.0, 115.0], [30.0, 65.0, 0.0, 30.0, -80.0, 115.0], [27.0, 62.2, 0.0, 38.0, -75.0, 103.5], [24.0, 59.4, 0.0, 46.0, -70.0, 92.0], [21.0, 56.6, 0.0, 54.0, -65.0, 80.5], [18.0, 53.8, 0.0, 62.0, -60.0, 69.0], [15.0, 51.0, 0.0, 70.0, -55.0, 57.5], [12.0, 48.2, 0.0, 78.0, -50.0, 46.0], [9.0, 45.4, 0.0, 86.0, -45.0, 34.5], [6.0, 42.6, 0.0, 94.0, -40.0, 23.0], [3.0, 39.8, 0.0, 102.0, -35.0, 11.5], [0.0, 37.0, 0.0, 110.0, -30.0, 0.0], [0.0, 37.0, 0.0, 110.0, -40.0, 0.0]])
	
	
	
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
		if (go > 0) and (go-1 < len(jointAngle[:,1])):
			J0M = Float64(data=jointAngle[go-1,0])
			J1M = Float64(data=jointAngle[go-1,1])
			J2M = Float64(data=jointAngle[go-1,2])
			J3M = Float64(data=jointAngle[go-1,3])
			J4M = Float64(data=jointAngle[go-1,4])
			J5M = Float64(data=jointAngle[go-1,5])
			J0.publish(J0M)
			J1.publish(J1M)
			J2.publish(J2M)
			J3.publish(J3M)
			J4.publish(J4M)
			J5.publish(J5M)
			print("IN LOOP")
			print(go)
			#input("enter")
		if go == 2 + len(jointAngle[:,1]):
			MoveDoneO.publish(MoveDone)
			go += 1

		
		print(go)
		rate.sleep()


	

if __name__ == '__main__':
	try:
		robo_com()
	except rospy.ROSInterruptException:
		pass