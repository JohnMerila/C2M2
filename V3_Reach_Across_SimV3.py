#!/usr/bin/env python3
#import tkinter
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
import numpy as np
import roboticstoolbox as rtb
from roboticstoolbox import trapezoidal
#from roboticstoolbox import Trajectory, qplot
from roboticstoolbox import quintic
import math 
from math import pi
import matplotlib.pyplot as plt
import time
'''

Store joint angles for 6 axis across a common move type:
Liner
	a. Inchworm
	b. Slinky
	c. Swing

Have a starting pose and end pose for each
Generate via points along the way for the robot to move between

ex.
Inchworm:
move up gripper
move gripper out
set down gripper

Slinky:
Swing arm about joint 1

Swing:
Swing arm about joint 0

Set time for motion to equivalent?
Set acceleration to equivalent?
Set max velocity to equivalent?
Set torque to equivalent and monitor the power consumed?

'''
import numpy as np
from roboticstoolbox.robot.ERobot import ERobot
from math import pi
import os
from roboticstoolbox import trapezoidal
from roboticstoolbox import trapezoidal_func
from roboticstoolbox import mtraj
import pybullet as p
import matplotlib.pyplot as plt
import swift

from spatialmath import SE3
JointPose = np.zeros(6)
print(JointPose)


class ConfigA(ERobot):
	"""
	Class that imports a planar3DOF Robot
	"""

	def __init__(self):

		args = super().URDF_read(
			"URDF_CONFAV2V1/urdf/URDF_CONFAV2V1.urdf",tld=ConfigA.load_my_path())
            #"CONFIGA_EXPORT.SLDASM/urdf/CONFIGA_EXPORT.SLDASM.urdf",tld=ConfigA.load_my_path())
		
		super().__init__(
			args[0],
			name=args[1])

		self.manufacturer = "Merlin"
		# self.ee_link = self.ets[9]
		
		self.qz = np.array([0,0,0,0,0,0])
		'''
		# zero angles, L shaped pose
		self.addconfiguration("qz", np.array([0, 0, 0]))

		# ready pose, arm up
		self.addconfiguration("qr", np.array([0, pi/2, -pi/2]))

		# straight and horizontal
		self.addconfiguration("qs", np.array([0, 0, -pi/2]))

		# nominal table top picking pose
		self.addconfiguration("qn", np.array([0, pi/4, pi]))
		'''
	@staticmethod    
	def load_my_path():
			print(__file__)
			os.chdir(os.path.dirname(__file__))

class ConfigB(ERobot):
	"""
	Class that imports a planar3DOF Robot
	"""

	def __init__(self):

		args = super().URDF_read(
			"URDF_CONFIGB_V3V4/urdf/URDF_CONFIGB_V3V4.urdf",tld=ConfigA.load_my_path())
            #"CONFIGA_EXPORT.SLDASM/urdf/CONFIGA_EXPORT.SLDASM.urdf",tld=ConfigA.load_my_path())
		
		super().__init__(
			args[0],
			name=args[1])

		self.manufacturer = "Merlin"
		# self.ee_link = self.ets[9]
		
		self.qz = np.array([0,0,0,0,0,0])
		'''
		# zero angles, L shaped pose
		self.addconfiguration("qz", np.array([0, 0, 0]))

		# ready pose, arm up
		self.addconfiguration("qr", np.array([0, pi/2, -pi/2]))

		# straight and horizontal
		self.addconfiguration("qs", np.array([0, 0, -pi/2]))

		# nominal table top picking pose
		self.addconfiguration("qn", np.array([0, pi/4, pi]))
		'''
	@staticmethod    
	def load_my_path():
			print(__file__)
			os.chdir(os.path.dirname(__file__))
'''         
if __name__ == '__main__':   # pragma nocover

	robot = ConfigA()
	print(robot)
'''

def getMove(MoveI,MoveF,vel,dtime):
    MoveI.astype(float)
    MoveF.astype(float)
    print(MoveI)
    print(MoveF)
    print(vel)
    print(dtime)
    distances = np.zeros(6)
    moveOut = MoveI.reshape(1,-1).astype(float)
    moveStep = vel*dtime
    print("movestep =", moveStep)
    for i in range(len(MoveI)):
        distances[i] = abs(MoveI[i] - MoveF[i])
    
    steps = 1/dtime*np.max(distances)/vel
    print(distances)
    print("NUMBER OF STEPS",steps)
    zeros = np.zeros((1, 6))
    keepMoving = True
    idx = 0
    while keepMoving and idx < steps:
        moveOut = np.vstack((moveOut,moveOut[idx,:]))
        idx += 1
        moved = 0
        #print("START OF LOOP")
        for i in range(len(MoveI)):
            #print("Start",moveOut[idx,i])
            #print("GOAL", MoveF[i])
            #print()

            #input("")
            if moveOut[idx-1,i] != MoveF[i]:
                moved = 1
                if moveOut[idx-1,i] < MoveF[i]:
                    moveOut[idx,i] = moveOut[idx-1,i] + moveStep
                    #print("Smaller",i)
                else:
                    #print(moveOut[idx-1,i])
                    moveOut[idx,i] = moveOut[idx-1,i] - moveStep
                    #print(moveOut[idx,i])
                    #print("Larger",i)

                #print("NOT EQUAL")
            #print("End",moveOut[idx,i])
        #print(moveOut[idx,:])
        #input("ENTER")
        if moved == 0:
            keepMoving = False
        #print(moveOut)
        #input("ENTER TO CONTINUE")
        #keepMoving = False
    print(moveOut)
    return moveOut

def makeMoveSetA(startPose,goalPose,numSteps):
    Lstart = np.copy(startPose)
    Lend = np.copy(goalPose)
    Lstart[0] = Lstart[0] +30
    Lstart[1] = Lstart[1] - 15
    Lend[1] = Lend[1] -15
    print(Lstart)
    input("")
    deltMove = (Lstart-Lend)/numSteps
    print(deltMove)
    newMove = np.copy(startPose)
    curMove = Lstart
    newMove = np.vstack((newMove,curMove))
    print(newMove)
    input("DDd")
    for i in range(numSteps):
        curMove = curMove - deltMove
        newMove = np.vstack((newMove,curMove))
    newMove = np.vstack((newMove, goalPose))
    print(newMove)
    input("DD")
    return newMove


def makeMoveSetB(startPose,goalPose,numSteps):
    Lstart = np.copy(startPose)
    Lend = np.copy(goalPose)
    Lstart[4] = Lstart[4] - 15
    Lend[4] = Lend[4] -15
    print(Lstart)
    input("")
    deltMove = (Lstart-Lend)/numSteps
    print(deltMove)
    newMove = np.copy(startPose)
    curMove = Lstart
    newMove = np.vstack((newMove,curMove))
    print(newMove)
    input("DDd")
    for i in range(numSteps):
        curMove = curMove - deltMove
        newMove = np.vstack((newMove,curMove))
    newMove = np.vstack((newMove, goalPose))
    print(newMove)
    input("DD")
    return newMove


#MAIN . . . why do I subscribe?
def robo_com():

    startA = np.array([0,35,0,110,35,0])
    goalA = np.array([30,75,0,30,72,120])
    moveSetA = makeMoveSetA(startA,goalA,10)
    moveSetBinA = makeMoveSetB(goalA,startA,10)
    
    #startB = np.array([0,90,0,0,90,0])
    #goalB = np.array([0,35,0,110,35,0])


    input("DOne")

    rospy.init_node('robo_com',anonymous=True)
    fig, ax = plt.subplots()

    robot = ConfigA()
    robotB = ConfigB()
    BLoc = SE3(.35,.35,0)


    
    ets = robot.ets()
    env = swift.Swift()
    env.launch(realtime=True)
    # env.add(robot)
    
    env.add(robotB)

    dt = 0.01

    #Move set for A

    # moveSetA = np.array([[0,35,0,110,35,0],
    #                     [0,30,0,110,35,0],
    #                     [0,30,0,90,35,0],
    #                     [0,50,0,60,35,0],
    #                     [0,65,0,40,35,0],
    #                     [0,75,0,0,35,0],
    #                     [0,85,0,0,90,0],
    #                     [0,90,0,0,90,0]])
    

    #Move Set for B in A, is flipped to operate in B for displaying

    # moveSetBinA = np.array([[30,75,0,30,75,120],
    #                     [30,75,0,30,60,120],
    #                     [0,35,0,110,30,0],
    #                     [0,35,0,110,35,0]])

    doMoveA = np.empty((0,6))
    for i in range(1,len(moveSetA[:,1])):
        doMoveATemp = getMove(moveSetA[i-1,:],moveSetA[i,:],1,0.1)
        doMoveA = np.vstack((doMoveA,doMoveATemp))

    moveSetB = moveSetBinA[:, ::-1]
    for i in range(len(moveSetB[:,1])):
        
        #moveSetB[i,2],moveSetB[i,3] = moveSetB[i,3],moveSetB[i,2]
        #moveSetB[i,0] = moveSetB[i,0]*-1
        moveSetB[i,1] = moveSetB[i,1]*-1
        #moveSetB[i,5] = moveSetB[i,5]*-1
    doMoveB = np.empty((0,6))
    for i in range(1,len(moveSetB[:,1])):
        doMoveBTemp = getMove(moveSetB[i-1,:],moveSetB[i,:],1,0.1)
        doMoveB = np.vstack((doMoveB,doMoveBTemp))
    
    ii = 0
    
    #Plot Robot B

    # while True and ii < len(doMoveB):
    #     robotB.q = np.radians(doMoveB[ii,:])
    #     env.step(0)
    #     time.sleep(dt)
    #     ii +=1

    #Plot Robot A

    # while True and ii < len(doMoveA):
    #     pose = robot.fkine(np.radians(doMoveA[ii,:]))
    #     print(pose)
    #     robot.q = np.radians(doMoveA[ii,:])
    #     env.step(0)
    #     time.sleep(dt)
    #     ii +=1


    TauA = np.empty((0,6))
    for i in range(len(doMoveA)):
        Taucur = robot.rne(np.radians(doMoveA[i,:]),np.zeros((6,)),np.zeros((6,)))*-9.81
        #print(Taucur)
        TauA = np.vstack((TauA,Taucur))
    TauB = np.empty((0,6))

    #robotB.gravity = np.array([9.81,0,0])

    for i in range(len(doMoveB)):
        Taucur = -1*robotB.rne(np.radians(doMoveB[i,:]),np.zeros((6,)),np.zeros((6,)))*9.81
        #print(Taucur)
        TauB = np.vstack((TauB,Taucur))
    
    TauB = TauB[:, ::-1]
    # ? why don't I flip the 2 and 3 back . . . I thought I would need to
    for i in range(len(doMoveB[:,1])):
        
        #moveSetB[i,2],moveSetB[i,3] = moveSetB[i,3],moveSetB[i,2]
        doMoveB[i,1] = doMoveB[i,1]*-1
        #moveSetB[i,5] = moveSetB[i,5]*-1

    doMoveB = doMoveB[:, ::-1]
    #for i in range(len(TauB[:,1])):
        
        #moveSetB[i,2],moveSetB[i,3] = moveSetB[i,3],moveSetB[i,2]
        #TauB[i,1] = TauB[i,1]*-1
        #TauB[i,5] = TauB[i,5]*-1


    TauTot = np.vstack((TauA,TauB))
    moveFull = np.vstack((doMoveA,doMoveB))


    #Display Plots

    input("Press Enter to display Plots")

    for i in range(0, moveFull.shape[1]):
        plt.plot(moveFull[:, i], label=f'Joint {i}') 
    #plt.plot(TauN)
    plt.legend()
    plt.show()
    
    for i in range(0, TauTot.shape[1]):
        plt.plot(TauTot[:, i], label=f'Joint {i}') 
    #plt.plot(TauN)
    plt.legend()
    plt.show()

    # Save Data
    input("Press Enter to go into saving")

    np.savetxt('/home/john/catkin_ws/src/c2m2_middleware/src/fig2/RAcsimMoveFullV2.csv', moveFull, delimiter=',')

    np.savetxt('/home/john/catkin_ws/src/c2m2_middleware/src/fig2/RAcsimTorqueFullV2.csv',TauTot, delimiter = ',')

    #moveFull.tofile('/home/john/catkin_ws/src/c2m2_middleware/src/fig2/simMoveFull.csv', sep = ',')
    #TauTot.tofile('/home/john/catkin_ws/src/c2m2_middleware/src/fig2/simTorqueFull.csv', sep = ',')
    input("MIGHT SAVE")


    

   

if __name__ == '__main__':
    
    robo_com()