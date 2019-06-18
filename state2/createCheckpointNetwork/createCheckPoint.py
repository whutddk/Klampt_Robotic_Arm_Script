# -*- coding: utf-8 -*-
# @File Name: createCheckPoint.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\state2\createCheckpointNetwork\createCheckPoint.py
# @Author: Ruige_Lee
# @Date:   2019-06-01 17:04:10
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-06-18 20:27:33
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/


## 产生 X-11 Y-17 Z-13 个节点 （2431）
## X:100~350 Y:-200~200 Z:100~400
# 编号 X(0-10) Y(0-16) Z(0-12)
# 结点编号 Z*187 + Y*11 + X

# 产生11*17*12 + 11*16*13 + 10*17*13 = 6742条edge
# edge 编号 (0-6741)
# 建立二维数组从pose映射到edge
# X方向edge 0 + 10 * 17 * 13
# Y方向edge 2210 + 11 * 16 * 13
# Z方向edge 4498 + 11 * 17 * 12




from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis
from klampt.model import ik

import random
from math import *




def ik_find_endCoordinate(N1,O1,A1,N2,O2,A2,N3,O3,A3,toolX,toolY,toolZ):
	L = 40.0
	wristX = toolX + A1 * L
	wristY = toolY + A2 * L
	wristZ = toolZ - A3 * L
	return wristX,wristY,wristZ



def ik_solve_Coordinate(wristX,wristY,wristZ):
	# if ( wristX < -0.0001 or wristX > 0.0001 ):	
	# 	pass
	# else:
	# 	wristX = 0.01	
	theta1 = atan2(wristY,wristX)

	s3 = ( (wristX*wristX) / ( cos(theta1)*cos(theta1) ) + (wristZ-264)*(wristZ-264) - 97844.29) / 97785
	theta3 = asin (s3)

	g = wristX / cos(theta1)
	d = 217.3 * cos( theta3 )
	f = 217.3 * sin(theta3) + 225
	h = wristZ - 264
	theta2 = atan2 (( h * d - g * f) , ( g * d + h * f )) 
	return theta1,theta2,theta3

def ik_solve_Posture(N1,O1,A1,N2,O2,A2,N3,O3,A3,theta1,theta2,theta3):

	r11 = N3*sin(theta2 + theta3) + N2*sin(theta1)*cos(theta2 + theta3) + cos(theta1)*N1*cos(theta2 + theta3) 
	r21 = N2*cos(theta1) - N1*sin(theta1)
	r31 = N3*cos(theta2 + theta3) - N2*sin(theta1)*sin(theta2 + theta3) - cos(theta1)*N1*sin(theta2 + theta3)	
	r12 = O3*sin(theta2 + theta3) + O2*sin(theta1)*cos(theta2 + theta3) + cos(theta1)*O1*cos(theta2 + theta3)
	r22 = O2*cos(theta1) - O1*sin(theta1)
	r32 = O3*cos(theta2 + theta3) - O2*sin(theta1)*sin(theta2 + theta3) - cos(theta1)*O1*sin(theta2 + theta3)
	r13 = A3*sin(theta2 + theta3) + A2*sin(theta1)*cos(theta2 + theta3) + cos(theta1)*A1*cos(theta2 + theta3) 
	r23 = A2*cos(theta1) - A1*sin(theta1)
	r33 = A3*cos(theta2 + theta3) - A2*sin(theta1)*sin(theta2 + theta3) - cos(theta1)*A1*sin(theta2 + theta3) 

	theta4 = atan2( ( -r23  ),( r33 ))
	theta6 = atan2( ( r12  ),( r11 ))
		
	if ( abs(theta4 + 3.14159) < abs(theta4) ):
		theta4 = theta4 + 3.14159
		theta5 = atan2( r13 , sqrt(r11*r11 + r12*r12))
		theta6 = theta6 - 3.14159
	else:
		theta5 = atan2( r13 , -sqrt(r11*r11 + r12*r12))

	if ( theta6 > 0 ):
		theta6 =  theta6 % 3.14159
	else:
		theta6 =  theta6 % -3.14159

	return theta4,theta5,theta6


if __name__ == "__main__":
	world = WorldModel()

	res = world.readFile('../../dual_anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	prmRobot = world.robot(0)
	ctlRobot = world.robot(1)
	vis.add("world",world)
	vis.show()	
	prmRobotPose = RobotPoser(prmRobot)
	ctlRobotPose = RobotPoser(ctlRobot)


## 产生 X-11 Y-17 Z-13 个节点 （2431）
## X:100~350 Y:-200~200 Z:100~400
# 编号 X(0-10) Y(0-16) Z(0-12)
# 结点编号 Z*187 + Y*11 + X

	with open('./poseTable.txt','w') as poseTableFile:
		for Z in range(0,13):
			for Y in range(0,17):
				for X in range(0,11):
					xRange = 100 + 25*X
					yRange = -200 + 25*Y
					zRange = 100 + 25*Z
					print (xRange,yRange,zRange)
					wristX,wristY,wristZ = ik_find_endCoordinate(-1,0,0,0,-1,0,0,0,-1,xRange,yRange,zRange)
					theta1,theta2,theta3 = ik_solve_Coordinate(wristX,wristY,wristZ)
					theta4,theta5,theta6 = ik_solve_Posture(-1,0,0,0,-1,0,0,0,-1,theta1,theta2,theta3)
					
					theta1 = round(theta1,3)
					theta2 = round(theta2,3)
					theta3 = round(theta3,3)
					theta4 = round(theta4,3)
					theta5 = round(theta5,3)
					theta6 = round(theta6,3)

					prmRobotPose.set([0,theta1,theta2,theta3,theta4,theta5,theta6,0])

					
					poseTableFile.write(str(theta1))
					poseTableFile.write(",")
					poseTableFile.write(str(theta2))
					poseTableFile.write(",")
					poseTableFile.write(str(theta3))
					poseTableFile.write(",")
					poseTableFile.write(str(theta4))
					poseTableFile.write(",")
					poseTableFile.write(str(theta5))
					poseTableFile.write(",")
					poseTableFile.write(str(theta6))

					poseTableFile.write("\n")

					print (theta1,theta2,theta3,theta4,theta5,theta6)
					ctlRobotPose.set([0,-0.5,-1.57,1.57,0,1.57,0,0])

					R061 = [ cos(theta6)*(cos(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta5)*(sin(theta1)*sin(theta4) - cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))) - sin(theta6)*(cos(theta4)*sin(theta1) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))), - sin(theta6)*(cos(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta5)*(sin(theta1)*sin(theta4) - cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))) - cos(theta6)*(cos(theta4)*sin(theta1) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))), sin(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) + cos(theta5)*(sin(theta1)*sin(theta4) - cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))]
					R062 = [ cos(theta6)*(cos(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) + sin(theta5)*(cos(theta1)*sin(theta4) + cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))) + sin(theta6)*(cos(theta1)*cos(theta4) - sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))),   cos(theta6)*(cos(theta1)*cos(theta4) - sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) - sin(theta6)*(cos(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) + sin(theta5)*(cos(theta1)*sin(theta4) + cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))), sin(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) - cos(theta5)*(cos(theta1)*sin(theta4) + cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))]
					R063 = [ cos(theta6)*(cos(theta5)*sin(theta2 +theta3) - cos(theta4)*sin(theta5)*cos(theta2+theta3)) + sin(theta4)*sin(theta6)*cos(theta2+theta3),                                         cos(theta6)*sin(theta4)*cos(theta2+theta3) - sin(theta6)*(cos(theta5)*sin(theta2 + theta3) - cos(theta4)*sin(theta5)*cos(theta2+theta3)),                       sin(theta5)*sin(theta2+theta3) + cos(theta4)*cos(theta5)*cos(theta2+theta3)]

					FK_X = (217.3*cos(theta1)*cos(theta2)*cos(theta3)) - 40*sin(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) - 40*cos(theta5)*(sin(theta1)*sin(theta4) - cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))) - 225*cos(theta1)*sin(theta2) - (217.3*cos(theta1)*sin(theta2)*sin(theta3))
					FK_Y = 40*cos(theta5)*(cos(theta1)*sin(theta4) + cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) - 40*sin(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) - 225*sin(theta1)*sin(theta2) + (217.3*cos(theta2)*cos(theta3)*sin(theta1)) - (217.3*sin(theta1)*sin(theta2)*sin(theta3))
					FK_Z = 225*cos(theta2) + (217.3*cos(theta2)*sin(theta3)) + (217.3*cos(theta3)*sin(theta2)) - 40*sin(theta5)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) - 40*cos(theta4)*cos(theta5)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) + 264

					# print ( "X,Y,Z",FK_X,FK_Y,FK_Z )

	while(1):
		time.sleep(0.1)
		pass


