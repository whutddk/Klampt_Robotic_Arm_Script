# -*- coding: utf-8 -*-
# @File Name: ikTest.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\ikMount\ikTest.py
# @Author: Ruige_Lee
# @Date:   2019-04-24 19:15:24
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-05-04 16:36:04
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/
# @File Name: ikTest.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\ikMount\ikTest.py
# @Author: 29505
# @Date:   2019-04-24 10:06:47
# @Last Modified by:   29505
# @Last Modified time: 2019-04-24 12:00:43
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/

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

	L = 0

	wristX = toolX + A1 * L
	wristY = toolY + A2 * L
	wristZ = toolZ + A3 * L

	return wristX,wristY,wristZ



def ik_solve_Coordinate(wristX,wristY,wristZ):


	theta1 = atan2(wristY,wristX)

	s3 = ( (wristX*wristX) / ( cos(theta1)*cos(theta1) ) + (wristZ-264)*(wristZ-264) - 97844.29) / 97785
	theta3 = asin (s3)
	# theta3 = asin((225*225 + 217.3*217.3 - (wristX*wristX) - (wristY*wristY) - (wristZ - 318)*(wristZ - 318) ) / ( 97785 ));

	
	# print ( "theta1=",theta1 )
	# print ( "theta3=",theta3 )

	d = 217.3 * cos( theta3 )
	f = 217.3 * sin(theta3) + 225
	g = wristX / cos(theta1)
	h = wristZ - 264


	theta2 = atan2 (( h * d - g * f) , ( g * d + h * f )) 

	
	# u1 = (f) / (h - d) - sqrt( (f / (h - d))*(f / (h - d)) - ((h+d)/(h-d)) );
	# theta2 = atan(u1) * 2 ;	
					
	# print ( "theta2=",theta2 )

	return theta1,theta2,theta3

def ik_solve_Posture(r11,r12,r13,r21,r22,r23,r31,r32,r33,theta1,theta2,theta3):
# R0E[3][3] = {{N1,O1,A1},{N2,O2,A2},{N3,O3,A3}}

	# r11 = N3*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + N2*sin(theta1)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) + cos(theta1)*N1*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))
	# r21 = cos(theta1)*N2 - N1*sin(theta1)
	# r31 = N3*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - N2*sin(theta1)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) - cos(theta1)*N1*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))
	
	# r12 = O3*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + O2*sin(theta1)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) + cos(theta1)*O1*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))
	# r22 = cos(theta1)*O2 - O1*sin(theta1)
	# r32 = O3*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - O2*sin(theta1)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) - cos(theta1)*O1*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))
	
	# r13 = A3*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + A1*cos(theta1)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) + A2*sin(theta1)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))
	# r23 = A2*cos(theta1) - A1*sin(theta1)
	# r33 = A3*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - A1*cos(theta1)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) - A2*sin(theta1)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2))



	# theta5 = atan2( r13 , sqrt(r11*r11 + r12*r12))
	

	# theta4 = atan2( ( -r23 / cos(theta5) ),( r33 / cos(theta5) ))
	
	


	# theta6 = atan2( ( -r12 / cos(theta5) ),( r11 / cos( theta5 ) ))
	

	theta4 = atan2( -r13 * sin(theta1) + r23 * cos(theta1), -r13 * cos(theta1) * cos(theta2+theta3) - r23 * sin(theta1) * cos(theta2+theta3) + r33 * sin(theta2+theta3))
	
	s5 = -( r13*( cos(theta1)*cos(theta2+theta3)*cos(theta4) + sin(theta1)*sin(theta4) ) + r23 * ( sin(theta1)*cos(theta2+theta3)*cos(theta4) - cos(theta1)*sin(theta4) ) - r33* ( sin(theta2+theta3)* cos(theta4) ) )
	c5 = r13*( -cos(theta1)*sin(theta2+theta3) ) + r23*( -sin(theta1)*sin(theta2+theta3) ) + r33*(-cos(theta2+theta3))
	theta5 = atan2(s5,c5)

	s6 = -r11*(cos(theta1)*cos(theta2+theta3)*sin(theta4) - sin(theta1)*cos(theta4)) - r21*(sin(theta1)*cos(theta2+theta3)*sin(theta4)+cos(theta1)*cos(theta4)) + r31*( sin(theta2+theta3)*sin(theta4) )
	c6 = r11*(( cos(theta1)*cos(theta2+theta3)*cos(theta4) +sin(theta1)*sin(theta4) )*cos(theta5) - cos(theta1)*sin(theta2+theta3)*sin(theta5) ) + r21*( ( sin(theta1)*cos(theta2+theta3)*cos(theta4) - cos(theta1)*sin(theta4) )*cos(theta5) - sin(theta1)*sin(theta2+theta3)*sin(theta5) ) - r31*(sin(theta2+theta3)*cos(theta4)*cos(theta5)+cos(theta2+theta3)*sin(theta5))

	theta6 = atan2(s6,c6)

	# print ( "theta4=",theta4 )
	# print ( "theta5=",theta5 )
	# print ( "theta6=",theta6 )
	return theta4,theta5,theta6



if __name__ == "__main__":


	world = WorldModel()

	res = world.readFile('../dual_anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	prmRobot = world.robot(0)
	ctlRobot = world.robot(1)

	vis.add("world",world)
	vis.show()


	# collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)
	ctlRobotPose = RobotPoser(ctlRobot)


	# robot= world.robot(0)
	# link = prmRobot.link(7)
	# obj = ik.objective(link,R=[1,0,0,0,0,1,0,-1,0],t=[0.3,0,0])

	# solver = ik.solver(obj)
	# solver.solve()
	for h in range(0,53):
		# wristX,wristY,wristZ = ik_find_endCoordinate(1,0,0,0,0,1,0,-1,0,300,0,10*h)
		theta1,theta2,theta3 = ik_solve_Coordinate(200,000,10*h)
		theta4,theta5,theta6 = ik_solve_Posture(1,0,0,0,1,0,0,0,1,theta1,theta2,theta3)


		prmRobotPose.set([0,theta1,theta2,theta3,theta4,theta5+1.75,theta6,0])
		print (theta1,theta2,theta3,theta4,theta5,theta6)
		ctlRobotPose.set([0,0,0.4,0.4,0,0,0,0])

	
	# 	obj = ik.objective(link,R=[1,0,0,0,0,1,0,-1,0],t=[0.3,0,0.01*h])

	# 	solver = ik.solver(obj)
	# 	solver.solve()

	# 	prmRobotPose.set(robot.getConfig())
		time.sleep(0.1)
	while(1):

		time.sleep(0.1)
		pass


