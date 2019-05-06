# -*- coding: utf-8 -*-
# @File Name: ikTest.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\ikMount\ikTest.py
# @Author: Ruige_Lee
# @Date:   2019-04-24 19:15:24
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-05-06 17:28:39
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


	d = 217.3 * cos( theta3 )
	f = 217.3 * sin(theta3) + 225
	g = wristX / cos(theta1)
	h = wristZ - 264


	theta2 = atan2 (( h * d - g * f) , ( g * d + h * f )) 


	return theta1,theta2,theta3

def ik_solve_Posture(N1,O1,A1,N2,O2,A2,N3,O3,A3,theta1,theta2,theta3):
# R0E[3][3] = {{N1,O1,A1},{N2,O2,A2},{N3,O3,A3}}

	r11 = N3*sin(theta2 + theta3) + N2*sin(theta1)*cos(theta2 + theta3) + cos(theta1)*N1*cos(theta2 + theta3) 
	r21 = N2*cos(theta1) - N1*sin(theta1)
	r31 = N3*cos(theta2 + theta3) - N2*sin(theta1)*sin(theta2 + theta3) - cos(theta1)*N1*sin(theta2 + theta3)
	
	r12 = O3*sin(theta2 + theta3) + O2*sin(theta1)*cos(theta2 + theta3) + cos(theta1)*O1*cos(theta2 + theta3)
	r22 = O2*cos(theta1) - O1*sin(theta1)
	r32 = O3*cos(theta2 + theta3) - O2*sin(theta1)*sin(theta2 + theta3) - cos(theta1)*O1*sin(theta2 + theta3)
	
	r13 = A3*sin(theta2 + theta3) + A2*sin(theta1)*cos(theta2 + theta3) + cos(theta1)*A1*cos(theta2 + theta3) 
	r23 = A2*cos(theta1) - A1*sin(theta1)
	r33 = A3*cos(theta2 + theta3) - A2*sin(theta1)*sin(theta2 + theta3) - cos(theta1)*A1*sin(theta2 + theta3) 

	# r11 = N3*cos(theta2 + theta3) - N2*sin(theta1)*sin(theta2 + theta3) - cos(theta1)*N1*sin(theta2 + theta3) 
	# r21 = - N2*cos(theta1) + N1*sin(theta1)
	# r31 = N3*sin(theta2 + theta3) + N2*sin(theta1)*cos(theta2 + theta3) + cos(theta1)*N1*cos(theta2 + theta3)
	
	# r12 = O3*cos(theta2 + theta3) - O2*sin(theta1)*sin(theta2 + theta3) - cos(theta1)*O1*sin(theta2 + theta3)
	# r22 = - O2*cos(theta1) + O1*sin(theta1)
	# r32 = O3*sin(theta2 + theta3) + O2*sin(theta1)*cos(theta2 + theta3) + cos(theta1)*O1*cos(theta2 + theta3)
	
	# r13 = A3*cos(theta2 + theta3) - A2*sin(theta1)*sin(theta2 + theta3) - cos(theta1)*A1*sin(theta2 + theta3) 
	# r23 = - A2*cos(theta1) + A1*sin(theta1)
	# r33 = A3*sin(theta2 + theta3) + A2*sin(theta1)*cos(theta2 + theta3) + cos(theta1)*A1*cos(theta2 + theta3) 


	# theta5 = atan2( r13 , sqrt(r11*r11 + r12*r12))
	
	# theta4 = atan2( ( -r23 / cos(theta5) ),( r33 / cos(theta5) ))
	
	# theta6 = atan2( ( -r12 / cos(theta5) ),( r11 / cos( theta5 ) ))
	theta4 = atan2(r23,r13)
	theta6 = atan2(-r32,r31)
	theta5 = asin(r33)
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
		# wristX,wristY,wristZ = ik_find_endCoordinate(0,0,1,0,1,0,-1,0,0,100,100,10*h)
		wristX,wristY,wristZ = ik_find_endCoordinate(1,0,0,0,1,0,0,0,1,100,000,10*h)
		theta1,theta2,theta3 = ik_solve_Coordinate(wristX,wristY,wristZ)
		theta4,theta5,theta6 = ik_solve_Posture(1,0,0,0,1,0,0,0,1,theta1,theta2,theta3)
		
		# obj = ik.objective(link,R=[1,0,0,0,0,1,0,-1,0],t=[0.2,0.2,0.01*h])
		# solver = ik.solver(obj)
		# solver.solve()
		# prmRobotPose.set(robot.getConfig())

		# theta1 = robot.getConfig()[1]
		# theta2 = robot.getConfig()[2]
		# theta3 = robot.getConfig()[3]
		# theta4 = robot.getConfig()[4]
		# theta5 = robot.getConfig()[5]
		# theta6 = robot.getConfig()[6]

		prmRobotPose.set([0,theta1,theta2,theta3,theta4,theta5,theta6,0])
		print (theta1,theta2,theta3,theta4,theta5,theta6)
		ctlRobotPose.set([0,0,0.4,0.4,0,0,0,0])

		# R041 = [ -cos(theta1)*sin(theta2+theta3), -sin(theta1)*sin(theta2+theta3), cos(theta2+theta3)]
		# R042 = [  sin(theta1),  -cos(theta1),0]
		# R043 = [  cos(theta1)*cos(theta2+theta3),  sin(theta1)*cos(theta2+theta3), sin(theta2+theta3)]

		# R061 = [ cos(theta6)*(cos(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta5)*(sin(theta1)*sin(theta4) - cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))) - sin(theta6)*(cos(theta4)*sin(theta1) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))), - sin(theta6)*(cos(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta5)*(sin(theta1)*sin(theta4) - cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))) - cos(theta6)*(cos(theta4)*sin(theta1) + sin(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))), sin(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) + cos(theta5)*(sin(theta1)*sin(theta4) - cos(theta4)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))]
		# R062 = [ cos(theta6)*(cos(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) + sin(theta5)*(cos(theta1)*sin(theta4) + cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))) + sin(theta6)*(cos(theta1)*cos(theta4) - sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))),   cos(theta6)*(cos(theta1)*cos(theta4) - sin(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) - sin(theta6)*(cos(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) + sin(theta5)*(cos(theta1)*sin(theta4) + cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))), sin(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) - cos(theta5)*(cos(theta1)*sin(theta4) + cos(theta4)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))]
		# R063 = [ cos(theta6)*(cos(theta5)*sin(theta2 +theta3) - cos(theta4)*sin(theta5)*cos(theta2+theta3)) + sin(theta4)*sin(theta6)*cos(theta2+theta3),                                         cos(theta6)*sin(theta4)*cos(theta2+theta3) - sin(theta6)*(cos(theta5)*sin(theta2 + theta3) - cos(theta4)*sin(theta5)*cos(theta2+theta3)),                       sin(theta5)*sin(theta2+theta3) + cos(theta4)*cos(theta5)*cos(theta2+theta3)]
		R061 = [ cos(theta4)*(cos(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta5)*(sin(theta1)*sin(theta6) - cos(theta6)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))) - sin(theta4)*(cos(theta6)*sin(theta1) + sin(theta6)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))), - sin(theta4)*(cos(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) - sin(theta5)*(sin(theta1)*sin(theta6) - cos(theta6)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))) - cos(theta4)*(cos(theta6)*sin(theta1) + sin(theta6)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2))), sin(theta5)*(cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3)) + cos(theta5)*(sin(theta1)*sin(theta6) - cos(theta6)*(cos(theta1)*cos(theta2)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)))]
		R062 = [ cos(theta4)*(cos(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) + sin(theta5)*(cos(theta1)*sin(theta6) + cos(theta6)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))) + sin(theta4)*(cos(theta1)*cos(theta6) - sin(theta6)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))),   cos(theta4)*(cos(theta1)*cos(theta6) - sin(theta6)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2))) - sin(theta4)*(cos(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) + sin(theta5)*(cos(theta1)*sin(theta6) + cos(theta6)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))), sin(theta5)*(cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3)) - cos(theta5)*(cos(theta1)*sin(theta6) + cos(theta6)*(cos(theta2)*sin(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)))]
		R063 = [ cos(theta4)*(cos(theta5)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) - cos(theta6)*sin(theta5)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))) + sin(theta4)*sin(theta6)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)),                                         cos(theta4)*sin(theta6)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3)) - sin(theta4)*(cos(theta5)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) - cos(theta6)*sin(theta5)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))),                       sin(theta5)*(cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2)) + cos(theta5)*cos(theta6)*(cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3))]


		print (R061)
		print (R062)
		print (R063)



		time.sleep(0.1)
	while(1):

		time.sleep(0.1)
		pass


