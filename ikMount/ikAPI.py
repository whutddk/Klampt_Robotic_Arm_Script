# -*- coding: utf-8 -*-
# @File Name: ikAPI.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\ikMount\ikAPI.py
# @Author: Ruige_Lee
# @Date:   2019-04-24 19:15:24
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-05-31 16:15:56
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


	robot= world.robot(0)
	link = prmRobot.link(7)
	# obj = ik.objective(link,R=[1,0,0,0,0,1,0,-1,0],t=[0.3,0,0])

	# solver = ik.solver(obj)
	# solver.solve()
	

	# for h in range(0,53):
	
	for h in range (0,10):
		obj = ik.objective(link,R=[1,0,0,0,0,1,0,-1,0],t=[0.01,0.3+h*0.01,0.1])
		solver = ik.solver(obj)
		solver.solve()
		prmRobotPose.set(robot.getConfig())

		theta1 = robot.getConfig()[1]
		theta2 = robot.getConfig()[2]
		theta3 = robot.getConfig()[3]
		theta4 = robot.getConfig()[4]
		theta5 = robot.getConfig()[5]
		theta6 = robot.getConfig()[6]

		prmRobotPose.set([0,theta1,theta2,theta3,theta4,theta5,theta6,0])
		print (theta1,theta2,theta3,theta4,theta5,theta6)
		ctlRobotPose.set([0,0,0.4,0.4,0,0,0,0])

		time.sleep(0.1)

	while(1):

		time.sleep(0.1)
		pass


