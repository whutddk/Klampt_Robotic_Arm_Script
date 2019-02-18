# -*- coding: utf-8 -*-
# @File Name: graph_create.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\prm4RA\graph_create.py
# @Author: Ruige_Lee
# @Date:   2018-12-29 14:36:47
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-02-18 14:43:48
# @Email: 295054118@whut.edu.cn"


from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis

import json

import random

################################################
 

_PI_ = 3.14159 



if __name__ == "__main__":


	world = WorldModel()

	res = world.readFile('./anno_graph.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	

	vis.add("world",world)
	vis.show()


	# collisionTest = WorldCollider(world)


	Robot0 = world.robot(0)
	Robot1 = world.robot(1)
	Robot2 = world.robot(2)
	Robot3 = world.robot(3)
	Robot4 = world.robot(4)
	Robot5 = world.robot(5)
	Robot6 = world.robot(6)
	Robot7 = world.robot(7)

	RobotPose0 = RobotPoser(Robot0)
	RobotPose1 = RobotPoser(Robot1)
	RobotPose2 = RobotPoser(Robot2)
	RobotPose3 = RobotPoser(Robot3)
	RobotPose4 = RobotPoser(Robot4)
	RobotPose5 = RobotPoser(Robot5)
	RobotPose6 = RobotPoser(Robot6)
	RobotPose7 = RobotPoser(Robot7)

	RobotPose0.set([0,0,0,0,0,0,0,0])
	RobotPose1.set([0,0,0,0,0,0,0,0])
	RobotPose2.set([0,0,0,0,0,0,0,0])
	RobotPose3.set([0,0,0,0,0,0,0,0])
	RobotPose4.set([0,0,0,0,0,0,0,0])
	RobotPose5.set([0,0,0,0,0,0,0,0])
	RobotPose6.set([0,0,0,0,0,0,0,0])
	RobotPose7.set([0,0,0,0,0,0,0,0])




	while(1):
		for i in range (0,100):
			if (i<12):
				RobotPose0.set([0,1.57/100 *i,-1.57/100 *i,1.57/100 *i,0,0,0,0])

			if (i<25):
				RobotPose1.set([0,1.57/100 *i,-1.57/100 *i,1.57/100 *i,0,0,0,0])

			if (i<37):
				RobotPose2.set([0,1.57/100 *i,-1.57/100 *i,1.57/100 *i,0,0,0,0])

			if (i<50):
				RobotPose3.set([0,1.57/100 *i,-1.57/100 *i,1.57/100 *i,0,0,0,0])

			if (i<62):
				RobotPose4.set([0,1.57/100 *i,-1.57/100 *i,1.57/100 *i,0,0,0,0])

			if (i<75):
				RobotPose5.set([0,1.57/100 *i,-1.57/100 *i,1.57/100 *i,0,0,0,0])

			if (i<87):
				RobotPose6.set([0,1.57/100 *i,-1.57/100 *i,1.57/100 *i,0,0,0,0])

			if (i<100):
				RobotPose7.set([0,1.57/100 *i,-1.57/100 *i,1.57/100 *i,0,0,0,0])


			time.sleep(0.01)
		time.sleep(1)
		pass




