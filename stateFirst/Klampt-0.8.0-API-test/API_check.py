# -*- coding: utf-8 -*-
# @File Name: API_check.py
# @File Path: M:\MAS2\Robotic Arm\Klampt_Robotic_Arm_Script\Klampt-0.8.0-API-test\API_check.py
# @Author: Ruige_Lee
# @Date:   2019-01-26 20:00:57
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-01-27 19:44:45
# @Email: 295054118@whut.edu.cn"

# -*- coding: utf-8 -*-

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

	res = world.readFile('../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	prmRobot = world.robot(0)

	vis.add("world",world)
	vis.show()


	collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)


	prmRobotPose.set([0,_PI_/2,-_PI_/2,0,0,_PI_/2,_PI_/4,0])




	while(1):
		time.sleep(0.1)







