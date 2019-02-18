# -*- coding: utf-8 -*-
# @File Name: ObGraph.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\obstacle-Reachable\ObGraph.py
# @Author: Ruige_Lee
# @Date:   2019-02-18 09:44:06
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-02-18 10:18:25
# @Email: 295054118@whut.edu.cn"

# @File Name: ObGraph.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\obstacle-Reachable\ObGraph.py
# @Author: 29505
# @Date:   2018-12-30 09:59:45
# @Last Modified by:   29505
# @Last Modified time: 2019-02-14 21:58:42
# @Email: 295054118@whut.edu.cn

from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis

import json

import random


def make_obstacle(world):

	grid = Geometry3D()

	grid.loadFile("../../terrains/cube.off")

	grid.transform([0.368,0,0,  0,0.960,0,  0,0,0.640],[0.120,-0.480,0])			

	Mesh = world.makeTerrain("obstacleArea")

	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.1,0.5,0.1,0.08)

	return 


if __name__ == "__main__":
	
	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			

	make_obstacle(world)
				

	robot = world.robot(0)

	vis.add("world",world)
	vis.show()

	collisionTest = WorldCollider(world)
	
	robotPose = RobotPoser(robot)
	


	while(1):
		time.sleep(0.1)
		vis.shown()
		pass


