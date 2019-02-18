# -*- coding: utf-8 -*-
# @File Name: CaGraph.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\obstacle-Reachable\CaGraph.py
# @Author: Ruige_Lee
# @Date:   2019-02-18 09:11:50
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-02-18 10:29:25
# @Email: 295054118@whut.edu.cn"

# @File Name: CaGraph.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\obstacle-Reachable\CaGraph.py
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


def make_testing_mesh(world):
	"""automatically create a mesh test grid
	"""

	for z in range(0,32):
		for y in range (0,32):
			for x in range (0,16):
				grid = Geometry3D()

				grid.loadFile("../../terrains/cube.off")

				grid.transform([0.023,0,0,  0,0.030,0,  0,0,0.020],[0.023*x + 0.120,0.030*y-0.480,0.020*z])			

				Mesh = world.makeTerrain("Grid," + "%3d"%x + "," + "%3d"%y + "," + "%3d"%z)

				Mesh.geometry().set(grid)
				Mesh.appearance().setColor(0.1,0.5,0.1,0.1)
	return 

def make_obstacle(world):

	grid = Geometry3D()

	grid.loadFile("../../terrains/cube.off")

	grid.transform([0.368,0,0,  0,0.960,0,  0,0,0.640],[0.120,-0.480,0])			

	Mesh = world.makeTerrain("obstacleArea")

	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.5,0.1,0.1,0.08)

	return 


if __name__ == "__main__":
	
	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			

	make_testing_mesh(world)
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


