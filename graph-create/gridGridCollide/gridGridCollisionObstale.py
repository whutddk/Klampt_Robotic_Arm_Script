# -*- coding: utf-8 -*-
# @File Name: gridGridCollisionObstale.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\gridGridCollide\gridGridCollisionObstale.py
# @Author: Ruige_Lee
# @Date:   2019-02-20 19:40:54
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-02-21 09:33:21
# @Email: 295054118@whut.edu.cn"



from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis

import json

import random

gridList = []



def make_obstacle(world):
	"""automatically create a mesh test grid
	"""
	grid = Geometry3D()

	grid.loadFile("../../terrains/cube.off")

	grid.transform([0.1,0,0,  0,0.1,0,  0,0,0.1],[0.25,0,0.24])			

	Mesh = world.makeTerrain("Obstacle")

	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.6,0.1,0.1,0.9)
	return 



if __name__ == "__main__":
	
	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			


	make_obstacle(world)
				
	
	#sim = Simulator(world)
	robot = world.robot(0)

	vis.add("world",world)
	vis.show()
	
	robotPose = RobotPoser(robot)
	
	collisionTest = WorldCollider(world)




	while(1):
		time.sleep(0.1)
		vis.shown()
		#pass

			#pass
			

