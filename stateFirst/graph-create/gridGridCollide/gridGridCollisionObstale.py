# -*- coding: utf-8 -*-
# @File Name: gridGridCollisionObstale.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\gridGridCollide\gridGridCollisionObstale.py
# @Author: Ruige_Lee
# @Date:   2019-02-20 19:40:54
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-02-21 10:49:17
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

	grid.transform([0.115,0,0,  0,0.150,0,  0,0,0.100],[0.23,0,0.20])			

	Mesh = world.makeTerrain("Obstacle")

	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.1,0.1,0.6,0.9)
	return 


def make_obstacle_mesh(world):

	for x in range(0,5):
		for y in range(0,5):
			for z in range(0,5):
				grid = Geometry3D()

				grid.loadFile("../../terrains/cube.off")

				grid.transform([0.0215,0,0,  0,0.0285,0,  0,0,0.0185],[0.23 + 0.023*x,0.030*y,0.20 + 0.020*z])			

				Mesh = world.makeTerrain("Obstacle")

				Mesh.geometry().set(grid)
				Mesh.appearance().setColor(1,0.1,0.1,1)
	return 

if __name__ == "__main__":
	
	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			


	# make_obstacle(world)
	make_obstacle_mesh(world)				
	
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
			

