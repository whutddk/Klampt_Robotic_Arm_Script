# -*- coding: utf-8 -*-
# @File Name: gridGridCollisionShow.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\gridGridCollide\gridGridCollisionShow.py
# @Author: Ruige_Lee
# @Date:   2019-02-20 19:40:54
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-02-20 21:20:34
# @Email: 295054118@whut.edu.cn"



from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis

import json

import random

global gridList
gridList = []

 

def make_show_mesh(world):
	"""automatically create a mesh test grid
	"""
	global gridList

	print (gridList)
	for index in gridList:
		x = index[0]
		y = index[1]
		z = index[2]

		grid = Geometry3D()

		grid.loadFile("../../terrains/cube.off")

		grid.transform([0.023,0,0,  0,0.030,0,  0,0,0.020],[0.023*x + 0.120,0.030*y-0.480,0.020*z])			

		Mesh = world.makeTerrain("showGrid," + "%3d"%x + "," + "%3d"%y + "," + "%3d"%z)

		Mesh.geometry().set(grid)
		Mesh.appearance().setColor(0.1,0.1,0.2,0.3)
	return 


if __name__ == "__main__":
	
	# global gridList

	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			
	with open('./collideGrid.json','r') as poseFile:
		data = poseFile.read()
		gridList = json.loads(data)
		
	pass

	make_show_mesh(world)
	


	collisionTest = WorldCollider(world)

	vis.add("world",world)
	vis.show()

	while(1):
		time.sleep(0.1)
		vis.shown()
		#pass

			#pass
			

