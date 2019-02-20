# -*- coding: utf-8 -*-
# @File Name: gridGridCollisionGet.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\gridGridCollide\gridGridCollisionGet.py
# @Author: Ruige_Lee
# @Date:   2019-02-20 19:40:54
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-02-20 21:34:21
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
				Mesh.appearance().setColor(0.1,0.1,0.2,0.0)
	return 

def record_edge_grid():

	global gridList

	Pose1 = [0,0,0,0,0,0,0]
	Pose2 = [0,-1.57,1.57,0,0,0,0]

	shoulderStart = Pose1[0]
	armStart = Pose1[1]
	elbowStart = Pose1[2]
	wristStart = Pose1[3]
	fingerStart = Pose1[4]
	toolStart = Pose1[5]

	shoulderEnd = Pose2[0]
	armEnd = Pose2[1]
	elbowEnd = Pose2[2]
	wristEnd = Pose2[3]
	fingerEnd = Pose2[4]
	toolEnd = Pose2[5]

	shoulderDis = (shoulderEnd - shoulderStart) / 100
	armDis = (armEnd - armStart) / 100
	elbowDis = ( elbowEnd - elbowStart ) / 100
	wristDis = ( wristEnd - wristStart ) / 100
	fingerDis = ( fingerEnd - fingerStart ) / 100
	toolDis = ( toolEnd - toolStart ) / 100

	oneEdge = [0 for m in range(0,16384)]

	for k in range (0,101):
		#time.sleep(0.01)
		robotPose.set([0,
			(shoulderStart + shoulderDis*k), 
			(armStart + armDis*k),
			(elbowStart + elbowDis*k), 
			(wristStart + wristDis*k),
			(fingerStart + fingerDis*k),
			(toolStart + toolDis*k),
			0])
		collisionTest = WorldCollider(world)

		for p,q in collisionTest.robotTerrainCollisions(0):
			result = q.getName()
			
			#print q.getName()
			x = int(result[5:8])
			y = int(result [9:12])
			z = int(result[13:16])
			oneEdge[1024*x+32*y+z] = 1
	

	for cnt in range(0,16384):
		if (oneEdge[cnt] == 1):
			x = cnt // 1024
			y = cnt % 1024 // 32
			z = cnt % 32

			gridList.append([x,y,z])
			
	with open('./collideGrid.json','w') as edgeFile:
		data = json.dumps(gridList)
		edgeFile.write(data)
	pass
			
	print (gridList)
	pass





if __name__ == "__main__":
	
	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			


	make_testing_mesh(world)
				
	
	#sim = Simulator(world)
	robot = world.robot(0)

	vis.add("world",world)
	vis.show()
	
	robotPose = RobotPoser(robot)
	
	collisionTest = WorldCollider(world)

	record_edge_grid()


	while(1):
		# time.sleep(0.1)
		vis.shown()
		#pass

			#pass
			

