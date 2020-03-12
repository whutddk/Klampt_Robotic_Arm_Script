# @File Name: spMain.py
# @File Path: K:\work\MAS2\Klampt_Robotic_Arm_Script\gridModelEncode\sphericalCoordinate\spMain.py
# @Author: 29505
# @Date:   2019-02-07 09:33:58
# @Last Modified by:   29505
# @Last Modified time: 2019-02-09 11:45:53
# @Email: 295054118@whut.edu.cn

from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis

import json

import random
import math

Pose = []
edge = []
edgeIndex = []

trueTable = []

def make_testing_mesh(world):
	"""automatically create a mesh test grid
	"""
	for z in range(0,32):
		for y in range (0,32):
			for x in range (0,16):

				radius = 0.120 + 0.023*x   #半径
				theta = -1.3197 + 0.0825*y #方位角
				fieta = -1.57 + 0.098*z    #天顶角

				XTest = (radius + 0.023) * math.cos(theta) * math.cos(fieta)
				ZMinTest = (radius + 0.023) * math.sin(fieta+0.098)+0.264
				ZMaxTest = (radius ) * math.sin(fieta)+0.264

				# if ( XTest < 0.1421): 
				# 	continue
				if (ZMinTest < 0):
					continue
				# if (ZMaxTest > 0.640):
				# 	continue

				grid = Geometry3D()

				grid.loadFile('./spModel/trapezoid'+str(x)+'_'+str(y)+'_'+str(z)+'.off')

				grid.transform([1,0,0,0,1,0,0,0,1],[0,0,0])			

				Mesh = world.makeTerrain("Grid," + "%3d"%x + "," + "%3d"%y + "," + "%3d"%z)

				Mesh.geometry().set(grid)
				Mesh.appearance().setColor(0.3,0.1,0.1,0.1)
	return 





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

	collisionTest = WorldCollider(world)
	
	robotPose = RobotPoser(robot)

	while(1):
		time.sleep(0.1)
		pass



			

