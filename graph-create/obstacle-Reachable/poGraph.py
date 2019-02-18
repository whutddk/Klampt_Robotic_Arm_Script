# -*- coding: utf-8 -*-
# @File Name: poGraph.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\obstacle-Reachable\poGraph.py
# @Author: Ruige_Lee
# @Date:   2019-02-18 09:11:50
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-02-18 10:16:37
# @Email: 295054118@whut.edu.cn"

# @File Name: poGraph.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\graph-create\obstacle-Reachable\poGraph.py
# @Author: 29505
# @Date:   2019-02-07 09:33:58
# @Last Modified by:   29505
# @Last Modified time: 2019-02-14 22:01:40
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


def make_testing_mesh(world):
	"""automatically create a mesh test grid
	"""

	for z in range(0,32):
		for rad in range (0,32):
			for x in range (0,16):

				grid = Geometry3D()

				grid.loadFile("../../gridModelEncode/PolarCoordinates/polarModel/trapezoid"+ str(x) +"_" +str(rad) +'.off')

				grid.transform([1,0,0,0,1,0,0,0,1],[0,0,0.020*z])			

				Mesh = world.makeTerrain("Grid," + "%3d"%x + "," + "%3d"%rad + "," + "%3d"%z)

				Mesh.geometry().set(grid)
				Mesh.appearance().setColor(0.1,0.5,0.1,0.08)
	return 



if __name__ == "__main__":
	
	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			

	make_testing_mesh(world)
				
	robot = world.robot(0)

	vis.add("world",world)
	vis.show()

	collisionTest = WorldCollider(world)
	
	robotPose = RobotPoser(robot)
	


	while(1):
		time.sleep(0.1)
		vis.shown()
		pass


			

