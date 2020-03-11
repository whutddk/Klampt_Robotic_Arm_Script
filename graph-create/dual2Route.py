# -*- coding: utf-8 -*-
# @File Name: dual_robotic_prm.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\dual_robotic_prm\dual_robotic_prm.py
# @Author: Ruige_Lee
# @Date:   2019-02-18 11:32:13
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-04-15 11:36:16
# @Email: 295054118@whut.edu.cn"

# @File Name: dual_robotic_prm.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\dual_robotic_prm\dual_robotic_prm.py
# @Author: whutddkUbuntu16
# @Date:   2018-12-27 19:14:09
# @Last Modified by:   29505
# @Last Modified time: 2019-02-16 16:11:09
# @Email: 295054118@whut.edu.cn
from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis


import random

################################################



_PI_ = 3.14159 
_STEPNUM_ = 25


def create_cube(world):
	grid = Geometry3D()

	grid.loadFile("../terrains/cube.off")

	for x in range(0,16):
		for y in range(0,16):
			for z in range(0,16):
				if x < 2:
					x_size = 0.015
					x_loc = -0.16+0.015*x
				elif x < 6:
					x_size = 0.020
					x_loc = -0.13+0.020*(x-2)
				elif x < 10:
					x_size = 0.025
					x_loc = -0.05+0.025*(x-6)
				elif x < 14:
					x_size = 0.020
					x_loc = 0.05+0.020*(x-10)
				else:
					x_size = 0.015 
					x_loc = 0.13+0.015*(x-14)
				
				if y < 2:
					y_size = 0.015
					y_loc = -0.16+0.015*y
				elif y < 6:
					y_size = 0.020
					y_loc = -0.13+0.020*(y-2)
				elif y < 10:
					y_size = 0.025
					y_loc = -0.05+0.025*(y-6)
				elif y < 14:
					y_size = 0.020
					y_loc = 0.05+0.020*(y-10)
				else:
					y_size = 0.015 
					y_loc = 0.13+0.015*(y-14)

				if z < 2:
					z_size = 0.015
					z_loc = -0.264+0.015*z
				elif z < 6:
					z_size = 0.020
					z_loc = -0.264 + 0.03 +0.020*(z-2)
				elif z < 10:
					z_size = 0.025
					z_loc = -0.264 + 0.11 +0.025*(z-6)
				elif z < 14:
					z_size = 0.020
					z_loc = -0.264 + 0.21 +0.020*(z-10)
				else:
					z_size = 0.015 
					z_loc = -0.264 + 0.29 + 0.015*(z-14)

				if (
					( x == 10 and y == 15 and z == 0) or
					( x == 9 and y == 14 and z == 0) or
					( x == 10 and y == 14 and z == 0) or
					( x == 9 and y == 13 and z == 0) or
					( x == 10 and y == 13 and z == 0) or

					( x == 10 and y == 15 and z == 1) or
					( x == 9 and y == 14 and z == 1) or
					( x == 10 and y == 14 and z == 1) or
					( x == 9 and y == 13 and z == 1) or
					( x == 10 and y == 13 and z == 1) or

					( x == 10 and y == 15 and z == 2) or
					( x == 9 and y == 14 and z == 2) or
					( x == 10 and y == 14 and z == 2) or
					( x == 9 and y == 13 and z == 2) or
					( x == 10 and y == 13 and z == 2) or

					( x == 10 and y == 15 and z == 3) or
					( x == 9 and y == 14 and z == 3) or
					( x == 10 and y == 14 and z == 3) or
					( x == 9 and y == 13 and z == 3) or
					( x == 10 and y == 13 and z == 3)
					):
					grid.loadFile("../terrains/cube.off")
					grid.transform([x_size,0,0,  0,y_size,0,  0,0,z_size],[x_loc,y_loc,z_loc])			
					Mesh = world.makeTerrain("Grid,")
					Mesh.geometry().set(grid)
					Mesh.appearance().setColor(0.5*(1+x%2),0.5*(1+y%2),0.5*(1+z%2),1)




if __name__ == "__main__":


	world = WorldModel()

	res = world.readFile('../dual_anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	prmRobot = world.robot(0)
	ctlRobot = world.robot(1)
	create_cube(world)
	vis.add("world",world)
	vis.show()


	# collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)
	ctlRobotPose = RobotPoser(ctlRobot)


	# prmRobotPose.set([0,-0.7514, -1.1506, 0.3394, 0.0, 1.493, -0.211,0])
	# ctlRobotPose.set([0,-1.5722, -1.5698, 0.0, 0.0, 1.5698, -0.0026,0])
	# prmRobotPose.set([0,-0.6286, -1.15, 0.34, 0.0, 1.4898, -0.2194,0])
	# ctlRobotPose.set([0,-1.5722, -1.5698, 0.0, 0.0, 1.5698, -0.0026,0])


	# prmRobotPose.set([0,0.2318, -1.15, 0.34, 0.0, 1.4898, -0.2194,0])
	# ctlRobotPose.set([0,-1.5722, -1.5698, 0.0, 0.0, 1.5698, -0.0026,0])

	# prmRobotPose.set([0,0.6734, -1.1412, 0.2, -0.1044, 1.7514,0,0])
	# prmRobotPose.set([0,0.6734, -1.1412, 0.5736, -0.1044, 1.7514,0,0])
	# ctlRobotPose.set([0,0.001, -1.5712, 0.0042, 0.0, 1.5698, -0.0022,0])

	prmRobotPose.set([0,-0.6464, -1.0006, 0.1146, 0.0102, 1.4632,0,0])
	# prmRobotPose.set([0,-0.7464, -1.1406, 0.4146, 0.0102, 1.4632,0,0])
	ctlRobotPose.set([0,0,-2.18,1.99,0,1.57,0,0])



	while(1):
		time.sleep(0.1)
		pass







# set ctlRobot Random Pose

# 	set 0,1,2,3,4 active 
# while(!10 group complete)
# 	for 10W edge:
# 		if ( one active  another not ) in all group:
# 			collision check
# 			if pass :
# 				activate another
# 				finish check
# 					if finish :
# 						used edgeNum ++
# 					  finished group mark 
				
			
# 			delete edge from 10W edgeList
