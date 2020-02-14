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

# 传送带1
	grid.transform([1,0,0,  0,0.1,0,  0,0,0.005],[-0.5,-0.27,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.1,0.1,0.2,0.9)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.02,0,0,  0,0.1,0,  0,0,0.02],[0.47,-0.27,-0.28])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.2,0.1,0.1,0.9)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.02,0,0,  0,0.1,0,  0,0,0.02],[-0.2,-0.27,-0.28])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.2,0.1,0.1,0.9)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.02,0,0,  0,0.1,0,  0,0,0.02],[0.2,-0.27,-0.28])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.2,0.1,0.1,0.9)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.02,0,0,  0,0.1,0,  0,0,0.02],[-0.5,-0.27,-0.28])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.2,0.1,0.1,0.9)

# 传送带2
	grid.loadFile("../terrains/cube.off")
	grid.transform([1,0,0,  0,0.1,0,  0,0,0.005],[-0.5,-0.38,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.1,0.1,0.2,0.9)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.02,0,0,  0,0.1,0,  0,0,0.02],[0.47,-0.38,-0.28])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.2,0.1,0.1,0.9)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.02,0,0,  0,0.1,0,  0,0,0.02],[-0.2,-0.38,-0.28])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.2,0.1,0.1,0.9)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.02,0,0,  0,0.1,0,  0,0,0.02],[0.2,-0.38,-0.28])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.2,0.1,0.1,0.9)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.02,0,0,  0,0.1,0,  0,0,0.02],[-0.5,-0.38,-0.28])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.2,0.1,0.1,0.9)



# 传送带上物品1
	grid.loadFile("../terrains/cube.off")
	grid.transform([0.025,0,0,  0,0.05,0,  0,0,0.05],[-0.3,-0.245,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.7,0.8,0.7,0.99)
	grid.loadFile("../terrains/cube.off")
	grid.transform([0.025,0,0,  0,0.05,0,  0,0,0.05],[0.3,-0.245,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.8,0.7,0.7,0.99)
	grid.loadFile("../terrains/cube.off")
	grid.transform([0.05,0,0,  0,0.05,0,  0,0,0.05],[0,-0.245,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.7,0.7,0.8,0.99)

# 传送带上物品2
	grid.loadFile("../terrains/cube.off")
	grid.transform([0.05,0,0,  0,0.05,0,  0,0,0.05],[-0.3,-0.355,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.7,0.7,0.8,0.99)
	grid.loadFile("../terrains/cube.off")
	grid.transform([0.025,0,0,  0,0.05,0,  0,0,0.05],[0.3,-0.355,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.7,0.8,0.7,0.99)
	grid.loadFile("../terrains/cube.off")
	grid.transform([0.025,0,0,  0,0.05,0,  0,0,0.05],[0,-0.355,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.8,0.7,0.7,0.99)



#码垛1
	grid.loadFile("../terrains/cube.off")
	grid.transform([0.025,0,0,  0,0.05,0,  0,0,0.05],[0,0,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.7,0.8,0.7,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.025,0,0,  0,0.05,0,  0,0,0.05],[0.025,0,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.8,0.7,0.7,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.05,0,0,  0,0.05,0,  0,0,0.05],[0,0,-0.214])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.7,0.7,0.8,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.05,0,0,  0,0.05,0,  0,0,0.05],[0,0,-0.164])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.8,0.8,0.7,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.05,0,0,  0,0.05,0,  0,0,0.05],[0,0,-0.114])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.8,0.7,0.8,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.05,0,0,  0,0.05,0,  0,0,0.05],[0,0,-0.064])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.7,0.8,0.8,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.025,0,0,  0,0.05,0,  0,0,0.05],[-0.025,0,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.7,0.8,0.9,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.025,0,0,  0,0.05,0,  0,0,0.05],[-0.05,0,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.9,0.8,0.7,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.05,0,0,  0,0.05,0,  0,0,0.05],[0.0,0.05,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.5,0.7,0.8,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.05,0,0,  0,0.05,0,  0,0,0.05],[0.0,0.1,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.8,0.7,0.5,0.99)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.05,0,0,  0,0.05,0,  0,0,0.05],[0.0,0.1,-0.214])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.8,0.5,0.8,0.99)

# 物流车
	
	grid.loadFile("../terrains/cube.off")
	grid.transform([0.320,0,0,  0,0.320,0,  0,0,0.01],[-0.16,-0.16,-0.264])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.2,0.1,0.1,0.9)

	grid.loadFile("../terrains/cube.off")
	grid.transform([0.320,0,0,  0,0.02,0,  0,0,0.02],[-0.16,-0.01,-0.284])			
	Mesh = world.makeTerrain("Grid,")
	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.1,0.2,0.1,0.9)


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


	prmRobotPose.set([0,0,0,0,0,0,0,0])
	ctlRobotPose.set([0,0,0,0,0,0,0,0])

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
