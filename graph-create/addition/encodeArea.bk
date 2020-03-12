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
import math
################################################



_PI_ = 3.14159 
_STEPNUM_ = 25




def create_cube(world):
	grid = Geometry3D()

	grid.loadFile("./cube.off")




# 作图思路，
# 先笛卡尔全覆盖
# 扫描天顶角+90~-36.3,半径142.1~482.3，
# 扫描天顶角-36.3~-90，先全覆盖，再清除超范围

# 先作二三连杆扫描区域，球心(-325,0,0)天顶角90~-36.3,半径142.1~482.3，方位角-90~-90
# 

	for x in range(0,20):
		for y in range ( -20, 20):
			for z in range ( -20,20 ):


				# 覆盖第一个球
				x_loc = 0.025*x-0.325
				y_loc = 0.025*y
				z_loc = 0.025*z

				radius = math.sqrt((x_loc + 0.325) * (x_loc + 0.325) + y_loc * y_loc + z_loc*z_loc)
				if ( (radius > 0.1421 and radius < 0.4823) and (z_loc > -0.264) ):


					grid.loadFile("./cube.off")
					grid.transform([0.025,0,0,  0,0.025,0,  0,0,0.025],[-0.325+0.025*x,0.025*y,0.025*z])			
					Mesh = world.makeTerrain("Grid,")
					Mesh.geometry().set(grid)
					Mesh.appearance().setColor(0.5,0.5,0.5,0.5)




if __name__ == "__main__":


	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	prmRobot = world.robot(0)

	create_cube(world)
	vis.add("world",world)
	vis.show()


	# collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)



	prmRobotPose.set([0,0,0,0,0,0,0,0])


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
