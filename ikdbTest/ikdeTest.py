# -*- coding: utf-8 -*-
# @File Name: ikdeTest.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\ikdbTest\ikdeTest.py
# @Author: Ruige_Lee
# @Date:   2019-05-18 20:58:59
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-05-18 21:04:44
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/



import ikdb
import klampt
import time
from klampt.model import ik

world = klampt.WorldModel()
#TODO: load the robot or world file that you will be using
world.loadFile('../anno_check.xml')
ikdb.functionfactory.registerDefaultFunctions()
ikdb.functionfactory.registerCollisionFunction(world)
ikdb.functionfactory.registerJointRangeCostFunction(world.robot(0))

problem = []
robot= world.robot(0)
link = robot.link(7)
problem.append(ik.objective(link,R=[1,0,0,0,0,1,0,-1,0],t=[0.3,0,0]))

robot = world.robot(0)
while (True):
	#generate a new problem
	#TODO: add Klamp't IKObjective objects to the list of objectives
	# problem.append(ik.objective(robot.link(...),...))
	# problem.append(ik.objective(robot.link(...),...))

  
	#run the solver
	soln = ikdb.solve(problem,activeDofs=None,feasibilityCheck='collisionFree',costFunction='jointRangeCost')
	if soln is not None:
		print ("Got a solution",soln)

#if you want to auto-populate the database, run the following lines:
time.sleep(600)
ikdb.flush()

