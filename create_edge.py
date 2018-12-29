# @File Name: create_edge.py
# @File Path: /home/whutddk/Klampt/klampt_robotic_arm_script/create_edge.py
# @Author: whutddkUbuntu16
# @Date:   2018-12-29 14:36:47
# @Last Modified by:   whutddkUbuntu16
# @Last Modified time: 2018-12-29 14:45:09
# @Email: 295054118@whut.edu.cn
from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis

import json

import random

################################################
 

_PI_ = 3.14159 


jointList = []
edgeIndexList = []
edgeIndex = []

def load_data():
	global edgeIndex
	global jointList

	with open('./jointList.json','r') as jointListFile:
			
		data = jointListFile.read()
		jointList = json.loads(data)
	
	with open('./edgeIndex.json','r') as edgeIndexFile:
			
		data = edgeIndexFile.read()
		edgeIndex = json.loads(data)



def save_jointList():
	global jointList
	with open('./jointList.json','w') as jointListFile:
		
		data = json.dumps(jointList)
		jointListFile.write(data)
	pass
	

def save_edgeIndex():
	global edgeIndex
	with open('./edgeIndex.json','w') as edgeIndexFile:
		
		data = json.dumps(edgeIndex)
		edgeIndexFile.write(data)
	pass




def robotSelfCollideCheck():
	collisionTest = WorldCollider(world)
	# for i in collisionTest.robots[0]:
	for i in range(0,8):
		for j in range (0,8):
			if ( ((i+1) == j) or ((i-1) == j) or ( i==j ) ):
				continue
			
			if collisionTest.geomList[j][1].collides(collisionTest.geomList[i][1]):
	 			return True
	return False


def prmRobotRandom():
	axisSave = [		
		random.uniform(-3.1416 , 3.1416),
		random.uniform(-2.0071 , 2.0071),
		random.uniform(-0.6981 , 3.8397),
		random.uniform(-3.1416 , 3.1416),
		random.uniform(-0.2617 , 3.4032),
		random.uniform(-3.1416 , 3.1416)
	]
	axis = axisSave[:]
	axis.insert(0,0)

	prmRobotPose.set(axis)

	return axisSave

def edge_constraint(jointSet):
	global jointList
	global edgeIndex
	global edgeNum

	jointLenth = len(jointList)


	for preIndex in range(0,jointLenth):
	# for prePose in poseList:

		if ( ( abs(jointList[preIndex][0] - jointSet[0]) < (0.417 / 180 * _PI_ * 100) ) and
			( abs(jointList[preIndex][1] - jointSet[1]) < (0.183 / 180 * _PI_ * 100) ) and
			( abs(jointList[preIndex][2] - jointSet[2]) < (0.25 / 180 * _PI_ * 100) ) and
			( abs(jointList[preIndex][3] - jointSet[3]) < (0.2 / 180 * _PI_ * 100) ) and
			( abs(jointList[preIndex][4] - jointSet[4]) < (0.2 / 180 * _PI_ * 100) ) and
			( abs(jointList[preIndex][5] - jointSet[5]) < (0.543 / 180 * _PI_ * 100) ) ):
			
			

			edgeNum = edgeNum + 1
			edge = [preIndex,jointLenth]
			edgeIndex.append(edge)
			jointList.append(jointSet)

			# save_edgeIndex()
			# save_jointList()
			print edge
	print len(edgeIndex)


###############################



if __name__ == "__main__":


	world = WorldModel()

	res = world.readFile('./anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	prmRobot = world.robot(0)

	vis.add("world",world)
	vis.show()


	collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)


	load_data()

	edgeNum = len(edgeIndex)
	print edgeNum


	while(edgeNum < 100000):

		# time.sleep(0.1)
		jointSet = prmRobotRandom()
		if ( False == robotSelfCollideCheck() ):
			
			edge_constraint(jointSet)





	while(1):
		time.sleep(0.1)
		pass




