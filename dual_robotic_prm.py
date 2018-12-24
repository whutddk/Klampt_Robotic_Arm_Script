from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis

import json

import random

pi = 3.14159 

Pose = []

def load_Pose():
	global Pose

	with open('./poseAxis.json','r') as poseFile:
		data = poseFile.read()
		Pose = json.loads(data)
		print Pose
	pass


def robotCollideRobot():
	collisionTest = WorldCollider(world)
	for i in collisionTest.robots[0]:
		for j in collisionTest.robots[1]:
			if j < 0: continue
			if i < 0: continue
			if collisionTest.geomList[j][1].collides(collisionTest.geomList[i][1]):
				return True
	return False



if __name__ == "__main__":
	# load_Pose()
	world = WorldModel()

	res = world.readFile('./anno_check.xml')
	if not res:`
		raise RuntimeError("Unable to load model ") 
			
	vis.add("world",world)

	prmRobot = world.robot(0)
	ctlRobot = world.robot(1)
	vis.show()
	collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)
	ctlRobotPose = RobotPoser(ctlRobot)


	prmRobotPose.set([0,pi/2,-pi/2, 0,   0, pi/2,0])
	

	for k in range (0,50):
		ctlRobotPose.set([0,-pi/100 * k + pi/2,   -pi/2    ,pi/2, 0, pi/2,0])
		collisionTest = WorldCollider(world)
		time.sleep(0.1)
	# for l in collisionTest.geomList[1]:
	# 	print l
		
		if (robotCollideRobot()):
			print "collision!"


		



	while(1):
		time.sleep(0.1)
		vis.show()
		pass










while(1):
	edgeBuff = edgeIndex

	activeEdgeList = []

	# 				01		02	03		04	12		13	14		23	24		34
	completeMask = [False,False,False,False,False,False,False,False,False,False]

	searchGroup = [[],[],[],[],[]]
	searchGroup[0] = [0]
	searchGroup[1] = [1]
	searchGroup[2] = [2]
	searchGroup[3] = [3]
	searchGroup[4] = [4]

	ctlRobotRandom()

	while( completeMask != [True,True,True,True,True,True,True,True,True,True] ):
		if ( (completeMask[3] == False) and (completeMask[6] == False) and (completeMask[8] == False) and (completeMask[9] == False) ):
			growGroup(4)
			pass
		if ( (completeMask[2] == False) and (completeMask[5] == False) and (completeMask[7] == False) ):
			growGroup(3)
			pass
		if ( (completeMask[1] == False) and (completeMask[4] == False) ):
			growGroup(2)
			pass
		if ( (completeMask[0] == False) ):
			growGroup(1)
			pass
		if ( True ):
			growGroup(0)
			pass 

		if ( stack ):
			break

return

############################################################################


def ctlRobotRandom():
	pass


def growGroup(groupNum):
	for edge in edgeBuff:
		result1,result2 = searchPosesInGroup(groupNum,edge[0],edge[1])

		if ( result1 == False and result2 == False ):
			# two pose are not in this edge,find next edge
			pass
		else:
			# at least one pose in edge, no matter how ,this edge should not querry next time
			edgeBuff_del_edge()

			if ( result1 == True and result2 == True ):
				# a used less edge 
				pass
			else:
				# collision check first
				if ( False == edgeSaftyCheck(edge[0],edge[1]) ):

					continue

				if ( result1 == True and result2 == False ):
					
					activeEdgeList.append([edge[0],edge[1]])#record as parents
					searchGroup[groupNum].append(edge[1])

					# all mix work should be finished in  mixCheckMark()
					mixCheckMark(groupNum,edge[1])
						
				elif ( result1 == False and result2 == True ):

					activeEdgeList.append([edge[1],edge[0]])#record as parents
					searchGroup[groupNum].append(edge[0])

					# all mix work should be finished in  mixCheckMark()
					mixCheckMark(groupNum,edge[0])
					

	return

#################################################################

def searchPosesInGroup(groupNum,PoseNum1,PoseNum2):

	result1 = False
	result2 = False

	for pose in searchGroup[groupNum]:
		if ( pose == PoseNum1 ):
			result1 = True
		if ( pose == PoseNum2 ):
			result2 = True
		if ( result1 == True and result2 == True ):
			break
	return result1, result2


def mixCheckMark(localGroupNum,mixPoseNum):

	result = False
	for mixGroupNum in range(0,5):
		if ( localGroupNum == mixGroupNum ):
			continue

		if ( searchPosesInGroup(mixGroupNum,mixPoseNum,mixPoseNum) == True,True ):
			# mix!
			seekPath(localGroupNum,mixGroupNum,mixPoseNum)
			markFlag()
			result = True
		
	return result

def seekPath(groupNum1,groupNum2,mixPose):
	pass

def markFlag()
	pass




# set ctlRobot Random Pose

#	set 0,1,2,3,4 active 
# while(!10 group complete)
	# for 10W edge:
		# if ( one active  another not ) in all group:
			# collision check
			# if pass :
				# activate another
				# finish check
					# if finish :
					# 	used edgeNum ++
					#   finished group mark 
				
			
			# delete edge from 10W edgeList
