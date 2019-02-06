# @File Name: dual_robotic_prm.py
# @File Path: /home/whutddk/Klampt/klampt_robotic_arm_script/dual_robotic_prm/dual_robotic_prm.py
# @Author: whutddkUbuntu16
# @Date:   2018-12-27 19:14:09
# @Last Modified by:   whutddkUbuntu16
# @Last Modified time: 2019-01-28 15:05:50
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
_STEPNUM_ = 25



jointList = []
edgeHeat = []
edgeIndex = []

edgeBuff = edgeIndex
completeMask = [False,False,False,False,False,False,False,False]
activePoseList = []
activeEdgeList = []
backwardPoseList = []


def load_jointList():
	global jointList

	with open('../../result/create_Edge_3m200ms/jointList.json','r') as jointListFile:
		data = jointListFile.read()
		jointList = json.loads(data)
		# print jointList
	return

def load_Index():
	global edgeIndex

	with open('../../result/create_Edge_3m200ms/edgeIndex.json','r') as edgeIndexFile:
		data = edgeIndexFile.read()
		edgeIndex = json.loads(data)
		
		# print edgeIndex
	return

def load_edgeHeat():
	global edgeHeat

	with open('./edgeHeat.json','r') as edgeHeatFile:
		data = edgeHeatFile.read()
		edgeHeat = json.loads(data)
		
		# print edgeHeat
	return

def save_edgeHeat():
	global edgeHeat

	with open('./edgeHeat.json','w') as edgeHeatFile:
		data = json.dumps(edgeHeat)
		edgeHeatFile.write(data)

	return


def robotCollideRobot():
	collisionTest = WorldCollider(world)
	for i in collisionTest.robots[0]:
		if i < 0: continue
		for j in collisionTest.robots[1]:
			if j < 0: continue		
			if collisionTest.geomList[j][1].collides(collisionTest.geomList[i][1]):
				return True
		for j in collisionTest.robots[2]:
			if j < 0: continue		
			if collisionTest.geomList[j][1].collides(collisionTest.geomList[i][1]):
				return True
	return False




def edgeSaftyCheck(startPoseNum,endPoseNum):
	global jointList


	endPose = jointList[endPoseNum]
	startPose = jointList[startPoseNum]

	stepAngle = [(endPose[0] - startPose[0]) / _STEPNUM_ ,(endPose[1] - startPose[1]) / _STEPNUM_ ,(endPose[2] - startPose[2]) / _STEPNUM_ ,(endPose[3] - startPose[3]) / _STEPNUM_ ,(endPose[4] - startPose[4]) / _STEPNUM_ ,(endPose[5] - startPose[5]) / _STEPNUM_ ] 


	for step in range(0,_STEPNUM_ + 1 ):
		# time.sleep(0.001)
		axis = [ 0,	startPose[0] + step*stepAngle[0],startPose[1] + step*stepAngle[1],startPose[2] + step*stepAngle[2],startPose[3] + step*stepAngle[3],startPose[4] + step*stepAngle[4],startPose[5] + step*stepAngle[5] ]		

		prmRobotPose.set(axis)

		if (robotCollideRobot()):
			return False
	return True


def searchPosesInGroup(poseList,PoseNum1,PoseNum2):

	result1 = False
	result2 = False
		
	for pose in poseList:
		if ( pose == PoseNum1 ):
			result1 = True
		if ( pose == PoseNum2 ):
			result2 = True
		if ( result1 == True and result2 == True ):
			break
	return result1, result2




def seekPath(endPoseNum):

	global activeEdgeList
	global edgeHeat
	global edgeIndex
	global backwardPoseList


	backwardPoseTemp = [endPoseNum]
	backwardPoseList.append(endPoseNum)
	while(1):
		for edge in activeEdgeList:
			# print "edge"
			# print edge
			result1,result2 = searchPosesInGroup(backwardPoseTemp,edge[0],edge[1])

			if ( ((result1 == True and result2 == True) 
				or (result1 == False and result2 == False)) ):
				pass

			elif ( result1 == True and result2 == False ):
	

				print "heat:" 
				print edge
				for i in range(0,100000):
					if ( edge == edgeIndex[i] ):
						edgeHeat[i] = edgeHeat[i] + 1

				# elif ( result1 == False and result2 == True ):

				# 	backwardPoseList.append(edge[0])
				# 	backwardPoseTemp.append(edge[0])

				res1,res2 = searchPosesInGroup(backwardPoseList,edge[1],edge[1])
				if ( res1 == True and res2 == True  ):
					print backwardPoseList
					print backwardPoseTemp
					return

				backwardPoseList.append(edge[1])
				backwardPoseTemp.append(edge[1])

	# print backwardPoseList
	# print backwardPoseTemp

	# print "break"

	return 

def mixCheckMark(poseNum):
	global completeMask

	if ( ( (poseNum == 1) and (completeMask[0] == False) ) 
		or ( (poseNum == 2) and (completeMask[1] == False) ) 
		or ( (poseNum == 3) and (completeMask[2] == False) ) 
		or ( (poseNum == 4) and (completeMask[3] == False) ) 
		or ( (poseNum == 5) and (completeMask[4] == False) )
		or ( (poseNum == 6) and (completeMask[5] == False) ) 
		or ( (poseNum == 7) and (completeMask[6] == False) ) 
		or ( (poseNum == 8) and (completeMask[7] == False) ) 
		  ):


	# mix!
		seekPath(poseNum)
		completeMask[poseNum-1] = True
		print completeMask
				
	return 


def growGroup():

	global edgeBuff
	global completeMask
	global activePoseList
	global activeEdgeList

	# for edgeIndex in range(0,100000):
	# 	edge = edgeBuff[edgeIndex]

	edgeTemp = edgeBuff[:]
	print len( activeEdgeList )
	for edge in edgeTemp:

		if (completeMask == [True,True,True,True,True,True,True,True]):
			break

		result1,result2 = searchPosesInGroup(activePoseList,edge[0],edge[1])

		if ( result1 == False and result2 == False ):
			# two pose are not in this edge,find next edge
			# print "error edge"
			# print edge
			pass
		else:
			# at least one pose in edge, no matter how ,this edge should not querry next time
			edgeBuff.remove(edge)
			if ( result1 == True and result2 == True ):
				# a used less edge 
				pass
			else:
				# collision check first
				# print edge
				if ( True == edgeSaftyCheck(edge[0],edge[1]) ):
					

					if ( result1 == True and result2 == False ):
						
						activePoseList.append( edge[1] )		
						# all mix work should be finished in  mixCheckMark()
						activeEdgeList.append( [edge[1],edge[0]] )#record as parents
						mixCheckMark(edge[1])
						# print edge[1]
							
					elif ( result1 == False and result2 == True ):

						activePoseList.append( edge[0] )
						# all mix work should be finished in  mixCheckMark()
						activeEdgeList.append( [edge[0],edge[1]] )#record as parents
						mixCheckMark(edge[0])
						# print edge[0]
				else:
					pass

	return




def trian_robot_check():
	global jointList
	global edgeBuff
	global edgeIndex
	global completeMask
	global activePoseList
	global activeEdgeList
	global backwardPoseList



	for sh in range ( 0,6 ):
		for ar in range ( 1,11 ):
			for el in range ( 1,11 ):
				print "new session"
				print "sh"
				print sh
				print "ar"
				print ar
				print "el"
				print el

				edgeBuff = edgeIndex[:]

				completeMask = [False,False,False,False,False,False,False,False]

			# we define Pose0 as start pose
				activePoseList = [0]
				activeEdgeList = []
				backwardPoseList = [0]




	
				ctlRobotPoseLeft.set([0,-1.57+0.314*sh,-2.0071+0.20071*ar,-0.6981 + 0.22681*el,0,1.57,0])
				ctlRobotPoseRight.set([0,0.314*sh,-2.0071+0.20071*ar,-0.6981 + 0.22681*el,0,1.57,0])
				prmRobotPose.set([0,0,-_PI_/2,_PI_/2,0,_PI_/2,0,0])
				#print robotCollideRobot()
				#time.sleep(0.1)
				for finalPoseNum in range(1,9):
					axis = jointList[finalPoseNum][:]
					axis.insert(0,0)
					prmRobotPose.set(axis)
					if (robotCollideRobot()):
						completeMask[finalPoseNum - 1] = True


				preActiveEdge = 0

				while( completeMask != [True,True,True,True,True,True,True,True] ):
					print "next loop"
					growGroup()

					curActiveEdge = len( activeEdgeList )
					if ( curActiveEdge == preActiveEdge ):
						print "check fail to finish!!!!!!!!!!!!!!!!"
						break
					else:
						preActiveEdge = curActiveEdge
					

				save_edgeHeat()
	return



###########################################


if __name__ == "__main__":

	load_jointList()
	load_Index()
	load_edgeHeat()

	world = WorldModel()

	res = world.readFile('../../trian_anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	prmRobot = world.robot(0)
	ctlRobotLeft = world.robot(1)
	ctlRobotRight = world.robot(2)
	 # vis.add("world",world)
	# vis.show()



	collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)
	ctlRobotPoseLeft = RobotPoser(ctlRobotLeft)
	ctlRobotPoseRight = RobotPoser(ctlRobotRight)


	trian_robot_check()

	# while(1):
		# time.sleep(0.1)
		# pass




