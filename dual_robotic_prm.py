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
_STEPNUM_ = 100



Pose = []
edgeHeat = []
edgeIndex = []



def load_Pose():
	global Pose

	with open('./poseAxis.json','r') as poseFile:
		data = poseFile.read()
		Pose = json.loads(data)
		print Pose
	return

def load_Index():
	global edgeIndex

	with open('./edgeIndex.json','r') as edgeIndexFile:
		data = edgeIndexFile.read()
		edgeIndex = json.loads(data)
		
		print edgeIndex
	return

def load_edgeHeat():
	global edgeHeat

	with open('./edgeHeat.json','r') as edgeHeatFile:
		data = edgeHeatFile.read()
		edgeHeat = json.loads(data)
		
		print edgeHeat
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
		for j in collisionTest.robots[1]:
			if j < 0: continue
			if i < 0: continue
			if collisionTest.geomList[j][1].collides(collisionTest.geomList[i][1]):
				return True
	return False


def ctlRobotRandom():
	axis = [0,		
		random.uniform(-3.1416 , 3.1416),
		random.uniform(-2.0071 , 2.0071),
		random.uniform(-0.6981 , 3.8397),
		random.uniform(-3.1416 , 3.1416),
		random.uniform(-1.3090 , 4.4506),
		random.uniform(-3.1416 , 3.1416)
	]

	ctlRobotPose.set(axis)

	return


def edgeSaftyCheck(startPose,endPose):
	
	stepangle = [0,
	(endPose[0] - startPose[0]) / _STEPNUM_ ,
	(endPose[1] - startPose[1]) / _STEPNUM_ ,
	(endPose[2] - startPose[2]) / _STEPNUM_ ,
	(endPose[3] - startPose[3]) / _STEPNUM_ ,
	(endPose[4] - startPose[4]) / _STEPNUM_ ,
	(endPose[5] - startPose[5]) / _STEPNUM_ ] 


	for step in range(0,_STEPNUM_):

		axis = [ 0,
		startPose[0] + step*stepangle[0],
		startPose[1] + step*stepangle[1],
		startPose[2] + step*stepangle[2],
		startPose[3] + step*stepangle[3],
		startPose[4] + step*stepangle[4],
		startPose[5] + step*stepangle[5] ]		

		prmRobotPose.set(axis)

		if (robotCollideRobot()):
			return False
	return True


def searchPosesInGroup(poseList,PoseNum1,PoseNum2):

	result1 = False
	result2 = False
		
	for pose in :
		if ( pose == PoseNum1 ):
			result1 = True
		if ( pose == PoseNum2 ):
			result2 = True
		if ( result1 == True and result2 == True ):
			break
	return result1, result2




def seekPath(endPoseNum):

	backwardPoseList = [endPoseNum]

	for edge in activeEdgeList:
		result1,result2 = searchPosesInGroup(backwardPoseList,edge[0],edge[1])

		if ( ((result1 == True and result2 == True) 
			or (result1 == False and result2 == False)) ):
			pass

		else:
			for i in range(0,100000):
				if ( edge == edgeIndex[i] ):
					edgeHeat[i] = edgeHeat[i] + 1

			if ( result1 == True and result2 == False ):
				backwardPoseList.append(edge[1])

			elif ( result1 == False and result2 == True ):
				backwardPoseList.append(edge[0])
	return 

def mixCheckMark(poseNum):

	if ( ( (poseNum == 1) and (completeMask[0] == False) ) 
		or ( (poseNum == 2) and (completeMask[1] == False) ) 
		or ( (poseNum == 3) and (completeMask[2] == False) ) 
		or ( (poseNum == 4) and (completeMask[3] == False) ) 
		or ( (poseNum == 5) and (completeMask[4] == False) ) ):
	# mix!
		seekPath(endPoseNum)
		completeMask[poseNum-1] = True
				
	return 


def growGroup():
	# for edgeIndex in range(0,100000):
	# 	edge = edgeBuff[edgeIndex]
	for edge in edgeBuff:
		result1,result2 = searchPosesInGroup(activePoseList,edge[0],edge[1])

		if ( result1 == False and result2 == False ):
			# two pose are not in this edge,find next edge
			pass
		else:
			# at least one pose in edge, no matter how ,this edge should not querry next time
			edgeBuff.remove(edge)

			if ( result1 == True and result2 == True ):
				# a used less edge 
				pass
			else:
				# collision check first
				if ( True == edgeSaftyCheck(edge[0],edge[1]) ):
					activeEdgeList.append( edge )#record as parents

					if ( result1 == True and result2 == False ):
						
						activePoseList.append( edge[1] )		
						# all mix work should be finished in  mixCheckMark()
						mixCheckMark(edge[1])
										
							
					elif ( result1 == False and result2 == True ):
						activeEdgeList.append( edge )#record as parents
						activePoseList.append( edge[0] )
						# all mix work should be finished in  mixCheckMark()
						mixCheckMark(edge[0])
				else:
					pass

	return




def dual_robot_check():
	
	while(1):
		edgeBuff = edgeIndex


		completeMask = [False,False,False,False,False]

	# we define Pose0 as start pose
		activePoseList = [0]
		activeEdgeList = []

		ctlRobotRandom()
		loopCnt = 0;
		while( completeMask != [True,True,True,True,True] ):
			
			growGroup()

			if ( loopCnt > 1000 ):
				break
			loopCnt = loopCnt + 1

		save_edgeHeat()
	return



###########################################


if __name__ == "__main__":

	load_Pose()
	load_Index()
	load_edgeHeat()

	world = WorldModel()

	res = world.readFile('./anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res
			
	

	prmRobot = world.robot(0)
	ctlRobot = world.robot(1)

	vis.add("world",world)
	vis.show()


	collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)
	ctlRobotPose = RobotPoser(ctlRobot)

	dual_robot_check()

	# while(1):
	# 	time.sleep(0.1)
	# 	pass







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
