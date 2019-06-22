# -*- coding: utf-8 -*-
# @File Name: offlineCollisionCheck.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\state2\offLineCollisionChecking\offlineCollisionCheck.py
# @Author: Ruige_Lee
# @Date:   2019-06-18 19:35:33
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-06-22 16:19:02
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/



from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis





poseList = []
edgeList = []


def checkingPose():
	with open('../createCheckpointNetwork/poseTable.txt','r') as poseTableFile:
		for line in poseTableFile.readlines():
			strTemp = ''
			Pose = []
			for char in line:
				if char == ',':
					data = float(strTemp)
					Pose.append(data)
					strTemp = ''
					continue
				if char == '\n':
					data = float(strTemp)
					Pose.append(data)
					poseList.append(Pose)
					# print (Pose)
					continue
				strTemp = strTemp + char
		# print (poseList)


def checkingEdge():
	with open('../createEdgeAddressing/edgeAddressing.txt','r') as edgeTableFile:	
		for line in edgeTableFile.readlines():
			strTemp = ''
			Edge = []
			for char in line:
				if char == ',':
					data = int(strTemp)
					Edge.append(data)
					strTemp = ''
					continue
				if char == '\n':
					data = int(strTemp)
					Edge.append(data)
					edgeList.append(Edge)
					# print (Edge)
					continue
				strTemp = strTemp + char
		# print (edgeList)
	pass

def checkingGrid(process):
	gridCnt = 0
	with open('./gridEncode'+ str(process) +'.txt','r') as gridEncodeFile:
		for line in gridEncodeFile.readlines():
			if line[0] == "[" and line[-2] == "]":
				gridCnt = gridCnt + 1
		print (gridCnt)
	return gridCnt


def checkPointReflash(process,data):
	with open('./gridEncode'+ str(process) +'.txt','a') as gridEncodeFile:
		gridEncodeFile.write(str(data))
		gridEncodeFile.write('\n')
	pass





def make_testing_mesh(world):
	"""automatically create a mesh test grid
	"""

	for z in range(0,32):
		for y in range (0,32):
			for x in range (0,16):
				grid = Geometry3D()

				grid.loadFile("../../terrains/cube.off")

				grid.transform([0.025,0,0,  0,0.025,0,  0,0,0.025],[0.025*x + 0.120,0.025*y-0.400,0.025*z])			

				Mesh = world.makeTerrain("Grid," + "%3d"%x + "," + "%3d"%y + "," + "%3d"%z)

				Mesh.geometry().set(grid)
				Mesh.appearance().setColor(0.1,0.1,0.2,0.1)
	return 


def create_Edge(process,Index):

	global poseList
	global edgeList


	print ("now Create Edge:")
	print (Index)

	i = edgeList[Index][0]
	j = edgeList[Index][1]

	shoulderStart = poseList[j][0]
	armStart = poseList[j][1]
	elbowStart = poseList[j][2]
	wristStart = poseList[j][3]
	fingerStart = poseList[j][4]
	toolStart = poseList[j][5]

	shoulderEnd = poseList[i][0]
	armEnd = poseList[i][1]
	elbowEnd = poseList[i][2]
	wristEnd = poseList[i][3]
	fingerEnd = poseList[i][4]
	toolEnd = poseList[i][5]

	shoulderDis = (shoulderEnd - shoulderStart) / 100
	armDis = (armEnd - armStart) / 100
	elbowDis = ( elbowEnd - elbowStart ) / 100
	wristDis = ( wristEnd - wristStart ) / 100
	fingerDis = ( fingerEnd - fingerStart ) / 100
	toolDis = ( toolEnd - toolStart ) / 100

	oneEdge = [0 for m in range(0,16384)]

	for k in range (0,101):
		#time.sleep(0.01)
		robotPose.set([0,
			(shoulderStart + shoulderDis*k), 
			(armStart + armDis*k),
			(elbowStart + elbowDis*k), 
			(wristStart + wristDis*k),
			(fingerStart + fingerDis*k),
			(toolStart + toolDis*k),
			0])
		collisionTest = WorldCollider(world)

		cnt = 0;
		for p,q in collisionTest.robotTerrainCollisions(0):
			result = q.getName()
			
			#print q.getName()
			x = int(result[5:8])
			y = int(result [9:12])
			z = int(result[13:16])
			oneEdge[512*z+16*y+x] = 1
			cnt = cnt + 1;


		print ("cnt in this frame")
		print (cnt)

	checkPointReflash(process,oneEdge)
	# edge.append(oneEdge)
	# store_Edge()
	pass




if __name__ == "__main__":
	
	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 

	process = int(sys.argv[1])

	print ("process=",process)
			
	checkingPose()
	checkingEdge()

	make_testing_mesh(world)
				
	
	robot = world.robot(0)

	# vis.add("world",world)
	# vis.show()

	collisionTest = WorldCollider(world)
	
	robotPose = RobotPoser(robot)
	
	edgeCnt = checkingGrid(process)
	edgeCnt = edgeCnt + process * 1000
	while(edgeCnt <= len(edgeList)):
		edgeCnt = checkingGrid(process)
		if (edgeCnt == 1000):
			break
		edgeCnt = edgeCnt + process * 1000

		create_Edge(process,edgeCnt)




	while(1):
		time.sleep(0.1)
		# vis.shown()
		#pass

			#pass
			


