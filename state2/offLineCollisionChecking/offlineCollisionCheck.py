# -*- coding: utf-8 -*-
# @File Name: offlineCollisionCheck.py
# @File Path: K:\work\MAS2\PRM_robotic_arm\Klampt_Robotic_Arm_Script\state2\offLineCollisionChecking\offlineCollisionCheck.py
# @Author: Ruige_Lee
# @Date:   2019-06-18 19:35:33
# @Last Modified by:   29505
# @Last Modified time: 2019-06-21 20:53:10
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/


poseList = []
edgeList = []
gridCnt = 0

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

def checkingGrid():

	global gridCnt
	with open('./gridEncode.txt','r') as gridEncodeFile:
		for line in gridEncodeFile.readlines():
			if line[0] == "{" and line[-1] == "}":
				gridCnt = gridCnt + 1
		print (gridCnt)
	pass


def checkPointReflash(data):
	with open('./gridEncode.txt','a') as gridEncodeFile:
		gridEncodeFile.write(data)
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

				grid.transform([0.023,0,0,  0,0.030,0,  0,0,0.020],[0.023*x + 0.120,0.030*y-0.480,0.020*z])			

				Mesh = world.makeTerrain("Grid," + "%3d"%x + "," + "%3d"%y + "," + "%3d"%z)

				Mesh.geometry().set(grid)
				Mesh.appearance().setColor(0.1,0.1,0.2,0.1)
	return 


def create_Edge(Index):

	global Pose
	global edgeIndex
	global edge

	print ("now Create Edge:")
	print (Index)

	i = edgeIndex[Index][0]
	j = edgeIndex[Index][1]

	shoulderStart = Pose[j][0]
	armStart = Pose[j][1]
	elbowStart = Pose[j][2]
	wristStart = Pose[j][3]
	fingerStart = Pose[j][4]
	toolStart = Pose[j][5]

	shoulderEnd = Pose[i][0]
	armEnd = Pose[i][1]
	elbowEnd = Pose[i][2]
	wristEnd = Pose[i][3]
	fingerEnd = Pose[i][4]
	toolEnd = Pose[i][5]

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
			oneEdge[1024*x+32*y+z] = 1
			cnt = cnt + 1;
			# print (x)
			# print (y)
			# print (z)
		print ("cnt in this frame")
		print (cnt)

	edge.append(oneEdge)
	store_Edge()
	pass




if __name__ == "__main__":
	
	world = WorldModel()

	res = world.readFile('../../anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			
	load_Pose()
	load_Index()
	load_edge()

	make_testing_mesh(world)
				
	
	#sim = Simulator(world)
	robot = world.robot(0)

	#vis.add("world",world)
	#vis.show()

	collisionTest = WorldCollider(world)
	
	robotPose = RobotPoser(robot)
	
	#print robotPose.get()
	while(len(edge) < 4096):
		create_Edge(len(edge))




	#while(1):
		#time.sleep(0.1)
		#vis.shown()
		#pass

			#pass
			


