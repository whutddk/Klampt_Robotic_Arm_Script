# -*- coding: utf-8 -*-
# @File Name: offlineCollisionCheck.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\state2\offLineCollisionChecking\offlineCollisionCheck.py
# @Author: Ruige_Lee
# @Date:   2019-06-18 19:35:33
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-06-20 19:58:13
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



checkingGrid()


