# -*- coding: utf-8 -*-
# @File Name: TrueTable.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\state2\createTrueTable\TrueTable.py
# @Author: Ruige_Lee
# @Date:   2019-07-07 11:34:44
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-07-07 14:52:53
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/

# 6742edge * 16384(5+5+4bit) grid
# 一个文件有16384个grid 和1000edge

collisionData = []
TrueTable = []

def load_CollideData(num):
	with open('../offLineCollisionChecking/gridEncode'+ str(num) +'.txt','r') as collideDataFile:
		for line in collideDataFile.readlines():
			oneEdge = []
			for char in line:
				if char == ',':
					continue
				if char == '0':
					oneEdge.append(0)
				if char == '1':
					oneEdge.append(1)
			collisionData.append(oneEdge)
			print (oneEdge)

def createEmptyTrueTable(num):
	for edge in range(0,1000):
		oneEdge = []
		for grid in range (0,16384):
			oneEdge.append(0)
		TrueTable.append(oneEdge)


def createTrueTable(num):
	with open('./trueTable'+ str(num) +'.json','w') as trueTableFile:
		edgeCnt = 0
		for edge in collisionData:
			for gridCnt in range(0,16384):
				TrueTable[gridCnt][edgeCnt] = edge[gridCnt]
			edgeCnt = edgeCnt + 1


load_CollideData(0)


