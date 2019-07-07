# -*- coding: utf-8 -*-
# @File Name: TrueTable.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\state2\createTrueTable\TrueTable.py
# @Author: Ruige_Lee
# @Date:   2019-07-07 11:34:44
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-07-07 15:19:40
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/

# 6742edge * 16384(5+5+4bit) grid
# 一个文件有16384个grid 和1000edge

import json
import sys



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
		

def createEmptyTrueTable():
	for grid in range(0,16384):
		oneGrid = []
		for grid in range (0,1000):
			oneGrid.append(0)
		TrueTable.append(oneGrid)


def createTrueTable(num):
	with open('./trueTable'+ str(num) +'.json','w') as trueTableFile:
		edgeCnt = 0
		for edge in collisionData:
			for gridCnt in range(0,16384):
				TrueTable[gridCnt][edgeCnt] = edge[gridCnt]
			print (edgeCnt)
			edgeCnt = edgeCnt + 1


		data = json.dumps(TrueTable)
		trueTableFile.write(data)

if __name__ == "__main__":
	process = int(sys.argv[1])
	load_CollideData(process )
	createEmptyTrueTable()
	createTrueTable(process )
