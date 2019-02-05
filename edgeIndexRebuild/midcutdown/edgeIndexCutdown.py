
# @File Name: edgeIndexCutdown.py
# @File Path: K:\work\MAS2\Klampt_Robotic_Arm_Script\edgeIndexRebuild\midcutdown\edgeIndexCutdown.py
# @Author: 29505
# @Date:   2019-02-05 20:38:20
# @Last Modified by:   29505
# @Last Modified time: 2019-02-05 21:37:09
# @Email: 295054118@whut.edu.cn

import sys
import time

import json




jointList = []
edgeIndex = []

newJointList = []
newEdgeIndex = []


def load_data():
	global edgeIndex
	global jointList



	with open('./jointList.json','r') as jointListFile:
			
		data = jointListFile.read()
		jointList = json.loads(data)
	
	with open('./edgeIndex.json','r') as edgeIndexFile:
			
		data = edgeIndexFile.read()
		edgeIndex = json.loads(data)

	pass



def save_jointList():
	global newJointList
	with open('./newJointList.json','w') as newJointListFile:
		
		data = json.dumps(newJointList)
		newJointListFile.write(data)
	pass
	

def save_edgeIndex():
	global newEdgeIndex
	with open('./newEdgeIndex.json','w') as newEdgeIndexFile:
		
		data = json.dumps(newEdgeIndex)
		newEdgeIndexFile.write(data)
	pass


# def edgeIndexCutdown():
load_data()
for i in range (0,100000):
	newEdgeIndex.append(edgeIndex[i])
save_edgeIndex()

for i in range (0,edgeIndex[99999][1]):
	newJointList.append(jointList[i])
save_jointList()
pass







# dry run
# 打开jointList 打开edgeIndex 

# for edgeIndex until aim

#	resave edgeIndex
#	search maxJointList rebuild

