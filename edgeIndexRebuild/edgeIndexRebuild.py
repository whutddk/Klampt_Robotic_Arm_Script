
# @File Name: edgeIndexRebuild.py
# @File Path: K:\work\MAS2\Klampt_Robotic_Arm_Script\edgeIndexRebuild\edgeIndexRebuild.py
# @Author: 29505
# @Date:   2019-02-05 20:38:20
# @Last Modified by:   29505
# @Last Modified time: 2019-02-06 10:06:57
# @Email: 295054118@whut.edu.cn




import sys
import time

import json


import sys
import time

import json




jointList = []
edgeIndex = []
edgeHeat = []

newJointList = []
newEdgeIndex = []
newEdgeHeat = []

def load_data():
	global edgeIndex
	global jointList
	global edgeHeat

	with open('./edgeHeat.json','r') as edgeHeatFile:
		data = edgeHeatFile.read()
		edgeHeat = json.loads(data)


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


# def heat_rebuild():
load_data()

while(1):
	maxHeat = 0
	heatNum = 0
	for heat in range(0,100000):
		if (edgeHeat[heat] > maxHeat):
			maxHeat = edgeHeat[heat]
			heatNum = heat
	if ( maxHeat == 0 ):
		break
	else:
		newEdgeIndex.append(edgeIndex[heatNum])
		edgeHeat[heatNum] = -1
		print (maxHeat)

heat = 0
while (len(newEdgeIndex) < 1024 ):
	if (edgeHeat[heat] == 0 ):
		newEdgeIndex.append(edgeIndex[heat])
	heat = heat +1 

	pass


print (newEdgeIndex)
print (len(newEdgeIndex)) 

pass



# heat run
# 打开jointList 打开edgeIndex 打开heat文件

# heat 生成二维，对一个维度进行排序
# 筛选出新的edgeIndex
# 查找joint，重新生成edge序号



