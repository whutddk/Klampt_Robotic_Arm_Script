# -*- coding: utf-8 -*-
# @File Name: separateJson.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\separateJson\separateJson.py
# @Author: Ruige_Lee
# @Date:   2019-02-18 11:32:21
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-03-03 15:53:08
# @Email: 295054118@whut.edu.cn"

# @File Name: separateJson.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\separateJson\separateJson.py
# @Author: 29505
# @Date:   2019-02-12 16:34:22
# @Last Modified by:   29505
# @Last Modified time: 2019-02-12 16:55:42
# @Email: 295054118@whut.edu.cn

import sys
import time
import json
import random

edge = []
edge512px = [[],[],[],[],[],[],[],[]]

fileLoc = 'F:/Klampt/100msx3grid-po/'

def load_edge():
	global edge

	with open( fileLoc +'edge.json','r') as edgeFile:
		data = edgeFile.read()
		edge = json.loads(data)
		#print edge


def store_Edge_512p8():
	global edge512px

	with open(fileLoc +'edge512p0.json','w') as edgeFile:
		data = json.dumps(edge512px[0])
		edgeFile.write(data)
	pass

	with open(fileLoc +'edge512p1.json','w') as edgeFile:
		data = json.dumps(edge512px[1])
		edgeFile.write(data)
	pass

	with open(fileLoc +'edge512p2.json','w') as edgeFile:
		data = json.dumps(edge512px[2])
		edgeFile.write(data)
	pass
		
	with open(fileLoc +'edge512p3.json','w') as edgeFile:
		data = json.dumps(edge512px[3])
		edgeFile.write(data)
	pass
		
	with open(fileLoc +'edge512p4.json','w') as edgeFile:
		data = json.dumps(edge512px[4])
		edgeFile.write(data)
	pass
		
	with open(fileLoc +'edge512p5.json','w') as edgeFile:
		data = json.dumps(edge512px[5])
		edgeFile.write(data)
	pass
		
	with open(fileLoc +'edge512p6.json','w') as edgeFile:
		data = json.dumps(edge512px[6])
		edgeFile.write(data)
	pass

	with open(fileLoc +'edge512p7.json','w') as edgeFile:
		data = json.dumps(edge512px[7])
		edgeFile.write(data)
	pass


load_edge()
print (len(edge))
for i in range (0,8):
	for j in range (i*512,(i+1)*512):
	 	edge512px[i].append(edge[j])
store_Edge_512p8()
