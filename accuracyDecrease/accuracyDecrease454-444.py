# -*- coding: utf-8 -*-
# @File Name: accuracyDecrease454-444.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\accuracyDecrease\accuracyDecrease454-444.py
# @Author: Ruige_Lee
# @Date:   2019-03-04 14:16:32
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-03-04 14:35:01
# @Email: 295054118@whut.edu.cn"




import sys
import time
import json



sourceFileLoc = './sp/4-5-4/100msx3/'
destFileLoc = './sp/4-4-4/100msx3/'


edgeSource = []
edgeDest = []

def load_edge(f):
	global edgeSource

	with open( sourceFileLoc +'edge512p'+str(f)+'.json','r') as edgeFile:
		data = edgeFile.read()
		edgeSource = json.loads(data)
		# print (edgeSource)


def store_Edge_512p8(f):
	global edgeDest

	with open( destFileLoc +'edge512p'+str(f)+'.json','w') as edgeFile:
		data = json.dumps(edgeDest)
		edgeFile.write(data)
	pass


def compress():
	global edgeSource
	global edgeDest

	
	for edge in edgeSource:
		compressEdge = []
		for i in range (0,128):
			for j in range (0,16):
				if (edge[32*i+j] == 1  or edge[32*i+16+j] == 1 ):
					compressEdge.append(1)
					# print (1)
				else:
					compressEdge.append(0)
					# print (0)
		edgeDest.append(compressEdge)
		# print (edgeDest)
	# print (edgeDest)
		


for f in range (0,8):
	load_edge(f)
	edgeDest = []
	compress()
	store_Edge_512p8(f)