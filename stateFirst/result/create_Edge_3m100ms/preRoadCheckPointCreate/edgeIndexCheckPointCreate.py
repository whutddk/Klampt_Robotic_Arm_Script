# -*- coding: utf-8 -*-
# @File Name: edgeIndexCheckPointCreate.py
# @File Path: M:\MAS2\Robotic Arm\Klampt_Robotic_Arm_Script\result\create_Edge_3m50ms\preRoadCheckPointCreate\edgeIndexCheckPointCreate.py
# @Author: Ruige_Lee
# @Date:   2019-01-26 20:00:57
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-01-26 20:44:15
# @Email: 295054118@whut.edu.cn"

import sys
import time

import json

import random


edgeIndex = []



for i in range (1,9):
	edgeIndex.append( [0,9 +(i-1)*31] )
	for j in range ( 9 +(i-1)*31,9 +(i)*31 ):
		edgeIndex.append( [j,j+1 ])
	edgeIndex.append( [8 +(i)*31,i])




with open('./edgeIndex.json','w') as edgeIndexFile:
	
	data = json.dumps(edgeIndex)
	edgeIndexFile.write(data)
pass
