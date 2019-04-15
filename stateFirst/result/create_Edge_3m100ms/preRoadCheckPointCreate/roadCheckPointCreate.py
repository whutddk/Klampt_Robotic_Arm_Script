# -*- coding: utf-8 -*-
# @File Name: roadCheckPointCreate.py
# @File Path: M:\MAS2\Robotic Arm\Klampt_Robotic_Arm_Script\result\create_Edge_3m50ms\preRoadCheckPointCreate\roadCheckPointCreate.py
# @Author: Ruige_Lee
# @Date:   2019-01-26 20:00:57
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-01-26 20:30:54
# @Email: 295054118@whut.edu.cn"

import sys
import time

import json

import random

p = [[0.01123, -1.32404, -0.41556, -0.00164, 1.73955, 0],[0.56577, -1.35028, 0.06264, -3.141026, 1.85242, 0.56],[0.00871, -1.31410, -0.15823, -0.01047, 1.47401, 0],[-0.55153, -1.35044, 0.063146, -0.00185, 1.28761, -0.55],[0.42077, -1.55091, 0.682296, -3.14079, 2.27198, 0.42],[0.00587, -1.45458, 0.422789, 0, 1.03311, 0],[-0.41034, -1.55140, 0.68351, -3.139590, 2.27294, -0.41],[1.4, -1.45458, 0.42279, 0, 1.03311, 0],[-1.4, -1.45458, 0.42279, 0, 1.03311, 0]]

jointList = []



for i in range (0,9):
	jointList.append(p[i])

for i in range (1,9):
	step = [ (p[i][0] - p[0][0])/32,(p[0][1] - p[i][1])/32,(p[i][2] - p[0][2])/32,(p[i][3] - p[0][3])/32,(p[i][4] - p[0][4])/32,(p[i][5] - p[0][5])/32 ]
	for j in range ( 1,32 ):
		jointList.append( [ (p[0][0] + step[0]*j),(p[0][1] + step[1]*j),(p[0][2] + step[2]*j),(p[0][3] + step[3]*j),(p[0][4] + step[4]*j),(p[0][5] + step[5]*j)])

		


with open('./jointList.json','w') as jointListFile:
	
	data = json.dumps(jointList)
	jointListFile.write(data)
pass