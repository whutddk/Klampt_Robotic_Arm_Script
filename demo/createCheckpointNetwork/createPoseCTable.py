# @File Name: createPoseCTable.py
# @File Path: K:\work\MAS2\PRM_robotic_arm\Klampt_Robotic_Arm_Script\state2\createCheckpointNetwork\createPoseCTable.py
# @Author: 29505
# @Date:   2019-06-07 18:10:31
# @Last Modified by:   29505
# @Last Modified time: 2019-06-07 18:38:29
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/

import sys
import time

FileData = "#include \"stdint.h\"\n\n" + "float poseTable[2431][6] = {\n" 

with open('./poseTable.txt','r') as poseTableFile:
	for line in poseTableFile.readlines():
		line = line[0:-1]
		FileData = FileData +  "	{"
		FileData = FileData + line
		FileData = FileData + "},\n"
	pass

FileData = FileData[:-2] + "\n};\n"
with open('./poseTable.c','w') as CTableFile:
	CTableFile.write(FileData)
