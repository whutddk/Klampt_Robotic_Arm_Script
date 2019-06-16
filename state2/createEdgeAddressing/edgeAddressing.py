# @File Name: edgeAddressing.py
# @File Path: K:\work\MAS2\PRM_robotic_arm\Klampt_Robotic_Arm_Script\state2\createEdgeAddressing\edgeAddressing.py
# @Author: 29505
# @Date:   2019-06-07 18:41:28
# @Last Modified by:   29505
# @Last Modified time: 2019-06-16 22:54:06
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/

## 产生 X-11 Y-17 Z-13 个节点 （2431）
# 产生11*17*12 + 11*16*13 + 10*17*13 = 6742条edge
# edge 编号 (0-6741)
# 建立二维数组从pose映射到edge
# X方向edge 0 + 10 * 17 * 13
# Y方向edge 2210 + 11 * 16 * 13
# Z方向edge 4498 + 11 * 17 * 12

import sys
import time


with open('./edgeAddressing.txt','w') as edgeAddressFile:
	for z in range ( 0, 13 ):
		for y in range ( 0, 17 ):
			for x in range (0,10):

				poseIndex = z*187 + y*11 + x
				edgeAddressFile.write(str(poseIndex) + "," + str(poseIndex+1) + '\n')

	for z in range ( 0, 13 ):
		for y in range ( 0, 16 ):
			for x in range (0,11):

				poseIndex =  z*187 + y*11 + x
				edgeAddressFile.write(str(poseIndex) + "," + str(poseIndex+11) + '\n')

	for z in range ( 0, 12 ):
		for y in range ( 0, 17 ):
			for x in range (0,11):

				poseIndex =  z*187 + y*11 + x
				edgeAddressFile.write(str(poseIndex) + "," + str(poseIndex+187) + '\n')

