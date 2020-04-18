# -*- coding: utf-8 -*-
# @File Name: edgeAddressingCtable.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\state2\createEdgeAddressing\edgeAddressingCtable.py
# @Author: Ruige_Lee
# @Date:   2019-06-07 18:41:28
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-07-28 20:14:44
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


with open('./edgeAddressing.c','w') as edgeAddressFile:

	nowtime = time.localtime(time.time())

	strSyntax = '/*******************************************\n'
	strSyntax = strSyntax + '****** Wuhan University of Technology ******\n'
	strSyntax = strSyntax + '****** Ruige Lee ******\n'
	strSyntax = strSyntax + 'year: ' + str(nowtime.tm_year) + '\n'
	strSyntax = strSyntax + 'month: ' + str(nowtime.tm_mon) + '\n'
	strSyntax = strSyntax + 'date: ' + str(nowtime.tm_mday) + '\n'
	strSyntax = strSyntax + 'hour: ' + str(nowtime.tm_hour) + '\n'
	strSyntax = strSyntax + 'minutes: ' + str(nowtime.tm_min) + '\n'
	strSyntax = strSyntax + 'second: ' + str(nowtime.tm_sec) + '\n'
	strSyntax = strSyntax + '********************************************/\n\n'


	strSyntax = strSyntax + '#include <stdint.h> \nconst uint16_t lookUpedgeTable[8192][2] = {\n'
	edgeAddressFile.write(strSyntax)



	for z in range ( 0, 13 ):
		for y in range ( 0, 17 ):
			for x in range (0,10):

				poseIndex = z*187 + y*11 + x
				edgeAddressFile.write( "	{" + str(poseIndex) + "," + str(poseIndex+1) + '},\n')

	for z in range ( 0, 13 ):
		for y in range ( 0, 16 ):
			for x in range (0,11):

				poseIndex =  z*187 + y*11 + x
				edgeAddressFile.write( "	{" + str(poseIndex) + "," + str(poseIndex+11) + '},\n')

	for z in range ( 0, 12 ):
		for y in range ( 0, 17 ):
			for x in range (0,11):

				poseIndex =  z*187 + y*11 + x
				edgeAddressFile.write( "	{" + str(poseIndex) + "," + str(poseIndex+187) + '},\n')
	edgeAddressFile.write("};\n")
