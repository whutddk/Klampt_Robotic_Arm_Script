# -*- coding: utf-8 -*-
# @File Name: createCheckPoint.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\state2\createSampleTable\createCheckPoint.py
# @Author: Ruige_Lee
# @Date:   2019-06-01 17:04:10
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2020-01-03 17:49:31
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/


## 产生 X-11 Y-17 Z-13 个节点 （2431）
## X:100~350 Y:-200~200 Z:100~400
# 编号 X(0-10) Y(0-16) Z(0-12)
# 结点编号 Z*187 + Y*11 + X

# 产生11*17*12 + 11*16*13 + 10*17*13 = 6742条edge
# edge 编号 (0-6741)
# 建立二维数组从pose映射到edge
# X方向edge 0 + 10 * 17 * 13
# Y方向edge 2210 + 11 * 16 * 13
# Z方向edge 4498 + 11 * 17 * 12




from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis
from klampt.model import ik

import random
from math import *








if __name__ == "__main__":
	world = WorldModel()

	res = world.readFile('../../dual_anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	prmRobot = world.robot(0)
	ctlRobot = world.robot(1)
	vis.add("world",world)
	vis.show()	
	prmRobotPose = RobotPoser(prmRobot)
	ctlRobotPose = RobotPoser(ctlRobot)


	## 产生 X-11 Y-17 Z-13 个节点 （2431）
	## X:100~350 Y:-200~200 Z:100~400
	# 编号 X(0-10) Y(0-16) Z(0-12)
	# 结点编号 Z*187 + Y*11 + X
	with open('./samplePose.c','w') as samplePoseC:
		with open('./samplePose.coe','w') as samplePose:
			cnt = 0
			while(cnt < 512):
				print(cnt)

				theta0 = (-1.57 + random.random() * 3.14); # -90 ~ +90
				theta1 = (-2.36 + random.random() * 3.14); # -135 ~ +135
				theta2 = (-0.7 + random.random() * 3.14); # -40 ~ +220
				theta3 = (-0.79 + random.random() * 1.57); # -90 ~ +90
				theta4 = (0 + random.random() * 3.14); # -90 ~ +90
				#samplePose5 = -12861 + rand() / 2147483647. * 25723; # -90 ~ +90

				prmRobotPose.set([0,theta0,theta1,theta2,theta3,theta4,0,0])

				ch = input("保留？")

				if ( ch == 'y'):
					cnt = cnt + 1

					samplePose0 = (int)(theta0 * 8192); # -90 ~ +90
					samplePose1 = (int)(theta1 * 8192); # -135 ~ +135
					samplePose2 = (int)(theta2 * 8192); # -40 ~ +220
					samplePose3 = (int)(theta3 * 8192); # -90 ~ +90
					samplePose4 = (int)(theta4 * 8192); # -90 ~ +90
					
					samplePoseC.write("{")
					samplePoseC.write((str)(samplePose0))
					samplePoseC.write(",")
					samplePoseC.write((str)(samplePose1))
					samplePoseC.write(",")
					samplePoseC.write((str)(samplePose2))
					samplePoseC.write(",")
					samplePoseC.write((str)(samplePose3))
					samplePoseC.write(",")
					samplePoseC.write((str)(samplePose4))
					samplePoseC.write("},\n")

					outPose4 = (hex)(samplePose4&0xffff)
					outPose3 = (hex)(samplePose3&0xffff)
					outPose2 = (hex)(samplePose2&0xffff)
					outPose1 = (hex)(samplePose1&0xffff)
					outPose0 = (hex)(samplePose0&0xffff)

					samplePose.write(outPose4[2:]+outPose3[2:]+outPose2[2:]+outPose1[2:]+outPose0[2:]+",")
				else:
					pass


			print ("finish")

