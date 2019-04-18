# @File Name: dual_robotic_host.py
# @File Path: K:\work\MAS2\PRM_robotic_arm\Klampt_Robotic_Arm_Script\hostSystem\dual_robotic_host.py
# @Author: 29505
# @Date:   2019-04-18 16:53:15
# @Last Modified by:   29505
# @Last Modified time: 2019-04-18 20:06:08
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/
# -*- coding: utf-8 -*-
# @File Name: dual_robotic_host.py
# @File Path: K:\work\MAS2\PRM_robotic_arm\Klampt_Robotic_Arm_Script\hostSystem\dual_robotic_host.py
# @Author: Ruige_Lee
# @Date:   2019-04-17 18:54:16
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-04-17 21:11:40
# @Email: 295054118@whut.edu.cn"


from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt import vis
from klampt.vis.glinterface import GLPluginInterface


import serial


_PI_ = 3.14159 
_STEPNUM_ = 25



class freeKlampt():
	def __init__(self):
		self.recBuf = []
		self.recChk = 1
		self.axisA = [0,0,0,0,0,0]
		self.axisB = [0,0,0,0,0,0]

		self.ser = serial.Serial("COM1")
		self.ser.baudrate = 9600
		self.ser.open()

		self.rob1 = [0]
		self.rob2 = [0]

	def send_powerUp(self):
		self.ser.write(255)
		self.ser.write(85)
		self.ser.write(170)
		self.ser.write(119)
		self.ser.write(0)
		self.ser.write(11)
		self.ser.write(128)


	def send_powerDown(self):
		self.ser.write(255)
		self.ser.write(85)
		self.ser.write(170)
		self.ser.write(119)
		self.ser.write(0)
		self.ser.write(12)
		self.ser.write(129)

	def send_enable(self):
		self.ser.write(255)
		self.ser.write(85)
		self.ser.write(170)
		self.ser.write(119)
		self.ser.write(0)
		self.ser.write(101)
		self.ser.write(218)

	def send_disable(self):
		self.ser.write(255)
		self.ser.write(85)
		self.ser.write(170)
		self.ser.write(119)
		self.ser.write(0)
		self.ser.write(104)
		self.ser.write(221)

	def get_robotPose(self):
		if (self.ser.in_waiting() != 0):
			self.recBuf.append(ser.read())

			bufLen = len(self.recBuf)
			if ( bufLen >= 28 
				and self.recBuf[bufLen - 28] == 251
				and self.recBuf[bufLen - 27] == 109
				and self.recBuf[bufLen - 26] == 37):

				dataOffset = bufLen - 25

				self.recChk = 397

				for i in range (0,6):
					self.recChk = self.recChk + self.recBuf[dataOffset + 2*i] + self.recBuf[dataOffset + 2*i + 1]

				for i in range (0,6):
					self.recChk = self.recChk + self.recBuf[dataOffset+12 + 2*i] + self.recBuf[dataOffset+12 + 2*i + 1]


				if ( self.recChk == self.recBuf[bufLen-1] ):
					for i in range (0,6):
						axisA[i] = self.recBuf[dataOffset + 2*i]*256 + self.recBuf[dataOffset + 2*i + 1]


					for i in range (0,6):
						axisB[i] = self.recBuf[dataOffset+12 + 2*i]*256 + self.recBuf[dataOffset+12 + 2*i + 1]

				
				self.recBuf = []





class KepBoardCapture(GLPluginInterface,freeKlampt):

	def __init__(self,world):

		GLPluginInterface.__init__(self)

		self.world = world

	def keyboardfunc(self,c, x, y):

		if ( c == "F11" ):
			freecar.send_powerUp()

		elif ( c == "F12" ):
			freecar.send_powerDown()

		elif ( c == "home" ):
			freecar.send_enable()

		elif ( c == "end" ):
			freecar.send_disable()


if __name__ == "__main__":

	world = WorldModel()

	res = world.readFile('../dual_anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res


	freeKlampt = freeKlampt()


	prmRobot = world.robot(0)
	ctlRobot = world.robot(1)


	plugin = KepBoardCapture(world,freeKlampt)
	vis.pushPlugin(plugin)

	vis.add("world",world)
	vis.show()
	

	# collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)
	ctlRobotPose = RobotPoser(ctlRobot)


	prmRobotPose.set([0,0,-1.57,1.57,0,1.57,0,0])
	ctlRobotPose.set([0,0,0,0,0,0,0,0])

	while(1):
		freeKlampt.get_robotPose()
		prmRobotPose.set([0,axisA[0],axisA[1],axisA[2],axisA[3],axisA[4],axisA[5],0])
		ctlRobotPose.set([0,axisB[0],axisB[1],axisB[2],axisB[3],axisB[4],axisB[5],0])
