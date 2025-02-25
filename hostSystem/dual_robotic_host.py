# -*- coding: utf-8 -*-
# @File Name: dual_robotic_host.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\hostSystem\dual_robotic_host.py
# @Author: Ruige_Lee
# @Date:   2019-04-22 17:09:53
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2020-01-09 16:26:21
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/
# @File Name: dual_robotic_host.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\hostSystem\dual_robotic_host.py
# @Author: 29505
# @Date:   2019-04-18 16:53:15
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-04-22 15:55:48
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
import binascii

_PI_ = 3.14159 
_STEPNUM_ = 25


class freeKlampt():
	def __init__(self):
		self.recBuf = []
		self.recChk = 1
		self.axisA = [0,0,0,0,0,0]
		self.axisB = [0,0,0,0,0,0]
		self.data = [0,0,0,0]
		self.data_pre = 0

		self.ser = serial.Serial("COM5")
		# self.ser = serial.Serial("COM4")
		# self.ser = serial.Serial("COM11")
		self.ser.baudrate = 115200
		self.ser.close()
		self.ser.open()

		self.rob1 = [0]
		self.rob2 = [0]

	def send_order1(self):
		# print (bytes([255]))
		self.ser.write(bytes([255]))
		self.ser.write(bytes([85]))
		self.ser.write(bytes([170]))
		self.ser.write(bytes([119]))
		self.ser.write(bytes([0]))
		self.ser.write(bytes([9]))
		self.ser.write(bytes([126]))

	def send_order2(self):
		# print (bytes([255]))
		self.ser.write(bytes([255]))
		self.ser.write(bytes([85]))
		self.ser.write(bytes([170]))
		self.ser.write(bytes([119]))
		self.ser.write(bytes([0]))
		self.ser.write(bytes([10]))
		self.ser.write(bytes([127]))


	def send_powerUp(self):
		# print (bytes([255]))
		self.ser.write(bytes([255]))
		self.ser.write(bytes([85]))
		self.ser.write(bytes([170]))
		self.ser.write(bytes([119]))
		self.ser.write(bytes([0]))
		self.ser.write(bytes([11]))
		self.ser.write(bytes([128]))


	def send_powerDown(self):
		self.ser.write(bytes([255]))
		self.ser.write(bytes([85]))
		self.ser.write(bytes([170]))
		self.ser.write(bytes([119]))
		self.ser.write(bytes([0]))
		self.ser.write(bytes([12]))
		self.ser.write(bytes([129]))

	def send_enable(self):
		self.ser.write(bytes([255]))
		self.ser.write(bytes([85]))
		self.ser.write(bytes([170]))
		self.ser.write(bytes([119]))
		self.ser.write(bytes([0]))
		self.ser.write(bytes([101]))
		self.ser.write(bytes([218]))
		# print(self.axisA)

	def send_disable(self):
		self.ser.write(bytes([255]))
		self.ser.write(bytes([85]))
		self.ser.write(bytes([170]))
		self.ser.write(bytes([119]))
		self.ser.write(bytes([0]))
		self.ser.write(bytes([104]))
		self.ser.write(bytes([221]))

	def get_robotPose(self):
		if (self.ser.in_waiting != 0):

			self.recBuf.append(    ord(self.ser.read())  )
			
			bufLen = len(self.recBuf)
			if ( bufLen >= 36 
				and self.recBuf[bufLen - 36] == 251
				and self.recBuf[bufLen - 35] == 109
				and self.recBuf[bufLen - 34] == 37):

				# print ( self.recBuf )

				dataOffset = bufLen - 33

				self.recChk = 397

				for i in range (0,6):
					self.recChk = self.recChk + self.recBuf[dataOffset + 2*i] + self.recBuf[dataOffset + 2*i + 1]

				for i in range (0,6):
					self.recChk = self.recChk + self.recBuf[dataOffset+12 + 2*i] + self.recBuf[dataOffset+12 + 2*i + 1]

				for i in range (0,4):
					self.recChk = self.recChk + self.recBuf[dataOffset+24 + 2*i] + self.recBuf[dataOffset+24 + 2*i + 1]


				if ( self.recChk%256 == self.recBuf[bufLen-1] ):
					for i in range (0,6):
						self.axisA[i] = (self.recBuf[dataOffset + 2*i]*256 + self.recBuf[dataOffset + 2*i + 1])
						if ( self.axisA[i] > 32767 ):
							self.axisA[i] = self.axisA[i] - 65536

						self.axisA[i] = self.axisA[i]/ 5000
					# print(self.axisA)

					for i in range (0,6):
						self.axisB[i] = (self.recBuf[dataOffset+12 + 2*i]*256 + self.recBuf[dataOffset+12 + 2*i + 1])
						if ( self.axisB[i] > 32767 ):
							self.axisB[i] = self.axisB[i] - 65536

						self.axisB[i] = self.axisB[i] / 5000
					# print(self.axisB)
					
					for i in range (0,4):
						self.data[i] = (self.recBuf[dataOffset+24 + 2*i]*256 + self.recBuf[dataOffset+24 + 2*i + 1])
						if ( self.data[i] > 32767 ):
							self.data[i] = self.data[i] - 65536

					if ( self.data_pre == self.data[0] ):
						pass
					else:
						print(self.axisA)
						print(self.axisB)
						print(self.data[0],self.data[1],self.data[2],self.data[3])	
						self.data_pre = self.data[0]	
					
				
				self.recBuf = []

			elif ( bufLen >= 80 ):
				self.recBuf = []






class KepBoardCapture(GLPluginInterface,freeKlampt):

	def __init__(self,world):

		GLPluginInterface.__init__(self)

		self.world = world

	def keyboardfunc(self,c, x, y):

		if ( c == 'f9' ):
			freeKlampt.send_order1()

		if ( c == 'f10' ):
			freeKlampt.send_order2()


		if ( c == "f11" ):
			freeKlampt.send_powerUp()
			# print ("done")

		elif ( c == "f12" ):
			freeKlampt.send_powerDown()
			# print ("done")

		elif ( c == "home" ):
			freeKlampt.send_enable()
			# print ("done")

		elif ( c == "end" ):
			freeKlampt.send_disable()
			# print ("done")


def make_obstacle(world):
	"""automatically create a mesh test grid
	"""
	grid = Geometry3D()

	grid.loadFile("../terrains/cube.off")

	grid.transform([0.075,0,0,  0,0.100,0,  0,0,0.25],[0.350,-0.125,0.1])			

	Mesh = world.makeTerrain("Grid")

	Mesh.geometry().set(grid)
	Mesh.appearance().setColor(0.5,0.1,0.1,0.8)
	return 





if __name__ == "__main__":

	world = WorldModel()

	res = world.readFile('../dual_anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

	
	# make_obstacle(world)

	freeKlampt = freeKlampt()


	prmRobot = world.robot(0)
	ctlRobot = world.robot(1)


	plugin = KepBoardCapture(world)
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
		prmRobotPose.set([0,freeKlampt.axisA[0],freeKlampt.axisA[1],freeKlampt.axisA[2],freeKlampt.axisA[3],freeKlampt.axisA[4],freeKlampt.axisA[5],0])
		ctlRobotPose.set([0,freeKlampt.axisB[0],freeKlampt.axisB[1],freeKlampt.axisB[2],freeKlampt.axisB[3],freeKlampt.axisB[4],freeKlampt.axisB[5],0])
