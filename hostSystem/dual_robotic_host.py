# -*- coding: utf-8 -*-
# @File Name: dual_robotic_host.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Klampt_Robotic_Arm_Script\hostSystem\dual_robotic_host.py
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





_PI_ = 3.14159 
_STEPNUM_ = 25



class freecars():
	def __init__(self):
		self.recBuf = []
		self.recChk = 0
		self.axisA = [0,0,0,0,0,0]
		self.axisB = [0,0,0,0,0,0]

		self.ser = serial.Serial("COM1")
		self.ser.baudrate = 9600
		self.ser.open()

		self.rob1 = [0]
		self.rob2 = [0]

	def send_powerUp(self):
		self.ser.write()

	def send_powerDown(self):
		self.ser.write()	

	def send_enable(self):
		self.ser.write()

	def send_disable(self):
		self.ser.write()

	def get_robotPose(self):
		if (self.ser.in_waiting() != 0):
			self.recBuf.append(ser.read())

			if ( len(self.recBuf) > 3 and len(self.recBuf) < 28 ):
				self.recChk = self.recChk + self.recBuf[len(self.recBuf)-1]%2


			elif ( len(self.recBuf) == 1 ):
				if ( self.recBuf[0] == 251 ):
					pass
				else:
					self.recBuf = []
					self.recChk = 0

			elif ( len(self.recBuf) == 2 ):
				if ( self.recBuf[1] == 109 ):
					pass
				else:
					self.recBuf = []
					self.recChk = 0

			elif ( len(self.recBuf) == 3 ):
				if ( self.recBuf[2] == 37 ):
					pass
				else:
					self.recBuf = []
					self.recChk = 0

			elif ( len(self.recBuf) == 28 ):
				if ( self.recBuf[27]%2 == self.recChk%2 ):

					for i in range (0,6):
						axisA[i] = self.recBuf[3+2*i]*256 + self.recBuf[3+2*i+1]

					for i in range (0,6):
						axisB[i] = self.recBuf[15+2*i]*256 + self.recBuf[15+2*i+1]

					pass
				
				self.recBuf = []
				self.recChk = 0




class KepBoardCapture(GLPluginInterface):

	def __init__(self,world):

		GLPluginInterface.__init__(self)

		self.world = world

	def keyboardfunc(self,c, x, y):

		pass




if __name__ == "__main__":

	world = WorldModel()

	res = world.readFile('../dual_anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
	del res

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
		time.sleep(0.1)
		pass

