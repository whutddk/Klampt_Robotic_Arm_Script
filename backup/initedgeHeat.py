# @File Name: initedgeHeat.py
# @File Path: /home/whutddk/Klampt/klampt_robotic_arm_script/backup/initedgeHeat.py
# @Author: whutddkUbuntu16
# @Date:   2018-12-26 19:58:19
# @Last Modified by:   whutddkUbuntu16
# @Last Modified time: 2019-01-17 20:49:58
# @Email: 295054118@whut.edu.cn

import sys
import time


import json

import random




def init_edgeHeat():


	with open('./edgeHeat.json','w') as edgeHeatFile:

		edgeHeat = []

		for i in range(0,1000000):
			edgeHeat.append(0)


		data = json.dumps(edgeHeat)


		edgeHeatFile.write(data)
	pass




if __name__ == "__main__":

	init_edgeHeat()	

