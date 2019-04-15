# @File Name: initedgeHeat.py
# @File Path: K:\work\MAS2\Klampt_Robotic_Arm_Script\backup\initedgeHeat.py
# @Author: whutddkUbuntu16
# @Date:   2018-12-26 19:58:19
# @Last Modified by:   29505
# @Last Modified time: 2019-02-06 18:28:51
# @Email: 295054118@whut.edu.cn

import sys
import time


import json

import random




def init_edgeHeat():


	with open('./edgeHeat.json','w') as edgeHeatFile:

		edgeHeat = []

		for i in range(0,100000):
			edgeHeat.append(0)


		data = json.dumps(edgeHeat)


		edgeHeatFile.write(data)
	pass




if __name__ == "__main__":

	init_edgeHeat()	

