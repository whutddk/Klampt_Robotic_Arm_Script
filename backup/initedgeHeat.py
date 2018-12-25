
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

