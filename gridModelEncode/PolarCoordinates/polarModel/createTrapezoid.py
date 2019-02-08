# @File Name: createTrapezoid.py
# @File Path: K:\work\MAS2\Klampt_Robotic_Arm_Script\gridModelEncode\PolarCoordinates\polarModel\createTrapezoid.py
# @Author: 29505
# @Date:   2019-02-08 09:18:50
# @Last Modified by:   29505
# @Last Modified time: 2019-02-08 10:03:34
# @Email: 295054118@whut.edu.cn

import sys
import time
import math

PI = 3.14159


for x in range (0,16):
	for rad in range(0,32):	
		with open('./trapezoid'+ str(x) +"_" +str(rad) +'.off','w') as modelFile:
			radius = 0.120 + 0.023*x
			theta = -1 + 0.0625*rad


			X1_1 = radius * math.cos(theta) 
			Y1_1 = radius * math.sin(theta) 			
			X1_2 = radius * math.cos(theta+0.0625) 
			Y1_2 = radius * math.sin(theta+0.0625)


			X2_1 = (radius + 0.023) * math.cos(theta)
			Y2_1 = (radius + 0.023) * math.sin(theta)
			X2_2 = (radius + 0.023) * math.cos(theta+0.0625)
			Y2_2 = (radius + 0.023) * math.sin(theta+0.0625))


			modelFile.write('OFF\n')
			modelFile.write('8 12 0\n')

			modelFile.write(str(X1_1) + ' ' +str(Y1_1)+ ' '+ '0' +'\n')
			modelFile.write(str(X1_2) + ' ' +str(Y1_2)+ ' '+ '0' +'\n')
			modelFile.write(str(X2_1) + ' ' +str(Y2_1)+ ' '+ '0' +'\n')
			modelFile.write(str(X2_2) + ' ' +str(Y2_2)+ ' '+ '0' +'\n')
			modelFile.write(str(X1_1) + ' ' +str(Y1_1)+ ' '+ '0.020' +'\n')
			modelFile.write(str(X1_2) + ' ' +str(Y1_2)+ ' '+ '0.020' +'\n')
			modelFile.write(str(X2_1) + ' ' +str(Y2_1)+ ' '+ '0.020' +'\n')
			modelFile.write(str(X2_2) + ' ' +str(Y2_2)+ ' '+ '0.020' +'\n')

			modelFile.write('3 0 1 3\n')
			modelFile.write('3 0 3 2\n')
			modelFile.write('3 4 6 7\n')
			modelFile.write('3 4 7 5\n')
			modelFile.write('3 0 4 5\n')
			modelFile.write('3 0 5 1\n')
			modelFile.write('3 2 3 7\n')
			modelFile.write('3 2 7 6\n')
			modelFile.write('3 0 2 6\n')
			modelFile.write('3 0 6 4\n')
			modelFile.write('3 1 5 7\n')
			modelFile.write('3 1 7 3')







	pass
