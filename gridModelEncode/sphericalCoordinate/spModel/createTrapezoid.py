# @File Name: createTrapezoid.py
# @File Path: K:\work\MAS2\Klampt_Robotic_Arm_Script\gridModelEncode\sphericalCoordinate\spModel\createTrapezoid.py
# @Author: 29505
# @Date:   2018-12-04 11:25:50
# @Last Modified by:   29505
# @Last Modified time: 2019-02-09 10:36:19
# @Email: 295054118@whut.edu.cn

import sys
import time
import math

PI = 3.14159


for x in range (0,16):
	for rad in range(0,32):
		for fi in range (0,32):
			with open('./trapezoid'+ str(x)+ '_'+ str(rad) +"_" +str(fi) +'.off','w') as modelFile:
				radius = 0.120 + 0.023*x
				theta = -1.3197 + 0.0825*rad
				fieta = -1.1442 + 0.075*fi

				X1_1 = radius * math.cos(theta) * math.sin(fieta)
				Y1_1 = radius * math.sin(theta) * math.sin(fieta)
				Z1_1 = radius * math.cos(fieta)+0.264

				X1_2 = radius * math.cos(theta) * math.sin(fieta+0.075)
				Y1_2 = radius * math.sin(theta) * math.sin(fieta+0.075)
				Z1_2 = radius * math.cos((fieta+0.075) ) + 0.264



				X2_1 = (radius + 0.023) * math.cos(theta) * math.sin(fieta)
				Y2_1 = (radius + 0.023) * math.sin(theta) * math.sin(fieta)
				Z2_1 = (radius + 0.023) * math.cos(fieta) + 0.264

				X2_2 = (radius + 0.023) * math.cos(theta) * math.sin(fieta + 0.075)
				Y2_2 = (radius + 0.023) * math.sin(theta) * math.sin(fieta + 0.075)
				Z2_2 = (radius + 0.023) * math.cos(fieta+0.075)+0.264



				X3_1 = radius * math.cos(theta + 0.0825) * math.sin(fieta)
				Y3_1 = radius * math.sin(theta + 0.0825) * math.sin(fieta)
				Z3_1 = radius * math.cos(fieta) + 0.264

				X3_2 = radius * math.cos(theta+0.0825) * math.sin( fieta + 0.075 )
				Y3_2 = radius * math.sin(theta+0.0825) * math.sin( fieta + 0.075 )
				Z3_2 = radius * math.cos(fieta+0.075) + 0.264



				X4_1 = (radius + 0.023) * math.cos(theta+0.0825) * math.sin(fieta )
				Y4_1 = (radius + 0.023) * math.sin(theta+0.0825) * math.sin(fieta )
				Z4_1 = (radius + 0.023) * math.cos(fieta)+0.264

				X4_2 = (radius + 0.023) * math.cos(theta+0.0825) * math.sin(fieta + 0.075)
				Y4_2 = (radius + 0.023) * math.sin(theta+0.0825) * math.sin(fieta + 0.075)
				Z4_2 = (radius + 0.023) * math.cos(fieta+0.075) + 0.264


				modelFile.write('OFF\n')
				modelFile.write('8 12 0\n')

				modelFile.write(str(X1_1) + ' ' +str(Y1_1)+ ' '+ str(Z1_1)  +'\n')
				modelFile.write(str(X1_2) + ' ' +str(Y1_2)+ ' '+ str(Z1_2)  +'\n')
				modelFile.write(str(X2_1) + ' ' +str(Y2_1)+ ' '+ str(Z2_1)  +'\n')
				modelFile.write(str(X2_2) + ' ' +str(Y2_2)+ ' '+ str(Z2_2)  +'\n')
				modelFile.write(str(X3_1) + ' ' +str(Y3_1)+ ' '+ str(Z3_1)  +'\n')
				modelFile.write(str(X3_2) + ' ' +str(Y3_2)+ ' '+ str(Z3_2)  +'\n')
				modelFile.write(str(X4_1) + ' ' +str(Y4_1)+ ' '+ str(Z4_1)  +'\n')
				modelFile.write(str(X4_2) + ' ' +str(Y4_2)+ ' '+ str(Z4_2)  +'\n')

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
