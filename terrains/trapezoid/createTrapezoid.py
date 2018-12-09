import sys
import time
import math

PI = 3.14159

for a in range(0,64):
	for r in range (0,16):

		with open('./trapezoid'+ str(a)+"_"+ str(r) +'.off','w') as modelFile:
			radius = 0.16 + 0.022*r
			angle = 2.8125*a
			X1 = -radius * math.cos(angle/180*PI) 
			X2 = -(radius+0.022) * math.cos(angle/180*PI) 
			X3 = -radius * math.cos((angle+2.8125)/180*PI) 
			X4 = -(radius+0.022) * math.cos((angle+2.8125)/180*PI) 

			Y1 = -radius * math.sin(angle/180*PI) 
			Y2 = -(radius+0.022) * math.sin((angle)/180*PI) 
			Y3 = -radius * math.sin((angle+2.8125)/180*PI) 
			Y4 = -(radius+0.022) * math.sin((angle+2.8125)/180*PI)

			modelFile.write('OFF\n')
			modelFile.write('8 12 0\n')

			modelFile.write(str(X1) + ' ' +str(Y1)+ ' 0\n')
			modelFile.write(str(X1) + ' ' +str(Y1)+' 0.032\n')
			modelFile.write(str(X2) + ' ' +str(Y2)+' 0\n')
			modelFile.write(str(X2) + ' ' +str(Y2)+' 0.032\n')
			modelFile.write(str(X3) + ' ' +str(Y3)+' 0\n')
			modelFile.write(str(X3) + ' ' +str(Y3)+' 0.032\n')
			modelFile.write(str(X4) + ' ' +str(Y4)+' 0\n')
			modelFile.write(str(X4) + ' ' +str(Y4)+' 0.032\n')

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
