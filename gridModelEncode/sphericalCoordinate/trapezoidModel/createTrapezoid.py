import sys
import time
import math

PI = 3.14159


for r in range (0,16):
	for fi in range (0,32):
		for thr in range(0,32):
			with open('./trapezoid'+ str(r)+ '_'+ str(fi) +"_" +str(thr) +'.off','w') as modelFile:
				radius = 0.132 + 0.031*r
				theta = 5.625*thr
				fieta = 4.0625*fi

				X1_1 = -radius * math.cos(theta/180*PI) * math.sin(fieta /180*PI)
				Y1_1 = -radius * math.sin(theta/180*PI) * math.sin(fieta /180*PI)
				Z1_1 = radius * math.cos(fieta/180*PI)+0.132
				X1_2 = -radius * math.cos(theta/180*PI) * math.sin((fieta+4.0625) /180*PI)
				Y1_2 = -radius * math.sin(theta/180*PI) * math.sin((fieta+4.0625) /180*PI)
				Z1_2 = radius * math.cos((fieta+4.0625) /180*PI)+0.132

				X2_1 = -(radius + 0.031) * math.cos(theta/180*PI) * math.sin(fieta /180*PI)
				Y2_1 = -(radius + 0.031) * math.sin((theta)/180*PI) * math.sin(fieta /180*PI)
				Z2_1 = (radius+ 0.031) * math.cos(fieta /180*PI)+0.132
				X2_2 = -(radius + 0.031) * math.cos(theta/180*PI) * math.sin((fieta+4.0625) /180*PI)
				Y2_2 = -(radius + 0.031) * math.sin((theta)/180*PI) * math.sin((fieta+4.0625) /180*PI)
				Z2_2 = (radius+ 0.031) * math.cos((fieta+4.0625) /180*PI)+0.132

				X3_1 = -radius * math.cos((theta+5.625)/180*PI) * math.sin(fieta /180*PI)
				Y3_1 = -radius * math.sin((theta+5.625)/180*PI) * math.sin(fieta /180*PI)
				Z3_1 = radius * math.cos(fieta /180*PI)+0.132
				X3_2 = -radius * math.cos((theta+5.625)/180*PI) * math.sin((fieta+4.0625) /180*PI)
				Y3_2 = -radius * math.sin((theta+5.625)/180*PI) * math.sin((fieta+4.0625) /180*PI)
				Z3_2 = radius * math.cos((fieta+4.0625) /180*PI)+0.132

				X4_1 = -(radius+0.031) * math.cos((theta+5.625)/180*PI) * math.sin(fieta /180*PI)
				Y4_1 = -(radius+0.031) * math.sin((theta+5.625)/180*PI) * math.sin(fieta /180*PI)
				Z4_1 = (radius+ 0.031) * math.cos(fieta /180*PI)+0.132
				X4_2 = -(radius+0.031) * math.cos((theta+5.625)/180*PI) * math.sin((fieta+4.0625) /180*PI)
				Y4_2 = -(radius+0.031) * math.sin((theta+5.625)/180*PI) * math.sin((fieta+4.0625) /180*PI)
				Z4_2 = (radius+ 0.031) * math.cos((fieta+4.0625) /180*PI)+0.132


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
