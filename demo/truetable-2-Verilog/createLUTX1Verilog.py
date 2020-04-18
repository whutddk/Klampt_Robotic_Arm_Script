# -*- coding: utf-8 -*-
# @File Name: createLUTX1Verilog.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Verilog_RA_Script\state2\truetable-2-Verilog\createLUTX1Verilog.py
# @Author: Ruige_Lee
# @Date:   2019-07-07 15:44:34
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-07-07 16:35:29
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/
# -*- coding: utf-8 -*-
# @File Name: createLUTX1Verilog.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Verilog_RA_Script\state2\truetable-2-Verilog\createLUTX1Verilog.py
# @Author: Ruige_Lee
# @Date:   2019-02-19 11:59:19
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-03-04 17:07:15
# @Email: 295054118@whut.edu.cn"

# @File Name: createLUTX1Verilog.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Verilog_RA_Script\truetable-2-Verilog\createLUTX1Verilog.py
# @Author: 29505
# @Date:   2019-02-13 11:04:49
# @Last Modified by:   29505
# @Last Modified time: 2019-02-13 16:00:23
# @Email: 295054118@whut.edu.cn

import sys
import time

import json

import time


trueTable = []
edgeNum = 0
gridNum = 16384

def load_trueTable(k):
	global trueTable

	with open( '../../../Klampt_Robotic_Arm_Script/state2/createTrueTable/trueTable'+ str(k) +'.json','r') as trueTableFile:
		data = trueTableFile.read()
		trueTable = json.loads(data)
	#print trueTable

	# edgeNum = len(trueTable[0])
	# print (edgeNum)
	pass


def write_verilog(k):
	global trueTable
	edgeNum = len(trueTable[0])
	nowtime = time.localtime(time.time())

	with open('./trueTableP'+ str(k) +'.v','w') as verilogFile:

		strSyntax = '/*******************************************\n'
		strSyntax = strSyntax + '****** Wuhan university of technology ******\n'
		strSyntax = strSyntax + '****** Ruige Lee ******\n'
		strSyntax = strSyntax + 'year: ' + str(nowtime.tm_year) + '\n'
		strSyntax = strSyntax + 'month: ' + str(nowtime.tm_mon) + '\n'
		strSyntax = strSyntax + 'date: ' + str(nowtime.tm_mday) + '\n'
		strSyntax = strSyntax + 'hour: ' + str(nowtime.tm_hour) + '\n'
		strSyntax = strSyntax + 'minutes: ' + str(nowtime.tm_min) + '\n'
		strSyntax = strSyntax + 'second: ' + str(nowtime.tm_sec) + '\n'
		strSyntax = strSyntax + '********************************************/\n\n'



		strSyntax = strSyntax +'module prm_LUTX1chkp'+ str(k) +'(\n'
		strSyntax = strSyntax + '	input [3:0] x,\n'
		strSyntax = strSyntax + '	input [4:0] y,\n'
		strSyntax = strSyntax + '	input [4:0] z,\n'
		strSyntax = strSyntax + '	output ['+ str(edgeNum-1) +':0] edge_mask_p'+ str(k) + '\n'
		strSyntax = strSyntax + ');\n\n'
		strSyntax = strSyntax + '	reg ['+ str(edgeNum-1) +':0] edge_mask_reg_p'+ str(k) + ';\n'
		strSyntax = strSyntax + '	assign edge_mask_p' + str(k) + '= edge_mask_reg_p'+ str(k) +';\n\n'
		strSyntax = strSyntax + 'always @( * ) begin\n' 


		verilogFile.write(strSyntax);
		for j in range(0,edgeNum): #all edge
			strData = '    case({z,y,x})\n'
			active  = False
			
			for i in range(0,gridNum): #all pix
				if (trueTable[i][j] == 1):
					active  = True
					strData = strData + '	14\'b' + bin(i)[2:14+2] +',\n'
			if ( active == True ):
				strData = strData[0:-2]
				strData = strData + ': edge_mask_reg_p'+ str(k) + '[' + str(j) + '] <= 1\'b1;\n '
			strData = strData + '		default: edge_mask_reg_p'+ str(k) + '[' + str(j) + '] <= 1\'b0;\n '
			strData = strData + '	endcase\n\n'
			verilogFile.write(strData)

		strSyntax =  'end\n'
		strSyntax = strSyntax + 'endmodule\n\n'

		verilogFile.write(strSyntax) 

	pass

for k in range(0,7):
	load_trueTable(k)
	write_verilog(k)
print ('done!')



