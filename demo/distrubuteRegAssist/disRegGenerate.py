# -*- coding: utf-8 -*-
# @File Name: disRegGenerate.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Verilog_RA_Script\state2\distrubuteRegAssist\disRegGenerate.py
# @Author: Ruige_Lee
# @Date:   2019-07-07 15:44:34
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-07-21 21:15:28
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



import sys
import time

import json

import time


regNum = 256

def write_verilog():

	nowtime = time.localtime(time.time())

	with open('./regTable.v','w') as verilogFile:

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


		for i in range(0,regNum):
			strSyntax = strSyntax +'reg [31:0] edgeReg_'+ str(i) +';\n'

		strSyntax = strSyntax + '\n\nassign edgeWire = {'

		for i in range(0,regNum-1): 
			strSyntax = strSyntax + 'edgeReg_'+ str(255-i) +', '

		strSyntax = strSyntax + 'edgeReg_'+ str(0) +'};\n\n\n '


		for i in range (0,regNum):
			strSyntax = strSyntax +' edgeReg_'+ str(i) +' <= 32\'b0;\n'

		strSyntax = strSyntax + 'case()\n'

		for i in range (0,regNum):
			strSyntax = strSyntax + '8\'d' + str(i) + ': edgeReg_'+ str(i) + ' <= S_AXI_WDATA | edgeReg_'+ str(i) + ';\n'
		strSyntax = strSyntax + 'endcase'


		strSyntax = strSyntax + '\n\nassign {'
		for i in range (0,regNum-1):
			strSyntax = strSyntax +'edgeWire_'+ str(255-i) +', '
		strSyntax = strSyntax + 'edgeWire_'+ str(0) +'} = edgeWire;\n\n\n '

		for i in range (0,regNum):
			strSyntax = strSyntax + 'wire [31:0] edgeWire_'+ str(i) + ';\n'


		verilogFile.write(strSyntax);
		



	pass


write_verilog()
print ('done!')



