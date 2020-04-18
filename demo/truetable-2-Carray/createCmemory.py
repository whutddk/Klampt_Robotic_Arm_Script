# -*- coding: utf-8 -*-
# @File Name: createCmemory.py
# @File Path: M:\MAS2\PRM_Robotic_Arm\Verilog_RA_Script\state2\truetable-2-Carray\createCmemory.py
# @Author: Ruige_Lee
# @Date:   2019-07-13 11:05:55
# @Last Modified by:   Ruige_Lee
# @Last Modified time: 2019-07-21 13:58:34
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/
import sys
import time

import json

trueTable = []


TempTrueTable = [[],[],[],[],[],[],[]]

edgeNum = 8192
gridNum = 16384
wordNum = 8192 // 32

wordtable = []


def load_trueTable():
	global trueTable

	for k in range (0,7):
		with open( '../../../Klampt_Robotic_Arm_Script/state2/createTrueTable/trueTable'+ str(k) +'.json','r') as trueTableFile:
			data = trueTableFile.read()
			TempTrueTable[k] = json.loads(data)

	for grid in range (0,16384):
		oneGrid = []
		for part in range (0,7):
			for edge in TempTrueTable[part][grid]:
				oneGrid.append(edge)
		
		# print (len(oneGrid))
		while( len(oneGrid) < 8192 ):
			oneGrid.append(0)
		trueTable.append(oneGrid)
	print (len(trueTable))

	pass


def mixInwords():
	global trueTable
	global wordtable

	for grid in trueTable:
		onegrid = []

		for wordNum in range(0,256):
			word = ''
			for i in range (0,32):
				word = word + str(grid[32*wordNum + i])
			result = hex(int(word,2))
			# print ('word =',word)
			# print ('result =',result)
			onegrid.append(result)
		wordtable.append(onegrid)
	# print (wordtable)




def write_Cfile():
	global trueTable
	nowtime = time.localtime(time.time())

	with open('./CMemoryTable.c','w') as CFile:

		strSyntax = '/*******************************************\n'
		strSyntax = strSyntax + '****** Wuhan University of Technology ******\n'
		strSyntax = strSyntax + '****** Ruige Lee ******\n'
		strSyntax = strSyntax + 'year: ' + str(nowtime.tm_year) + '\n'
		strSyntax = strSyntax + 'month: ' + str(nowtime.tm_mon) + '\n'
		strSyntax = strSyntax + 'date: ' + str(nowtime.tm_mday) + '\n'
		strSyntax = strSyntax + 'hour: ' + str(nowtime.tm_hour) + '\n'
		strSyntax = strSyntax + 'minutes: ' + str(nowtime.tm_min) + '\n'
		strSyntax = strSyntax + 'second: ' + str(nowtime.tm_sec) + '\n'
		strSyntax = strSyntax + '********************************************/\n\n'


		strSyntax = strSyntax + '__attribute__ ((section("dataBf1"))) volatile uint32_t lookUpTrueTable[16384]['+str(wordNum)+'] = {\n'
		CFile.write(strSyntax)
		
		for grid in range(0,gridNum):	#all case all grid  16384
			strSyntax = '	'
			strSyntax = strSyntax + '{'
			for edge in wordtable[grid]:
				
				strSyntax = strSyntax + edge + ','

			strSyntax = strSyntax[:-1] + "},\n"
			CFile.write(strSyntax)
			# print (strSyntax)
			
		strSyntax = '};\n'
		CFile.write(strSyntax)

			


load_trueTable()
mixInwords()
write_Cfile()
print ('done!')



