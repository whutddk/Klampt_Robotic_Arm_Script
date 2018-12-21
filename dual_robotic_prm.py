from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis

import json

import random

pi = 3.14159 

Pose = []

def load_Pose():
	global Pose

	with open('./poseAxis.json','r') as poseFile:
		data = poseFile.read()
		Pose = json.loads(data)
		print Pose
	pass


def robotCollideRobot():
	collisionTest = WorldCollider(world)
	for i in collisionTest.robots[0]:
		for j in collisionTest.robots[1]:
			if j < 0: continue
			if i < 0: continue
			if collisionTest.geomList[j][1].collides(collisionTest.geomList[i][1]):
				return True
	return False



if __name__ == "__main__":
	# load_Pose()
	world = WorldModel()

	res = world.readFile('./anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			
	vis.add("world",world)

	prmRobot = world.robot(0)
	ctlRobot = world.robot(1)
	vis.show()
	collisionTest = WorldCollider(world)
	
	prmRobotPose = RobotPoser(prmRobot)
	ctlRobotPose = RobotPoser(ctlRobot)


	prmRobotPose.set([0,pi/2,-pi/2, 0,   0, pi/2,0])
	

	for k in range (0,50):
		ctlRobotPose.set([0,-pi/100 * k + pi/2,   -pi/2    ,pi/2, 0, pi/2,0])
		collisionTest = WorldCollider(world)
		time.sleep(0.1)
	# for l in collisionTest.geomList[1]:
	# 	print l
		
		if (robotCollideRobot()):
			print "collision!"


		



	while(1):
		time.sleep(0.1)
		vis.show()
		pass

# set ctlRobot Random Pose

#	set 0,1,2,3,4 active 
# while(!10 group complete)
	# for 10W edge:
		# if ( one active  another not ) in all group:
			# collision check
			# if pass :
				# activate another
				# finish check
					# if finish :
					# 	used edgeNum ++
					#   finished group mark 
				
			
			# delete edge from 10W edgeList
