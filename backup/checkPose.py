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



if __name__ == "__main__":
	load_Pose()
	world = WorldModel()

	res = world.readFile('./anno_check.xml')
	if not res:
		raise RuntimeError("Unable to load model ") 
			
	vis.add("world",world)

	robot = world.robot(0)
	vis.show()
	collisionTest = WorldCollider(world)
	
	robotPose = RobotPoser(robot)



	robotPose.set([0,pi/2,-pi/2,0,0,pi/2,0])
	collisionTest = WorldCollider(world)

	print len(Pose)
	for i in range(0,453):
		Pose[i].insert(0,0)
		# realPose.append(Pose[i])
		print (Pose[i])
		robotPose.set(Pose[i])
		collisionTest = WorldCollider(world)
		time.sleep(1)
		vis.show()





			#pass
			

