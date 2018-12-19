from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis

import json

import random

pi = 3.14159 




if __name__ == "__main__":
	
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


	while(1):
		time.sleep(0.1)
		vis.shown()
		pass

			#pass
			

