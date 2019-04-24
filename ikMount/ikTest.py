# @File Name: ikTest.py
# @File Path: K:\work\MAS2\PRM_robotic_arm\Klampt_Robotic_Arm_Script\ikMount\ikTest.py
# @Author: 29505
# @Date:   2019-04-24 10:06:47
# @Last Modified by:   29505
# @Last Modified time: 2019-04-24 11:37:18
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/

from klampt import *
from klampt.model.collide import *
import sys
import time
from klampt.sim import *
from klampt import vis
from klampt.model import ik

import random


if __name__ == "__main__":


    world = WorldModel()

    res = world.readFile('../dual_anno_check.xml')
    if not res:
        raise RuntimeError("Unable to load model ") 
    del res

    prmRobot = world.robot(0)
    ctlRobot = world.robot(1)

    vis.add("world",world)
    vis.show()


    # collisionTest = WorldCollider(world)
    
    prmRobotPose = RobotPoser(prmRobot)
    ctlRobotPose = RobotPoser(ctlRobot)


    robot= world.robot(0)
    link = prmRobot.link(7)
    obj = ik.objective(link,R=[1,0,0,0,0,1,0,-1,0],t=[0.3,0,0])

    solver = ik.solver(obj)
    solver.solve()

    prmRobotPose.set(robot.getConfig())
    ctlRobotPose.set([0,0,0,0,0,0,0,0])

    while(1):
        time.sleep(0.1)
        pass


