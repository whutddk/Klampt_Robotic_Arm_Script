# @File Name: ikTest.py
# @File Path: K:\work\MAS2\PRM_robotic_arm\Klampt_Robotic_Arm_Script\ikMount\ikTest.py
# @Author: 29505
# @Date:   2019-04-24 10:06:47
# @Last Modified by:   29505
# @Last Modified time: 2019-04-24 10:46:29
# @Email: 295054118@whut.edu.cn
# @page: https://whutddk.github.io/
import klampt
from klampt.model import ik



world = klampt.WorldModel()
world.loadElement("../robots/probot_6axis.rob")

robot= world.robot(0)
link = robot.link(7)
print (robot.getConfig())

obj = ik.objective(link,local=[1,0,0],world=[1.5,0,1])
solver = ik.solver(obj)
solver.solve()

robot.getConfig()

print (solver.getResidual())