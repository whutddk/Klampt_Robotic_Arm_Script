import sys
from klampt import*
from klampt import vis
from klampt.io import resource
from klampt.model import coordinates
from klampt.model import trajectory
import random, time, math

if __name__=="__main__":
    point = coordinates.addPoint("point")
    vis.add("point", point)
    traj = trajectory.Trajectory()
    minDis = -1
    maxDis = 1
    for i in range(10):
        traj.times.append(i/2.0)
        minDis =-1
        maxDis = 1
        val = 1
        if i%2==1:
            val = -1
        traj.milestones.append([(maxDis-minDis)*i/10.0+minDis, 0, val*0.2])
    traj2 = trajectory.HermiteTrajectory()
    traj2.makeSpline(traj)
    vis.animate("point", traj2)
    vis.show()

    iteration = 0
    while vis.shown():
        iteration+=1
    vis.kill()
