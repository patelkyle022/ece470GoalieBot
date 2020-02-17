from move_helper import *
import sim
import sys
import math
import time

global PI
PI = math.pi


sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit('Could not connect to remote API server')

jointHandles = [-1, -1, -1, -1, -1, -1]

getJointHandles(clientID, jointHandles)

print(jointHandles)


targetPos1 = [90*PI/180, 90*PI/180, -90*PI/180, 90*PI/180, 90*PI/180, 90*PI/180]

sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
time.sleep(1)

setTargetPosition(clientID, jointHandles, targetPos1)

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
sim.simxFinish(clientID)