import sim
import sys
import math
import time
import move_helper

global PI
PI = math.pi


sim.simxFinish(-1)
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit('Could not connect to remote API server')

jointHandles = [-1, -1, -1, -1, -1, -1]

#Get joint handles
#for i in range(0,6):
#    jointHandles[i] = sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i + 1), sim.simx_opmode_blocking) [1]


print(move_helper.getJointHandles(clientID, jointHandles))

#Get Proximity Sensor Handle
errorCode, sensorHandle = sim.simxGetObjectHandle(clientID, 'Ball_Sensor', sim.simx_opmode_blocking)
print(sensorHandle)


#Set a target position for movement
targetPos1 = [45*PI/180, -90*PI/180, 0, 0, 0, 0]



sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
time.sleep(1)

#Read Prox Sensor
errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSNV = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
print("Before Movement: " + str(detectionState))

time.sleep(1)


#Send command for moving robot.
move_helper.setTargetPosition(clientID, jointHandles, targetPos1)
#for i in range(0,6):
#    sim.simxSetJointTargetPosition(clientID, jointHandles[i], targetPos1[i], sim.simx_opmode_oneshot_wait)

time.sleep(1)

#Read Prox Sensor
(errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector) = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
print("After Movement: " + str(detectionState))

sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
sim.simxFinish(clientID)