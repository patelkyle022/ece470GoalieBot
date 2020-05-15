import numpy as np
from sim import simxSetObjectPosition, simxReadProximitySensor, \
    simx_opmode_oneshot_wait, simx_opmode_streaming, simx_opmode_oneshot

#this function executes while calc_ball_position and our analytical inverse kinematics calculation executes
def move_ball(clientID, ball_handle, path):
    for pos in range(path.shape[0]): #set ball position to each point in the path
        simxSetObjectPosition(clientID, ball_handle, -1, path[pos], simx_opmode_oneshot_wait)
        
#this function executes while move_ball executes
def calc_ball_position(clientID, sensor_handle, path, detectedPoints):
    found1 = False
    found2 = False
    count = 0
    for i in range(path.shape[0]):
        detectionData = simxReadProximitySensor(clientID, sensor_handle, simx_opmode_streaming)
        print(i) #too little time between reads results in garbage data
        if (detectionData[1] and not found1): #detectionData[1]: ball detection is true and first point hasn't been found
            detectedPoints[0] = detectionData[2]
            found1 = True
        #wait a sufficient amount of time to get a second position, so that a "velocity"/slope of movement can be found
        if (detectionData[1] and found1 and not found2):
            if (count > 50):
                detectedPoints[1] = detectionData[2]
                return
            count += 1


    

