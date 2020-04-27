import numpy as np
from scipy.linalg import expm
from sim import simxSetObjectPosition, simxReadProximitySensor, \
    simx_opmode_oneshot_wait, simx_opmode_streaming, simx_opmode_oneshot
import time
w = np.array([[ 0.,  0.,  1.], \
       [-1.,  0.,  0.], \
       [-1.,  0.,  0.], \
       [-1.,  0.,  0.], \
       [ 0.,  0.,  1.], \
       [-1.,  0.,  0.]])


def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


#perform forward kinematics of UR3 with an input joint angles
def getT(clientID, thetas, w, q, v):
    identity = np.eye(4)

    targetAngles = [0,0,0,0,0,0]
    for i in range(6):
        targetAngles[i] = thetas[i]

    wb = np.zeros((6,3,3))
    for i in range(6):
        wb[i] = skew(w[i])

    Sb = np.zeros((6,4,4))
    for i in range(6):
        Sb[i,:3,:3] = wb[i]
        Sb[i,:3,3] = np.array(v[i]).T
        Sb[i] *= targetAngles[i]

    e_Sb = np.zeros((6,4,4))
    for i in range(6):
        e_Sb[i] = expm(Sb[i])

    T = np.eye(4)
    for i in range(6):
        T = np.matmul(T, e_Sb[i])

    T = np.matmul(T, identity)

    return T 


def move_ball(clientID, ball_handle, path):
    for pos in range(path.shape[0]):
        simxSetObjectPosition(clientID, ball_handle, -1, path[pos], simx_opmode_oneshot_wait)
    


def calc_ball_position(clientID, sensor_handle, path, detectedPoints):
    found1 = False
    found2 = False
    count = 0
    for i in range(path.shape[0]):
        detectionData = simxReadProximitySensor(clientID, sensor_handle, simx_opmode_streaming)
        print(i)
        if (detectionData[1] and not found1):
            detectedPoints[0] = detectionData[2]
            found1 = True
        #wait a sufficient amount of time to get a second position, so that a "velocity"/slope of movement can be found
        if (detectionData[1] and found1 and not found2):
            if (count > 50):
                detectedPoints[1] = detectionData[2]
                return
            count += 1


    

