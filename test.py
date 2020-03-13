import sim
import sys
import math
import time
import numpy as np
from scipy.linalg import expm

global PI
PI = math.pi

M = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
base_handle = 0
end_effector_handle = 0
end_effector_wrt_base = [0,0,0]

def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

def getT(clientID, jointHandles, thetas):
    targetAngles = [0,0,0,0,0,0]
    for i in range(6):
        targetAngles[i] = thetas[i]

    q = np.zeros((6,3))

    for i in range(6):
        q[i] = np.array(sim.simxGetObjectPosition(clientID, jointHandles[i], base_handle, sim.simx_opmode_blocking) [1])

    w = np.zeros((6,3))
    w[0] = np.array([0, 0, 1]).T
    w[1] = np.array([-1, 0, 0]).T
    w[2] = np.array([-1, 0, 0]).T
    w[3] = np.array([-1, 0, 0]).T
    w[4] = np.array([0, 0, 1]).T
    w[5] = np.array([-1, 0, 0]).T

    v = np.zeros((6,3))
    for i in range(6):
        v[i] = np.cross(-w[i], q[i])

    S = np.zeros((6,6))
    for i in range(6):
        S[i,:3] = w[i]
        S[i,3:] = v[i] 

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

    T = np.matmul(T, M)

    return T

def main():
    sim.simxFinish(-1)
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect to remote API server')

    global M
    global base_handle
    global end_effector_handle
    global end_effector_wrt_base

    base_handle = sim.simxGetObjectHandle(clientID, "UR3", sim.simx_opmode_blocking) [1]
    end_effector_handle = sim.simxGetObjectHandle(clientID, "UR3_link7", sim.simx_opmode_blocking) [1]
    end_effector_wrt_base = sim.simxGetObjectPosition(clientID, end_effector_handle, base_handle, sim.simx_opmode_blocking) [1]
    M = [[-1,   0,      0, end_effector_wrt_base[0]], 
         [0,    -1,     0, end_effector_wrt_base[1]], 
         [0,    0,      1, end_effector_wrt_base[2]], 
         [0,    0,      0,                        1]]

    jointHandles = [-1, -1, -1, -1, -1, -1]

    #Get joint handles
    for i in range(0,6):
        jointHandles[i] = sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i + 1), sim.simx_opmode_blocking) [1]

    # print(jointHandles)

    #Get Proximity Sensor Handle
    sensorHandle = sim.simxGetObjectHandle(clientID, 'Ball_Sensor', sim.simx_opmode_blocking) [1]
    # print(sensorHandle)

    thetas1 = [45*PI/180, -90*PI/180, 0, 0, 0, 0] 

    T = getT(clientID, jointHandles, thetas1)

    print(T)

    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
    time.sleep(1)

    #Read Prox Sensor
    errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSNV = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
    # print("Before Movement: " + str(detectionState))

    time.sleep(1)


    #Send command for moving robot.
    for i in range(0,6):
        sim.simxSetJointTargetPosition(clientID, jointHandles[i], thetas1[i], sim.simx_opmode_oneshot_wait)
    '''
    ball_start_point = detectedPoint
    ball_handle = detectedObjectHandle
    errorCode, path_handle = sim.simxgetObjectHandle(clientID, 'Path', sim.simx_opmode_blocking)
    straight_line_path = np.array([[]])

    while
        simx.setObjectPosition(ball_handle, )
    '''
    time.sleep(1)

    #Read Prox Sensor
    (errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector) = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
    # print("After Movement: " + str(detectionState))

    print(sim.simxGetObjectPosition(clientID, end_effector_handle, base_handle, sim.simx_opmode_blocking) [1])

    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sim.simxFinish(clientID)
    return 1

if __name__ == "__main__":
    main()
