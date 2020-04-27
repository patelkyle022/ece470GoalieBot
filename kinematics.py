import sim
import sys
import numpy as np
import modern_robotics as mr
import math
import move_helper as move
import time
from scipy.spatial.transform import Rotation
import random

global PI
PI = math.pi

def degToRad(angle):
    return angle * PI / 180

def startSimulation():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 50)
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect to remote API server')
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
    return clientID

def endSimulation(clientID):
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sim.simxFinish(clientID)

def getSlist(clientID, joint_handles, base_handle):
    q = np.zeros((6,3))
    for i in range(6):
        q[i] = np.array(sim.simxGetObjectPosition(clientID, joint_handles[i], base_handle, sim.simx_opmode_blocking)[1])

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

    Slist = np.zeros((6,6))
    for i in range(6):
        Slist[i,:3] = w[i]
        Slist[i,3:] = v[i] 

    return Slist
    

def main():
    clientID = startSimulation()

    # Handles
    joint_handles = [-1, -1, -1, -1, -1, -1]
    for i in range(0,6):
        joint_handles[i] = sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i + 1), sim.simx_opmode_blocking) [1]
    base_handle = sim.simxGetObjectHandle(clientID, "UR3", sim.simx_opmode_blocking)[1]
    end_effector_handle = sim.simxGetObjectHandle(clientID, "UR3_link7_visible", sim.simx_opmode_blocking)[1]
    
    end_effector_wrt_base = sim.simxGetObjectPosition(clientID, end_effector_handle, base_handle, sim.simx_opmode_streaming)[1]
    end_effector_quat_wrt_base = sim.simxGetObjectQuaternion(clientID, end_effector_handle, base_handle, sim.simx_opmode_streaming)[1]

    time.sleep(0.5)

    # Get end effector position and rotation wrt base
    end_effector_wrt_base = sim.simxGetObjectPosition(clientID, end_effector_handle, base_handle, sim.simx_opmode_oneshot_wait)[1]
    end_effector_quat_wrt_base = sim.simxGetObjectQuaternion(clientID, end_effector_handle, base_handle, sim.simx_opmode_oneshot_wait)[1]
    

    R_end_effector_wrt_base = Rotation.from_quat(end_effector_quat_wrt_base).as_matrix()
    P_end_effector_wrt_base = end_effector_wrt_base

    
    M = mr.RpToTrans(R_end_effector_wrt_base, P_end_effector_wrt_base)
    print(M)

    # Forward Kinematics
    Slist = getSlist(clientID, joint_handles, base_handle)
    theta = [degToRad(180), degToRad(0), degToRad(0), degToRad(0), degToRad(0), degToRad(0)]
    T_endeff_in_base = mr.FKinSpace(M, Slist.T, theta)

    print(T_endeff_in_base)

    success = False
    while (not success):
        theta_new, success = mr.IKinSpace(Slist.T, M, T_endeff_in_base, [random.random() ,random.random() ,random.random() ,random.random() ,random.random() ,random.random()], 0.01, 0.01)


    time.sleep(1)

    move.setTargetPosition(clientID, joint_handles, theta_new)

    errorCode = 1
    while (errorCode): 
        errorCode, end_effector_wrt_base = sim.simxGetObjectPosition(clientID, end_effector_handle, base_handle, sim.simx_opmode_oneshot_wait)

    print(end_effector_wrt_base)

    time.sleep(1)

    move.setTargetPosition(clientID, joint_handles, [0, 0, 0, 0, 0, 0])

    endSimulation(clientID)
    return 1
    

if __name__ == "__main__":
    main()