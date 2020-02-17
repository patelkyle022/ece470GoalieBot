import sim

def getJointHandles(clientID, jointHandles):
    for i in range(0,6):
        jointHandles[i] = sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i + 1), sim.simx_opmode_blocking) [1]

def setTargetPosition(clientID, jointHandles, targetPosition):
    for i in range(0,6):
        sim.simxSetJointTargetPosition(clientID, jointHandles[i], targetPosition[i], sim.simx_opmode_oneshot_wait)