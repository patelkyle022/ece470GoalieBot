import sim
import sys
import math
import time
import timeit
import numpy as np
from scipy.linalg import expm
import modern_robotics as mr

global PI
PI = math.pi

#just a 4x4 identity matrix
global identity 
identity = np.eye(4)

#the w's for the joints of the UR3 (all are revolute)
global w
w = np.zeros((6,3))
w[0] = np.array([0, 0, 1]).T
w[1] = np.array([-1, 0, 0]).T
w[2] = np.array([-1, 0, 0]).T
w[3] = np.array([-1, 0, 0]).T
w[4] = np.array([0, 0, 1]).T
w[5] = np.array([-1, 0, 0]).T

#global q for the positions of each joint
global q
q = np.zeros((6,3))

#v's for each joint
global v
v = np.zeros((6,3))

base_handle = 0
end_effector_handle = 0
end_effector_wrt_base = [0,0,0]

#get the skew form of a 3x1 column vector
def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

#get forward kinematics of UR3 with an input joint angles
def getT(clientID, jointHandles, thetas):
    global w
    global v
    global identity

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

def main():
    global q
    global w
    global v
    global base_handle
    sim.simxFinish(-1)
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect to remote API server')
        
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

    jointHandles = [-1, -1, -1, -1, -1, -1]
    #Get joint handles
    for i in range(0,6):
        jointHandles[i] = sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i + 1), sim.simx_opmode_blocking) [1]

    #set q values
    for i in range(6):
        q[i] = np.array(sim.simxGetObjectPosition(clientID, jointHandles[i], base_handle, sim.simx_opmode_blocking) [1])

    #set v values
    for i in range(6):
        v[i] = np.cross(-w[i], q[i])

    #calculate screw axes
    S = np.zeros((6,6))
    for i in range(6):
        S[i,:3] = w[i]
        S[i,3:] = v[i] 

    S = S.T  #actual screw axes is transpose of this

    global end_effector_handle
    global identity

    
    #defining random endpoint constraints
    minY = -0.54; maxY = 0.54
    minZ = 0.30
    maxZ = 0.60
    path_length = np.random.randint(700, 800)

    #the end position of the ball, random each time!
    endX = -1.2056 #position of base in world frame
    endY = np.random.uniform(minY, maxY)
    endZ = np.random.uniform(minZ, maxZ)
    print('endY: ' + str(endY))
    print('endZ: ' + str(endZ))
    

    #handle of UR3
    base_handle = sim.simxGetObjectHandle(clientID, "UR3", sim.simx_opmode_blocking)[1]
    
    #handle for end-effector
    end_effector_handle = sim.simxGetObjectHandle(clientID, "UR3_link7", sim.simx_opmode_blocking)[1]
    
    #position of UR3 end effector wrt UR3 base frame
    end_effector_wrt_base = sim.simxGetObjectPosition(clientID, end_effector_handle, base_handle, sim.simx_opmode_blocking)[1]

    #handle for the ball
    ball_handle = sim.simxGetObjectHandle(clientID, 'Ball', sim.simx_opmode_blocking)[1]

    #handle for collisions
    collision_handle = sim.simxGetCollisionHandle(clientID, 'Collision', sim.simx_opmode_blocking)[1]

    #handle for ball sensor/proximity sensor
    sensorHandle = sim.simxGetObjectHandle(clientID, 'Ball_Sensor', sim.simx_opmode_blocking)[1]

    #start position of ball in world frame
    ball_start_pos = np.array([2.350, 0, 0.05])

    #in world frame coords, the path that the ball takes towards the goal
    x_path = np.linspace(ball_start_pos[0], endX, path_length)
    y_path = np.linspace(ball_start_pos[1], endY, path_length)
    z_path = np.linspace(ball_start_pos[2], endZ, path_length)
    path = np.vstack((x_path, y_path, z_path)).T
    
    #home position of UR3 end-effector in coords of UR3 base frame
    M_endeff_in_base = np.array([[-1,   0,      0, end_effector_wrt_base[0]], 
         [0,    -1,     0, end_effector_wrt_base[1]], 
         [0,    0,      1, end_effector_wrt_base[2]], 
         [0,    0,      0,                        1]])
    #position and orientation of proximity sensore wrt UR3 base frame
    T_sensor_in_base = np.array([[0, 0, -1, end_effector_wrt_base[0]], [0, -1, 0,  end_effector_wrt_base[1]] \
    , [-1, 0, 0,  end_effector_wrt_base[2]], [0, 0, 0, 1]])

    
    #the actual end point of the ball based on path in the world frame
    end_pt = path[path_length - 1]

    #the position and orientation of the UR3 base frame wrt world frame (T_wb)
    T_base_in_world = np.array([[-1, 0, 0, -1.4001], [0, -1, 0, -0.000086], [0, 0, 1, 0.043], [0, 0, 0, 1]])
    #T_bw
    T_world_in_base = np.linalg.inv(T_base_in_world)

    #end pt coords in homog form
    end_pt_homogenous_coords = np.array([[end_pt[0], end_pt[1], end_pt[2], 1]]).T

    #T_bw * p_w = p_b (4,4) * (4, 1) -> (4, 1), left out final element -> (3, 1)
    #the actual endpt of the ball in the base frame
    end_pt_ball_in_base = np.matmul(T_world_in_base, end_pt_homogenous_coords)[:3]

    
    #T_sd - desired position and orientation of end effector in base (spatial) frame
    T_sd = M_endeff_in_base
    T_sd[:3, 3:] = np.array([end_pt_ball_in_base])
    

    #Sensor readings of ball positions
    detectedPoint1 = None
    detectedPoint2 = None
    found1 = False
    found2 = False
    count = 0
    for i in range(path.shape[0]):
        sim.simxSetObjectPosition(clientID, ball_handle, -1, path[i], sim.simx_opmode_oneshot_wait)
        detectionData = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
        #get a first detected position
        if (detectionData[1] and not found1):
            detectedPoint1 = detectionData[2]
            found1 = True
        #wait a sufficient amount of time to get a second position, so that a "velocity"/slope of movement can be found
        if (detectionData[1] and found1 and not found2):
            if (count > 50):
                detectedPoint2 = detectionData[2]
                found2 = True
            count += 1

    #point_in_sensorX - homogeneous coords of detectedPoints in the frame of the proximity sensor
    point_in_sensor1 = np.array([[detectedPoint1[0]], [detectedPoint1[1]], [detectedPoint1[2]], [1]])
    point_in_sensor2 = np.array([[detectedPoint2[0]], [detectedPoint2[1]], [detectedPoint2[2]], [1]])

    #the detected points, in the frame of the UR3 base
    #T_bs * p_s = p_b
    point1_ball_in_base = np.matmul(T_sensor_in_base, point_in_sensor1)[:3]
    point2_ball_in_base = np.matmul(T_sensor_in_base, point_in_sensor2)[:3]
    
    #the predicted direction the ball is moving in, based off of predicted points 1 and 2
    vector = (1.0*point2_ball_in_base - point1_ball_in_base)

    #convert to unit vector
    vector /= np.linalg.norm(vector)

    #if the difference between detected points is too small
    if (vector[0] == 0):
        vector[0] += 0.000000001

    #find the estimated position of the ball when it is in the yz plane of the UR3
    t = (end_effector_wrt_base[0] - point2_ball_in_base[0]) / vector[0]
    final_x = end_effector_wrt_base[0]
    final_y = point2_ball_in_base[1] + (t * vector[1])
    final_z = point2_ball_in_base[2] + (t * vector[2])
    
    #the predicted point of the ball
    predicted_point = np.array([[final_x], [final_y[0]], [final_z[0]]])
    print("Predicted Point: " + str(predicted_point))
    print("End Point Ball in Base: " + str(end_pt_ball_in_base))
    #compare prediction to actual end pt of ball
    difference = predicted_point - end_pt_ball_in_base
    print("Difference: " + str(difference))


    
    #final position and orientation of end-effector based on predicted ball location
    #orientation doesn't need to change, just the position to block the ball
    final_T = np.copy(M_endeff_in_base)
    final_T[:3,3:] = predicted_point
    #our initial guess of what the UR3 joint angles should be to get to finalT
    initialGuess = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #use inverse kinematics to find joint angles that will get us to finalT
    (thetas1, success) = mr.IKinSpace(S, M_endeff_in_base, final_T, initialGuess, 0.8, 0.015)
    print('thetas1: ' + str(thetas1))
    #set UR3 joints to angles to block ball!
    for i in range(6):
        sim.simxSetJointTargetPosition(clientID, jointHandles[i], thetas1[i], sim.simx_opmode_oneshot_wait)

    time.sleep(1)

         

    print("Actual End Effector Position " + str(sim.simxGetObjectPosition(clientID, end_effector_handle, base_handle, sim.simx_opmode_blocking)[1]))

    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sim.simxFinish(clientID)
    return 1

if __name__ == "__main__":
    main()