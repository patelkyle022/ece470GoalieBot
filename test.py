import sim
import sys
import numpy as np
import utils
import threading
import simConst as sc

#generate_path gives a random direction towards the goal with velocity 2 m/s
def generate_path():
    #defining random endpoint constraints
    ball_start_pos = np.array([2.350, 0, 0.05])
    theta =  np.pi * np.random.uniform(1 / 6, 5 / 6)
    r = 0.54 * np.random.random_sample()
    endY = r * np.cos(theta)
    endZ = 0.152 + r * np.sin(theta)
    path_length = np.random.randint(400, 500)
    #the end position of the ball, random each time!
    endX = -1.4001 #position of base in world frame
    #in world frame coords, the path that the ball takes towards the goal
    x_path = np.linspace(ball_start_pos[0], endX, path_length)
    y_path = np.linspace(ball_start_pos[1], endY, path_length)
    z_path = np.linspace(ball_start_pos[2], endZ, path_length)
    path = np.vstack((x_path, y_path, z_path)).T
    return path


def main():
    sim.simxFinish(-1)
    
    clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)
    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect to remote API server')
    sim.simxSynchronous(clientID, 1) #synchronous operation necessary for 
    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
    
    
    jointHandles = [-1, -1, -1, -1, -1, -1]
    #Get joint handles
    for i in range(0,6):
        jointHandles[i] = sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i + 1), sim.simx_opmode_blocking) [1]
        
    #handle of UR3
    base_handle = sim.simxGetObjectHandle(clientID, "UR3", sim.simx_opmode_blocking)[1]
    
    #handle for end-effector
    end_effector_handle = sim.simxGetObjectHandle(clientID, "UR3_link7", sim.simx_opmode_blocking)[1]
    
    #position of UR3 end effector wrt UR3 base frame
    end_effector_wrt_base = sim.simxGetObjectPosition(clientID, end_effector_handle, base_handle, sim.simx_opmode_blocking)[1]

    #handle for the ball
    ball_handle = sim.simxGetObjectHandle(clientID, 'Ball', sim.simx_opmode_blocking)[1]

    #handle for ball sensor/proximity sensor
    sensorHandle = sim.simxGetObjectHandle(clientID, 'Ball_Sensor', sim.simx_opmode_blocking)[1]

    #position and orientation of proximity sensore wrt UR3 base frame
    T_sensor_in_base = np.array( \
        [[0, 0, -1, end_effector_wrt_base[0]], 
        [0, -1, 0,  end_effector_wrt_base[1]], \
        [-1, 0, 0,  end_effector_wrt_base[2]], 
        [0, 0, 0, 1]])

    #the actual end point of the ball based on path in the world frame
    
    
    path = generate_path()
    end_pt = path[path.shape[0] - 1]
    detectedPoints = np.zeros((2, 3))
    #the position and orientation of the UR3 base frame wrt world frame (T_wb)
    T_base_in_world = np.array([[-1, 0, 0, -1.4001], [0, -1, 0, -0.000086], [0, 0, 1, 0.043], [0, 0, 0, 1]])
    #T_bw
    T_world_in_base = np.linalg.inv(T_base_in_world)
    #end pt coords in homog form
    end_pt_homogenous_coords = np.array([[end_pt[0], end_pt[1], end_pt[2], 1]]).T
    #T_bw * p_w = p_b (4,4) * (4, 1) -> (4, 1), left out final element -> (3, 1)
    #the actual endpt of the ball in the base frame
    end_pt_ball_in_base = np.matmul(T_world_in_base, end_pt_homogenous_coords)[:3]



    detection_thread = threading.Thread(target=utils.calc_ball_position, args=(clientID, sensorHandle, path, detectedPoints))
    motion_thread = threading.Thread(target=utils.move_ball, args=(clientID, ball_handle, path))
    motion_thread.start()
    detection_thread.start()
    detection_thread.join()
    #point_in_sensorX - homogeneous coords of detectedPoints in the frame of the proximity sensor
    point_in_sensor1 = np.array([[detectedPoints[0][0]], [detectedPoints[0][1]], [detectedPoints[0][2]], [1]])
    point_in_sensor2 = np.array([[detectedPoints[1][0]], [detectedPoints[1][1]], [detectedPoints[1][2]], [1]])
    
    #T_bs * p_s = p_b
    point1_ball_in_base = np.matmul(T_sensor_in_base, point_in_sensor1)[:3]
    point2_ball_in_base = np.matmul(T_sensor_in_base, point_in_sensor2)[:3]    
    vector = (1.0*point2_ball_in_base - point1_ball_in_base)

    #convert to unit vector
    vector /= np.linalg.norm(vector)
    t = (end_effector_wrt_base[0] - point2_ball_in_base[0]) / vector[0]
    final_x = end_effector_wrt_base[0]
    final_y = point2_ball_in_base[1] + (t * vector[1])
    final_z = point2_ball_in_base[2] + (t * vector[2])
    predicted_point = np.array([[final_x], [final_y[0]], [final_z[0]]])
    difference = predicted_point - end_pt_ball_in_base
    print("Difference: " + str(difference))
     
    #the predicted direction the ball is moving in, based off of predicted points 1 and 2
    
    #if the difference between detected points is too small
    if (vector[0] == 0):
        vector[0] += 0.000000001

    #find the estimated position of the ball when it is in the yz plane of the UR3

    
    #the predicted point of the ball
    
    print("Predicted Point: " + str(predicted_point))
    print("End Point Ball in Base: " + str(end_pt_ball_in_base))
    #compare prediction to actual end pt of ball
    
    #our initial guess of what the UR3 joint angles should be to get to finalT
    thetas = np.array([0.0, 0.0, 0.0])

    #get a rough initial guess for theta of joint 2
    predicted_y = predicted_point[1]
    predicted_z = predicted_point[2]

    position_joint2 = sim.simxGetObjectPosition(clientID, jointHandles[1], base_handle, sim.simx_opmode_blocking)[1]
    print(position_joint2)
    joint2_y = position_joint2[1]
    joint2_z = position_joint2[2]

    #distance from predicted point to joint2 in yz plane
    d1 = np.sqrt((joint2_y - predicted_y)**2 + (joint2_z - predicted_z)**2)
    z_length = predicted_z - joint2_z
    y_length = predicted_y - joint2_y

    #our thetas to be calculated
    theta2_guess = 0.0
    theta3_guess = 0.0
    theta4_guess = 0.0
    
    #define relevant link lengths
    L1 = 0.244
    L2 = 0.213
    L3 = 0.083

    #if it's in this range, it doesn't make sense to move others
    if d1 < (L1 - L2):
        theta2_guess = np.arctan2(y_length, z_length)
        theta3_guess = 0.0
        theta4_guess = 0.0
    elif d1 <= (L1 + L2 - L3):
        top2 = L2**2 - L1**2 - d1**2
        bot2 = -2*L1*d1
        theta2_guess = np.arctan2(y_length, z_length) - np.arccos(top2/bot2)

        top3 = d1**2 - L1**2 - L2**2
        bot3 = -2*L1*L2
        theta3_guess = np.pi - np.arccos(top3/bot3)

        #fold in on self
        theta4_guess = np.pi/2

    else:
        d2 = d1 - L1
        theta2_guess = np.arctan2(y_length, z_length)

        top3_2 = L3**2 - L2**2 - d2**2
        bot3_2 = -2*d2*L2
        print("top3_2/bot3_2: " + str(top3_2/bot3_2))
        theta3_guess = np.arccos(top3_2/bot3_2)

        top4 = d2**2 - L2**2 - L3**2
        bot4 = -2*L2*L3
        print("top4/bot4: " + str(top4/bot4))
        theta4_guess = np.arccos(top4/bot4) - np.pi


    thetas[0] = theta2_guess
    thetas[1] = theta3_guess
    thetas[2] = theta4_guess
    print("thetas: " + str(thetas))
    
    for i in range(1, 4):
       sim.simxSetJointTargetPosition(clientID, jointHandles[i], thetas[i - 1], sim.simx_opmode_oneshot_wait)


    motion_thread.join()   

    print("Actual End Effector Position " + str(sim.simxGetObjectPosition(clientID, end_effector_handle, base_handle, sim.simx_opmode_blocking)[1]))
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sim.simxFinish(clientID)
    return 1

if __name__ == "__main__":
    main()
