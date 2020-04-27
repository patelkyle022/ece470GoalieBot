import numpy as np
from scipy.linalg import expm
from sim import simxGetObjectPosition
#lengths of each link
L1 = 0.152
L2 = 0.120
L3 = 0.244
L4 = 0.093
L5 = 0.213
L6 = 0.083
L7 = 0.083
L8 = 0.082
L9 = 0.0535
L10 = 0.059

def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

def Get_MS():
    S = np.zeros((6,6))
    M = np.array([[0, -1, 0, 0.39], [0, 0, -1, 0.401], [1, 0, 0, 0.2155], [0, 0, 0, 1]]) #pose of end-effector in world frame 
    w = np.array([[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0]]) #unit angular velocity vectors
    q = np.array([[-0.15, 0.15, 0.01],[-0.15, 0.27, 0.162],[0.094, 0.27, 0.162],\
             [0.307, 0.177, 0.162],[0.307, 0.26, 0.162],[0.39, 0.26, 0.162]]) #joint center locations with respect to real measurements from the world frame origin
    v = np.zeros((6,3))
    for i in range(6):
        v[i] = np.cross(-w[i], q[i])
        S[i] = np.append(w[i], v[i])
    
    return M, S

def get_T(thetas, M, S):
    w_arr = S[:, :3] 
    v_arr = S[:, 3:]
    wb_arr = np.array([skew(w_arr[i]) for i in range(6)])
    sb_arr = np.zeros((6, 4, 4))
    e_sb_arr = np.zeros((6, 4, 4))
    T = np.eye(4)
    for i in range(6):
        sb_arr[i, :3, :3] = wb_arr[i]
        sb_arr[i, :3, 3:] = np.array([v_arr[i]]).T
        sb_arr[i] *= thetas[i]
        e_sb_arr[i] = expm(sb_arr[i]) 
    for i in range(6):
        T = T @ e_sb_arr[i]
    
    T = T @ M
    return T

def IK(point_w, yaw):
    thetas = np.zeros(6)
    xgrip = point_w[0] + 1.4001 #in base frame 
    ygrip = point_w[1] #in base frame
    zgrip = point_w[2] #in base frame
    xcen_0 = L3 + L5 + L7 #xcen when theta1 is 0 in base frame 
    ycen_0 = 0 #ycen when theta1 is 0 (shaft is parallel to x-axis) in base frame
    xcen = xgrip
    ycen = ygrip
    zcen = zgrip
    #alpha: the offset angle between the end-effector center and the ray passing through x3end, y3end in the xy plane 
    thetas[0] = np.arctan2(ycen, xcen)
    thetas[5] = thetas[0] + 0.5 * np.pi - yaw 
    beta = np.arcsin(ycen / np.sqrt(xcen ** 2 + ycen ** 2)) - thetas[0] 
    D = np.sqrt(xcen ** 2 + ycen ** 2) * np.cos(beta) - L7 #cases where D is negative aren't really in play because of UR3 angle restrictions
    x3end = D*np.cos(thetas[0]) 
    y3end = D*np.sin(thetas[0])
    z3end = zcen + L8 + L10

    d = z3end - L1
    R = np.sqrt(x3end ** 2 + y3end ** 2 +  d ** 2)
    gamma = np.arcsin(d / R) 
    delta = np.arccos((R ** 2 + L3 ** 2 - L5 ** 2) / (2 * L3 * R)) 
    epsilon = np.arccos((L3 ** 2 + L5 ** 2 - R ** 2) / (2 * L3 * L5))
    thetas[1] = -gamma - delta
    thetas[2] = np.pi - epsilon 
    thetas[3] = -thetas[1] - thetas[2]  
    thetas[4] = -0.5 * np.pi
    return thetas



points = np.array([[0.25, 0.25, 0.1], [0.2, 0.05, 0.15], [0.4, 0.05, 0.15], \
                 [0.4, 0.25, 0.15], [0.35, 0.3, 0.1], [0.15, -0.1, 0.25], [0.2, 0.4, 0.05]])
yaw = np.radians([0, 25, 0, 15, -10, -45, 45])
thetas = np.array([IK(points[i], yaw[i]) for i in range(list(points.shape)[0])])
print(np.degrees(thetas))
M, S = Get_MS()
print(np.round(np.array([get_T(thetas[i], M, S) for i in range(list(points.shape)[0])]), 3))
