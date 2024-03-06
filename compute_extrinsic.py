# Visualization script in matplotlib for stationary aruco marker and moving UAV vicon data

import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from utils import transformations
import cv2
import os
from scipy.optimize import least_squares
from picam.transformation_properties_drone import get_T_DC
import argparse

parser = argparse.ArgumentParser(description="")
parser.add_argument('--filename', type=str, default="calibrationtest50.pkl", help='pickle filename to import data')
parser.add_argument('--outlier', type=float, default=1.0, help='cutoff for filtering before loss computation')
parser.add_argument('--targetID', type=int, default=0, help='aruco marker ID to detect')

args = parser.parse_args()

with open(args.filename,'rb') as handle:
    data = pickle.load(handle)

pose_C = []
pose_W = []
T_WD = []

target_ID = args.targetID

######################################################## PART I: INPECT COORDINATES ########################################################

for j in range(len(data)):
    for a in range(len(data[j][3][1])):
        if data[j][3][1][a] == target_ID:

            snapstate = data[j][2]
            T_WD_j = transformations.quaternion_matrix(snapstate[11:15])
            T_WD_j[:3,3] = snapstate[:3]
            T_WD.append(T_WD_j)

            Ts = data[j][3][0][a]
            Ts_inv = np.linalg.inv(Ts)
            pose_C.append(Ts_inv)

            # Just constant 
            marker_GT = data[0][1]
            q = marker_GT[11:15] # x, y, z, w
            T_WM = transformations.quaternion_matrix(q)
            T_WM[:3, 3] = marker_GT[:3]
            pose_W.append(T_WM)


# img = cv2.imread('calibration_imgs/img0.png')
# cv2.imshow("img",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

marker_GT = data[0][1]

spotted_count = []
Ts_full = []
snapstate_full = []
target_ID = 0

scale = 0.4
xaxis_h = np.array([1*scale,0,0,1])
yaxis_h = np.array([0,1*scale,0,1])
zaxis_h = np.array([0,0,1*scale,1])

# Unit vectors along axes
xaxis = np.array([1, 0, 0])
yaxis = np.array([0, 1, 0])
zaxis = np.array([0, 0, 1])

q = marker_GT[11:15] # x, y, z, w
T_WM = transformations.quaternion_matrix(q)
T_WM[:3, 3] = marker_GT[:3]

distance_all = []

#### PLOT IN WORLD COORDINATE ####
for j in range(len(pose_C)):
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    #ax.plot(0,0,0, 'x',color='red',label="world center")
    #ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='green',label='marker location')

    T_WD_unit = T_WD[j]
    pose_C_unit = pose_C[j]

    scale = 0.5
    xaxis_h = np.array([1*scale,0,0,1])
    yaxis_h = np.array([0,1*scale,0,1])
    zaxis_h = np.array([0,0,1*scale,1])

    ax.plot([0,xaxis[0]*scale],[0,xaxis[1]*scale],[0,xaxis[2]*scale],color='red')
    ax.plot([0,yaxis[0]*scale],[0,yaxis[1]*scale],[0,yaxis[2]*scale],color='green')
    ax.plot([0,zaxis[0]*scale],[0,zaxis[1]*scale],[0,zaxis[2]*scale],color='blue')

    marker_center = T_WM[:4,3]
    marker_tail_x = T_WM@xaxis_h
    marker_tail_y = T_WM@yaxis_h
    marker_tail_z = T_WM@zaxis_h

    ax.plot([marker_center[0],marker_tail_x[0]],[marker_center[1],marker_tail_x[1]],[marker_center[2],marker_tail_x[2]],color='red')
    ax.plot([marker_center[0],marker_tail_y[0]],[marker_center[1],marker_tail_y[1]],[marker_center[2],marker_tail_y[2]],color='green')
    ax.plot([marker_center[0],marker_tail_z[0]],[marker_center[1],marker_tail_z[1]],[marker_center[2],marker_tail_z[2]],color='blue')

    R2 = np.array([[0,-1,0,0],
                [1,0,0,0],
                [0,0,1,0],
                [0,0,0,1]])

    R1 = np.array([[1,0,0,0],
                [0,-1,0,0],
                [0,0,1,0],
                [0,0,0,1]])
    
    xaxis_h = R2@R1@xaxis_h
    yaxis_h = R2@R1@yaxis_h
    zaxis_h = R2@R1@zaxis_h

    drone_center = T_WD_unit[:4,3]
    drone_tail_x = T_WD_unit@xaxis_h
    drone_tail_y = T_WD_unit@yaxis_h
    drone_tail_z = T_WD_unit@zaxis_h

    #ax.plot(T_WD_unit[0][3],T_WD_unit[1][3],T_WD_unit[2][3],'x',color='purple',label=f'drone{j} location')
    ax.plot([drone_center[0],drone_tail_x[0]],[drone_center[1],drone_tail_x[1]],[drone_center[2],drone_tail_x[2]],color='red')
    ax.plot([drone_center[0],drone_tail_y[0]],[drone_center[1],drone_tail_y[1]],[drone_center[2],drone_tail_y[2]],color='green')
    ax.plot([drone_center[0],drone_tail_z[0]],[drone_center[1],drone_tail_z[1]],[drone_center[2],drone_tail_z[2]],color='blue')

    aruco_head =   T_WM@R2@pose_C_unit.dot(np.array([0,0,0,1]))
    aruco_tail_x = T_WM@R2@pose_C_unit.dot(xaxis_h)
    aruco_tail_y = T_WM@R2@pose_C_unit.dot(yaxis_h)
    aruco_tail_z = T_WM@R2@pose_C_unit.dot(zaxis_h)

    #ax.plot(aruco_head[0],aruco_head[1],aruco_head[2],'x',color='teal',label="Camera Estimate")
    ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red',linewidth = 0.5)
    ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green',linewidth = 0.5)
    ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue',linewidth = 0.5)

    euler_marker = transformations.euler_from_matrix(Ts)
    euler_marker = np.round(euler_marker,2)
    euler_marker = euler_marker * 180 / np.pi
    euler_marker = np.round(euler_marker,2)

    distance_drone2marker = np.linalg.norm(T_WD_unit[:3,3]-aruco_head[:3])
    distance_all.append(distance_drone2marker)

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend()
    plt.axis('equal')
    plt.title(f"euler angle {euler_marker}")
    plt.show()

plt.hist(distance_all)
plt.title("Outlier: Distance between Drone GT and Estimated Drone Cam Location")
plt.show() 

    
######################################################## PART II: COMPUTE TRANSFORMATION ########################################################
pose_C = np.array(pose_C) # N x 4 x 4
pose_W = np.array(pose_W) # N x 4 x 4
T_WD = np.array(T_WD) # N x 4 x 4

idx = np.where(np.array(distance_all)<args.outlier)[0]
print('# raw samples: %d, # filtered samples: %d, # removed outliers: %d'%(pose_W.shape[0], len(idx), pose_W.shape[0]-len(idx) ))
pose_C = pose_C[idx,:,:]
pose_W = pose_W[idx,:,:]
T_WD = T_WD[idx,:,:]

def loss(_T_DC, verbose=False):
    q = _T_DC[:4]
    t = _T_DC[4:]
    T_DC = transformations.quaternion_matrix(q)
    T_DC[:3, 3] = t
    T_DC = T_DC.reshape(1, 4, 4)
    pose_W_cal = np.matmul(T_WD, np.matmul(T_DC, pose_C))
    position_W = pose_W_cal[:, :3, 3]
    if verbose:
        return position_W
    return ((position_W - pose_W[:, :3, 3])**2).sum()

x0 = np.zeros(7)
x0[:4] = transformations.quaternion_from_matrix(get_T_DC())
x0[4:] = get_T_DC()[:3, 3]

# import ipdb; ipdb.set_trace()

print(f"Starting loss: {loss(x0)}")
res = least_squares(loss, x0)
print(f"Ending Loss {loss(res.x)}")
q=res.x[:4]
t=res.x[4:]
euler = transformations.euler_from_quaternion(q, 'syxz')

print(res.x.tolist())
# print(res.cost)
# print(res.optimality)