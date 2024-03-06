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
parser.add_argument('--filename', type=str, default="datacollect3.pkl", help='pickle filename to import data')
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
            pose_C.append(Ts)

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
marker_GT = [2.33148876953125, 1.1829923095703125, 0.6608572998046875, -8.76462683081627e-05, -0.0005406544804573059, 0.00033645230531692506, -0.007194867610931396, -0.015786824226379396, 0.008081820487976074, 2.379852294921875, -0.00091552734375, 0.00327301025390625, -0.01073455810546875, 0.9282073974609375, 0.3719024658203125, 1709741184.374893]

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

    drone_chase_center = T_WM[:4,3]
    drone_chase_tail_x = T_WM@xaxis_h
    drone_chase_tail_y = T_WM@yaxis_h
    drone_chase_tail_z = T_WM@zaxis_h

    ax.plot(drone_chase_center[0],drone_chase_center[1],drone_chase_center[2],'x',color='red', label='Chaser Drone')
    ax.plot([drone_chase_center[0],drone_chase_tail_x[0]],[drone_chase_center[1],drone_chase_tail_x[1]],[drone_chase_center[2],drone_chase_tail_x[2]],color='red')
    ax.plot([drone_chase_center[0],drone_chase_tail_y[0]],[drone_chase_center[1],drone_chase_tail_y[1]],[drone_chase_center[2],drone_chase_tail_y[2]],color='green')
    ax.plot([drone_chase_center[0],drone_chase_tail_z[0]],[drone_chase_center[1],drone_chase_tail_z[1]],[drone_chase_center[2],drone_chase_tail_z[2]],color='blue')

    # R2 = np.array([[0,-1,0,0],
    #             [1,0,0,0],
    #             [0,0,1,0],
    #             [0,0,0,1]])

    # R1 = np.array([[1,0,0,0],
    #             [0,-1,0,0],
    #             [0,0,1,0],
    #             [0,0,0,1]])
    
    R2 = np.array([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,1]])

    R1 = np.array([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,1]])
    xaxis_h = R2@R1@xaxis_h
    yaxis_h = R2@R1@yaxis_h
    zaxis_h = R2@R1@zaxis_h

    lead_drone_center = T_WD_unit[:4,3]
    lead_drone_tail_x = T_WD_unit@xaxis_h
    lead_drone_tail_y = T_WD_unit@yaxis_h
    lead_drone_tail_z = T_WD_unit@zaxis_h

    # Lead Drone
    ax.plot(lead_drone_center[0],lead_drone_center[1],lead_drone_center[2],'x',color='purple',label=f'drone{j} location')
    ax.plot([lead_drone_center[0],lead_drone_tail_x[0]],[lead_drone_center[1],lead_drone_tail_x[1]],[lead_drone_center[2],lead_drone_tail_x[2]],color='red')
    ax.plot([lead_drone_center[0],lead_drone_tail_y[0]],[lead_drone_center[1],lead_drone_tail_y[1]],[lead_drone_center[2],lead_drone_tail_y[2]],color='green')
    ax.plot([lead_drone_center[0],lead_drone_tail_z[0]],[lead_drone_center[1],lead_drone_tail_z[1]],[lead_drone_center[2],lead_drone_tail_z[2]],color='blue')

    def rotx(angle):
        return np.array([[1,0,0,0],
                         [0,np.cos(angle),-np.sin(angle),0],
                         [0,np.sin(angle),np.cos(angle),0],
                        [0,0,0,1]])
    

    # T_DC = np.identity(4)
    T_DC = rotx(np.pi/2)
    aruco_head =   T_WD_unit@T_DC@pose_C_unit.dot(np.array([0,0,0,1]))
    aruco_tail_x = T_WD_unit@T_DC@pose_C_unit.dot(xaxis_h)
    aruco_tail_y = T_WD_unit@T_DC@pose_C_unit.dot(yaxis_h)
    aruco_tail_z = T_WD_unit@T_DC@pose_C_unit.dot(zaxis_h)

    # ax.plot(aruco_head[0],aruco_head[1],aruco_head[2],'x',color='teal',label="Camera Estimate")
    # ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red',linewidth = 0.5)
    # ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green',linewidth = 0.5)
    # ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue',linewidth = 0.5)

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
