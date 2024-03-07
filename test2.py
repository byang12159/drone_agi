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

def rotz(angle):
    return np.array([[np.cos(angle),-np.sin(angle),0,0],
                    [np.sin(angle),np.cos(angle),0,0],
                    [0,0,1,0],
                    [0,0,0,1]])

def rotx(angle):
    return np.array([[1,0,0,0],
                    [0,np.cos(angle),-np.sin(angle),0],
                    [0,np.sin(angle),np.cos(angle),0],
                    [0,0,0,1]])
    

def rotz3(angle):
    return np.array([[np.cos(angle),-np.sin(angle),0],
                    [np.sin(angle),np.cos(angle),0],
                    [0,0,1]])

def roty3(angle):
    return np.array([[np.cos(angle),0,np.sin(angle)],
                     [0,1,0],
                    [-np.sin(angle),0,np.cos(angle)]])
parser = argparse.ArgumentParser(description="")
parser.add_argument('--filename', type=str, default="datacollect_B_1.pkl", help='pickle filename to import data')
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
            chaser_GT = data[0][1]
            q = chaser_GT[11:15] # x, y, z, w
            T_WC = transformations.quaternion_matrix(q)
            T_WC[:3, 3] = chaser_GT[:3]
            pose_W.append(T_WC)


# img = cv2.imread('calibration_imgs/img0.png')
# cv2.imshow("img",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

chaser_GT = data[0][1]
chaser_GT = [2.108356201171875, 0.814648681640625, 0.6669138793945313, -0.0004423944652080536, 1.543397270143032e-05, 0.001357092261314392, -0.014364360809326172, 0.00015283389389514923, 0.04490167999267578, 1.9195709228515625, -0.002685546875, 0.03118133544921875, -0.01715850830078125, 0.8185806274414062, 0.5732955932617188, 1709766963.8105612]

totaldiff = []
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
# xaxis_h = rotz(np.pi/2) @np.array([1*scale,0,0,1])
# yaxis_h = rotz(np.pi/2) @np.array([0,1*scale,0,1])
# zaxis_h = rotz(np.pi/2) @np.array([0,0,1*scale,1])

# # Unit vectors along axes
# xaxis = rotz3(np.pi/2) @np.array([1, 0, 0])
# yaxis = rotz3(np.pi/2) @np.array([0, 1, 0])
# zaxis = rotz3(np.pi/2) @np.array([0, 0, 1])

q = chaser_GT[11:15] # x, y, z, w
T_WC = transformations.quaternion_matrix(q)
T_WC[:3, 3] = chaser_GT[:3]

distance_all = []
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.plot(0,0,0, 'x',color='red',label="cam center")
#### PLOT IN WORLD COORDINATE ####
for j in range(len(pose_C)):



    T_WD_unit = T_WD[j]
    # leaddronevicon =[0.08403057098388672, -0.40664501953125, 0.05818780517578125, 0.00010395781695842743, 0.00019048668444156646, -1.051737181842327e-05, 0.010276329994201661, 0.0018479542732238769, -0.004929679870605469, 3.1351547241210938, -0.0021820068359375, -0.09761810302734375, 0.0123748779296875, -0.00211334228515625, 0.995147705078125, 1709767258.1344569]
    # q = leaddronevicon[11:15] # x, y, z, w
    # T_WD_unit = transformations.quaternion_matrix(q)
    # T_WD_unit[:3, 3] = leaddronevicon[:3]

    pose_C_unit = pose_C[j]

    ax.plot([0,xaxis[0]*scale],[0,xaxis[1]*scale],[0,xaxis[2]*scale],color='red')
    ax.plot([0,yaxis[0]*scale],[0,yaxis[1]*scale],[0,yaxis[2]*scale],color='green')
    ax.plot([0,zaxis[0]*scale],[0,zaxis[1]*scale],[0,zaxis[2]*scale],color='blue')

    drone_diff = chaser_GT[:3]-T_WD_unit[:3,3]
#     print("dronediff",drone_diff)
    drone_chase_center = np.array([drone_diff[2],-drone_diff[1],drone_diff[0]])
    drone_chase_tail_x = T_WC@xaxis_h
    drone_chase_tail_y = T_WC@yaxis_h
    drone_chase_tail_z = T_WC@zaxis_h

#     ax.plot(drone_chase_center[0],drone_chase_center[1],drone_chase_center[2],'*',color='cyan', label='Leader Drone')
#     ax.plot([drone_chase_center[0],drone_chase_tail_x[0]],[drone_chase_center[1],drone_chase_tail_x[1]],[drone_chase_center[2],drone_chase_tail_x[2]],color='red')
#     ax.plot([drone_chase_center[0],drone_chase_tail_y[0]],[drone_chase_center[1],drone_chase_tail_y[1]],[drone_chase_center[2],drone_chase_tail_y[2]],color='green')
#     ax.plot([drone_chase_center[0],drone_chase_tail_z[0]],[drone_chase_center[1],drone_chase_tail_z[1]],[drone_chase_center[2],drone_chase_tail_z[2]],color='blue')

    lead_drone_center = T_WD_unit[:4,3]
    lead_drone_tail_x = T_WD_unit@xaxis_h
    lead_drone_tail_y = T_WD_unit@yaxis_h
    lead_drone_tail_z = T_WD_unit@zaxis_h

    # Lead Drone
#     ax.plot(lead_drone_center[0],lead_drone_center[1],lead_drone_center[2],'x',color='purple',label=f'lead drone{j} location')
#     ax.plot([lead_drone_center[0],lead_drone_tail_x[0]],[lead_drone_center[1],lead_drone_tail_x[1]],[lead_drone_center[2],lead_drone_tail_x[2]],color='red')
#     ax.plot([lead_drone_center[0],lead_drone_tail_y[0]],[lead_drone_center[1],lead_drone_tail_y[1]],[lead_drone_center[2],lead_drone_tail_y[2]],color='green')
#     ax.plot([lead_drone_center[0],lead_drone_tail_z[0]],[lead_drone_center[1],lead_drone_tail_z[1]],[lead_drone_center[2],lead_drone_tail_z[2]],color='blue')

    # Estimated position using camera
    T_DC = rotx(np.pi/2)
    T_DC[:3,3] = -np.array([-0.01089176, -0.01491055,  0.04597474])
    # T_DC = np.identity(4)
    T_ML = np.identity(4)
    T_ML[0,3] = -0.2

    aruco_head =   pose_C_unit.dot(np.array([0,0,0,1]))
    aruco_tail_x = pose_C_unit.dot(xaxis_h)
    aruco_tail_y = pose_C_unit.dot(yaxis_h)
    aruco_tail_z = pose_C_unit.dot(zaxis_h)

    difference = np.linalg.norm(aruco_head[:3]-drone_chase_center)

    
    if difference < 0.5:
       print("difference",aruco_head[:3]-drone_chase_center)
       totaldiff.append(aruco_head[:3]-drone_chase_center)
       # ax.plot(drone_chase_center[0],drone_chase_center[1],drone_chase_center[2],'*',color='cyan', label='Leader Drone')
       ax.plot(aruco_head[0],aruco_head[1],aruco_head[2],'x',color='teal',label="Camera Estimate")

       ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red',linewidth = 0.5)
       ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green',linewidth = 0.5)
       ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue',linewidth = 0.5)

        
    # aruco_head =   T_WC@T_DC@pose_C_unit@np.array([0,0,0,1])
    # aruco_tail_x = T_WC@T_DC@pose_C_unit@(xaxis_h)
    # aruco_tail_y = T_WC@T_DC@pose_C_unit@(yaxis_h)
    # aruco_tail_z = T_WC@T_DC@pose_C_unit@(zaxis_h)
#     print("cam esti",aruco_head)
    
    euler_marker = transformations.euler_from_matrix(Ts)
    euler_marker = np.round(euler_marker,2)
    euler_marker = euler_marker * 180 / np.pi
    euler_marker = np.round(euler_marker,2)

    distance_drone2marker = np.linalg.norm(T_WD_unit[:3,3]-aruco_head[:3])
    distance_all.append(distance_drone2marker)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
# plt.legend()
plt.axis('equal')
plt.title(f"euler angle {euler_marker}")
plt.show()

    
totaldiff = np.array(totaldiff)
print("avgx",np.mean(totaldiff[:,0]))
print("avgy",np.mean(totaldiff[:,1]))
print("avgz",np.mean(totaldiff[:,2]))