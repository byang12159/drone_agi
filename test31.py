import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from utils import transformations
import cv2
import os

with open("calibrationtest50.pkl",'rb') as handle:
    data = pickle.load(handle)


# img = cv2.imread('calibration_imgs/img0.png')
# cv2.imshow("img",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()


marker_GT = [-0.29408993530273436, -0.07971382904052735, 0.4234147644042969, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.048583984375, 0.0, -0.0160369873046875, -0.0204010009765625, -0.0467987060546875, 0.998565673828125, 1709219631.4415877]

# Drone bot L
drone_GT1 = [1.0519122314453124, -1.3999959716796875, 0.11841184997558593, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0345993041992188, 0.0, -0.154296875, -0.1273956298828125, 0.47806549072265625, 0.855224609375, 1709220287.3616054]
# drone bot R
drone_GT2 = [1.0336710205078126, 1.2452677001953125, 0.10209367370605468, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5389938354492188, 0.0, -0.1080474853515625, -0.16516876220703125, 0.6761093139648438, 0.709869384765625, 1709220345.8642378]

drone_GTs = [drone_GT1,drone_GT2]
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


# for i in range(len(drone_GTs)):
#     drone_GT = drone_GTs[i]
#     xaxis_h = np.array([1*scale,0,0,1])
#     yaxis_h = np.array([0,1*scale,0,1])
#     zaxis_h = np.array([0,0,1*scale,1])

#     R2 = np.array([[0,-1,0,0],
#                 [1,0,0,0],
#                 [0,0,1,0],
#                 [0,0,0,1]])
    
#     R1 = np.array([[1,0,0,0],
#                 [0,-1,0,0],
#                 [0,0,1,0],
#                 [0,0,0,1]])
#     xaxis_h = R2@R1@xaxis_h
#     yaxis_h = R2@R1@yaxis_h
#     zaxis_h = R2@R1@zaxis_h

#     q = drone_GT[11:15] # x, y, z, w
#     T_W2 = transformations.quaternion_matrix(q)
#     T_W2[:3, 3] = drone_GT[:3]

#     drone_center = T_W2[:4,3]
#     drone_tail_x = T_W2@xaxis_h
#     drone_tail_y = T_W2@yaxis_h
#     drone_tail_z = T_W2@zaxis_h

#     ax.plot(drone_GT[0],drone_GT[1],drone_GT[2],'x',color='purple',label=f'drone{i} location')
#     ax.plot([drone_center[0],drone_tail_x[0]],[drone_center[1],drone_tail_x[1]],[drone_center[2],drone_tail_x[2]],color='red')
#     ax.plot([drone_center[0],drone_tail_y[0]],[drone_center[1],drone_tail_y[1]],[drone_center[2],drone_tail_y[2]],color='green')
#     ax.plot([drone_center[0],drone_tail_z[0]],[drone_center[1],drone_tail_z[1]],[drone_center[2],drone_tail_z[2]],color='blue')
for j in range(len(data)):
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.plot(0,0,0, 'x',color='red',label="World Center")
    ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='green',label='marker location')

    scale = 0.5
    xaxis_h = np.array([1*scale,0,0,1])
    yaxis_h = np.array([0,1*scale,0,1])
    zaxis_h = np.array([0,0,1*scale,1])

    marker_center = T_WM[:4,3]
    marker_tail_x = T_WM@xaxis_h
    marker_tail_y = T_WM@yaxis_h
    marker_tail_z = T_WM@zaxis_h

    ax.plot([marker_center[0],marker_tail_x[0]],[marker_center[1],marker_tail_x[1]],[marker_center[2],marker_tail_x[2]],color='red')
    ax.plot([marker_center[0],marker_tail_y[0]],[marker_center[1],marker_tail_y[1]],[marker_center[2],marker_tail_y[2]],color='green')
    ax.plot([marker_center[0],marker_tail_z[0]],[marker_center[1],marker_tail_z[1]],[marker_center[2],marker_tail_z[2]],color='blue')

    snapstate = data[j][2]
    # print(snapstate)
    ax.plot(snapstate[0],snapstate[1],snapstate[2],'o',color='green')

    Ts= data[j][3][0][0]
    
    # Unit vectors along axes
    xaxis = np.array([1, 0, 0])
    yaxis = np.array([0, 1, 0])
    zaxis = np.array([0, 0, 1])

    q = marker_GT[11:15] # x, y, z, w
    T_WM = transformations.quaternion_matrix(q)
    T_WM[:3, 3] = marker_GT[:3]

    scale = 1
    ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green',label='Drone Ground Truth')
    ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='cyan',label='Marker Ground Truth')
    ########### PLOT for Marker Poses ###########
    xaxis_h = np.array([1,0,0,1])
    yaxis_h = np.array([0,1,0,1])
    zaxis_h = np.array([0,0,1,1])

    marker_center = T_WM[:4,3]
    marker_tail_x = T_WM@xaxis_h
    marker_tail_y = T_WM@yaxis_h
    marker_tail_z = T_WM@zaxis_h

    ax.plot([marker_center[0],marker_tail_x[0]*scale],[marker_center[1],marker_tail_x[1]*scale],[marker_center[2],marker_tail_x[2]*scale],color='red')
    ax.plot([marker_center[0],marker_tail_y[0]*scale],[marker_center[1],marker_tail_y[1]*scale],[marker_center[2],marker_tail_y[2]*scale],color='green')
    ax.plot([marker_center[0],marker_tail_z[0]*scale],[marker_center[1],marker_tail_z[1]*scale],[marker_center[2],marker_tail_z[2]*scale],color='blue')


    # ########### PLOT for Camera Poses ###########
    translation = Ts[:3,3]
    rotation = np.linalg.inv(Ts[:3,:3])
    Ts_inv = np.linalg.inv(Ts)

    aruco_head =   T_WM@Ts_inv.dot(np.array([0,0,0,1]))
    aruco_tail_x = T_WM@Ts_inv.dot(xaxis_h*scale)
    aruco_tail_y = T_WM@Ts_inv.dot(yaxis_h*scale)
    aruco_tail_z = T_WM@Ts_inv.dot(zaxis_h*scale)
    # aruco_head =   Ts@T_WM.dot(np.array([0,0,0,1]))
    # aruco_tail_x = Ts@T_WM.dot(xaxis_h*scale)
    # aruco_tail_y = Ts@T_WM.dot(yaxis_h*scale)
    # aruco_tail_z = Ts@T_WM.dot(zaxis_h*scale)

    # print("one",T_WM@Ts_inv)
    # print("two",T_WM.dot(Ts_inv))

    ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red')
    ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green')
    ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue')


    ########### PLOT for Drone Poses ###########
    T_WD = transformations.quaternion_matrix(snapstate[11:15])
    T_WD[:3,3] = snapstate[:3]
    drone_axes_scale = 1

    drone_head = T_WD[:3,3]
    drone_axes_tip_x = T_WD@xaxis_h
    drone_axes_tip_y = T_WD@yaxis_h
    drone_axes_tip_z = T_WD@zaxis_h

    # ax.plot(drone_head[0],drone_head[1],drone_head[2],'x',color='green')
    ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
    ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
    ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')

  
      

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    # plt.legend()
    plt.axis('equal')
    plt.show()

# fig = plt.figure()
# ax = fig.add_subplot(111,projection='3d')
# for j in range(len(data)):
#     snapstate = data[j][2]
#     print(snapstate)
#     ax.plot(snapstate[0],snapstate[1],snapstate[2],'o',color='green')

# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# plt.legend()
# plt.axis('equal')
# plt.show()
# # scale = 1
# # ax.plot([0,xaxis[0]*scale],[0,xaxis[1]*scale],[0,xaxis[2]*scale],color='red')
# # ax.plot([0,yaxis[0]*scale],[0,yaxis[1]*scale],[0,yaxis[2]*scale],color='green')
# # ax.plot([0,zaxis[0]*scale],[0,zaxis[1]*scale],[0,zaxis[2]*scale],color='blue')


# # Ts = np.array([[ 0.91056476, -0.40396361, -0.08766537, -0.00262141],
# #        [-0.15009306, -0.12550218, -0.98067389,  0.09899701],
# #        [ 0.38515437,  0.90612505, -0.17491   ,  2.06689062],
# #        [ 0.        ,  0.        ,  0.        ,  1.        ]])
# # Ts_inv = np.linalg.inv(Ts)
# # aruco_head =   T_WM@R2@Ts_inv.dot(np.array([0,0,0,1]))
# # aruco_tail_x = T_WM@R2@Ts_inv.dot(xaxis_h)
# # aruco_tail_y = T_WM@R2@Ts_inv.dot(yaxis_h)
# # aruco_tail_z = T_WM@R2@Ts_inv.dot(zaxis_h)

# # ax.plot(aruco_head[0],aruco_head[1],aruco_head[2],'x',color='red')
# # ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red',linewidth = 0.5)
# # ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green',linewidth = 0.5)
# # ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue',linewidth = 0.5)

