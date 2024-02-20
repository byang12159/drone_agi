import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from utils import transformations


with open("calibration_data_test.p",'rb') as handle:
    data = pickle.load(handle)

# n = 300
# print(len(data[n]))
# print(data[n][0])
# print(data[n][1])
# print("1")
# print(data[n][2])
# print("2")
# print(data[n][2][0][0])

marker_GT = data[0][0]
q = marker_GT[11:15] # x, y, z, w
T_WM = transformations.quaternion_matrix(q)
T_WM[:3, 3] = marker_GT[:3]
print(T_WM)

drone_x = []
drone_y = []
drone_z = []

idx_spotted = []
# marker_TS = []


for i,dat in enumerate(data):
    x,y,z = dat[1][:3]
    drone_x.append(x)
    drone_y.append(y)
    drone_z.append(z)

    if len(dat[2][0]) > 0:
        idx_spotted.append(i)
        # marker_TS.append(dat[2][0][0])



for idx in idx_spotted:

    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    Ts = data[idx][2][0][0]
    snapstate=data[idx][1]

    # Unit vectors along axes
    xaxis = np.array([1, 0, 0])
    yaxis = np.array([0, 1, 0])
    zaxis = np.array([0, 0, 1])

    q = marker_GT[11:15] # x, y, z, w
    T_WM = transformations.quaternion_matrix(q)
    T_WM[:3, 3] = marker_GT[:3]

    translation = Ts[:3,3]

    #backward
    rotation = np.linalg.inv(Ts[:3,:3])
    scale = 1
    aruco_head = rotation.dot(np.array([0,0,0])*scale - translation)
    aruco_tail_x = rotation.dot(xaxis*scale - translation)
    aruco_tail_y = rotation.dot(yaxis*scale - translation)
    aruco_tail_z = rotation.dot(zaxis*scale - translation)


    ###################### PLOT for Marker Poses ######################
    ax.plot([0,xaxis[0]*scale],[0,xaxis[1]*scale],[0,xaxis[2]*scale],color='red')
    ax.plot([0,yaxis[0]*scale],[0,yaxis[1]*scale],[0,yaxis[2]*scale],color='green')
    ax.plot([0,zaxis[0]*scale],[0,zaxis[1]*scale],[0,zaxis[2]*scale],color='blue')

    xaxis_h = np.array([1,0,0,1])
    yaxis_h = np.array([0,1,0,1])
    zaxis_h = np.array([0,0,1,1])

    marker_center = T_WM@np.array([0,0,0,1])
    marker_tail_x = T_WM@xaxis_h
    marker_tail_y = T_WM@yaxis_h
    marker_tail_z = T_WM@zaxis_h

    ###################### PLOT for Camera Poses ######################
    aruco_head = T_WM@np.array([aruco_head[0],aruco_head[1],aruco_head[2],1])
    aruco_tail_x = T_WM@np.array([aruco_tail_x[0],aruco_tail_x[1],aruco_tail_x[2],1])
    aruco_tail_y = T_WM@np.array([aruco_tail_y[0],aruco_tail_y[1],aruco_tail_y[2],1])
    aruco_tail_z = T_WM@np.array([aruco_tail_z[0],aruco_tail_z[1],aruco_tail_z[2],1])
    ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red')
    ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green')
    ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue')


    ###################### PLOT for Drone Poses ######################
    T_WD = transformations.quaternion_matrix(snapstate[11:15])
    T_WD[:3,3] = snapstate[:3]
    drone_axes_scale = 1

    drone_head = T_WD[:3,3]
    drone_axes_tip_x = T_WD@xaxis_h
    drone_axes_tip_y = T_WD@yaxis_h
    drone_axes_tip_z = T_WD@zaxis_h

    ax.plot(drone_head[0],drone_head[1],drone_head[2],'x',color='green')
    ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
    ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
    ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend()
    plt.axis('equal')
    print("marker estimate at position ",aruco_head[0],aruco_head[1],aruco_head[2])
    plt.show()


print("difference",drone_head-aruco_head)