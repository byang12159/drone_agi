import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pickle
#from utils import transformations
def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.
    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True
    """
    import numpy
    import math
    q = numpy.array(quaternion[:4], dtype=numpy.float64, copy=True)
    nq = numpy.dot(q, q)
    _EPS = numpy.finfo(float).eps * 4.0
    if nq < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = numpy.outer(q, q)
    return numpy.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=numpy.float64)


with open("calibration_data_test_circle2.p",'rb') as handle:
    data = pickle.load(handle)

marker_GT = data[0][0]
idx_spotted = []
Ts_all = []
snapstate_all = []
target_ID = 0

xaxis_h = np.array([1,0,0,1])
yaxis_h = np.array([0,1,0,1])
zaxis_h = np.array([0,0,1,1])

for j in range(len(data)):
    if len(data[j][2][0]) != 0:
        for a in range(len(data[j][2][1])):
            if data[j][2][1][a] == target_ID:
                Ts_j = data[j][2][0][a]
                # euler_Ts_j = transformations.euler_from_matrix(Ts_j)
                # if euler_Ts_j[0] >= 0:
                idx_spotted.append(j)
                Ts_all.append(data[j][2][0][a])
                snapstate_all.append(data[j][1])

marker_GT  = data[0][0]

snapstate_all = np.array(snapstate_all)


############################ WORLD COORD ############################
########################################################
########################################################
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.plot(0,0,0, 'x',color='red',label="Vicon Zero")
ax.plot(snapstate_all[:,0],snapstate_all[:,1],snapstate_all[:,2],'*')
# Unit vectors along axes
xaxis = np.array([1, 0, 0])
yaxis = np.array([0, 1, 0])
zaxis = np.array([0, 0, 1])
q = marker_GT[11:15] # x, y, z, w
T_WM = quaternion_matrix(q)
T_WM[:3, 3] = marker_GT[:3]
# ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green',label='Drone Ground Truth')
ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='red',label='Marker Ground Truth')
ax.plot([0,xaxis[0]],[0,xaxis[1]],[0,xaxis[2]],color='red')
ax.plot([0,yaxis[0]],[0,yaxis[1]],[0,yaxis[2]],color='green')
ax.plot([0,zaxis[0]],[0,zaxis[1]],[0,zaxis[2]],color='blue')


for i in range(len(snapstate_all)):
    snapstate = snapstate_all[i]
    Ts = Ts_all[i]
    # Unit vectors along axes
    xaxis = np.array([1, 0, 0])
    yaxis = np.array([0, 1, 0])
    zaxis = np.array([0, 0, 1])
    q = marker_GT[11:15] # x, y, z, w
    T_WM = quaternion_matrix(q)
    T_WM[:3, 3] = marker_GT[:3]
    scale = 1
    ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green',label='Drone Ground Truth')
    ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='cyan',label='Marker Ground Truth')
    ax.plot([0,xaxis[0]],[0,xaxis[1]],[0,xaxis[2]],color='red')
    ax.plot([0,yaxis[0]],[0,yaxis[1]],[0,yaxis[2]],color='green')
    ax.plot([0,zaxis[0]],[0,zaxis[1]],[0,zaxis[2]],color='blue')

    # ########### PLOT for Camera Poses ###########
    Ts_inv = np.linalg.inv(Ts)
    aruco_head =   T_WM@Ts_inv.dot(np.array([0,0,0,1]))
    aruco_tail_x = T_WM@Ts_inv.dot(xaxis_h)
    aruco_tail_y = T_WM@Ts_inv.dot(yaxis_h)
    aruco_tail_z = T_WM@Ts_inv.dot(zaxis_h)
    ax.plot(aruco_head[0],aruco_tail_y[1],aruco_head[2],'x',color='red',label='CAm Ground Truth')
    # ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red')
    # ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green')
    # ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue')

plt.axis('equal')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim(-4,4)
ax.set_ylim(-4,4)
ax.set_zlim(0,2)



############################ Cam COORD ############################
########################################################
########################################################

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
for i in range(len(snapstate_all)):
    snapstate = snapstate_all[i]
    Ts = Ts_all[i]
    # Unit vectors along axes
    xaxis = np.array([1, 0, 0])
    yaxis = np.array([0, 1, 0])
    zaxis = np.array([0, 0, 1])
    q = marker_GT[11:15] # x, y, z, w
    T_WM = quaternion_matrix(q)
    T_WM[:3, 3] = marker_GT[:3]
    scale = 1
 
    ax.plot(0,0,0, 'x',color='red',label="Vicon Zero")
    ax.plot([0,xaxis[0]],[0,xaxis[1]],[0,xaxis[2]],color='red')
    ax.plot([0,yaxis[0]],[0,yaxis[1]],[0,yaxis[2]],color='green')
    ax.plot([0,zaxis[0]],[0,zaxis[1]],[0,zaxis[2]],color='blue')
    ########### PLOT for Marker Poses ###########
    # xaxis_h = np.array([1,0,0,1])
    # yaxis_h = np.array([0,1,0,1])
    # zaxis_h = np.array([0,0,1,1])
    # marker_center = T_WM[:4,3]
    # marker_tail_x = T_WM@xaxis_h
    # marker_tail_y = T_WM@yaxis_h
    # marker_tail_z = T_WM@zaxis_h
    # ax.plot([marker_center[0],marker_tail_x[0]],[marker_center[1],marker_tail_x[1]],[marker_center[2],marker_tail_x[2]],color='red')
    # ax.plot([marker_center[0],marker_tail_y[0]],[marker_center[1],marker_tail_y[1]],[marker_center[2],marker_tail_y[2]],color='green')
    # ax.plot([marker_center[0],marker_tail_z[0]],[marker_center[1],marker_tail_z[1]],[marker_center[2],marker_tail_z[2]],color='blue')

    # ########### PLOT for Camera Poses ###########
    Ts_inv = np.linalg.inv(Ts)
    aruco_head =   Ts_inv.dot(np.array([0,0,0,1]))
    aruco_tail_x = Ts_inv.dot(xaxis_h)
    aruco_tail_y = Ts_inv.dot(yaxis_h)
    aruco_tail_z = Ts_inv.dot(zaxis_h)
    ax.plot(aruco_head[0],aruco_head[1],aruco_head[2],'x',color='red')
    # ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red')
    # ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green')
    # ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue')
    ########### PLOT for Drone Poses ###########
    # T_WD = quaternion_matrix(snapstate[11:15])
    # T_WD[:3,3] = snapstate[:3]
    # drone_axes_scale = 1
    # drone_head = T_WD[:3,3]
    # xaxis_h = np.array([1,0,0,1])
    # yaxis_h = np.array([0,1,0,1])
    # zaxis_h = np.array([0,0,1,1])
    # drone_axes_tip_x = T_WD@xaxis_h
    # drone_axes_tip_y = T_WD@yaxis_h
    # drone_axes_tip_z = T_WD@zaxis_h
    # #print(drone_head)
    # ax.plot(snapstate[0],snapstate[1],snapstate[2],'*',color='green')
    # ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
    # ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
    # ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim(-4,4)
ax.set_ylim(-4,4)
ax.set_zlim(0,2)

plt.show()


print("difference",np.linalg.norm(snapstate[:3]-aruco_head[:3]))