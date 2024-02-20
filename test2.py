import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from utils import transformations

marker_GT=[1.3087847900390626, -1.9492880859375, 0.4168743896484375, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0011444091796875, 0.0, 0.00150299072265625, 0.00135040283203125, 0.00072479248046875, 1.0, 1708384416.5562568]
Ts = np.array([[ 0.94899689,  0.31405799, -0.02779369, -0.10590953],
       [ 0.02064642, -0.14986781, -0.98849045,  0.19216856],
       [-0.3146087 ,  0.93750052, -0.14870826,  1.57354859],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
snapstate=np.array([1.9476622314453125, -3.414154296875, 0.9985899047851563, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.45192718505859375, 0.0, -0.049346923828125, -0.0082244873046875, 0.22440338134765625, 0.9732131958007812, 1708385912.130761])

# Unit vectors along axes
xaxis = np.array([1, 0, 0])
yaxis = np.array([0, 1, 0])
zaxis = np.array([0, 0, 1])

q = marker_GT[11:15] # x, y, z, w
T_WM = transformations.quaternion_matrix(q)
T_WM[:3, 3] = marker_GT[:3]

scale = 1
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.plot(0,0,0, 'x',color='red',label="Vicon Zero")
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
plt.legend()
plt.axis('equal')
print("marker estimate at position ",aruco_head[0],aruco_head[1],aruco_head[2])
plt.show()


# print("difference",drone_head-aruco_head)

print("marker location \n",marker_center)
print("drone location \n",drone_head)
print("camera location \n",aruco_head)