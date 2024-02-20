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


fig = plt.figure()


ax = fig.add_subplot(111,projection='3d')
ax.plot(0,0,0, 'x',color='red')
translation = Ts[:3,3]
# #forward
# aruco_tail_x = Ts[:3,:3].dot(xaxis*scale) + translation
# aruco_tail_y = Ts[:3,:3].dot(yaxis*scale) + translation
# aruco_tail_z = Ts[:3,:3].dot(zaxis*scale) + translation


#backward
rotation = np.linalg.inv(Ts[:3,:3])
scale = 1
aruco_head = rotation.dot(np.array([0,0,0])*scale - translation)
aruco_tail_x = rotation.dot(xaxis*scale - translation)
aruco_tail_y = rotation.dot(yaxis*scale - translation)
aruco_tail_z = rotation.dot(zaxis*scale - translation)


#    ax.plot(translation[0],translation[1],translation[2], 'x',color='blue',label=f'Ts #{i}')

########### PLOT for Marker Poses ###########
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

# ax.plot([marker_center[0],marker_tail_x[0]*scale],[marker_center[1],marker_tail_x[1]*scale],[marker_center[2],marker_tail_x[2]*scale],color='red')
# ax.plot([marker_center[0],marker_tail_y[0]*scale],[marker_center[1],marker_tail_y[1]*scale],[marker_center[2],marker_tail_y[2]*scale],color='green')
# ax.plot([marker_center[0],marker_tail_z[0]*scale],[marker_center[1],marker_tail_z[1]*scale],[marker_center[2],marker_tail_z[2]*scale],color='blue')


# ########### PLOT for Camera Poses ###########
# ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red')
# ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green')
# ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue')
aruco_head = T_WM@np.array([aruco_head[0],aruco_head[1],aruco_head[2],1])
aruco_tail_x = T_WM@np.array([aruco_tail_x[0],aruco_tail_x[1],aruco_tail_x[2],1])
aruco_tail_y = T_WM@np.array([aruco_tail_y[0],aruco_tail_y[1],aruco_tail_y[2],1])
aruco_tail_z = T_WM@np.array([aruco_tail_z[0],aruco_tail_z[1],aruco_tail_z[2],1])
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

ax.plot(drone_head[0],drone_head[1],drone_head[2],'x',color='green')
ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')

# drone_head = T_WM@drone_head
# drone_axes_tip_x = T_WM@drone_axes_tip_x
# drone_axes_tip_y = T_WM@drone_axes_tip_y
# drone_axes_tip_z = T_WM@drone_axes_tip_z

# ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
# ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
# ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')

# drone_rotation = transformations.quaternion_matrix(snapstate[11:15])
# drone_translation = snapstate[:3]
# drone_axes_scale = 1
# drone_axes_tip_x = drone_translation + drone_rotation[:3,:3].dot(xaxis*drone_axes_scale)
# drone_axes_tip_y = drone_translation + drone_rotation[:3,:3].dot(yaxis*drone_axes_scale)
# drone_axes_tip_z = drone_translation + drone_rotation[:3,:3].dot(zaxis*drone_axes_scale)
# ax.plot(drone_translation[0],drone_translation[1],drone_translation[2],'x',color='blue')

# print("drone pos estimate",drone_translation[0]-marker_GT[0],drone_translation[1]-marker_GT[1],drone_translation[2]-marker_GT[2])
# ax.plot([drone_translation[0]-marker_GT[0], drone_axes_tip_x[0]-marker_GT[0]], [drone_translation[1]-marker_GT[1], drone_axes_tip_x[1]-marker_GT[1]], [drone_translation[2]-marker_GT[2],drone_axes_tip_x[2]-marker_GT[2]], color='red',   linewidth=0.5)
# ax.plot([drone_translation[0]-marker_GT[0], drone_axes_tip_y[0]-marker_GT[0]], [drone_translation[1]-marker_GT[1], drone_axes_tip_y[1]-marker_GT[1]], [drone_translation[2]-marker_GT[2],drone_axes_tip_y[2]-marker_GT[2]], color='green', linewidth=0.5)
# ax.plot([drone_translation[0]-marker_GT[0], drone_axes_tip_z[0]-marker_GT[0]], [drone_translation[1]-marker_GT[1], drone_axes_tip_z[1]-marker_GT[1]], [drone_translation[2]-marker_GT[2],drone_axes_tip_z[2]-marker_GT[2]], color='blue',  linewidth=0.5)



ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.legend()
plt.axis('equal')
print("marker estimate at position ",aruco_head[0],aruco_head[1],aruco_head[2])
plt.show()


print("difference",drone_head-aruco_head)