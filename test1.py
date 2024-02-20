import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


Ts1 = np.array([[ 0.99235446, -0.12341466,  0.00120344, -0.13466576],
      [-0.12337095, -0.99218506, -0.01866596,  0.00412619],
      [ 0.00349769,  0.01837477, -0.99982505,  0.44603353],
      [ 0.        ,  0.        ,  0.        ,  1.        ]])


Ts2 = np.array([[ 0.99650939,  0.0826815 ,  0.01152373,  0.09713301],
      [ 0.04667233, -0.43734274, -0.89808297,  0.2059705 ],
      [-0.06921503,  0.89548596, -0.43967509,  0.88839393],
      [ 0.        ,  0.        ,  0.        ,  1.        ]])


Ts3 = np.array([[ 0.66306562,  0.74813183,  0.02535244,  0.02049873],
      [ 0.50157316, -0.418891  , -0.75693771,  0.00738784],
      [-0.55566929,  0.51461548, -0.65299507,  0.66544974],
      [ 0.        ,  0.        ,  0.        ,  1.        ]])

Ts4 = np.array([[ 0.94899689,  0.31405799, -0.02779369, -0.10590953],
       [ 0.02064642, -0.14986781, -0.98849045,  0.19216856],
       [-0.3146087 ,  0.93750052, -0.14870826,  1.57354859],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])


# Ts_full = [Ts1,Ts2,Ts3,Ts4]
Ts_full = [Ts4]

xaxis = np.array([1,0,0])
yaxis = np.array([0,1,0])
zaxis = np.array([0,0,1])
scale = 0.2








for i in range(len(Ts_full)):
   fig = plt.figure()
   ax = fig.add_subplot(111,projection='3d')
   ax.plot(0,0,0, 'x',color='red')
   Ts = Ts_full[i]
   translation = Ts[:3,3]
   # #forward
   # aruco_tail_x = Ts[:3,:3].dot(xaxis*scale) + translation
   # aruco_tail_y = Ts[:3,:3].dot(yaxis*scale) + translation
   # aruco_tail_z = Ts[:3,:3].dot(zaxis*scale) + translation


   #backward
#    rotation = np.linalg.inv(Ts[:3,:3])
#    aruco_head_x = rotation.dot(np.array([0,0,0])*scale - translation)
#    aruco_head_y = rotation.dot(np.array([0,0,0])*scale - translation)
#    aruco_head_z = rotation.dot(np.array([0,0,0])*scale - translation)
#    aruco_tail_x = rotation.dot(xaxis*scale - translation)
#    aruco_tail_y = rotation.dot(yaxis*scale - translation)
#    aruco_tail_z = rotation.dot(zaxis*scale - translation)
   Ts_inv = np.linalg.inv(Ts)
   aruco_head = Ts_inv.dot(np.array([0,0,0,1]))
   xaxis_h = np.array([1,0,0,1])
   yaxis_h = np.array([0,1,0,1])
   zaxis_h = np.array([0,0,1,1])
   aruco_tail_x = Ts_inv.dot(xaxis_h)
   aruco_tail_y = Ts_inv.dot(yaxis_h)
   aruco_tail_z = Ts_inv.dot(zaxis_h)
   


#    ax.plot(translation[0],translation[1],translation[2], 'x',color='blue',label=f'Ts #{i}')


   ax.plot([0,xaxis[0]*scale],[0,xaxis[1]*scale],[0,xaxis[2]*scale],color='red')
   ax.plot([0,yaxis[0]*scale],[0,yaxis[1]*scale],[0,yaxis[2]*scale],color='green')
   ax.plot([0,zaxis[0]*scale],[0,zaxis[1]*scale],[0,zaxis[2]*scale],color='blue')

#    ax.plot(aruco_head_x[0],aruco_head_x[1],aruco_head_x[2], 'x',color = 'blue' )


   ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red')
   ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green')
   ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue')


   ax.set_xlabel('x')
   ax.set_ylabel('y')
   ax.set_zlabel('z')
   plt.legend()
   plt.axis('equal')

#    print("marker estimate at position ",aruco_head_x[0],aruco_head_x[1],aruco_head_x[2])
plt.show()
###################################

import pickle
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
print("snap",snapstate)




q = marker_GT[11:15] # x, y, z, w
T_WM = transformations.quaternion_matrix(q)
T_WM[:3, 3] = marker_GT[:3]
print("Transform_world marker",T_WM)

drone_x = []
drone_y = []
drone_z = []

idx_spotted = []
marker_TS = []



# Unit vectors along axes
x_axis = np.array([1, 0, 0])
y_axis = np.array([0, 1, 0])
z_axis = np.array([0, 0, 1])



# Apply rotation and translation To Drone
drone_axes_x = []
drone_axes_y = []
drone_axes_z = []

##########################################################################################################################################

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
aruco_head_x = rotation.dot(np.array([0,0,0])*scale - translation)
aruco_head_y = rotation.dot(np.array([0,0,0])*scale - translation)
aruco_head_z = rotation.dot(np.array([0,0,0])*scale - translation)
aruco_tail_x = rotation.dot(xaxis*scale - translation)
aruco_tail_y = rotation.dot(yaxis*scale - translation)
aruco_tail_z = rotation.dot(zaxis*scale - translation)


#    ax.plot(translation[0],translation[1],translation[2], 'x',color='blue',label=f'Ts #{i}')


ax.plot([0,xaxis[0]*scale],[0,xaxis[1]*scale],[0,xaxis[2]*scale],color='red')
ax.plot([0,yaxis[0]*scale],[0,yaxis[1]*scale],[0,yaxis[2]*scale],color='green')
ax.plot([0,zaxis[0]*scale],[0,zaxis[1]*scale],[0,zaxis[2]*scale],color='blue')

ax.plot(aruco_head_x[0],aruco_head_x[1],aruco_head_x[2], 'x',color = 'blue' )


ax.plot([aruco_head_x[0],aruco_tail_x[0]],[aruco_head_x[1],aruco_tail_x[1]],[aruco_head_x[2],aruco_tail_x[2]],color='red')
ax.plot([aruco_head_y[0],aruco_tail_y[0]],[aruco_head_y[1],aruco_tail_y[1]],[aruco_head_y[2],aruco_tail_y[2]],color='green')
ax.plot([aruco_head_z[0],aruco_tail_z[0]],[aruco_head_z[1],aruco_tail_z[1]],[aruco_head_z[2],aruco_tail_z[2]],color='blue')

########### PLOT for Drone Poses ###########
drone_rotation = transformations.quaternion_matrix(snapstate[11:15])
drone_translation = snapstate[:3]
drone_axes_scale = 1
drone_axes_tip_x = drone_translation + drone_rotation[:3,:3].dot(x_axis*drone_axes_scale)
drone_axes_tip_y = drone_translation + drone_rotation[:3,:3].dot(y_axis*drone_axes_scale)
drone_axes_tip_z = drone_translation + drone_rotation[:3,:3].dot(z_axis*drone_axes_scale)
ax.plot(drone_translation[0],drone_translation[1],drone_translation[2],'x',color='blue')

print("drone pos estimate",drone_translation[0]-marker_GT[0],drone_translation[1]-marker_GT[1],drone_translation[2]-marker_GT[2])
ax.plot([drone_translation[0]-marker_GT[0], drone_axes_tip_x[0]-marker_GT[0]], [drone_translation[1]-marker_GT[1], drone_axes_tip_x[1]-marker_GT[1]], [drone_translation[2]-marker_GT[2],drone_axes_tip_x[2]-marker_GT[2]], color='red',   linewidth=0.5)
ax.plot([drone_translation[0]-marker_GT[0], drone_axes_tip_y[0]-marker_GT[0]], [drone_translation[1]-marker_GT[1], drone_axes_tip_y[1]-marker_GT[1]], [drone_translation[2]-marker_GT[2],drone_axes_tip_y[2]-marker_GT[2]], color='green', linewidth=0.5)
ax.plot([drone_translation[0]-marker_GT[0], drone_axes_tip_z[0]-marker_GT[0]], [drone_translation[1]-marker_GT[1], drone_axes_tip_z[1]-marker_GT[1]], [drone_translation[2]-marker_GT[2],drone_axes_tip_z[2]-marker_GT[2]], color='blue',  linewidth=0.5)





ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.legend()
plt.axis('equal')
print("marker estimate at position ",aruco_head_x[0],aruco_head_x[1],aruco_head_x[2])
plt.show()

##########################################################################################################################################

# fig = plt.figure()
# ax = fig.add_subplot(111,projection='3d')
# ax.plot(0,0,0, 'x',color='red')
# translation = Ts[:3,3]
# # #forward
# # aruco_tail_x = Ts[:3,:3].dot(xaxis*scale) + translation
# # aruco_tail_y = Ts[:3,:3].dot(yaxis*scale) + translation
# # aruco_tail_z = Ts[:3,:3].dot(zaxis*scale) + translation


# #backward
# rotation = np.linalg.inv(Ts[:3,:3])
# scale = 1
# aruco_head_x = rotation.dot(np.array([0,0,0])*scale - translation)
# aruco_head_y = rotation.dot(np.array([0,0,0])*scale - translation)
# aruco_head_z = rotation.dot(np.array([0,0,0])*scale - translation)
# aruco_tail_x = rotation.dot(xaxis*scale - translation)
# aruco_tail_y = rotation.dot(yaxis*scale - translation)
# aruco_tail_z = rotation.dot(zaxis*scale - translation)


# #    ax.plot(translation[0],translation[1],translation[2], 'x',color='blue',label=f'Ts #{i}')

# marker_x = T_WM[:3,:3].dot(x_axis) + T_WM[:3,3]
# marker_y = T_WM[:3,:3].dot(y_axis) + T_WM[:3,3]
# marker_z = T_WM[:3,:3].dot(z_axis) + T_WM[:3,3]
# ########### PLOT for Aruco Marker Center ###########
# ax.plot(marker_GT[0],marker_GT[1],marker_GT[2], 'x', color='r')
# axes_size = 1
# # marker_GT = np.array([0,0,0])
# # marker_x = np.array([1,0,0])
# # marker_y = np.array([0,1,0])
# # marker_z = np.array([0,0,1])
# ax.plot([marker_GT[0], marker_x[0]], [marker_GT[1], marker_x[1]], [marker_GT[2],marker_x[2]], color='red', linewidth=2)
# ax.plot([marker_GT[0], marker_y[0]], [marker_GT[1], marker_y[1]], [marker_GT[2],marker_y[2]], color='green', linewidth=2)
# ax.plot([marker_GT[0], marker_z[0]], [marker_GT[1], marker_z[1]], [marker_GT[2],marker_z[2]], color='blue', linewidth=2)

# # ax.plot(aruco_head_x[0],aruco_head_x[1],aruco_head_x[2], 'x',color = 'blue' )

# (Ts_inv@T_WM).dot()
# ax.plot([aruco_head_x[0],aruco_tail_x[0]],[aruco_head_x[1],aruco_tail_x[1]],[aruco_head_x[2],aruco_tail_x[2]],color='red')
# ax.plot([aruco_head_y[0],aruco_tail_y[0]],[aruco_head_y[1],aruco_tail_y[1]],[aruco_head_y[2],aruco_tail_y[2]],color='green')
# ax.plot([aruco_head_z[0],aruco_tail_z[0]],[aruco_head_z[1],aruco_tail_z[1]],[aruco_head_z[2],aruco_tail_z[2]],color='blue')

# ########### PLOT for Drone Poses ###########
# drone_rotation = transformations.quaternion_matrix(snapstate[11:15])
# drone_translation = snapstate[:3]
# drone_axes_scale = 1
# drone_axes_tip_x = drone_translation + drone_rotation[:3,:3].dot(x_axis*drone_axes_scale)
# drone_axes_tip_y = drone_translation + drone_rotation[:3,:3].dot(y_axis*drone_axes_scale)
# drone_axes_tip_z = drone_translation + drone_rotation[:3,:3].dot(z_axis*drone_axes_scale)
# ax.plot(drone_translation[0],drone_translation[1],drone_translation[2],'x',color='blue')

# print("drone pos estimate",drone_translation[0]-marker_GT[0],drone_translation[1]-marker_GT[1],drone_translation[2]-marker_GT[2])
# ax.plot([drone_translation[0], drone_axes_tip_x[0]], [drone_translation[1], drone_axes_tip_x[1]], [drone_translation[2],drone_axes_tip_x[2]], color='red',   linewidth=0.5)
# ax.plot([drone_translation[0], drone_axes_tip_y[0]], [drone_translation[1], drone_axes_tip_y[1]], [drone_translation[2],drone_axes_tip_y[2]], color='green', linewidth=0.5)
# ax.plot([drone_translation[0], drone_axes_tip_z[0]], [drone_translation[1], drone_axes_tip_z[1]], [drone_translation[2],drone_axes_tip_z[2]], color='blue',  linewidth=0.5)


# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# plt.legend()
# plt.axis('equal')
# print("marker estimate at position ",aruco_head_x[0],aruco_head_x[1],aruco_head_x[2])
# plt.show()


# ##########################################################################################################################################
# fig = plt.figure()
# ax = fig.add_subplot(111,projection='3d')
# # ax.scatter(drone_x,drone_y,drone_z)


# # Apply rotation and translation To Marker 
# marker_x = T_WM[:3,:3].dot(x_axis) + T_WM[:3,3]
# marker_y = T_WM[:3,:3].dot(y_axis) + T_WM[:3,3]
# marker_z = T_WM[:3,:3].dot(z_axis) + T_WM[:3,3]
# ########### PLOT for Aruco Marker Center ###########
# ax.plot(marker_GT[0],marker_GT[1],marker_GT[2], 'x', color='r')
# axes_size = 1
# # marker_GT = np.array([0,0,0])
# # marker_x = np.array([1,0,0])
# # marker_y = np.array([0,1,0])
# # marker_z = np.array([0,0,1])
# ax.plot([marker_GT[0], marker_x[0]], [marker_GT[1], marker_x[1]], [marker_GT[2],marker_x[2]], color='red', linewidth=2)
# ax.plot([marker_GT[0], marker_y[0]], [marker_GT[1], marker_y[1]], [marker_GT[2],marker_y[2]], color='green', linewidth=2)
# ax.plot([marker_GT[0], marker_z[0]], [marker_GT[1], marker_z[1]], [marker_GT[2],marker_z[2]], color='blue', linewidth=2)



# ########### PLOT for Drone Poses ###########
# drone_rotation = transformations.quaternion_matrix(snapstate[11:15])
# drone_translation = snapstate[:3]
# drone_axes_scale = 1
# drone_axes_tip_x = drone_translation + drone_rotation[:3,:3].dot(x_axis*drone_axes_scale)
# drone_axes_tip_y = drone_translation + drone_rotation[:3,:3].dot(y_axis*drone_axes_scale)
# drone_axes_tip_z = drone_translation + drone_rotation[:3,:3].dot(z_axis*drone_axes_scale)
# ax.plot(drone_translation[0],drone_translation[1],drone_translation[2],'x',color='blue')

# print("drone pos estimate",drone_translation[0]-marker_GT[0],drone_translation[1]-marker_GT[1],drone_translation[2]-marker_GT[2])
# ax.plot([drone_translation[0], drone_axes_tip_x[0]], [drone_translation[1], drone_axes_tip_x[1]], [drone_translation[2],drone_axes_tip_x[2]], color='red',   linewidth=0.5)
# ax.plot([drone_translation[0], drone_axes_tip_y[0]], [drone_translation[1], drone_axes_tip_y[1]], [drone_translation[2],drone_axes_tip_y[2]], color='green', linewidth=0.5)
# ax.plot([drone_translation[0], drone_axes_tip_z[0]], [drone_translation[1], drone_axes_tip_z[1]], [drone_translation[2],drone_axes_tip_z[2]], color='blue',  linewidth=0.5)


# # ########### PLOT for Camera Poses ###########
# marker_center = np.array([marker_GT[0],marker_GT[1],marker_GT[2]])
# rotation = np.linalg.inv(Ts[:3,:3])
# translation = Ts[:3,3]
# scale = 1
# aruco_head = rotation.dot(marker_center*scale - translation)
# marker_x = np.array([1,0,0])
# marker_y = np.array([0,1,0])
# marker_z = np.array([0,0,1])
# aruco_tail_x = rotation.dot(marker_x*scale - translation)
# aruco_tail_y = rotation.dot(marker_y*scale - translation)
# aruco_tail_z = rotation.dot(marker_z*scale - translation)

# ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red')
# ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green')
# ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue')

# ax.set_xlabel('X Axis')
# ax.set_ylabel('Y Axis')
# ax.set_zlabel('Z Axis')
# # ax.set_zlim(0,2)
# plt.axis('equal')
# ax.set_title('World Coordinate')
# plt.show()