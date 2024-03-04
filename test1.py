import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

marker_GT = [-0.29893997192382815, -0.07764772033691406, 0.40728857421875, -0.00011816411465406417, 0.00024836893379688265, -0.0016576665639877319, -0.002879172086715698, 0.0037432663440704348, -0.054626338958740236, 3.028656005859375, 0.0009613037109375, -0.00151824951171875, 0.0489654541015625, -0.05614471435546875, 0.997222900390625, 1709416480.4523191]


# Unit vectors along axes
xaxis = np.array([1, 0, 0])
yaxis = np.array([0, 1, 0])
zaxis = np.array([0, 0, 1])

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.plot(0,0,0, 'x',color='red',label="world center")
ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='green',label='marker location')

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