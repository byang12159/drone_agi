import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from utils import transformations

#configuration1
# marker_GT=[1.3087847900390626, -1.9492880859375, 0.4168743896484375, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0011444091796875, 0.0, 0.00150299072265625, 0.00135040283203125, 0.00072479248046875, 1.0, 1708384416.5562568]
# Ts = np.array([[ 0.94899689,  0.31405799, -0.02779369, -0.10590953],
#        [ 0.02064642, -0.14986781, -0.98849045,  0.19216856],
#        [-0.3146087 ,  0.93750052, -0.14870826,  1.57354859],
#        [ 0.        ,  0.        ,  0.        ,  1.        ]])
# snapstate=np.array([1.9476622314453125, -3.414154296875, 0.9985899047851563, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.45192718505859375, 0.0, -0.049346923828125, -0.0082244873046875, 0.22440338134765625, 0.9732131958007812, 1708385912.130761])

#configuration2
lead_GT= [-0.8779127807617187, -0.4027943115234375, 0.42511734008789065, -3.213620511814952e-06, 0.00011175540089607239, 0.0002843405902385712, -0.0006304032802581787, 0.004727667331695557, 0.011586643218994141, 0.067779541015625, -0.00141143798828125, -0.09230804443359375, 0.0273895263671875, 0.036468505859375, 0.9946823120117188, 1711032945.7473]

chase_GT_1 = [1.290982421875, -0.8248975830078125, 0.703713134765625, -3.272620216012001e-05, -0.00010548879951238633, 0.00015273547172546387, -0.0011584341526031494, -0.0015305432081222535, 0.0006933321356773377, 1.5769119262695312, 0.00145721435546875, 0.04804229736328125, 0.06327056884765625, 0.7064590454101562, 0.7032852172851562, 1711032604.84571]
chase_GT_2 = [0.2796621398925781, -1.8550081787109376, 0.6992968139648438, -5.335561558604241e-05, -0.00045087558031082153, 2.1568933501839638e-05, -0.0014044431447982788, -0.01916812515258789, -0.00474750804901123, 0.6205215454101562, 0.00305938720703125, 0.04004669189453125, -0.055816650390625, 0.30600738525390625, 0.9495468139648438, 1711033474.175141]
marker_GT = [0.3468333740234375, -0.4496033325195313, 0.40073040771484375, 0.00043244883418083193, 4.0757115930318835e-05, 0.0002267577052116394, 0.014019826889038085, 0.004433740139007569, 0.0073454370498657225, 0.0750274658203125, 0.00026702880859375, -0.0055389404296875, -0.00489044189453125, 0.03748321533203125, 0.999267578125, 1711034920.7607558]
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
# ax.plot(0,0,0, 'x',color='red',label="Vicon Zero")
    
# Unit vectors along axes
xaxis = np.array([0, 1, 0])
yaxis = np.array([1, 0, 0])
zaxis = np.array([0, 0, 1])

xaxis_h = np.array([0,1,0,1])
yaxis_h = np.array([1,0,0,1])
zaxis_h = np.array([0,0,1,1])

q = lead_GT[11:15] # x, y, z, w
T_WL = transformations.quaternion_matrix(q)
T_WL[:3, 3] = lead_GT[:3]
drone_head = T_WL[:3,3]
drone_axes_tip_x = T_WL@xaxis_h
drone_axes_tip_y = T_WL@yaxis_h
drone_axes_tip_z = T_WL@zaxis_h

ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')




T_WC = transformations.quaternion_matrix(chase_GT_1[11:15])
T_WC[:3,3] = chase_GT_1[:3]
drone_axes_scale = 1


q = marker_GT[11:15] # x, y, z, w
T_WL = transformations.quaternion_matrix(q)
T_WL[:3, 3] = marker_GT[:3]
drone_head = T_WL[:3,3]
drone_axes_tip_x = T_WL@xaxis_h
drone_axes_tip_y = T_WL@yaxis_h
drone_axes_tip_z = T_WL@zaxis_h

ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')



drone_head = T_WC[:3,3]
drone_axes_tip_x = T_WC@xaxis_h
drone_axes_tip_y = T_WC@yaxis_h
drone_axes_tip_z = T_WC@zaxis_h

ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')


T_WC = transformations.quaternion_matrix(chase_GT_2[11:15])
T_WC[:3,3] = chase_GT_2[:3]
drone_axes_scale = 1

drone_head = T_WC[:3,3]
drone_axes_tip_x = T_WC@xaxis_h
drone_axes_tip_y = T_WC@yaxis_h
drone_axes_tip_z = T_WC@zaxis_h

ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')



    
ax.plot(lead_GT[0],lead_GT[1],lead_GT[2],'x',color='green',label='laed Ground Truth')
ax.plot(chase_GT_1[0],chase_GT_1[1],chase_GT_1[2],'x',color='red',label='Chase Ground Truth')
ax.plot(chase_GT_2[0],chase_GT_2[1],chase_GT_2[2],'x',color='cyan',label='Chase Ground Truth')
ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='blue',label='Chase Ground Truth')

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.legend()
plt.axis('equal')
plt.show()

#     ########### PLOT for Marker Poses ###########


#     marker_center = T_WM[:4,3]
#     marker_tail_x = T_WM@xaxis_h
#     marker_tail_y = T_WM@yaxis_h
#     marker_tail_z = T_WM@zaxis_h

# #    ax.plot([marker_center[0],marker_tail_x[0]*scale],[marker_center[1],marker_tail_x[1]*scale],[marker_center[2],marker_tail_x[2]*scale],color='red')
# #    ax.plot([marker_center[0],marker_tail_y[0]*scale],[marker_center[1],marker_tail_y[1]*scale],[marker_center[2],marker_tail_y[2]*scale],color='green')
# #    ax.plot([marker_center[0],marker_tail_z[0]*scale],[marker_center[1],marker_tail_z[1]*scale],[marker_center[2],marker_tail_z[2]*scale],color='blue')


#     # ########### PLOT for Camera Poses ###########
#     translation = Ts[:3,3]
#     rotation = np.linalg.inv(Ts[:3,:3])
#     Ts_inv = np.linalg.inv(Ts) 

#     aruco_head =    Ts_inv.dot(np.array([0,0,0,1]))
#     aruco_tail_x = Ts_inv.dot(xaxis_h*scale)
#     aruco_tail_y = Ts_inv.dot(yaxis_h*scale)
#     aruco_tail_z = Ts_inv.dot(zaxis_h*scale)
#     # aruco_head =   Ts@T_WM.dot(np.array([0,0,0,1]))
#     # aruco_tail_x = Ts@T_WM.dot(xaxis_h*scale)
#     # aruco_tail_y = Ts@T_WM.dot(yaxis_h*scale)
#     # aruco_tail_z = Ts@T_WM.dot(zaxis_h*scale)

#     # print("one",T_WM@Ts_inv)
#     # print("two",T_WM.dot(Ts_inv))

#     ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red')
#     ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green')
#     ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue')


#     ########### PLOT for Drone Poses ###########

#     ax.set_xlabel('x')
#     ax.set_ylabel('y')
#     ax.set_zlabel('z')
#     plt.legend()
#     plt.axis('equal')
#     print("marker estimate at position ",aruco_head[0],aruco_head[1],aruco_head[2])
# plt.show()


# # print("difference",drone_head-aruco_head)

# print("marker location \n",marker_center)
# print("drone location \n",drone_head)
# print("camera location \n",aruco_head)