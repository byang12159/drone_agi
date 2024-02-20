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
marker_GT=[0.8875617065429687, -2.516373291015625, 0.4258682861328125, -0.00021718406677246095, -0.00019746483862400054, 0.0002651964724063873, 0.000815876841545105, -0.010379474639892578, 0.013380368232727051, 3.1372833251953125, 2.288818359375e-05, -0.0033721923828125, -0.004730224609375, -0.0021820068359375, 0.9999771118164062, 1708468968.413967]
Ts1 = np.array([[ 0.42952355,  0.90238792,  0.03472129, -0.18052803],
       [ 0.25248883, -0.08308808, -0.96402581,  0.16715718],
       [-0.86704032,  0.42283853, -0.26353115,  1.19785363],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
snapstate1=np.array([2.0024239501953125, -2.91594384765625, 0.744324462890625, 0.038414569854736326, 0.035592620849609374, 0.1220384292602539, -0.17960023498535158, 0.06345080947875976, 0.1052345962524414, 0.95635986328125, 0.04616546630859375, -0.12380218505859375, -0.05188751220703125, 0.45844268798828125, 0.8785247802734375, 1708469495.4765668])

Ts2 = np.array([[ 0.47661439,  0.87873743,  0.02567599, -0.09316901],
       [ 0.59207005, -0.29926664, -0.74825967, -0.03745136],
       [-0.64983981,  0.37183331, -0.6629089 ,  0.97117785],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
snapstate2=np.array([2.054693603515625, -3.03667138671875, 1.12412890625, -0.3689264221191406, 0.3905461730957031, -0.031088310241699217, -0.6347525634765625, 0.8028709716796875, -0.1617691192626953, 0.979156494140625, -0.27582550048828125, -0.18599700927734375, -0.02028656005859375, 0.46932220458984375, 0.86297607421875, 1708469606.5373085])

Ts3 = np.array([[ 0.90020292,  0.40318237, -0.16455602,  0.03039284],
       [ 0.07928834, -0.52331607, -0.8484419 ,  0.08119778],
       [-0.42819162,  0.7507225 , -0.50305831,  0.99815954],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
snapstate3=np.array([1.424218505859375, -3.52644970703125, 1.070912841796875, 0.21766708374023438, -0.026369462966918944, 0.14444662475585937, -0.6512006225585938, 0.9867909545898438, -0.13551054382324218, 0.8382492065429688, 0.23267364501953125, 0.01107025146484375, 0.00445556640625, 0.40930938720703125, 0.9123153686523438, 1708469858.294972])

Ts_full = [Ts1,Ts2,Ts3]
snapstate_full = [snapstate1,snapstate2,snapstate3]


for i in range(len(Ts_full)):
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.plot(0,0,0, 'x',color='red',label="Vicon Zero")
    Ts= Ts_full[i]
    snapstate = snapstate_full[i]
    # Unit vectors along axes
    xaxis = np.array([1, 0, 0])
    yaxis = np.array([0, 1, 0])
    zaxis = np.array([0, 0, 1])

    q = marker_GT[11:15] # x, y, z, w
    T_WM = transformations.quaternion_matrix(q)
    T_WM[:3, 3] = marker_GT[:3]

    scale = 1
    
#    ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green',label='Drone Ground Truth')
#    ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='cyan',label='Marker Ground Truth')
    ########### PLOT for Marker Poses ###########
    xaxis_h = np.array([1,0,0,1])
    yaxis_h = np.array([0,1,0,1])
    zaxis_h = np.array([0,0,1,1])

    marker_center = T_WM[:4,3]
    marker_tail_x = T_WM@xaxis_h
    marker_tail_y = T_WM@yaxis_h
    marker_tail_z = T_WM@zaxis_h

#    ax.plot([marker_center[0],marker_tail_x[0]*scale],[marker_center[1],marker_tail_x[1]*scale],[marker_center[2],marker_tail_x[2]*scale],color='red')
#    ax.plot([marker_center[0],marker_tail_y[0]*scale],[marker_center[1],marker_tail_y[1]*scale],[marker_center[2],marker_tail_y[2]*scale],color='green')
#    ax.plot([marker_center[0],marker_tail_z[0]*scale],[marker_center[1],marker_tail_z[1]*scale],[marker_center[2],marker_tail_z[2]*scale],color='blue')


    # ########### PLOT for Camera Poses ###########
    translation = Ts[:3,3]
    rotation = np.linalg.inv(Ts[:3,:3])
    Ts_inv = np.linalg.inv(Ts) 

    aruco_head =    Ts_inv.dot(np.array([0,0,0,1]))
    aruco_tail_x = Ts_inv.dot(xaxis_h*scale)
    aruco_tail_y = Ts_inv.dot(yaxis_h*scale)
    aruco_tail_z = Ts_inv.dot(zaxis_h*scale)
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

    ax.plot(snapstate[0]-marker_GT[0], snapstate[1]-marker_GT[1],snapstate[2]-marker_GT[2],'*',color='brown')
    # ax.plot(drone_head[0],drone_head[1],drone_head[2],'x',color='green')
#    ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
#    ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
#    ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')

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