import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
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
marker_GT = [0.19199954223632812, 0.027038570404052734, 0.4160528869628906, 0.00018454967439174652, -0.000207060843706131, -0.00027632442116737365, 0.007449865818023682, -0.010549224853515626, -0.014547050476074219, 3.1265869140625, 0.0030364990234375, -0.02341461181640625, 0.00458526611328125, -0.0071258544921875, 0.9996871948242188, 1709134152.8482578]
snapstate1 =  [0.14299026489257813, -0.7319353637695313, 1.0732603759765624, 0.02414802360534668, -0.016112598419189453, -0.004285656929016114, -0.018286571502685547, 0.05059247589111328, 0.2437384490966797, 0.01468658447265625, -0.0452880859375, -0.21590423583984375, -0.01139068603515625, 0.00463104248046875, 0.9763412475585938, 1709136678.3512197]
Ts1 = np.array([[ 0.99916545,  0.03095087, -0.02665426,  0.05880824],
       [-0.00683009, -0.51676502, -0.85610003,  0.10718842],
       [-0.04027103,  0.85556762, -0.51612236,  0.91213851],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
snapstate2 = [-1.80584716796875, 1.6302332763671874, 1.984008544921875, 0.009252525329589844, -0.0023522355556488037, -0.0045342850685119625, -0.05579570007324219, 0.1621397247314453, 0.08427718353271485, 1.094024658203125, 0.0727386474609375, 0.1906280517578125, -0.09641265869140625, 0.8449859619140625, -0.49027252197265625, 1709137128.6455402]
Ts2 = np.array([[-0.34578267, -0.9355177 ,  0.07239462,  0.4082873 ],
       [-0.42513611,  0.08742171, -0.90089774,  0.20631378],
       [ 0.83647692, -0.34229239, -0.42795126,  2.90133002],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
snapstate3 = [0.0884438247680664, 0.023736513137817383, 1.8927960205078125, 0.017331722259521485, 0.008663010597229005, 0.00129761803150177, 0.01074010944366455, 0.05979350662231445, 0.016555883407592772, 3.0251083374023438, 0.07447052001953125, -0.7071456909179688, -0.04300689697265625, -0.12424468994140625, 0.694732666015625, 1709137397.6940603]
Ts3 = np.array([[ 0.93573693, -0.23260882,  0.26512174, -0.30596506],
      [-0.21647359, -0.97222666, -0.08896353, -0.13228724],
      [ 0.27845212,  0.02585461, -0.96010206,  1.39413258],
      [ 0.        ,  0.        ,  0.        ,  1.        ]])

Ts_all = [Ts1,Ts2,Ts3]
snapstate_all = [snapstate1, snapstate2, snapstate3]
for i in range(3):
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
    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.plot(0,0,0, 'x',color='red',label="Vicon Zero")
    ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green',label='Drone Ground Truth')
    ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='cyan',label='Marker Ground Truth')
    ax.plot([0,xaxis[0]],[0,xaxis[1]],[0,xaxis[2]],color='red')
    ax.plot([0,yaxis[0]],[0,yaxis[1]],[0,yaxis[2]],color='green')
    ax.plot([0,zaxis[0]],[0,zaxis[1]],[0,zaxis[2]],color='blue')
    ########### PLOT for Marker Poses ###########
    xaxis_h = np.array([1,0,0,1])
    yaxis_h = np.array([0,1,0,1])
    zaxis_h = np.array([0,0,1,1])
    marker_center = T_WM[:4,3]
    marker_tail_x = T_WM@xaxis_h
    marker_tail_y = T_WM@yaxis_h
    marker_tail_z = T_WM@zaxis_h
    ax.plot([marker_center[0],marker_tail_x[0]],[marker_center[1],marker_tail_x[1]],[marker_center[2],marker_tail_x[2]],color='red')
    ax.plot([marker_center[0],marker_tail_y[0]],[marker_center[1],marker_tail_y[1]],[marker_center[2],marker_tail_y[2]],color='green')
    ax.plot([marker_center[0],marker_tail_z[0]],[marker_center[1],marker_tail_z[1]],[marker_center[2],marker_tail_z[2]],color='blue')
    # ########### PLOT for Camera Poses ###########
    Ts_inv = np.linalg.inv(Ts)
    aruco_head =   T_WM@Ts_inv.dot(np.array([0,0,0,1]))
    aruco_tail_x = T_WM@Ts_inv.dot(xaxis_h)
    aruco_tail_y = T_WM@Ts_inv.dot(yaxis_h)
    aruco_tail_z = T_WM@Ts_inv.dot(zaxis_h)
    ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red')
    ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green')
    ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue')
    ########### PLOT for Drone Poses ###########
    T_WD = quaternion_matrix(snapstate[11:15])
    T_WD[:3,3] = snapstate[:3]
    drone_axes_scale = 1
    drone_head = T_WD[:3,3]
    xaxis_h = np.array([1,0,0,1])
    yaxis_h = np.array([0,1,0,1])
    zaxis_h = np.array([0,0,1,1])
    drone_axes_tip_x = T_WD@xaxis_h
    drone_axes_tip_y = T_WD@yaxis_h
    drone_axes_tip_z = T_WD@zaxis_h
    #print(drone_head)
    ax.plot(snapstate[0],snapstate[1],snapstate[2],'*',color='green')
    ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
    ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
    ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.axis('equal')
    plt.show()


    print("difference",np.linalg.norm(snapstate[:3]-aruco_head[:3]))