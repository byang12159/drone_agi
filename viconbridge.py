#!/usr/bin/env python
from pymavlink import mavutil
import sys, os
import numpy as np
import pickle
import time
from picam.picam_aruco import get_camera,detect_aruco,release_camera
from utils import transformations
from picam.transformation_properties_drone import get_T_DC
import cv2

marker_GT_state = [0.8422297973632813, -2.490376953125, 0.4268939514160156, 0.0011990131139755248, 0.0023979015350341797, -0.0013976448774337769, 0.022845638275146483, 0.05173447799682617, -0.03210985946655273, 0.0230865478515625, -0.0040740966796875, -0.01971435546875, 0.02188873291015625, 0.01209259033203125, 0.9994964599609375, 1708277391.1825695]

def extract_mavlink(msg):
    # Data = [x y z vx vy vz ax ay az + ]
    data= [0, ] * (9 + 2 + 4 + 1)

    data[0] = msg.x / 1000.
    data[1] = msg.y / 1000.
    data[2] = msg.z / 1000.
    data[3] = msg.vx / 1000.
    data[4] = msg.vy / 1000.
    data[5] = msg.vz / 1000.
    data[6] = msg.ax / 1000.
    data[7] = msg.ay / 1000.
    data[8] = msg.az / 1000.

    # use msg.covariance to store the yaw and yaw_rate, and q
    offset = 100.
    data[9] = msg.covariance[0] - offset
    data[10] = msg.covariance[1] - offset

    data[11] = msg.covariance[2] - offset
    data[12] = msg.covariance[3] - offset
    data[13] = msg.covariance[4] - offset
    data[14] = msg.covariance[5] - offset

    now = time.time()
    data[-1] = now

    rounded = [round(x,2) for x in data[9:]]
    # print("ALL:",data )

    return data

def main():
    calibration_data=[]
    T_DC = get_T_DC()

    targetid = 0 
    camera_mtx = np.array([[488.37405729 ,  0. ,        332.86930938],
            [  0.   ,      490.86269977 ,234.46234393],
            [  0.    ,       0.     ,      1.        ]])
    scale =1
    camera_mtx = camera_mtx * scale
    camera_mtx[2,2] = 1.

    distortion_param = np.array([-0.4253985 ,  0.24863036 ,-0.00162259 ,-0.003012  , -0.09376853])

    cam = get_camera()

    # create a mavlink serial instance
    master = mavutil.mavlink_connection('udpin:0.0.0.0:10085')
    print("starting")
    while True:
        msg = master.recv_match(blocking=False)

        Ts, ids = detect_aruco(camera_mtx,distortion_param,cam, visualize=True)
        print("IDs:   ",ids)

        if not msg:
            # print("watiting for message")
            continue

        if msg.get_type() == 'LOCAL_POSITION_NED_COV':

            current_state_vicon = extract_mavlink(msg)
            snap_state = current_state_vicon.copy()
            print("Currentstate",current_state_vicon)
            if len(Ts)>0:
                print(f"Translation x:{round(-Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break

            datapackage = [marker_GT_state, current_state_vicon, (Ts,ids)]
            calibration_data.append(datapackage)
            # pose_C = Ts[ids.index(targetid)]
            # q = snap_state[11:15] # x, y, z, w
            # T_WD = transformations.quaternion_matrix(q)
            # T_WD[:3, 3] = snap_state[:3]
            # pose_D = T_DC.dot(pose_C)
            # pose_W = T_WD.dot(pose_D)
            # target = pose_W[:3, 3]

            # print("Target",target)        
        elif msg.get_type() == 'ATT_POS_MOCAP':
            pass

    cv2.destroyAllWindows()
    release_camera(cam)

    pickle.dump(calibration_data, open("calibration_data_test.p", "wb"))

        

if __name__ == '__main__':
    main()

    # with open('datalog_Vicon.pkl','wb') as file:
    #     pickle.dump(datastorage,file)
    
    print("finished vicon bridge log")