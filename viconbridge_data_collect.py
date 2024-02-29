#!/usr/bin/env python
from pymavlink import mavutil
import sys, os
import numpy as np
import pickle
import time
from picam.picam_aruco import get_camera,detect_aruco,release_camera
from utils import transformations
#from picam.transformation_properties_drone import get_T_DC
import cv2

marker_GT_state = [0.6488789672851563, 0.9260479736328125, 0.4038212585449219, 4.081189632415772e-05, 0.00019996635615825652, 1.3682435266673565e-05, 0.00803756046295166, 0.022949226379394533, 0.00794167184829712, 0.04355621337890625, 0.00087738037109375, -0.0262451171875, 0.01726531982421875, 0.02228546142578125, 0.9992599487304688, 1708964609.9356403]

#vicon coordinate: up = +z, towards computer is +x, facing arena, to right is +y
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

    # rounded = [round(x,2) for x in data[9:]]
    # print("ALL:",data )

    return data

def main():
    frame_count = 0
    output_folder_1 = "captured_frames"

    # Create the output folder if it doesn't exist
    if not os.path.exists(output_folder_1):
        os.makedirs(output_folder_1)
        print(f"Created folder: {output_folder_1}")

    calibration_data=[]
    #T_DC = get_T_DC()

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

        Ts, ids, frame = detect_aruco(camera_mtx,distortion_param,cam, visualize=True)
        #print("IDs:   ",ids)

        if not msg:
            # print("watiting for message")
            continue

        if msg.get_type() == 'LOCAL_POSITION_NED_COV':

            current_state_vicon = extract_mavlink(msg)
            snap_state = current_state_vicon.copy()
            #print("1) Currentstate",snap_state)
            if len(Ts)>0 and ids[0] == 0:
                print(f"Translation x:{round(-Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")
                datapackage = [marker_GT_state, snap_state, (Ts,ids)]
                print(datapackage)
                calibration_data.append(datapackage)
                frame_count += 1
                file_path = os.path.join(output_folder_1, f"captured_frame_{frame_count}.jpg")
                cv2.imwrite(file_path, frame)
                print(f"Frame {frame_count} captured and saved to: {file_path}")
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break

            #print("2) Current aruco Ts", Ts)
            
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

    pickle.dump(calibration_data, open("calibration_data_test_circle.p", "wb"))

        

if __name__ == '__main__':
    main()

    # with open('datalog_Vicon.pkl','wb') as file:
    #     pickle.dump(datastorage,file)
    
    print("finished vicon bridge log")