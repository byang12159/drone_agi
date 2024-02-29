#!/usr/bin/env python
from pymavlink import mavutil
import sys, os
import numpy as np
import matplotlib.pyplot as plt
import pickle
import time
from picam.picam_aruco import get_camera,detect_aruco,release_camera
from utils import transformations
from picam.transformation_properties_drone import get_T_DC
import cv2
from DroneInterface import Drone

marker_GT_state = [-0.29408993530273436, -0.07971382904052735, 0.4234147644042969, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.048583984375, 0.0, -0.0160369873046875, -0.0204010009765625, -0.0467987060546875, 0.998565673828125, 1709219631.4415877]

drone = Drone()
current_state = drone.state

#vicon coordinate: up = +z, towards computer is +x, facing arena, to right is +y

def update_plot(data):
    plt.clf()  # Clear the previous plot
    plt.plot(data)  # Plot the new data
    plt.draw()  # Draw the updated plot
    plt.pause(0.001)  # Pause to allow the plot to update

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
    return data

def main():
    calibration_data=[]
    drone_state = []

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
    count = 0 
    aruco_detection=0

    while True:
        Flag_vicon = False
        flag_detection = False
        msg = master.recv_match(blocking=False)

        Ts, ids, frame = detect_aruco(camera_mtx,distortion_param,cam, visualize=True)

        if not msg:
            print("watiting for message")
            flag_vicon = True
            continue

        if msg.get_type() == 'LOCAL_POSITION_NED_COV':

            current_state_vicon = extract_mavlink(msg)
            print("Curr Vicon state",round(current_state_vicon[0],2),round(current_state_vicon[1],2),round(current_state_vicon[2],2))
            # print("Curr Vicon state",current_state_vicon)
            calibration_data.append([count,marker_GT_state, current_state_vicon])

            # print("2) Current aruco Ts", Ts)
                  
        elif msg.get_type() == 'ATT_POS_MOCAP':
            pass    

        if len(Ts)>0:
            print("IDs:   ",ids)
            print(f"Translation x:{round(-Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")
            # cv2.imwrite(f'img{count}.png',frame)
            aruco_detection = (Ts,ids)

        # if flag_vicon and flag_detection:
        #     calibration_data.append([count,marker_GT_state, current_state_vicon, aruco_detection])

        #     count +=1
        #     print("appended history")

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
    
    cv2.destroyAllWindows()
    release_camera(cam)

    with open('calibrationtest40.pkl', 'wb') as handle:
        pickle.dump(calibration_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
        print("Finished dumping pickle")

    # fig = plt.figure()
    # ax = fig.add_subplot(111,projection='3d')
    # ax.plot(0,0,0, 'x',color='red',label="World Center")
    # ax.plot(marker_GT_state[0],marker_GT_state[1],marker_GT_state[2],'x',color='green',label='marker location')
    # for i in range(len(drone_state)):
    #     ax.plot(drone_state[i][0],drone_state[i][1],drone_state[i][2], 'x',color='red')

    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # plt.legend()
    # plt.axis('equal')
    # plt.show()
if __name__ == '__main__':
    main()


    # with open('datalog_Vicon.pkl','wb') as file:
    #     pickle.dump(datastorage,file)
    
    print("finished script")