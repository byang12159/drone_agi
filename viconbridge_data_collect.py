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

marker_GT_state = [-0.29353912353515627, -0.07437275695800781, 0.4261025390625, 0.0005176302790641785, 0.0004048292338848114, -1.3104693964123727e-05, 0.012791335105895996, 0.015273360252380372, -0.0025080912113189698, 3.0648727416992188, -0.00089263916015625, -0.01818084716796875, -0.0217437744140625, -0.03874969482421875, 0.9988479614257812, 1709167569.5936272]

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
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        msg = master.recv_match(blocking=False)

        Ts, ids, frame = detect_aruco(camera_mtx,distortion_param,cam, visualize=True)
        print("IDs:   ",ids)

        if not msg:
            print("watiting for message")
            continue

        if msg.get_type() == 'LOCAL_POSITION_NED_COV':

            current_state_vicon = extract_mavlink(msg)
            snap_state = current_state_vicon.copy()
            print("1) Currentstate",snap_state)
            if len(Ts)>0:
                print(f"Translation x:{round(-Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")
                datapackage = [count,marker_GT_state, snap_state, (Ts,ids)]
                print("datapck",datapackage)
                # cv2.imwrite(f'img{count}.png',frame)
                calibration_data.append(datapackage)
                drone_state.append(snap_state)
                count +=1
                # update_plot(data)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break


            print("2) Current aruco Ts", Ts)
                  
        elif msg.get_type() == 'ATT_POS_MOCAP':
            pass    

    
    cv2.destroyAllWindows()
    release_camera(cam)

    # with open('calibrationtest33.pkl', 'wb') as handle:
    #     pickle.dump(calibration_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
    # pickle.dump(calibration_data, open("calibration_data_test32.p", "wb"))

    print("Finished dumping pickle")

    fig = plt.figure()
    ax = fig.add_subplot(111,projection='3d')
    ax.plot(0,0,0, 'x',color='red',label="World Center")
    ax.plot(marker_GT_state[0],marker_GT_state[1],marker_GT_state[2],'x',color='green',label='marker location')
    for i in range(len(drone_state)):
        ax.plot(drone_state[i][0],drone_state[i][1],drone_state[i][2], 'x',color='red')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend()
    plt.axis('equal')
    plt.show()
if __name__ == '__main__':
    main()


    # with open('datalog_Vicon.pkl','wb') as file:
    #     pickle.dump(datastorage,file)
    
    print("finished vicon bridge log")