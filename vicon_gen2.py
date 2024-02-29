#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)

import rospy
import numpy as np
# import pickle5 as pickle
import argparse
from DroneInterface import Drone
from geometry_msgs.msg import PoseArray, Pose
from functools import partial
from picam.picam_aruco import get_camera,detect_aruco,release_camera
import time
import os
import pickle
import cv2
import matplotlib.pyplot as plt

marker_GT_state = [-0.29408993530273436, -0.07971382904052735, 0.4234147644042969, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.048583984375, 0.0, -0.0160369873046875, -0.0204010009765625, -0.0467987060546875, 0.998565673828125, 1709219631.4415877]


parser = argparse.ArgumentParser(description="")
parser.add_argument('-id','--gates_id', type=int, nargs='+', action='append', help='IDs of the ArUco markers.')
parser.add_argument('--markersize', type=float, default=59, help='Size of the ArUco marker in mm.')
args = parser.parse_args()


def main():
    targetid = 0 
    camera_mtx = np.array([[488.37405729 ,  0. ,        332.86930938],
            [  0.   ,      490.86269977 ,234.46234393],
            [  0.    ,       0.     ,      1.        ]])
    scale =1
    camera_mtx = camera_mtx * scale
    camera_mtx[2,2] = 1.
    distortion_param = np.array([-0.4253985 ,  0.24863036 ,-0.00162259 ,-0.003012  , -0.09376853])
    cam = get_camera()
    count = 0 
    aruco_detection=0
    calibration_data=[]

    rospy.init_node('gate_vision_module', anonymous=True)
    drone = Drone()
    current_state = drone.state
    dt  = 0.2
    rate = rospy.Rate(1. / dt)
    print("endter2")

    while not rospy.is_shutdown():
        snap_state = current_state.copy()
        print("Snap",snap_state)

        Ts, ids, frame = detect_aruco(camera_mtx,distortion_param,cam, visualize=True)

        if len(Ts)>0:
            print("IDs:   ",ids)
            print(f"Translation x:{round(-Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")
            # cv2.imwrite(f'img{count}.png',frame)
            calibration_data.append([count,marker_GT_state, snap_state, (Ts,ids)])
            count +=1
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break

    cv2.destroyAllWindows()
    release_camera(cam)

    with open('calibrationtest50.pkl', 'wb') as handle:
        pickle.dump(calibration_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
        print("Finished dumping pickle")

if __name__ == '__main__':
    main()