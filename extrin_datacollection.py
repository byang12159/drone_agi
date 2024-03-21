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

#marker_GT_state = [-0.29408993530273436, -0.07971382904052735, 0.4234147644042969, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.048583984375, 0.0, -0.0160369873046875, -0.0204010009765625, -0.0467987060546875, 0.998565673828125, 1709219631.4415877]
#marker_GT_state = [-0.31170803833007815, -0.09010250854492187, 0.4247349853515625, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5690994262695312, 0.0, -0.0259552001953125, -0.00555419921875, 0.7064895629882812, 0.7072296142578125, 1709569393.1408904]
#marker_GT_state = [2.28994677734375, 0.09273167419433594, 0.666801513671875, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.542755126953125, 0.0, -0.0039520263671875, -0.0919036865234375, -0.7108688354492188, 0.6972808837890625, 1709738831.7512767]
#marker_GT_state = [2.33148876953125, 1.1829923095703125, 0.6608572998046875, -8.76462683081627e-05, -0.0005406544804573059, 0.00033645230531692506, -0.007194867610931396, -0.015786824226379396, 0.008081820487976074, 2.379852294921875, -0.00091552734375, 0.00327301025390625, -0.01073455810546875, 0.9282073974609375, 0.3719024658203125, 1709741184.374893]
marker_GT_state = [0.3345789489746094, -0.4534208984375, 0.3993171997070312, -0.00022939509153366088, 0.0002018704116344452, 0.0005077255964279175, -0.001613927960395813, 0.00436529016494751, 0.01503278636932373, 0.08768463134765625, 0.0005340576171875, -0.0048675537109375, -0.01605224609375, 0.04373931884765625, 0.9989013671875, 1711039770.5233386]

parser = argparse.ArgumentParser(description="")
parser.add_argument('-id','--gates_id', type=int, default=0, help='IDs of the ArUco markers')
parser.add_argument('--markersize', type=float, default=100, help='Size of the ArUco marker in mm')
parser.add_argument('--save_dir', type=str, default="", help='directory to save')
parser.add_argument('--save', type=str, default="", help='filename to save')
args = parser.parse_args()


def main():
    #targetid = args.id 
    camera_mtx = np.array([[488.37405729 ,  0. ,        332.86930938],
            [  0.   ,      490.86269977 ,234.46234393],
            [  0.    ,       0.     ,      1.        ]])

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

    while not rospy.is_shutdown():
        snap_state = current_state.copy()
        print("Snap",snap_state)

        Ts, ids, frame = detect_aruco(camera_mtx,distortion_param,cam, visualize=True, marker_size=args.markersize)

        if len(Ts)>0:
            print("IDs:   ",ids)
            print(f"Translation x:{round(-Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")
            cv2.imwrite(f'img{count}.png',frame)
            calibration_data.append([count,marker_GT_state, snap_state, (Ts,ids)])
            count +=1
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break

    # Quitting 
    cv2.destroyAllWindows()
    release_camera(cam)

    if len(args.save)>0:
        save_file_path = os.path.join(args.save_dir, args.save)
        with open(f'{save_file_path}.pkl', 'wb') as handle:
            pickle.dump(calibration_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
            print(f"Finished dumping pickle data in {save_file_path}")

if __name__ == '__main__':
    if len(args.save_dir)>0:
        if not os.path.exists(args.save_dir):
            os.makedirs(args.save_dir)
            print(f"Directory '{args.save_dir}' created.")
        else:
            print(f"Directory '{args.save_dir}' already exists.")

    main()