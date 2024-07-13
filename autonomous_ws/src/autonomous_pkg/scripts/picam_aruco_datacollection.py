#!/usr/bin/env python3
from line_profiler import LineProfiler
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point
import pickle

import numpy as np
import cv2
import cv2.aruco as aruco
import math
import time
# from eulerconvert import rotationMatrixToEulerAngles

camera_mtx = np.array([[546.84164912 ,  0. ,     349.18316327],
                [  0.   ,      547.57957461 ,215.54486004],
                [  0.    ,       0.     ,      1.        ]])


scale = 1
camera_mtx = camera_mtx * scale

distortion_param = np.array([0.04203676, -0.11190902, 0.00080842, 0.00151827, 0.14878741])

marker_size = 118 #mm

alltime = []


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def detect_aruco(cap=None, aruco_dict=None, parameters=None, save=None, visualize=False):
    starttimearuco = time.time()
    def cleanup_cap():
        pass
    if cap is None:
        cap = get_camera()
        cleanup_cap = lambda : cap.release()

    ret, frame = cap.read()

    time1 = time.time() - starttimearuco

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    markerCorners, markerIds, rejectedCandidates= aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    time2 = time.time() - starttimearuco
    
    Ts = []
    ids = []
    if markerIds is not None :
        rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, markerLength=marker_size, cameraMatrix=camera_mtx, distCoeffs=distortion_param)
        # rvecs, tvecs, objpoints = aruco.estimatePoseSingleMarkers(markerCorners, marker_size, , )
        for i in range(len(markerIds)):
            # Unpacking the compounded list structure
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]
            # print("Rvec",rvec)
            # print("Tvec",tvec)
            ids.append(markerIds[i][0])
            if save or visualize:
                frame = aruco.drawDetectedMarkers( frame, markerCorners, markerIds )
                frame = cv2.drawFrameAxes(frame, camera_mtx, distortion_param, rvec, tvec,length = 100, thickness=6)
        
            rotation_mtx, jacobian = cv2.Rodrigues(rvec)
            translation = tvec
            T = np.identity(4)
            T[:3, :3] = rotation_mtx
            T[:3, 3] = translation / 1000.
            Ts.append(T)

    if save:
        cv2.imwrite(save, frame)

    if visualize:
        cv2.imshow("camera view", frame)

    time3 = time.time() - starttimearuco

    return Ts, ids, time1, time2, time3

def get_camera(select_buffer, select_framerate, select_imgwidth,select_imgheight):
    cap = cv2.VideoCapture(0)
    
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 100) 
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    cv2.waitKey(500)

    return cap

def release_camera(cap):
    cap.release()

def publisher(select_buffer, select_framerate, select_imgwidth,select_imgheight, filename):
    rospy.init_node('picam', anonymous=True)
    pub = rospy.Publisher("/leader_waypoint", Point, queue_size=1)
    rate = rospy.Rate(40)  # 1 Hz

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()

    cap = get_camera(select_buffer, select_framerate, select_imgwidth,select_imgheight)
    time.sleep(2)
    print("Picam cap status",cap)
    
    count_detect = 0
    while not rospy.is_shutdown():
        starttime = time.time()
        Ts, ids, time1, time2, time3 = detect_aruco(cap, aruco_dict, parameters)

        if len(Ts)>0 and ids[0] == 0:
            # print(f"Translation x:{round(-Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")

            # Publish the message
            displacement_msg = Point()
            displacement_msg.x =  Ts[0][2, 3]
            displacement_msg.y = -Ts[0][0, 3]
            displacement_msg.z = -Ts[0][1, 3]
      
            pub.publish(displacement_msg)
            # rospy.loginfo("Aruco Detection, Published Point message: {}".format(displacement_msg))

            count_detect+=1
        

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break

        runtime = time.time() - starttime

        data.append([select_buffer, select_framerate, select_imgwidth,select_imgheight, 
                     runtime, time1, time2, time3, Ts, ids])
        
        if count_detect>= 100:
            ret, frame = cap.read()
            cv2.imwrite("image/{}.jpg".format(filename), frame)
            break
        print("Count ",count_detect)
        rate.sleep()

    cv2.destroyAllWindows()
    release_camera(cap)



if __name__ == '__main__':
    
    # buffersizes = [1, 2, 3, 4]
    # framerates = [30, 65, 100]
    # imgwidths = [1280, 800, 640, 320]
    # imgheights = [800, 600, 480, 240]

    buffersizes = [2]
    framerates = [100]
    imgwidths = [640]
    imgheights = [480]

    total_data =[]

    distance = '6'
    pickle_file = 'picam_data_640480_dist{}.pickle'.format(distance)

    for b in buffersizes:
        for f in framerates:
            for index in range(len(imgwidths)):

                data = []
                print("starting ",b, f, imgwidths[index],imgheights[index])
                try:
                    publisher(b, f, imgwidths[index],imgheights[index], pickle_file)
                except rospy.ROSInterruptException:
                    pass

                total_data.append(data)
                time.sleep(3)




    # Pickling the data (writing to a file)
    with open(pickle_file, 'wb') as f:
        pickle.dump(total_data, f)
        print(f'Data has been pickled to {pickle_file}')
        

    # print(alltime)
    # print("datapoints num: ", len(alltime))
    # alltimearray = np.array(alltime)
    # print("max: ",np.max(alltimearray))
    # print("min: ",np.min(alltimearray))
    # print("std: ",np.std(alltimearray))
    # print("mean: ",np.mean(alltimearray))