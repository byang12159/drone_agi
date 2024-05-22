#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Point
# import geometry_msgs.msg as geometry_msgs

import numpy as np
import cv2
import cv2.aruco as aruco
import math
import time
# from eulerconvert import rotationMatrixToEulerAngles

camera_mtx = np.array([[488.37405729 ,  0. ,        332.86930938],
                [  0.   ,      490.86269977 ,234.46234393],
                [  0.    ,       0.     ,      1.        ]])


scale = 1
camera_mtx = camera_mtx * scale

distortion_param = np.array([-0.4253985 ,  0.24863036 ,-0.00162259 ,-0.003012  , -0.09376853])

marker_size = 57 #mm



# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
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

def detect_aruco(cap=None, save=None, visualize=False):

    def cleanup_cap():
        pass
    if cap is None:
        cap = get_camera()
        cleanup_cap = lambda : cap.release()

    ret, frame = cap.read()
 
    frame = cv2.flip(frame,-1)
    # width = int(frame.shape[1] * scale)
    # height = int(frame.shape[0] * scale)
    # dim = (width, height)
    # frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()
    markerCorners, markerIds, rejectedCandidates= aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    # parameters =  cv2.aruco.DetectorParameters()
    # detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)
    # markerCorners, markerIds, rejectedCandidates= detector.detectMarkers(gray)
   
    frame = aruco.drawDetectedMarkers( frame, markerCorners, markerIds )
    Ts = []
    ids = []
    if markerIds is not None:
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

    # Multiple ids in 1D list
    # Mutiple Ts, select first marker 2d array by Ts[0]
    return Ts, ids, frame

def get_camera():
    cap = cv2.VideoCapture(0)

    img_width = 640
    img_height = 480
    frame_rate = 60
    cap.set(2, img_width)
    cap.set(4, img_height)
    cap.set(5, frame_rate)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(2)
    return cap

def release_camera(cap):
    cap.release()

def publisher():
    rospy.init_node('picam', anonymous=True)
    pub = rospy.Publisher("/leader_waypoint", Point, queue_size=2)
    rate = rospy.Rate(20)  # 1 Hz

    cap = get_camera()
    time.sleep(2)
    print("Picam cap status",cap)
    
    while not rospy.is_shutdown():
        s = time.time()
        Ts, ids, framnum = detect_aruco(cap)
        # print('run time %.3fs'%(time.time() - s))
        print("IDs:   ",ids)
        # print("TS is:   ",Ts)

        if len(Ts)>0 and ids[0] == 0:
            print(f"Translation x:{round(-Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")

            # Publish the message
            displacement_msg = Point()
            displacement_msg.x =  Ts[0][2, 3]
            displacement_msg.y = -Ts[0][0, 3]
            displacement_msg.z = Ts[0][1, 3]
      
            pub.publish(displacement_msg)
            rospy.loginfo("Published Point message: {}".format(displacement_msg))
        

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        rate.sleep()

    cv2.destroyAllWindows()
    release_camera(cap)



if __name__ == '__main__':
    
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

    print("DONE PICAM")

