#!/usr/bin/env python3
from line_profiler import LineProfiler
import rospy
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Point
# import geometry_msgs.msg as geometry_msgs

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

# marker_size = 53.5 #mm
# marker_size = 118 #mm
marker_size = 182.5

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
    # starttimearuco = time.time()
    def cleanup_cap():
        pass
    if cap is None:
        cap = get_camera()
        cleanup_cap = lambda : cap.release()

    ret, frame = cap.read()
    # timenow = time.time()
    # print("time1", timenow-starttimearuco)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)

    markerCorners, markerIds, rejectedCandidates= aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # timenow = time.time()
    # print("time2", timenow-starttimearuco)

    detection = False
    Ts = None

    if markerIds is not None :        
        for i,id in enumerate(markerIds):
            if id[0] == 85:
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners[i], markerLength=marker_size, cameraMatrix=camera_mtx, distCoeffs=distortion_param)
                rvec = rvecs[0]
                tvec = tvecs[0]

                if save or visualize:
                    frame = aruco.drawDetectedMarkers( frame, markerCorners, markerIds )
                    frame = cv2.drawFrameAxes(frame, camera_mtx, distortion_param, rvec, tvec,length = 100, thickness=6)
            
                # rotation_mtx, jacobian = cv2.Rodrigues(rvec)
                # translation = tvec
                Ts = np.identity(4)
                # Ts[:3, :3] = rotation_mtx
                Ts[:3, 3] = tvec / 1000.
                detection = True

    # if save is not None:
    #     savefile = "image/{}.jpg".format(time.time())
    #     cv2.imwrite(savefile, frame)

    if visualize:
        cv2.imshow("camera view", frame)

    # timenow = time.time()
    # print("time3", timenow-starttimearuco)
    return Ts, detection

def get_camera():
    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 100) 
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)

    cv2.waitKey(500)
    return cap

def shutdown_hook(cap):
    release_camera(cap)
    cv2.destroyAllWindows()

def release_camera(cap):
    cap.release()

def publisher():
    rospy.init_node('picam', anonymous=True)
    pub = rospy.Publisher("/leader_waypoint", Point, queue_size=3)
    pub_detection = rospy.Publisher("/aruco_detection", Bool, queue_size=1)
    rate = rospy.Rate(60)  # Hz

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()

    cap = get_camera()
    time.sleep(2)

    rospy.on_shutdown(lambda: shutdown_hook(cap))
    count = 0

    while not rospy.is_shutdown():
        # starttime = time.time()
        Ts, detection_check = detect_aruco(cap, aruco_dict, parameters, save="image/{}.jpg".format(count))

        # print("time exit: ",time.time()-starttime)
        
        if detection_check:
            # print(f"Translation x:{round(-Ts[0, 3],2)} y:{round(Ts[1, 3],2)} z:{round(Ts[2, 3],2)}")
            displacement_msg = Point()
            displacement_msg.x =  Ts[2, 3]
            displacement_msg.y = -Ts[0, 3]
            displacement_msg.z = -Ts[1, 3]
            
            detection_msg = Bool(data=True)
            pub_detection.publish(detection_msg)

            pub.publish(displacement_msg)
        else:        
            detection_msg = Bool(data=False)
            pub_detection.publish(detection_msg)
        
            # rospy.loginfo("Aruco Detection, Published Point message: {}".format(displacement_msg))
        
        
        # key = cv2.waitKey(1) & 0xFF
        # if key == ord('q'): break

        # runtime = time.time() - starttime
        # print('run time %.3fs'%(runtime))
        count +=1
        rate.sleep()
    

if __name__ == '__main__':

    try:
        publisher()
    except rospy.ROSInterruptException:
        pass


