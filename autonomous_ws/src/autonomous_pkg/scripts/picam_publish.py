#!/usr/bin/env python3
from line_profiler import LineProfiler
import rospy
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import mask_detection
# import geometry_msgs.msg as geometry_msgs
import os
import opencv_gate

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
# marker_size = 182.5 #mm
marker_size = 125.2
marker_ID = 85


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



def cv_image_to_ros_image(cv_image):
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")  
    return ros_image

def detect_gate(cap=None, save=None, visualize=False):
    # starttimearuco = time.time()
    def cleanup_cap():
        pass
    if cap is None:
        cap = get_camera()
        cleanup_cap = lambda : cap.release()

    ret, frame = cap.read()
    marked_frame, gate_pos = opencv_gate.process_frame(frame)
    if len(gate_pos) < 3:
        print("No gate found")
        gate_pos = None
    else:
        print(gate_pos)
    return frame, gate_pos
    # timenow = time.time()
    # print("time1", timenow-starttimearuco)

    # edges_red = mask_detection.process_frame_red(frame)
    # contours_red, _ = cv2.findContours(edges_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # ellipses_red = mask_detection.detect_ellipses_in_contours(contours_red)

    # ellipses = ellipses_red
    # ellipses_pair, radius_pair = mask_detection.select_best_double_ellipses(ellipses)
    # depth = None
    # center = None
    # gate_pos = None
    # if len(ellipses_pair) > 0:
    #     if radius_pair[0] > radius_pair[1]:
    #         ellipse = ellipses_pair[0]
    #         ellipse_2 = ellipses_pair[1]
    #     else:
    #         ellipse = ellipses_pair[1]
    #         ellipse_2 = ellipses_pair[0]

    #     if ellipse:
    #         ellipse_points_1 = cv2.ellipse2Poly(
    #             center=(int(ellipse[0][0]), int(ellipse[0][1])),
    #             axes=(int(ellipse[1][0] / 2), int(ellipse[1][1] / 2)),
    #             angle=int(ellipse[2]),
    #             arcStart=0,
    #             arcEnd=360,
    #             delta=1
    #         )
    #         x_min_1, y_min_1, width_rect_1, height_rect_1 = cv2.boundingRect(ellipse_points_1)

    #         cv2.rectangle(frame, (x_min_1, y_min_1), (x_min_1 + width_rect_1, y_min_1 + height_rect_1), (0, 255, 0), 2)
    #         center_1 = (x_min_1+width_rect_1/2.0,y_min_1+height_rect_1/2.0)
    #         depth_1 = mask_detection.calculate_depth(height_rect_1)


    #         ellipse_points_2 = cv2.ellipse2Poly(
    #             center=(int(ellipse_2[0][0]), int(ellipse_2[0][1])),
    #             axes=(int(ellipse_2[1][0] / 2), int(ellipse_2[1][1] / 2)),
    #             angle=int(ellipse_2[2]),
    #             arcStart=0,
    #             arcEnd=360,
    #             delta=1
    #         )
    #         x_min_2, y_min_2, width_rect_2, height_rect_2 = cv2.boundingRect(ellipse_points_2)

    #         cv2.rectangle(frame, (x_min_2, y_min_2), (x_min_2 + width_rect_2, y_min_2 + height_rect_2), (0, 255, 0), 2)
    #         center_2 = (x_min_2+width_rect_2/2.0,y_min_2+height_rect_2/2.0)
    #         depth_2 = mask_detection.calculate_depth(height_rect_2*mask_detection.scaling_ratio)

    #         center = ((center_1[0]+center_2[0])/2.0,(center_1[1]+center_2[1])/2.0)
    #         if ellipse == ellipse_2:
    #             depth = depth_1
    #         else:
    #             depth = (depth_1 + depth_2)/2
    #         gate_pos = [center[0],center[1],depth]
    #         cv2.putText(frame, f"Gate (Center: {gate_pos[0]},{gate_pos[1]}, Depth: {gate_pos[2]}):", (x_min_1, y_min_1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # if gate_pos is None:
    #     print("No gate found")
    # else:
    #     print(gate_pos)
    # if save is not None:
    #     # savefile = "image/{}.jpg".format(time.time())
    #     # print(os.getcwd()+save)
    #     cv2.imwrite(os.path.join("~",save), frame)

    # if visualize:
    #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #     cv2.imshow("camera view", gray)

    # # timenow = time.time()
    # # print("time3", timenow-starttimearuco)
    # return frame, gate_pos


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
    pub_image_raw = rospy.Publisher('/picam_raw', CompressedImage, queue_size=10)
    pub_gate = rospy.Publisher('/gate_pos', PointStamped, queue_size=3)
    pub_gate_detection = rospy.Publisher('/gate_detection', Bool, queue_size=1)
    rate = rospy.Rate(40)  # Hz
    # Drop from 60 to 40 to publish images

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()

    bridge = CvBridge()
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90] 


    cap = get_camera()
    time.sleep(2)

    rospy.on_shutdown(lambda: shutdown_hook(cap))
    count = 0
    print("STARTING")

    while not rospy.is_shutdown():
        starttime = time.time()
        # frame, gate_pos = detect_gate(cap, save="image/{}.jpg".format(count), visualize=False)
        frame, gate_pos = detect_gate(cap, visualize=False)

        # print("time exit: ",time.time()-starttime)

        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)
        if not success:
            rospy.logwarn("Failed to encode image")
            continue
        compressed_image_msg = CompressedImage()
        compressed_image_msg.header.stamp = rospy.Time.now()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = encoded_image.tobytes()
        pub_image_raw.publish(compressed_image_msg)

        if gate_pos is not None:
            gate_pos_msg = PointStamped()
            gate_pos_msg.header.stamp = rospy.Time.now()
            gate_pos_msg.point.x = gate_pos[0]
            gate_pos_msg.point.y = gate_pos[1]
            gate_pos_msg.point.z = gate_pos[2]
            pub_gate.publish(gate_pos_msg)
            pub_gate_detection.publish(Bool(data=True))
        else:
            pub_gate_detection.publish(Bool(data=False))

        # key = cv2.waitKey(1) & 0xFF
        # if key == ord('q'): break

        runtime = time.time() - starttime
        print('run time %.3fs'%(runtime))
        count +=1
        rate.sleep()
    

if __name__ == '__main__':

    try:
        publisher()
    except rospy.ROSInterruptException:
        pass


