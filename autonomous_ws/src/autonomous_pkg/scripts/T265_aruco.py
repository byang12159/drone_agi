#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import cv2.aruco as aruco
from math import tan, pi
from picam_aruco import detect_aruco
from geometry_msgs.msg import Point
import time



class ImageSubscriber:
    def __init__(self):
        
        # Set up an OpenCV window to visualize the results
        # WINDOW_TITLE = 'Realsense'
        # cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

        # Configure the OpenCV stereo algorithm. See
        # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
        # description of the parameters
        window_size = 8
        self.min_disp = 0
        # must be divisible by 16
        self.num_disp = 112 - self.min_disp
        self.max_disp = self.min_disp + self.num_disp
        self.stereo = cv2.StereoSGBM_create(minDisparity = self.min_disp,
                                        numDisparities = self.num_disp,
                                        blockSize = 16,
                                        P1 = 8*3*window_size**2,
                                        P2 = 32*3*window_size**2,
                                        disp12MaxDiff = 1,
                                        uniquenessRatio = 10,
                                        speckleWindowSize = 100,
                                        speckleRange = 32)

        # Translate the intrinsics from librealsense into OpenCV

        self.K_left  = np.array([[284.93951416015625, 0.0, 419.4148864746094],
                                 [0.0, 284.89691162109375, 403.57611083984375],
                                 [0.0, 0.0, 1.0]])
        self.D_left  = np.array([-0.008140385150909424, 0.04617336019873619, -0.04361281171441078, 0.008108303882181644])
        K_right = np.array([[285.3443908691406, 0.0, 409.6065979003906],
                            [0.0, 285.5675048828125, 407.6676940917969],
                            [0.0, 0.0, 1.0]])                                             
        D_right = np.array([-0.0033637969754636288, 0.036712050437927246, -0.034943610429763794, 0.005407519172877073])
        (width, height) = (848,800)

        # Get the relative extrinsics between the left and right camera
        # (R, T) = get_extrinsics(streams["left"], streams["right"])

        R = np.array([[ 0.99998,         -0.00184385,      -0.00602005 ],
        [0.00183268,       0.999997,        -0.00186   ],
        [0.00602346,       0.00184893,       0.99998]])
        T = np.array([ -0.0637905895709991,  6.11625218880363e-05,  -0.00013126520627327])

        # We need to determine what focal length our undistorted images should have
        # in order to set up the camera matrices for initUndistortRectifyMap.  We
        # could use stereoRectify, but here we show how to derive these projection
        # matrices from the calibration and a desired height and field of view

        # We calculate the undistorted focal length:
        #
        #         h
        # -----------------
        #  \      |      /
        #    \    | f  /
        #     \   |   /
        #      \ fov /
        #        \|/
        stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
        stereo_height_px = 300          # 300x300 pixel stereo output
        stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

        # We set the left rotation to identity and the right rotation
        # the rotation between the cameras
        R_left = np.eye(3)
        R_right = R

        # The stereo algorithm needs max_disp extra pixels in order to produce valid
        # disparity on the desired output region. This changes the width, but the
        # center of projection should be on the center of the cropped image
        stereo_width_px = stereo_height_px + self.max_disp
        stereo_size = (stereo_width_px, stereo_height_px)
        stereo_cx = (stereo_height_px - 1)/2 + self.max_disp
        stereo_cy = (stereo_height_px - 1)/2

        # Construct the left and right projection matrices, the only difference is
        # that the right projection matrix should have a shift along the x axis of
        # baseline*focal_length
        P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                            [0, stereo_focal_px, stereo_cy, 0],
                            [0,               0,         1, 0]])
        P_right = P_left.copy()
        P_right[0][3] = T[0]*stereo_focal_px

        # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
        # since we will crop the disparity later
        Q = np.array([[1, 0,       0, -(stereo_cx - self.max_disp)],
                        [0, 1,       0, -stereo_cy],
                        [0, 0,       0, stereo_focal_px],
                        [0, 0, -1/T[0], 0]])

        # Create an undistortion map for the left and right camera which applies the
        # rectification and undoes the camera distortion. This only has to be done
        # once
        m1type = cv2.CV_32FC1
        (self.lm1, self.lm2) = cv2.fisheye.initUndistortRectifyMap(self.K_left, self.D_left, R_left, P_left, stereo_size, m1type)
        (self.rm1, self.rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
        self.undistort_rectify = {"left"  : (self.lm1, self.lm2),
                            "right" : (self.rm1, self.rm2)}
        
        print("LM",self.lm1)

        # Initialize the ROS node
        rospy.init_node('image_subscriber', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        pub = rospy.Publisher("/leader_waypoint", Point, queue_size=1)
        rate = rospy.Rate(40)  # 1 Hz

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/t265/fisheye1/image_raw', Image, self.image_callback)



    
    def image_callback(self, msg):
        try:
            print("IN callback")
            startcall = time.time()
            frame_copy = np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width))
            cv2.imwrite("t265_image_raw.png",frame_copy)

            # Undistort and crop the center of the frames
            center_undistorted = {"left" : cv2.remap(src = frame_copy,
                                            map1 = self.lm1,
                                            map2 = self.lm2,
                                            interpolation = cv2.INTER_LINEAR),
                                    "right" : cv2.remap(src = frame_copy,
                                            map1 = self.rm1,
                                            map2 = self.rm2,
                                            interpolation = cv2.INTER_LINEAR)}

            # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
            disparity = self.stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

            # re-crop just the valid part of the disparity
            disparity = disparity[:,self.max_disp:]

            # convert disparity to 0-255 and color it
            disp_vis = 255*(disparity - self.min_disp)/ self.num_disp
            disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
            color_image = cv2.cvtColor(center_undistorted["left"][:,self.max_disp:], cv2.COLOR_GRAY2RGB)

            cv2.imwrite("t265_image.png",color_image)
            Ts,Ids = self.detect_aruco(color_image)
            # cv2.imshow("camera view", color_image)
            print("time call:",time.time()-startcall)
            print("ids",Ids)
            print("TS:\n",Ts[0][:,3])

            # cv2.imshow("realsense view", color_image)
            # cv2.waitKey(1) 

    
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {}".format(e))
            return

    def run(self):
        # Keep the node running
        rospy.spin()
    def detect_aruco(self,frame, save=None, visualize=True):
        
        starttimearuco = time.time()

        timenow = time.time()
        print("time1", timenow-starttimearuco)

        # width = int(frame.shape[1] * scale)
        # height = int(frame.shape[0] * scale)
        # dim = (width, height)
        # frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)


        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters_create()
        markerCorners, markerIds, rejectedCandidates= aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        timenow = time.time()
    
        dist_coeffs = np.zeros((5, 1))
        marker_size = 53.5 #mm

        Ts = []
        ids = []
        if markerIds is not None :
            rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, markerLength=marker_size, cameraMatrix=self.K_left, distCoeffs=dist_coeffs)
            # rvecs, tvecs, objpoints = aruco.estimatePoseSingleMarkers(markerCorners, marker_size, , )
            for i in range(len(markerIds)):
                # Unpacking the compounded list structure
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                print("tvec",tvec)
                # print("Rvec",rvec)
                # print("Tvec",tvec)
                ids.append(markerIds[i][0])
                if save or visualize:
                    frame = aruco.drawDetectedMarkers( frame, markerCorners, markerIds )
                    frame = cv2.drawFrameAxes(frame, self.K_left, dist_coeffs, rvec, tvec,length = 100, thickness=6)
            
                rotation_mtx, jacobian = cv2.Rodrigues(rvec)
                translation = tvec
                T = np.identity(4)
                T[:3, :3] = rotation_mtx
                T[:3, 3] = translation / 1000.
                Ts.append(T)

        if save:
            cv2.imwrite(save, frame)

        if visualize:
            cv2.imshow("realsense view", frame)
            cv2.waitKey(1)

        timenow = time.time()
        print("time3", timenow-starttimearuco)
        # time.sleep(1)
        return Ts, ids

if __name__ == '__main__':
    # Create an instance of the ImageSubscriber class
    image_subscriber = ImageSubscriber()

    # Run the subscriber
    image_subscriber.run()

    # Destroy the OpenCV window on exit
    cv2.destroyAllWindows()

