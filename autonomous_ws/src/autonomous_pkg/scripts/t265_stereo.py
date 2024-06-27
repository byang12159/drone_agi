 #!/usr/bin/python3
import cv2
import numpy as np
from math import tan, pi
from picam_aruco import detect_aruco
 

try:
    # Set up an OpenCV window to visualize the results
    WINDOW_TITLE = 'Realsense'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

    # Configure the OpenCV stereo algorithm. See
    # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    # description of the parameters
    window_size = 8
    min_disp = 0
    # must be divisible by 16
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                    numDisparities = num_disp,
                                    blockSize = 16,
                                    P1 = 8*3*window_size**2,
                                    P2 = 32*3*window_size**2,
                                    disp12MaxDiff = 1,
                                    uniquenessRatio = 10,
                                    speckleWindowSize = 100,
                                    speckleRange = 32)

    # Translate the intrinsics from librealsense into OpenCV
    K_left  = np.array([[284.93951416015625, 0.0, 419.4148864746094],[0.0, 284.89691162109375, 403.57611083984375],[0.0, 0.0, 1.0]])
    D_left  = np.array([-0.008140385150909424, 0.04617336019873619, -0.04361281171441078, 0.008108303882181644])
    K_right = np.array([[285.3443908691406, 0.0, 409.6065979003906],[0.0, 285.5675048828125, 407.6676940917969],[0.0, 0.0, 1.0]])                                             
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
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
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
    Q = np.array([[1, 0,       0, -(stereo_cx - max_disp)],
                    [0, 1,       0, -stereo_cy],
                    [0, 0,       0, stereo_focal_px],
                    [0, 0, -1/T[0], 0]])

    # Create an undistortion map for the left and right camera which applies the
    # rectification and undoes the camera distortion. This only has to be done
    # once
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left"  : (lm1, lm2),
                        "right" : (rm1, rm2)}

    print("lm1:",lm1)
    print(undistort_rectify)
    mode = "stack"

    frame_copy = np.load('cv_array.npy')
    # Undistort and crop the center of the frames
    center_undistorted = {"left" : cv2.remap(src = frame_copy,
                                    map1 = undistort_rectify["left"][0],
                                    map2 = undistort_rectify["left"][1],
                                    interpolation = cv2.INTER_LINEAR),
                            "right" : cv2.remap(src = frame_copy,
                                    map1 = undistort_rectify["right"][0],
                                    map2 = undistort_rectify["right"][1],
                                    interpolation = cv2.INTER_LINEAR)}

    # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
    disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

    # re-crop just the valid part of the disparity
    disparity = disparity[:,max_disp:]

    # convert disparity to 0-255 and color it
    disp_vis = 255*(disparity - min_disp)/ num_disp
    disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
    color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)

    # Ts, ids, framnum = detect_aruco(color_image)
    # print("TS",Ts)
    if mode == "stack":
        cv2.imshow(WINDOW_TITLE, color_image)
    if mode == "overlay":
        ind = disparity >= min_disp
        color_image[ind, 0] = disp_color[ind, 0]
        color_image[ind, 1] = disp_color[ind, 1]
        color_image[ind, 2] = disp_color[ind, 2]
        cv2.imshow(WINDOW_TITLE, color_image)

    key = cv2.waitKey(0)
    cv2.destroyAllWindows()  # Close the window
finally:
    pass
    
