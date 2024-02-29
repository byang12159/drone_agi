import numpy as np
import cv2
import cv2.aruco as aruco
import math
import time
import pyrealsense2 as rs
# from eulerconvert import rotationMatrixToEulerAngles

scale = 1.0

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


# marker size mm marker_size=83
def detect_aruco(camera_mtx, distortion_param,cap=None, save=None, visualize=False, marker_size=100):

    if cap is None:
        cap = get_camera()
        time.sleep(2)
        print("capstatus",cap)


    ret, frame = cap.read()
 
    frame = cv2.flip(frame,-1)
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dim = (width, height)
    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    # parameters = aruco.DetectorParameters_create()
    # markerCorners, markerIds, rejectedCandidates= aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)
    markerCorners, markerIds, rejectedCandidates= detector.detectMarkers(gray)
   
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

def detect_aruco_realsense(pipeline=None, align = None, save=None, visualize=True, marker_size=100):

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        return

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.circle(color_image, (200,200),5,(0,255,0),2)
    depth_image = cv2.circle(depth_image, (200,200),5,(0,255,0),2)

    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
    print(color_intrin)
    width = 640
    height = 480
    dim = (width, height)
    frame = cv2.resize(color_image, dim, interpolation = cv2.INTER_AREA)

    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    # parameters = aruco.DetectorParameters_create()
    # markerCorners, markerIds, rejectedCandidates= aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters =  cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)
    markerCorners, markerIds, rejectedCandidates= detector.detectMarkers(gray)
   
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
    return Ts, ids

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

def get_camera_realsense():
    
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    return pipeline

def release_camera(cap):
    cap.release()

def release_camera_realsense(pipeline):
    pipeline.stop()


if __name__ == "__main__":
#     x:-0.34 y:-0.31 z:0.92
# Translation x:0.16 y:0.24 z:0.57

    
    usb = True
    if usb == True:

        camera_mtx = np.array([[488.37405729 ,  0. ,        332.86930938],
                    [  0.   ,      490.86269977 ,234.46234393],
                    [  0.    ,       0.     ,      1.        ]])

        camera_mtx = camera_mtx * scale
        camera_mtx[2,2] = 1.

        distortion_param = np.array([-0.4253985 ,  0.24863036 ,-0.00162259 ,-0.003012  , -0.09376853])

        cam = get_camera()
        time.sleep(2)
        while True:
            s = time.time()
            Ts, ids = detect_aruco(camera_mtx, distortion_param,cam, visualize=True)
            print('run time %.3fs'%(time.time() - s))
            print("IDs:   ",ids)
            print("TS is:   ",Ts)
    
            if len(Ts)>0:
                print(f"Translation x:{round(-Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break

        cv2.destroyAllWindows()
        release_camera(cam)

    else:
        # camera_mtx = np.array([[907.1031494140625, 0.0, 638.451416015625],
        #                        [0.0, 907.1742553710938, 366.6768493652344],
        #                         [0.0, 0.0, 1.0]])
        camera_mtx = np.array([[604.735, 0.0, 318.968],
                        [0.0, 604.735, 244.451],
                        [0.0, 0.0, 1.0]])

        camera_mtx = camera_mtx * scale
        camera_mtx[2,2] = 1.

        distortion_param = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])


        pipeline= get_camera_realsense()

        # distortion_model: "plumb_bob"

        while True:
            s = time.time()
            Ts, ids = detect_aruco_realsense(pipeline, visualize=True)
            print('run time %.3fs'%(time.time() - s))
            print("IDs:   ",ids)
            print("TS is:   ",Ts)
    
            if len(Ts)>0:
                print(f"Translation x:{round(Ts[0][0, 3],2)} y:{round(Ts[0][1, 3],2)} z:{round(Ts[0][2, 3],2)}")
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
        cv2.destroyAllWindows()
        release_camera_realsense(pipeline)

 