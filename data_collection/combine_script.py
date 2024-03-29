import cv2
from pymavlink import mavutil
import sys, os
import pyrealsense2 as rs
import numpy as np
import pickle
import time

# Open a video capture object for the USB camera (usually 0 or 1 depending on your setup)
cap = cv2.VideoCapture(0)

# Check if the camera is opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set the frame width and height (adjust as needed)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Specify the folder to save images
output_folder_1 = "captured_frames_raw"

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder_1):
    os.makedirs(output_folder_1)
    print(f"Created folder: {output_folder_1}")

# Specify the folder to save images
output_folder_2 = "realsense_color"

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder_2):
    os.makedirs(output_folder_2)
    print(f"Created folder: {output_folder_2}")

# Specify the folder to save images
output_folder_3 = "realsense_depth"

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder_3):
    os.makedirs(output_folder_3)
    print(f"Created folder: {output_folder_3}")
output_folder_4 = "detection_images"
# Create the output folder if it doesn't exist
if not os.path.exists(output_folder_4):
    os.makedirs(output_folder_4)
    print(f"Created folder: {output_folder_4}")

# Variable to track the number of frames captured
frame_count = 0

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
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
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

print(rs.intrinsics())
f = open("x_y_algorithm_data", "w")
f.write("x   y   depth\n")
f.close()
<<<<<<< HEAD
=======
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = cv2.flip(frame, 0)
    # Check if the frame is read successfully
    if not ret:
        print("Error: Couldn't read frame.")
        break
>>>>>>> 566a390adbfccd35362b702536c5d4790ef1301c

f = open("x_y_algorithm_data_vicon", "w")
f.write("x   y   depth\n")
f.close()

master = mavutil.mavlink_connection('udpin:0.0.0.0:10085')

while True:
    msg = master.recv_match(blocking=False)
    if not msg:
            continue
    if msg.get_type() == 'LOCAL_POSITION_NED_COV':
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
    
<<<<<<< HEAD
=======
    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame() #is a 640x360 depth image
    depth_frame = cv2.flip(depth_frame, 0)
    depth_frame_image = np.asanyarray(depth_frame.get_data())
    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()


    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
    print(color_intrin)

    # Validate that both frames are valid
    if not aligned_depth_frame or not color_frame:
        continue

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = cv2.flip(depth_image, 0)
    color_image = cv2.flip(color_image, 0)
    # Remove background - Set pixels further than clipping_distance to grey
    # grey_color = 153
    # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
    # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

    # Render images:
    #   depth align to color on left
    #   depth on right
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    #images = np.hstack((bg_removed, depth_colormap))
    images = np.hstack((color_image, depth_colormap))
    #images = np.hstack((color_image, depth_image))

    cv2.imwrite("realsense_color.jpg", color_image)
    cv2.imwrite("realsense_depth.jpg", depth_colormap)

    greenLower = (40, 50, 60)
    greenUpper = (68, 255, 255)
    img = color_image
    blurred = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    x_coordiantes = []
    y_coordiantes = []
    ret,thresh = cv2.threshold(mask,127,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        M = cv2.moments(c)

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        x_coordiantes.append(cX)
        y_coordiantes.append(cY)
    f = 489.5384
    cx = 307.82908611
    cy = 244.48380801
    if len(x_coordiantes) < 2 or len(y_coordiantes) < 2:
        continue
    drone_center_x = int((x_coordiantes[0]+x_coordiantes[1])/2)
    drone_center_y = int((y_coordiantes[0]+y_coordiantes[1])/2)
    depth = depth_frame_image[drone_center_x,drone_center_y]
    dist = np.sqrt((cx-x_coordiantes[0])**2+(cy-y_coordiantes[0]**2))
    horizontal_depth = np.sqrt(depth**2-dist**2)
    x = ((drone_center_x)/f)*horizontal_depth
    y = ((drone_center_y)/f)*horizontal_depth
    f = open("x_y_algorithm_data", "a")
    f.write(str(x))
    f.write(str(y))
    f.write(str(horizontal_depth))
    f.write("\n")
    f.close()

    frame_count += 1
    file_path = os.path.join(output_folder_1, f"captured_frame_{frame_count}.jpg")
    cv2.imwrite(file_path, frame)
    print(f"Frame {frame_count} captured and saved to: {file_path}")

    file_path = os.path.join(output_folder_2, f"realsense_color_{frame_count}.jpg")
    cv2.imwrite(file_path, frame)
    print(f"Frame {frame_count} captured and saved to: {file_path}")

    file_path = os.path.join(output_folder_3, f"realsense_depth_{frame_count}.jpg")
    cv2.imwrite(file_path, frame)
    print(f"Frame {frame_count} captured and saved to: {file_path}")
>>>>>>> 566a390adbfccd35362b702536c5d4790ef1301c

        # Capture frame-by-frame
        ret, frame = cap.read()

        # Check if the frame is read successfully
        if not ret:
            print("Error: Couldn't read frame.")
            break

        # Display the frame
        cv2.imshow("USB Camera", frame)

        # Check for key press (27 is the ASCII code for the 'Esc' key)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("Exiting...")
            break

        
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame() #is a 640x360 depth image
        depth_frame_image = np.asanyarray(depth_frame.get_data())
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()


        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        print(color_intrin)

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        # grey_color = 153
        # depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        # bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #images = np.hstack((bg_removed, depth_colormap))
        images = np.hstack((color_image, depth_colormap))
        #images = np.hstack((color_image, depth_image))

        cv2.imwrite("realsense_color.jpg", color_image)
        cv2.imwrite("realsense_depth.jpg", depth_colormap)

        greenLower = (44, 50, 60)
        greenUpper = (68, 255, 255)
        img = color_image
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        x_coordiantes = []
        y_coordiantes = []
        # cv2.imshow("Image",mask)
        # cv2.waitKey(0)
        ret,thresh = cv2.threshold(mask,127,255,0)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            M = cv2.moments(c)

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            x_coordiantes.append(cX)
            y_coordiantes.append(cY)
        f = 604.735
        cx = 318.968
        cy = 244.676
        if len(x_coordiantes) < 2 or len(y_coordiantes) < 2:
            continue
        drone_center_x = int((x_coordiantes[0]+x_coordiantes[1])/2)
        drone_center_y = int((y_coordiantes[0]+y_coordiantes[1])/2)
        depth = depth_frame_image[y_coordiantes[0],x_coordiantes[0]]
        dist = np.sqrt((cx-x_coordiantes[0])**2+(cy-y_coordiantes[0])**2)
        horizontal_depth = np.sqrt(depth**2-dist**2)
        x = ((drone_center_x)/f)*horizontal_depth
        y = ((drone_center_y)/f)*horizontal_depth
        print(x,y,depth, horizontal_depth)
        calculated_coordinates = (x/1000,y/1000,depth/1000, horizontal_depth/1000)
        f = open("x_y_algorithm_data", "a")
        f.write(str(calculated_coordinates))
        f.write("\n")
        f.close()
        f = open("x_y_algorithm_data_vicon", "a")
        vicon_coordinates = (data[0],data[1],data[2])
        f.write(str(vicon_coordinates))
        f.write("\n")
        f.close()
        frame_count += 1
        file_path = os.path.join(output_folder_1, f"captured_frame_{frame_count}.jpg")
        cv2.imwrite(file_path, frame)
        print(f"Frame {frame_count} captured and saved to: {file_path}")
        cv2.circle(color_image,(x_coordiantes[0],y_coordiantes[0]),5,(255,255,255),-1)
        cv2.circle(color_image,(int(cx),int(cy)),5,(255,255,255),-1)
        cv2.circle(depth_colormap,(x_coordiantes[0],y_coordiantes[0]),5,(255,255,255),-1)
        file_path = os.path.join(output_folder_2, f"realsense_color_{frame_count}.jpg")
        cv2.imwrite(file_path, color_image)
        print(f"Frame {frame_count} captured and saved to: {file_path}")

        file_path = os.path.join(output_folder_3, f"realsense_depth_{frame_count}.jpg")
        cv2.imwrite(file_path, depth_colormap)
        print(f"Frame {frame_count} captured and saved to: {file_path}")
        file_path = os.path.join(output_folder_4, f"ball_detection{frame_count}.jpg")
        cv2.imwrite(file_path, mask)
        print(f"Frame {frame_count} captured and saved to: {file_path}")

    elif msg.get_type() == 'ATT_POS_MOCAP':
            pass
    cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
    cv2.imshow('Align Example', images)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
pipeline.stop()
