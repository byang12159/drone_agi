import pyrealsense2 as rs
import numpy as np
import cv2
import os

output_folder_1 = "checkerboard_D435i"

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder_1):
    os.makedirs(output_folder_1)
    print(f"Created folder: {output_folder_1}")

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

# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

frame_count = 0

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
output_folder_2 = "checkerboard_picam"

# Create the output folder if it doesn't exist
if not os.path.exists(output_folder_2):
    os.makedirs(output_folder_2)
    print(f"Created folder: {output_folder_2}")

# Variable to track the number of frames captured
frame_count = 0


try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        # depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not color_frame:
            break

        ret, frame = cap.read()
        if not ret:
            print("Error: Couldn't read frame.")
            break
        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # Show images
        images = np.hstack((color_image, frame))
        cv2.namedWindow('Left: RealSense   Right: Picam', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)

        # Check for key press (27 is the ASCII code for the 'Esc' key)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            print("Exiting...")
            break
        elif key == ord('c'):
            # Save the frame when the 'c' key is pressed
            frame_count += 1
            file_path1 = os.path.join(output_folder_1, f"captured_frame_{frame_count}.jpg")
            cv2.imwrite(file_path1, color_image)
            print(f"Frame {frame_count} captured and saved to: {file_path1}")

            file_path2 = os.path.join(output_folder_2, f"captured_frame_{frame_count}.jpg")
            cv2.imwrite(file_path2, frame)
            print(f"Frame {frame_count} captured and saved to: {file_path2}")

finally:
    # Stop streaming
    pipeline.stop()

    cap.release()
    cv2.destroyAllWindows()
