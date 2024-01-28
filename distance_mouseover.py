import cv2
import pyrealsense2

from realsense_depth import *

# Initialize Camera Intel Realsense
dc = DepthCamera()
ret, depth_frame, color_frame = dc.get_frame()
