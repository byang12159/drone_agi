import pyrealsense2 as rs
import cv2
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline = rs.pipeline()

pipe_profile = pipeline.start(config)

frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
# Intrinsics & Extrinsics
depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
color_to_depth_extrin = color_frame.profile.get_extrinsics_to(depth_frame.profile)

print ("\n Depth intrinsics: " + str(depth_intrin))
print ("\n Color intrinsics: " + str(color_intrin))
print ("\n Depth to color extrinsics: " + str(depth_to_color_extrin))

# Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
depth_sensor = pipe_profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print ("\n\t depth_scale: " + str(depth_scale))

depth_pixel = [200, 200]   # Random pixel
print ("\n\t depth_pixel: " + str(depth_pixel))
depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_scale)
color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
print ("\n\t color_pixel: " + str(color_pixel))

d = cv2.circle(color_frame, (200,200), 5, (0,255,0),2)
c = cv2.circle(depth_frame, ())