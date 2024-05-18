# Simple example that shows the depth of a point by D435i

import cv2
import pyrealsense2
from realsense_D435i import *

point = (400,300)

def showdist(event, x,y,args,params):
    global point
    point = (x,y)

# Initialize Camera Intel Realsense
dc = DepthCamera()

cv2.namedWindow("color frame")
cv2.setMouseCallback("color frame", showdist)

while True:
    ret, depth_frame, color_frame = dc.get_frame()

    cv2.circle(color_frame, point, 4, (0,0,255))
    distance = depth_frame[point[1],point[0]]

    cv2.putText(color_frame, "{}mm".format(distance), (point[0],point[1]-20), cv2.FONT_HERSHEY_PLAIN, 2,(0,0,0),2)

    cv2.imshow("depth", depth_frame)
    cv2.imshow("color frame", color_frame)
    key = cv2.waitKey(1)
    if key == 27:
        break
