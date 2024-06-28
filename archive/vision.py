#!/usr/bin/env python
import pickle
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from message_filters import TimeSynchronizer, Subscriber
import numpy as np 
from cv_bridge import CvBridge
import cv2
import os
import cv2



def callback(data1, data2, cap):
    # Your synchronized callback function
    # Process data1 and data2 together
    rospy.loginfo("Recieving images")

    # Images for T265
    cv_image1 = bridge.imgmsg_to_cv2(data1,"passthrough")
    image1_filename ="Fisheye1/Image1_{}.png".format(count)
    cv_image2 = bridge.imgmsg_to_cv2(data2,"passthrough")
    image2_filename ="Fisheye2/Image2_{}.png".format(count)

    # capture frame for arducam
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to read frame")
        exit()
    arducam_filename = "arducam_{}".format(count)
    cv2.imshow("cam stream", frame)

    cv2.imwrite(arducam_filename, frame)
    cv2.imwrite(image1_filename, cv_image1)
    cv2.imwrite(image2_filename, cv_image2)
    count += 1

    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

def listener():
    rospy.init_node('odom_subscriber', anonymous=True)

    # Define your subscribers
    # rospy.Subscriber("/camera/odom/sample", Odometry, callback)
    sub1 = rospy.Subscriber("/camera/fisheye1/image_raw", Image)
    sub2 = rospy.Subscriber("/camera/fisheye2/image_raw", Image)

    # Open arducam stream
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("error: could not open arducam")
        exit()
    
    # Synchronize messages based on their timestamps
    ts = TimeSynchronizer([sub1, sub2, cap], 10)
    ts.registerCallback(callback)

    rospy.spin()

    cap.release()
    cv2.destroyAllWindows()


count = 0
if __name__ == '__main__':
    listener()
    
    # file_path = "camdata.pkl"
    # file_path2 = "imgdata.pkl"
    # with open(file_path, 'wb') as file:
    #     pickle.dump(information,file )
    #with open(file_path2, 'wb') as file:
    #    pickle.dump(information2,file)
    print("Saved Data")
