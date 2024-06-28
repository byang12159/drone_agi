#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.window_name = 'Image Stream'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            #cv2.imwrite("testimage.png",cv_image)
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)  # Adjust the delay as needed
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")
            #print("error here")
    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        image_subscriber = ImageSubscriber()
        image_subscriber.run()
    except rospy.ROSInterruptException:
        pass

