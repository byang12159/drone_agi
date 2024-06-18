#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ImageSubscriber:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_subscriber', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/t265/fisheye1/image_raw', Image, self.image_callback)

        # Create a window to display the image
        cv2.namedWindow("Image Window", cv2.WINDOW_NORMAL)

    def image_to_numpy(self, image_msg):
        # Extract image data from the Image message
        img_data = image_msg.data
        
        # Convert image data to a NumPy array
        np_arr = np.frombuffer(img_data, np.uint8)
        
        # Reshape NumPy array to a 2D or 3D array depending on the image encoding
        if image_msg.encoding == 'mono8':
            # Grayscale image
            np_arr = np_arr.reshape((image_msg.height, image_msg.width))
        elif image_msg.encoding == 'rgb8' or image_msg.encoding == 'bgr8':
            # Color image
            np_arr = np_arr.reshape((image_msg.height, image_msg.width, 3))
        else:
            # Handle other encodings if needed
            raise NotImplementedError(f"Unsupported image encoding: {image_msg.encoding}")
        
        return np_arr
    
    def image_callback(self, msg):
        try:
            cv_image = np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width))

            print(type(cv_image), cv_image.shape)
            np.save('cv_array.npy', cv_image)
            # print("DATA: ",msg.width, msg.height,msg.encoding)
            # # print(msg.data)
            # image_a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height,msg.width)
            # cv_image = cv2.normalize(image_a, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            # # print(cv_image[0])
            # print(cv_image[10])

            np.savetxt('array_data.txt', cv_image, fmt='%d')
            
            # Debug: Check the shape and type of the image
            rospy.loginfo("Image shape: {}".format(cv_image.shape))
            rospy.loginfo("Image dtype: {}".format(cv_image.dtype))

            # Debug: Check the min and max values of the image
            rospy.loginfo("Image min value: {}".format(np.min(cv_image)))
            rospy.loginfo("Image max value: {}".format(np.max(cv_image)))

            # cv2.imshow("opencv ros image", image)
            # # Convert the ROS Image message to a CV image
            # # Use "mono8" encoding for monochrome images
            # cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {}".format(e))
            return

        # Display the image
        cv2.imshow("Image Window", cv_image)

        # Wait for a key event for 1ms
        cv2.waitKey(0)
        cv2.destroyAllWindows()  # Close the window

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    # Create an instance of the ImageSubscriber class
    image_subscriber = ImageSubscriber()

    # Run the subscriber
    image_subscriber.run()

    # Destroy the OpenCV window on exit
    cv2.destroyAllWindows()

