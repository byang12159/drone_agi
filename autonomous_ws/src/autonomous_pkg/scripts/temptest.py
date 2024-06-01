#!/usr/bin/env python3
# import rospy
# import time
# from std_msgs.msg import Float32, String
# import geometry_msgs.msg as geometry_msgs
# import agiros_msgs.msg as agiros_msgs
# import sys

# pos = None

# def callback(data):
#     # This function is called every time a new Point message is received
#     # Republish the received point message to a new topic

#     # print(data.pose.position)
#     # rospy.signal_shutdown("Data received, shutting down.")
#     global pos

#     pos = data.pose.position.x

# def publisher():
#     global pos
#     rospy.init_node('picam_test', anonymous=True)
#     rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, callback)
#     print(pos)
#     rospy.spin()

# if __name__ == '__main__':

#     import sys

#     # Open a file in write mode
#     with open('output.txt', 'w') as f:
#         # Redirect standard output to the file
#         sys.stdout = f
        
#         # Example print statements
#         print("This will be written to the file.")
#         print("So will this.")

#         print("DONE PICAM TEST")
#     # Reset standard output to its original value
#     sys.stdout = sys.__stdout__

#     try:
#         publisher()
#     except rospy.ROSInterruptException:
#         pass

import numpy.linalg as la
import numpy as np
a = np.array([1,4,3])
b = np.array([1,0,0])

print(la.norm(a-b))



# import rospy
# import logging

# def configure_logging():
#     # Create a logger
#     logger = logging.getLogger('rosout')
#     logger.setLevel(logging.INFO)

#     # Create a file handler
#     log_file = 'logfile.log'  # Update this path
#     fh = logging.FileHandler(log_file)
#     fh.setLevel(logging.INFO)

#     # Create a formatter and set it for the handler
#     formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
#     fh.setFormatter(formatter)

#     # Add the file handler to the logger
#     logger.addHandler(fh)

# def simple_node():
#     rospy.init_node('simple_node', anonymous=True)
#     configure_logging()
#     rate = rospy.Rate(1)  # 1 Hz
#     while not rospy.is_shutdown():
#         rospy.loginfo("This is an info message")
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         simple_node()
#     except rospy.ROSInterruptException:
#         pass

