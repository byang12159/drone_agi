#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import Point
import time

def talker():
    # Initialize the node with the name 'talker'
    rospy.init_node('talker', anonymous=True)
    
    # Create a publisher that will publish to the '/my_topic' topic using the String message type
    pub = rospy.Publisher('/fake_waypoint', Point, queue_size=10)
    
    # Set the rate at which the messages will be published (10 Hz)
    rate = rospy.Rate(10)  # 10hz
    count = 0

    while not rospy.is_shutdown():

        if count%2==0:
            displacement_msg = Point()
            displacement_msg.x =  0.0
            displacement_msg.y = 0.5
            displacement_msg.z = 1.0
        else:
            displacement_msg = Point()
            displacement_msg.x =  0.0
            displacement_msg.y = -0.5
            displacement_msg.z = 1.0
    
        pub.publish(displacement_msg)

        count +=1

        rospy.loginfo("Publishing point {}".format(displacement_msg))
        time.sleep(5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
