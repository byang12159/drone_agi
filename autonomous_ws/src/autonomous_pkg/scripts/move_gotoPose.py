#!/usr/bin/env python3
import rospy
import sys
import logging
import time
from std_msgs.msg import Float32, String
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs


def talker():
    # Initialize the ROS node named 'talker'
    rospy.init_node('talker', anonymous=True)
    
    quad_namespace = "kingfisher"
    pub = rospy.Publisher(quad_namespace+"/agiros_pilot/go_to_pose", 
                        geometry_msgs.PoseStamped, 
                        queue_size=1)
    
    # Set the rate at which to publish messages (10 Hz)
    rate = rospy.Rate(1)  # 10 Hz

    count = 0

    while not rospy.is_shutdown() and count <=5:

        if count%2 == 0:
            target = geometry_msgs.PoseStamped()
            target.pose.position.x = float(0.0)
            target.pose.position.y = float(0.5)
            target.pose.position.z = float(1.0) 
            target.pose.orientation.w = float(1.0)
            target.pose.orientation.z = float(0.0)

            pub.publish(target)
        else:
            target = geometry_msgs.PoseStamped()
            target.pose.position.x = float(0.0)
            target.pose.position.y = float( -0.5 )
            target.pose.position.z = float(1.0) 
            target.pose.orientation.w = float(1.0)
            target.pose.orientation.z = float(0.0)

            pub.publish(target)
       
        
        # Sleep to maintain the loop rate
        rate.sleep()
        count += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
