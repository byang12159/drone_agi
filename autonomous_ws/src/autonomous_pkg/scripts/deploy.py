#!/usr/bin/env python3
import rospy
import sys
import logging
import time
from std_msgs.msg import Float32, String, Float64MultiArray
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs
import numpy as np
import numpy.linalg as la

current_pose = None
end_deploy = False
target_vicon_point = np.array([1.6, 2.8, 1.3])
# target_vicon_point = np.array([3.1, 2.3, 1.3])

def callback_state(data):
    global current_pose
    current_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

def vicon_callback(data):
    global current_pose, end_deploy, target_vicon_point

    current_vicon_point = np.array([data.data[0],data.data[1],data.data[2]])

    displacement = target_vicon_point-current_vicon_point
    if la.norm(displacement)<0.1:
        end_deploy = True
        return
    
    to_move = current_pose+displacement

    target = geometry_msgs.PoseStamped()
    target.pose.position.x = to_move[0]
    target.pose.position.y = to_move[1]
    target.pose.position.z = to_move[2]
    target.pose.orientation.w = float(1.0)
    target.pose.orientation.z = float(0.0)

    pub.publish(target)
    
    # rospy.loginfo("Aruco Pose : {}".format([data.x,data.y,data.z]))

    # rospy.loginfo("Publishing Pose to Ctrl: {}".format([target.pose.position.x,target.pose.position.y, target.pose.position.z]))
    print("loop")
    # sub_aruco.unregister()


if __name__ == '__main__':
    try:
        rospy.init_node('deploy_node', anonymous=True)
        rate = rospy.Rate(10)#Hz

        sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, callback_state)

        sub_aruco = rospy.Subscriber("/vicon_estimate", Float64MultiArray, vicon_callback)

        quad_namespace = "kingfisher"
        pub = rospy.Publisher(quad_namespace+"/agiros_pilot/go_to_pose", 
                            geometry_msgs.PoseStamped, 
                            queue_size=1)
        
        while not rospy.is_shutdown() and end_deploy==False:
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    print("Finished moving to initial position")