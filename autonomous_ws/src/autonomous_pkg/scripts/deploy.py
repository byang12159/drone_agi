#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float32, String
import geometry_msgs.msg as geometry_msgs

global_x = 0.0
global_y = 0.0
global_z = 1.0 # Drone initialization height at 1m

def point_callback(data):
    # This function is called every time a new Point message is received
    # Republish the received point message to a new topic

    global global_x, global_y, global_z

    if data.x != 0.0 and data.y != 0.0 and data.z != 0.0:
        # Aruco Detection        
        global_x += (data.x - 2.0)
        global_y += data.y
        global_z += data.z
    
    # Safety height filter
    if global_z >= 2.5:
        global_z = 2.5
        
    target = geometry_msgs.PoseStamped()
    target.pose.position.x = float(global_x)
    target.pose.position.y = float(global_y)
    target.pose.position.z = float(global_z) 
    target.pose.orientation.w = float(1.0)
    target.pose.orientation.z = float(0.0)

    pub.publish(target)
    rospy.loginfo("Publishing Pose to Ctrl: {}".format(target))


if __name__ == '__main__':
    try:
        
        rospy.init_node('picam_test', anonymous=True)

        rospy.Subscriber("/leader_waypoint", geometry_msgs.Point, point_callback)

        quad_namespace = "kingfisher"
        pub = rospy.Publisher(quad_namespace+"/agiros_pilot/go_to_pose", 
                            geometry_msgs.PoseStamped, 
                            queue_size=5)

        # Spin to keep the node alive
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


# def publisher():
#     rospy.init_node('picam_test', anonymous=True)

#     quad_namespace = "kingfisher"
#     pub = rospy.Publisher(quad_namespace+"/agiros_pilot/go_to_pose", 
#                           geometry_msgs.PoseStamped, 
#                           queue_size=1)

#     # Set the loop rate (in Hz)
#     rate = rospy.Rate(1)  # 1 Hz
       
    # while not rospy.is_shutdown():
    #     target = geometry_msgs.PoseStamped()
    #     target.pose.position.x = float(2.0)
    #     target.pose.position.y = float(0.0)
    #     target.pose.position.z = float(2.0)
    #     target.pose.orientation.w = float(1.0)
    #     target.pose.orientation.z = float(0.0)

    #     pub.publish(target)
    #     # Publish the message
        

    #     # Log the message to the console
    #     rospy.loginfo("Publishing: hello")

    #     # Sleep to maintain the loop rate
    #     rate.sleep()

# if __name__ == '__main__':

#     try:
#         publisher()
#     except rospy.ROSInterruptException:
#         pass

#     print("DONE PICAM TEST")

