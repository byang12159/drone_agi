#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float32, String
import geometry_msgs.msg as geometry_msgs

def point_callback(data):
    # This function is called every time a new Point message is received
    # Republish the received point message to a new topic

    target = geometry_msgs.PoseStamped()
    target.pose.position.x = float(data.x - 2.0)
    target.pose.position.y = float(data.y)
    target.pose.position.z = float(data.z)
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

