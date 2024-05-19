#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float32, String
import geometry_msgs.msg as geometry_msgs



def publisher():
    rospy.init_node('picam_test', anonymous=True)

    quad_namespace = "kingfisher"
    pub = rospy.Publisher(quad_namespace+"/agiros_pilot/go_to_pose", 
                          geometry_msgs.PoseStamped, 
                          queue_size=1)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    time.sleep(2)
    
    target = geometry_msgs.PoseStamped()
    target.pose.position.x = float(2.2)
    target.pose.position.y = float(0.0)
    target.pose.position.z = float(2.0)
    target.pose.orientation.w = float(1.0)
    target.pose.orientation.z = float(0.0)

    pub.publish(target)
    rospy.loginfo("Publishing: hello")

       
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

if __name__ == '__main__':

    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

    print("DONE PICAM TEST")
