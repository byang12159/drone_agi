#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, String
# import geometry_msgs.msg as geometry_msgs



def publisher():
    rospy.init_node('picam', anonymous=True)
    pub = rospy.Publisher("/leader_waypoint", String, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(1)  # 1 Hz

    
    while not rospy.is_shutdown():
        # Publish the message
        pub.publish("hello")

        # Log the message to the console
        rospy.loginfo("Publishing: hello")

        # Sleep to maintain the loop rate
        rate.sleep()



if __name__ == '__main__':

    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

    print("DONE PICAM")