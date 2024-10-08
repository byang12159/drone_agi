import rospy 
import agiros_msgs.msg as agiros_msgs 
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point, Pose, PointStamped, PoseArray
import numpy as np 
from std_msgs.msg import Float32, String, Bool,Float64MultiArray
from pyquaternion import Quaternion
import time 
from mav_msgs import Actuators

pub = rospy.Publisher('/kingfisher/agiros_pilot/motor_speed',Actuators,queue_size=1)

if __name__ == "__main__":
    rospy.init_node('test_motor_speed')
    rate = rospy.Rate(10)

    for i in range(100):
        command = Actuators(
            angular_velocities = [1000,0,0,0]
        )

        pub.publish(command)
        rate.sleep()