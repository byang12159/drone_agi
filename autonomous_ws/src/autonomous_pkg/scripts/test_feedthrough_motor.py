import rospy 
import agiros_msgs.msg as agiros_msgs 
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point, Pose, PointStamped, PoseArray
import numpy as np 
from std_msgs.msg import Float32, String, Bool,Float64MultiArray
from pyquaternion import Quaternion
import time 

pub = rospy.Publisher('/kingfisher/agiros_pilot/feedthrough_command',agiros_msgs.Command,queue_size=1)

if __name__ == "__main__":
    rospy.init_node('test_feedthrough_command')
    rate = rospy.Rate(10)

    for i in range(100):
        is_single_rotor_thrust = True

        collective_thrust = max(2+i*0.08,0)
        print(i, collective_thrust)

        bodyrates = Vector3(x=0,y=0,z=0)

        command = agiros_msgs.Command(
            is_single_rotor_thrust = is_single_rotor_thrust,
            # collective_thrust = collective_thrust,
            # bodyrates = bodyrates
            thrusts = [collective_thrust, collective_thrust/2,collective_thrust, collective_thrust/2]
        )

        pub.publish(command)
        rate.sleep()