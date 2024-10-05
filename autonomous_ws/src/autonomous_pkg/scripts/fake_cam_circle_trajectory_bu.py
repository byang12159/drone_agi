#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import time
import numpy as np
import matplotlib.pyplot as plt
from rrt_star_3d import main_func

def euler_to_quaternion(yaw):
    # Convert angles from degrees to radians if necessary
    # roll = math.radians(roll)
    # pitch = math.radians(pitch)
    # yaw = math.radians(yaw)

    # Compute the quaternion components
    q_w = np.cos(yaw / 2)
    q_x = 0.0  # Since roll and pitch are zero
    q_y = 0.0  # Since roll and pitch are zero
    q_z = np.sin(yaw / 2)

    return (q_x, q_y, q_z, q_w)

def waypoint():
    # from matplotlib.animation import FuncAnimation, PillowWriter
    def xyz(args, real_t):
        period, sizex, sizey = args
        # print('period/sizex/sizey: ', period, sizex, sizey)

        if not period:
            period = real_t[-1]

        t = real_t / period * 2 * np.pi

        # Parameters for the 2D ellipsoid
        a = sizex  # semi-major axis in x direction
        b = sizey  # semi-major axis in y direction
        
        # Parametric equations for a 2D ellipse
        x = a * np.sin(t)
        y = -b * np.cos(t)+b

        z = np.ones_like(y) * 1
        # TO RUN SADDLE, comment out above and comment in z=np.cos and z+=2.22
        # z = np.cos(2*t)
        # z += 1
        # y += 0.5
        # x += 1.5

        return x, y, z

    # import matplotlib.pyplot as plt
    # from mpl_toolkits.mplot3d import Axes3D
    dt = 0.05
    x_list = []
    y_list = []
    z_list = []
    xang_list = []
    yang_list = []
    zang_list = []
    # yaw_list = []

    x_init = 0
    y_init = 0
    z_init = 0

    prev_x = x_init 
    prev_y = y_init 
    prev_z = z_init 

    pose_list = []
    for i in range(1, 400):
        x,y,z = xyz([20, 1.5, 1.5], 0.1*i)
        yaw = np.arctan2(y-prev_y, x-prev_x)
        # plt.plot([x, prev_x], [y, prev_y],'b')
        # plt.plot([prev_x, prev_x+(x-prev_x)*3], [prev_y, prev_y+(y-prev_y)*3],'r')

        prev_x = x 
        prev_y = y 
        prev_z = z
        qx, qy, qz, qw = euler_to_quaternion(yaw)

        pos = Point(x=prev_x, y=prev_y, z=prev_z)
        ori = Quaternion(x=qx,y=qy,z=qz,w=qw)

        pose = Pose(position=pos, orientation = ori)

        pose_list.append(pose)
    return pose_list

def talker():
    # Initialize the node with the name 'talker'
    rospy.init_node('talker', anonymous=True)
    
    # Create a publisher that will publish to the '/my_topic' topic using the String message type
    pub = rospy.Publisher('/fake_waypoint_list', PoseArray, queue_size=1)
    
    # Set the rate at which the messages will be published (10 Hz)
    rate = rospy.Rate(1)  # 10hz
    count = 0

    pose_list = waypoint()
    # for i in range(2):
    # for i in range(len(x_list)):
    #     displacement_msg = Point()
    #     displacement_msg.x = x_list[i]
    #     displacement_msg.y = y_list[i]
    #     displacement_msg.z = z_list[i]

    for i in range(3):
        posearr_msg = PoseArray(poses = pose_list)

        pub.publish(posearr_msg)

        time.sleep(1)
        # count +=1

        rospy.loginfo("Publishing point {}".format(posearr_msg))
        # time.sleep(40)

            # if count >=6:
                # break
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
