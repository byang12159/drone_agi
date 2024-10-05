#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import Point
import time

def waypoint():
    import numpy as np
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
        x = a * np.cos(t)
        y = b * np.sin(t)

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

    for i in range(400):
        x,y,z = xyz([20,1.5,1.5],dt*i)
        x_n,y_n,z_n = xyz([20,1.5,1.5],dt*(i+1))

        if i == 0:
            x_init, y_init, z_init = x,y,z
            x = 0
            y = 0
        else:
            x -= x_init
            y -= y_init

        x_n -= x_init
        y_n -= y_init
        Vx = x_n - x
        Vy = y_n - y
        Vz = z_n - z

        # Calculate the magnitude of the vector
        magnitude = np.sqrt(Vx**2 + Vy**2 + Vz**2)

        # Calculate the cosines of the orientation angles
        cos_alpha = Vx / magnitude
        cos_beta = Vy / magnitude
        cos_gamma = Vz / magnitude

        # Calculate the angles in radians
        alpha = np.arccos(cos_alpha)
        beta = np.arccos(cos_beta)
        gamma = np.arccos(cos_gamma)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z)
        xang_list.append(alpha)
        yang_list.append(beta)
        zang_list.append(gamma)
    return x_list, y_list, z_list, xang_list, yang_list, zang_list
    # return x_list, y_list, z_list, yaw_list

def talker():
    # Initialize the node with the name 'talker'
    rospy.init_node('talker', anonymous=True)
    
    # Create a publisher that will publish to the '/my_topic' topic using the String message type
    pub = rospy.Publisher('/fake_waypoint', Point, queue_size=10)
    
    # Set the rate at which the messages will be published (10 Hz)
    rate = rospy.Rate(1)  # 10hz
    count = 0

    x_list, y_list, z_list, xang_list, yang_list, zang_list = waypoint()
    # while not rospy.is_shutdown():
    for i in range(len(x_list)):
        displacement_msg = Point()
        displacement_msg.x = x_list[i]
        displacement_msg.y = y_list[i]
        displacement_msg.z = z_list[i]

        pub.publish(displacement_msg)

        # count +=1

        rospy.loginfo("Publishing point {}".format(displacement_msg))
        time.sleep(0.05)

        # if count >=6:
            # break
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
