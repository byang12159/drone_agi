#!/usr/bin/env python3
import rospy
from pymavlink import mavutil
import sys, os
import numpy as np
from std_msgs.msg import Float64MultiArray, Bool
import pickle

def main():
    global datastorage
    rospy.init_node('vicon_bridge', anonymous=True)
    pub = rospy.Publisher('/vicon_estimate', Float64MultiArray, queue_size=1)
    # sub_detection = rospy.Subscriber("/aruco_detection", Bool, queue_size=3)

    # create a mavlink serial instance
    master = mavutil.mavlink_connection('udpin:0.0.0.0:10086')

    data = Float64MultiArray()

    data.data = [0, ] * (9 + 1)


    while not rospy.is_shutdown():
        msg = master.recv_match(blocking=False)
        if not msg:
            continue

        # print(msg)

        if msg.get_type() == 'LOCAL_POSITION_NED_COV':
            data.data[0] = msg.x / 1000.
            data.data[1] = msg.y / 1000.
            data.data[2] = msg.z / 1000.
            data.data[3] = msg.vx / 1000.
            data.data[4] = msg.vy / 1000.
            data.data[5] = msg.vz / 1000.
            data.data[6] = msg.ax / 1000.
            data.data[7] = msg.ay / 1000.
            data.data[8] = msg.az / 1000.

            # print("Coordinaes recieved")
            # use msg.covaricane to store the yaw and yaw_rate, and q
            # offset = 100.
            # data.data[9] = msg.covariance[0] - offset
            # data.data[10] = msg.covariance[1] - offset

            # data.data[11] = msg.covariance[2] - offset
            # data.data[12] = msg.covariance[3] - offset
            # data.data[13] = msg.covariance[4] - offset
            # data.data[14] = msg.covariance[5] - offset

            # now = rospy.get_rostime()
            # now = now.to_sec()
            data.data[-1] = msg.time_usec
            pub.publish(data)

            # print("x y z: ",round(data.data[0],5), round(data.data[1],5), round(data.data[2],5))
            print("Full Data: ",data.data)

            datastorage.append(data.data[:])

        elif msg.get_type() == 'ATT_POS_MOCAP':
            pass

if __name__ == '__main__':
    datastorage = []
    save_file = "data_Vicon_7_1D.pkl"

    try:
        main()
    except Exception as e:
        print("Error occurred:{}".format(e))

    with open(save_file ,'wb') as file:
        pickle.dump(datastorage,file)

    print("finished vicon bridge log, saved to {}".format(save_file))