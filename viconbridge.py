#!/usr/bin/env python
from pymavlink import mavutil
import sys, os
import numpy as np
import pickle
import time

def extract_mavlink(msg):
    # Data = [x y z vx vy vz ax ay az + ]
    data= [0, ] * (9 + 2 + 4 + 1)

    data[0] = msg.x / 1000.
    data[1] = msg.y / 1000.
    data[2] = msg.z / 1000.
    data[3] = msg.vx / 1000.
    data[4] = msg.vy / 1000.
    data[5] = msg.vz / 1000.
    data[6] = msg.ax / 1000.
    data[7] = msg.ay / 1000.
    data[8] = msg.az / 1000.

    # use msg.covariance to store the yaw and yaw_rate, and q
    offset = 100.
    data[9] = msg.covariance[0] - offset
    data[10] = msg.covariance[1] - offset

    data[11] = msg.covariance[2] - offset
    data[12] = msg.covariance[3] - offset
    data[13] = msg.covariance[4] - offset
    data[14] = msg.covariance[5] - offset

    now = time.time()
    data[-1] = now

    # rounded = [round(x,2) for x in data[9:]]
    print("position xyz:",round(data[0],2),round(data[1],2),round(data[2],2) )

    return data

def main():
    # create a mavlink serial instance
    master = mavutil.mavlink_connection('udpin:0.0.0.0:10085')
    f = open("x_y_z_data_vicon", "w")
    f.write("x   y   z\n")
    f.close()
    while True:
        msg = master.recv_match(blocking=False)
        if not msg:
            continue

        if msg.get_type() == 'LOCAL_POSITION_NED_COV':

            data = extract_mavlink(msg)
            # print(data)
            # f = open("x_y_z_data_vicon", "a")
            # f.write(str(data[0]))
            # f.write(" ")
            # f.write(str(data[1]))
            # f.write(" ")
            # f.write(str(data[2]))
            # f.write("\n")
            # f.close()
            # TODO GET PICAM PICTURE

            # TODO GET D435 PICTURE

        elif msg.get_type() == 'ATT_POS_MOCAP':
            pass

if __name__ == '__main__':
    main()

    # with open('datalog_Vicon.pkl','wb') as file:
    #     pickle.dump(datastorage,file)

    print("finished vicon bridge log")