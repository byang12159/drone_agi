#!/usr/bin/env python3
#!/usr/bin/env python
import rospy
from pymavlink import mavutil
import sys, os
# import numpy as np
from std_msgs.msg import Float64MultiArray

def main():
    rospy.init_node('vicon_bridge', anonymous=True)
    pub = rospy.Publisher('/vicon_estimate', Float64MultiArray, queue_size=1)

    # create a mavlink serial instance
    master = mavutil.mavlink_connection('udpin:0.0.0.0:10085')

    data = Float64MultiArray()

    data.data = [0, ] * (9 + 2 + 4 + 1)

    while not rospy.is_shutdown():
        msg = master.recv_match(blocking=False)
        if not msg:
            continue

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

            # use msg.covaricane to store the yaw and yaw_rate, and q
            offset = 100.
            data.data[9] = msg.covariance[0] - offset
            data.data[10] = msg.covariance[1] - offset

            data.data[11] = msg.covariance[2] - offset
            data.data[12] = msg.covariance[3] - offset
            data.data[13] = msg.covariance[4] - offset
            data.data[14] = msg.covariance[5] - offset

            now = rospy.get_rostime()
            now = now.to_sec()
            data.data[-1] = now
            pub.publish(data)

            print("data")
        elif msg.get_type() == 'ATT_POS_MOCAP':
            pass

if __name__ == '__main__':
    main()

# from pymavlink import mavutil
# import sys, os
# # import numpy as np
# import pickle
# import time
# import rospy
# from std_msgs.msg import Float64MultiArray

# def extract_mavlink(msg):
#     # Data = [x y z vx vy vz ax ay az + ]
#     data= [0, ] * (9 + 2 + 4 + 1)

#     data[0] = msg.x / 1000.
#     data[1] = msg.y / 1000.
#     data[2] = msg.z / 1000.
#     data[3] = msg.vx / 1000.
#     data[4] = msg.vy / 1000.
#     data[5] = msg.vz / 1000.
#     data[6] = msg.ax / 1000.
#     data[7] = msg.ay / 1000.
#     data[8] = msg.az / 1000.

#     # use msg.covariance to store the yaw and yaw_rate, and q
#     offset = 100.
#     data[9] = msg.covariance[0] - offset
#     data[10] = msg.covariance[1] - offset

#     data[11] = msg.covariance[2] - offset
#     data[12] = msg.covariance[3] - offset
#     data[13] = msg.covariance[4] - offset
#     data[14] = msg.covariance[5] - offset

#     now = time.time()
#     data[-1] = now

#     # rounded = [round(x,2) for x in data[9:]]
#     print("position xyz:",round(data[0],2),round(data[1],2),round(data[2],2) )
#     # print("data",data)
#     return data

# def main():
#     rospy.init_node('vicon_bridge', anonymous=True)
#     pub = rospy.Publisher('/vicon_estimate', Float64MultiArray, queue_size=1)

#     # create a mavlink serial instance
#     master = mavutil.mavlink_connection('udpin:0.0.0.0:10085')
#     calibration_data =[]
#     while True:
#         msg = master.recv_match(blocking=False)
#         if not msg:
#             continue

#         if msg.get_type() == 'LOCAL_POSITION_NED_COV':

#             data = extract_mavlink(msg)
#             # calibration_data.append(data)
#             # print(data)
#             # f = open("x_y_z_data_vicon", "a")
#             # f.write(str(data[0]))
#             # f.write(" ")
#             # f.write(str(data[1]))
#             # f.write(" ")
#             # f.write(str(data[2]))
#             # f.write("\n")
#             # f.close()
#             # TODO GET PICAM PICTURE

#             # TODO GET D435 PICTURE

#         elif msg.get_type() == 'ATT_POS_MOCAP':
#             pass
    

# if __name__ == '__main__':
#     main()

#     # with open('datalog_Vicon.pkl','wb') as file:
#     #     pickle.dump(datastorage,file)

#     print("finished vicon bridge log")


