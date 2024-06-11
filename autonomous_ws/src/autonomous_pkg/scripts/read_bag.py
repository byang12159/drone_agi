#!/usr/bin/env python
import rosbag
# import pandas as pd
from geometry_msgs.msg import Point
import csv 

data = []
data.append(['time','x','y','z'])

# Open the bag file
with rosbag.Bag('uav_experiment.bag') as bag:
    for topic, msg, t in bag.read_messages(topics=['/leader_waypoint']):
        data.append([t.to_sec(),
            msg.x,
            msg.y,
            msg.z,
        ])

print(data)

# CSV file path
csv_file = 'data.csv'
with open(csv_file, 'wb') as f:
    writer = csv.writer(f)
    writer.writerows(data)

print("List saved to ") 
# print(f"List saved to '{csv_file}'")
# time = []
# x_pos = []
# for i in range(len(data)):
#     time.append(data[i].get('time'))
#     x_pos.append(data[i].get('x'))



