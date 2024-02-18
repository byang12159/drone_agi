import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

with open("calibration_data_test.p",'rb') as handle:
    data = pickle.load(handle)

# n = 300
# print(len(data[n]))
# print(data[n][0])
# print(data[n][1])
# print("1")
# print(data[n][2])
# print("2")
# print(data[n][2][0][0])

drone_x = []
drone_y = []
drone_z = []

idx_spotted = []
marker_TS = []
marker_GT = data[0][0]

for i,dat in enumerate(data):
    x,y,z = dat[1][:3]
    drone_x.append(x)
    drone_y.append(y)
    drone_z.append(z)

    if len(dat[2][0]) > 0:
        idx_spotted.append(i)
        marker_TS.append(dat[2][0][0])

        
# print(idx_spotted)
# print(marker_TS)
# fig = plt.figure()
# ax = fig.add_subplot(111,projection='3d')
# ax.scatter(drone_x,drone_y,drone_z)
# ax.plot(marker_GT[0],marker_GT[1],marker_GT[2], 'x', color='r')
# plt.show()