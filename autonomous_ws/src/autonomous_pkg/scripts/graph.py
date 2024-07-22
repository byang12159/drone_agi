import pickle
import matplotlib.pyplot as plt
import numpy as np
import os
import numpy.linalg as la
# datafile_c = 'vicon_logger_P0_lead_2.pkl'
datafile_c = 'vicon_logger_P0_agi.pkl'

folder = 'Vicon_data'
current_file_dir = os.path.dirname(__file__)

relative_path = os.path.join(current_file_dir, folder, datafile_c)
with open(relative_path, 'rb') as f:
    data = pickle.load(f)
    print(data) 
data_agi= np.array(data)

x = data_agi[:,0]
y = data_agi[:,1]
z = data_agi[:,2]
vx = data_agi[:,3]
vy = data_agi[:,4]
vz = data_agi[:,5]
ax = data_agi[:,6]
ay = data_agi[:,7]
az = data_agi[:,8]
time = data_agi[:,-1]


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Create a new figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the data
ax.scatter(x[0],y[0],z[0],'r')
ax.plot(x, y, z, label='AGI')
# ax.plot(x_lead, y_lead, z_lead, label='LEAD')

# Customize the plot
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('3D Line Plot')
ax.set_box_aspect([1, 1, 1]) 
ax.legend()

# Show the plot
plt.show()

# fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(14, 10))
# times = time - time[0]
# xval.plot(times, x, label = "GT Pos x")
# # xval.plot(times, GT_NED_states_L[:,3], label = "GT Vel x")
# # xval.plot(times, GT_NED_states_L[:,6], label = "GT Accel x")
# xval.legend()  
# yval.plot(times, y, label = "GT Pos y")
# # yval.plot(times, GT_NED_states_L[:,4], label = "GT Vel y")
# # yval.plot(times, GT_NED_states_L[:,7], label = "GT Accel y")
# yval.legend()
# zval.plot(times, z, label = "GT Pos z")
# # zval.plot(times, GT_NED_states_L[:,5], label = "GT Vel z")
# # zval.plot(times, GT_NED_states_L[:,8], label = "GT Accel z")
# zval.legend()
# plt.show()

# fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(14, 10))
# times = time_lead - time_lead[0]
# xval.plot(times, x_lead, label = "GT Pos x lead")
# # xval.plot(times, GT_NED_states_L[:,3], label = "GT Vel x")
# # xval.plot(times, GT_NED_states_L[:,6], label = "GT Accel x")
# xval.legend()  
# yval.plot(times, y_lead, label = "GT Pos y")
# # yval.plot(times, GT_NED_states_L[:,4], label = "GT Vel y")
# # yval.plot(times, GT_NED_states_L[:,7], label = "GT Accel y")
# yval.legend()
# zval.plot(times, z_lead, label = "GT Pos z")
# # zval.plot(times, GT_NED_states_L[:,5], label = "GT Vel z")
# # zval.plot(times, GT_NED_states_L[:,8], label = "GT Accel z")
# zval.legend()
# plt.show()

# # datafile = 'data_vicon_1D_P1_2_agi.pkl'
# # datafile_L = 'data_vicon_1D_P1_2_lead.pkl'

# # folder = 'Vicon_data'
# # current_file_dir = os.path.dirname(__file__)

# # relative_path = os.path.join(current_file_dir, folder, datafile)
# # with open(relative_path, 'rb') as f:
# #     data = pickle.load(f)
# #     print(data) 
# # data_agi= np.array(data)

# # relative_path = os.path.join(current_file_dir, folder, datafile_L)
# # with open(relative_path, 'rb') as f:
# #     data = pickle.load(f) 
# #     print(data)
# # data_lead= np.array(data)

# # x = data_agi[:,0]
# # y = data_agi[:,1]
# # z = data_agi[:,2]
# # vx = data_agi[:,3]
# # vy = data_agi[:,4]
# # vz = data_agi[:,5]
# # ax = data_agi[:,6]
# # ay = data_agi[:,7]
# # az = data_agi[:,8]

# # x_lead = data_lead[:,0]
# # y_lead = data_lead[:,1]
# # z_lead = data_lead[:,2]
# # vx_lead = data_lead[:,3]
# # vy_lead = data_lead[:,4]
# # vz_lead = data_lead[:,5]
# # ax_lead = data_lead[:,6]
# # ay_lead = data_lead[:,7]
# # az_lead = data_lead[:,8]

# # import matplotlib.pyplot as plt
# # from mpl_toolkits.mplot3d import Axes3D
# # import numpy as np

# # # Create a new figure
# # fig = plt.figure()
# # ax = fig.add_subplot(111, projection='3d')

# # # Plot the data
# # ax.plot(x, y, z, label='3D Line')
# # ax.plot(x_lead, y_lead, z_lead, label='3D Line')

# # # Customize the plot
# # ax.set_xlabel('X axis')
# # ax.set_ylabel('Y axis')
# # ax.set_zlabel('Z axis')
# # ax.set_title('3D Line Plot')
# # ax.legend()

# # # Show the plot
# # plt.show()

# # Match Time data:
# if data_agi.shape[0] > data_l.shape[0]:
#     short = data_l
#     long = data_agi
# else:
#     short = data_agi
#     long = data_l

# short_times = short[:,-1]
# long_times = long[:,-1]

# min_index = -1
# min_loss = 1000000
# for i in range(long.shape[0]-short.shape[0]+1):
#     difference = short_times-long_times[i:i+len(short_times)]
#     loss = la.norm(difference)
#     print(loss)
#     if loss < min_loss:
#         min_index = i
#         min_loss = loss

# print("Min index:", min_index)



# # timestep = 0.033
# # fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(14, 10))
# # times = np.arange(0,len(x))*timestep
# # xval.plot(times, x, label = "GT Pos x")
# # # xval.plot(times, GT_NED_states_L[:,3], label = "GT Vel x")
# # # xval.plot(times, GT_NED_states_L[:,6], label = "GT Accel x")
# # xval.legend()  
# # yval.plot(times, y, label = "GT Pos y")
# # # yval.plot(times, GT_NED_states_L[:,4], label = "GT Vel y")
# # # yval.plot(times, GT_NED_states_L[:,7], label = "GT Accel y")
# # yval.legend()
# # zval.plot(times, z, label = "GT Pos z")
# # # zval.plot(times, GT_NED_states_L[:,5], label = "GT Vel z")
# # # zval.plot(times, GT_NED_states_L[:,8], label = "GT Accel z")
# # zval.legend()
# # plt.show()

# # fig, (vxval, vyval, vzval) = plt.subplots(3, 1, figsize=(14, 10))
# # times = np.arange(0,len(x))*timestep
# # vxval.plot(times, vx, label = "GT vel x")
# # # xval.plot(times, GT_NED_states_L[:,3], label = "GT Vel x")
# # # xval.plot(times, GT_NED_states_L[:,6], label = "GT Accel x")
# # vxval.legend()  
# # vyval.plot(times, vy, label = "GT vel y")
# # # yval.plot(times, GT_NED_states_L[:,4], label = "GT Vel y")
# # yval.plot(times, GT_NED_states_L[:,7], label = "GT Accel y")
# vyval.legend()
# vzval.plot(times, vz, label = "GT vel z")
# # zval.plot(times, GT_NED_states_L[:,5], label = "GT Vel z")
# # zval.plot(times, GT_NED_states_L[:,8], label = "GT Accel z")
# vzval.legend()
# plt.show()