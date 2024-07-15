import numpy as np
import os
# Initialize an empty list to store the data points
data_points = []
current_file_dir = os.path.dirname(__file__)

relative_path = os.path.join(current_file_dir,'data.txt')
# Read the data from the text file
with open(relative_path, 'r') as file:
    for line in file:
        # Extract the list from the line
        if "Full Data:" in line:
            start_index = line.find('[')
            end_index = line.find(']')
            data_str = line[start_index:end_index + 1]
            # Convert the string representation of the list to an actual list
            data_point = eval(data_str)  # Alternatively, you can use ast.literal_eval
            data_points.append(data_point)

# Convert to a NumPy array if needed
data = np.array(data_points)


x = data[:,0]
y = data[:,1]
z = data[:,2]
time = data[:,-1]

###############################################
import pickle
datafile_c = 'vicon_logger_P0_agi.pkl'

folder = 'Vicon_data'
current_file_dir = os.path.dirname(__file__)

relative_path = os.path.join(current_file_dir, folder, datafile_c)
with open(relative_path, 'rb') as f:
    data = pickle.load(f)
    print(data) 
data_agi= np.array(data)

xagi = data_agi[:,0]
yagi= data_agi[:,1]
zagi = data_agi[:,2]
timeagi = data_agi[:,-1]



import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Create a new figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the data
ax.scatter(x[0],y[0],z[0],'r')
ax.plot(x, y, z, label='leader')
ax.plot(xagi, yagi, zagi, label='AGI')
ax.scatter(xagi[0:10],yagi[0:10],zagi[0:10],color = 'red')
# ax.plot(x_lead, y_lead, z_lead, label='LEAD')

# Customize the plot
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('3D Line Plot')
ax.set_zlim(0, 2) 
ax.set_box_aspect([1, 1, 1]) 

ax.legend()

# Show the plot
plt.show()


fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(14, 10))
times = time - time[0]
xval.plot(times, x, label = "GT Pos lead x")
# xval.plot(times, GT_NED_states_L[:,3], label = "GT Vel x")
# xval.plot(times, GT_NED_states_L[:,6], label = "GT Accel x")
xval.legend()  
yval.plot(times, y, label = "GT Pos y")
# yval.plot(times, GT_NED_states_L[:,4], label = "GT Vel y")
# yval.plot(times, GT_NED_states_L[:,7], label = "GT Accel y")
yval.legend()
zval.plot(times, z, label = "GT Pos z")
# zval.plot(times, GT_NED_states_L[:,5], label = "GT Vel z")
# zval.plot(times, GT_NED_states_L[:,8], label = "GT Accel z")
zval.legend()
plt.show()

fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(14, 10))
times = timeagi - timeagi[0]
xval.plot(times, xagi, label = "GT Pos agi x")
# xval.plot(times, GT_NED_states_L[:,3], label = "GT Vel x")
# xval.plot(times, GT_NED_states_L[:,6], label = "GT Accel x")
xval.legend()  
yval.plot(times, yagi, label = "GT Pos y")
# yval.plot(times, GT_NED_states_L[:,4], label = "GT Vel y")
# yval.plot(times, GT_NED_states_L[:,7], label = "GT Accel y")
yval.legend()
zval.plot(times, zagi, label = "GT Pos z")
# zval.plot(times, GT_NED_states_L[:,5], label = "GT Vel z")
# zval.plot(times, GT_NED_states_L[:,8], label = "GT Accel z")
zval.legend()
plt.show()