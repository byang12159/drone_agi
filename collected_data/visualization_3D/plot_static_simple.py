import pyvista as pv
import numpy as np
import copy

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
 
df = pd.read_csv('drone_plot/exp_08-15-19-48.csv')

# Convert DataFrame to numpy array
data_array = df.values
column_headings = df.columns.tolist()
print(column_headings)

x_c = df['TX_c'].tolist()
y_c = df['TY_c'].tolist()
z_c = df['TZ_c'].tolist()
Rx_c = df['RX_c'].tolist()
Ry_c = df['RY_c'].tolist()
Rz_c = df['RZ_c'].tolist()

x_l = df['TX_l'].tolist()
y_l = df['TY_l'].tolist()
z_l = df['TZ_l'].tolist()
Rx_l = df['RX_l'].tolist()
Ry_l = df['RY_l'].tolist()
Rz_l = df['RZ_l'].tolist()

# Convert data to unit meters
x_c = np.array(x_c)/1000
y_c = np.array(y_c)/1000
z_c = np.array(z_c)/1000
Rx_c = np.array(Rx_c)
Ry_c = np.array(Ry_c)
Rz_c = np.array(Rz_c)

x_l = np.array(x_l)/1000
y_l = np.array(y_l)/1000
z_l = np.array(z_l)/1000
Rx_l = np.array(Rx_l)
Ry_l = np.array(Ry_l)
Rz_l = np.array(Rz_l)

lower = 2000
upper = 3500
x_c = x_c[lower:upper]
y_c = y_c[lower:upper]
z_c = z_c[lower:upper]
Rx_c = Rx_c[lower:upper]
Ry_c = Ry_c[lower:upper]
Rz_c = Rz_c[lower:upper]

x_l = x_l[lower:upper]
y_l = y_l[lower:upper]
z_l = z_l[lower:upper]
Rx_l = Rx_l[lower:upper]
Ry_l = Ry_l[lower:upper]
Rz_l = Rz_l[lower:upper]

fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(14, 10))
xval.plot(x_c, label = "GT Pos Chaser x")
xval.plot(x_l, label = "GT Pos Leader x")
# xval.plot(times, GT_NED_states_L[:,3], label = "GT Vel x")
# xval.plot(times, GT_NED_states_L[:,6], label = "GT Accel x")
xval.legend()  
yval.plot(y_c, label = "GT Chaser y")
yval.plot(y_l, label = "GT Leader y")
# yval.plot(times, GT_NED_states_L[:,4], label = "GT Vel y")
# yval.plot(times, GT_NED_states_L[:,7], label = "GT Accel y")
yval.legend()
zval.plot(z_c, label = "GT Chaser z")
zval.plot(z_l, label = "GT Leader z")
# zval.plot(times, GT_NED_states_L[:,5], label = "GT Vel z")
# zval.plot(times, GT_NED_states_L[:,8], label = "GT Accel z")
zval.legend()
plt.show()
