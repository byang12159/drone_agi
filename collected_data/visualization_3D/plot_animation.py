from __future__ import annotations

import pyvista as pv
import numpy as np
import copy
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load Trajectory CSV File
df = pd.read_csv('visualization_3D/exp_08-15-19-48.csv')

gif_save_name = "visualization_3D/drone_animation.gif"

# Load your 3D model
mesh_c = pv.read('visualization_3D/chaser_drone_pyramid.stl')
mesh_l = pv.read('visualization_3D/leader_drone.stl')

def create_transformation_matrix(x, y, z, yaw, pitch, roll):
    # Convert degrees to radians
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    roll = np.radians(roll)

    # Translation matrix
    T = np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

    # Rotation matrices around the x, y, and z axes
    Rx = np.array([
        [1, 0, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch), 0],
        [0, np.sin(pitch), np.cos(pitch), 0],
        [0, 0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(roll), 0, np.sin(roll), 0],
        [0, 1, 0, 0],
        [-np.sin(roll), 0, np.cos(roll), 0],
        [0, 0, 0, 1]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, 0],
        [np.sin(yaw), np.cos(yaw), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # Combine the transformations
    transformation_matrix = T @ Rz @ Ry @ Rx
    return transformation_matrix

##########################################################################################
################################ TRAJECTORY DATA PREP #################################### 
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

trajectory_pos_l = np.stack((x_c,y_c,z_c), axis=1)
trajectory_orient_l = np.stack((Rx_c,Ry_c,Rz_c), axis=1)

trajectory_pos_c = np.stack((x_l,y_l,z_l), axis=1)
trajectory_orient_c = np.stack((Rx_l,Ry_l,Rz_l), axis=1)

bound_points = np.array([[np.max(trajectory_pos_c[:,0]),np.max(trajectory_pos_c[:,1]),np.max(trajectory_pos_c[:,2])],
                         [np.max(trajectory_pos_l[:,0]),np.max(trajectory_pos_l[:,1]),np.max(trajectory_pos_l[:,2])],
                         [np.min(trajectory_pos_c[:,0]),np.min(trajectory_pos_c[:,1]),np.min(trajectory_pos_c[:,2])],
                         [np.min(trajectory_pos_l[:,0]),np.min(trajectory_pos_l[:,1]),np.min(trajectory_pos_l[:,2])]])
bound_point = pv.PolyData(bound_points)
bound_point.lines = np.hstack(([bound_points.shape[0]], np.arange(bound_points.shape[0])))
bound_point["colors"] = np.full(bound_points.shape[0], 0)  

##########################################################################################
############################# STL MODEL ORIENTATION PREP ################################# 

# Define the scaling factors
scale_factors = 0.001
# mesh_c.points[:] = (mesh_c.points - mesh_c.center)*scale_factors + mesh_c.center
mesh_c.points[:] = (mesh_c.points)*scale_factors
# pyramid.points[:] = (pyramid.points - pyramid.center)*scale_factors + pyramid.center

x, y, z = 0,0,0
yaw, pitch, roll = 0,0,0
transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)
mesh_c.transform(transformation_matrix)


scale_factors = 0.001
# mesh_c.points[:] = (mesh_c.points - mesh_c.center)*scale_factors + mesh_c.center
mesh_l.points[:] = (mesh_l.points)*scale_factors
# pyramid.points[:] = (pyramid.points - pyramid.center)*scale_factors + pyramid.center

x, y, z = 0,0,0
yaw, pitch, roll = 0,0,0
transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)
mesh_l.transform(transformation_matrix)

##########################################################################################
###################################### PLOT MODEL ######################################## 

plotter = pv.Plotter(notebook=False, off_screen=True)
plotter.add_mesh(bound_points, color='white', line_width=5)

# Open a gif
plotter.open_gif(gif_save_name)

# Update Z and write a frame for each updated position
framecount = 0
for phase in range(1,len(x_l),10):
    framecount +=1

    trajectory_pos_l_select = trajectory_pos_l[:phase]
    trajectory_l = pv.PolyData(trajectory_pos_l_select) # Create a PolyData object
    trajectory_l.lines = np.hstack(([trajectory_pos_l_select.shape[0]], np.arange(trajectory_pos_l_select.shape[0]))) # Add lines connecting the points
    trajectory_l["colors"] = np.full(trajectory_pos_l_select.shape[0], 255)  # Specify a color for the trajectory # Red color

    trajectory_pos_c_select = trajectory_pos_c[:phase]
    trajectory_c = pv.PolyData(trajectory_pos_c_select)
    trajectory_c.lines = np.hstack(([trajectory_pos_c_select.shape[0]], np.arange(trajectory_pos_c_select.shape[0])))
    trajectory_c["colors"] = np.full(trajectory_pos_c_select.shape[0], 255)  

    # Add the trajectory to the plotter
    plotter.add_mesh(trajectory_c, color='red', line_width=5)
    plotter.add_mesh(trajectory_l, color='red', line_width=5)

    index = phase
    x, y, z = x_c[index], y_c[index], z_c[index]
    yaw, pitch, roll = Rx_c[index], Ry_c[index], Rz_c[index]
    transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)

    current_mesh= f"mesh_c_{index}"
    current_mesh = copy.deepcopy(mesh_c)
    current_mesh.transform(transformation_matrix)
    temp_chaser_mesh = plotter.add_mesh(current_mesh, color='orange')

    x, y, z = x_l[index], y_l[index], z_l[index]
    yaw, pitch, roll = Rx_l[index], Ry_l[index], Rz_l[index]
    transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)

    current_mesh= f"mesh_l_{index}"
    current_mesh = copy.deepcopy(mesh_l)
    current_mesh.transform(transformation_matrix)
    temp_leader_mesh = plotter.add_mesh(current_mesh, color='blue')

    # Write a frame. This triggers a render.
    plotter.write_frame()

    plotter.remove_actor(temp_leader_mesh)
    plotter.remove_actor(temp_chaser_mesh)

# Closes and finalizes movie
plotter.close()


print("DONE")
print(f'framecount: {framecount}')