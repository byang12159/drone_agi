import pyvista as pv
import numpy as np
import copy
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load Trajectory CSV File
df = pd.read_csv('visualization_3D/exp_08-15-19-48.csv')

# Load your 3D model
mesh_c = pv.read('visualization_3D/chaser_drone_pyramid.stl')
mesh_l = pv.read('visualization_3D/leader_drone.stl')


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

trajectory_pos_l = np.stack((x_c,y_c,z_c), axis=1)
trajectory_orient_l = np.stack((Rx_c,Ry_c,Rz_c), axis=1)

# Create a PolyData object
trajectory_l = pv.PolyData(trajectory_pos_l)
# Add lines connecting the points
trajectory_l.lines = np.hstack(([trajectory_pos_l.shape[0]], np.arange(trajectory_pos_l.shape[0])))
# Specify a color for the trajectory
trajectory_l["colors"] = np.full(trajectory_pos_l.shape[0], 255)  # Red color

trajectory_pos_c = np.stack((x_l,y_l,z_l), axis=1)
trajectory_orient_c = np.stack((Rx_l,Ry_l,Rz_l), axis=1)

trajectory_c = pv.PolyData(trajectory_pos_c)
trajectory_c.lines = np.hstack(([trajectory_pos_c.shape[0]], np.arange(trajectory_pos_c.shape[0])))
trajectory_c["colors"] = np.full(trajectory_pos_c.shape[0], 255)  # Red color


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

# Define the vertices of the pyramid
pyramid_width = 25
pyramid_height = 60
point_vertex = [0, -82, 0]
pointa = [pyramid_width, point_vertex[1]-pyramid_height, pyramid_width]
pointb = [-pyramid_width, point_vertex[1]-pyramid_height, pyramid_width]
pointc = [-pyramid_width, point_vertex[1]-pyramid_height, -pyramid_width]
pointd = [pyramid_width,point_vertex[1]-pyramid_height, -pyramid_width]


# Create a PyVista Pyramid mesh object
# Pyramid takes a list of 5 points: 4 for the base and 1 for the apex
pyramid = pv.Pyramid([pointa, pointb, pointc, pointd, point_vertex])




##########################################################################################
############################# STL MODEL ORIENTATION PREP ################################# 

# Define the scaling factors
scale_factors = 0.001
print("center: ",mesh_c.center)
# mesh_c.points[:] = (mesh_c.points - mesh_c.center)*scale_factors + mesh_c.center
mesh_c.points[:] = (mesh_c.points)*scale_factors
# pyramid.points[:] = (pyramid.points - pyramid.center)*scale_factors + pyramid.center

x, y, z = 0,0,0
yaw, pitch, roll = 0,0,0
transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)
mesh_c.transform(transformation_matrix)

# Apply transformation
# mesh_c_0 = copy.deepcopy(mesh_c)
# mesh_c.transform(transformation_matrix)

scale_factors = 0.001
print("center: ",mesh_l.center)
# mesh_c.points[:] = (mesh_c.points - mesh_c.center)*scale_factors + mesh_c.center
mesh_l.points[:] = (mesh_l.points)*scale_factors
# pyramid.points[:] = (pyramid.points - pyramid.center)*scale_factors + pyramid.center

x, y, z = 0,0,0
yaw, pitch, roll = 0,0,0
transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)
mesh_l.transform(transformation_matrix)

##########################################################################################
################################### PLOT STL MODEL ####################################### 
# Create a plotter object
plotter = pv.Plotter()

for i in range(0,len(x_c),100):
    index = i
    x, y, z = x_c[index], y_c[index], z_c[index]
    yaw, pitch, roll = Rx_c[index], Ry_c[index], Rz_c[index]
    transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)

    current_mesh= f"mesh_c_{i}"
    current_mesh = copy.deepcopy(mesh_c)
    current_mesh.transform(transformation_matrix)
    plotter.add_mesh(current_mesh, color='orange')

for i in range(0,len(x_l),40):
    index = i
    x, y, z = x_l[index], y_l[index], z_l[index]
    yaw, pitch, roll = Rx_l[index], Ry_l[index], Rz_l[index]
    transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)

    current_mesh= f"mesh_l_{i}"
    current_mesh = copy.deepcopy(mesh_l)
    current_mesh.transform(transformation_matrix)
    plotter.add_mesh(current_mesh, color='blue')




# Add the transformed mesh to the plotter
# plotter.add_mesh(mesh_c_0, color='orange')


# Add the pyramid mesh to the plotter
# plotter.add_mesh(pyramid, show_edges=True, line_width=5, color='white')

# Add the trajectory to the plotter
plotter.add_mesh(trajectory_c, color='red', line_width=5)
plotter.add_mesh(trajectory_l, color='red', line_width=5)

# plotter.add_axes_at_origin()

# Set camera position for better visualization
# plotter.camera_position = 'xy'
plotter.camera.view_angle = 30.0
plotter.camera.position = (5.486,1.55,2.6)
plotter.camera.elevation = 0.0
plotter.camera.azimuth = 0.0
plotter.camera.roll = -98.97

# View camera parameters in interactive plot
def my_cpos_callback():
    plotter.add_text(f"{str(plotter.camera.position)},\n {str(plotter.camera.roll)},\n {str(plotter.camera.azimuth)},\n {str(plotter.camera.elevation)}", name="cpos")
    return

plotter.add_key_event("p", my_cpos_callback)

# Display the plot
plotter.show()



print("Done")
