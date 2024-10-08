import pyvista as pv
import numpy as np
import copy
# Load your 3D model
mesh_c = pv.read('chaser_drone_normal.stl')
mesh_l = pv.read('leader_drone.stl')


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




##########################################################################################
############################# STL MODEL ORIENTATION PREP ################################# 

# Define the scaling factors
scale_factors = 0.001
print("center: ",mesh_l.center)
# mesh_c.points[:] = (mesh_c.points - mesh_c.center)*scale_factors + mesh_c.center
mesh_l.points[:] = (mesh_l.points)*scale_factors
# pyramid.points[:] = (pyramid.points - pyramid.center)*scale_factors + pyramid.center

x, y, z = 0,0,0
yaw, pitch, roll = 0,0,0
transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)
mesh_l.transform(transformation_matrix)

# Apply transformation
mesh_c_0 = copy.deepcopy(mesh_c)
mesh_c.transform(transformation_matrix)

##########################################################################################
################################### PLOT STL MODEL ####################################### 
# Create a plotter object
plotter = pv.Plotter()

plotter.add_mesh(mesh_l, color='orange')




# Add the transformed mesh to the plotter
plotter.add_mesh(mesh_c_0, color='orange')


# Add the pyramid mesh to the plotter
# plotter.add_mesh(pyramid, show_edges=True, line_width=5, color='white')

# Add the trajectory to the plotter

plotter.add_axes_at_origin()

# Set camera position for better visualization
plotter.camera_position = 'xy'

# Display the plot
plotter.show()

