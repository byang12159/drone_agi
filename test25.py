

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Number of vectors to sample
num_samples = 100

# Generate random angles for spherical coordinates
phi = np.random.uniform(0, np.pi, num_samples)
theta = np.random.uniform(0, 2*np.pi, num_samples)

# Convert spherical coordinates to Cartesian coordinates
x = np.sin(phi) * np.cos(theta)
y = np.sin(phi) * np.sin(theta)
z = np.cos(phi)

# Plot the vectors
fig = plt.figure(figsize=(8,8))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)

# Set equal aspect ratio for all axes
ax.set_box_aspect([1,1,1])

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Sampling Vectors from a Unit Sphere')

plt.show()
