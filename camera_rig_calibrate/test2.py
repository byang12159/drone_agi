import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# R = np.array([[ 0.69970652, -0.04754301,  0.71284672],
#  [ 0.29618189 , 0.92730362 ,-0.22887613],
#  [-0.65014389  ,0.37127841  ,0.66292177]])
# T = np.array([[-11.08921313],
#  [ -1.48844197],
#  [  6.795194  ]])

R=np.array([[-0.16832685,  0.92932762 , 0.32865826],
 [-0.9229677 , -0.03151955 ,-0.3835846 ],
 [-0.3461166 , -0.36790855,  0.86304496]]) 
T=np.array( [[-6.42644347],
 [10.30600924],
 [ 4.94616494]])

R=np.array([[ 0.99830221 ,-0.02694498 , 0.05163975],
 [ 0.02514447 , 0.99906375,  0.03520491],
 [-0.05254  ,  -0.03384668  ,0.99804506]]) 
T=np.array( [[-1.43722831],
 [ 1.10645029],
 [-0.23924951]]
)


 

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Generate some random data for quiver plot
vector = np.array([[1],[0],[0]])
translated = vector + T
rotated = np.dot(R,vector) + translated


# Generate random vector data
U = 1 # X.shape gives the shape of X
V = 0
W = 0
# Create a 3D quiver plot
ax.quiver(0,0,0, U, V, W, length=5, normalize=True, color='r')
ax.quiver(translated[0],translated[1],translated[2], U, V, W, length=5, normalize=True, color='g')
ax.quiver(translated[0],translated[1],translated[2], rotated[0],rotated[1],rotated[2], length=5, normalize=True, color='b')
# Set labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

ax.set_xlim([-14,2])
# Show the plot
plt.show()

mtx2 = np.array([[489.53842117,  0.,         307.82908611],
                [  0. ,        489.98143193, 244.48380801],
                [  0.   ,        0.         ,  1.        ]])

dist2 = np.array([-0.44026799,  0.32228178,  0.00059873,  0.00154265, -0.18461811])


import cv2
img = cv2.imread("./checkerboard_D435i/captured_frame_1.jpg")
circleimg = cv2.circle(img, (200,300), 5,(0,255,0),2)


img2 = cv2.imread("./checkerboard_picam/captured_frame_1.jpg")
img2 = cv2.flip(img2,-1)
img2 = cv2.undistort(img2, mtx2, dist2, None, mtx2)
circleimg2 = cv2.circle(img2, (int(200),int(300)), 5,(255,0,0),2)
circleimg2 = cv2.circle(img2, (int(200-T[0]),int(300-T[1])), 5,(0,255,0),2)

images = np.hstack((circleimg,circleimg2))
cv2.imshow("realsense",images)
cv2.waitKey(0)
cv2.destroyAllWindows()