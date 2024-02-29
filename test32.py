import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from utils import transformations
import cv2
import os

with open("calibrationtest33.pkl",'rb') as handle:
    data = pickle.load(handle)


# img = cv2.imread('calibration_imgs/img0.png')
# cv2.imshow("img",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

marker_GT = data[0][1]
marker_GT = [-0.28981097412109375, -0.07496332550048829, 0.4260650634765625, -6.402622442692518e-06, -0.00010892694443464279, -0.0001646561175584793, -0.0036016862392425535, -0.006207201480865478, -0.008212990760803222, 3.039642333984375, 0.00030517578125, -0.017181396484375, -0.0204925537109375, -0.05127716064453125, 0.9983291625976562, 1709169189.4365358]

spotted_count = []
Ts_full = []
snapstate_full = []
target_ID = 0

for j in range(len(data)):
    if len(data[j][3][0]) != 0:
        for a in range(len(data[j][3][1])):
            if data[j][3][1][a] == target_ID:
                # print(data[j][2][0][a])

                spotted_count.append(data[j][0])
                Ts_full.append(data[j][3][0][a])
                snapstate_full.append(data[j][2])

print(spotted_count)

# folder_path = 'calibration_imgs'
# # Get the list of files in the folder
# image_files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]

# # Iterate through the image files
# for image_file in image_files:
#     try:
#         image_number = int(image_file.split("img")[1].split(".")[0])
#     except ValueError:
#         print(f"Error: Invalid filename format for {image_file}")
#         continue
    
#     # Check if the number matches the target number
#     if image_number in spotted_count:
#         # Read the image
#         image_path = os.path.join(folder_path, image_file)
#         image = cv2.imread(image_path)
        
#         # Check if the image was read successfully
#         if image is not None:
#             # Display the image
#             cv2.imshow(image_file, image)
#             cv2.waitKey(0)  # Wait for any key press
#             cv2.destroyAllWindows()  # Close all OpenCV windows after displaying the image
#         else:
#             print(f"Error: Unable to read image {image_path}")



scale = 0.4
xaxis_h = np.array([1*scale,0,0,1])
yaxis_h = np.array([0,1*scale,0,1])
zaxis_h = np.array([0,0,1*scale,1])

# Unit vectors along axes
xaxis = np.array([1, 0, 0])
yaxis = np.array([0, 1, 0])
zaxis = np.array([0, 0, 1])

q = marker_GT[11:15] # x, y, z, w
T_WM = transformations.quaternion_matrix(q)
T_WM[:3, 3] = marker_GT[:3]


fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.plot(0,0,0, 'x',color='red',label="World Center")
ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='green',label='marker location')

scale = 0.5
xaxis_h = np.array([1*scale,0,0,1])
yaxis_h = np.array([0,1*scale,0,1])
zaxis_h = np.array([0,0,1*scale,1])

marker_center = T_WM[:4,3]
marker_tail_x = T_WM@xaxis_h
marker_tail_y = T_WM@yaxis_h
marker_tail_z = T_WM@zaxis_h

ax.plot([marker_center[0],marker_tail_x[0]],[marker_center[1],marker_tail_x[1]],[marker_center[2],marker_tail_x[2]],color='red')
ax.plot([marker_center[0],marker_tail_y[0]],[marker_center[1],marker_tail_y[1]],[marker_center[2],marker_tail_y[2]],color='green')
ax.plot([marker_center[0],marker_tail_z[0]],[marker_center[1],marker_tail_z[1]],[marker_center[2],marker_tail_z[2]],color='blue')

# for i in range(len(drone_GTs)):
#     drone_GT = drone_GTs[i]
#     xaxis_h = np.array([1*scale,0,0,1])
#     yaxis_h = np.array([0,1*scale,0,1])
#     zaxis_h = np.array([0,0,1*scale,1])

#     R2 = np.array([[0,-1,0,0],
#                 [1,0,0,0],
#                 [0,0,1,0],
#                 [0,0,0,1]])
    
#     R1 = np.array([[1,0,0,0],
#                 [0,-1,0,0],
#                 [0,0,1,0],
#                 [0,0,0,1]])
#     xaxis_h = R2@R1@xaxis_h
#     yaxis_h = R2@R1@yaxis_h
#     zaxis_h = R2@R1@zaxis_h

#     q = drone_GT[11:15] # x, y, z, w
#     T_W2 = transformations.quaternion_matrix(q)
#     T_W2[:3, 3] = drone_GT[:3]

#     drone_center = T_W2[:4,3]
#     drone_tail_x = T_W2@xaxis_h
#     drone_tail_y = T_W2@yaxis_h
#     drone_tail_z = T_W2@zaxis_h

#     ax.plot(drone_GT[0],drone_GT[1],drone_GT[2],'x',color='purple',label=f'drone{i} location')
#     ax.plot([drone_center[0],drone_tail_x[0]],[drone_center[1],drone_tail_x[1]],[drone_center[2],drone_tail_x[2]],color='red')
#     ax.plot([drone_center[0],drone_tail_y[0]],[drone_center[1],drone_tail_y[1]],[drone_center[2],drone_tail_y[2]],color='green')
#     ax.plot([drone_center[0],drone_tail_z[0]],[drone_center[1],drone_tail_z[1]],[drone_center[2],drone_tail_z[2]],color='blue')



# scale = 1
# ax.plot([0,xaxis[0]*scale],[0,xaxis[1]*scale],[0,xaxis[2]*scale],color='red')
# ax.plot([0,yaxis[0]*scale],[0,yaxis[1]*scale],[0,yaxis[2]*scale],color='green')
# ax.plot([0,zaxis[0]*scale],[0,zaxis[1]*scale],[0,zaxis[2]*scale],color='blue')


for j in range(len(data)):
    snapstate = data[j][2]
    ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green')
# for i in range(len(spotted_count)):
#     Ts = Ts_full[i]
#     snapstate = snapstate_full[i]

#     Ts_inv = np.linalg.inv(Ts)

#     ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green')
    
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.legend()
plt.axis('equal')
plt.show()
