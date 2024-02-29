import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from utils import transformations
import cv2
import os

with open("calibrationtest50.pkl",'rb') as handle:
    data = pickle.load(handle)


# img = cv2.imread('calibration_imgs/img0.png')
# cv2.imshow("img",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

marker_GT = data[0][1]
marker_GT = [-0.28981097412109375, -0.07496332550048829, 0.4260650634765625, -6.402622442692518e-06, -0.00010892694443464279, -0.0001646561175584793, -0.0036016862392425535, -0.006207201480865478, -0.008212990760803222, 3.039642333984375, 0.00030517578125, -0.017181396484375, -0.0204925537109375, -0.05127716064453125, 0.9983291625976562, 1709169189.4365358]
drone_GT1= [1.2164635009765625, -0.5174666137695313, 0.11720265197753907, 0.00014077851176261903, 0.00022857576608657836, -0.0004212753772735596, 0.008719792366027832, 0.010835598945617676, -0.016013004302978515, 3.103302001953125, -8.392333984375e-05, -0.10720062255859375, -0.0258026123046875, -0.02208709716796875, 0.9936599731445312, 1709169353.4387295]
drone_GT2 = [1.0514326171875, 1.19684423828125, 0.1096310043334961, -0.00028265872597694396, 0.0015068602561950683, 0.0007717918157577515, -0.0010816322565078734, 0.03434585571289062, 0.016588937759399413, 0.10186767578125, 0.0201263427734375, -0.08457183837890625, -0.03607940673828125, 0.048583984375, 0.9945755004882812, 1709169729.394639]
drone_GT3 = [1.0261141357421875, 1.1710343017578124, 0.10111953735351563, 0.002472440481185913, -7.44369849562645e-05, -0.001165463924407959, -0.004083392143249512, -0.0002686820924282074, 0.004775155067443848, 1.5845718383789062, -0.0264434814453125, -0.10842132568359375, -0.16357421875, 0.6927947998046875, 0.6939163208007812, 1709170160.222613]
# drone_GT4= [1.0412796630859376, -1.7929696044921875, 0.12859934997558595, 0.00014598588645458222, -0.00016538506746292114, 0.0004957308173179627, 0.013071868896484375, 0.006639921188354493, -0.012630967140197754, 1.737884521484375, -0.00188446044921875, -0.0413665771484375, -0.07588958740234375, 0.759765625, 0.6444320678710938, 1709171481.5559406]
# drone_GT5=[1.368216796875, -1.3759149169921876, 1.1167769775390626, -0.004878599643707276, -0.0041912136077880855, -0.013506125450134278, -0.025300798416137697, -0.066332275390625, 0.02696200942993164, 0.8592605590820312, 0.02422332763671875, -0.0810394287109375, -0.01654815673828125, 0.4170379638671875, 0.905120849609375, 1709171560.1802452]
# drone_GT6=[1.3798590087890625, -1.323875732421875, 1.172374267578125, -0.0038009302616119384, -0.06734403991699218, 0.03973449325561523, -0.026865692138671876, 0.044438472747802736, -0.21982913208007812, 0.803680419921875, 0.11092376708984375, -0.00965118408203125, 0.039215087890625, 0.3917083740234375, 0.9192047119140625, 1709171734.8782995]
# drone_GT7 = [1.5237664794921875, -1.246639404296875, 1.133124267578125, 0.014110595703125, -0.004351170063018799, 0.01181017017364502, 0.061221080780029295, -0.03956103515625, 0.11602855682373046, 0.8471908569335938, 0.02675628662109375, -0.0706634521484375, 0.0120391845703125, 0.41277313232421875, 0.9080123901367188, 1709171807.2037954]
drone_GT8 = [1.3081563720703124, -1.181772216796875, 0.9472601928710938, 0.04618138885498047, -0.022743911743164064, 0.11290267181396485, -0.05565329360961914, -0.05912537384033203, -0.030833280563354493, 0.914642333984375, 0.0216522216796875, -0.091705322265625, -0.04076385498046875, 0.43978118896484375, 0.8924789428710938, 1709172002.7875063]
drone_GTs = [drone_GT1,drone_GT2,drone_GT3,drone_GT8]
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

for i in range(len(drone_GTs)):
    drone_GT = drone_GTs[i]
    xaxis_h = np.array([1*scale,0,0,1])
    yaxis_h = np.array([0,1*scale,0,1])
    zaxis_h = np.array([0,0,1*scale,1])

    R2 = np.array([[0,-1,0,0],
                [1,0,0,0],
                [0,0,1,0],
                [0,0,0,1]])
    
    R1 = np.array([[1,0,0,0],
                [0,-1,0,0],
                [0,0,1,0],
                [0,0,0,1]])
    xaxis_h = R2@R1@xaxis_h
    yaxis_h = R2@R1@yaxis_h
    zaxis_h = R2@R1@zaxis_h

    q = drone_GT[11:15] # x, y, z, w
    T_W2 = transformations.quaternion_matrix(q)
    T_W2[:3, 3] = drone_GT[:3]

    drone_center = T_W2[:4,3]
    drone_tail_x = T_W2@xaxis_h
    drone_tail_y = T_W2@yaxis_h
    drone_tail_z = T_W2@zaxis_h

    ax.plot(drone_GT[0],drone_GT[1],drone_GT[2],'x',color='purple',label=f'drone{i} location')
    ax.plot([drone_center[0],drone_tail_x[0]],[drone_center[1],drone_tail_x[1]],[drone_center[2],drone_tail_x[2]],color='red')
    ax.plot([drone_center[0],drone_tail_y[0]],[drone_center[1],drone_tail_y[1]],[drone_center[2],drone_tail_y[2]],color='green')
    ax.plot([drone_center[0],drone_tail_z[0]],[drone_center[1],drone_tail_z[1]],[drone_center[2],drone_tail_z[2]],color='blue')



scale = 1
ax.plot([0,xaxis[0]*scale],[0,xaxis[1]*scale],[0,xaxis[2]*scale],color='red')
ax.plot([0,yaxis[0]*scale],[0,yaxis[1]*scale],[0,yaxis[2]*scale],color='green')
ax.plot([0,zaxis[0]*scale],[0,zaxis[1]*scale],[0,zaxis[2]*scale],color='blue')


Ts = np.array([[ 0.91056476, -0.40396361, -0.08766537, -0.00262141],
       [-0.15009306, -0.12550218, -0.98067389,  0.09899701],
       [ 0.38515437,  0.90612505, -0.17491   ,  2.06689062],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])
Ts_inv = np.linalg.inv(Ts)
aruco_head =   T_WM@R2@Ts_inv.dot(np.array([0,0,0,1]))
aruco_tail_x = T_WM@R2@Ts_inv.dot(xaxis_h)
aruco_tail_y = T_WM@R2@Ts_inv.dot(yaxis_h)
aruco_tail_z = T_WM@R2@Ts_inv.dot(zaxis_h)

ax.plot(aruco_head[0],aruco_head[1],aruco_head[2],'x',color='red')
ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red',linewidth = 0.5)
ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green',linewidth = 0.5)
ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue',linewidth = 0.5)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.legend()
plt.axis('equal')
plt.show()

# for i in range(len(spotted_count)):
#     Ts = Ts_full[i]
#     snapstate = snapstate_full[i]

#     Ts_inv = np.linalg.inv(Ts)

#     ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green')


#     aruco_head =   T_WM@Ts_inv.dot(np.array([0,0,0,1]))
#     aruco_tail_x = T_WM@Ts_inv.dot(xaxis_h)
#     aruco_tail_y = T_WM@Ts_inv.dot(yaxis_h)
#     aruco_tail_z = T_WM@Ts_inv.dot(zaxis_h)

#     ax.plot(aruco_head[0],aruco_head[1],aruco_head[2],'x',color='red')
    # ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red',linewidth = 0.5)
    # ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green',linewidth = 0.5)
    # ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue',linewidth = 0.5)



#     aruco_head =   Ts_inv.dot(np.array([0,0,0,1]))
#     aruco_tail_x = Ts_inv.dot(xaxis_h)
#     aruco_tail_y = Ts_inv.dot(yaxis_h)
#     aruco_tail_z = Ts_inv.dot(zaxis_h)

#     ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red',linewidth = 0.5)
#     ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green',linewidth = 0.5)
#     ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue',linewidth = 0.5)

#     extrinsic = [-1.1821123183663413, 0.6909067890182983, -0.3760973929406938, -0.046566694921098876, -0.3191826790883203, 1.54573429744888, 0.5815594116570081]
#     quaternion = extrinsic[:4]
#     translation = extrinsic[4:]
#     T_DC = transformations.quaternion_matrix(quaternion)
#     T_DC[:3, 3] = translation
#     print("T_DC\n",T_DC, "\n euler\n",(180/np.pi)*np.array(transformations.euler_from_quaternion(quaternion)))


#     T_WD = transformations.quaternion_matrix(snapstate[11:15])
#     T_WD[:3,3] = snapstate[:3]

#     # ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green',label='Drone Ground Truth')
#     drone_head = Ts_inv@T_DC@np.array([0,0,0,1])
#     ax.plot(drone_head[0],drone_head[1],drone_head[2],'x',color='purple',label='Drone Ground Truth')




# fig = plt.figure()
# ax = fig.add_subplot(111,projection='3d')
# ax.plot(0,0,0, 'x',color='red',label="World Center")
# scale = 1
# ax.plot([0,xaxis[0]*scale],[0,xaxis[1]*scale],[0,xaxis[2]*scale],color='m')
# ax.plot([0,yaxis[0]*scale],[0,yaxis[1]*scale],[0,yaxis[2]*scale],color='y')
# ax.plot([0,zaxis[0]*scale],[0,zaxis[1]*scale],[0,zaxis[2]*scale],color='c')

# count = 0
# for i in range(len(idx_spotted)):
#     Ts = Ts_full[i]
#     snapstate = snapstate_full[i]

#     ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green',label='Drone Ground Truth')

#     ########### PLOT for Marker Poses ###########
#     scale = 0.5
#     xaxis_h = np.array([1*scale,0,0,1])
#     yaxis_h = np.array([0,1*scale,0,1])
#     zaxis_h = np.array([0,0,1*scale,1])

#     marker_center = T_WM[:4,3]
#     marker_tail_x = T_WM@xaxis_h
#     marker_tail_y = T_WM@yaxis_h
#     marker_tail_z = T_WM@zaxis_h

#     ax.plot([marker_center[0],marker_tail_x[0]],[marker_center[1],marker_tail_x[1]],[marker_center[2],marker_tail_x[2]],color='red')
#     ax.plot([marker_center[0],marker_tail_y[0]],[marker_center[1],marker_tail_y[1]],[marker_center[2],marker_tail_y[2]],color='green')
#     ax.plot([marker_center[0],marker_tail_z[0]],[marker_center[1],marker_tail_z[1]],[marker_center[2],marker_tail_z[2]],color='blue')

#     ########## PLOT for Drone Poses ###########
#     T_WD = transformations.quaternion_matrix(snapstate[11:15])
#     T_WD[:3,3] = snapstate[:3]
#     drone_axes_scale = 1

#     drone_head = T_WD[:3,3]
#     drone_axes_tip_x = T_WD@xaxis_h
#     drone_axes_tip_y = T_WD@yaxis_h
#     drone_axes_tip_z = T_WD@zaxis_h

#     # ax.plot(drone_head[0],drone_head[1],drone_head[2],'x',color='green')
#     ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red',linewidth=0.5)
#     ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green',linewidth=0.5)
#     ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue',linewidth=0.5)

#     count += 1
#     if count >= 100:
#         break

#     # translation = Ts[:3,3]
#     # rotation = np.linalg.inv(Ts[:3,:3])
#     # Ts_inv = np.linalg.inv(Ts)

#     # aruco_head =   Ts_inv.dot(np.array([0,0,0,1]))
#     # aruco_tail_x = Ts_inv.dot(xaxis_h)
#     # aruco_tail_y = Ts_inv.dot(yaxis_h)
#     # aruco_tail_z = Ts_inv.dot(zaxis_h)

#     # ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red',linewidth = 0.5)
#     # ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green',linewidth = 0.5)
#     # ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue',linewidth = 0.5)

#     # extrinsic = [-1.1821123183663413, 0.6909067890182983, -0.3760973929406938, -0.046566694921098876, -0.3191826790883203, 1.54573429744888, 0.5815594116570081]
#     # quaternion = extrinsic[:4]
#     # translation = extrinsic[4:]
#     # T_DC = transformations.quaternion_matrix(quaternion)
#     # T_DC[:3, 3] = translation

#     # T_WD = transformations.quaternion_matrix(snapstate[11:15])
#     # T_WD[:3,3] = snapstate[:3]


#     # drone_head = Ts_inv@T_DC@np.array([0,0,0,1])
#     # ax.plot(drone_head[0],drone_head[1],drone_head[2],'x',color='purple',label='Drone Ground Truth')

# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# # plt.legend()
# plt.axis('equal')
# plt.show()




# # fig = plt.figure()
# # ax = fig.add_subplot(111,projection='3d')
# # for i in range(len(idx_spotted)):
# #     Ts = Ts_full[i]
# #     snapstate = snapstate_full[i]

# #     q = marker_GT[11:15] # x, y, z, w
# #     T_WM = transformations.quaternion_matrix(q)
# #     T_WM[:3, 3] = marker_GT[:3]

# #     scale = 1

# #     ax.plot(0,0,0, 'x',color='red',label="Vicon Zero")
# #     ax.plot(snapstate[0],snapstate[1],snapstate[2],'x',color='green',label='Drone Ground Truth')
# #     # ax.plot(marker_GT[0],marker_GT[1],marker_GT[2],'x',color='cyan',label='Marker Ground Truth')
# #     ########### PLOT for Marker Poses ###########
# #     scale = 0.5
# #     xaxis_h = np.array([1*scale,0,0,1])
# #     yaxis_h = np.array([0,1*scale,0,1])
# #     zaxis_h = np.array([0,0,1*scale,1])

# #     marker_center = T_WM[:4,3]
# #     marker_tail_x = T_WM@xaxis_h
# #     marker_tail_y = T_WM@yaxis_h
# #     marker_tail_z = T_WM@zaxis_h

# #     # ax.plot([marker_center[0],marker_tail_x[0]*scale],[marker_center[1],marker_tail_x[1]*scale],[marker_center[2],marker_tail_x[2]*scale],color='red')
# #     # ax.plot([marker_center[0],marker_tail_y[0]*scale],[marker_center[1],marker_tail_y[1]*scale],[marker_center[2],marker_tail_y[2]*scale],color='green')
# #     # ax.plot([marker_center[0],marker_tail_z[0]*scale],[marker_center[1],marker_tail_z[1]*scale],[marker_center[2],marker_tail_z[2]*scale],color='blue')


# #     # ########### PLOT for Camera Poses ###########
# #     translation = Ts[:3,3]
# #     rotation = np.linalg.inv(Ts[:3,:3])
# #     Ts_inv = np.linalg.inv(Ts)

# #     aruco_head =   T_WM@Ts_inv.dot(np.array([0,0,0,1]))
# #     aruco_tail_x = T_WM@Ts_inv.dot(xaxis_h)
# #     aruco_tail_y = T_WM@Ts_inv.dot(yaxis_h)
# #     aruco_tail_z = T_WM@Ts_inv.dot(zaxis_h)
# #     # aruco_head =   Ts@T_WM.dot(np.array([0,0,0,1]))
# #     # aruco_tail_x = Ts@T_WM.dot(xaxis_h*scale)
# #     # aruco_tail_y = Ts@T_WM.dot(yaxis_h*scale)
# #     # aruco_tail_z = Ts@T_WM.dot(zaxis_h*scale)

# #     # print("one",T_WM@Ts_inv)
# #     # print("two",T_WM.dot(Ts_inv))

# #     ax.plot([aruco_head[0],aruco_tail_x[0]],[aruco_head[1],aruco_tail_x[1]],[aruco_head[2],aruco_tail_x[2]],color='red',linewidth = 0.5)
# #     ax.plot([aruco_head[0],aruco_tail_y[0]],[aruco_head[1],aruco_tail_y[1]],[aruco_head[2],aruco_tail_y[2]],color='green',linewidth = 0.5)
# #     ax.plot([aruco_head[0],aruco_tail_z[0]],[aruco_head[1],aruco_tail_z[1]],[aruco_head[2],aruco_tail_z[2]],color='blue',linewidth = 0.5)


# #     ########### PLOT for Drone Poses ###########
# #     T_WD = transformations.quaternion_matrix(snapstate[11:15])
# #     T_WD[:3,3] = snapstate[:3]
# #     drone_axes_scale = 1

# #     drone_head = T_WD[:3,3]
# #     drone_axes_tip_x = T_WD@xaxis_h
# #     drone_axes_tip_y = T_WD@yaxis_h
# #     drone_axes_tip_z = T_WD@zaxis_h

# #     # # ax.plot(drone_head[0],drone_head[1],drone_head[2],'x',color='green')
# #     # ax.plot([drone_head[0],drone_axes_tip_x[0]],[drone_head[1],drone_axes_tip_x[1]],[drone_head[2],drone_axes_tip_x[2]],color='red')
# #     # ax.plot([drone_head[0],drone_axes_tip_y[0]],[drone_head[1],drone_axes_tip_y[1]],[drone_head[2],drone_axes_tip_y[2]],color='green')
# #     # ax.plot([drone_head[0],drone_axes_tip_z[0]],[drone_head[1],drone_axes_tip_z[1]],[drone_head[2],drone_axes_tip_z[2]],color='blue')

# # ax.set_xlabel('x')
# # ax.set_ylabel('y')
# # ax.set_zlabel('z')
# # # plt.legend()
# # plt.axis('equal')
# # print("marker estimate at position ",aruco_head[0],aruco_head[1],aruco_head[2])
# # plt.show()


# #     # print("difference",drone_head-aruco_head)

# #     # print("marker location \n",marker_center)
# #     # print("drone location \n",drone_head)
# #     # print("camera location \n",aruco_head)
    