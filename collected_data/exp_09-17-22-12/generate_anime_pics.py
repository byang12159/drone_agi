import pandas as pd
import matplotlib.pyplot as plt
import os 
import ast
import copy 
import numpy as np 
import re 
from datetime import datetime
import pytz
import gc 

def find_index_range(array, value_range):
    start_idx = np.searchsorted(array, value_range[0], side='left')
    end_idx = np.searchsorted(array, value_range[1], side='right') - 1
    return start_idx, end_idx

def convertTime(timestr:str):
    timestamp_str = timestr.strip('DST').strip(' ')
    timezone = pytz.timezone('America/Chicago')
    dt_naive = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
    dt_aware = timezone.localize(dt_naive)
    unix_time = dt_aware.timestamp()
    return unix_time

script_dir = os.path.dirname(os.path.realpath(__file__))
# Load the CSV files
fn1 = os.path.join(script_dir, '_log_messages_ctrl.csv')
ctrl = pd.read_csv(fn1)
fn2 = os.path.join(script_dir, '_log_messages_PF.csv')
pf = pd.read_csv(fn2)
fn3 = os.path.join(script_dir, 'exp_09-17-22-12.csv')
vicon = pd.read_csv(fn3)
fn4 = os.path.join(script_dir, '_t265_odom_sample.csv')
t265 = pd.read_csv(fn4)
fn5 = os.path.join(script_dir, '_kingfisher_agiros_pilot_state.csv')
agistate = pd.read_csv(fn5)
fn6 = os.path.join(script_dir, '_kingfisher_agiros_pilot_velocity_command.csv')
agivel = pd.read_csv(fn6)
fn7 = os.path.join(script_dir, 'exp_09-17-22-12.xcp')
xcp = open(fn7, 'r')
xcpstring = xcp.read()
starttime = r"(?<=Capture).*?(?<=START_TIME=\")(.*?)(?=\")"
endtime = r"(?<=Capture).*?(?<=END_TIME=\")(.*?)(?=\")"    
viconstarttime = re.findall(starttime, xcpstring, re.MULTILINE)[0]
viconendtime = re.findall(endtime, xcpstring, re.MULTILINE)[0]
fn8 = os.path.join(script_dir, '_picam.csv')
picam = pd.read_csv(fn8)

picam_ts = []
picam_data = []
for row in picam.iterrows():
    time_stamp = row[1]['timestamp']
    # if time_stamp > max_ctrl_ts:
    #     break
    data = row[1]['fn']
    picam_ts.append(time_stamp)
    picam_data.append(copy.deepcopy(data))
picam_ts = np.array(picam_ts)

viconstarttime = convertTime(viconstarttime)
viconendtime = convertTime(viconendtime)

ctrl_ts = []
ctrl_data = []
for row in ctrl.iterrows():
    time_stamp = row[1]['timestamp']
    data = list(ast.literal_eval(row[1]['data']))
    ctrl_ts.append(time_stamp)
    ctrl_data.append(copy.deepcopy(data))
ctrl_data = np.array(ctrl_data)
ctrl_ts = np.array(ctrl_ts)
max_ctrl_ts = ctrl_ts[-1]

pf_ts = []
pf_data = []
for row in pf.iterrows():
    time_stamp = row[1]['timestamp']
    data = list(ast.literal_eval(row[1]['data']))
    pf_ts.append(time_stamp)
    pf_data.append(copy.deepcopy(data))
pf_data = np.array(pf_data)
pf_ts = np.array(pf_ts)

t265_ts = []
t265_data = []
t265_cov = []
for row in t265.iterrows():
    time_stamp = row[1]['timestamp']
    data = list(ast.literal_eval(row[1]['data']))
    cov = np.array(list(ast.literal_eval(row[1]['cov']))).reshape((6,6))
    
    t265_ts.append(time_stamp)
    t265_data.append(copy.deepcopy(data))
    t265_cov.append(cov)
t265_data = np.array(t265_data)
t265_cov = np.array(t265_cov)
t265_ts = np.array(t265_ts)

agivel_ts = []
agivel_data = []
for row in agivel.iterrows():
    time_stamp = row[1]['timestamp']
    data = list(ast.literal_eval(row[1]['data']))
    agivel_ts.append(time_stamp)
    agivel_data.append(copy.deepcopy(data))
agivel_data = np.array(agivel_data)
agivel_ts = np.array(agivel_ts)

data_array = vicon.values
column_headings = vicon.columns.tolist()
print("Column Headings:")
print(column_headings)
drone_lead = data_array[:,2:8]
drone_lead[:,3:] /= 1000
drone_chase = data_array[:,8:]
drone_chase[:,3:] /= 1000
vicon_data = np.hstack((drone_lead, drone_chase))
vicon_ts = np.linspace(viconstarttime, viconendtime, vicon_data.shape[0])

# Compute the first time when leader drone reachers bottom of ellipsoid 
ts = None 
flag =  True
for i in range(vicon_ts.shape[0]):
    chaser_x = -vicon_data[i,4]
    chaser_y = vicon_data[i,3]
    chaser_z = vicon_data[i,5]
    # if chaser_y>2.4 and chaser_y<2.6 and chaser_z>1.4 and chaser_z<1.6:
    #     flag = True
    if flag and chaser_y>0.4 and chaser_y<0.6 and chaser_z>0.9 and chaser_z<1.1:
        ts = vicon_ts[i]
        break

start_ts = ts-6
end_ts = ts+40
print(f"start_ts: {start_ts}")
start_idx, end_idx = find_index_range(vicon_ts, [start_ts, end_ts])
vicon_step = vicon_ts[1] - vicon_ts[0]
leader_pos = vicon_data[start_idx:end_idx,3:6]
chaser_pos = vicon_data[start_idx:end_idx, 9:12]
dist_diff = np.linalg.norm(leader_pos-chaser_pos, axis=1)
nan_start = 0
nan_end = 1
for i in range(1,len(dist_diff)-1):
    # if np.isnan(dist_diff[i]):
    #     dist_diff[i] = (dist_diff[i-1]+dist_diff[i+1])/2
    if np.isnan(dist_diff[i]) and not np.isnan(dist_diff[i-1]):
        nan_start = i 
        nan_end = i+1 
    elif np.isnan(dist_diff[i]):
        nan_end = nan_end+1 
    
    if np.isnan(dist_diff[i]) and not np.isnan(dist_diff[i+1]):
        if nan_start == 0:
            # val_start = dist_diff[nan_start-1]
            val_end = dist_diff[nan_end]
            # vals = np.linspace(val_start, val_end, nan_end-nan_start+2)
            dist_diff[nan_start:nan_end] = val_end
        else:
            val_start = dist_diff[nan_start-1]
            val_end = dist_diff[nan_end]
            vals = np.linspace(val_start, val_end, nan_end-nan_start+2)
            dist_diff[nan_start:nan_end] = vals[1:-1]
            
if np.isnan(dist_diff[-1]):
    dist_diff = dist_diff[:nan_start]
    end_ts -= (vicon_ts[1]-vicon_ts[0])*(len(dist_diff)-nan_start)

dist_sum = (dist_diff*vicon_step).sum()
dist_mean = dist_sum/(end_ts-start_ts)
dist_mean_frac = dist_mean/1
dist_sum_frac = dist_sum/1
print(f"dist_sum: {dist_sum}")
print(f"dist_mean: {dist_mean}")
print(f"dist_sum_frac: {dist_sum_frac}")
print(f"dist_mean_frac: {dist_mean_frac}")

# Get the total amount of time in recovery mode (mode 3)
start_idx, end_idx = find_index_range(pf_ts, [start_ts, end_ts])

total_time = 0
rs_idx = 0
re_idx = 0
num_recovery = 0
for i in range(start_idx, end_idx):
    mode = pf_data[i,6]
    if mode == 3 and pf_data[i-1,6]!=3:
        rs_idx = i 
        re_idx = i+1
        num_recovery += 1 
    elif mode==3:
        re_idx += 1 

    if mode == 3 and pf_data[i+1,6]!=3:
        total_time += (pf_ts[re_idx]-pf_ts[rs_idx]) 


jump_start = 0
jump_end = 0
in_range = False
for i in range(1,len(vicon_ts)-1):
    # if np.isnan(dist_diff[i]):
    #     dist_diff[i] = (dist_diff[i-1]+dist_diff[i+1])/2
    print(np.linalg.norm(vicon_data[i,(3,4,5,9,10,11)]-vicon_data[i-1,(3,4,5,9,10,11)]))
    if (not (-3.3<vicon_data[i,4]<-2.7)) and (-3.3<vicon_data[i-1,4]<-2.7):
        jump_start = i 
        jump_end = i+1
        # in_range = True 
    elif not (-3.3<vicon_data[i,4]<-2.7):
        jump_end = jump_end+1 
    
    if (not (-3.3<vicon_data[i,4]<-2.7)) and (-3.3<vicon_data[i+1,4]<-2.7):
        val_start = vicon_data[jump_start-1,:]
        val_end = vicon_data[jump_end,:]
        vals = np.linspace(val_start, val_end, jump_end-jump_start+2)
        vicon_data[jump_start:jump_end,:] = vals[1:-1,:]

# Create correlation between vicon and other info
start_ts = start_ts-2
end_ts = end_ts+0.5
start_idx, end_idx = find_index_range(vicon_ts, [start_ts, end_ts])

vicon_data_segment = vicon_data[start_idx:end_idx,:]
vicon_ts_segment = vicon_ts[start_idx:end_idx]
picam_list = []
mode_list = []
nan_start = 0
nan_end = 1
for i in range(1,len(vicon_ts_segment)-1):
    # if np.isnan(dist_diff[i]):
    #     dist_diff[i] = (dist_diff[i-1]+dist_diff[i+1])/2
    if np.any(np.isnan(vicon_data_segment[i,:])) and not np.any(np.isnan(vicon_data_segment[i-1,:])):
        nan_start = i 
        nan_end = i+1 
    elif np.any(np.isnan(vicon_data_segment[i,:])):
        nan_end = nan_end+1 
    
    if np.any(np.isnan(vicon_data_segment[i,:])) and not np.any(np.isnan(vicon_data_segment[i+1])):
        val_start = vicon_data_segment[nan_start-1,:]
        val_end = vicon_data_segment[nan_end,:]
        vals = np.linspace(val_start, val_end, nan_end-nan_start+2)
        vicon_data_segment[nan_start:nan_end,:] = vals[1:-1,:]

# for i in range(1,len(vicon_ts_segment)-1):
#     if np.linalg.norm(vicon_data_segment[i,:] - (vicon_data_segment[i-1,:]+vicon_data_segment[i+1,:])/2) > 0.5:
#         vicon_data_segment[i,:] = (vicon_data_segment[i-1,:]+vicon_data_segment[i+1,:])/2

# if np.isnan(dist_diff[-1]):
#     dist_diff = dist_diff[:nan_start]
#     end_ts -= (vicon_ts[1]-vicon_ts[0])*(len(dist_diff)-nan_start)

for i in range(vicon_ts_segment.shape[0]):
    ts = vicon_ts_segment[i]
    idx = (np.abs(picam_ts-ts)).argmin()
    picam_list.append(picam_data[idx])
    idx = (np.abs(pf_ts-ts)).argmin()
    mode_list.append(pf_data[idx,(6, 7)])

import pyvista as pv 
import cv2 

# Load your 3D model
mesh_c = pv.read(os.path.join(script_dir,'chaser_drone_pyramid.stl'))
mesh_l = pv.read(os.path.join(script_dir,'leader_drone.stl'))

def plot_line_3d(start, end, ax=None, color="blue", line_width=1):
    if ax is None:
        ax = pv.Plotter()

    a = start
    b = end

    # Preview how this line intersects this mesh
    line = pv.Line(a, b)

    ax.add_mesh(line, color=color, line_width=line_width)
    return ax

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

t = np.arange(0,15,0.05)
t = t/15*2*np.pi 
a = 2 
b = 0.5 
ellip = np.zeros((t.shape[0],3))
ellip[:,1] = a*np.sin(t)+0.5
ellip[:,2] = b*np.cos(t)+1.5
ellip[:,0] = 3.22

for i in range(4000, len(vicon_ts_segment), 10):
    print(i)
    plotter = pv.Plotter(off_screen=True)
    line_ellip = pv.PolyData(ellip)
    line_ellip.lines = np.hstack(([ellip.shape[0]], np.arange(ellip.shape[0])))
    plotter.add_mesh(line_ellip, color = 'yellow', opacity=0.01)
    
    line_leader  = pv.PolyData(vicon_data_segment[:i,3:6])
    line_leader.lines = np.hstack(([vicon_data_segment[:i,:].shape[0]], np.arange(vicon_data_segment[:i,:].shape[0])))
    line_chaser  = pv.PolyData(vicon_data_segment[:i,9:12])
    line_chaser.lines = np.hstack(([vicon_data_segment[:i,:].shape[0]], np.arange(vicon_data_segment[:i,:].shape[0])))
    plotter.add_mesh(line_leader, color = 'blue', point_size=2)
    plotter.add_mesh(line_chaser, color = 'blue', point_size=2)

    x, y, z = vicon_data_segment[i, 9:12]
    yaw, pitch, roll = vicon_data_segment[i, 6:9]
    transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)
    chaser_mesh= f"mesh_c_{0}"
    chaser_mesh = copy.deepcopy(mesh_c)
    chaser_mesh.transform(transformation_matrix)
    if mode_list[i][0] == 3:
        color = 'red'
    else:
        color = 'green'
    plotter.add_mesh(chaser_mesh, color=color)

    x, y, z = vicon_data_segment[i, 3:6]
    yaw, pitch, roll = vicon_data_segment[i, 0:3]
    transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)
    leader_mesh= f"mesh_l_{0}"
    leader_mesh = copy.deepcopy(mesh_l)
    leader_mesh.transform(transformation_matrix)
    plotter.add_mesh(leader_mesh, color='orange')

    plotter.camera.view_angle = 120
    plotter.camera.azimuth = 0
    plotter.camera.elevation = -20
    # plotter.camera.zoom(1.5)
    # plotter.camera.view_angle = 30
    plotter.camera.focal_point = (0.41457177749999985, -2.014522247, 1.5287900394999998)
    plotter.camera.camera_position = [
        (6.106786981246179, 3.677692956746179, 7.221005243246177),
        (0.41457177749999985, -2.014522247, 1.5287900394999998),
        (0.0, 0.0, 1.0)
    ]
    plotter.camera.zoom(6)
    # plotter.show()
    image_array = plotter.screenshot(return_img = True)

    cv_img = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
    picam_img = cv2.imread(picam_list[i])
    picam_img = cv2.resize(picam_img, (int(picam_img.shape[1]/2), int(picam_img.shape[0]/2)))
    border_thickness = 10 
    picam_img_border = cv2.copyMakeBorder(
        picam_img, 
        top=border_thickness,
        bottom=border_thickness,
        left=border_thickness,
        right=border_thickness,
        borderType=cv2.BORDER_CONSTANT, 
        value = [255,0,0]
    )
    y, x = cv_img.shape[0]-picam_img.shape[0]-30,10
    cv_img[y:y+picam_img_border.shape[0], x:x+picam_img_border.shape[1]] = picam_img_border
    # plt.imshow(cv_img)
    # plt.show()
    cv2.imwrite(f"frames/frame_{i:05d}.png",cv_img)
    # del line_ellip
    # del line_leader
    # del line_chaser
    # del leader_mesh
    # del chaser_mesh
    # del plotter
    # del cv_img
    # del picam_img
    # del picam_img_border
    # gc.collect()
    # plt.imshow(cv_img)
    # plt.show()

# plotter = pv.Plotter(off_screen=True)
# point_leader = vicon_data_segment[0, 3:6]
# point_chaser = vicon_data_segment[0,9:]
# for i in range(10, len(vicon_ts_segment), 10):
#     print(i)
#     # point_leader = np.vstack((point_leader, vicon_data_segment[i,3:6]))
#     # point_chaser = np.vstack((point_chaser, vicon_data_segment[i,9:12]))
#     # plot_line_3d(point_leader[-2,:], point_leader[-1,:], plotter, line_width=10)
#     # plot_line_3d(point_chaser[-2,:], point_chaser[-1,:], plotter, line_width=10)
#     # line_leader  = pv.PolyData(point_leader[-2:])
#     # line_leader.lines = np.hstack(([2], np.arange(2)))
#     # line_chaser = pv.PolyData(point_chaser[-2,])
#     # # line_chaser.lines = np.hstack(([vicon_data_segment.shape[0]], np.arange(vicon_data_segment.shape[0])))
#     # line_leader.lines = np.hstack(([2], np.arange(2)))
#     # plotter.add_mesh(line_leader, color = 'blue')
#     # plotter.add_mesh(line_chaser, color = 'blue')
#     # # index = phase

#     x, y, z = vicon_data_segment[i,9:12]
#     yaw, pitch, roll = vicon_data_segment[i,6:9]
#     transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)
#     current_mesh= f"mesh_c_{0}"
#     current_mesh = copy.deepcopy(mesh_c)
#     current_mesh.transform(transformation_matrix)
#     if mode_list[i][0] == 3:
#         color = 'red'
#     else:
#         color = 'green'
#     temp_chaser_mesh = plotter.add_mesh(current_mesh, color=color)

#     x, y, z = vicon_data_segment[i,3:6]
#     yaw, pitch, roll = vicon_data_segment[i,0:3]
#     transformation_matrix = create_transformation_matrix(x, y, z, yaw, pitch, roll)
#     current_mesh= f"mesh_l_{0}"
#     current_mesh = copy.deepcopy(mesh_l)
#     current_mesh.transform(transformation_matrix)
#     temp_leader_mesh = plotter.add_mesh(current_mesh, color='orange')

#     plotter.camera.view_angle = 30
#     plotter.camera.azimuth = 0
#     plotter.camera.elevation = 0
#     plotter.camera.view_angle = 30
#     plotter.camera.focal_point = (0.41457177749999985, -2.014522247, 1.5287900394999998)
#     plotter.camera.camera_position = [
#         (6.106786981246179, 3.677692956746179, 7.221005243246177),
#         (0.41457177749999985, -2.014522247, 1.5287900394999998),
#         (0.0, 0.0, 1.0)
#     ]
#     # plotter.camera.zoom(0.3)

#     image_array = plotter.screenshot(return_img = True)
#     cv_img = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
#     picam_img = cv2.imread(picam_list[i])
#     picam_img = cv2.resize(picam_img, (int(picam_img.shape[1]/2), int(picam_img.shape[0]/2)))
#     border_thickness = 10 
#     picam_img_border = cv2.copyMakeBorder(
#         picam_img, 
#         top=border_thickness,
#         bottom=border_thickness,
#         left=border_thickness,
#         right=border_thickness,
#         borderType=cv2.BORDER_CONSTANT, 
#         value = [255,0,0]
#     )
#     y, x = cv_img.shape[0]-picam_img.shape[0]-20,20
#     cv_img[y:y+picam_img_border.shape[0], x:x+picam_img_border.shape[1]] = picam_img_border
#     # plt.imshow(cv_img)
#     # plt.show()
#     cv2.imwrite(f"frames/frame_{i:05d}.png",cv_img)

#     plotter.remove_actor(temp_leader_mesh)
#     plotter.remove_actor(temp_chaser_mesh)


# # plotter.show()
