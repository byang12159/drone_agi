import pandas as pd
import matplotlib.pyplot as plt
import os 
import ast
import copy 
import numpy as np 
import re 

script_dir = os.path.dirname(os.path.realpath(__file__))
# Load the CSV files
fn1 = os.path.join(script_dir, '_log_messages_ctrl.csv')
ctrl = pd.read_csv(fn1)
fn2 = os.path.join(script_dir, '_log_messages_PF.csv')
pf = pd.read_csv(fn2)
fn3 = os.path.join(script_dir, 'exp_07-29-12-27.csv')
vicon = pd.read_csv(fn3)
fn4 = os.path.join(script_dir, '_t265_odom_sample.csv')
t265 = pd.read_csv(fn4)
fn5 = os.path.join(script_dir, '_kingfisher_agiros_pilot_state.csv')
agistate = pd.read_csv(fn5)
fn6 = os.path.join(script_dir, '_kingfisher_agiros_pilot_velocity_command.csv')
agivel = pd.read_csv(fn6)
# Convert the time columns to datetime
# ctrl['timestamp'] = ctrl['timestamp']
# pf['timestamp'] = pf['timestamp']

ctrl_ts = []
ctrl_data = []
for row in ctrl.iterrows():
    time_stamp = row[1]['timestamp']
    data = list(ast.literal_eval(row[1]['data']))
    ctrl_ts.append(time_stamp)
    ctrl_data.append(copy.deepcopy(data))
ctrl_data = np.array(ctrl_data)

pf_ts = []
pf_data = []
for row in pf.iterrows():
    time_stamp = row[1]['timestamp']
    data = list(ast.literal_eval(row[1]['data']))
    pf_ts.append(time_stamp)
    pf_data.append(copy.deepcopy(data))
pf_data = np.array(pf_data)

t265_ts = []
t265_data = []
for row in t265.iterrows():
    time_stamp = row[1]['timestamp']
    # data = list(ast.literal_eval(row[1]['data']))
    # pdata = row[1]['pose']
    # position_pattern = re.compile(r'position:\s+x:\s+([-+]?\d*\.\d+|\d+)\s+y:\s+([-+]?\d*\.\d+|\d+)\s+z:\s+([-+]?\d*\.\d+|\d+)')
    # orientation_pattern = re.compile(r'orientation:\s+x:\s+([-+]?\d*\.\d+|\d+)\s+y:\s+([-+]?\d*\.\d+|\d+)\s+z:\s+([-+]?\d*\.\d+|\d+)\s+w:\s+([-+]?\d*\.\d+|\d+)')
    # position_match = position_pattern.search(pdata)
    # orientation_match = orientation_pattern.search(pdata)    
    # px,py,pz = position_match.groups()
    # # ox,oy,oz,ow = orientation_match.groups()
    # vdata = row[1]['twist']
    # linear_pattern = re.compile(r'linear:\s+x:\s+([-+]?\d*\.\d+|\d+)\s+y:\s+([-+]?\d*\.\d+|\d+)\s+z:\s+([-+]?\d*\.\d+|\d+)')
    # angular_pattern = re.compile(r'angular:\s+x:\s+([-+]?\d*\.\d+|\d+)\s+y:\s+([-+]?\d*\.\d+|\d+)\s+z:\s+([-+]?\d*\.\d+|\d+)\s+w:\s+([-+]?\d*\.\d+|\d+)')
    # linear_match = linear_pattern.search(vdata)
    # angular_match = angular_pattern.search(vdata)    
    # vpx,vpy,vpz = position_match.groups()
    # vox,voy,voz,vow = orientation_match.groups()
    data = list(ast.literal_eval(row[1]['data']))
    
    t265_ts.append(time_stamp)
    t265_data.append(copy.deepcopy(data))
t265_data = np.array(t265_data)

agivel_ts = []
agivel_data = []
for row in agivel.iterrows():
    time_stamp = row[1]['timestamp']
    data = list(ast.literal_eval(row[1]['data']))
    agivel_ts.append(time_stamp)
    agivel_data.append(copy.deepcopy(data))


    # data = list(ast.literal_eval(row[1]['data']))
    # vdata = row[1]['twist']
    # linear_pattern = re.compile(r'linear:\s+x:\s+([-+]?\d*\.\d+|\d+)\s+y:\s+([-+]?\d*\.\d+|\d+)\s+z:\s+([-+]?\d*\.\d+|\d+)')
    # angular_pattern = re.compile(r'angular:\s+x:\s+([-+]?\d*\.\d+|\d+)\s+y:\s+([-+]?\d*\.\d+|\d+)\s+z:\s+([-+]?\d*\.\d+|\d+)')
    # linear_match = linear_pattern.search(vdata)
    # angular_match = angular_pattern.search(vdata)    
    # vpx,vpy,vpz = position_match.groups()
    # vox,voy,voz = orientation_match.groups()
    
    # agivel_ts.append(time_stamp)
    # tmp = [
    #     vpx,vpy,vpz,
    #     vox,voy,voz
    # ]
    # agivel_data.append([float(elem) for elem in tmp])
agivel_data = np.array(agivel_data)



data_array = vicon.values

column_headings = vicon.columns.tolist()

print("Column Headings:")
print(column_headings)

x_c = np.array(vicon['TX'].tolist()).reshape((-1,1))
y_c = np.array(vicon['TY'].tolist()).reshape((-1,1))
z_c = np.array(vicon['TZ'].tolist()).reshape((-1,1))

x_c = x_c/1000
y_c = y_c/1000
z_c = z_c/1000

x_r = np.array(vicon['RX'].tolist()).reshape((-1,1))
y_r = np.array(vicon['RY'].tolist()).reshape((-1,1))
z_r = np.array(vicon['RZ'].tolist()).reshape((-1,1))

vicon_data = np.hstack((x_r, y_r, z_r, x_c, y_c, z_c))

tmp = np.zeros((ctrl_data.shape[0], ctrl_data.shape[1]+3))
tmp[:,:ctrl_data.shape[1]] = ctrl_data
for i in range(ctrl_data.shape[0]):
    point = ctrl_data[i,8:11]
    dist = np.linalg.norm(vicon_data[:,3:]-point, axis = 1) 
    idx = np.argmin(dist)
    tmp[i, 12] = vicon_data[i, 0]
    tmp[i, 13] = vicon_data[i, 1]
    tmp[i, 14] = vicon_data[i, 2]
ctrl_data = tmp

# Oscilating Frames
# 1760-1827
# 1906-1993
# 2019-


plt.figure(0)
plt.plot(ctrl_ts, ctrl_data[:,0], label='prediction_on')
plt.plot(ctrl_ts, ctrl_data[:,2], label='current_x')
plt.plot(ctrl_ts, ctrl_data[:,5], label='target_x')
plt.plot(ctrl_ts, ctrl_data[:,8], label='vicon_x')
plt.plot(pf_ts, pf_data[:,6], label='mode')
# plt.plot(pf_ts, pf_data[:,7]/30, label='prediciton_count')
plt.plot(t265_ts, t265_data[:,0], label='t265_x')
plt.plot([1722274235.0403125,1722274235.0403125],[0,3],'b')
# plt.plot([1722206112.2036602,1722206112.2036602],[0,3],'r')
# plt.plot([1722206114.5905025,1722206114.5905025],[0,3],'b')
# plt.plot([1722206117.1908855,1722206117.1908855],[0,3],'r')
# plt.plot([1722206118.1939206,1722206118.1939206],[0,3],'b')
plt.plot(agivel_ts, agivel_data[:,0], label='agivel')
plt.legend()

plt.figure(1)
plt.plot(ctrl_ts, ctrl_data[:,0], label='prediction_on')
plt.plot(ctrl_ts, ctrl_data[:,3], label='current_y')
plt.plot(ctrl_ts, ctrl_data[:,6], label='target_y')
plt.plot(ctrl_ts, ctrl_data[:,9], label='vicon_y')
# plt.plot(ctrl_ts, ctrl_data[:,9])
# plt.plot(ctrl_ts, ctrl_data[:,13]*30)
plt.plot(pf_ts, pf_data[:,6], label='mode')
# plt.plot(pf_ts, pf_data[:,7]/30, label='prediciton_count')
plt.plot(t265_ts, t265_data[:,1], label='t265_y')
plt.plot([1722274235.0403125,1722274235.0403125],[0,3],'b')
# plt.plot([1722206112.2036602,1722206112.2036602],[0,3],'r')
# plt.plot([1722206114.5905025,1722206114.5905025],[0,3],'b')
# plt.plot([1722206117.1908855,1722206117.1908855],[0,3],'r')
# plt.plot([1722206118.1939206,1722206118.1939206],[0,3],'b')
plt.plot(agivel_ts, agivel_data[:,1], label='agivel')
plt.legend()

plt.figure(2)
plt.plot(ctrl_ts, ctrl_data[:,0], label='prediction_on')
plt.plot(ctrl_ts, ctrl_data[:,4], label='current_z')
plt.plot(ctrl_ts, ctrl_data[:,7], label='target_z')
plt.plot(ctrl_ts, ctrl_data[:,10], label='vicon_z')
# plt.plot(ctrl_ts, ctrl_data[:,10])
# plt.plot(ctrl_ts, ctrl_data[:,14])
plt.plot(pf_ts, pf_data[:,6], label='mode')
# plt.plot(pf_ts, pf_data[:,7]/30, label='prediciton_count')
plt.plot(t265_ts, t265_data[:,2], label='t265_z')
plt.plot([1722274235.0403125,1722274235.0403125],[0,3],'b')
# plt.plot([1722206112.2036602,1722206112.2036602],[0,3],'r')
# plt.plot([1722206114.5905025,1722206114.5905025],[0,3],'b')
# plt.plot([1722206117.1908855,1722206117.1908855],[0,3],'r')
# plt.plot([1722206118.1939206,1722206118.1939206],[0,3],'b')
plt.plot(agivel_ts, agivel_data[:,2], label='agivel')
plt.legend()

plt.figure(3)
plt.plot(agivel_ts, agivel_data[:,0])
plt.plot([1722274235.0403125,1722274235.0403125],[0,3],'b')
plt.figure(4)
plt.plot(agivel_ts, agivel_data[:,1])
plt.plot([1722274235.0403125,1722274235.0403125],[0,3],'b')
plt.figure(5)
plt.plot(agivel_ts, agivel_data[:,2])
plt.plot([1722274235.0403125,1722274235.0403125],[0,3],'b')
plt.show()
# # Set the time column as the index
# df1.set_index('timestamp', inplace=True)
# df2.set_index('timestamp', inplace=True)

# # # Resample the data to a common frequency if necessary (e.g., 1 second)
# # # This step is optional and depends on your data and plotting requirements
# # df1 = df1.resample('1S').mean()
# # df2 = df2.resample('1S').mean()

# # Merge the dataframes on time
# merged_df = pd.merge_asof(df1, df2, on='timestamp', suffixes=('_file1', '_file2'))

# # Plot the data
# plt.figure()

# # Assuming you have columns 'data1' in file1 and 'data2' in file2
# plt.plot(merged_df.index, merged_df['data1_file1'], label='Data1 from File1')
# plt.plot(merged_df.index, merged_df['data2_file2'], label='Data2 from File2')

# plt.xlabel('Time')
# plt.ylabel('Value')
# plt.title('Time Aligned Data from Two CSV Files')
# plt.legend()
# plt.show()
