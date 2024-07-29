import pandas as pd
import matplotlib.pyplot as plt
import os 
import ast
import copy 
import numpy as np 

script_dir = os.path.dirname(os.path.realpath(__file__))
# Load the CSV files
fn1 = os.path.join(script_dir, '_log_messages_ctrl.csv')
ctrl = pd.read_csv(fn1)
fn2 = os.path.join(script_dir, '_log_messages_PF.csv')
pf = pd.read_csv(fn2)
fn3 = os.path.join(script_dir, 'exp_07-28-17-26.csv')
vicon = pd.read_csv(fn3)

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

plt.figure(0)
plt.plot(ctrl_ts, ctrl_data[:,0])
plt.plot(ctrl_ts, ctrl_data[:,5])
# plt.plot(ctrl_ts, ctrl_data[:,8])
# plt.plot(ctrl_ts, ctrl_data[:,12]*30)
plt.plot(pf_ts, pf_data[:,6])
plt.plot(pf_ts, pf_data[:,7]/30)
plt.plot([1722206114.704959048,1722206114.704959048],[0,3],'b')
plt.figure(1)
plt.plot(ctrl_ts, ctrl_data[:,0])
plt.plot(ctrl_ts, ctrl_data[:,6])
# plt.plot(ctrl_ts, ctrl_data[:,9])
# plt.plot(ctrl_ts, ctrl_data[:,13]*30)
plt.plot(pf_ts, pf_data[:,6])
plt.plot(pf_ts, pf_data[:,7]/30)
plt.plot([1722206114.704959048,1722206114.704959048],[0,3],'b')
plt.figure(2)
plt.plot(ctrl_ts, ctrl_data[:,0])
plt.plot(ctrl_ts, ctrl_data[:,7])
# plt.plot(ctrl_ts, ctrl_data[:,10])
# plt.plot(ctrl_ts, ctrl_data[:,14])
plt.plot(pf_ts, pf_data[:,6])
plt.plot(pf_ts, pf_data[:,7]/30)
plt.plot([1722206114.704959048,1722206114.704959048],[0,3],'b')
plt.figure(3)
plt.plot(pf_ts, pf_data[:,7])
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
