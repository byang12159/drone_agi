import pandas as pd
import matplotlib.pyplot as plt
import os 
import ast
import copy 
import numpy as np 
import re 
from datetime import datetime
import pytz

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
fn3 = os.path.join(script_dir, 'exp_08-27-19-08.csv')
vicon = pd.read_csv(fn3)
fn4 = os.path.join(script_dir, '_t265_odom_sample.csv')
t265 = pd.read_csv(fn4)
fn5 = os.path.join(script_dir, '_kingfisher_agiros_pilot_state.csv')
agistate = pd.read_csv(fn5)
fn6 = os.path.join(script_dir, '_kingfisher_agiros_pilot_velocity_command.csv')
agivel = pd.read_csv(fn6)
fn7 = os.path.join(script_dir, 'exp_08-27-19-08.xcp')
xcp = open(fn7, 'r')
xcpstring = xcp.read()
starttime = r"(?<=Capture).*?(?<=START_TIME=\")(.*?)(?=\")"
endtime = r"(?<=Capture).*?(?<=END_TIME=\")(.*?)(?=\")"    
viconstarttime = re.findall(starttime, xcpstring, re.MULTILINE)[0]
viconendtime = re.findall(endtime, xcpstring, re.MULTILINE)[0]

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
    # if time_stamp > max_ctrl_ts:
    #     break
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
    # if time_stamp > max_ctrl_ts:
    #     break
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
    # if time_stamp > max_ctrl_ts:
    #     break
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
for i in range(vicon_ts.shape[0]):
    chaser_x = -vicon_data[i,4]
    chaser_y = vicon_data[i,3]
    chaser_z = vicon_data[i,5]
    if chaser_y>0.4 and chaser_y<0.6 and chaser_z>0.9 and chaser_z<1.1:
        ts = vicon_ts[i]
        break

start_ts = ts-3.5
end_ts = ts+37.5
print(f"start_ts: {start_ts}")
start_idx, end_idx = find_index_range(vicon_ts, [start_ts, end_ts])
vicon_step = vicon_ts[1] - vicon_ts[0]
leader_pos = vicon_data[start_idx:end_idx,3:6]
chaser_pos = vicon_data[start_idx:end_idx, 9:12]
dist_diff = np.linalg.norm(leader_pos-chaser_pos, axis=1)
for i in range(1,len(dist_diff)-1):
    # if np.isnan(dist_diff[i]):
    #     dist_diff[i] = (dist_diff[i-1]+dist_diff[i+1])/2
    if np.isnan(dist_diff[i]) and not np.isnan(dist_diff[i-1]):
        nan_start = i 
        nan_end = i+1 
    elif np.isnan(dist_diff[i]):
        nan_end = nan_end+1 
    
    if np.isnan(dist_diff[i]) and not np.isnan(dist_diff[i+1]):
        val_start = dist_diff[nan_start-1]
        val_end = dist_diff[nan_end]
        vals = np.linspace(val_start, val_end, nan_end-nan_start+2)
        dist_diff[nan_start:nan_end] = vals[1:-1]

displacement=2.0
aways = 0
awaye = 0
total_time = 0 
tmp_ts = vicon_ts[start_idx:end_idx]
for i in range(1, len(dist_diff)):
    if dist_diff[i] > displacement+0.5 and not dist_diff[i-1]>displacement+0.5:
        aways = i 
        awaye = i+1 
    elif dist_diff[i]>displacement+0.5:
        awaye = awaye+1 
    if dist_diff[i]>displacement+0.5 and i+1>=len(dist_diff):
        ts_start = tmp_ts[aways-1]
        ts_end = tmp_ts[awaye-1]
        total_time = total_time + (ts_end-ts_start)
    elif dist_diff[i]>displacement+0.5 and not dist_diff[i+1]>displacement+0.5:
        ts_start = tmp_ts[aways-1]
        ts_end = tmp_ts[awaye]
        total_time = total_time + (ts_end-ts_start)
print(f"frac chaser away: {total_time/40}")

# start_idx, end_idx = find_index_range(pf_ts, [start_ts, end_ts])
# tmp = pf_data[start_idx:end_idx]
# tmp_ts = pf_ts[start_idx:end_idx]
# sees = 0
# seee = 0
# total_lose_t = 0
# for i in range(len(tmp_ts)):
#     if tmp[i,17] == 1 and i == 0:
#         sees = i+1
#         seee = i+1 
#     elif tmp[i,17] == 1 and tmp[i-1,17]!=1:
#         sees = i 
#         seee = i+1
#     elif tmp[i,17] == 1:
#         seee += 1
#     if tmp[i,17] == 1 and i+1 >=len(tmp_ts):
#         sees_ts = tmp_ts[sees-1]
#         seee_ts = tmp_ts[seee-1]
#         total_lose_t += (seee_ts - sees_ts)
#     elif tmp[i,17] == 1 and tmp[i+1,17]!=1:
#         sees_ts = tmp_ts[sees-1]
#         seee_ts = tmp_ts[seee]
#         total_lose_t += (seee_ts- sees_ts)
# print(f"total time not see: {total_lose_t}")

tmp = copy.deepcopy(dist_diff)
for i in range(1,len(tmp)):
    if np.abs(tmp[i] - tmp[i-1])>0.05:
        tmp[i] = np.sign(tmp[i]-tmp[i-1])*0.05+tmp[i-1]

# plt.plot(dist_diff)
# plt.plot(tmp)
# plt.show()

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
    if (mode == 3 or mode ==4) and (pf_data[i-1,6]!=3 and pf_data[i-1,6]!=4):
        rs_idx = i 
        re_idx = i+1
        num_recovery += 1 
    elif mode==3 or mode == 4:
        re_idx += 1 

    if (mode == 3 or mode == 4) and (pf_data[i+1,6]!=3 and pf_data[i+1,6]!=4):
        total_time += (pf_ts[re_idx]-pf_ts[rs_idx]) 

utotal_time = 0
us_idx = 0
us_idx = 0 
num_us = 0
for i in range(start_idx, end_idx):
    prediction_count = pf_data[i,7]
    if prediction_count>30 and pf_data[i-1,7]<=30:
        us_idx = i 
        ue_idx = i+1 
        num_us += 1
    elif prediction_count>30:
        ue_idx += 1 

    if prediction_count > 30 and pf_data[i+1, 7]<=30:
        utotal_time += (pf_ts[ue_idx]-pf_ts[us_idx])

print(f"total time in recovery: {total_time}")
print(f"num_recovery: {num_recovery}")
print(f'ASDT: {(40-total_time)}')
print(f'total time in unstable mode: {utotal_time}')
print(f'num transition to unstable mode: {num_us}')
print(f'ASDT_um: {(40-utotal_time)}')
print(f'max_dist: {(tmp).max()}')

plt.figure(0)
# plt.plot(ctrl_ts, ctrl_data[:,0], label='prediction_on')
plt.plot(ctrl_ts, ctrl_data[:,2], label='current_x')
plt.plot(ctrl_ts, ctrl_data[:,5], label='target_x')
# plt.plot(ctrl_ts, ctrl_data[:,5]-ctrl_data[:,2], label='leader_x')
plt.plot(ctrl_ts, ctrl_data[:,8], label='vicon_x')
# plt.plot(ctrl_ts, np.abs(ctrl_data[:,2]-ctrl_data[:,8]), label='diff')
plt.plot(pf_ts, pf_data[:,0], label='pf_pos')
plt.plot(pf_ts, pf_data[:,3], label='pf_v')
plt.plot(pf_ts, pf_data[:,6], label='mode')
plt.plot(vicon_ts, vicon_data[:,4], label='vicon_leadpos')
# plt.plot(pf_ts, pf_data[:,17], label='detected?')
# plt.plot(pf_ts, pf_data[:,7], label='prediciton_count')
# plt.plot(t265_ts, t265_data[:,0], label='t265_x')
# plt.plot(t265_ts, t265_cov[:,0,0],label='t265_cov')
# plt.plot(agivel_ts, agivel_data[:,0], label='agivel')
plt.legend()

plt.figure(1)
# plt.plot(ctrl_ts, ctrl_data[:,0], label='prediction_on')
plt.plot(ctrl_ts, ctrl_data[:,3], label='current_y')
plt.plot(ctrl_ts, ctrl_data[:,6], label='target_y')
# plt.plot(ctrl_ts, ctrl_data[:,6]-ctrl_data[:,3], label='leader_y')
plt.plot(ctrl_ts, ctrl_data[:,9], label='vicon_y')
# plt.plot(ctrl_ts, ctrl_data[:,9])
# plt.plot(ctrl_ts, np.abs(ctrl_data[:,3]-ctrl_data[:,9]), label='diff')
# plt.plot(ctrl_ts, ctrl_data[:,13]*30)
plt.plot(pf_ts, pf_data[:,1], label='kf_pos')
plt.plot(pf_ts, pf_data[:,4], label='kf_v')
plt.plot(pf_ts, pf_data[:,6], label='mode')
plt.plot(vicon_ts, vicon_data[:,3], label='vicon_leaderpos')
plt.plot(pf_ts, pf_data[:,13], label='pf_pos')
plt.plot(pf_ts, pf_data[:,14], label='pf_v')
plt.plot([start_ts,start_ts],[-2,2],label='start_ts')
plt.plot([end_ts,end_ts],[-2,2],label='end_ts')
plt.plot([ts,ts],[-2,2],label='ts')
# plt.plot(pf_ts, pf_data[:,7], label='prediciton_count')
# plt.plot(t265_ts, t265_data[:,1], label='t265_y')
# plt.plot(t265_ts, t265_cov[:,1,1],label='t265_cov')
# plt.plot(agivel_ts, agivel_data[:,1], label='agivel')
plt.legend()

plt.figure(2)
# plt.plot(ctrl_ts, ctrl_data[:,0], label='prediction_on')
plt.plot(ctrl_ts, ctrl_data[:,4], label='current_z')
plt.plot(ctrl_ts, ctrl_data[:,7], label='target_z')
# plt.plot(ctrl_ts, ctrl_data[:,7]-ctrl_data[:,4], label='leader_z')
plt.plot(ctrl_ts, ctrl_data[:,10], label='vicon_z')
# plt.plot(ctrl_ts, ctrl_data[:,10])
# plt.plot(ctrl_ts, np.abs(ctrl_data[:,4]-ctrl_data[:,10]), label='diff')
# plt.plot(ctrl_ts, ctrl_data[:,14])
plt.plot(pf_ts, pf_data[:,2], label='pf_pos')
plt.plot(pf_ts, pf_data[:,5], label='pf_v')
plt.plot(pf_ts, pf_data[:,6], label='mode')
plt.plot(vicon_ts, vicon_data[:,5], label='vicon_pos')
# plt.plot(pf_ts, pf_data[:,7], label='prediciton_count')
# plt.plot(t265_ts, t265_data[:,2], label='t265_z')
# plt.plot(t265_ts, t265_cov[:,2,2],label='t265_cov')
# plt.plot(agivel_ts, agivel_data[:,2], label='agivel')
plt.legend()

# plt.figure(3)
# plt.plot(ctrl_ts[1:], (ctrl_data[1:,2]-ctrl_data[:-1,2])/(ctrl_ts[1:]-ctrl_ts[:-1]), label='finite diff v')
# # plt.plot(agivel_ts, agivel_data[:,0])
# # plt.plot([1722375434.4892318,1722375434.4892318],[0,3],'b')
# plt.figure(4)
# plt.plot(ctrl_ts[1:], (ctrl_data[1:,3]-ctrl_data[:-1,3])/(ctrl_ts[1:]-ctrl_ts[:-1]), label='finite diff v')
# # plt.plot(agivel_ts, agivel_data[:,1])
# # plt.plot([1722375434.4892318,1722375434.4892318],[0,3],'b')
# plt.figure(5)
# plt.plot(ctrl_ts[1:], (ctrl_data[1:,3]-ctrl_data[:-1,3])/(ctrl_ts[1:]-ctrl_ts[:-1]), label='finite diff v')
# plt.plot(agivel_ts, agivel_data[:,2])
# plt.plot([1722375434.4892318,1722375434.4892318],[0,3],'b')
fig = plt.figure(6)
ax = fig.add_subplot(111, projection='3d')
ax.plot(-vicon_data[:,4], vicon_data[:,3], vicon_data[:,5], label='drone_lead')
ax.plot(-vicon_data[:,10], vicon_data[:,9], vicon_data[:,11], label='drone_chase')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.legend()
# plt.figure(7)
# plt.plot(lead_velocity)
# plt.ylabel('lead velocity')
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
