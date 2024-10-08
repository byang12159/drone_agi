import pandas as pd
import matplotlib.pyplot as plt
import os 
import ast
import copy 
import numpy as np 
import re 
from datetime import datetime
import pytz
from scipy.optimize import linprog

def find_tightest_separator(x, y):
    n = len(x)
    
    # We need to minimize or maximize c (the intercept)
    c = np.zeros(2)  # [lambda, c, slack variables...]
    c[1] = 2  # Minimize -c for tightest fit
    c[0] = x[-1]
    # Constraints
    A = np.zeros((n, 2))
    A[:, 0] = -x  # For lambda
    A[:, 1] = -1  # For c
    # A[:, 2:] = np.eye(n)  # For slack variables (non-negative constraints)
    
    b = -y
    
    # Bounds for lambda (can be positive or negative), c, and slack variables
    bounds = [(None, None), (None, None)] 
    
    # Solving the linear programming problem
    res = linprog(c, A_ub=A, b_ub=b, bounds=bounds, method='highs')
    
    if res.success:
        lambda_ = res.x[0]
        c = res.x[1]
        return lambda_, c
    else:
        return None, None 
        # raise ValueError("No solution found")

    
def convertTime(timestr:str):
    timestamp_str = timestr.strip('DST').strip(' ')
    timezone = pytz.timezone('America/Chicago')
    dt_naive = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
    dt_aware = timezone.localize(dt_naive)
    unix_time = dt_aware.timestamp()
    return unix_time

def list_subfolders(folder_path):
    return [f.name for f in os.scandir(folder_path) if f.is_dir()]

def find_index_range(array, value_range):
    start_idx = np.searchsorted(array, value_range[0], side='left')
    end_idx = np.searchsorted(array, value_range[1], side='right') - 1
    return start_idx, end_idx

script_dir = os.path.dirname(os.path.realpath(__file__))
processed_bag_dir = os.path.join(script_dir, 'data_processed')
bag_dir = os.path.join(script_dir, 'data')

sub_folders = list_subfolders(processed_bag_dir)

reference_ctrl_ts = None 
reference_vicon_ts = None
reference_x = None 
reference_y = None 
reference_z = None 

for sub_folder in sub_folders:
    data_dir = os.path.join(processed_bag_dir, sub_folder)
    # Load the CSV files
    fn1 = os.path.join(data_dir, '_log_messages_ctrl.csv')
    ctrl = pd.read_csv(fn1)

    vicon_csv_fn =f"{sub_folder}.csv"
    fn3 = os.path.join(bag_dir, vicon_csv_fn)
    vicon = pd.read_csv(fn3)

    vicon_xcp_fn = f"{sub_folder}.xcp"
    fn7 = os.path.join(bag_dir, vicon_xcp_fn)
    xcp = open(fn7, 'r')
    xcpstring = xcp.read()
    starttime = r"(?<=Capture).*?(?<=START_TIME=\")(.*?)(?=\")"
    endtime = r"(?<=Capture).*?(?<=END_TIME=\")(.*?)(?=\")"    
    viconstarttime = re.findall(starttime, xcpstring, re.MULTILINE)[0]
    viconendtime = re.findall(endtime, xcpstring, re.MULTILINE)[0]

    viconstarttime = convertTime(viconstarttime)
    viconendtime = convertTime(viconendtime)

    if "17-25" in sub_folder:
        print("stop")
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
    if reference_ctrl_ts is None:
        reference_ctrl_ts = ctrl_ts[0]
    time_offset = ctrl_ts[0] - reference_ctrl_ts 
    ctrl_ts = ctrl_ts - time_offset
    one_count = 0
    negone_count = 0
    start_t = 0
    end_t = 0
    for i in range(ctrl_data.shape[0]):
        if ctrl_data[i,5] == 1 and ctrl_data[i-1,5]!=1:
            one_count += 1 
        if one_count == 2:
            start_t = ctrl_ts[i] 
            break
    for i in range(ctrl_data.shape[0]):
        if ctrl_data[i,5] == -1 and ctrl_data[i-1,5]!=-1:
            negone_count += 1
        if negone_count == 2:
            end_t = ctrl_ts[i]
            break

    data_array = vicon.values
    column_headings = vicon.columns.tolist()
    if len(column_headings)<10:
        # print("Column Headings:")
        # print(column_headings)
        drone_chase = data_array[:,2:]
        drone_chase[:,3:] /= 1000
        vicon_data = drone_chase
        vicon_ts = np.linspace(viconstarttime, viconendtime, vicon_data.shape[0])

        if reference_x is None:
            reference_x = vicon_data[:300,3].mean()
        offset_x = vicon_data[:300,3].mean()-reference_x 
        vicon_data[:,3] = vicon_data[:,3] - offset_x
        if reference_y is None:
            reference_y  = vicon_data[:300,4].mean()
        offset_y = vicon_data[:300,4].mean()-reference_y 
        vicon_data[:,4] = vicon_data[:,4] - offset_y
        if reference_z is None:
            reference_z  = vicon_data[:300,5].mean()
        offset_z = vicon_data[:300,5].mean()-reference_z 
        vicon_data[:,5] = vicon_data[:,5] - offset_z
        # if reference_vicon_ts is None:
        #     reference_vicon_ts = vicon_ts[0]
        # time_offset = vicon_ts[0] - reference_vicon_ts 
        vicon_ts = vicon_ts - time_offset

        idx_start, idx_end = find_index_range(vicon_ts, [start_t, 7.7+1.7242777000e9])
        # idx_tmp_start, idx_tmp_end = find_index_range(vicon_ts, [9+1.7242777000e9,12+1.7242777000e9])
        idx_start += 5

        array = np.abs(1-(-vicon_data[idx_start:idx_end,4]))
        # array = np.clip(1-(-vicon_data[idx_start:idx_end,4]),a_min = 0, a_max = None)
        ln_array = np.log(array)
        x = vicon_ts[idx_start:idx_end] - vicon_ts[idx_start]
        y = ln_array 
        lam, c = find_tightest_separator(x,y)
        print(f"lambda {sub_folder[10:]}: {lam}")

        plt.figure(1)
        # plt.plot(vicon_ts[idx_start:idx_end], np.abs(1-(-vicon_data[idx_start:idx_end,4])), label=f"{sub_folder[10:]}_orig")
        plt.plot(vicon_ts[idx_start:idx_end], ln_array, label=f"{sub_folder[10:]}_ln")
        plt.plot(x+vicon_ts[idx_start], lam*x+c, label = f"{sub_folder[10:]}_sep")
        plt.legend()

        plt.figure(0)
        plt.plot(ctrl_ts, ctrl_data[:,5], label=f'{sub_folder[10:]}_target_x')
        plt.plot(vicon_ts, -vicon_data[:,4], label=f'{sub_folder[10:]}_vicon_chaser')
        plt.plot([start_t, start_t], [-2,2],label=f'{sub_folder[10:]}_start_t')
        plt.plot([end_t, end_t], [-2,2],label=f'{sub_folder[10:]}_end_t')
        plt.legend()

        # plt.figure(1)
        # plt.plot(vicon_ts, vicon_data[:,3], label=f'{sub_folder[10:]}_vicon_chaser')

        # plt.legend()
    else:
        # print("Column Headings:")
        # print(column_headings)
        drone_lead = data_array[:,2:8]
        drone_lead[:,3:] /= 1000
        drone_chase = data_array[:,8:]
        drone_chase[:,3:] /= 1000
        vicon_data = np.hstack((drone_lead, drone_chase))
        vicon_ts = np.linspace(viconstarttime, viconendtime, vicon_data.shape[0])

        if reference_x is None:
            reference_x = vicon_data[:300,9].mean()
        offset_x = vicon_data[:300,9].mean()-reference_x 
        vicon_data[:,9] = vicon_data[:,9] - offset_x
        if reference_y is None:
            reference_y  = vicon_data[:300,10].mean()
        offset_y = vicon_data[:300,10].mean()-reference_y 
        vicon_data[:,10] = vicon_data[:,10] - offset_y
        if reference_z is None:
            reference_z  = vicon_data[:300,11].mean()
        offset_z = vicon_data[:300,11].mean()-reference_z 
        vicon_data[:,11] = vicon_data[:,11] - offset_z
        # if reference_vicon_ts is None:
        #     reference_vicon_ts = vicon_ts[0]
        # time_offset = vicon_ts[0] - reference_vicon_ts 
        vicon_ts = vicon_ts - time_offset

        if sub_folder == "exp_08-21-16-57":
            idx_start, idx_end = find_index_range(vicon_ts, [start_t, 8.3+1.7242777000e9])
        elif sub_folder == "exp_08-21-17-02":
            idx_start, idx_end = find_index_range(vicon_ts, [start_t, 7.4+1.7242777000e9])
        elif sub_folder == "exp_08-21-17-18":
            idx_start, idx_end = find_index_range(vicon_ts, [start_t, 10.1+1.7242777000e9])
        else:
            idx_start, idx_end = None, None 
        idx_start += 5
        
        array = np.abs(1-(-vicon_data[idx_start:idx_end,10]))
        # array = np.clip(1-(-vicon_data[idx_start:idx_end,10]),a_min = 0, a_max = None)
        ln_array = np.log(array)
        x = vicon_ts[idx_start:idx_end] - vicon_ts[idx_start]
        y = ln_array 
        lam, c = find_tightest_separator(x,y)
        print(f"lambda {sub_folder[10:]}: {lam}")

        plt.figure(1)
        # plt.plot(vicon_ts[idx_start:idx_end], np.abs(1-(-vicon_data[idx_start:idx_end,10])), label=f"{sub_folder[10:]}_orig")
        plt.plot(vicon_ts[idx_start:idx_end], ln_array, label=f"{sub_folder[10:]}_ln")
        plt.plot(x+vicon_ts[idx_start], lam*x+c, label = f"{sub_folder[10:]}_sep")
        plt.legend()


        plt.figure(0)
        # plt.plot(ctrl_ts, ctrl_data[:,0], label='prediction_on')
        # plt.plot(ctrl_ts, ctrl_data[:,2], label=f'{sub_folder[10:]}_current_x')
        plt.plot(ctrl_ts, ctrl_data[:,5], label=f'{sub_folder[10:]}_target_x')
        plt.plot(vicon_ts, -vicon_data[:,10], label=f'{sub_folder[10:]}_vicon_chaser')
        plt.plot([start_t, start_t], [-2,2],label=f'{sub_folder[10:]}_start_t')
        plt.plot([end_t, end_t], [-2,2],label=f'{sub_folder[10:]}_end_t')
        plt.legend()

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
