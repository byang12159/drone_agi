import numpy as np
from scipy.signal import butter, filtfilt, savgol_filter

import pandas as pd
import matplotlib.pyplot as plt
import os 
import ast
import copy 
import numpy as np 
import re 
from datetime import datetime
import pytz
from scipy.signal import savgol_filter

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
fn3 = os.path.join(script_dir, 'exp_08-18-18-17.csv')
vicon = pd.read_csv(fn3)
fn7 = os.path.join(script_dir, 'exp_08-18-18-17.xcp')
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
    if time_stamp > max_ctrl_ts:
        break
    data = list(ast.literal_eval(row[1]['data']))
    pf_ts.append(time_stamp)
    pf_data.append(copy.deepcopy(data))
pf_data = np.array(pf_data)
pf_ts = np.array(pf_ts)

data_array = vicon.values
column_headings = vicon.columns.tolist()
print("Column Headings:")
print(column_headings)
drone_chas = data_array[:,2:8]
drone_chas[:,3:] /= 1000
drone_lead = data_array[:,8:]
drone_lead[:,3:] /= 1000
vicon_data = np.hstack((drone_chas, drone_lead))
vicon_ts = np.linspace(viconstarttime, viconendtime, vicon_data.shape[0])

x = ctrl_data[:,3]
y = ctrl_data[:,2]
z = ctrl_data[:,4]
tx = ctrl_data[:,6]
ty = ctrl_data[:,5]
tz = ctrl_data[:,7]

# vdiff = (ty[1:]-ty[:-1])/(ctrl_ts[1:]-ctrl_ts[:-1])
vdiff = (tx[1:]-tx[:-1])/0.03

from scipy.signal import butter, lfilter, lfilter_zi

fs = 1/((ctrl_ts[1:]-ctrl_ts[:-1]).mean())

cutoff = 2 # Cutoff frequency in Hz
order = 6  # Filter order
nyq = 0.5 * fs  # Nyquist frequency
normal_cutoff = cutoff / nyq  # Normalized cutoff frequency
b, a = butter(order, normal_cutoff, btype='low', analog=False)

zi = lfilter_zi(b, a)  # Initial conditions
filtered_signal = []

k = 10
for i in range(vdiff.shape[0]):
    chunk = np.ones(k)*vdiff[0]
    if i<k:
        tmp = vdiff[0:i+1]
        chunk[(k-1-i):] = tmp
    else:
        chunk = vdiff[i+1-k:i+1]    
    chunkfiltered, zi = lfilter(b,a,chunk, zi=zi)
    filtered_signal.append(chunkfiltered[-1])

emf = None 
target_emf = []
alpha = 0.4
for i in range(ctrl_data.shape[0]):
    if emf == None:
        emf = ctrl_data[i,6]
    emf = alpha*ctrl_data[i,6]+(1-alpha)*emf 
    target_emf.append(emf)
target_emf = np.array(target_emf)

window = None
window_size = 10
target_sgf = []
for i in range(ctrl_data.shape[0]):
    if window is None:
        window = np.ones(window_size)*ctrl_data[i,6]
    else:
        window = np.append(window[1:], ctrl_data[i,6])
    
smoothed_data = savgol_filter(ctrl_data[:,6], window_length=100, polyorder=4)
    # target_sgf.append(smoothed_data[-1])
target_sgf = np.array(target_sgf)

target_kf = []
A = np.array([
    [1,0,0.03],
    [1,0,0],
    [0.05*1/0.03,0.05*-1/0.03,0.95*1]
])
Q = np.eye(3)
H = np.array([[1,0,0],[0,1,0]])
R = np.eye(2)

x = np.array([[ctrl_data[0,6]],[ctrl_data[0,6]],[0]])
P = np.eye(3)

target_kf = [copy.deepcopy(x)]
for i in range(1, ctrl_data.shape[0]):
    x = A@x
    P = A@P@A.T + Q

    z = np.array([[ctrl_data[i,6]],[ctrl_data[i-1,6]]])
    y = z-H@x 
    S = H@P@H.T + R 
    K = P@H.T@np.linalg.inv(S)

    x = x + K@y               # Updated state estimate
    P = P - K@H@P 
    target_kf.append(copy.deepcopy(x))

target_kf = np.array(target_kf)
# plt.plot(ctrl_ts, ctrl_data[:,0], label='prediction_on')
# plt.plot(ctrl_ts, ctrl_data[:,3], label='current_x')
plt.figure(0)
plt.plot(ctrl_ts, ctrl_data[:,6], label='target_x')
plt.plot(ctrl_ts, target_emf, label='target_x_emf')
plt.plot(ctrl_ts, smoothed_data, label='target_x_sgf')
plt.plot(ctrl_ts, target_kf[:,0], label='target_x_kf')
plt.legend()

# plt.figure(1)
plt.plot(ctrl_ts, target_kf[:,2], label='target_vx_kf')
# plt.plot(ctrl_ts[:-1], (target_emf[1:]-target_emf[:-1])/0.03, label='target_vx_emf')
# plt.plot(ctrl_ts, ctrl_data[:,5]-ctrl_data[:,2], label='leader_x')
# plt.plot(ctrl_ts, ctrl_data[:,8], label='vicon_x')
# plt.plot(vicon_ts, -vicon_data[:,4], label='vicon_chaser')
# plt.plot(vicon_ts, -vicon_data[:,10], label='vicon_leader')
# plt.plot(ctrl_ts[:-1], (ctrl_data[1:,6]-ctrl_data[:-1,6])/0.03, label='finite_diff_v')
# plt.plot(ctrl_ts, np.abs(ctrl_data[:,2]-ctrl_data[:,8]), label='diff')
# plt.plot(pf_ts, pf_data[:,3], label='pf_v')
# plt.plot(pf_ts, pf_data[:,6], label='mode')
# plt.plot(ctrl_ts[1:], filtered_signal, label="vdiff_filtered")
# plt.plot(pf_ts, pf_data[:,7], label='prediciton_count')
# plt.plot(t265_ts, t265_data[:,0], label='t265_x')
# plt.plot(t265_ts, t265_cov[:,0,0],label='t265_cov')
# plt.plot([1722375434.4892318,1722375434.4892318],[0,3],'b')
# # plt.plot([1722206112.2036602,1722206112.2036602],[0,3],'r')
# plt.plot([1722279507.539467,1722279507.539467],[0,3],'b')
# plt.plot([1722279512.1651788,1722279512.1651788],[0,3],'b')
# plt.plot([1722279556.0982528,1722279556.0982528],[0,3],'b')
# plt.plot(agivel_ts, agivel_data[:,0], label='agivel')
plt.legend()
plt.show()
# Generate sample data
# y = low_frequency_signal + high_frequency_noise

# Low-pass Butterworth filter
cutoff = 30  # Choose based on your signal
nyq = 0.5 * fs
normal_cutoff = cutoff / nyq
b, a = butter(N=order, Wn=normal_cutoff, btype='low', analog=False)
y_filtered = filtfilt(b, a, y)

# # Numerical derivative (finite difference)
# dy_filtered = np.gradient(y_filtered, dx)

# # Alternatively, Savitzky-Golay differentiation
# y_deriv = savgol_filter(y, window_length=window_size, polyorder=poly_order, deriv=1)
