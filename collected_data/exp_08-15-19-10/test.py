import numpy as np
from scipy.signal import butter, filtfilt, savgol_filter

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

x = ctrl_data[:,2]
y = ctrl_data[:,3]
z = ctrl_data[:,4]
tx = ctrl_data[:,5]
ty = ctrl_data[:,6]
tz = ctrl_data[:,7]

# vdiff = (ty[1:]-ty[:-1])/(ctrl_ts[1:]-ctrl_ts[:-1])
vdiff = (ty[1:]-ty[:-1])/0.03

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


plt.plot(ctrl_ts[1:], vdiff, label="vdiff")
plt.plot(ctrl_ts[1:], filtered_signal, label="vdiff_filtered")
plt.plot(pf_ts, pf_data[:,4],label="vpf")
plt.plot(pf_ts, pf_data[:,6], label='mode')
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

# Numerical derivative (finite difference)
dy_filtered = np.gradient(y_filtered, dx)

# Alternatively, Savitzky-Golay differentiation
y_deriv = savgol_filter(y, window_length=window_size, polyorder=poly_order, deriv=1)
