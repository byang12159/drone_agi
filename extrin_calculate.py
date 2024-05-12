import sys
import pickle
import numpy as np
from scipy.optimize import least_squares
from picam.transformation_properties_drone import get_T_DC
from utils import transformations

with open("datacollect_B_1.pkl",'rb') as handle:
    data = pickle.load(handle)

pose_C = []
pose_W = []
T_WD = []

target_ID = 0 

for j in range(len(data)):
    if len(data[j][2][0]) is not 0:
        for a in range(len(data[j][2][1])):
            if data[j][2][1][a] == target_ID:

                snapstate = data[j][1]
                T_WD_j = transformations.quaternion_matrix(snapstate[11:15])
                T_WD_j[:3,3] = snapstate[:3]
                T_WD.append(T_WD_j)

                Ts = data[j][2][0][a]
                Ts_inv = np.linalg.inv(Ts)
                pose_C.append(Ts_inv)

                marker_GT = data[0][0]
                q = marker_GT[11:15] # x, y, z, w
                T_WM = transformations.quaternion_matrix(q)
                T_WM[:3, 3] = marker_GT[:3]
                pose_W.append(T_WM)
    
pose_C = np.array(pose_C) # N x 4 x 4
pose_W = np.array(pose_W) # N x 4 x 4
T_WD = np.array(T_WD) # N x 4 x 4
print(pose_C.shape)
print(pose_W.shape)
print(T_WD.shape)

marker_gt = np.array(data[0][0][:3]).reshape(1, -1)

idx = np.where(((pose_W[:,:3,3] - marker_gt)**2).sum(axis=1)<0.1)[0]
print('# raw samples: %d, # filtered samples: %d'%(pose_W.shape[0], len(idx)))
pose_C = pose_C[idx,:,:]
pose_W = pose_W[idx,:,:]
T_WD = T_WD[idx,:,:]

def loss(_T_DC, verbose=False):
    q = _T_DC[:4]
    t = _T_DC[4:]
    T_DC = transformations.quaternion_matrix(q)
    T_DC[:3, 3] = t
    T_DC = T_DC.reshape(1, 4, 4)
    pose_W = np.matmul(T_WD, np.matmul(T_DC, pose_C))
    position_W = pose_W[:, :3, 3]
    if verbose:
        return position_W
    return ((position_W - marker_gt)**2).sum()

x0 = np.zeros(7)
x0[:4] = transformations.quaternion_from_matrix(get_T_DC())
x0[4:] = get_T_DC()[:3, 3]

# import ipdb; ipdb.set_trace()

print(f"Starting loss: {loss(x0)}")
res = least_squares(loss, x0)
print(f"Ending Loss {loss(res.x)}")
q=res.x[:4]
t=res.x[4:]
euler = transformations.euler_from_quaternion(q, 'syxz')

print(res.x.tolist())
# print(res.cost)
# print(res.optimality)