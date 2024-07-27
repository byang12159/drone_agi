import pickle
import numpy as np
import os

file_path = 'picam_data_640480_dist7.pickle'
current_path = os.path.dirname(os.path.abspath(__file__))

# Combine paths using os.path.join
full_path = os.path.join(current_path, file_path)

with open(full_path,'rb') as file:
    data = pickle.load(file)

# data = np.array(data)
print(len(data))
print(len(data[0]))