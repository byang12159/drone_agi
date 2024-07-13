import pickle
import os
import numpy as np

# Get the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Specify the file name
file_name = 'picam_data_640480_dist0_5.pickle'

# Create the full path to the file
file_path = os.path.join(script_dir, file_name)

print(f'The full path to the file is: {file_path}')


with open(file_path, 'rb') as f:
    loaded_data = pickle.load(f)
    print('Data has been unpickled:')
    print(len(loaded_data))

for data in loaded_data:
    data = np.array(data)
    print("Dpone")