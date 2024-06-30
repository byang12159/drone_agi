import pickle

pickle_file = 'picam_data.pickle'

with open(pickle_file, 'rb') as f:
    loaded_data = pickle.load(f)
    print('Data has been unpickled:')
    print(len(loaded_data))