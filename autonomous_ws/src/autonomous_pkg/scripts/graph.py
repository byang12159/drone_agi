import pickle
import matplotlib.pyplot as plt
import numpy as np

with open('Fruits.pkl', 'rb') as f:

    data = pickle.load(f) # deserialize using load()
    print(data) # print student names

y = []

print("length data",len(data))

for i in range(len(data)-55):
    y.append(data[i][1])

t = np.linspace(0,10,len(y))

plt.plot(t,y)
plt.xlabel("Time Step")
plt.ylabel("Y position (m)")
plt.show()

