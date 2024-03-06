import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from utils import transformations
from scipy.optimize import least_squares

R = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

print(transformations.quaternion_from_matrix(R))

# def s(theta, t):
#     x = theta[0] + theta[2] * np.cos(t)
#     y = theta[1] + theta[2] * np.sin(t)
#     return np.array([x, y])

# ts = np.linspace(0, 2 * np.pi)
# cx = 1.5; cy = 1.3; r = 0.75; noise = 0.05
# ss = s([cx, cy, r], ts)
# ss[0] += noise * np.random.rand(ts.shape[0])
# ss[1] += noise * np.random.rand(ts.shape[0])

# def fun(theta):
#     return (s(theta, ts) - ss).flatten()

# theta0 = [0,0,0]
# res2 = least_squares(fun, theta0)
# print(res2.x.tolist())

# traj = s(res2.x.tolist(),ts)
# plt.plot(ss[0],ss[1],'o')
# plt.plot(traj[0],traj[1])
# plt.show()


# x0 = np.zeros(7)
# # Define a learning rate and number of iterations
# learning_rate = 0.001
# num_iterations = 1000

# # Define a cost function (e.g., mean squared error)
# def cost_function(x, A, b):
#     error = A @ x - b
#     return np.mean(error**2)

# # Gradient descent loop
# for i in range(num_iterations):
#     # Calculate gradient
#     gradient = 2 * A.T @ (A @ x - b)
    
#     # Update x
#     x = x - learning_rate * gradient

# print(x)