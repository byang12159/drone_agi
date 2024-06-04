import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Prediction():
    def __init__(self, num_trajectory, num_steps, accel_range, follow_depth, timestep):
        self.num_trajectory = num_trajectory
        self.num_steps = num_steps
        self.accel_range = accel_range
        self.timestep = timestep
        self.follow_depth = follow_depth

        self.cam_fov_h = 100 #degrees
        self.cam_fov_d = 138 #degrees

    def compute_cam_bound(self):
        # Given FOV of pinhole camera and distance from camera, computes the rectangle range of observable image

        rec_width = 2 * (np.tan(np.deg2rad(self.cam_fov_h/2)) * self.follow_depth )
        b = 2 * (np.tan(np.deg2rad(self.cam_fov_d/2)) * self.follow_depth )
        rec_height = np.sqrt(b**2 - rec_width**2)

        return rec_width,rec_height


    def plot_rec(self, ax, min_x,max_x,min_y,max_y,min_z,max_z):
        x = [min_x, max_x, max_x, min_x, min_x, max_x, max_x, min_x]
        y = [min_y, min_y, max_y, max_y, min_y, min_y, max_y, max_y]
        z = [min_z, min_z, min_z, min_z, max_z, max_z, max_z, max_z]

        # Define connections between the corner points
        connections = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ]

        # Plot wireframe
        for connection in connections:
            ax.plot([x[connection[0]], x[connection[1]]],
                    [y[connection[0]], y[connection[1]]],
                    [z[connection[0]], z[connection[1]]], 'k-')


    def prediction(self, initial_state, visualize = False):
       
        total_trajectories=[]

        # Generate Trajectories
        for i in range(self.num_trajectory):
            trajectory = [initial_state]
            for s in range(self.num_steps):
                a = np.random.uniform(-self.accel_range, self.accel_range, size=3)
                v = trajectory[-1][3:6] + a * self.timestep
                p = trajectory[-1][:3] + v * self.timestep
                trajectory.append([p[0], p[1], p[2], v[0], v[1], v[2], a[0],a[1],a[2]])

            total_trajectories.append(trajectory)
        
        # Find Hyper-rectangles of Trajectories
        total_trajectories=np.array(total_trajectories)
        rectangle = []
        for s in range(self.num_steps+1):
            min_x = np.min(total_trajectories[:,s,0])
            max_x = np.max(total_trajectories[:,s,0])
            min_y = np.min(total_trajectories[:,s,1])
            max_y = np.max(total_trajectories[:,s,1])
            min_z = np.min(total_trajectories[:,s,2])
            max_z = np.max(total_trajectories[:,s,2])
            rectangle.append([min_x,max_x,min_y,max_y,min_z,max_z])

        if visualize: 
            print("ENDT")
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            

            for i in range(self.num_trajectory):
                trajectory=np.array(total_trajectories[i])
                ax.plot(trajectory[:,0], trajectory[:,1], trajectory[:,2], color='b')

            for j in range(self.num_steps+1):
                rectangle_j = rectangle[j]
                self.plot_rec(ax, rectangle_j[0],rectangle_j[1],rectangle_j[2],rectangle_j[3],rectangle_j[4],rectangle_j[5])

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            plt.show()
        
        averaged_trajectory = np.mean(total_trajectories, axis=0)
        return averaged_trajectory, rectangle


if __name__ == "__main__":

    pred = Prediction(num_trajectory = 50, num_steps = 15, accel_range = 5, follow_depth=2, timestep = 0.01)

    state_est = [0,0,0,1,2,0,0,0,0]
    averaged_trajectory, rectangle = pred.prediction(initial_state=state_est, visualize=True)

    print("DONE")