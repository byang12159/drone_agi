import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
import torch
import time

torch.manual_seed(42)
class Prediction():
    def __init__(self, fov_h= 70):
        # Given FOV of pinhole camera and distance from camera, computes the rectangle range of observable image
        # Arducam B0385 Global Shutter Camera

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.fov_h = torch.tensor(fov_h).to(self.device)
        self.aspect_ratio = torch.tensor(4/3).to(self.device)
        self.fov_h_radians = self.fov_h*torch.pi/180.0
        self.fov_v_radians = 2 * torch.atan(torch.tan(self.fov_h_radians / 2) / self.aspect_ratio)
        
    
    def calculate_viewable_area(self, distance):
        viewable_width = 2 * distance * torch.tan(self.fov_h_radians / 2)
        viewable_height = 2 * distance * torch.tan(self.fov_v_radians / 2)
        
        return viewable_width, viewable_height


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
                    [z[connection[0]], z[connection[1]]], 'k-', color='red')

    def get_min_sphere(self, centerpoint, min_x,max_x,min_y,max_y,min_z,max_z):
        min_radius = 10000
        x = [min_x, max_x, max_x, min_x, min_x, max_x, max_x, min_x]
        y = [min_y, min_y, max_y, max_y, min_y, min_y, max_y, max_y]
        z = [min_z, min_z, min_z, min_z, max_z, max_z, max_z, max_z]

        for i in range(8):
            distance = la.norm(centerpoint - np.array([x[i],y[i],z[i]]))
            if distance < min_radius:
                min_radius = distance

        return min_radius

    def find_prediction(self, initial_state, ego_state, timestep, accel_range, steps = 2, num_trajectory = 100, visualize = False):
        
        # initial_state = initial_state.to(self.device)
        ego_state = torch.from_numpy(ego_state).to(self.device)

        # create x6 due to 6D dimension: x y z vx vy vz
        total_trajectories = torch.empty(num_trajectory, (steps+1)*6 ).to(self.device)

        total_trajectories[:, 0] = initial_state[0]
        total_trajectories[:, 1] = initial_state[1]
        total_trajectories[:, 2] = initial_state[2]
        total_trajectories[:, 3] = initial_state[3]
        total_trajectories[:, 4] = initial_state[4]
        total_trajectories[:, 5] = initial_state[5]

        for s in range(6, total_trajectories.shape[1], 6):
            sample_accel = torch.rand(num_trajectory,3).to(self.device)* (2 * accel_range) + accel_range
            new_vel = total_trajectories[:,s-3:s]   + sample_accel * timestep
            new_pos = total_trajectories[:,s-6:s-3] + new_vel * timestep
        
            # print(sample_accel.shape)
            # print(new_vel.shape)
            # print(new_pos.shape)
            # print(total_trajectories[:, s].shape)
            # print(torch.rand(num_trajectory).shape)
            total_trajectories[:, s]   = new_pos[:,0]
            total_trajectories[:, s+1] = new_pos[:,1]
            total_trajectories[:, s+2] = new_pos[:,2]
            total_trajectories[:, s+3] = new_vel[:,0]
            total_trajectories[:, s+4] = new_vel[:,1]
            total_trajectories[:, s+5] = new_vel[:,2]
                

        # Find Hyper-rectangles of Trajectories
        rectangle = []
        
        bound_y,bound_z = self.calculate_viewable_area(torch.norm(ego_state-initial_state[:3]))

        for s in range(6, total_trajectories.shape[1], 6):
            min_x = torch.min(total_trajectories[:,s])
            max_x = torch.max(total_trajectories[:,s])
            min_y = torch.min(total_trajectories[:,s+1])
            max_y = torch.max(total_trajectories[:,s+1])
            min_z = torch.min(total_trajectories[:,s+2])
            max_z = torch.max(total_trajectories[:,s+2])

            if torch.abs(max_y-min_y)  > bound_y:
                break
            if torch.abs(max_z-min_z)  > bound_z:
                break
            rectangle.append([min_x,max_x,min_y,max_y,min_z,max_z])

        # # Find predict trajectory by average
        # predict_trajectory = []
        # for s in range(steps+1):
        #     x_val = np.mean(total_trajectories[:,s,0])
        #     y_val = np.mean(total_trajectories[:,s,1])
        #     z_val = np.mean(total_trajectories[:,s,2])
        #     predict_trajectory.append([x_val, y_val, z_val])
        #     print(predict_trajectory)
        #     return

        # # Compute Variance
        # variance = []
        # for r in range(steps):
        #     variance.append(self.get_min_sphere(predict_trajectory[r],rectangle[r][0], rectangle[r][1], rectangle[r][2], rectangle[r][3], rectangle[r][4], rectangle[r][5]))

        # if visualize:
        #     # Trajectory Plot
        #     # fig = plt.figure(1)
        #     # ax = fig.add_subplot(111, projection='3d')

        #     # predict_trajectory = np.array(predict_trajectory)
        #     # ax.plot(predict_trajectory[:,0], predict_trajectory[:,1], predict_trajectory[:,2], '-x',color='r', label="Avg Traj")

        #     # for t in range(num_trajectory):
        #     #     ax.plot(total_trajectories[t][:,0], total_trajectories[t][:,1], total_trajectories[t][:,2], '-x',color='b')

        #     # for r in range(len(rectangle)):
        #     #     self.plot_rec(ax, rectangle[r][0], rectangle[r][1], rectangle[r][2], rectangle[r][3], rectangle[r][4], rectangle[r][5])

        #     # # plot_rec(ax, 0,1,-bound_y/2,bound_y/2,-bound_z/2,bound_y/2)
        #     # ax.set_xlabel('X')
        #     # ax.set_ylabel('Y')
        #     # ax.set_zlabel("Z")
        #     # plt.show()

        #     # Variance Plot
        #     s = np.arange(0,steps,1)
        #     print(variance)
        #     plt.figure(figsize=(10, 5))
        #     plt.plot(s,variance, marker='o')
        #     plt.xlabel('Steps')
        #     plt.ylabel('Variance')
        #     plt.title('Prediction Variance')
        #     plt.grid(True)
        #     plt.show()


        return rectangle, total_trajectories
    
if __name__ == "__main__":
    p = Prediction()
    for i in range(1):
        starttime = time.time()
        rectangles, total_traj = p.find_prediction(torch.tensor([2.0,3.,4.,0.,0.,0.]), torch.tensor([1.8,0.,4.]), 0.01, 4, steps = 2, num_trajectory = 6, visualize = False)
        print("runtime",time.time()-starttime)
        print(rectangles)