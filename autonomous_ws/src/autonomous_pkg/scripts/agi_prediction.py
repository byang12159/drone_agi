import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
import torch
import time

# torch.manual_seed(42)
class Prediction():
    def __init__(self, num_trajectory, steps, fov_h= 70):
        # Given FOV of pinhole camera and distance from camera, computes the rectangle range of observable image
        # Arducam B0385 Global Shutter Camera

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # self.fov_h = torch.tensor(fov_h).to(self.device)
        self.fov_h = fov_h
        # self.aspect_ratio = torch.tensor(4/3).to(self.device)
        self.aspect_ratio = 4/3
        self.fov_h_radians = self.fov_h*np.pi/180.0
        self.fov_v_radians = 2 * np.arctan(np.tan(self.fov_h_radians / 2) / self.aspect_ratio)

        self.num_trajectory = num_trajectory
        self.steps = steps

        # create steps x6 due to 6D dimension state: x y z vx vy vz
        self.total_trajectories = torch.empty(self.num_trajectory, (self.steps+1), 6).to(self.device)
        
    
    def calculate_viewable_area(self, distance):
        viewable_width = 2 * distance * torch.tan(self.fov_h_radians / 2)
        viewable_height = 2 * distance * torch.tan(self.fov_v_radians / 2)
        
        return viewable_width, viewable_height

    # def calculate_backoff(self,bound):
    #     return bound/(torch.tan(self.fov_h_radians / 2))

    def calculate_backoff(self, rect):
        center = (rect[0,:] + rect[1,:])/2 
        rad = (rect[1,:] - rect[0,:])/2 
        backoff_y = rad[1]/np.tan(self.fov_h_radians/2)
        backoff_z = rad[2]/np.tan(self.fov_v_radians/2)
        backoff_dist = max(backoff_y, backoff_z)-0.5
        if backoff_dist < 0:
            backoff_dist = 0 
        # else:
        #     backoff_dist = backoff_dist+x_dist
        return [center[0]+backoff_dist, center[1], center[2]]

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

    def compute_reach(self, initial_state,  timestep, accel_range,  detection_rate):
        
        initial_state = torch.from_numpy(initial_state).to(self.device)
        # ego_state = torch.from_numpy(ego_state).to(self.device)

        for s in range(1, self.total_trajectories.shape[1]):
            sample_accel = torch.rand(self.num_trajectory,3).to(self.device)* (2 * accel_range) - accel_range
            new_vel = self.total_trajectories[:,s,3:]   + sample_accel * timestep
            new_pos = self.total_trajectories[:,s,:3] + new_vel * timestep
      
            self.total_trajectories[:,s,:3] = new_pos 
            self.total_trajectories[:,s,3:] = new_vel
            # self.total_trajectories[:, s]   = new_pos[:,0]
            # self.total_trajectories[:, s+1] = new_pos[:,1]
            # self.total_trajectories[:, s+2] = new_pos[:,2]
            # self.total_trajectories[:, s+3] = new_vel[:,0]
            # self.total_trajectories[:, s+4] = new_vel[:,1]
            # self.total_trajectories[:, s+5] = new_vel[:,2]
  

        # # Find bound of volume and impose IRL Flying arena physical constraints
        # x_vals = self.total_trajectories[:,0::6]
        # y_vals = self.total_trajectories[:,1::6]
        # z_vals = self.total_trajectories[:,2::6]
        # min_x = max(torch.min(x_vals).item(), initial_state[0]-0.5).item()
        # max_x = min(torch.max(x_vals).item(), initial_state[0]+0.3).item()
        # min_y = torch.min(y_vals).item()
        # max_y = torch.max(y_vals).item()
        # min_z = torch.min(z_vals).item()
        # max_z = torch.max(z_vals).item()
        # bounds = [min_x,max_x, min_y,max_y, min_z,max_z]
        rectangle = []
        for i in range(1, self.total_trajectories.shape[1]):
            min_x = torch.min(self.total_trajectories[:,i,0]).item()
            max_x = torch.max(self.total_trajectories[:,i,0]).item()
            min_y = torch.min(self.total_trajectories[:,i,1]).item()
            max_y = torch.max(self.total_trajectories[:,i,1]).item()
            min_z = torch.min(self.total_trajectories[:,i,2]).item()
            max_z = torch.max(self.total_trajectories[:,i,2]).item()

            rectangle.append([[min_x, min_y, min_z], [max_x, max_y, max_z]])

        rectangle = np.array(rectangle)

        # Compute needed backoff given y-bound
        # backoff = self.calculate_backoff((max_y-min_y))
        backoff_state = self.calculate_backoff(rectangle[-1,:,:])

        return self.total_trajectories, backoff_state, rectangle

       

if __name__ == "__main__":
    p = Prediction(num_trajectory = 100, steps=10)
    
    leader_state = np.array([2.0,0,4.,0.,0.,0.])
    ego_pose = np.array([1.8,0.,4.])

    starttime = time.time()

    total_trajectories, backoff, bounds = p.compute_reach(leader_state, 0.1, 5, 0.94)

    bound_vertex = np.array([bounds[0]-backoff, (bounds[2]+bounds[3])/2,  (bounds[4]+bounds[5])/2])
    
    print("runtime",time.time()-starttime)


    ########## Visualize prep ########## 
    np_total_trajectories = total_trajectories.cpu().numpy()
    trajectories = []
    for j in range(p.num_trajectory):
        single_trajectory = np.zeros((3, p.steps+1))
        index = 0
        for s in range(0, np_total_trajectories.shape[1]):
            single_trajectory[0,index] = np_total_trajectories[j][s][0]
            single_trajectory[1,index] = np_total_trajectories[j][s][1]
            single_trajectory[2,index] = np_total_trajectories[j][s][2]
            index+=1
        trajectories.append(single_trajectory)

    # 3D Figure
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')

    for t in range(p.num_trajectory):
        ax.plot(trajectories[t][0,:], trajectories[t][1,:], trajectories[t][2,:], '-x',color='b')
    
    p.plot_rec(ax, bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5])

    ax.scatter(leader_state[0],leader_state[1], leader_state[2], color='red', label='leader drone')
    ax.scatter(ego_pose[0],ego_pose[1], ego_pose[2], color='green', label='chaser drone')
    ax.scatter(bound_vertex[0], bound_vertex[1], bound_vertex[2], color='green', label='chaser drone')

    # Plot pyramid
    base_corners = np.array([
    [bounds[0], bounds[2], bounds[4]],
    [bounds[0], bounds[3], bounds[4]],
    [bounds[0], bounds[3], bounds[5]],
    [bounds[0], bounds[2], bounds[5]]
    ])

    vertex = bound_vertex
    points = np.vstack((base_corners, vertex))

    # Plot the base of the pyramid
    for i in range(len(base_corners)):
        next_i = (i + 1) % len(base_corners)
        ax.plot([base_corners[i, 0], base_corners[next_i, 0]],
                [base_corners[i, 1], base_corners[next_i, 1]],
                [base_corners[i, 2], base_corners[next_i, 2]], color='b')

    # Plot the edges connecting the base to the vertex
    for i in range(len(base_corners)):
        ax.plot([base_corners[i, 0], vertex[0]],
                [base_corners[i, 1], vertex[1]],
                [base_corners[i, 2], vertex[2]], color='b')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()
