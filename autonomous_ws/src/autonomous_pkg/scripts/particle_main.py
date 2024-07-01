import numpy as np
import scipy
import time
from numpy import cos, sin
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
from particle_filter import ParticleFilter
from scipy.integrate import odeint
import os
from pathlib import Path
from scipy.spatial.transform import Rotation
import copy
import torch

class RunParticle():
    def __init__(self,starting_state, width=320, height=320, fov=50, batch_size=32):

        self.inital_state = starting_state

        ####################### Initialize Variables #######################

        self.format_particle_size = 0
        # bounds for particle initialization, meters + degrees
        self.total_particle_states = 6
        self.filter_dimension = 3
        self.min_bounds = {'px':-0.5,'py':-0.5,'pz':-0.5,'rz':-2.5,'ry':-179.0,'rx':-2.5,'pVx':-0.5,'pVy':-0.5,'pVz':-0.5,'Ax':-0.5,'Ay':-0.5,'Az':-0.5}
        self.max_bounds = {'px':0.5,'py':0.5,'pz':0.5,'rz':2.5,'ry':179.0,'rx':2.5,      'pVx':0.5, 'pVy':0.5, 'pVz':0.5, 'Ax':0.5,'Ay':0.5,'Az':0.5}

        self.num_particles = 900
        
        self.state_est_history = []

        self.use_convergence_protection = True
        self.convergence_noise = 0.2

        self.sampling_strategy = 'random'
        self.num_updates =0
        # self.control = Controller()

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    

        ####################### Generate Initial Particles #######################
        self.get_initial_distribution()

        # add initial pose estimate before 1st update step
        position_est = self.filter.compute_simple_position_average()
        velocity_est = self.filter.compute_simple_velocity_average()

        state_est = torch.cat((position_est, velocity_est))
        
        self.state_est_history.append(state_est)

    def mat3d(self, x,y,z):
        # Create a 3D figure
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        ax.scatter(x,y,z,'*')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        ax.set_xlim(-40, 40)  # Set X-axis limits
        ax.set_ylim(-40, 40)  # Set Y-axis limits
        ax.set_zlim(-40, 40)  # Set Z-axis limits
        # Show the plot
        plt.show()

    def get_initial_distribution(self):
        # get distribution of particles from user, generate np.array of (num_particles, 6)
        
        min_bounds = torch.tensor([self.min_bounds['px'], self.min_bounds['py'], self.min_bounds['pz'], 
                           self.min_bounds['pVx'], self.min_bounds['pVy'], self.min_bounds['pVz']], dtype=torch.float32)
        max_bounds = torch.tensor([self.max_bounds['px'], self.max_bounds['py'], self.max_bounds['pz'], 
                                self.max_bounds['pVx'], self.max_bounds['pVy'], self.max_bounds['pVz']], dtype=torch.float32)

        min_bounds = min_bounds.to(self.device)
        max_bounds = max_bounds.to(self.device)

        # Generate the initial particles noise using uniform distribution
        self.initial_particles_noise = torch.rand((self.num_particles, self.total_particle_states), device=self.device) * (max_bounds - min_bounds) + min_bounds
                
        # Dict of position + rotation, with position as np.array(300x6)
        self.initial_particles = self.set_initial_particles()
        
        # Initiailize particle filter class with inital particles
        self.filter = ParticleFilter(self.initial_particles ,self.device)

        # # Create a 3D figure
        # fig = plt.figure()
        # ax = fig.add_subplot(projection='3d')
        # ax.scatter(self.initial_particles.get('position')[:,0],self.initial_particles.get('position')[:,1],self.initial_particles.get('position')[:,2],'*')
        # ax.scatter(self.inital_state[0],self.inital_state[1],self.inital_state[2],'*')
        # ax.set_xlabel('X Label')
        # ax.set_ylabel('Y Label')
        # ax.set_zlabel('Z Label')
        # # ax.set_xlim(-4, 4)  # Set X-axis limits
        # # ax.set_ylim(-4, 4)  # Set Y-axis limits
        # # ax.set_zlim(-4, 4)  # Set Z-axis limits
        # # Show the plot
        # plt.show()

    def set_initial_particles(self):
        initial_positions = torch.zeros((self.num_particles, self.filter_dimension), dtype=torch.float32, device=self.device)
        initial_velocities = torch.zeros((self.num_particles, self.filter_dimension), dtype=torch.float32, device=self.device)
        
        for index, particle in enumerate(self.initial_particles_noise):
            x = self.inital_state[0] + particle[0]
            y = self.inital_state[1] + particle[1]
            z = self.inital_state[2] + particle[2]
            Vx = particle[3]
            Vy = particle[4]
            Vz = particle[5]

            initial_positions[index,:] = torch.tensor([x, y, z], device='cuda')
            initial_velocities[index,:] = torch.tensor([Vx, Vy, Vz], device='cuda')

        return {'position': initial_positions, 'velocity': initial_velocities}


    def odometry_update(self, system_time_interval):
        # Use current estimate of x,y,z,Vx,Vy,Vz and dynamics model to compute most probable system propagation

        self.filter.particles['position'] += system_time_interval*self.filter.particles['velocity']
   

    
    def get_loss(self, current_pose, current_vel, particle_poses, particle_vel):
  
        position_loss = torch.sqrt(torch.sum((particle_poses -current_pose) ** 2, dim=1))
        velocity_loss = torch.sqrt(torch.sum((particle_vel   -current_vel)  ** 2, dim=1))

        losses = position_loss + 0.8*velocity_loss

        return losses

    def rgb_run(self,current_pose, past_states1, time_step):
        start_time = time.time() 

        current_pose = torch.tensor(current_pose).to(self.device)
        past_states1 = torch.tensor(past_states1).to(self.device)
        
        self.odometry_update(time_step) 
        # make copies to prevent mutations
        particles_position_before_update = self.filter.particles['position'].clone().detach()
        particles_velocity_before_update = self.filter.particles['velocity'].clone().detach()

        current_velocity  = (current_pose-past_states1[:3])/time_step

        losses = self.get_loss(current_pose, current_velocity, particles_position_before_update, particles_velocity_before_update)

        offset_val = 1
        self.filter.weights = 1/(losses+offset_val)

        # Resample Weights
        self.filter.update()
        self.num_updates += 1

        position_est = self.filter.compute_weighted_position_average()
        velocity_est = self.filter.compute_weighted_velocity_average()
        state_est = torch.cat((position_est, velocity_est))

        self.state_est_history.append(state_est)

        # Update odometry step
        # print("state est:",state_est)
        print(f"Update # {self.num_updates}, Iteration runtime: {time.time() - start_time}")

        # # Update velocity with newest observation:
        # self.filter.update_vel(current_pose,timestep)
        # Update velocity with newest observation:
        # self.filter.update_vel(particles_position_before_update,current_pose,position_est, lastpose,time_step)
        variance = self.filter.compute_var()

        return state_est, variance
    
#######################################################################################################################################
if __name__ == "__main__":

    simple_trajx = np.arange(0,100,1).reshape(100,1)
    simple_traj = np.hstack((simple_trajx, np.ones_like(simple_trajx), np.zeros_like(simple_trajx)))

    mcl = RunParticle(starting_state=simple_traj[0])    

    start_time = time.time()
    
    particle_state_est=[[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]
    variance_history = []
    PF_history = [np.array(mcl.filter.particles)]
    prediction_history = []
    
    # Assume constant time step between trajectory stepping
    time_step = 1

    for iter in range(1,100):
        
        state_est, variance = mcl.rgb_run(current_pose= simple_traj[iter], past_states1=particle_state_est[-1], time_step=time_step )   

        particle_state_est.append(state_est)
        variance_history.append(variance)
        PF_history.append(np.array(mcl.filter.particles))
    
    # PF_history_x = np.array(PF_history_x)
    # PF_history_y = np.array(PF_history_y)
    # PF_history_z = np.array(PF_history_z)

    # times = np.arange(0,time_step*len(pose_est_history_x),time_step)
    # velocity_GT = (mcl.ref_traj[1:]-mcl.ref_traj[:-1])/time_step


    # fig, (vel) = plt.subplots(1, 1, figsize=(14, 10))
    # vel.plot(times, velocity_GT[:len(times),0], label = "GT Vel x")
    # vel.plot(times, velocity_GT[:len(times),1], label = "GT Vel y")
    # vel.plot(times, velocity_GT[:len(times),2], label = "GT Vel z")
    # vel.legend()
    # plt.show()

    particle_state_est = np.array(particle_state_est[2:])
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    t = np.linspace(0, 32, 1000)
    ax.plot(simple_traj[:,0],simple_traj[:,1],simple_traj[:,2], color = 'b')
    ax.plot(particle_state_est[:,0], particle_state_est[:,1], particle_state_est[:,2], color = 'g')
    plt.show()

    # SIM_TIME = 40.0 
    # DT = SIM_TIME/len(pose_est_history_x)  # time tick [s]
    # print("DT is ",DT)
    # time = 0.0
    # show_animation = False
    # count = 0

    # # Initialize a 3D plot
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # # Simulation loop
    # while SIM_TIME >= time:
    #     time += DT
        

    #     if show_animation:
    #         ax.cla()  # Clear the current axis

    #         # For stopping simulation with the esc key.
    #         fig.canvas.mpl_connect('key_release_event',
    #                             lambda event: [exit(0) if event.key == 'escape' else None])

    #         # Plot the trajectory up to the current count in 3D
    #         ax.plot(mcl.ref_traj[:count, 0], mcl.ref_traj[:count, 1], mcl.ref_traj[:count, 2], "*k")
    #         ax.plot(pose_est_history_x[count], pose_est_history_y[count], pose_est_history_z[count], "*r" )
    #         # ax.plot(PF_history_x[count],PF_history_y[count],PF_history_y[count], 'o',color='blue', alpha=0.5)
    #         # Additional plotting commands can be added here
            
    #         ax.set_xlabel('X')
    #         ax.set_ylabel('Y')
    #         ax.set_zlabel('Z')
    #         ax.set_xlim(-40, 40)  # Set X-axis limits
    #         ax.set_ylim(-40, 40)  # Set Y-axis limits
    #         ax.set_zlim(-40, 40)  # Set Z-axis limits

    #         ax.axis("equal")
    #         ax.set_title('3D Trajectory Animation')
    #         plt.grid(True)
    #         plt.pause(0.001)
    #     count += 1  # Increment count to update the trajectory being plotted

    # # Show the final plot after the simulation ends
    # plt.show()


    print("FINISHED CODE")