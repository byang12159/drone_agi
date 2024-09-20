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
torch.manual_seed(42)

class RunParticle():
    def __init__(self,starting_state, width=320, height=320, fov=50, batch_size=32):

        self.inital_state = starting_state

        ####################### Initialize Variables #######################

        self.format_particle_size = 0
        # bounds for particle initialization, meters + degrees
        self.total_particle_states = 6
        self.filter_dimension = 3
        self.min_bounds = {'px':-0.5,'py':-0.5,'pz':-0.5,'rz':-2.5,'ry':-179.0,'rx':-2.5,'pVx':-0.3,'pVy':-0.3,'pVz':-0.3,'Ax':-0.5,'Ay':-0.5,'Az':-0.5}
        self.max_bounds = {'px':0.5,'py':0.5,'pz':0.5,'rz':2.5,'ry':179.0,'rx':2.5,      'pVx':0.3, 'pVy':0.3, 'pVz':0.3, 'Ax':0.5,'Ay':0.5,'Az':0.5}

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

        self.past_10_velocity = None

        time_step = 0.03
        self.kf_y = None
        self.kf_P_y = np.eye(3) 
        self.kf_A_y = np.array([
            [1,0,time_step],
            [1,0,0],
            [0.05*1/0.03, 0.05*(-1)/0.03, 0.95*1]
        ])
        self.kf_Q_y = np.eye(3)
        self.kf_H_y = np.array([[1,0,0],[0,1,0]])
        self.kf_R_y = np.eye(2)

        self.kf_z = None
        self.kf_P_z = np.eye(3) 
        self.kf_A_z = np.array([
            [1,0,time_step],
            [1,0,0],
            [0.01*1/0.03, 0.01*(-1)/0.03, 0.99*1]
        ])
        self.kf_Q_z = np.eye(3)
        self.kf_H_z = np.array([[1,0,0],[0,1,0]])
        self.kf_R_z = np.eye(2)

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
        
        for index, particle_noise in enumerate(self.initial_particles_noise):
            x = self.inital_state[0] + particle_noise[0]
            y = self.inital_state[1] + particle_noise[1]
            z = self.inital_state[2] + particle_noise[2]
            Vx = particle_noise[3]
            Vy = particle_noise[4]
            Vz = particle_noise[5]

            initial_positions[index,:] = torch.tensor([x, y, z], device=self.device)
            initial_velocities[index,:] = torch.tensor([Vx, Vy, Vz], device=self.device)

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

        current_y = current_pose[1]
        past_y = past_states1[1]
        if self.kf_y is None:
            self.kf_y = np.array([[current_y],[current_y],[0]])
        self.kf_y = self.kf_A_y@self.kf_y 
        self.kf_P_y = self.kf_A_y@self.kf_P_y@self.kf_A_y.T+self.kf_Q_y 
        z = np.array([[current_y], [past_y]])
        y = z-self.kf_H_y@self.kf_y 
        S = self.kf_H_y@self.kf_P_y@self.kf_H_y.T+self.kf_R_y 
        self.K = self.kf_P_y@self.kf_H_y.T@np.linalg.inv(S)
        self.kf_y = self.kf_y+self.K@y 
        self.kf_P_y = self.kf_P_y-self.K@self.kf_H_y@self.kf_P_y 

        current_z = current_pose[2]
        past_z = past_states1[2]
        if self.kf_z is None:
            self.kf_z = np.array([[current_z],[current_z],[0]])
        self.kf_z = self.kf_A_z@self.kf_z 
        self.kf_P_z = self.kf_A_z@self.kf_P_z@self.kf_A_z.T+self.kf_Q_z 
        z = np.array([[current_z], [past_z]])
        y = z-self.kf_H_z@self.kf_z 
        S = self.kf_H_z@self.kf_P_z@self.kf_H_z.T+self.kf_R_z 
        self.K = self.kf_P_z@self.kf_H_z.T@np.linalg.inv(S)
        self.kf_z = self.kf_z+self.K@y 
        self.kf_P_z = self.kf_P_z-self.K@self.kf_H_z@self.kf_P_z 


        current_pose = torch.tensor(current_pose).to(self.device)
        past_states1 = torch.tensor(past_states1).to(self.device)
        
        self.odometry_update(time_step) 
        # make copies to prevent mutations
        particles_position_before_update = self.filter.particles['position'].clone().detach()
        particles_velocity_before_update = self.filter.particles['velocity'].clone().detach()

        current_velocity  = (current_pose-past_states1[:3])/time_step
        # if self.past_10_velocity is None:
        #     self.past_10_velocity = current_velocity.repeat(10,1)
        # else:
        #     self.past_10_velocity = torch.cat((self.past_10_velocity[1:,:], current_velocity), dim=0)

        losses = self.get_loss(current_pose, current_velocity, particles_position_before_update, particles_velocity_before_update)

        offset_val = 1
        self.filter.weights = 1/(losses+offset_val)

        # Resample Weights
        self.filter.update()
        self.num_updates += 1

        position_est = self.filter.compute_weighted_position_average()
        velocity_est = self.filter.compute_weighted_velocity_average()
        state_est = torch.cat((position_est, velocity_est, torch.FloatTensor([position_est[1],velocity_est[1],position_est[2],velocity_est[2]]).cuda()))
        state_est[1] = self.kf_y[0,0]
        state_est[2] = self.kf_z[0,0]
        state_est[4] = self.kf_y[2,0]
        state_est[5] = self.kf_z[2,0]
        # state_est = torch.cat((position_est, current_velocity))

        self.state_est_history.append(state_est)

        # Update odometry step
        # print("state est:",state_est)
        # print(f"Update # {self.num_updates}, Iteration runtime: {time.time() - start_time}")

        # # Update velocity with newest observation:
        # self.filter.update_vel(current_pose,timestep)
        # Update velocity with newest observation:
        # self.filter.update_vel(particles_position_before_update,current_pose,position_est, lastpose,time_step)
        variance = self.filter.compute_var()

        return state_est, variance
    
#######################################################################################################################################
if __name__ == "__main__":

    simple_trajx = np.arange(0,1,0.03)
    simple_trajx = simple_trajx.reshape(simple_trajx.shape[0],1)
    simple_trajx = np.concatenate((np.zeros((15,1)), simple_trajx, np.ones((15,1))), axis=0)
    simple_traj = np.hstack((simple_trajx, np.ones_like(simple_trajx), np.zeros_like(simple_trajx)))

    mcl = RunParticle(starting_state=simple_traj[0])    

    start_time = time.time()
    
    particle_state_est=[[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]
    variance_history = []
    PF_history = [np.array(mcl.filter.particles['position'].cpu())]
    prediction_history = []
    
    # Assume constant time step between trajectory stepping
    time_step = 0.03

    for iter in range(1,simple_traj.shape[0]):
        
        state_est, variance = mcl.rgb_run(current_pose= simple_traj[iter], past_states1=particle_state_est[-1], time_step=time_step )   

        particle_state_est.append(state_est.cpu().numpy())
        variance_history.append(variance)
        PF_history.append(np.array(mcl.filter.particles['position'].cpu()))
    


    particle_state_est = particle_state_est[2:]
    particle_state_est = np.array(particle_state_est)
    
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    t = np.linspace(0, 32, 1000)
    ax.plot(simple_traj[:,0],simple_traj[:,1],simple_traj[:,2], color = 'b')
    ax.plot(particle_state_est[:,0], particle_state_est[:,1], particle_state_est[:,2], color = 'g')
    plt.show()

    times = np.arange(0,particle_state_est.shape[0]*0.03, 0.03)
    y_min = -0.5
    y_max = 1.5
    fig, (posx,posy,posz,velx,vely,velz) = plt.subplots(6, 1, figsize=(16, 10))
    posx.plot(times, particle_state_est[:,0], label = "Est Pos x")
    posx.plot(times, simple_traj[1:,0], label = "GT Pos x")
    posx.set_ylim(y_min, y_max)
    posx.legend() 
    posy.plot(times, particle_state_est[:,1], label = "Est Pos y")
    posy.plot(times, simple_traj[1:,1], label = "GT Pos y")
    posy.set_ylim(y_min, y_max)
    posy.legend()
    posz.plot(times, particle_state_est[:,2], label = "Est Pos z")
    posz.plot(times, simple_traj[1:,2], label = "GT Pos z")
    posz.set_ylim(y_min, y_max)
    posz.legend()
    velx.plot(times, particle_state_est[:,3], label = "GT Vel x")
    velx.legend() 
    vely.plot(times, particle_state_est[:,4], label = "GT Vel y")
    vely.legend()
    velz.plot(times, particle_state_est[:,5], label = "GT Vel z")
    velz.legend()
    plt.tight_layout()
    plt.show()


    # Particle Viewer
    # for i in range(len(PF_history)):
    #     fig = plt.figure(1)
    #     ax = fig.add_subplot(111, projection='3d')
    #     t = np.linspace(0, 32, 1000)
    #     ax.plot(simple_traj[:,0],simple_traj[:,1],simple_traj[:,2], color = 'b')
    #     ax.scatter(particle_state_est[i,0], particle_state_est[i,1], particle_state_est[i,2], c='r', s=100)
    #     ax.scatter(PF_history[i][:,0], PF_history[i][:,1], PF_history[i][:,2], c='g', alpha=0.15)
    #     plt.show()

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

