import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from particle_filter import ParticleFilter
from prediction import Prediction

class RunParticle():
    def __init__(self,starting_state, width=320, height=320, fov=50, batch_size=32):

        self.inital_state = starting_state

        ####################### Initialize Variables #######################

        self.format_particle_size = 0
        # bounds for particle initialization, meters + degrees
        self.total_particle_states = 9
        self.filter_dimension = 3
        self.min_bounds = {'px':-0.5,'py':-0.5,'pz':-0.5,'rz':-2.5,'ry':-179.0,'rx':-2.5,'pVx':-0.5,'pVy':-0.5,'pVz':-0.5,'Ax':-0.5,'Ay':-0.5,'Az':-0.5}
        self.max_bounds = {'px':0.5,'py':0.5,'pz':0.5,'rz':2.5,'ry':179.0,'rx':2.5,      'pVx':0.5, 'pVy':0.5, 'pVz':0.5, 'Ax':0.5,'Ay':0.5,'Az':0.5}

        self.num_particles = 1800
        
        self.state_est_history = []

        self.use_convergence_protection = True
        self.convergence_noise = 0.2

        self.sampling_strategy = 'random'
        self.num_updates =0
        # self.control = Controller()

        ####################### Generate Initial Particles #######################
        self.get_initial_distribution()

        # add initial pose estimate before 1st update step
        position_est = self.filter.compute_simple_position_average()
        velocity_est = self.filter.compute_simple_velocity_average()
        accel_est = self.filter.compute_simple_accel_average()
        state_est = np.concatenate((position_est, velocity_est, accel_est))
        print("state_est",state_est)
        self.state_est_history.append(state_est)

        print("state",state_est)


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
        self.initial_particles_noise = np.random.uniform(
            np.array([self.min_bounds['px'], self.min_bounds['py'], self.min_bounds['pz'],self.min_bounds['pVx'], self.min_bounds['pVy'], self.min_bounds['pVz'],self.min_bounds['Ax'], self.min_bounds['Ay'], self.min_bounds['Az']]),
            np.array([self.max_bounds['px'], self.max_bounds['py'], self.max_bounds['pz'],self.max_bounds['pVx'], self.max_bounds['pVy'], self.max_bounds['pVz'],self.max_bounds['Ax'], self.max_bounds['Ay'], self.max_bounds['Az']]),
            size = (self.num_particles, self.total_particle_states))
        
        # Dict of position + rotation, with position as np.array(300x6)
        self.initial_particles = self.set_initial_particles()
        
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

        # self.mat3d(self.initial_particles.get('position')[:,0],self.initial_particles.get('position')[:,1],self.initial_particles.get('position')[:,2])

        # Initiailize particle filter class with inital particles
        self.filter = ParticleFilter(self.initial_particles)


    def set_initial_particles(self):
        initial_positions =  np.zeros((self.num_particles, self.filter_dimension))
        initial_velocities = np.zeros((self.num_particles, self.filter_dimension))
        initial_accels = np.zeros((self.num_particles, self.filter_dimension))
        
        for index, particle in enumerate(self.initial_particles_noise):
            # Initialize at origin location
            # i = self.ref_traj[0]
            i = self.inital_state
            x = i[0] + particle[0]
            y = i[1] + particle[1]
            z = i[2] + particle[2]
            Vx = particle[3]
            Vy = particle[4]
            Vz = particle[5]
            Accelx = particle[6]
            Accely = particle[7]
            Accelz = particle[8]

            # set positions
            initial_positions[index,:] = [x, y, z]
            initial_velocities[index,:] = [Vx, Vy, Vz]
            initial_accels[index,:] = [Accelx, Accely, Accelz]

        return  {'position':initial_positions, 'velocity':initial_velocities, 'accel':initial_accels}


    def odometry_update(self,current_pose, system_time_interval ):
        # Use current estimate of x,y,z,Vx,Vy,Vz and dynamics model to compute most probable system propagation
     
        # offset = system_time_interval*curr_state_est[3:]
        # offset = system_time_interval*curr_vel_est
        # coef = 0.7
        offsets=[]
        for i in range(self.num_particles):
            # offset = system_time_interval*self.filter.particles['velocity'][i]
            increment_vel = self.filter.particles['velocity'][i] + system_time_interval*self.filter.particles['accel'][i]
            offset = system_time_interval*increment_vel
            offsets.append(offset)
            self.filter.particles['position'][i] += offset
        offsets = np.array(offsets)
        # return np.average(offsets)
    
    def get_loss(self, current_pose, current_vel, current_accel, particle_poses, particle_vel, particle_accel):
        losses = []
        # print("cur",current_pose,last_pose)

        for i, particle in enumerate(particle_poses):
            loss = np.sqrt((current_pose[0]-particle[0])**2 + (current_pose[1]-particle[1])**2 + (current_pose[2]-particle[2])**2 
                           + 0.9*((current_vel[0]-particle_vel[i][0])**2+ + (current_vel[1]-particle_vel[i][1])**2+ + (current_vel[2]-particle_vel[i][2])**2) 
                           + 0.7*((current_accel[0]-particle_accel[i][0])**2+ + (current_accel[1]-particle_accel[i][1])**2+ + (current_accel[2]-particle_accel[i][2])**2))
            # loss = np.sqrt((current_pose[0]-particle[0])**2 )
            losses.append(loss)
                   
        return losses

    def rgb_run(self,current_pose, past_states, time_step):
        start_time = time.time() 

        self.odometry_update(current_pose,time_step) 
        # make copies to prevent mutations
        particles_position_before_update = np.copy(self.filter.particles['position'])
        particles_velocity_before_update = np.copy(self.filter.particles['velocity'])
        particles_accel_before_update    = np.copy(self.filter.particles['accel'])

        # velest = np.mean(particles_velocity_before_update,axis=0)
        # acelest = np.mean(particles_accel_before_update,axis=0)


        # current_velocity     = (np.array(current_pose)-np.array(past_states[-1][:3]))/time_step
        # current_acceleration = (np.array(current_velocity)-np.array(past_states[-1][3:6]))/time_step
        current_velocity     = (np.array(current_pose)-np.array(past_states[-1][:3]))/time_step
        current_acceleration = (np.array(past_states[-1][3:6])-np.array(past_states[-2][3:6]))/time_step
        losses = self.get_loss(current_pose, current_velocity, current_acceleration, particles_position_before_update, particles_velocity_before_update, particles_accel_before_update)

        temp = 1
        for index, particle in enumerate(particles_position_before_update):
            self.filter.weights[index] = 1/(losses[index]+temp)

        # Resample Weights
        self.filter.update()
        self.num_updates += 1

        position_est = self.filter.compute_weighted_position_average()
        velocity_est = self.filter.compute_weighted_velocity_average()
        accel_est = self.filter.compute_weighted_accel_average()
        state_est = np.concatenate((position_est, velocity_est, accel_est))

        self.state_est_history.append(state_est)

        # Update odometry step
        # print("state est:",state_est)
        print(f"Update # {self.num_updates}, Iteration runtime: {time.time() - start_time}")

        # # Update velocity with newest observation:
        # self.filter.update_vel(current_pose,timestep)
        # Update velocity with newest observation:
        # self.filter.update_vel(particles_position_before_update,current_pose,position_est, lastpose,time_step)

        return state_est
    
#######################################################################################################################################
if __name__ == "__main__":

    # Generate data 
    t = np.arange(0,5,0.01)
    x = np.zeros_like(t)
    y = np.sin(t*2*np.pi/5)
    z = np.ones_like(t)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    plt.show()

    # 2d Plot
    plt.plot(t, y, label='sin(t)')
    # Add labels and title
    plt.xlabel('X axis')
    plt.ylabel('Y axis')
    plt.title('Basic Line Plot')
    plt.legend()
    plt.show()

    simple_traj = np.hstack((x.reshape(x.shape[0],1), y.reshape(y.shape[0],1), z.reshape(z.shape[0],1)))
    
    # Assume constant time step between trajectory stepping
    timestep = 0.01

    mcl = RunParticle(starting_state=simple_traj[0])   
    pred = Prediction(num_trajectory = 50, num_steps = 15, accel_range = 5, follow_depth=2, timestep = timestep)

    pose_est_history_x = []
    pose_est_history_y = []
    pose_est_history_z = []
    velocity_est_history_x = []
    velocity_est_history_y =[]
    velocity_est_history_z = []
    PF_history_x = []
    PF_history_y = []
    PF_history_z = []
    PF_history_x.append(np.array(mcl.filter.particles['position'][:,0]).flatten())
    PF_history_y.append(np.array(mcl.filter.particles['position'][:,1]).flatten())
    PF_history_z.append(np.array(mcl.filter.particles['position'][:,2]).flatten())

    velocity_GT = []
    accel_GT = []

    global_state_history_L=[]
    global_state_history_C=[]
    particle_state_est=[[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]

    
    for iter in range(1, x.shape[0]):
        lead_pose = simple_traj[iter]
        state_est = mcl.rgb_run(current_pose=lead_pose, past_states = particle_state_est, time_step=timestep)   

        particle_state_est.append(state_est)

        predict_trajectory, rectangles = pred.prediction(initial_state=state_est)

    
    # Post Processing
    particle_state_est = np.array(particle_state_est)
    times = np.arange(0,particle_state_est.shape[0]-2)*timestep

    fig, (posx,posy,posz) = plt.subplots(3, 1, figsize=(14, 10))
    posx.plot(times, particle_state_est[2:,0], label = "Filter Pos x")
    # posx.plot(times, global_state_history_L[:,0], label = "GT Pos x")
    posx.legend()
    posy.plot(times, particle_state_est[2:,1], label = "Filter Pos y")    
    posy.plot(times, simple_traj[1:,1], label = "GT Pos y")
    posy.legend()
    posz.plot(times, particle_state_est[2:,2], label = "Filter Pos z")
    # posz.plot(times, global_state_history_L[:,2], label = "GT Pos z")
    posz.legend()

    # fig, (velx,vely,velz) = plt.subplots(3, 1, figsize=(14, 10))
    # velx.plot(times, particle_state_est[2:,3], label = "Filter Vel x")
    # # velx.plot(times, velocity_GT[:,0], label = "GT Vel x")
    # # velx.set_ylim(-1,2)
    # velx.legend()
    # vely.plot(times, particle_state_est[2:,4], label = "Filter Vel y")    
    # # vely.plot(times, velocity_GT[:,1], label = "GT Vel y")
    # vely.legend()
    # velz.plot(times, particle_state_est[2:,5], label = "Filter Vel z")
    # # velz.plot(times, velocity_GT[:,2], label = "GT Vel z")
    # velz.legend()

    # fig, (posx,velx,accelx) = plt.subplots(3, 1, figsize=(14, 10))
    # posx.plot(times, particle_state_est[2:,0], label = "Filter Accel x")
    # # posx.plot(times, global_state_history_L[:,0], label = "GT Accel x")
    # posx.legend()
    # velx.plot(times, particle_state_est[2:,3], label = "Filter Accel y")
    # # velx.plot(times, velocity_GT[:,0], label = "GT Accel y")
    # velx.legend()
    # accelx.plot(times, particle_state_est[2:,6], label = "Filter Accel z")    
    # # accelx.plot(times, accel_GT[:,0], label = "GT Accel z")
    # accelx.legend()

    plt.show()









    # # Initialize mcl Position
    # est_states = np.zeros((len(mcl.ref_traj) ,6)) # x y z vx vy vz
    # gt_states  = np.zeros((len(mcl.ref_traj) ,16))
    # iteration_count = np.arange(0,len(mcl.ref_traj) , 1, dtype=int)

    # start_time = time.time()

    # pose_est_history_x = []
    # pose_est_history_y = []
    # pose_est_history_z = []
    # velocity_est_history_x = []
    # velocity_est_history_y =[]
    # velocity_est_history_z = []
    # PF_history_x = []
    # PF_history_y = []
    # PF_history_z = []

    # # cam_init = np.array([[0,0,0,mcl.ref_traj[0][0]],
    # #                      [0,0,0,mcl.ref_traj[0][1]],
    # #                      [0,0,0,mcl.ref_traj[0][2]]])
    
    # # cam_init_pos = cam_init[0:3, 3]
    # # cam_rpy = R.from_matrix(cam_init[0:3, 0:3]).as_euler('xyz')
    # # drone_init = np.array([
    # #     cam_init_pos[0], 0, cam_rpy[0]-np.pi/2, 0, 
    # #     cam_init_pos[1], 0, cam_rpy[1], 0, 
    # #     cam_init_pos[2], 0, cam_rpy[2]+np.pi/2, 0, 
    # # ])

    # # ref_init = np.array([0,1])

    # # state = drone_init 
    # # ref = ref_init 
    # # traj = np.concatenate(([0], drone_init, drone_init, ref_init)).reshape((1,-1))
    

    # # Assume constant time step between trajectory stepping
    # time_step = 1

    # last_pos = [0,0,0]
    # print(simple_traj.shape)
    # for iter in range(1,100):
        
    #     state_est, oldparticlepos = mcl.rgb_run(current_pose= simple_traj[iter], )   
    #     pose_est_history_x.append(state_est[0])
    #     pose_est_history_y.append(state_est[1])
    #     pose_est_history_z.append(state_est[2])
    #     velocity_est_history_x.append(state_est[3])
    #     velocity_est_history_y.append(state_est[4])
    #     velocity_est_history_z.append(state_est[5])

    #     PF_history_x.append(np.array(mcl.filter.particles['position'][:,0]).flatten())
    #     PF_history_y.append(np.array(mcl.filter.particles['position'][:,1]).flatten())
    #     PF_history_z.append(np.array(mcl.filter.particles['position'][:,2]).flatten())
    
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

    # fig = plt.figure(1)
    # ax = fig.add_subplot(111, projection='3d')
    # # ax.plot(x, y, z, color='b')
    # t = np.linspace(0, 32, 1000)
    # x = mcl.ref_traj[:,0]
    # y = mcl.ref_traj[:,1]
    # z = mcl.ref_traj[:,2]
    # plt.figure(1)
    # ax.plot(simple_traj[:,0],simple_traj[:,1],simple_traj[:,2], color = 'b')
    # ax.plot(pose_est_history_x,pose_est_history_y,pose_est_history_z, color = 'g')
    # plt.show()

    # # SIM_TIME = 40.0 
    # # DT = SIM_TIME/len(pose_est_history_x)  # time tick [s]
    # # print("DT is ",DT)
    # # time = 0.0
    # # show_animation = False
    # # count = 0

    # # # Initialize a 3D plot
    # # fig = plt.figure()
    # # ax = fig.add_subplot(111, projection='3d')

    # # # Simulation loop
    # # while SIM_TIME >= time:
    # #     time += DT
        

    # #     if show_animation:
    # #         ax.cla()  # Clear the current axis

    # #         # For stopping simulation with the esc key.
    # #         fig.canvas.mpl_connect('key_release_event',
    # #                             lambda event: [exit(0) if event.key == 'escape' else None])

    # #         # Plot the trajectory up to the current count in 3D
    # #         ax.plot(mcl.ref_traj[:count, 0], mcl.ref_traj[:count, 1], mcl.ref_traj[:count, 2], "*k")
    # #         ax.plot(pose_est_history_x[count], pose_est_history_y[count], pose_est_history_z[count], "*r" )
    # #         # ax.plot(PF_history_x[count],PF_history_y[count],PF_history_y[count], 'o',color='blue', alpha=0.5)
    # #         # Additional plotting commands can be added here
            
    # #         ax.set_xlabel('X')
    # #         ax.set_ylabel('Y')
    # #         ax.set_zlabel('Z')
    # #         ax.set_xlim(-40, 40)  # Set X-axis limits
    # #         ax.set_ylim(-40, 40)  # Set Y-axis limits
    # #         ax.set_zlim(-40, 40)  # Set Z-axis limits

    # #         ax.axis("equal")
    # #         ax.set_title('3D Trajectory Animation')
    # #         plt.grid(True)
    # #         plt.pause(0.001)
    # #     count += 1  # Increment count to update the trajectory being plotted

    # # # Show the final plot after the simulation ends
    # # plt.show()


    # print("FINISHED CODE")