#!/usr/bin/env python3
import rospy
import sys
import logging
import time
from std_msgs.msg import Float32, String, Bool,Float64MultiArray
import geometry_msgs.msg as geometry_msgs
import agiros_msgs.msg as agiros_msgs
import numpy as np
from geometry_msgs.msg import TwistStamped, Twist, Vector3, Point, Pose, PointStamped, PoseArray, Quaternion
import numpy.linalg as la
import pickle
import time
import signal
import numpy.linalg as la
from pyquaternion import Quaternion
import casadi as ca 

target_pose_p = None
new_message_received = False
prediction_on = False
prediction_count = 40
data_storage = []
state_est = None
# vicon_pose = None
accumulated_backoff = 0
current_mode = 1
mode_count = 0
# did_prediction = False

# MPC Constants
m = 1
I_x = I_y = I_z = 0.01
# I_x = 0.0025
# I_y = 0.0021
# I_z = 0.0043
g = 9.81

class MPC_Controller:
    def __init__(
            self,
            # Time parameters
            dt = 0.05,                                                   # Time step
            N = 20,                                                     # Prediction horizon   
            # Weighting Metrices
            Q = np.diag([100, 100, 100, 0, 0, 5, 0, 0, 0, 0, 0, 0]),    # State error weights
            R = np.diag([1, 1, 1, 1]),                                  # Control input weights
            # Control input constraints
            ft_min = 0,                                                 # Thrust cannot be negative
            ft_max = 10 * m * g,                                        # Maximum thrust (adjust based on motor specs)

            tau_min = np.array([-20, -20, -20]),                        # Min torques
            tau_max = np.array([20, 20, 20]),                           # Max torques
    ):
        
        # Initializing MPC Parameters 
        self.dt = dt 
        self.N = N 
        
        self.Q = Q 
        self.R = R

        self.ft_min = ft_min 
        self.ft_max = ft_max 

        self.tau_min = tau_min 
        self.tau_max = tau_max 

        self.u_min = np.concatenate(([ft_min], tau_min))
        self.u_max = np.concatenate(([ft_max], tau_max))

        self.nx = 12
        self.nu = 4

        # Initialize Ros
        rospy.init_node('controller_node', anonymous=True)
        
        self.ctrl_dt = dt
        self.rate = rospy.Rate(int(1/dt))  # Hz

        self.pub = rospy.Publisher(
            '/kingfisher/agiros_pilot/feedthrough_command',
            agiros_msgs.Command,
            queue_size=1
        )
        
        self.pub_log = rospy.Publisher('/log_messages_ctrl', Float64MultiArray, queue_size=10)
        self.log_data = Float64MultiArray()
        self.log_data.data = [0, ] * (2 + 3 + 3 + 3 + 2 +1)


        rospy.Subscriber('/fake_waypoint_list', PoseArray, self.callback_ref, queue_size=1)

        self.waypoints_list = np.array([])
        self.accumulated_dist_list = np.array([])
        self.vicon_pose = None
        self.target_pose = None
        self.new_message_received = False
        # rospy.Subscriber('/leader_waypoint',PointStamped, self.callback,queue_size=1)

        self.current_pose = None 

        sub_startpoint = rospy.Subscriber("/kingfisher/agiros_pilot/state", agiros_msgs.QuadState, self.callback_state, queue_size=1)
        sub_vicon = rospy.Subscriber("/vicon_estimate", Float64MultiArray, self.callback_vicon, queue_size=1)

        signal.signal(signal.SIGINT, self.signal_handler)

        while self.waypoints_list.size<=0 or self.accumulated_dist_list.size<=0 or self.current_pose is None or self.vicon_pose is None :
            print("Waiting for reference trajectory", self.waypoints_list.size, self.accumulated_dist_list.size, self.current_pose, self.vicon_pose)
            rospy.sleep(0.1) 

        self.initialize_MPC()

    def initialize_MPC(self):
        print("Initializing MPC Solver")
        self.opti = ca.Opti()
        # Decision variables for states and controls
        self.X = self.opti.variable(self.nx, self.N+1)   # States over the horizon
        self.U = self.opti.variable(self.nu, self.N)     # Controls over the horizon

        # Parameters (initial state and reference trajectory)
        self.X0 = self.opti.parameter(self.nx)      # Initial state
        self.XR = self.opti.parameter(self.nx, self.N+1) # Reference states
        
        print("Initializing MPC Objective Function")
        # Objective function
        self.objective = 0

        for k in range(self.N):
            # State error
            e = self.X[:, k] - self.XR[:, k]
            self.objective += ca.mtimes([e.T, self.Q, e])
            
            # Control effort
            du = self.U[:, k]
            self.objective += ca.mtimes([du.T, self.R, du])

        # Terminal cost (optional)
        e_terminal = self.X[:, self.N] - self.XR[:, self.N]
        self.objective += ca.mtimes([e_terminal.T, self.Q, e_terminal])

        print("Initiallizing MPC Constraints")
        # Initial condition constraint
        self.opti.subject_to(self.X[:, 0] == self.X0)

        for k in range(self.N):
            # Dynamics constraints
            x_next = self.quadrotor_dynamics(self.X[:, k], self.U[:, k], self.dt)
            self.opti.subject_to(self.X[:, k+1] == x_next)
            
            # Control constraints
            self.opti.subject_to(self.u_min <= self.U[:, k])
            self.opti.subject_to(self.U[:, k] <= self.u_max)

        # Set the objective
        self.opti.minimize(self.objective)

        # Solver options
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        self.opti.solver('ipopt', opts)

        print("MPC Initialized")

    def signal_handler(self, sig, frame):
        rospy.loginfo("Ctrl+C pressed. Exiting...")
        rospy.signal_shutdown("Ctrl+C pressed")
        sys.exit(0)

    def callback_ref(self, data:PoseArray):
        if self.waypoints_list.size>0:
            print("rejecting repeated waypoints list")
            return
        print("message received")
        posearray = data.poses
        print("Length: {}".format(len(posearray))) 
        waypoints_list_tmp = []
        accumulated_dist_list_tmp = []
        for i in range(len(posearray)):
            pose = posearray[i]
            pos = pose.position 
            ori = pose.orientation

            x,y,z = pos.x, pos.y, pos.z 
            qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w 
            quad = Quaternion(qw, qx, qy, qz)
            psi, theta, phi = quad.yaw_pitch_roll

            if i==0:
                vx=vy=vz=0
                p=q=r=0
            else:
                vx = (x-waypoints_list_tmp[-1][0])/self.dt
                vy = (y-waypoints_list_tmp[-1][1])/self.dt
                vz = (z-waypoints_list_tmp[-1][2])/self.dt

                p = (phi-waypoints_list_tmp[-1][3])/self.dt
                q = (theta-waypoints_list_tmp[-1][4])/self.dt
                r = (psi-waypoints_list_tmp[-1][5])/self.dt

            waypoints_list_tmp.append([
                x, y, z,
                phi, theta, psi,
                vx, vy, vz,
                p, q, r
            ])

            if i==0:
                accumulated_dist_list_tmp.append(0)
            else:
                prev_x, prev_y, prev_z = posearray[i-1].position.x,posearray[i-1].position.y,posearray[i-1].position.z
                accumulated_dist_list_tmp.append(
                    accumulated_dist_list_tmp[-1]
                    + np.linalg.norm([x-prev_x, y-prev_y, z-prev_z]))

        waypoints_list = np.array(waypoints_list_tmp)
        dist = np.linalg.norm(self.current_pose-waypoints_list[0,:])
        # if dist>0.5:
        #     print("Starting point far away from current pose, Reject")
        #     return 
        self.waypoints_list = waypoints_list
        self.accumulated_dist_list = np.array(accumulated_dist_list_tmp)
        self.new_message_received = True    
        np.savez('waypoints.npz', waypoints_list = self.waypoints_list, accumulated_dist_list = self.accumulated_dist_list)
    
    def callback_state(self, data):
        # print(data)
        x, y, z = data.pose.position.x, data.pose.position.y, data.pose.position.z   
        qx, qy, qz, qw = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
        quat = Quaternion(qw, qx, qy, qz)
        psi, theta, phi = quat.yaw_pitch_roll
        
        vx, vy, vz = data.velocity.linear.x, data.velocity.linear.y, data.velocity.linear.z
        p, q, r = data.velocity.angular.x, data.velocity.angular.y, data.velocity.angular.z
        # yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        # print("Current pose: {}, {}, {}, {}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z, yaw))
        self.current_pose = np.array([
            x,y,z,
            phi, theta, psi,
            vx, vy, vz, 
            p, q, r
        ])
    
    def callback_vicon(self, data):
        self.vicon_pose = np.array([data.data[0], data.data[1], data.data[2]])

    def quadrotor_dynamics(self, state, u, dt):
        """
        Simplified 2D quadrotor dynamics.
        x: state vector [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
        u: control input [ax, ay]
        dt: time step
        """
        x = state[0]
        y =state[1]
        z =state[2]
        phi =state[3]
        theta =state[4]
        psi =state[5]
        vx =state[6]
        vy =state[7]
        vz =state[8]
        p =state[9]
        q =state[10]
        r = state[11] 
        ft = u[0] 
        taux = u[1] 
        tauy = u[2] 
        tauz = u[3]

        t1 = vx                                                                             # x
        t2 = vy                                                                             # y
        t3 = vz                                                                             # z
        t4 = p+q*ca.sin(phi)*ca.tan(theta)+r*ca.cos(phi)*ca.tan(theta)                      # phi (ox)
        t5 = q*ca.cos(phi)-r*ca.sin(phi)                                                    # theta (oy)
        t6 = q*ca.sin(phi)/ca.cos(theta)+r*ca.cos(phi)/ca.cos(theta)                        # psi (oz)
        t7 = 0+(1/m)*(ca.sin(phi)*ca.sin(psi)+ca.cos(phi)*ca.cos(psi)*ca.sin(theta))*ft    # vx
        t8 = 0+(1/m)*(ca.cos(psi)*ca.sin(phi)-ca.cos(phi)*ca.sin(psi)*ca.sin(theta))*ft    # vy
        t9 = -g+(1/m)*(ca.cos(phi)*ca.cos(theta))*ft                                        # vz
        t10 = (I_y-I_z)/I_x*q*r+1/I_x*taux                                                  # p (phi_dot)
        t11 = (I_z-I_x)/I_y*p*r+1/I_y*tauy                                                  # q (theta_dot)
        t12 = (I_x-I_y)/I_z*p*q+1/I_z*tauz                                                  # r (psi_dot)
        
        state_next = ca.vertcat(
            x+t1*dt,
            y+t2*dt,
            z+t3*dt,
            phi+t4*dt,
            theta+t5*dt,
            psi+t6*dt, 
            vx+t7*dt, 
            vy+t8*dt, 
            vz+t9*dt,
            p+t10*dt,
            q+t11*dt,
            r+t12*dt,
        )
        
        return state_next

    def MPC_loop(self,):
        prev_yaw_error = None
        # rospy.loginfo("Starting velPID ctrl, current state: {}, target pos: {}".format(self.current_pose, active_target_pose))
        for i in range(self.waypoints_list.shape[0]):
            starttime = time.time()
            x0 = self.current_pose
            self.opti.set_value(self.X0,x0)

            # Extract reference trajectory for current horizon
            idx_start = i
            idx_end = i + self.N + 1
            if idx_end > len(self.waypoints_list):
                idx_end = len(self.waypoints_list)
                idx_start = idx_end - self.N - 1
            x_ref_horizon = self.waypoints_list[idx_start:idx_end, :]

            # Pad reference if at the end
            if x_ref_horizon.shape[0] < self.N + 1:
                last_ref = x_ref_horizon[-1, :]
                pad_size = self.N + 1 - x_ref_horizon.shape[0]
                x_ref_horizon = np.vstack((x_ref_horizon, np.tile(last_ref, (pad_size, 1))))

            # Set reference trajectory parameter
            # Full state x_ref
            self.opti.set_value(self.XR, x_ref_horizon.T)   

            # # For simplicity, set reference velocities to zero
            # xr = np.zeros((self.nx, self.N+1))
            # xr[0:6, :] = x_ref_horizon[:, 0:6]
            # self.opti.set_value(self.XR, xr.T)
            print(">>>>>>>>>>>>>>")
            print("x0: ",x0)
            print("xref: ", x_ref_horizon[0,:])

            # Solve the optimization problem
            try:
                sol = self.opti.solve()
            except RuntimeError as e:
                print(e)
                print(f"Solver failed at time step {i}")
                break

            # Extract the optimal control
            u_opt = sol.value(self.U[:, 0])

            # Publish optimal control 
            is_single_rotor_thrust = False 
            collective_thrust = u_opt[0]
            bodyrates = Vector3(x=u_opt[1],y=u_opt[2],z=u_opt[3])

            command = agiros_msgs.Command(
                is_single_rotor_thrust = is_single_rotor_thrust,
                collective_thrust = collective_thrust,
                bodyrates = bodyrates
            )

            self.pub.publish(command)
            
            print("x_inferred: ", sol.value(self.X[:,0]))

            if self.new_message_received:
                break

            # Warm start
            self.opti.set_initial(self.X, sol.value(self.X))
            self.opti.set_initial(self.U, sol.value(self.U))

            # ROS Logging 
            self.log_data.data[0] = int(prediction_on)
            self.log_data.data[1] = int(new_message_received)
            self.log_data.data[2] = self.current_pose[0]
            self.log_data.data[3] = self.current_pose[1]
            self.log_data.data[4] = self.current_pose[2]
            # self.log_data.data[5] = active_target_pose[0]
            # self.log_data.data[6] = active_target_pose[1]
            # self.log_data.data[7] = active_target_pose[2]
            self.log_data.data[8] = self.vicon_pose[0]
            self.log_data.data[9] = self.vicon_pose[1]
            self.log_data.data[10] = self.vicon_pose[2]
            # self.log_data.data[11] = target_idx 
            # self.log_data.data[12] = look_ahead 
            now = rospy.get_rostime()
            now = now.to_sec()
            self.log_data.data[-1] = now
            self.pub_log.publish(self.log_data)
            self.rate.sleep()  
   
    def main(self):
        
        while not rospy.is_shutdown():
            #Add prediciotn siwth here

 
            self.new_message_received = False

            # Inner loop runs until a new message is received
            self.MPC_loop()

            # ROS Logging 
            self.log_data.data[0] = int(prediction_on)
            self.log_data.data[1] = int(new_message_received)
            self.log_data.data[2] = self.current_pose[0]
            self.log_data.data[3] = self.current_pose[1]
            self.log_data.data[4] = self.current_pose[2]
            self.log_data.data[5] = 0
            self.log_data.data[6] = 0
            self.log_data.data[7] = 0
            self.log_data.data[8] = self.vicon_pose[0]
            self.log_data.data[9] = self.vicon_pose[1]
            self.log_data.data[10] = self.vicon_pose[2]
            now = rospy.get_rostime()
            now = now.to_sec()
            self.log_data.data[-1] = now
            self.pub_log.publish(self.log_data)

            print("Exiting MPC loop")
            break

  
    def _reached_target_position(self, current_position, target_position):
        position_tolerance = 0.005
        return la.norm(current_position - target_position) < position_tolerance

    def _limitVelocity(self, velocity):
        global current_mode, mode_count
        # current_mode = 1
        if current_mode < 4:
            max_velocity_l = 1.0
            mode_count = 0
        else:
            # exp_vel = np.exp(0.01 * mode_count) - 0.8
            exp_vel = 1/100*mode_count+0.05 
            print("EXP", exp_vel)
            max_velocity_l = min(1.0, exp_vel)
            # max_velocity_l = 0.2
            mode_count += 1

        
        # exp_vel = np.exp(0.01 * mode_count) - 0.8
        # max_velocity_l = min(1.0, exp_vel)    
        # # max_velocity_l = 0.2
        max_velocity = 1.0

        velocity[0] = np.clip(velocity[0], -max_velocity_l, max_velocity_l)
        velocity[1] = np.clip(velocity[1], -max_velocity, max_velocity)
        velocity[2] = np.clip(velocity[2], -max_velocity, max_velocity)

        # if current_mode == 4:          
        print("velocity: ", velocity)

        return velocity
    
    def _limitPos(self, velocity):
        # global vicon_pose

        # max_x = 3.0
        # min_x = -2.6
        # max_y = 1.8
        # min_y = -3.6
        # max_z = 3.5
        # (4.2, 3.9)
        # (-2.5, 3.9)
        # (-2.5,-3.5)
        # (4.2, -3.5)
        # max_y = 3.6
        # min_y = -3.2
        # max_x = 3.9
        # min_x = -2.2
        # max_z = 2.2
        # min_z = 0.3

        max_y = 3.2
        min_y = -2.4
        max_x = 3.1
        min_x = -2.0
        max_z = 3.0
        min_z = 0.1

        if self.vicon_pose[0]>=max_x or self.vicon_pose[0]<=min_x:
            print("LIMIT REACHED X: {}".format(self.vicon_pose[0]))
            velocity[0] =0.0
        if self.vicon_pose[1]>=max_y or self.vicon_pose[1]<=min_y:
            print("LIMIT REACHED Y: {}".format(self.vicon_pose[1]))
            velocity[1] = 0.0
        if self.vicon_pose[2]>=max_z or self.vicon_pose[2]<=min_z:
            print("LIMIT REACHED Z: {}".format(self.vicon_pose[2]))
            velocity[2] = 0.0

        if self.vicon_pose[2]>=2.7:
            print(f"DANGEROUS HEIGHT Z: {self.vicon_pose[2]}; STOPPING")
            velocity[0] = 0
            velocity[1] = 0
            velocity[2] = 0
        
        return velocity



if __name__ == '__main__':
    ctrl = MPC_Controller()
    try:
        ctrl.main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught")

    
    # save_file = "data_aruco.pkl"

    # with open(save_file ,'wb') as file:
    #     pickle.dump(data_storage,file)

    # print("finished controller log, saved to {}".format(save_file))

