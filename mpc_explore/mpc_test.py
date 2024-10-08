import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint 
import time 
m = 1
I_x = I_y = I_z = 1
g = 9.81

def quadrotor_dynamics_cont(state, t, ft, taux, tauy, tauz):
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
    # ft = u[0] 
    # taux = u[1] 
    # tauy = u[2] 
    # tauz = u[3]

    # state_next = ca.vertcat(
    #     x+vx*dt,
    #     y+vy*dt,
    #     z+vz*dt,
    #     phi+q*np.sin(psi)/np.cos(theta)+r*np.cos(psi)/np.cos(theta)*dt,
    #     theta+,
    #     psi+, 
    #     vx+, 
    #     vy+, 
    #     vz+,
    #     p+,
    #     q+,
    #     r+,
    # )
    t1 = vx                                                                             # x
    t2 = vy                                                                             # y
    t3 = vz                                                                             # z
    t4 = p+q*ca.sin(phi)*ca.tan(theta)+r*ca.cos(phi)*ca.tan(theta)                      # phi (ox)
    t5 = q*ca.cos(phi)-r*ca.sin(phi)                                                    # theta (oy)
    t6 = q*ca.sin(phi)/ca.cos(theta)+r*ca.cos(phi)/ca.cos(theta)                        # psi (oz)
    t7 = 0+(1/m)*(ca.sin(phi)*ca.sin(psi)+ca.cos(phi)*ca.cos(psi)*ca.sin(theta))*ft     # vx
    t8 = 0+(1/m)*(ca.cos(psi)*ca.sin(phi)-ca.cos(phi)*ca.sin(psi)*ca.sin(theta))*ft     # vy
    t9 = -g+(1/m)*(ca.cos(phi)*ca.cos(theta))*ft                                        # vz
    t10 = (I_y-I_z)/I_x*q*r+1/I_x*taux                                                  # p (phi_dot)
    t11 = (I_z-I_x)/I_y*p*r+1/I_y*tauy                                                  # q (theta_dot)
    t12 = (I_x-I_y)/I_z*p*q+1/I_z*tauz                                                  # r (psi_dot)
    
    return [t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12]

def quadrotor_dynamics(state, u, dt):
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

    # state_next = ca.vertcat(
    #     x+vx*dt,
    #     y+vy*dt,
    #     z+vz*dt,
    #     phi+q*np.sin(psi)/np.cos(theta)+r*np.cos(psi)/np.cos(theta)*dt,
    #     theta+,
    #     psi+, 
    #     vx+, 
    #     vy+, 
    #     vz+,
    #     p+,
    #     q+,
    #     r+,
    # )
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

# Time parameters
dt = 0.1            # Time step
N = 10              # Prediction horizon

# Simulation parameters
sim_time = 200  # Total simulation time
num_steps = int(sim_time / dt)

# Weighting matrices
Q = np.diag([100, 100, 100, 0, 0, 5, 1, 1, 1, 0, 0, 1])    # State error weights
R = np.diag([1, 1, 1, 1])            # Control input weights

# Control input constraints
ft_min = 0          # Thrust cannot be negative
ft_max = 10 * m * g  # Maximum thrust (adjust based on motor specs)

tau_min = np.array([-20, -20, -20])  # Min torques
tau_max = np.array([20, 20, 20])     # Max torques

u_min = np.concatenate(([ft_min], tau_min))
u_max = np.concatenate(([ft_max], tau_max))

# Number of states and controls
nx = 12
nu = 4

from scipy.interpolate import UnivariateSpline
# Example waypoints
waypoints = np.array([
    [0, 0, 0],
    [2, 2, 0],
    [4, 0, 0],
    [6, 2, 0],
    [8, 0, 0],
    [6, 2, 0],
    [4, 0, 0],
])

t_array = [0]
for i in range(1, waypoints.shape[0]):
    t_array.append(t_array[-1]+np.linalg.norm(waypoints[i,:]-waypoints[i-1,:]))

t_array = np.array(t_array)/t_array[-1]

traj_x = UnivariateSpline(t_array, waypoints[:,0], k=3, s=0)
traj_y = UnivariateSpline(t_array, waypoints[:,1], k=3, s=0)
traj_z = UnivariateSpline(t_array, waypoints[:,2], k=3, s=0)

ts = np.linspace(0, 1, num_steps)

x_ref = []
y_ref = []
z_ref = []
phi_ref = []
theta_ref = []
psi_ref = []
vx_ref = []
vy_ref = []
vz_ref = []
p_ref = []
q_ref = []
r_ref = []
for i in range(len(ts)):
    t = ts[i]
    x = traj_x(t)
    y = traj_y(t)
    z = traj_z(t)
    phi = 0
    theta = 0
    if i==0:
        psi = np.pi/4
    else:
        prev_t = ts[i-1 ]
        psi = np.arctan2(y-traj_y(prev_t), y-traj_x(prev_t) )
    if i==0:
        vx, vy, vz = 0, 0, 0
    else:
        vx = (x-x_ref[-1])/dt 
        vy = (y-y_ref[-1])/dt 
        vz = (z-z_ref[-1])/dt 
    if i==0:
        p, q, r = 0, 0, 0
    else:
        p = 0
        q = 0
        r = (psi-psi_ref[-1])/dt 

    x_ref.append(x)
    y_ref.append(y)
    z_ref.append(z)
    vx_ref.append(vx)
    vy_ref.append(vy)
    vz_ref.append(vz)
    phi_ref.append(phi)
    theta_ref.append(theta)
    psi_ref.append(psi)
    p_ref.append(p)
    q_ref.append(q)
    r_ref.append(r)

ref_traj = np.vstack((x_ref, y_ref, z_ref, vx_ref, vy_ref, vz_ref, phi_ref, theta_ref, psi_ref, p_ref, q_ref, r_ref)).T

# Create optimization variables
opti = ca.Opti()

# Decision variables for states and controls
X = opti.variable(nx, N+1)   # States over the horizon
U = opti.variable(nu, N)     # Controls over the horizon

# Parameters (initial state and reference trajectory)
X0 = opti.parameter(nx)      # Initial state
XR = opti.parameter(nx, N+1) # Reference states

# Objective function
objective = 0

for k in range(N):
    # State error
    e = X[:, k] - XR[:, k]
    objective += ca.mtimes([e.T, Q, e])
    
    # Control effort
    du = U[:, k]
    objective += ca.mtimes([du.T, R, du])

# Terminal cost (optional)
e_terminal = X[:, N] - XR[:, N]
objective += ca.mtimes([e_terminal.T, Q, e_terminal])

# Define constraints
constraints = []

# Initial condition constraint
opti.subject_to(X[:, 0] == X0)

for k in range(N):
    # Dynamics constraints
    x_next = quadrotor_dynamics(X[:, k], U[:, k], dt)
    opti.subject_to(X[:, k+1] == x_next)
    
    # Control constraints
    opti.subject_to(u_min <= U[:, k])
    opti.subject_to(U[:, k] <= u_max)

# Set the objective
opti.minimize(objective)

# Solver options
opts = {'ipopt.print_level': 0, 'print_time': 0}
opti.solver('ipopt', opts)

# Initial state
x0 = np.array([0, 0, 0, 0, 0, np.pi/4, 0, 0, 0, 0, 0, 0])

# Storage for results
state_history = np.zeros((nx, num_steps+1))
state_history[:, 0] = x0
control_history = np.zeros((nu, num_steps))

# Start simulation
start_time = time.time()
loop_times = []
for t in range(num_steps):
    # Set initial state parameter
    loop_start_time = time.time()
    opti.set_value(X0, x0)
    
    # Extract reference trajectory for current horizon
    idx_start = t
    idx_end = t + N + 1
    if idx_end > len(ref_traj):
        idx_end = len(ref_traj)
        idx_start = idx_end - N - 1
    x_ref_horizon = ref_traj[idx_start:idx_end, :]
    
    # Pad reference if at the end
    if x_ref_horizon.shape[0] < N + 1:
        last_ref = x_ref_horizon[-1, :]
        pad_size = N + 1 - x_ref_horizon.shape[0]
        x_ref_horizon = np.vstack((x_ref_horizon, np.tile(last_ref, (pad_size, 1))))
    
    # Set reference trajectory parameter
    xr = np.zeros((nx, N+1))
    # xr[0, :] = x_ref_horizon[:, 0]
    # xr[1, :] = x_ref_horizon[:, 1]
    # For simplicity, set reference velocities to zero
    opti.set_value(XR, x_ref_horizon.T)
    
    # Solve the optimization problem
    try:
        sol = opti.solve()
    except RuntimeError:
        print(f"Solver failed at time step {t}")
        break
    
    # Extract the optimal control
    u_opt = sol.value(U[:, 0])
    
    # Apply the control input to the system (simulate one time step)
    x_next_tmp = quadrotor_dynamics(x0, u_opt, dt)
    x0_tmp = x_next_tmp.full().flatten()
    tmp_dt = np.linspace(0,dt,5)
    res = odeint(quadrotor_dynamics_cont, x0, tmp_dt, args=tuple(u_opt))
    x_next = res[-1,:]
    x0 = x_next
    
    # Store the results
    state_history[:, t+1] = x0
    control_history[:, t] = u_opt
    
    # Warm start
    opti.set_initial(X, sol.value(X))
    opti.set_initial(U, sol.value(U))
    loop_times.append(time.time()-loop_start_time)
end_time = time.time()
print(f"Total time: {end_time-start_time}")
print(f"Average time: {(end_time-start_time)/num_steps}")
print(f"Max time: {max(loop_times)}, Min time: {min(loop_times)}")

# Extract positions
x_pos = state_history[0, :]
y_pos = state_history[1, :]

# Plot trajectory
plt.figure(0, figsize=(10, 6))
plt.plot(ref_traj[:, 0], ref_traj[:, 1], 'r--', label='Reference Path')
plt.plot(x_pos, y_pos, 'b-', label='Quadrotor Path')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Quadrotor MPC Trajectory Tracking')
plt.legend()
plt.grid()

plt.figure(1)
plt.plot(ts, ref_traj[:,0])
plt.plot(ts, x_pos[1:])

plt.figure(2)
plt.plot(ts, ref_traj[:,1])
plt.plot(ts, y_pos[1:])

plt.figure(3)
plt.plot(ts, loop_times)
plt.show()