from functions import *
from movements import *

from math import sqrt, sin, cos, pi
import numpy as np
import control as ct
from control.matlab import *
import matplotlib.pyplot as plt
from scipy.signal import square

# Simulation parameters
Ts = 0.01
time = 10
T = np.arange(0, time, Ts)

# ==========================
# 1) Construct the discrete linear system
# ==========================
Ts = 0.01

# Define the nonlinear system
io_sys = ct.nlsys(
    updfnc_omniwheel, outfnc_omniwheel, inputs=('u1', 'u2', 'u3'),
    outputs=('x', 'y', 'phi', 'dx', 'dy', 'dphi'),
    name='omniwheel', states=('x', 'y', 'phi', 'dx', 'dy', 'dphi')
)

# Find the equilibrium point
X0 = [0, 0, 0, 0, 0, 0]
eqpt = ct.find_eqpt(io_sys, X0, 0)
xeq = eqpt[0]

# Linearize the system at the equilibrium point
lin_omni = ct.linearize(io_sys, xeq, 0)
sys_z = ct.c2d(lin_omni, Ts, method='zoh')  # Convert to discrete-time system
A_z, B_z, C_z, D_z = sys_z.A, sys_z.B, sys_z.C, sys_z.D

sys_lin_z = ct.StateSpace(A_z, B_z, C_z, D_z, Ts)

# Define the initial state in the linearized space
# It must be a vector with as many dimensions as states in A_z
X0 = [0, 0, 0, 0, 0, 0]

# PID parameters for each wheel
# [21.1, 20.4, 21.1]  [18.1, 15.9, 18.1]  [0.03, 0.02, 0.03]
Kp = np.array([30, 30, 30])
Ki = np.array([0, 0, 0])
Kd = np.array([0, 0, 0])
beta = 0.6  # Filter coefficient for discrete PID

# Desired angular velocity for each wheel (could come from a trajectory generator)
# omega_ref = np.zeros((len(T), 3))
# omega_ref[:, 0] = 5 * (np.mod(T, 2) < 1).astype(float)  # Example square pulse
# omega_ref[:, 1] = 10 * (np.mod(T, 2) < 1).astype(float)
# omega_ref[:, 2] = -20 * (np.mod(T, 2) < 1).astype(float)

# omega_ref, T = generate_omega_ref_trajectory(
#     mov_tuples_list=[
#         (True, 2, 0, 2),
#         (True, 2, -90, 2),
#         (False, 2, 0, 2),
#         (False, 2, -90, 2)

#     ], Ts=0.01
# )

omega_ref = []

time = 10
T = np.arange(0, time, Ts)
n_steps = int(time / Ts)
for t_steps in T:
  x_v, y_v = circ_move(True, 1, 360, 2, t_steps)
  omega_vals = [
    lin_to_ang_vel(x_v, y_v, 1),
    lin_to_ang_vel(x_v, y_v, 2),
    lin_to_ang_vel(x_v, y_v, 3)
  ]
  omega_ref.append(omega_vals)
omega_ref = np.array(omega_ref)

# Initialize
U_pid = np.zeros((len(T), 3))
e = np.zeros((len(T), 3))  # Error for each wheel
prev_prev_output = np.zeros(3)  # u[k-2] for discrete PID

# Compute discrete PID coefficients
b0 = (Kp * (1 + beta*Ts)) + (Ki * Ts * (1 + beta*Ts)) + (Kd * beta)
b1 = (-Kp * (2 + beta*Ts)) + (Ki * Ts) + (Kd * (2 * beta))
b2 = Kp + (Kd * beta)
a0 = 1 + beta*Ts
a1 = -(2 + beta*Ts)
a2 = 1

# Extract system matrices
A, B, C, D = A_z, B_z, C_z, D_z
x = np.zeros((6,))  # initial state
y_log = np.zeros((len(T), 6))
x_log = np.zeros((len(T), 6))

# Initialize data containers before the loop
X_samples = []
Y_samples = []

# Control loop simulation
for k in range(2, len(T)):
    # Measure: get current wheel velocities (dx, dy, dphi can be converted)
    vel_real = x[3:6]

    # Compute error
    e[k] = omega_ref[k] - vel_real

    # Discrete PID law (from pid_calc_discrete in pid_ext.c)
    # output = (b0/a0)*error + (b1/a0)*error[k-1] + (b2/a0)*error[k-2]
    #          - (a1/a0)*output[k-1] - (a2/a0)*output[k-2]
    U_pid[k] = ((b0/a0)*e[k] + 
                (b1/a0)*e[k-1] + 
                (b2/a0)*e[k-2] - 
                (a1/a0)*U_pid[k-1] - 
                (a2/a0)*prev_prev_output)
    
    # Update previous output for next iteration
    prev_prev_output = U_pid[k-1].copy()

    # Apply input to discrete system: x[k+1] = A*x[k] + B*U[k]
    x = A @ x + B @ U_pid[k]
    y = C @ x + D @ U_pid[k]

    # Log data
    x_log[k] = x
    y_log[k] = y

    # ------------------------------------------
    # Collect training sample for this timestep
    # ------------------------------------------
    # Input features: [s[k], r[k], e[k], e[k-1], e[k-2]]
    # Here: s[k] = current measured velocity (can flatten or choose one axis)
    # r[k] = reference velocity
    s_k = vel_real.flatten()
    r_k = omega_ref[k].flatten()
    e_k = e[k].flatten()
    e_k1 = e[k-1].flatten()
    e_k2 = e[k-2].flatten()

    # Flatten all to 1D and concatenate (for multiwheel systems)
    X_k = np.concatenate([s_k, r_k, e_k, e_k1, e_k2])

    # Target output: [Kp, Ki, Kd] (same for all wheels)
    Y_k = np.array([Kp, Ki, Kd])

    X_samples.append(X_k)
    Y_samples.append(Y_k)

# Convert to numpy arrays after loop
X_samples = np.array(X_samples)
Y_samples = np.array(Y_samples)

# -------------------------------
# After loop: reuse U_pid
# -------------------------------
U = U_pid  # shape: (N, num_inputs)

t_out, y_out = ct.input_output_response(sys_lin_z, T, U.T, X0)

x_pos = y_out[0]
y_pos = y_out[1]

x_pos_world = x_pos * np.cos((-1.5*np.pi/6)+np.pi) - y_pos * np.sin((-1.5*np.pi/6)+np.pi)
y_pos_world = x_pos * np.sin((-1.5*np.pi/6)+np.pi) + y_pos * np.cos((-1.5*np.pi/6)+np.pi)

# np.savez(f'/content/drive/MyDrive/pid_autotuning/data/train/run_n16{Ts}{Kp}{Ki}{Kd}_4.npz', X=X_samples, y=Y_samples)
# np.savez(f'/content/drive/MyDrive/pid_autotuning/data/to_predict/run_{Ts}{Kp}{Ki}{Kd}.npz', X=X_samples, y=Y_samples)

fig, axs = plt.subplots(2, 1, figsize=(15, 20))

ax = axs[1]  # Reference to current subplot

# Plot results
ax.plot(T, omega_ref[:,0], '--', label='ω_ref1')
ax.plot(T, y_log[:,3], label='ω1 actual')
ax.plot(T, omega_ref[:,1], '--', label='ω_ref2')
ax.plot(T, y_log[:,4], label='ω2 actual')
ax.plot(T, omega_ref[:,2], '--', label='ω_ref3')
ax.plot(T, y_log[:,5], label='ω3 actual')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Wheel Velocities')
ax.legend()
ax.set_title('Incremental PID Velocity Tracking per Wheel')
ax.grid()

# 4.2) Plot position outputs (trajectory)
ax = axs[0]  # Reference to current subplot
ax.plot(x_pos_world, y_pos_world, marker="o", label="Robot Path")

# Ensure equal axis scaling for better spatial interpretation
ax.set_aspect('equal', adjustable='datalim')
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_title("Robot Position and Orientation")
ax.legend()
ax.grid()

plt.tight_layout()
plt.show()