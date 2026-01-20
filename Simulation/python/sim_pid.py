from functions import *
from movements import *

from math import sqrt, sin, cos, pi
import numpy as np
import control as ct
from control.matlab import *
import matplotlib.pyplot as plt
from scipy.signal import square
import pandas as pd

# Simulation parameters
Ts = 0.01

omega_ref = []

omega_ref, T = generate_omega_ref_trajectory(
    mov_tuples_list=[
       #(type    ,fw/cw,  lv, ang, r, t)
        ('linear', True,  12,  90, 0, 10),
        ('linear', True,  10,   0, 0, 5),
        ('linear', False, 20,   0, 0, 7),
        ('circular', True, 5, 360, 20, 28)
    ], Ts=Ts
)

# ==========================
# 1) Construct the discrete linear system
# ==========================
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
# Kp = np.array([21.1, 20.4, 21.1])
# Ki = np.array([18.1, 15.9, 18.1])
# Kd = np.array([0.03, 0.02, 0.03])

Kp = np.array([2.5, 2.3, 2.6])
Ki = np.array([0.002, 0.001, 0.002])
Kd = np.array([0.003, 0.002, 0.003])

# Initialize
U_pid = np.zeros((len(T), 3))
e = np.zeros((len(T), 3))  # Error for each wheel

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

    # Incremental PID law
    dU = (Kp * 8.3  * (e[k] - e[k-1]) +
          Ki * 8500 * e[k] +
          Kd * (e[k] - 2*e[k-1] + e[k-2]))

    # Update control signal
    U_pid[k] = U_pid[k-1] + dU

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

# Save X_samples to CSV with appropriate headers
# import pandas as pd
# header = [
#     'motor_state_0', 'motor_state_1', 'motor_state_2',
#     'motor_setpoint_0', 'motor_setpoint_1', 'motor_setpoint_2',
#     'error_0_k', 'error_0_k1', 'error_0_k2',
#     'error_1_k', 'error_1_k1', 'error_1_k2',
#     'error_2_k', 'error_2_k1', 'error_2_k2'
# ]
# df_X_samples = pd.DataFrame(X_samples, columns=header)
# csv_filename = f'X_samples_Kp{Kp[0]}_Ki{Ki[0]}_Kd{Kd[0]}.csv'
# df_X_samples.to_csv(csv_filename, index=False)
# print(f"X_samples saved to: {csv_filename}")

# -------------------------------
# After loop: reuse U_pid
# -------------------------------
U = U_pid  # shape: (N, num_inputs)

t_out, y_out = ct.input_output_response(sys_lin_z, T, U.T, X0)

x_pos = y_out[0]
y_pos = y_out[1]

angle = np.pi/4 - np.pi/2
x_pos_world = (x_pos * np.cos(angle) - y_pos * np.sin(angle))
y_pos_world = (x_pos * np.sin(angle) + y_pos * np.cos(angle))

# np.savez(f'/content/drive/MyDrive/pid_autotuning/data/train/run_n16{Ts}{Kp}{Ki}{Kd}_4.npz', X=X_samples, y=Y_samples)
# np.savez(f'/content/drive/MyDrive/pid_autotuning/data/to_predict/run_{Ts}{Kp}{Ki}{Kd}.npz', X=X_samples, y=Y_samples)

# fig, axs = plt.subplots(2, 1, figsize=(15, 20))

# ax = axs[1]  # Reference to current subplot

# Plot results
plt.plot(T, omega_ref[:,0], '--', label='ω_ref1')
plt.plot(T, y_log[:,3], label='ω1 actual')
plt.plot(T, omega_ref[:,1], '--', label='ω_ref2')
plt.plot(T, y_log[:,4], label='ω2 actual')
plt.plot(T, omega_ref[:,2], '--', label='ω_ref3')
plt.plot(T, y_log[:,5], label='ω3 actual')
plt.xlabel('Time [s]')
plt.ylabel('Wheel Velocities')
plt.legend()
plt.title('PID Velocity Tracking per Wheel')
plt.grid()

plt.show()

# 4.2) Plot position outputs (trajectory)
plt.plot(x_pos_world, y_pos_world, marker="o", label="Robot Path")

# Ensure equal axis scaling for better spatial interpretation
plt.gca().set_aspect('equal', adjustable='box')
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Robot Position and Orientation")
plt.legend()
plt.grid()

plt.show()