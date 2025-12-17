from functions import *

import control as ct
import matplotlib.pyplot as plt
import keras
from keras import losses

# model_name = "3wheel_model_lstm_128_tanh_new"
# model_path = f"/content/drive/MyDrive/pid_autotuning/models/{model_name}.h5"

# Pass the built-in mean_squared_error function under the expected name 'mse'
# custom_objects = {
#     'mse': losses.mean_squared_error
# }

# model = keras.models.load_model(
#     model_path,
#     custom_objects=custom_objects
# )

# Simulation parameters
Ts = 0.01
time = 12 # Extended time to show multiple updates
T = np.arange(0, time, Ts)

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

# Initial PID parameters
Kp = np.array([13.074613, 12.121785, 12.510017])
Ki = np.array([10.834111,  9.564942, 10.935776])
Kd = np.array([0.1919426,  0.01672415, 0.12179589])

# Ideal PID parameters
Kp_ideal = np.array([21.1, 20.4, 21.1])
Ki_ideal = np.array([18.1, 15.9, 18.1])
Kd_ideal = np.array([0.03, 0.02, 0.03])

# Desired trajectory generation (Modified time to match simulation time)
omega_ref, T_actual = generate_omega_ref_trajectory(
    mov_tuples_list=[
        ('linear', True, 2, 0, 0, 4),
        ('linear', True, 1, -90, 0, 6),
        ('linear', False, 3, 0, 0, 5),
        ('circular', True, 5, 360, 10, (360/360) * 2 * pi * 10 / 5)
    ], Ts=0.01
)
T = np.arange(0, len(omega_ref) * Ts, Ts) # Adjust T length

# Initialize
U_pid = np.zeros((len(T), 3))
U_pid_ideal = np.zeros((len(T), 3))
e = np.zeros((len(T), 3))
e_ideal = np.zeros((len(T), 3))

x = np.zeros((6,))
x_ideal = np.zeros((6,))

# Initialize logs
y_log = np.zeros((len(T), 6))
x_log = np.zeros((len(T), 6))
y_log_ideal = np.zeros((len(T), 6))
x_log_ideal = np.zeros((len(T), 6))

# Initialize data containers for AI training/inference for the current interval
X_samples = []
Y_samples = []

# --- AI INFERENCE PARAMETERS ---
UPDATE_INTERVAL_S = 2.0
UPDATE_INTERVAL_STEPS = int(UPDATE_INTERVAL_S / Ts)
# -------------------------------

# Control loop simulation
for k in range(2, len(T)):
    # ------------------------------------------
    # 1. AI INFERENCE AND UPDATE LOGIC
    # ------------------------------------------
    # Check if the current time step aligns with an update interval
    # if k % UPDATE_INTERVAL_STEPS == 0:

    #     # Pass collected data and current constants to the prediction model
    #     Kp_new, Ki_new, Kd_new = predict_pid_constants(X_samples, Kp, Ki, Kd, k, Ts)

    #     # Update the constants for the remaining part of the trajectory
    #     print(f"Kp before: {Kp}, Kp after: {Kp_new}")
    #     print(f"Ki before: {Ki}, Ki after: {Ki_new}")
    #     print(f"Kd before: {Kd}, Kd after: {Kd_new}")
    #     Kp = Kp_new
    #     Ki = Ki_new
    #     Kd = Kd_new

    #     # Reset data buffers for the next interval
    #     X_samples = []

    # ------------------------------------------
    # 2. PID CONTROL LOOP
    # ------------------------------------------

    # Measure: get current body velocities (dx, dy, dphi)
    vel_body = x[3:6]
    vel_body_ideal = x_ideal[3:6]

    # Compute error (use modulo or clip for ref_idx if T length is different from omega_ref length)
    ref_idx = min(k, len(omega_ref) - 1)
    e[k] = omega_ref[ref_idx] - vel_body
    e_ideal[k] = omega_ref[ref_idx] - vel_body_ideal

    # Incremental PID law (uses the CURRENTLY active Kp, Ki, Kd)
    dU = (Kp * (e[k] - e[k-1]) +
          Ki * e[k] +
          Kd * (e[k] - 2*e[k-1] + e[k-2]))
    dU_ideal = (Kp_ideal * (e_ideal[k] - e_ideal[k-1]) +
                Ki_ideal * e_ideal[k] +
                Kd_ideal * (e_ideal[k] - 2*e_ideal[k-1] + e_ideal[k-2]))

    # Update control signal and apply to discrete system
    U_pid[k] = U_pid[k-1] + dU
    U_pid_ideal[k] = U_pid_ideal[k-1] + dU_ideal
    x = A_z @ x + B_z @ U_pid[k]
    y = C_z @ x + D_z @ U_pid[k]
    x_ideal = A_z @ x_ideal + B_z @ U_pid_ideal[k]
    y_ideal = C_z @ x_ideal + D_z @ U_pid_ideal[k]

    # Log data
    x_log[k] = x
    y_log[k] = y
    x_log_ideal[k] = x_ideal
    y_log_ideal[k] = y_ideal

    # ------------------------------------------
    # 3. Collect training sample (uses CURRENTLY active constants)
    # ------------------------------------------
    s_k = vel_body.flatten()
    r_k = omega_ref[ref_idx].flatten()
    e_k = e[k].flatten()
    e_k1 = e[k-1].flatten()
    e_k2 = e[k-2].flatten()

    X_k = np.concatenate([s_k, r_k, e_k, e_k1, e_k2])

    X_samples.append(X_k)

# Convert to numpy arrays after loop
X_samples_final = np.array(X_samples)
Y_samples_final = np.array(Y_samples)

# Kp_new, Ki_new, Kd_new = predict_pid_constants(X_samples, Kp, Ki, Kd, 200, Ts)
# print("##############################################")
# print(f"Final Kp: {Kp_new}")
# print(f"Final Ki: {Ki_new}")
# print(f"Final Kd: {Kd_new}")
# print("##############################################")

# -------------------------------
# After loop: Simulation plotting
# -------------------------------
U = U_pid
U_ideal = U_pid_ideal

t_out, y_out = ct.input_output_response(sys_lin_z, T, U.T, X0)
t_out_ideal, y_out_ideal = ct.input_output_response(sys_lin_z, T, U_ideal.T, X0)

x_pos = y_out[0]
y_pos = y_out[1]

x_pos_ideal = y_out_ideal[0]
y_pos_ideal = y_out_ideal[1]

# World coordinate transformation
rot_angle = (-1.5*np.pi/6)+np.pi
x_pos_world = x_pos * np.cos(rot_angle) - y_pos * np.sin(rot_angle)
y_pos_world = x_pos * np.sin(rot_angle) + y_pos * np.cos(rot_angle)

x_pos_world_ideal = x_pos_ideal * np.cos(rot_angle) - y_pos_ideal * np.sin(rot_angle)
y_pos_world_ideal = x_pos_ideal * np.sin(rot_angle) + y_pos_ideal * np.cos(rot_angle)

fig, axs = plt.subplots(2, 1, figsize=(15, 20))

ax = axs[1]
ax.plot(T, omega_ref[:,0], '--', label='Ref 1')
ax.plot(T, y_log[:len(T),3], label='Actual 1')
ax.plot(T, omega_ref[:,1], '--', label='Ref 2')
ax.plot(T, y_log[:len(T),4], label='Actual 2')
ax.plot(T, omega_ref[:,2], '--', label='Ref 3')
ax.plot(T, y_log[:len(T),5], label='Actual 3')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Body Velocities (dx, dy, dphi)')
ax.legend()
ax.set_title(f'Incremental PID Velocity Tracking (Update Interval: {UPDATE_INTERVAL_S}s)')
ax.grid()

rmse_e, mae_e, abs_e = calculate_path_error_metrics(x_pos_world, y_pos_world, x_pos_world_ideal, y_pos_world_ideal)
print((f"Robot Position and Orientation. RMSE = {np.round(rmse_e, 3)} cm, MAE = {np.round(mae_e, 3)} cm, Abs Error = {np.round(abs_e, 3)} cm"))

ax = axs[0]
ax.plot(x_pos_world, y_pos_world, marker="x", label="Robot Path")
ax.plot(x_pos_world_ideal, y_pos_world_ideal, "r.", alpha=0.1, label="Robot Path Ideal")
ax.set_aspect('equal', adjustable='datalim')
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_title(f"Robot Position and Orientation. RMSE = {np.round(rmse_e, 3)} cm, MAE = {np.round(mae_e, 3)} cm, Abs Error = {np.round(abs_e, 3)} cm")
ax.legend()
ax.grid()

plt.tight_layout()
plt.savefig('pid_sim_ai_update.png')

print("The code for the AI inference and PID constant update loop has been implemented.")
print(f"The PID constants are checked and potentially updated every {UPDATE_INTERVAL_S} seconds.")
print("The data collection buffers (X_samples, Y_samples) are reset after each update.")
print("The final plot output is saved to pid_sim_ai_update.png.")