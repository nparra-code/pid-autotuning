from functions import *
import os

import control as ct
import matplotlib.pyplot as plt
import keras
from keras import losses

model_name = "lstm64tanh_64relu_exp_1401"
model_path = f"python/models/{model_name}.h5"

# Pass the built-in mean_squared_error function under the expected name 'mse'
custom_objects = {
    'mse': losses.mean_squared_error
}

model = keras.models.load_model(
    model_path,
    custom_objects=custom_objects
)

def plot_comparison_graphs(sp_before, sp_after, vel_log_before, vel_log_after, pos_before, pos_after, pos_ideal, pid_before, pid_after, time):
    """
    Generate comparison plots for before/after PID tuning
    
    Args:
        samples_before: numpy array of samples before tuning
        samples_after: numpy array of samples after tuning
        pid_before: tuple of (Kp, Ki, Kd) before tuning
        pid_after: tuple of (Kp, Ki, Kd) after tuning
        timestamp: timestamp string for filename
    """
    print("Generating comparison visualization...")
    
    # Create figure with subplots
    fig = plt.figure(figsize=(20, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    
    # Extract time vectors and normalize to start from 0
    time_before = time
    time_after = time + 1
    
    motor_names = ['Right Wheel (Motor 1)', 'Left Wheel (Motor 2)', 'Back Wheel (Motor 3)']
    colors_before = ['#FF6B6B', '#4ECDC4', '#45B7D1']
    colors_after = ['#2ECC40', '#FF851B', '#B10DC9']
    
    # Plot motor tracking for each motor (left column)
    for motor_idx in range(3):
        ax = fig.add_subplot(gs[motor_idx, 0])
        
        # Before tuning
        state_before = vel_log_before[:len(T),motor_idx + 3]
        setpoint_before = sp_before[:, motor_idx]
        
        # After tuning
        state_after = vel_log_after[:len(T), motor_idx + 3]
        setpoint_after = sp_after[:, motor_idx]
        
        # Plot setpoint and states for "before" period
        ax.plot(time_before, setpoint_before, 'k--', linewidth=2, 
               label='Setpoint', alpha=0.7, zorder=5)
        ax.plot(time_before, state_before, color=colors_before[motor_idx], 
               linewidth=1.5, label='Before Tuning', alpha=0.7)
        
        # Plot setpoint and states for "after" period (normalized time)
        ax.plot(time_after, setpoint_after, 'k--', linewidth=2, 
               alpha=0.7, zorder=5)
        ax.plot(time_after, state_after, color=colors_after[motor_idx], 
               linewidth=1.5, label='After Tuning', alpha=0.7)
        
        # Calculate PID evaluation metrics
        # Before tuning metrics
        error_before = state_before - setpoint_before
        oscillation_before = np.var(np.diff(error_before))
        steady_state_error_before = np.mean(np.abs(error_before[-int(len(error_before)*0.3):]))
        
        # Settling time: time to reach and stay within 5% of setpoint
        settling_threshold = 0.05 * np.mean(np.abs(setpoint_before))
        settling_idx_before = None
        for i in range(len(error_before) - 10):
            if np.all(np.abs(error_before[i:]) < settling_threshold):
                settling_idx_before = i
                break
        settling_time_before = time_before[settling_idx_before] if settling_idx_before else time_before[-1]
        
        # After tuning metrics
        error_after = state_after - setpoint_after
        oscillation_after = np.var(np.diff(error_after))
        steady_state_error_after = np.mean(np.abs(error_after[-int(len(error_after)*0.3):]))
        
        settling_idx_after = None
        for i in range(len(error_after) - 10):
            if np.all(np.abs(error_after[i:]) < settling_threshold):
                settling_idx_after = i
                break
        settling_time_after = time_after[settling_idx_after] if settling_idx_after else time_after[-1]
        
        # Add metrics as text annotation
        metrics_text = f'BEFORE: Osc: {oscillation_before:.4f}, SSE: {steady_state_error_before:.4f}\n'
        metrics_text += f'AFTER: Osc: {oscillation_after:.4f}, SSE: {steady_state_error_after:.4f}\n'

        ax.text(0.7, 0.98, metrics_text, transform=ax.transAxes,
               verticalalignment='top', fontsize=8,
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Velocity (rad/s)')
        ax.set_title(f'{motor_names[motor_idx]} - Tracking Performance')
        ax.legend(loc='upper left', fontsize=9)
        ax.grid(True, alpha=0.3)
    
    # Calculate trajectories
    x_desired_before, y_desired_before = pos_ideal
    x_before, y_before = pos_before
    
    x_desired_after, y_desired_after = pos_ideal
    x_after, y_after = pos_after
    
    # Plot trajectory comparison (right top)
    ax_traj = fig.add_subplot(gs[0:2, 1])
    
    # Plot both desired trajectories (they should be similar but may have different lengths)
    ax_traj.plot(x_desired_before * 100, y_desired_before * 100, 'k--', linewidth=2.5, alpha=0.8, 
                label='Desired Trajectory', zorder=5)
    ax_traj.plot(x_before * 100, y_before * 100, color='#FF6B6B', linewidth=2, 
                label='BEFORE', alpha=0.7)
    ax_traj.plot(x_after * 100, y_after * 100, color='#2ECC40', linewidth=2, 
                label='AFTER', alpha=0.7)

    # Mark start and end points
    ax_traj.scatter(x_desired_before[0] * 100, y_desired_before[0] * 100, color='blue', s=150, 
                   marker='o', label='Start', zorder=10, edgecolors='black', linewidths=2)
    ax_traj.scatter(x_desired_before[-1] * 100, y_desired_before[-1] * 100, color='red', s=150, 
                   marker='s', label='End (Desired)', zorder=10, edgecolors='black', linewidths=2)
    ax_traj.scatter(x_after[-1] * 100, y_after[-1] * 100, color='green', s=150, 
                   marker='^', label='End', zorder=10, edgecolors='black', linewidths=2)

    ax_traj.set_xlabel('X Position (cm)')
    ax_traj.set_ylabel('Y Position (cm)')
    ax_traj.set_title('Robot Trajectory Comparison', fontsize=14, fontweight='bold')
    ax_traj.legend(loc='best')
    ax_traj.grid(True, alpha=0.3)
    ax_traj.axis('equal')
    
    # Plot trajectory error over time (right bottom)
    ax_error = fig.add_subplot(gs[2, 1])
    
    # Calculate errors for each dataset separately (they may have different lengths)
    error_before = np.sqrt((x_desired_before - x_before)**2 + (y_desired_before - y_before)**2) * 100  # cm
    error_after = np.sqrt((x_desired_after - x_after)**2 + (y_desired_after - y_after)**2) * 100  # cm
    
    ax_error.plot(time_before, error_before, color='#FF6B6B', linewidth=2, 
                 label='BEFORE', alpha=0.7)
    ax_error.fill_between(time_before, 0, error_before, color='#FF6B6B', alpha=0.2)
    
    ax_error.plot(time_after, error_after, color='#2ECC40', linewidth=2, 
                 label='AFTER', alpha=0.7)
    ax_error.fill_between(time_after, 0, error_after, color='#2ECC40', alpha=0.2)
    
    ax_error.set_xlabel('Time (s)')
    ax_error.set_ylabel('Position Error (cm)')
    ax_error.set_title('Trajectory Tracking Error Over Time', fontsize=12, fontweight='bold')
    ax_error.legend(loc='upper right')
    ax_error.grid(True, alpha=0.3)
    
    # Add statistics text (calculate separately for each dataset)
    max_before = np.max(error_before)
    rmse_before = np.sqrt(np.mean(error_before**2))
    mae_before = np.mean(np.abs(error_before))
    stddev_before = np.std(error_before)

    max_after = np.max(error_after)
    rmse_after = np.sqrt(np.mean(error_after**2))
    mae_after = np.mean(np.abs(error_after))
    stddev_after = np.std(error_after)

    improvement = ((mae_before - mae_after) / mae_before) * 100 if mae_before > 0 else 0

    stats_text = f'BEFORE: Max={max_before:.2f}cm, RMSE={rmse_before:.2f}cm, MAE={mae_before:.2f}cm, StdDev={stddev_before:.2f}cm\n'
    stats_text += f'AFTER: Max={max_after:.2f}cm, RMSE={rmse_after:.2f}cm, MAE={mae_after:.2f}cm, StdDev={stddev_after:.2f}cm\n'
    stats_text += f'Change: {improvement:.1f}%\n'
    
    ax_error.text(0.02, 0.98, stats_text, transform=ax_error.transAxes,
                 verticalalignment='top', fontsize=9,
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Add title with PID constants
    Kp_before, Ki_before, Kd_before = pid_before
    Kp_after, Ki_after, Kd_after = pid_after

    # Format arrays to avoid scientific notation
    def format_array(arr):
        return [f"{val:.4f}" for val in arr]
    
    title_text = f'PID Simulator Auto-Tuning Results Comparison\n'
    title_text += f'BEFORE: Kp={Kp_before}, Ki={Ki_before}, Kd={Kd_before}\n'
    title_text += f'AFTER: Kp=[{Kp_after[0]:.4f}, {Ki_after[0]:.4f}, {Kd_after[0]:.4f}], Ki=[{Kp_after[1]:.4f}, {Ki_after[1]:.4f}, {Kd_after[1]:.4f}], Kd=[{Kp_after[2]:.4f}, {Ki_after[2]:.4f}, {Kd_after[2]:.4f}]'

    fig.suptitle(title_text, fontsize=14, fontweight='bold')
    
    # Save figure
    filename = os.path.join("./", f'pid_tuning_comparison_{model_name}.png')
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Saved comparison plot: {filename}")
    
    plt.close(fig)
    
    return filename

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
Kp = np.array([1, 1, 1])
Ki = np.array([0, 0, 0])
Kd = np.array([0, 0, 0])

# Ideal PID parameters
Kp_ideal = np.array([2.5, 2.3, 2.6])
Ki_ideal = np.array([0.002, 0.001, 0.002])
Kd_ideal = np.array([0.003, 0.002, 0.003])

# Desired trajectory generation (Modified time to match simulation time)
omega_ref, T = generate_omega_ref_trajectory(
    mov_tuples_list=[
       #(type    ,fw/cw,  lv, ang, r, t)
        ('linear', True,  12,  90, 0, 10),
        ('linear', True,  10,   0, 0, 5),
        ('linear', False, 20,   0, 0, 7),
        ('circular', True, 5, 360, 20, 28)
    ], Ts=Ts
)

# Initialize
U_pid = np.zeros((len(T), 3))
U_pid_ideal = np.zeros((len(T), 3))
e = np.zeros((len(T), 3))
e_ideal = np.zeros((len(T), 3))

x = np.zeros((6,))
x_ideal = np.zeros((6,))

# Initialize logs
y_log_before = np.zeros((len(T), 6))
x_log = np.zeros((len(T), 6))
y_log_ideal = np.zeros((len(T), 6))
x_log_ideal = np.zeros((len(T), 6))

# Initialize data containers for AI training/inference for the current interval
X_samples = []
Y_samples = []

# Control loop simulation
for k in range(2, len(T)):
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
    dU = (abs(Kp) * 8.3  * (e[k] - e[k-1]) +
          abs(Ki) * 8500 * e[k] +
          abs(Kd) * (e[k] - 2*e[k-1] + e[k-2]))

    dU_ideal = (abs(Kp_ideal) * 8.3  * (e_ideal[k] - e_ideal[k-1]) +
                abs(Ki_ideal) * 8500 * e_ideal[k] +
                abs(Kd_ideal) * (e_ideal[k] - 2*e_ideal[k-1] + e_ideal[k-2]))

    # Update control signal and apply to discrete system
    U_pid[k] = U_pid[k-1] + dU
    U_pid_ideal[k] = U_pid_ideal[k-1] + dU_ideal
    x = A_z @ x + B_z @ U_pid[k]
    y = C_z @ x + D_z @ U_pid[k]
    x_ideal = A_z @ x_ideal + B_z @ U_pid_ideal[k]
    y_ideal = C_z @ x_ideal + D_z @ U_pid_ideal[k]

    # Log data
    x_log[k] = x
    y_log_before[k] = y
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

Kp_new, Ki_new, Kd_new = predict_pid_constants(model, X_samples)
print("##############################################")
print('Inference:')
print(f"Kp = np.array([{', '.join(map(str, Kp_new))}])")
print(f"Ki = np.array([{', '.join(map(str, Ki_new))}])")
print(f"Kd = np.array([{', '.join(map(str, Kd_new))}])")
print("##############################################")

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
rot_angle = (np.pi/4) - np.pi/2  # 45 degrees - 90 degrees
x_pos_world_before = x_pos * np.cos(rot_angle) - y_pos * np.sin(rot_angle)
y_pos_world_before = x_pos * np.sin(rot_angle) + y_pos * np.cos(rot_angle)

x_pos_world_ideal = x_pos_ideal * np.cos(rot_angle) - y_pos_ideal * np.sin(rot_angle)
y_pos_world_ideal = x_pos_ideal * np.sin(rot_angle) + y_pos_ideal * np.cos(rot_angle)

x_pos_world_after = None
y_pos_world_after = None
y_log_after = None

reps = 2  # Number of tuning iterations
for i in range(reps):

    Kp = Kp_new
    Ki = Ki_new
    Kd = Kd_new

    # Initialize
    U_pid = np.zeros((len(T), 3))
    e = np.zeros((len(T), 3))

    x = np.zeros((6,))
    x_ideal = np.zeros((6,))

    # Initialize logs
    y_log_after = np.zeros((len(T), 6))
    x_log = np.zeros((len(T), 6))

    # Initialize data containers for AI training/inference for the current interval
    X_samples = []
    Y_samples = []

    # Control loop simulation
    for k in range(2, len(T)):
        # ------------------------------------------
        # 2. PID CONTROL LOOP
        # ------------------------------------------

        # Measure: get current body velocities (dx, dy, dphi)
        vel_body = x[3:6]

        # Compute error (use modulo or clip for ref_idx if T length is different from omega_ref length)
        ref_idx = min(k, len(omega_ref) - 1)
        e[k] = omega_ref[ref_idx] - vel_body

        # Incremental PID law (uses the CURRENTLY active Kp, Ki, Kd)
        dU = (abs(Kp) * 8.3  * (e[k] - e[k-1]) +
            abs(Ki) * 8500 * e[k] +
            abs(Kd) * (e[k] - 2*e[k-1] + e[k-2]))

        dU_ideal = (abs(Kp_ideal) * 8.3  * (e_ideal[k] - e_ideal[k-1]) +
                    abs(Ki_ideal) * 8500 * e_ideal[k] +
                    abs(Kd_ideal) * (e_ideal[k] - 2*e_ideal[k-1] + e_ideal[k-2]))

        # Update control signal and apply to discrete system
        U_pid[k] = U_pid[k-1] + dU
        U_pid_ideal[k] = U_pid_ideal[k-1] + dU_ideal
        x = A_z @ x + B_z @ U_pid[k]
        y = C_z @ x + D_z @ U_pid[k]
        x_ideal = A_z @ x_ideal + B_z @ U_pid_ideal[k]
        y_ideal = C_z @ x_ideal + D_z @ U_pid_ideal[k]

        # Log data
        x_log[k] = x
        y_log_after[k] = y

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

    if i < reps - 1:
        Kp_new, Ki_new, Kd_new = predict_pid_constants(model, X_samples)
        print("##############################################")
        print('Inference:')
        print(f"Kp = np.array([{', '.join(map(str, Kp_new))}])")
        print(f"Ki = np.array([{', '.join(map(str, Ki_new))}])")
        print(f"Kd = np.array([{', '.join(map(str, Kd_new))}])")
        print("##############################################")

    # -------------------------------
    # After loop: Simulation plotting
    # -------------------------------
    U = U_pid

    t_out, y_out = ct.input_output_response(sys_lin_z, T, U.T, X0)

    x_pos = y_out[0]
    y_pos = y_out[1]

    # World coordinate transformation
    rot_angle = (np.pi/4) - np.pi/2  # 45 degrees - 90 degrees
    x_pos_world_after = x_pos * np.cos(rot_angle) - y_pos * np.sin(rot_angle)
    y_pos_world_after = x_pos * np.sin(rot_angle) + y_pos * np.cos(rot_angle)

# Generate comparison plots
plot_comparison_graphs(
    sp_before=omega_ref,
    sp_after=omega_ref,
    vel_log_before=y_log_before,
    vel_log_after=y_log_after,
    pos_before=(x_pos_world_before, y_pos_world_before),
    pos_after=(x_pos_world_after, y_pos_world_after),
    pos_ideal=(x_pos_world_ideal, y_pos_world_ideal),
    pid_before=([1, 1, 1], [0, 0, 0], [0, 0, 0]),
    pid_after=([Kp[0], Ki[0], Kd[0]], [Kp[1], Ki[1], Kd[1]], [Kp[2], Ki[2], Kd[2]]),
    time=T
)