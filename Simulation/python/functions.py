"""
@file functions.py
@brief Core functions for omniwheel robot simulation and PID control
@details This module contains the main functions for simulating an omniwheel robot system,
         including state update functions, trajectory generation, PID simulation, and 
         quality metrics calculation.
"""

import numpy as np
from math import sqrt, sin, cos, pi
from movements import lin_to_ang_vel, lin_move, circ_move

def updfnc_omniwheel(t, X, U, params):
    """
    @brief Update function for omniwheel robot dynamics
    @details Computes the state derivative for a 3-wheel omniwheel robot considering motor dynamics,
             planetary gearboxes, and slip model
    
    @param t Current time (scalar)
    @param X State vector [x, y, phi, dx, dy, dphi] - position and velocity in world frame
    @param U Control input vector [u1, u2, u3] - voltage commands for three motors
    @param params Dictionary containing robot and motor parameters:
                  - d: Distance of wheels from robot center (m)
                  - r: Wheel radius (m)
                  - K_1, K_2, K_3: Motor constants (V/rad/s)
                  - Ra_1, Ra_2, Ra_3: Motor armature resistances (Ohm)
                  - M: Robot mass (kg)
                  - I: Robot moment of inertia (kg*m^2)
                  - delt: Wheel angle offset (rad)
                  - n: Gearbox reduction ratios [n1, n2, n3]
                  - eta: Gearbox efficiencies [eta1, eta2, eta3]
    
    @return State derivative vector [dx, dy, dphi, ddx, ddy, ddphi]
    """
    # Parameter setup
    d = params.get('d', 0.099)   # Distance of the wheels wrt robot center
    r = params.get('r', 0.0325)  # Wheel radius

    # Motor constants
    K_1 = params.get('K_1', 0.259)
    K_2 = params.get('K_2', 0.238)
    K_3 = params.get('K_3', 0.204)
    Ra_1 = params.get('Ra_1', 1.3211)
    Ra_2 = params.get('Ra_2', 1.3111)
    Ra_3 = params.get('Ra_3', 1.3010)

    # print(f'Running updfnc_omniwheel with motor constants: {K_1}, {K_2}, {K_3}, {Ra_1}, {Ra_2}, {Ra_3}')

    # Robot parameters
    M = params.get('M', 1.944)
    I = params.get('I', 0.015)
    delt = params.get('delt', np.pi/6)

    # Map states
    x, y, phi, dx, dy, dphi = X
    phi = phi + delt

    # Matrices
    H = np.array([[0, -1, d],
                  [np.cos(delt), np.sin(delt), d],
                  [-np.cos(delt), np.sin(delt), d]])

    D = np.array([[1/M, 0, 0],
                  [0, 1/M, 0],
                  [0, 0, 1/I]])

    Rphi = np.array([[np.cos(phi), np.sin(phi), 0],
                     [-np.sin(phi), np.cos(phi), 0],
                     [0, 0, 1]])

    Rphi_ = np.array([[-np.sin(phi), np.cos(phi), 0],
                      [-np.cos(phi), -np.sin(phi), 0],
                      [0, 0, 0]])

    # Lambda matrices (without n/r factors)
    Lambda_E = np.diag([K_1/Ra_1, K_2/Ra_2, K_3/Ra_3])
    Lambda_w = np.diag([K_1**2/Ra_1, K_2**2/Ra_2, K_3**2/Ra_3])

    # -----------------------------------------------------------------------
    # PLANETARY GEARBOXES CONSIDERATIONS
    # Reduction ratios (e.g., [16.0, 15.9, 16.1])
    n = params.get('n', np.array([16.0, 15.9, 16.1]))
    # Gearbox Efficiencies (e.g., [0.9, 0.95, 0.88])
    eta = params.get('eta', np.array([0.95, 0.98, 0.89]))

    # Ensure n and eta are 1D arrays for element-wise operations
    if np.isscalar(n):
        n = np.array([n, n, n])
    if np.isscalar(eta):
        eta = np.array([eta, eta, eta])

    # Damping and Ratio factors (element-wise vectors)
    N_over_R = n / r
    N_squared_over_Eta_R_squared = (n**2) / (eta * r**2)
    # Convert vectors to diagonal matrices for matrix multiplication
    N_R_diag = np.diag(N_over_R)
    N_Eta_R_diag = np.diag(N_squared_over_Eta_R_squared)
    # -----------------------------------------------------------------------

    # New equation implementation
    # term_input = (n/r) * (D @ H.T @ Lambda_E @ U)
    term_input = D @ H.T @ Lambda_E @ N_R_diag @ np.diag(eta) @ U

    # -----------------------------------------------------------------------
    # Start of Simple Slip Model Implementation
    U_norm = np.linalg.norm(U)  # Use a norm or max of U as an indicator of high revs

    U_threshold = 10.0  # Tune this threshold (e.g., 50% of max voltage)
    Slip_Factor_Min = 0.5 # Minimum effective acceleration when slipping

    # Calculate a reduction factor: 1.0 (no slip) down to Slip_Factor_Min (max slip)
    if U_norm > U_threshold:
        # Scale the factor based on how far U is above the threshold
        reduction_factor = 1.0 - (1.0 - Slip_Factor_Min) * (U_norm - U_threshold) / U_threshold
        reduction_factor = np.clip(reduction_factor, Slip_Factor_Min, 1.0)
    else:
        reduction_factor = 1.0

    term_input_with_slip = term_input * reduction_factor
    # -----------------------------------------------------------------------

    # term_damping = (n**2 / r**2) * (D @ H.T @ Lambda_w @ H @ Rphi @ X[3:])
    term_damping = D @ H.T @ Lambda_w @ N_Eta_R_diag @ H @ Rphi @ X[3:]

    #ddq = np.linalg.inv(Rphi) @ (term_input - term_damping - Rphi_ @ X[3:])
    ddq = np.linalg.inv(Rphi) @ (term_input_with_slip - term_damping - Rphi_ @ X[3:])

    # State derivative
    return np.array([dx, dy, dphi, ddq[0], ddq[1], ddq[2]])


def outfnc_omniwheel(t, X, U, params):
    """
    @brief Output function for omniwheel robot
    @details Returns the full state vector as output (identity mapping)
    
    @param t Current time (scalar)
    @param X State vector [x, y, phi, dx, dy, dphi]
    @param U Control input vector [u1, u2, u3]
    @param params Parameter dictionary (not used in this function)
    
    @return State vector X unchanged
    """
    return X

def generate_omega_ref_trajectory(mov_tuples_list, Ts):
  """
  @brief Generate reference omega trajectory from movement descriptions
  @details Converts high-level movement commands (linear, circular) into wheel velocity references
  
  @param mov_tuples_list List of movement tuples, each tuple contains:
                         (mov_type, forward/cw, linear_vel, angle, radius, time)
                         - mov_type: 'linear' or 'circular'
                         - forward/cw: True for forward/clockwise, False otherwise
                         - linear_vel: Linear velocity (m/s)
                         - angle: Angle in degrees
                         - radius: Radius for circular motion (m)
                         - time: Duration of movement (s)
  @param Ts Sampling time period (s)
  
  @return Tuple (omega_ref, total_sim_time) where:
          - omega_ref: numpy array of shape (N, 3) with wheel velocities
          - total_sim_time: time vector for the trajectory
  """
  # compute total simulation time vector
  for mov_tuple in mov_tuples_list:
    print(mov_tuple)
  total_sim_time = np.arange(0, sum([mov_tuple[5] for mov_tuple in mov_tuples_list]), Ts)

  # create lists to accumulate omega_ref values
  omega_ref_list = []

  for mov_tuple in mov_tuples_list:
    mov_type, f, lin_vel, angle, radius, time = mov_tuple
    for step in range(int(time / Ts)):
      t = step * Ts
      if mov_type == 'linear':
        x_v, y_v = lin_move(f, lin_vel, angle)
      elif mov_type == 'circular':
        x_v, y_v = circ_move(f, lin_vel, angle, radius, t)
      else:
        print("Invalid movement type. Expected 'linear' or 'circular'.")
        return
      omega_vals = [
        lin_to_ang_vel(x_v, y_v, 1),
        lin_to_ang_vel(x_v, y_v, 2),
        lin_to_ang_vel(x_v, y_v, 3)
      ]
      omega_ref_list.append(omega_vals)

  # convert list to numpy array
  omega_ref = np.array(omega_ref_list)
  return omega_ref, total_sim_time

# --- Define helper functions ---
def quality_metrics(ref, actual, T, tol=0.05):
    """
    @brief Compute quality metrics for PID controller performance
    @details Calculates steady-state error, overshoot, and oscillation metrics
    
    @param ref Reference trajectory array of shape (N, num_outputs)
    @param actual Actual trajectory array of shape (N, num_outputs)
    @param T Time vector (not used in current implementation)
    @param tol Tolerance for steady-state detection (default: 0.05)
    
    @return Tuple (setpoint_error, overshoot, oscillation) containing:
            - setpoint_error: Mean absolute steady-state error
            - overshoot: Mean overshoot percentage across all outputs
            - oscillation: Mean variance of error in steady-state region
    """
    N = len(ref)
    ref_final = np.mean(ref[int(0.8*N):], axis=0)
    act_final = np.mean(actual[int(0.8*N):], axis=0)

    # --- Steady-state error ---
    setpoint_error = np.mean(np.abs(ref_final - act_final))

    # --- Overshoot ---
    overshoot = np.mean([
        max(0, (np.max(actual[:, i]) - ref_final[i]) / (abs(ref_final[i]) + 1e-6))
        for i in range(ref.shape[1])
    ])

    # --- Oscillation (variance around final value after steady-state) ---
    steady_idx = int(0.8 * N)
    oscillation = np.mean([
        np.var(actual[steady_idx:, i] - ref_final[i])
        for i in range(ref.shape[1])
    ])

    return setpoint_error, overshoot, oscillation

# Create the inverse kinematic mapping to get wheel velocities from robot velocities
# From functions.py, we have H matrix that maps wheel velocities to robot velocities
# We need H^-1 or pseudo-inverse to go from robot velocities to wheel velocities
def get_wheel_velocities_from_robot_state(robot_vel, phi, d=0.099):
    """
    @brief Convert robot velocities to wheel velocities using inverse kinematics
    @details Uses the H matrix from omniwheel kinematics to map from robot body frame
             velocities to individual wheel velocities
    
    @param robot_vel Robot velocity vector [dx, dy, dphi] in body frame
    @param phi Current robot orientation angle (rad)
    @param d Distance of wheels from robot center (m, default: 0.099)
    
    @return Wheel velocity vector [w1, w2, w3] (rad/s)
    """
    delt = np.pi/6
    phi = phi + delt
    
    H = np.array([[0, -1, d],
                  [np.cos(delt), np.sin(delt), d],
                  [-np.cos(delt), np.sin(delt), d]])
    
    Rphi = np.array([[np.cos(phi), np.sin(phi), 0],
                     [-np.sin(phi), np.cos(phi), 0],
                     [0, 0, 1]])
    
    # wheel_vel = H @ Rphi @ robot_vel (in local frame)
    wheel_vel = H @ Rphi @ robot_vel
    return wheel_vel

def run_pid_sim(Kp, Ki, Kd, beta, A, B, C, D, T, omega_ref, Ts, save_to_file, file_name, directory):
    """
    @brief Run PID control loop simulation with specified gains
    @details Simulates a discrete-time PID controller applied to a linear system
    
    @param Kp Proportional gains array [Kp1, Kp2, Kp3]
    @param Ki Integral gains array [Ki1, Ki2, Ki3]
    @param Kd Derivative gains array [Kd1, Kd2, Kd3]
    @param beta Setpoint weighting factor (not currently used)
    @param A State transition matrix (discrete-time)
    @param B Input matrix (discrete-time)
    @param C Output matrix (discrete-time)
    @param D Feedthrough matrix (discrete-time)
    @param T Time vector for simulation
    @param omega_ref Reference trajectory array of shape (N, 3)
    @param Ts Sampling time period (s)
    @param save_to_file Boolean flag to save training data
    @param file_name Filename for saved data (without extension)
    @param directory Directory path for saved data
    
    @return Tuple (set_err, overshoot, osc, y_log, U_pid) containing:
            - set_err: Steady-state error metric
            - overshoot: Overshoot metric
            - osc: Oscillation metric
            - y_log: Output trajectory log
            - U_pid: Control input history
    """
    x = np.zeros((A.shape[0],))
    U_pid = np.zeros((len(T), 3))
    e = np.zeros((len(T), 3))
    x_log = np.zeros((len(T), 6))
    y_log = np.zeros((len(T), 6))

    # Initialize data containers before the loop
    X_samples = []
    Y_samples = []

    for k in range(2, len(T)):
        # Get robot velocities in world frame
        robot_vel = x[3:6]  # [dx, dy, dphi]
        phi_current = x[2]

        # Compute error in wheel velocity space
        e[k] = omega_ref[k] - robot_vel

        # Incremental PID law
        dU = (Kp * 8.3 * (e[k] - e[k-1]) +
            Ki * 8500 * e[k] +
            Kd * (e[k] - 2*e[k-1] + e[k-2]))

        # Update control signal
        U_pid[k] = U_pid[k-1] + dU
    
        x = A @ x + B @ U_pid[k]
        y = C @ x + D @ U_pid[k]
        x_log[k] = x
        y_log[k] = y

        if save_to_file:
          s_k = robot_vel.flatten()
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

    if save_to_file:
      X_samples = np.array(X_samples)
      Y_samples = np.array(Y_samples)
      np.savez(f'{directory}{file_name}.npz', X=X_samples, y=Y_samples)

    set_err, overshoot, osc = quality_metrics(omega_ref, y_log[:, 3:6], T)
    return set_err, overshoot, osc, y_log, U_pid

def generate_random_motor_params():
    """
    @brief Generate random motor parameters for Monte Carlo simulation
    @details Creates random variations of motor constants and armature resistances
             within ±40% of nominal values
    
    @return Dictionary containing:
            - K_1, K_2, K_3: Motor constants (V/rad/s)
            - Ra_1, Ra_2, Ra_3: Motor armature resistances (Ohm)
    """
    dev_c = 0.4  # 35% deviation
    nom_K = 0.259
    nom_Ra = 1.3111
    K_vals = np.random.uniform(nom_K - (nom_K*dev_c), nom_K + (nom_K*dev_c), size=3)      # K1, K2, K3
    Ra_vals = np.random.uniform(nom_Ra - (nom_Ra*dev_c), nom_Ra + (nom_Ra*dev_c), size=3)     # Ra1, Ra2, Ra3

    params = {
        'K_1': K_vals[0], 'K_2': K_vals[1], 'K_3': K_vals[2],
        'Ra_1': Ra_vals[0], 'Ra_2': Ra_vals[1], 'Ra_3': Ra_vals[2],
    }
    return params

def build_sequences(X, seq_len=20):
    """
    @brief Build sequences for RNN input from time series data
    @details Creates sliding window sequences of specified length
    
    @param X Input data array of shape (N, num_features)
    @param seq_len Sequence length for sliding window (default: 20)
    
    @return Numpy array of shape (N-seq_len, seq_len, num_features)
    """
    X_seq = []
    for i in range(len(X) - seq_len):
        X_seq.append(X[i:i+seq_len])
    return np.array(X_seq)

def calculate_path_error_metrics(x_actual, y_actual, x_ideal, y_ideal):
    """
    @brief Calculate path tracking error metrics
    @details Computes RMSE, MAE, and total absolute error for trajectory following
    
    @param x_actual Actual x-coordinates of robot trajectory
    @param y_actual Actual y-coordinates of robot trajectory
    @param x_ideal Ideal/reference x-coordinates
    @param y_ideal Ideal/reference y-coordinates
    
    @return Tuple (rmse, mae, abs_err) containing:
            - rmse: Root mean square error (m)
            - mae: Mean absolute error (m)
            - abs_err: Total absolute error (m)
    """
    # 1. Calculate the error in X and Y components at each step
    error_x = x_ideal - x_actual
    error_y = y_ideal - y_actual

    abs_err = np.sum(np.abs(error_x)+np.abs(error_y))

    # 2. Calculate the Instantaneous Position Error (Euclidean Distance)
    # This is the distance between the actual (x_actual, y_actual) and ideal (x_ideal, y_ideal)
    e_pos = np.sqrt(error_x**2 + error_y**2)

    # 3. Calculate Root Mean Square Error (RMSE)
    # The magnitude of the instantaneous error vector is squared, averaged, then rooted.
    rmse = np.sqrt(np.mean(e_pos**2))

    # 4. Calculate Mean Absolute Error (MAE)
    # The average of the instantaneous errors.
    mae = np.mean(e_pos)

    return rmse, mae, abs_err

# --- AI Model Placeholder Function ---
def predict_pid_constants(model, X_samples):
    """
    @brief Predict PID constants using trained AI model
    @details Uses RNN model to predict optimal PID gains based on collected trajectory samples
    
    @param model Trained Keras/TensorFlow model for PID prediction
    @param X_samples Collected samples array of shape (N, 15) containing
                    [state, reference, error] sequences
    
    @return Tuple (Kp_new, Ki_new, Kd_new) containing:
            - Kp_new: Predicted proportional gains [Kp1, Kp2, Kp3]
            - Ki_new: Predicted integral gains [Ki1, Ki2, Ki3]
            - Kd_new: Predicted derivative gains [Kd1, Kd2, Kd3]
    """
    

    X_all = []
    X_seq = build_sequences(X_samples, 20)
    X_all.append(X_seq)
    X_all = np.concatenate(X_all, axis=0)

    # --- PLACEHOLDER LOGIC: Adjust Kp for wheel 1 ---
    y_pred = model.predict(X_all, verbose=1)
    mean_gains = np.mean(y_pred, axis=0)

    Kp_new = np.array([mean_gains[0], mean_gains[1], mean_gains[2]])
    Ki_new = np.array([mean_gains[3], mean_gains[4], mean_gains[5]])
    Kd_new = np.array([mean_gains[6], mean_gains[7], mean_gains[8]])

    return Kp_new, Ki_new, Kd_new