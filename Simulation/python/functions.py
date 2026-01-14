import numpy as np
from math import sqrt, sin, cos, pi
from movements import lin_to_ang_vel, lin_move, circ_move

def updfnc_omniwheel(t, X, U, params):
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
    return X

def generate_omega_ref_trajectory(mov_tuples_list, Ts):
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
    """Compute steady-state error, overshoot, oscillation, and settling time."""
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
    Convert robot velocities [dx, dy, dphi] to wheel velocities [w1, w2, w3]
    using the H matrix from the omniwheel kinematics
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
    """Run PID loop simulation for given gains."""
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
    """Generate 1 random set of K_n and Ra_n."""
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
    X_seq = []
    for i in range(len(X) - seq_len):
        X_seq.append(X[i:i+seq_len])
    return np.array(X_seq)

def calculate_path_error_metrics(x_actual, y_actual, x_ideal, y_ideal):
    """
    Calculates the instantaneous position error, RMSE, and MAE
    for the robot's global path tracking.
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
    Placeholder for AI model inference. In a real application, this would:
    1. Aggregate or process X_samples (state, reference, error data) collected.
    2. Load the trained AI model.
    3. Run model.predict() to get new Kp, Ki, Kd values.

    For demonstration, we check the current time 'T[k]' and update Kp1 at t=5.0s.
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

    # print(f'Kp = np.array([{mean_gains[0]}, {mean_gains[1]}, {mean_gains[2]}])')
    # print(f'Ki = np.array([{mean_gains[3]}, {mean_gains[4]}, {mean_gains[5]}])')
    # print(f'Kd = np.array([{mean_gains[6]}, {mean_gains[7]}, {mean_gains[8]}])')

    # print(f"\n--- PID CONSTANTS UPDATED at t={T_current}s ---")
    # print(f"Kp before: {current_Kp}, Kp after: {Kp_new}")
    # print("-----------------------------------------\n")

    return Kp_new, Ki_new, Kd_new