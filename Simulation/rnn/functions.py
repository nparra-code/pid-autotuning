import numpy as np
import pandas as pd
import re
import os

def build_sequences(X, y, seq_len=20):
    X_seq, y_seq = [], []
    for i in range(len(X) - seq_len):
        X_seq.append(X[i:i+seq_len])
        y_seq.append(y[i+seq_len-1].flatten())  # ensure (3,), not (3,3)
    return np.array(X_seq), np.array(y_seq)

def load_dataset(folder, seq_len=20):
    X_all, y_all = [], []
    import glob
    for f in glob.glob(f"{folder}/*.npz"):
        data = np.load(f)
        X, y = data['X'], data['y']
        X_seq, y_seq = build_sequences(X, y, seq_len)
        X_all.append(X_seq)
        y_all.append(y_seq)
    X_all = np.concatenate(X_all, axis=0)
    y_all = np.concatenate(y_all, axis=0)
    return X_all, y_all

def parse_pid_gains_from_filename(filename):
    """
    Extract PID gains from filename format:
    'telemetry_x_x_[Kp0, Kp1, Kp2][Ki0, Ki1, Ki2][Kd0, Kd1, Kd2].csv'
    
    Returns:
        numpy array of shape (3, 3) where rows are [motor0, motor1, motor2]
        and columns are [Kp, Ki, Kd]
    """
    basename = os.path.basename(filename)
    
    # Find all bracketed arrays in the filename
    pattern = r'\[([^\]]+)\]'
    matches = re.findall(pattern, basename)
    
    if len(matches) < 3:
        raise ValueError(f"Could not parse PID gains from filename: {filename}")
    
    # Parse the three arrays: Kp, Ki, Kd
    kp_values = [float(x.strip()) for x in matches[0].split(',')]
    ki_values = [float(x.strip()) for x in matches[1].split(',')]
    kd_values = [float(x.strip()) for x in matches[2].split(',')]
    
    # Return as array where each row is [Kp, Ki, Kd] for that motor
    return np.array([
        [kp_values[0], ki_values[0], kd_values[0]],  # motor 0
        [kp_values[1], ki_values[1], kd_values[1]],  # motor 1
        [kp_values[2], ki_values[2], kd_values[2]]   # motor 2
    ])

def load_experimental_csv(csv_path):
    """
    Load experimental telemetry data from CSV and extract training samples.
    
    CSV columns expected:
    - motor_state_0, motor_state_1, motor_state_2 (current states)
    - motor_setpoint_0, motor_setpoint_1, motor_setpoint_2 (references)
    - error_0_k, error_0_k1, error_0_k2 (errors for motor 0)
    - error_1_k, error_1_k1, error_1_k2 (errors for motor 1)
    - error_2_k, error_2_k1, error_2_k2 (errors for motor 2)
    
    Filename format: 'telemetry_x_x_[Kps][Kis][Kds].csv'
    
    Returns:
        X_samples: numpy array of shape (N, 15) containing [s_k, r_k, e_k, e_k1, e_k2]
        Y_samples: numpy array of shape (N, 9) containing [Kp0, Ki0, Kd0, Kp1, Ki1, Kd1, Kp2, Ki2, Kd2]
    """
    # Load CSV
    df = pd.read_csv(csv_path)
    
    # Parse PID gains from filename
    pid_gains = parse_pid_gains_from_filename(csv_path)  # shape: (3, 3)
    
    # Flatten to [Kp0, Ki0, Kd0, Kp1, Ki1, Kd1, Kp2, Ki2, Kd2]
    Y_k = pid_gains.flatten()  # shape: (9,)
    
    X_samples = []
    Y_samples = []
    
    for idx in range(len(df)):
        # Extract features for this timestep
        s_k = df.loc[idx, ['motor_state_0', 'motor_state_1', 'motor_state_2']].values
        r_k = df.loc[idx, ['motor_setpoint_0', 'motor_setpoint_1', 'motor_setpoint_2']].values
        e_k = df.loc[idx, ['error_0_k', 'error_1_k', 'error_2_k']].values
        e_k1 = df.loc[idx, ['error_0_k1', 'error_1_k1', 'error_2_k1']].values
        e_k2 = df.loc[idx, ['error_0_k2', 'error_1_k2', 'error_2_k2']].values
        
        # Concatenate to match simulation format: [s_k, r_k, e_k, e_k1, e_k2]
        X_k = np.concatenate([s_k, r_k, e_k, e_k1, e_k2])  # shape: (15,)
        
        X_samples.append(X_k)
        Y_samples.append(Y_k)
    
    return np.array(X_samples), np.array(Y_samples)

def load_experimental_dataset(folder, seq_len=20):
    """
    Load all experimental CSV files from a folder and build sequences for RNN training.
    
    Args:
        folder: path to folder containing CSV files
        seq_len: sequence length for RNN
        
    Returns:
        X_seq: numpy array of shape (N, seq_len, 15)
        y_seq: numpy array of shape (N, 9)
    """
    import glob
    
    X_all, y_all = [], []
    
    for csv_file in glob.glob(f"{folder}/*.csv"):
        print(f"Processing {csv_file}")
        try:
            X, y = load_experimental_csv(csv_file)
            X_seq, y_seq = build_sequences(X, y, seq_len)
            X_all.append(X_seq)
            y_all.append(y_seq)
        except Exception as e:
            print(f"Error processing {csv_file}: {e}")
            continue
    
    if not X_all:
        raise ValueError(f"No valid CSV files found in {folder}")
    
    X_all = np.concatenate(X_all, axis=0)
    y_all = np.concatenate(y_all, axis=0)
    
    return X_all, y_all