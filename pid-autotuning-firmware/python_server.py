#!/usr/bin/env python3
"""
PID Autotuner Server
Receives robot telemetry data, runs RNN inference, returns PID constants
"""

import socket
import struct
import numpy as np
import logging
from typing import Tuple, Optional
import zlib
from datetime import datetime
import os
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for server
import csv
import json

import keras
from keras import losses

model_name = "3wheel_model_lstm_128_tanh_experimental_1812"
model_path = f"{model_name}.h5"
RUNS_TO_CONVERGE = 4

# Configure logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Protocol constants
MSG_TYPE_DATA_BATCH = 0x01
MSG_TYPE_PID_RESPONSE = 0x02
MSG_TYPE_ACK = 0x03
MSG_TYPE_ERROR = 0x04
MSG_TYPE_IDENT_DATA = 0x05
MSG_TYPE_INFERENCE_REQUEST = 0x06  # New: Request inference on accumulated data
PROTOCOL_VERSION = 1
PORT = 8888

# Data structures matching ESP32 side
ROBOT_SAMPLE_STRUCT = struct.Struct('<I15f')  # timestamp + 15 floats
# I=uint32, f=float (timestamp, 
#                     motor_state[3], motor_setpoint[3], errors[3][3])
# Total: 1 uint32 + 3 floats + 3 floats + 9 floats = 1 + 15 floats

HEADER_STRUCT = struct.Struct('<BBHI')  # type, version, length, checksum (unsigned)
PID_RESPONSE_STRUCT = struct.Struct('<9fBB')  # 9 floats (3x3 PID) + 2 bytes

# Visualization configuration
PLOT_DIR = "autotuning_plots"
os.makedirs(PLOT_DIR, exist_ok=True)

# Data logging configuration
LOG_DIR = "autotuning_logs"
os.makedirs(LOG_DIR, exist_ok=True)
CSV_HEADER = [
    "timestamp_ms",
    "motor_state_0", "motor_state_1", "motor_state_2",
    "motor_setpoint_0", "motor_setpoint_1", "motor_setpoint_2",
    "error_0_k", "error_0_k1", "error_0_k2",
    "error_1_k", "error_1_k1", "error_1_k2",
    "error_2_k", "error_2_k1", "error_2_k2"
]

def build_sequences(X, seq_len=20):
    X_seq = []
    for i in range(len(X) - seq_len):
        X_seq.append(X[i:i+seq_len])
    return np.array(X_seq)

def evaluate_pid_performance(samples: np.ndarray, weight_oscillation=0.6, weight_steady_state=0.4) -> float:
    """
    Evaluate PID performance based on oscillation and steady-state error.
    Lower score is better.
    
    Args:
        samples: numpy array with shape (n_samples, 16) containing
                 [timestamp, motor_state[3], motor_setpoint[3], errors[3][3]]
        weight_oscillation: Weight for oscillation metric (default 0.6)
        weight_steady_state: Weight for steady-state error metric (default 0.4)
    
    Returns:
        score: Combined performance score (lower is better)
    """
    # Extract motor states and setpoints
    motor_state = samples[:, 1:4]      # Shape: (n_samples, 3)
    motor_setpoint = samples[:, 4:7]   # Shape: (n_samples, 3)
    
    # Calculate tracking errors
    errors = motor_state - motor_setpoint
    
    # Measure oscillation: use the variance of the derivative (rate of change)
    # Higher variance = more oscillation
    dt_errors = np.diff(errors, axis=0)
    oscillation_score = np.mean(np.var(dt_errors, axis=0))
    
    # Measure steady-state error: use the mean absolute error in the last 30% of samples
    # (assuming system should have settled by then)
    steady_state_window = int(len(samples) * 0.3)
    if steady_state_window < 10:
        steady_state_window = min(10, len(samples))
    
    steady_state_errors = errors[-steady_state_window:]
    steady_state_score = np.mean(np.abs(steady_state_errors))
    
    # Normalize scores (simple approach - you can improve this)
    # Oscillation typically ranges 0-10, steady-state 0-5
    oscillation_normalized = oscillation_score / 10.0
    steady_state_normalized = steady_state_score / 5.0
    
    # Combined score
    combined_score = (weight_oscillation * oscillation_normalized + 
                     weight_steady_state * steady_state_normalized)
    
    logger.info(f"Performance evaluation: oscillation={oscillation_score:.4f}, "
               f"steady_state={steady_state_score:.4f}, combined_score={combined_score:.4f}")
    
    return combined_score

def calculate_robot_trajectory(samples, wheel_radius=0.03, use_setpoint=False):
    """
    Calculate robot trajectory using 3-wheel omni kinematics
    
    Args:
        samples: numpy array with shape (n_samples, 16) containing
                 [timestamp, motor_state[3], motor_setpoint[3], errors[3][3]]
        wheel_radius: Wheel radius in meters (default 3cm = 0.03m)
        use_setpoint: If True, use setpoints; if False, use actual states
    
    Returns:
        x, y, theta: Robot position and orientation arrays
    """
    r = wheel_radius
    wheel_angles = np.array([0, 2*np.pi/3, 4*np.pi/3])  # 0°, 120°, 240°
    
    # Extract wheel velocities
    if use_setpoint:
        w0 = samples[:, 4]  # motor_setpoint_0
        w1 = samples[:, 5]  # motor_setpoint_1
        w2 = samples[:, 6]  # motor_setpoint_2
    else:
        w0 = samples[:, 1]  # motor_state_0
        w1 = samples[:, 2]  # motor_state_1
        w2 = samples[:, 3]  # motor_state_2
    
    # Time vector
    time = samples[:, 0] / 1000.0  # Convert ms to seconds
    dt = np.diff(time)
    dt = np.append(dt, dt[-1] if len(dt) > 0 else 0.01)
    
    # Initialize position and orientation
    x = np.zeros(len(samples))
    y = np.zeros(len(samples))
    theta = np.zeros(len(samples))
    
    # Forward kinematics
    for i in range(1, len(samples)):
        vx_robot = r * (w0[i] * np.cos(wheel_angles[0]) + 
                       w1[i] * np.cos(wheel_angles[1]) + 
                       w2[i] * np.cos(wheel_angles[2])) / 3.0
        
        vy_robot = r * (-w0[i] * np.sin(wheel_angles[0]) - 
                       w1[i] * np.sin(wheel_angles[1]) - 
                       w2[i] * np.sin(wheel_angles[2])) / 3.0
        
        omega = 0  # Simplified (no rotation tracking without wheel base info)
        
        # Transform to global frame
        # vx_global = vx_robot * np.cos(theta[i-1]) - vy_robot * np.sin(theta[i-1])
        # vy_global = vx_robot * np.sin(theta[i-1]) + vy_robot * np.cos(theta[i-1])
        vx_global = vx_robot * np.cos((-1.5*np.pi/6)+np.pi) - vy_robot * np.sin((-1.5*np.pi/6)+np.pi)
        vy_global = vx_robot * np.sin((-1.5*np.pi/6)+np.pi) + vy_robot * np.cos((-1.5*np.pi/6)+np.pi)
        
        x[i] = x[i-1] + vx_global * dt[i]
        y[i] = y[i-1] + vy_global * dt[i]
        theta[i] = theta[i-1] + omega * dt[i]
    
    return x, y, theta

def plot_comparison_graphs(samples_before, samples_after, pid_before, pid_after, timestamp):
    """
    Generate comparison plots for before/after PID tuning
    
    Args:
        samples_before: numpy array of samples before tuning
        samples_after: numpy array of samples after tuning
        pid_before: tuple of (Kp, Ki, Kd) before tuning
        pid_after: tuple of (Kp, Ki, Kd) after tuning
        timestamp: timestamp string for filename
    """
    logger.info("Generating comparison visualization...")
    
    # Create figure with subplots
    fig = plt.figure(figsize=(20, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    
    # Extract time vectors and normalize to start from 0
    time_before = (samples_before[:, 0] - samples_before[0, 0]) / 1000.0
    time_after = (samples_after[:, 0] - samples_after[0, 0]) / 1000.0
    
    motor_names = ['Right Wheel (Motor 0)', 'Left Wheel (Motor 1)', 'Back Wheel (Motor 2)']
    colors_before = ['#FF6B6B', '#4ECDC4', '#45B7D1']
    colors_after = ['#2ECC40', '#FF851B', '#B10DC9']
    
    # Plot motor tracking for each motor (left column)
    for motor_idx in range(3):
        ax = fig.add_subplot(gs[motor_idx, 0])
        
        # Before tuning
        state_before = samples_before[:, motor_idx + 1]
        setpoint_before = samples_before[:, motor_idx + 4]
        
        # After tuning
        state_after = samples_after[:, motor_idx + 1]
        setpoint_after = samples_after[:, motor_idx + 4]
        
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
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Velocity (rad/s)')
        ax.set_title(f'{motor_names[motor_idx]} - Tracking Performance')
        ax.legend(loc='best', fontsize=9)
        ax.grid(True, alpha=0.3)
    
    # Calculate trajectories
    x_desired_before, y_desired_before, _ = calculate_robot_trajectory(samples_before, use_setpoint=True)
    x_before, y_before, _ = calculate_robot_trajectory(samples_before, use_setpoint=False)
    
    x_desired_after, y_desired_after, _ = calculate_robot_trajectory(samples_after, use_setpoint=True)
    x_after, y_after, _ = calculate_robot_trajectory(samples_after, use_setpoint=False)
    
    # Plot trajectory comparison (right top)
    ax_traj = fig.add_subplot(gs[0:2, 1])
    
    # Plot both desired trajectories (they should be similar but may have different lengths)
    ax_traj.plot(x_desired_before, y_desired_before, 'k--', linewidth=2.5, alpha=0.8, 
                label='Desired Trajectory', zorder=5)
    ax_traj.plot(x_before, y_before, color='#FF6B6B', linewidth=2, 
                label='Before Tuning', alpha=0.7)
    ax_traj.plot(x_after, y_after, color='#2ECC40', linewidth=2, 
                label='After Tuning', alpha=0.7)
    
    # Mark start and end points
    ax_traj.scatter(x_desired_before[0], y_desired_before[0], color='blue', s=150, 
                   marker='o', label='Start', zorder=10, edgecolors='black', linewidths=2)
    ax_traj.scatter(x_desired_before[-1], y_desired_before[-1], color='red', s=150, 
                   marker='s', label='End (Desired)', zorder=10, edgecolors='black', linewidths=2)
    ax_traj.scatter(x_after[-1], y_after[-1], color='green', s=150, 
                   marker='^', label='End (After Tuning)', zorder=10, edgecolors='black', linewidths=2)
    
    ax_traj.set_xlabel('X Position (m)')
    ax_traj.set_ylabel('Y Position (m)')
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
                 label='Before Tuning', alpha=0.7)
    ax_error.fill_between(time_before, 0, error_before, color='#FF6B6B', alpha=0.2)
    
    ax_error.plot(time_after, error_after, color='#2ECC40', linewidth=2, 
                 label='After Tuning', alpha=0.7)
    ax_error.fill_between(time_after, 0, error_after, color='#2ECC40', alpha=0.2)
    
    ax_error.set_xlabel('Time (s)')
    ax_error.set_ylabel('Position Error (cm)')
    ax_error.set_title('Trajectory Tracking Error Over Time', fontsize=12, fontweight='bold')
    ax_error.legend(loc='best')
    ax_error.grid(True, alpha=0.3)
    
    # Add statistics text (calculate separately for each dataset)
    mean_before = np.mean(error_before)
    max_before = np.max(error_before)
    rmse_before = np.sqrt(np.mean(error_before**2))
    
    mean_after = np.mean(error_after)
    max_after = np.max(error_after)
    rmse_after = np.sqrt(np.mean(error_after**2))
    
    improvement = ((mean_before - mean_after) / mean_before) * 100 if mean_before > 0 else 0
    
    stats_text = f'BEFORE: Mean={mean_before:.2f}cm, Max={max_before:.2f}cm, RMSE={rmse_before:.2f}cm\n'
    stats_text += f'AFTER: Mean={mean_after:.2f}cm, Max={max_after:.2f}cm, RMSE={rmse_after:.2f}cm\n'
    stats_text += f'Improvement: {improvement:.1f}%\n'
    stats_text += f'Note: Before={len(samples_before)} samples, After={len(samples_after)} samples'
    
    ax_error.text(0.02, 0.98, stats_text, transform=ax_error.transAxes,
                 verticalalignment='top', fontsize=9,
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Add title with PID constants
    Kp_before, Ki_before, Kd_before = pid_before
    Kp_after, Ki_after, Kd_after = pid_after
    
    title_text = f'PID Auto-Tuning Results Comparison\n'
    title_text += f'Before: Kp={Kp_before}, Ki={Ki_before}, Kd={Kd_before}\n'
    title_text += f'After: Kp={Kp_after}, Ki={Ki_after}, Kd={Kd_after}'
    
    fig.suptitle(title_text, fontsize=14, fontweight='bold')
    
    # Save figure
    filename = os.path.join(PLOT_DIR, f'pid_tuning_comparison_{timestamp}.png')
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    logger.info(f"Saved comparison plot: {filename}")
    
    plt.close(fig)
    
    return filename

def build_sequences(X, seq_len=20):
    X_seq = []
    for i in range(len(X) - seq_len):
        X_seq.append(X[i:i+seq_len])
    return np.array(X_seq)

class DataLogger:
    """Handles logging of telemetry data to files"""
    
    def __init__(self, session_id: str, pid_constants: Optional[np.ndarray] = None, log_dir: str = LOG_DIR):
        self.log_dir = log_dir
        self.session_id = session_id
        self.pid_constants = pid_constants
        
        # Create log directory if it doesn't exist
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Format PID constants for filename if provided
        pid_suffix = ""
        if pid_constants is not None:
            Kp = pid_constants[0].tolist() if hasattr(pid_constants[0], 'tolist') else list(pid_constants[0])
            Ki = pid_constants[1].tolist() if hasattr(pid_constants[1], 'tolist') else list(pid_constants[1])
            Kd = pid_constants[2].tolist() if hasattr(pid_constants[2], 'tolist') else list(pid_constants[2])
            pid_suffix = f"_Kp{Kp}_Ki{Ki}_Kd{Kd}"
        
        # File paths
        self.csv_path = os.path.join(self.log_dir, f"autotuning_{self.session_id}{pid_suffix}.csv")
        self.raw_path = os.path.join(self.log_dir, f"autotuning_{self.session_id}{pid_suffix}_raw.bin")
        self.meta_path = os.path.join(self.log_dir, f"autotuning_{self.session_id}{pid_suffix}_meta.json")
        
        # Statistics
        self.total_samples = 0
        self.session_start = datetime.now()
        
        # Initialize CSV file
        self._init_csv()
        
        # Initialize metadata
        self._init_metadata()
        
        logger.info(f"Data logger initialized. Session ID: {self.session_id}")
        logger.info(f"CSV log: {self.csv_path}")
    
    def _init_csv(self):
        """Initialize CSV file with header"""
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)
        logger.info(f"Created CSV file: {self.csv_path}")
    
    def _init_metadata(self):
        """Initialize metadata file"""
        metadata = {
            "session_id": self.session_id,
            "start_time": self.session_start.isoformat(),
            "protocol_version": PROTOCOL_VERSION,
            "total_samples": 0,
            "csv_file": os.path.basename(self.csv_path),
            "raw_file": os.path.basename(self.raw_path),
            "pid_constants": self.pid_constants.tolist() if self.pid_constants is not None else None
        }
        self._write_metadata(metadata)
    
    def _write_metadata(self, metadata: dict):
        """Write metadata to JSON file"""
        with open(self.meta_path, 'w') as f:
            json.dump(metadata, f, indent=2)
    
    def log_samples(self, samples: np.ndarray):
        """
        Log samples to CSV and raw binary files
        
        Args:
            samples: Numpy array of samples (n_samples, 16)
        """
        # Write to CSV
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            for sample in samples:
                writer.writerow(sample)
        
        # Write to raw binary (for exact reproduction)
        with open(self.raw_path, 'ab') as f:
            for sample in samples:
                # Pack sample data using the struct format
                # Convert timestamp to int, keep rest as floats
                timestamp = int(sample[0])
                float_values = [float(x) for x in sample[1:]]
                packed = ROBOT_SAMPLE_STRUCT.pack(timestamp, *float_values)
                f.write(packed)
        
        # Update statistics
        self.total_samples += len(samples)
        
        logger.info(f"Logged {len(samples)} samples (total: {self.total_samples})")
    
    def finalize(self):
        """Finalize logging session and update metadata"""
        metadata = {
            "session_id": self.session_id,
            "start_time": self.session_start.isoformat(),
            "end_time": datetime.now().isoformat(),
            "duration_seconds": (datetime.now() - self.session_start).total_seconds(),
            "protocol_version": PROTOCOL_VERSION,
            "total_samples": self.total_samples,
            "csv_file": os.path.basename(self.csv_path),
            "raw_file": os.path.basename(self.raw_path),
            "pid_constants": self.pid_constants.tolist() if self.pid_constants is not None else None
        }
        self._write_metadata(metadata)
        logger.info(f"Session finalized: {self.total_samples} samples")

class PIDAutotuner:
    """RNN-based PID autotuner with performance tracking"""
    
    def __init__(self):
        self.iteration = 0
        self.converged = False
        
        # Track all iterations for comparison
        self.iteration_history = []  # List of dicts with {pid_constants, samples, score}
        self.best_iteration = None
        self.best_score = float('inf')
        
        logger.info("PID Autotuner initialized with performance tracking")
        
        # Load trained RNN model
        custom_objects = {
            'mse': losses.mean_squared_error
        }

        self.model = keras.models.load_model(
            model_path,
            custom_objects=custom_objects
        )
    
    def predict_pid(self, samples: np.ndarray) -> Tuple[np.ndarray, bool]:
        """
        Run RNN inference to predict PID constants
        
        Args:
            samples: Robot samples array shape (n_samples, 16)
                Format from ESP32: [timestamp, motor_state[3], motor_setpoint[3], errors[3][3]]
                errors[3][3] is organized as: [motor0_e0, motor0_e1, motor0_e2, motor1_e0, ...]
        
        Returns:
            pid_constants: Array of shape (3, 3) for [Kp, Ki, Kd] x 3 motors
            converged: Whether tuning has converged
        """
        logger.info(f"Input samples shape: {samples.shape}")
        
        # Extract components (skip timestamp column 0)
        # Columns: 0=timestamp, 1-3=motor_state, 4-6=motor_setpoint, 7-15=errors[3][3]
        motor_state = samples[:, 1:4]      # Shape: (n_samples, 3)
        motor_setpoint = samples[:, 4:7]   # Shape: (n_samples, 3)
        errors_flat = samples[:, 7:16]     # Shape: (n_samples, 9)
        
        # Reshape errors from [motor0_all, motor1_all, motor2_all] to [all_motors_t0, all_motors_t1, all_motors_t2]
        # errors_flat is [e0_k, e0_k1, e0_k2, e1_k, e1_k1, e1_k2, e2_k, e2_k1, e2_k2]
        # We need: [e0_k, e1_k, e2_k, e0_k1, e1_k1, e2_k1, e0_k2, e1_k2, e2_k2]
        errors_reshaped = errors_flat.reshape(-1, 3, 3)  # (n_samples, 3_motors, 3_times)
        errors_transposed = errors_reshaped.transpose(0, 2, 1)  # (n_samples, 3_times, 3_motors)
        errors_reordered = errors_transposed.reshape(-1, 9)  # (n_samples, 9)
        
        # Construct features matching training format: [state, setpoint, e_k, e_k1, e_k2]
        features = np.concatenate([motor_state, motor_setpoint, errors_reordered], axis=1)
        logger.info(f"Features shape after reordering: {features.shape} (expected: (n_samples, 15))")
        
        X_all = []
        X_seq = build_sequences(features, 20)
        logger.info(f"Sequence shape after build_sequences: {X_seq.shape}")
        X_all.append(X_seq)
        X_all = np.concatenate(X_all, axis=0)
        logger.info(f"Final input shape for model: {X_all.shape}")

        y_pred = self.model.predict(X_all, verbose=1)

        mean_gains = np.mean(y_pred, axis=0)
        logger.info(np.round(mean_gains, 2))
        
        self.iteration += 1
        
        # RNN inference
        logger.info(f"Running RNN inference on {len(samples)} samples")
        
        logger.info(f'Kp = np.array([{mean_gains[0]}, {mean_gains[1]}, {mean_gains[2]}])')
        logger.info(f'Ki = np.array([{mean_gains[3]}, {mean_gains[4]}, {mean_gains[5]}])')
        logger.info(f'Kd = np.array([{mean_gains[6]}, {mean_gains[7]}, {mean_gains[8]}])')
        pid_constants = np.array([
            [mean_gains[0], mean_gains[3], mean_gains[6]],
            [mean_gains[1], mean_gains[4], mean_gains[7]],
            [mean_gains[2], mean_gains[5], mean_gains[8]]
        ])
        
        # Evaluate performance of these PID constants
        performance_score = evaluate_pid_performance(samples)
        
        # Store this iteration
        self.iteration_history.append({
            'iteration': self.iteration,
            'pid_constants': pid_constants.copy(),
            'samples': samples.copy(),
            'score': performance_score
        })
        
        # Check if this is the best iteration so far
        if performance_score < self.best_score:
            self.best_score = performance_score
            self.best_iteration = self.iteration
            logger.info(f"⭐ NEW BEST! Iteration {self.iteration} with score {performance_score:.4f}")
        else:
            logger.info(f"Iteration {self.iteration} score {performance_score:.4f} "
                       f"(best is still iteration {self.best_iteration} with {self.best_score:.4f})")
        
        # Check convergence
        converged = self.iteration >= RUNS_TO_CONVERGE
        
        logger.info(f"Iteration {self.iteration}: Kp={pid_constants[0]}, "
                   f"Ki={pid_constants[1]}, Kd={pid_constants[2]}")
        
        return pid_constants, converged
    
    def get_best_pid_constants(self) -> np.ndarray:
        """
        Get the best PID constants from all iterations
        
        Returns:
            pid_constants: Best PID constants array of shape (3, 3)
        """
        if self.best_iteration is None or len(self.iteration_history) == 0:
            logger.warning("No iterations recorded yet, returning default values")
            return np.array([[4, 4, 4], [0, 0, 0], [0, 0, 0]])
        
        best_iter_data = self.iteration_history[self.best_iteration - 1]
        logger.info(f"Returning BEST PID constants from iteration {self.best_iteration} "
                   f"with score {best_iter_data['score']:.4f}")
        
        return best_iter_data['pid_constants']
    
    def get_best_iteration_data(self):
        """Get the complete data from the best iteration"""
        if self.best_iteration is None or len(self.iteration_history) == 0:
            return None
        return self.iteration_history[self.best_iteration - 1]
    
    def reset(self):
        """Reset the autotuner for a new session"""
        self.iteration = 0
        self.converged = False
        self.iteration_history = []
        self.best_iteration = None
        self.best_score = float('inf')
        logger.info("PID Autotuner reset for new session")


class TelemetryServer:
    """Server to handle robot telemetry and PID autotuning"""
    
    def __init__(self, port: int = PORT):
        self.port = port
        self.autotuner = PIDAutotuner()
        self.server_socket = None
        
        # Accumulator for data batches (new for inference request workflow)
        self.accumulated_samples = []
        
        # Track data for visualization
        self.samples_before_tuning = None
        self.samples_best_tuning = None  # Store samples from best iteration
        self.samples_after_tuning = None
        self.pid_constants_before = None
        self.pid_constants_after = None
        self.tuning_phase = "before"  # "before", "tuning", "after"
        self.session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Data loggers for before and after tuning
        self.logger_before = None
        self.logger_after = None
        
        logger.info("Telemetry server initialized with data accumulation and visualization support")
    
    def calculate_crc32(self, data: bytes) -> int:
        """Calculate CRC32 checksum"""
        return zlib.crc32(data) & 0xFFFFFFFF
    
    def parse_header(self, data: bytes) -> Tuple[int, int, int, int]:
        """Parse message header"""
        msg_type, version, length, checksum = HEADER_STRUCT.unpack(data)
        return msg_type, version, length, checksum
    
    def parse_data_batch(self, data: bytes) -> np.ndarray:
        """Parse data batch into numpy array"""
        # First 4 bytes: sample_count (uint16) + sequence_number (uint16)
        sample_count, seq_num = struct.unpack('<HH', data[:4])
        logger.info(f"Received batch: {sample_count} samples, seq={seq_num}")
        
        # Parse samples
        offset = 4
        samples = []
        
        for i in range(sample_count):
            sample_data = data[offset:offset + ROBOT_SAMPLE_STRUCT.size]
            sample = ROBOT_SAMPLE_STRUCT.unpack(sample_data)
            samples.append(sample)
            offset += ROBOT_SAMPLE_STRUCT.size
        
        # Convert to numpy array
        samples_array = np.array(samples)
        logger.info(f"Parsed samples: {samples_array}")
        
        return samples_array
    
    def create_pid_response(self, pid_constants: np.ndarray, 
                           iteration: int, converged: bool) -> bytes:
        """Create PID response message"""
        # Flatten PID constants (Kp0, Kp1, Kp2, Ki0, Ki1, Ki2, Kd0, Kd1, Kd2)
        pid_flat = pid_constants.T.flatten()
        
        # Pack payload
        payload = PID_RESPONSE_STRUCT.pack(
            *pid_flat, iteration, 1 if converged else 0
        )
        
        # Calculate checksum
        checksum = self.calculate_crc32(payload)
        
        # Create header
        header = HEADER_STRUCT.pack(
            MSG_TYPE_PID_RESPONSE,
            PROTOCOL_VERSION,
            len(payload),
            checksum
        )
        
        return header + payload
    
    def handle_client(self, client_socket: socket.socket, address: str):
        """Handle a single client connection"""
        logger.info(f"Client connected from {address}")
        
        try:
            while True:
                # Receive header
                header_data = client_socket.recv(HEADER_STRUCT.size)
                
                if not header_data:
                    logger.info("Client disconnected (no data)")
                    break
                    
                if len(header_data) < HEADER_STRUCT.size:
                    logger.warning(f"Incomplete header received: {len(header_data)} bytes, expected {HEADER_STRUCT.size}")
                    logger.debug(f"Header data: {header_data.hex()}")
                    break

                msg_type, version, length, checksum = self.parse_header(header_data)
                logger.info(f"Header: type={msg_type}, version={version}, "
                           f"length={length}, checksum={checksum:08X}")
                
                # Handle different message types
                if msg_type == MSG_TYPE_DATA_BATCH:
                    # Data batch - accumulate samples
                    logger.debug(f"Received DATA_BATCH message")
                    
                    # Validate protocol version
                    if version != PROTOCOL_VERSION:
                        logger.warning(f"Protocol version mismatch: {version} (expected {PROTOCOL_VERSION})")
                    
                    # Receive payload
                    payload = b''
                    while len(payload) < length:
                        chunk = client_socket.recv(min(4096, length - len(payload)))
                        if not chunk:
                            raise ConnectionError("Connection lost during payload receive")
                        payload += chunk
                    
                    logger.debug(f"Received {len(payload)} bytes payload")
                    
                    # Verify checksum
                    calc_checksum = self.calculate_crc32(payload)
                    if calc_checksum != checksum:
                        logger.error(f"Checksum mismatch: calculated={calc_checksum:08X}, received={checksum:08X}")
                        continue
                    
                    # Parse and accumulate data
                    samples = self.parse_data_batch(payload)
                    self.accumulated_samples.append(samples)
                    logger.info(f"Accumulated batch {len(self.accumulated_samples)} "
                               f"({samples.shape[0]} samples, total: {sum(s.shape[0] for s in self.accumulated_samples)} samples)")
                    
                    # No immediate response - just accumulate
                    continue
                
                elif msg_type == MSG_TYPE_INFERENCE_REQUEST:
                    # Inference request - process all accumulated data
                    logger.info(f"Received INFERENCE_REQUEST - processing {len(self.accumulated_samples)} batches")
                    logger.info(f"Current tuning phase: {self.tuning_phase}")
                    
                    if len(self.accumulated_samples) == 0:
                        logger.warning("Inference requested but no data accumulated!")
                        # Send error or previous PID values
                        continue
                    
                    # Concatenate all accumulated samples
                    all_samples = np.vstack(self.accumulated_samples)
                    logger.info(f"Running inference on {all_samples.shape[0]} total samples")
                    
                    # Store samples based on current phase
                    if self.tuning_phase == "before":
                        # First data batch - before tuning
                        logger.info("Storing BEFORE tuning data")
                        if self.autotuner.iteration == 0:
                            self.samples_before_tuning = all_samples.copy()
                            
                            # Initialize data logger for "before" phase
                            self.pid_constants_before = np.array([
                                [4, 4, 4],  # Initial Kp
                                [0, 0, 0],  # Initial Ki
                                [0, 0, 0]   # Initial Kd
                            ])
                            self.logger_before = DataLogger(
                                session_id=f"{self.session_timestamp}_before",
                                pid_constants=self.pid_constants_before
                            )
                            self.logger_before.log_samples(all_samples)
                            self.logger_before.finalize()
                        
                        # Run inference to get new PID constants
                        pid_constants, converged = self.autotuner.predict_pid(all_samples)
                        
                        # Send new PID response (current iteration result)
                        response = self.create_pid_response(
                            pid_constants, 
                            self.autotuner.iteration, 
                            converged
                        )
                        client_socket.sendall(response)
                        logger.info(f"Sent PID response: iteration={self.autotuner.iteration}, converged={converged}")
                        
                        # Move to after phase only on the last inference iteration
                        if self.autotuner.iteration == RUNS_TO_CONVERGE-1:
                            # Get the BEST PID constants from all iterations
                            self.pid_constants_after = self.autotuner.get_best_pid_constants()
                            best_iter_data = self.autotuner.get_best_iteration_data()
                            
                            logger.info(f"🏆 Selected BEST iteration {best_iter_data['iteration']} "
                                       f"with score {best_iter_data['score']:.4f}")
                            logger.info(f"Best PID: Kp={self.pid_constants_after[0]}, "
                                       f"Ki={self.pid_constants_after[1]}, "
                                       f"Kd={self.pid_constants_after[2]}")
                            
                            # Store the best iteration's samples for plotting
                            self.samples_best_tuning = best_iter_data['samples'].copy()
                            
                            self.tuning_phase = "after"
                            logger.info("Phase changed to 'after' - waiting for post-tuning data...")
                        else:
                            logger.info(f"Continuing inference phase (iteration {self.autotuner.iteration}/{RUNS_TO_CONVERGE-1})")
                        
                        self.accumulated_samples = []
                        
                    elif self.tuning_phase == "after":
                        # Second data batch - after tuning
                        logger.info("Storing AFTER tuning data")
                        self.samples_after_tuning = all_samples.copy()
                        
                        # Initialize data logger for "after" phase
                        self.logger_after = DataLogger(
                            session_id=f"{self.session_timestamp}_after",
                            pid_constants=self.pid_constants_after
                        )
                        self.logger_after.log_samples(all_samples)
                        self.logger_after.finalize()
                        
                        # Generate comparison plots
                        if self.samples_before_tuning is not None:
                            try:
                                # Use the best iteration samples for comparison
                                plot_file = plot_comparison_graphs(
                                    self.samples_before_tuning,
                                    self.samples_after_tuning,
                                    (self.pid_constants_before[0], 
                                     self.pid_constants_before[1], 
                                     self.pid_constants_before[2]),
                                    (self.pid_constants_after[0], 
                                     self.pid_constants_after[1], 
                                     self.pid_constants_after[2]),
                                    self.session_timestamp
                                )
                                logger.info(f"✓ Generated comparison plot: {plot_file}")
                            except Exception as e:
                                logger.error(f"Failed to generate plots: {e}", exc_info=True)
                        
                        # Send acknowledgment with BEST PID constants
                        response = self.create_pid_response(
                            self.pid_constants_after, 
                            self.autotuner.iteration, 
                            True  # Mark as converged/complete
                        )
                        client_socket.sendall(response)
                        logger.info("Sent final acknowledgment with BEST PID constants")
                        
                        # Reset for next session
                        self.tuning_phase = "before"
                        self.accumulated_samples = []
                        self.session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        self.logger_before = None
                        self.logger_after = None
                        self.autotuner.reset()  # Reset autotuner for next session
                        logger.info("Session complete - reset for next tuning cycle")
                    
                    continue
                
                else:
                    # Unknown message type
                    logger.warning(f"Unexpected message type: {msg_type}")
                    logger.debug(f"Raw header: {header_data.hex()}")
                    # Try to recover by reading and discarding payload if length is reasonable
                    if 0 < length < 65536:
                        discarded = client_socket.recv(length)
                        logger.debug(f"Discarded {len(discarded)} bytes of unexpected payload")
                    continue
        
        except Exception as e:
            logger.error(f"Error handling client: {e}")
        finally:
            client_socket.close()
            logger.info(f"Connection closed: {address}")
    
    def start(self):
        """Start the server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(5)
        
        logger.info(f"Server listening on port {self.port}")
        
        try:
            while True:
                client_socket, address = self.server_socket.accept()
                logger.info(f"New connection from {address}")
                
                # Handle client (single-threaded for simplicity)
                # For multiple robots, use threading.Thread
                self.handle_client(client_socket, address)
        
        except KeyboardInterrupt:
            logger.info("Server shutting down...")
        finally:
            if self.server_socket:
                self.server_socket.close()


if __name__ == "__main__":
    server = TelemetryServer()
    server.start()