#!/usr/bin/env python3
"""
Telemetry Data Analysis Script
Analyzes and visualizes logged telemetry data
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from pathlib import Path
import json

def load_session_data(csv_path):
    """Load telemetry data from CSV file"""
    print(f"Loading data from: {csv_path}")
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} samples")
    return df

def load_metadata(meta_path):
    """Load session metadata"""
    if os.path.exists(meta_path):
        with open(meta_path, 'r') as f:
            return json.load(f)
    return None

def plot_individual_motor_tracking(df, output_dir=None):
    """Plot individual motor tracking: state vs setpoint for each wheel"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    motor_names = ['Right Wheel (Motor 0)', 'Left Wheel (Motor 1)', 'Back Wheel (Motor 2)']
    
    for i, (ax, name) in enumerate(zip(axes, motor_names)):
        time = df['timestamp_ms'] / 1000
        actual = df[f'motor_state_{i}']
        setpoint = df[f'motor_setpoint_{i}']
        
        ax.plot(time, actual, label='Actual', alpha=0.8, linewidth=1.5, color='C0')
        ax.plot(time, setpoint, label='Setpoint', alpha=0.7, linewidth=1.5, 
                linestyle='--', color='C1')
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Velocity (rad/s)')
        ax.set_title(name)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Add shaded error region
        error = setpoint - actual
        ax.fill_between(time, setpoint, actual, alpha=0.2, color='red', 
                        label='Error region')
    
    plt.tight_layout()
    
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'individual_motor_tracking.png'), dpi=150)
        print(f"Saved: individual_motor_tracking.png")
    
    plt.show()

def plot_errors(df, output_dir=None):
    """Plot PID errors over time for all motors"""
    plt.figure(figsize=(14, 10))
    
    # Motor 0 errors
    plt.subplot(3, 1, 1)
    plt.plot(df['timestamp_ms'] / 1000, df['error_0_k'], label='Error(k)', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, df['error_0_k1'], label='Error(k-1)', alpha=0.5)
    plt.plot(df['timestamp_ms'] / 1000, df['error_0_k2'], label='Error(k-2)', alpha=0.3)
    plt.xlabel('Time (s)')
    plt.ylabel('Error')
    plt.title('Motor 0: PID Error History')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Motor 1 errors
    plt.subplot(3, 1, 2)
    plt.plot(df['timestamp_ms'] / 1000, df['error_1_k'], label='Error(k)', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, df['error_1_k1'], label='Error(k-1)', alpha=0.5)
    plt.plot(df['timestamp_ms'] / 1000, df['error_1_k2'], label='Error(k-2)', alpha=0.3)
    plt.xlabel('Time (s)')
    plt.ylabel('Error')
    plt.title('Motor 1: PID Error History')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Motor 2 errors
    plt.subplot(3, 1, 3)
    plt.plot(df['timestamp_ms'] / 1000, df['error_2_k'], label='Error(k)', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, df['error_2_k1'], label='Error(k-1)', alpha=0.5)
    plt.plot(df['timestamp_ms'] / 1000, df['error_2_k2'], label='Error(k-2)', alpha=0.3)
    plt.xlabel('Time (s)')
    plt.ylabel('Error')
    plt.title('Motor 2: PID Error History')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'pid_errors.png'), dpi=150)
        print(f"Saved: pid_errors.png")
    
    plt.show()

def plot_error_magnitude(df, output_dir=None):
    """Plot overall error magnitude for each motor"""
    plt.figure(figsize=(12, 6))
    
    # Calculate error magnitudes (absolute values of current error)
    error_mag_0 = np.abs(df['error_0_k'])
    error_mag_1 = np.abs(df['error_1_k'])
    error_mag_2 = np.abs(df['error_2_k'])
    
    plt.plot(df['timestamp_ms'] / 1000, error_mag_0, label='Motor 0', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, error_mag_1, label='Motor 1', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, error_mag_2, label='Motor 2', alpha=0.7)
    
    plt.xlabel('Time (s)')
    plt.ylabel('|Error|')
    plt.title('PID Error Magnitude (Absolute Values)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'error_magnitude.png'), dpi=150)
        print(f"Saved: error_magnitude.png")
    
    plt.show()

def calculate_robot_trajectory(df, wheel_radius=0.03, use_setpoint=False):
    """
    Calculate robot trajectory using 3-wheel omni kinematics
    
    Args:
        df: DataFrame with motor states/setpoints
        wheel_radius: Wheel radius in meters (default 3cm = 0.03m)
        use_setpoint: If True, use setpoints; if False, use actual states
    
    Returns:
        x, y, theta: Robot position and orientation arrays
    
    Three-wheel omni robot kinematics:
    - Wheels are arranged at 120° intervals (0°, 120°, 240°)
    - Motor 0 (Right): 0° 
    - Motor 1 (Left): 120°
    - Motor 2 (Back): 240°
    
    Forward kinematics from wheel velocities to robot velocity:
    [vx]     [  cos(0°)    cos(120°)   cos(240°)  ]   [w0]
    [vy]  = r[ -sin(0°)   -sin(120°)  -sin(240°)  ] * [w1]
    [wz]     [   1/L         1/L          1/L      ]   [w2]
    
    Where:
    - r = wheel radius
    - L = distance from wheel to robot center
    - w0, w1, w2 = wheel angular velocities
    """
    
    # Robot configuration
    r = wheel_radius  # Wheel radius in meters
    
    # Wheel angles (in radians)
    # Motor 0 (Right wheel): 0° (pointing right)
    # Motor 1 (Left wheel): 120° (pointing left-forward)
    # Motor 2 (Back wheel): 240° (pointing left-backward)
    wheel_angles = np.array([0, 2*np.pi/3, 4*np.pi/3])  # 0°, 120°, 240°
    
    # Get wheel velocities
    suffix = 'setpoint' if use_setpoint else 'state'
    w0 = df[f'motor_{suffix}_0'].values  # Right wheel
    w1 = df[f'motor_{suffix}_1'].values  # Left wheel
    w2 = df[f'motor_{suffix}_2'].values  # Back wheel
    
    # Time vector
    time = df['timestamp_ms'].values / 1000.0  # Convert to seconds
    dt = np.diff(time)
    dt = np.append(dt, dt[-1])  # Extend to match length
    
    # Initialize position and orientation
    x = np.zeros(len(df))
    y = np.zeros(len(df))
    theta = np.zeros(len(df))
    
    # Forward kinematics matrix (simplified, assuming L is normalized)
    # For a proper omni robot: vx = r * (w0*cos(0) + w1*cos(120) + w2*cos(240)) / 3
    #                          vy = r * (-w0*sin(0) - w1*sin(120) - w2*sin(240)) / 3
    
    for i in range(1, len(df)):
        # Calculate robot velocities in robot frame
        vx_robot = r * (w0[i] * np.cos(wheel_angles[0]) + 
                       w1[i] * np.cos(wheel_angles[1]) + 
                       w2[i] * np.cos(wheel_angles[2])) / 3.0
        
        vy_robot = r * (-w0[i] * np.sin(wheel_angles[0]) - 
                       w1[i] * np.sin(wheel_angles[1]) - 
                       w2[i] * np.sin(wheel_angles[2])) / 3.0
        
        # For rotation, assuming all wheels contribute equally (simplified)
        # In a real system, this depends on the wheel base radius L
        omega = 0  # No rotation info without knowing L, keep heading constant
        
        # Transform to global frame
        vx_global = vx_robot * np.cos(theta[i-1]) - vy_robot * np.sin(theta[i-1])
        vy_global = vx_robot * np.sin(theta[i-1]) + vy_robot * np.cos(theta[i-1])
        
        # Integrate position
        x[i] = x[i-1] + vx_global * dt[i]
        y[i] = y[i-1] + vy_global * dt[i]
        theta[i] = theta[i-1] + omega * dt[i]
    
    return x, y, theta

def plot_robot_trajectory(df, output_dir=None):
    """Plot robot trajectory (desired vs actual)"""
    wheel_radius = 0.03  # 3cm wheels
    
    # Calculate desired trajectory (from setpoints)
    x_desired, y_desired, theta_desired = calculate_robot_trajectory(df, wheel_radius, use_setpoint=True)
    
    # Calculate actual trajectory (from states)
    x_actual, y_actual, theta_actual = calculate_robot_trajectory(df, wheel_radius, use_setpoint=False)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))
    
    # Plot 1: Trajectory comparison
    time = df['timestamp_ms'] / 1000
    
    # Desired path
    ax1.plot(x_desired, y_desired, 'b--', alpha=0.7, linewidth=2, label='Desired Path')
    ax1.scatter(x_desired[0], y_desired[0], color='blue', s=100, marker='o', 
               label='Start (Desired)', zorder=5)
    ax1.scatter(x_desired[-1], y_desired[-1], color='blue', s=100, marker='s', 
               label='End (Desired)', zorder=5)
    
    # Actual path
    sc = ax1.scatter(x_actual, y_actual, c=time, cmap='viridis', s=10, alpha=0.6, 
                    label='Actual Path', zorder=3)
    ax1.scatter(x_actual[0], y_actual[0], color='green', s=100, marker='o', 
               label='Start (Actual)', zorder=5)
    ax1.scatter(x_actual[-1], y_actual[-1], color='red', s=100, marker='s', 
               label='End (Actual)', zorder=5)
    
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Robot Trajectory: Desired vs Actual')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    plt.colorbar(sc, ax=ax1, label='Time (s)')
    
    # Plot 2: Trajectory error over time
    position_error = np.sqrt((x_desired - x_actual)**2 + (y_desired - y_actual)**2)
    
    ax2.plot(time, position_error * 100, linewidth=1.5)  # Convert to cm
    ax2.fill_between(time, 0, position_error * 100, alpha=0.3)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position Error (cm)')
    ax2.set_title('Trajectory Tracking Error')
    ax2.grid(True, alpha=0.3)
    
    # Add statistics text
    mean_error = np.mean(position_error) * 100
    max_error = np.max(position_error) * 100
    final_error = position_error[-1] * 100
    
    stats_text = f'Mean Error: {mean_error:.2f} cm\n'
    stats_text += f'Max Error: {max_error:.2f} cm\n'
    stats_text += f'Final Error: {final_error:.2f} cm'
    
    ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'robot_trajectory.png'), dpi=150)
        print(f"Saved: robot_trajectory.png")
    
    plt.show()

def calculate_tracking_error(df):
    """Calculate tracking error statistics from recorded errors"""
    errors = []
    for i in range(3):
        error = df[f'error_{i}_k']  # Current error values
        mae = np.mean(np.abs(error))
        rmse = np.sqrt(np.mean(error**2))
        errors.append({
            'motor': i,
            'mae': mae,
            'rmse': rmse,
            'max_error': np.max(np.abs(error)),
            'mean_error': np.mean(error),
            'std_error': np.std(error)
        })
    
    return errors

def print_statistics(df, metadata=None):
    """Print summary statistics"""
    print("\n" + "="*60)
    print("TELEMETRY DATA ANALYSIS")
    print("="*60)
    
    if metadata:
        print("\nSession Information:")
        print(f"  Session ID: {metadata.get('session_id', 'N/A')}")
        print(f"  Start Time: {metadata.get('start_time', 'N/A')}")
        print(f"  End Time: {metadata.get('end_time', 'N/A')}")
        print(f"  Duration: {metadata.get('duration_seconds', 0):.2f} seconds")
        print(f"  Total Samples: {metadata.get('total_samples', 0)}")
        print(f"  Total Batches: {metadata.get('total_batches', 0)}")
    
    print(f"\nData Overview:")
    print(f"  Total samples: {len(df)}")
    print(f"  Time range: {df['timestamp_ms'].min()/1000:.2f}s - {df['timestamp_ms'].max()/1000:.2f}s")
    print(f"  Duration: {(df['timestamp_ms'].max() - df['timestamp_ms'].min())/1000:.2f}s")
    
    print(f"\nMotor States (avg ± std):")
    for i in range(3):
        state = df[f'motor_state_{i}']
        print(f"  Motor {i}: {state.mean():.2f} ± {state.std():.2f}")
    
    print(f"\nMotor Setpoints (avg ± std):")
    for i in range(3):
        setpoint = df[f'motor_setpoint_{i}']
        print(f"  Motor {i}: {setpoint.mean():.2f} ± {setpoint.std():.2f}")
    
    print(f"\nPID Error Statistics:")
    errors = calculate_tracking_error(df)
    for err in errors:
        print(f"  Motor {err['motor']}:")
        print(f"    Mean Error:  {err['mean_error']:+.4f}")
        print(f"    Std Dev:     {err['std_error']:.4f}")
        print(f"    MAE:         {err['mae']:.4f}")
        print(f"    RMSE:        {err['rmse']:.4f}")
        print(f"    Max Error:   {err['max_error']:.4f}")
    
    print("="*60 + "\n")

def main():
    """Main analysis function"""
    # Check command line arguments
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_telemetry.py <csv_file>")
        print("\nExample:")
        print("  python3 analyze_telemetry.py telemetry_logs/telemetry_20251210_143025.csv")
        
        # Try to find the most recent log file
        log_dir = Path("telemetry_logs")
        if log_dir.exists():
            csv_files = sorted(log_dir.glob("telemetry_*[!_raw].csv"))
            if csv_files:
                print(f"\nMost recent log file found:")
                print(f"  {csv_files[-1]}")
                response = input("\nAnalyze this file? [Y/n]: ")
                if response.lower() != 'n':
                    csv_path = csv_files[-1]
                else:
                    return
            else:
                print("\nNo log files found in telemetry_logs/")
                return
        else:
            print("\nNo telemetry_logs/ directory found.")
            return
    else:
        csv_path = Path(sys.argv[1])
    
    # Check if file exists
    if not csv_path.exists():
        print(f"Error: File not found: {csv_path}")
        return
    
    # Load data
    df = load_session_data(csv_path)
    
    # Load metadata if available
    meta_path = csv_path.parent / csv_path.name.replace('.csv', '_meta.json')
    metadata = load_metadata(meta_path)
    
    # Print statistics
    print_statistics(df, metadata)
    
    # Create output directory for plots
    output_dir = csv_path.parent / f"analysis_{csv_path.stem}"
    output_dir.mkdir(exist_ok=True)
    print(f"Saving plots to: {output_dir}\n")
    
    # Generate plots
    print("Generating plots...")
    print("  1. Individual motor tracking...")
    plot_individual_motor_tracking(df, output_dir)
    print("  2. Robot trajectory...")
    plot_robot_trajectory(df, output_dir)
    print("  3. PID errors...")
    plot_errors(df, output_dir)
    print("  4. Error magnitude...")
    plot_error_magnitude(df, output_dir)
    
    print(f"\nAnalysis complete! Plots saved to: {output_dir}")

if __name__ == "__main__":
    main()
