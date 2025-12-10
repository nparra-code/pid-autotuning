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

def plot_motor_speeds(df, output_dir=None):
    """Plot motor speeds over time"""
    plt.figure(figsize=(14, 6))
    
    # Plot actual speeds
    plt.subplot(2, 1, 1)
    plt.plot(df['timestamp_ms'] / 1000, df['motor_speed_0'], label='Motor 0', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, df['motor_speed_1'], label='Motor 1', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, df['motor_speed_2'], label='Motor 2', alpha=0.7)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (units)')
    plt.title('Motor Speeds')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot target vs actual for motor 0
    plt.subplot(2, 1, 2)
    plt.plot(df['timestamp_ms'] / 1000, df['motor_speed_0'], label='Actual', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, df['target_speed_0'], label='Target', alpha=0.7, linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (units)')
    plt.title('Motor 0: Target vs Actual Speed')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'motor_speeds.png'), dpi=150)
        print(f"Saved: motor_speeds.png")
    
    plt.show()

def plot_motor_currents(df, output_dir=None):
    """Plot motor currents over time"""
    plt.figure(figsize=(12, 6))
    
    plt.plot(df['timestamp_ms'] / 1000, df['motor_current_0'], label='Motor 0', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, df['motor_current_1'], label='Motor 1', alpha=0.7)
    plt.plot(df['timestamp_ms'] / 1000, df['motor_current_2'], label='Motor 2', alpha=0.7)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Current (A)')
    plt.title('Motor Currents')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'motor_currents.png'), dpi=150)
        print(f"Saved: motor_currents.png")
    
    plt.show()

def plot_trajectory(df, output_dir=None):
    """Plot robot trajectory"""
    plt.figure(figsize=(10, 10))
    
    # Color by time
    scatter = plt.scatter(df['position_x'], df['position_y'], 
                         c=df['timestamp_ms'] / 1000, 
                         cmap='viridis', 
                         s=2, 
                         alpha=0.6)
    
    # Mark start and end
    plt.plot(df['position_x'].iloc[0], df['position_y'].iloc[0], 
             'go', markersize=10, label='Start')
    plt.plot(df['position_x'].iloc[-1], df['position_y'].iloc[-1], 
             'ro', markersize=10, label='End')
    
    plt.colorbar(scatter, label='Time (s)')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Robot Trajectory')
    plt.legend()
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'trajectory.png'), dpi=150)
        print(f"Saved: trajectory.png")
    
    plt.show()

def plot_heading(df, output_dir=None):
    """Plot robot heading over time"""
    plt.figure(figsize=(12, 6))
    
    # Convert heading from radians to degrees
    heading_deg = np.rad2deg(df['heading'])
    
    plt.plot(df['timestamp_ms'] / 1000, heading_deg, alpha=0.7)
    plt.xlabel('Time (s)')
    plt.ylabel('Heading (degrees)')
    plt.title('Robot Heading')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'heading.png'), dpi=150)
        print(f"Saved: heading.png")
    
    plt.show()

def calculate_tracking_error(df):
    """Calculate tracking error statistics"""
    errors = []
    for i in range(3):
        error = df[f'target_speed_{i}'] - df[f'motor_speed_{i}']
        mae = np.mean(np.abs(error))
        rmse = np.sqrt(np.mean(error**2))
        errors.append({
            'motor': i,
            'mae': mae,
            'rmse': rmse,
            'max_error': np.max(np.abs(error))
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
    
    print(f"\nMotor Speeds (avg ± std):")
    for i in range(3):
        speed = df[f'motor_speed_{i}']
        print(f"  Motor {i}: {speed.mean():.2f} ± {speed.std():.2f}")
    
    print(f"\nMotor Currents (avg ± std):")
    for i in range(3):
        current = df[f'motor_current_{i}']
        print(f"  Motor {i}: {current.mean():.2f} ± {current.std():.2f} A")
    
    print(f"\nTracking Error Statistics:")
    errors = calculate_tracking_error(df)
    for err in errors:
        print(f"  Motor {err['motor']}:")
        print(f"    MAE:  {err['mae']:.4f}")
        print(f"    RMSE: {err['rmse']:.4f}")
        print(f"    Max:  {err['max_error']:.4f}")
    
    print(f"\nPosition Range:")
    print(f"  X: [{df['position_x'].min():.2f}, {df['position_x'].max():.2f}]")
    print(f"  Y: [{df['position_y'].min():.2f}, {df['position_y'].max():.2f}]")
    
    print(f"\nHeading Range:")
    print(f"  {np.rad2deg(df['heading'].min()):.2f}° - {np.rad2deg(df['heading'].max()):.2f}°")
    
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
    plot_motor_speeds(df, output_dir)
    plot_motor_currents(df, output_dir)
    plot_trajectory(df, output_dir)
    plot_heading(df, output_dir)
    
    print(f"\nAnalysis complete! Plots saved to: {output_dir}")

if __name__ == "__main__":
    main()
