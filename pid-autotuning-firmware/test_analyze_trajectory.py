#!/usr/bin/env python3
"""
Test script to demonstrate the new trajectory analysis features
Creates synthetic test data to verify the plotting functions
"""

import pandas as pd
import numpy as np
import os

# Create test data directory
os.makedirs("telemetry_logs", exist_ok=True)

# Simulation parameters
duration = 10  # seconds
sample_rate = 100  # Hz
num_samples = duration * sample_rate

# Time vector
time_ms = np.linspace(0, duration * 1000, num_samples)

# Create a test trajectory: robot moves in a circle, then straight
t = time_ms / 1000.0

# Desired movement: circle for first 5 seconds, then straight line
omega_circle = 2 * np.pi / 5  # One rotation in 5 seconds
linear_speed = 0.2  # 0.2 m/s

# Calculate desired robot velocities
vx_desired = np.where(t < 5, linear_speed * np.cos(omega_circle * t), linear_speed)
vy_desired = np.where(t < 5, linear_speed * np.sin(omega_circle * t), 0)

# Convert robot velocities to wheel velocities (inverse kinematics)
# For 3-wheel omni robot with wheels at 0°, 120°, 240°
wheel_radius = 0.03  # 3cm

# Wheel angles
theta0 = 0  # Right wheel
theta1 = 2 * np.pi / 3  # Left wheel (120°)
theta2 = 4 * np.pi / 3  # Back wheel (240°)

# Inverse kinematics (simplified, assuming no rotation)
w0_desired = (vx_desired * np.cos(theta0) - vy_desired * np.sin(theta0)) / wheel_radius
w1_desired = (vx_desired * np.cos(theta1) - vy_desired * np.sin(theta1)) / wheel_radius
w2_desired = (vx_desired * np.cos(theta2) - vy_desired * np.sin(theta2)) / wheel_radius

# Add some realistic tracking error (noise + lag)
tracking_error = 0.1  # 10% error
lag_samples = 5  # 50ms lag

w0_actual = w0_desired * (1 + np.random.normal(0, tracking_error, num_samples))
w1_actual = w1_desired * (1 + np.random.normal(0, tracking_error, num_samples))
w2_actual = w2_desired * (1 + np.random.normal(0, tracking_error, num_samples))

# Apply lag
w0_actual = np.concatenate([np.zeros(lag_samples), w0_actual[:-lag_samples]])
w1_actual = np.concatenate([np.zeros(lag_samples), w1_actual[:-lag_samples]])
w2_actual = np.concatenate([np.zeros(lag_samples), w2_actual[:-lag_samples]])

# Calculate errors
error_0_k = w0_desired - w0_actual
error_1_k = w1_desired - w1_actual
error_2_k = w2_desired - w2_actual

# Create error history (k-1 and k-2)
error_0_k1 = np.roll(error_0_k, 1)
error_0_k2 = np.roll(error_0_k, 2)
error_1_k1 = np.roll(error_1_k, 1)
error_1_k2 = np.roll(error_1_k, 2)
error_2_k1 = np.roll(error_2_k, 1)
error_2_k2 = np.roll(error_2_k, 2)

# Create DataFrame
data = {
    'timestamp_ms': time_ms,
    'motor_state_0': w0_actual,
    'motor_state_1': w1_actual,
    'motor_state_2': w2_actual,
    'motor_setpoint_0': w0_desired,
    'motor_setpoint_1': w1_desired,
    'motor_setpoint_2': w2_desired,
    'error_0_k': error_0_k,
    'error_0_k1': error_0_k1,
    'error_0_k2': error_0_k2,
    'error_1_k': error_1_k,
    'error_1_k1': error_1_k1,
    'error_1_k2': error_1_k2,
    'error_2_k': error_2_k,
    'error_2_k1': error_2_k1,
    'error_2_k2': error_2_k2
}

df = pd.DataFrame(data)

# Save to CSV
csv_path = "telemetry_logs/telemetry_test_trajectory.csv"
df.to_csv(csv_path, index=False)

print(f"✓ Created test data: {csv_path}")
print(f"  Duration: {duration}s")
print(f"  Samples: {num_samples}")
print(f"  Sample rate: {sample_rate} Hz")
print(f"\nTest scenario:")
print(f"  0-5s: Robot moves in a circle (radius ~0.16m)")
print(f"  5-10s: Robot moves in a straight line")
print(f"\nTo analyze this data, run:")
print(f"  python3 analyze_telemetry.py {csv_path}")
