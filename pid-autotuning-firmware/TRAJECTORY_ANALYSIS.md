# Telemetry Analysis - Trajectory Visualization Update

## Overview
Updated the telemetry analysis script to provide better visualization of PID control performance and robot movement.

## New Features

### 1. Individual Motor Tracking Plots
**Function:** `plot_individual_motor_tracking()`

- Creates **3 separate subplots**, one for each wheel
- Each plot shows:
  - Actual motor state (solid blue line)
  - Desired setpoint (dashed orange line)
  - Shaded error region between them
- Better visualization of individual wheel performance
- Easier to spot tracking issues on specific motors

**Output:** `individual_motor_tracking.png`

### 2. Robot Trajectory Visualization
**Function:** `plot_robot_trajectory()`

Uses **3-wheel omni robot kinematics** to reconstruct the robot's path:

#### Kinematic Model
```
Robot Configuration:
- 3 wheels with 3cm radius (0.03m)
- Wheels arranged at 120° intervals:
  * Motor 0 (Right): 0°
  * Motor 1 (Left): 120°
  * Motor 2 (Back): 240°

Forward Kinematics:
  vx_robot = r * (w0*cos(0°) + w1*cos(120°) + w2*cos(240°)) / 3
  vy_robot = r * (-w0*sin(0°) - w1*sin(120°) - w2*sin(240°)) / 3

Where:
  - r = wheel radius (0.03m)
  - w0, w1, w2 = wheel angular velocities (rad/s)
  - vx, vy = robot velocity in x and y directions
```

#### Trajectory Plots

**Left plot:** 2D trajectory comparison
- Blue dashed line: Desired path (from setpoints)
- Colored scatter: Actual path (from motor states)
  - Color indicates time progression
  - Green circle: Start position
  - Red square: End position

**Right plot:** Position error over time
- Shows tracking error magnitude in centimeters
- Shaded area under the curve
- Statistics box showing:
  - Mean error
  - Maximum error
  - Final error

**Output:** `robot_trajectory.png`

### 3. Retained Error Analysis Plots

- **PID Errors** (`plot_errors()`): Shows error history k, k-1, k-2 for each motor
- **Error Magnitude** (`plot_error_magnitude()`): Absolute error comparison across motors

## Changes Summary

### Removed
- ❌ `plot_motor_states()` - Old combined plot of all motors

### Added
- ✅ `plot_individual_motor_tracking()` - 3 separate wheel tracking plots
- ✅ `calculate_robot_trajectory()` - Kinematic trajectory reconstruction
- ✅ `plot_robot_trajectory()` - Desired vs actual path visualization

### Modified
- Enhanced error statistics in `print_statistics()`
- Reorganized plot generation order in `main()`

## Usage

### Analyze Real Data
```bash
# Start data logger server
python3 data_logger_server.py

# Flash and run firmware
idf.py flash monitor

# After collecting data
python3 analyze_telemetry.py telemetry_logs/telemetry_YYYYMMDD_HHMMSS.csv
```

### Test with Synthetic Data
```bash
# Generate test trajectory data
python3 test_analyze_trajectory.py

# Analyze it
python3 analyze_telemetry.py telemetry_logs/telemetry_test_trajectory.csv
```

## Output Files

When you run the analysis, it creates a directory with 4 plots:

```
telemetry_logs/
└── analysis_telemetry_YYYYMMDD_HHMMSS/
    ├── individual_motor_tracking.png  (NEW - 3 wheel tracking plots)
    ├── robot_trajectory.png           (NEW - 2D path visualization)
    ├── pid_errors.png                 (Error history k, k-1, k-2)
    └── error_magnitude.png            (Absolute errors)
```

## Kinematic Parameters

Currently using default values:
- **Wheel radius:** 3cm (0.03m)
- **Wheel arrangement:** 120° spacing (0°, 120°, 240°)

To change wheel radius, modify the call in `plot_robot_trajectory()`:
```python
wheel_radius = 0.03  # Change this value in meters
```

## Notes

- The trajectory calculation assumes the robot starts at origin (0, 0)
- Rotation (angular velocity) is not calculated as it requires the wheelbase radius L
- Position is computed by numerical integration of velocities
- Motor states and setpoints should be in **rad/s** (angular velocity)

## Interpretation

### Good PID Performance
- Actual trajectory closely follows desired trajectory
- Small position error (few centimeters)
- Motor states track setpoints with minimal delay
- Error is centered around zero (no bias)

### Poor PID Performance
- Large deviation between paths
- High position error
- Significant lag between setpoint and actual
- Persistent positive/negative error (bias)

## Mathematical Details

### Wheel Velocity to Robot Velocity
The transformation from wheel velocities to robot velocities uses:

```python
# For each wheel at angle θ:
vx_contribution = w * r * cos(θ)
vy_contribution = w * r * (-sin(θ))

# Sum all contributions and average:
vx_robot = (vx0 + vx1 + vx2) / 3
vy_robot = (vy0 + vy1 + vy2) / 3
```

### Integration for Position
Position is calculated by integrating velocity over time:

```python
x[i] = x[i-1] + vx * dt
y[i] = y[i-1] + vy * dt
```

Where `dt` is the time step between samples (~0.01s for 100Hz).
