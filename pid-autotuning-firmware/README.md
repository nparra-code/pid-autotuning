# PID Autotuning Firmware for Omniwheel Robot

| Developed in | ESP32-S3 | Linux |
| ------------ | -------- | ----- |

## Overview

This project implements an adaptive PID control system for a three-wheeled omniwheel robot using machine learning-based autotuning. The system uses a Long Short-Term Memory (LSTM) Recurrent Neural Network to automatically tune PID parameters for each wheel motor, enabling robust and adaptive control in dynamic environments.

## Project Goals

The primary objective is to develop an intelligent control system that:

- **Automatically tunes PID parameters** for each of the three omniwheel motors
- **Minimizes manual calibration** by learning tuning of control parameters from operational data
- **Improves control performance** over time through continuous learning

## System Architecture

### Hardware Components

- **ESP32-S3 microcontroller**: Main controller running the robot firmware
- **Three omniwheel motors**: Arranged at 120° intervals with 30° orientation angle (δ)
- **Motor encoders**: Provide velocity feedback for each wheel
- **Communication interface**: Wireless link to external computing station where the AI model is running

### Software Architecture

The system operates in a client-server architecture:

1. **Robot Firmware (ESP32-S3)**:
   - Executes predefined motion patterns
   - Collects real-time sensor data (motor velocities, errors, control signals)
   - Applies PID control with current parameters
   - Communicates with the inference station

2. **Inference Station (External)**:
   - Runs the LSTM RNN model
   - Processes data received from the robot
   - Generates optimized PID parameters (Kp, Ki, Kd) for each wheel
   - Sends updated parameters back to the robot

3. **Training Pipeline**:
   - Data collection from robot operation
   - LSTM model training on collected trajectories
   - Model validation and deployment

## Autotuning Process

The autotuning cycle follows these steps:

1. **Pattern Execution**: Robot executes a predefined motion pattern with current PID parameters
2. **Data Collection**: Firmware collects time-series data including:
   - Current state (position, velocity, orientation)
   - Reference velocities for each wheel
   - Control errors (e[k], e[k-1], e[k-2])
   - Applied control signals
3. **Data Transmission**: Collected data is sent to the inference station
4. **Model Inference**: LSTM model analyzes the trajectory and generates new PID parameters
5. **Parameter Update**: Robot receives and applies new PID constants
6. **Performance Comparison**: Robot re-executes the pattern to evaluate improvement
7. **Iteration**: Process repeats until optimal performance is achieved

## Current Implementation Status

### Implemented Features

- Basic robot movement calculation (`mov_calculation.c`)
- Control framework structure (`control_main.c`)
- Main firmware initialization (`firmware_main.c`)

### In Development

- PID control loop implementation
- Data collection and logging system
- Communication protocol with inference station
- LSTM model training pipeline
- Real-time parameter update mechanism
- Performance metrics and comparison system

## Mathematical Model

The robot's dynamics are based on a three-wheeled omniwheel configuration:

### Kinematics
- **World frame**: (x, y, φ) - robot position and orientation
- **Body frame**: (x_b, y_b, φ_b) - local robot coordinates
- **Wheel velocities**: ω₁, ω₂, ω₃ - angular velocities of each wheel

### Control System
- **State feedback**: Current velocities measured by encoders
- **Reference tracking**: Desired velocities from motion planning
- **PID output**: Motor voltages for each wheel

The LSTM model learns the mapping:
```
Input: [s[k], r[k], e[k], e[k-1], e[k-2]] → Output: [Kp, Ki, Kd]
```

Where:
- `s[k]`: Current measured state
- `r[k]`: Reference setpoint
- `e[k], e[k-1], e[k-2]`: Current and past errors

## Build and Flash

### Prerequisites

- ESP-IDF v5.5.1 or later
- ESP32-S3 development board
- Python 3.8+ for training pipeline

### Configuration

```bash
idf.py menuconfig
```

Navigate to **Example Configuration** to set:
- Communication parameters (WiFi/Serial)
- PID initial values
- Data logging settings
- Motion pattern parameters

### Build

```bash
idf.py build
```

### Flash and Monitor

```bash
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type `Ctrl-]`)

## Project Structure

```
pid-autotuning-firmware/
├── CMakeLists.txt
├── README.md
├── main/
│   ├── CMakeLists.txt
│   ├── control_main.c       # PID control implementation
│   ├── firmware_main.c      # Main application entry
│   ├── mov_calculation.c    # Movement kinematics
│   └── include/
│       ├── control_main.h
│       └── mov_calculation.h
└── Simulation/              # LSTM training scripts (external)
```

## References

- [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/index.html)
- [Control Systems with Python](https://python-control.readthedocs.io/)
- LSTM Networks for Time Series Prediction
- Adaptive PID Control for Robotics Applications

---

**Note**: This is an active research and development project. The PID autotuning functionality is under development, and the current codebase provides the foundational framework for robot control and communication.