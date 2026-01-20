# Telemetry Server Documentation

This directory contains Python servers for robot telemetry data handling, motor identification, and PID autotuning.

## Server Overview

### 1. `python_server.py` - PID Autotuner Server (Primary)

**Purpose**: Receives robot telemetry data, runs RNN inference, and returns optimized PID constants.

**Features**:
- Real-time telemetry data reception via TCP sockets
- LSTM RNN model inference for PID parameter prediction
- Performance evaluation with oscillation and steady-state error metrics
- Before/after comparison visualizations
- Trajectory reconstruction from wheel velocities
- Data logging to CSV and binary formats
- CRC32 checksum validation
- Support for multiple tuning iterations

**Operating Modes**:
- `MODE = 0`: RNN Inference mode (uses trained LSTM model)
- `MODE = 1`: Classical PID constants mode (returns predefined values)

**Configuration**:
```python
MODEL_NAME = "lstm64tanh_32relu_exp_1401"  # LSTM model selection
PORT = 8888                                 # Server port
RUNS_TO_CONVERGE = 2                        # Iterations before convergence
TELEMETRY_SAMPLE_WINDOW = 100               # Samples per batch
```

**Usage**:
```bash
python3 python_server.py
```

The server will:
1. Load the trained LSTM model from `models/` directory
2. Listen on port 8888 for ESP32 connections
3. Receive telemetry batches from robot
4. Evaluate performance metrics
5. Run RNN inference (if MODE=0) or return classical constants (if MODE=1)
6. Send PID parameters back to robot
7. Generate comparison visualizations after tuning
8. Save logs to `autotuning_logs_new/` directory

---

### 2. `data_logger_server.py` - Data Logger Server

**Purpose**: Receives robot telemetry data and saves it to local files for offline analysis.

**Features**:
- Logs all received telemetry data to CSV and binary formats
- Creates session metadata with timestamps and statistics
- Validates data integrity with CRC32 checksums
- Sends acknowledgments back to ESP32
- No inference or PID calculations

**Usage**:
```bash
python3 data_logger_server.py
```

**Output Files** (saved in `telemetry_logs/`):

1. **CSV File** (`telemetry_YYYYMMDD_HHMMSS_[Kp][Ki][Kd].csv`)
   - Human-readable format with headers
   - Easy import into Excel, MATLAB, Python
   - Columns:
     - `timestamp_ms`: Milliseconds since boot
     - `motor_state_0/1/2`: Current wheel velocities (rad/s)
     - `motor_setpoint_0/1/2`: Reference wheel velocities (rad/s)
     - `error_0/1/2_k/k1/k2`: PID error history for each motor

2. **Binary File** (`telemetry_YYYYMMDD_HHMMSS_[Kp][Ki][Kd]_raw.bin`)
   - Raw binary format (exact data as received)
   - Full precision preservation
   - Useful for replay or detailed analysis

3. **Metadata File** (`telemetry_YYYYMMDD_HHMMSS_[Kp][Ki][Kd]_meta.json`)
   - Session information
   - Start/end timestamps
   - Total samples and batches
   - Duration statistics
   - PID constants used
   - File references

---

### 3. `motor_ident_server.py` - Motor Identification Server

**Purpose**: Receives PWM inputs and motor velocity responses for system identification.

**Features**:
- Real-time step response data collection
- Automated step change detection
- Parameter extraction:
  - Dead time (Td)
  - Time constant (tau)
  - Process gain (Kp)
  - Rise time (10%-90%)
  - Settling time (2% criterion)
  - Overshoot percentage
- CSV data logging with analysis results
- Visualization of step responses
- Separate port (8889) from main telemetry

**Usage**:
```bash
python3 motor_ident_server.py
```

**Output**:
- CSV files in `motor_ident_logs/`
- Step response plots
- Parameter summary for classical tuning methods (Ziegler-Nichols, Cohen-Coon)

---

### 4. `analyze_telemetry.py` - Post-Analysis Tool

**Purpose**: Loads and visualizes logged telemetry data.

**Features**:
- Individual motor tracking plots (actual vs setpoint)
- PID error history visualization
- Combined motor performance comparison
- Statistical metrics calculation
- High-resolution image export

**Usage**:
```bash
python3 analyze_telemetry.py path/to/telemetry.csv
```

**Generated Plots**:
- Individual motor tracking (3 subplots)
- Error history (3 motors with k, k-1, k-2)
- Combined performance overview
- Performance statistics summary

---

## Communication Protocol

### Telemetry Protocol (Port 8888)

**Data Structure** (`ROBOT_SAMPLE_STRUCT`):
```c
typedef struct {
    uint32_t timestamp_ms;        // Milliseconds since boot
    float motor_state[3];         // Current wheel velocities [rad/s]
    float motor_setpoint[3];      // Reference wheel velocities [rad/s]
    float errors[3][3];           // PID errors: [motor][k, k-1, k-2]
} ROBOT_SAMPLE_STRUCT;            // Total: 64 bytes (16 floats)
```

**Message Types**:

1. **Telemetry Batch** (ESP32 → Server):
   ```
   [BATCH_SIZE: 4 bytes]
   [ROBOT_SAMPLE_1: 64 bytes]
   [ROBOT_SAMPLE_2: 64 bytes]
   ...
   [CRC32: 4 bytes]
   ```

2. **Inference Request** (Server → ESP32):
   ```
   MSG_TYPE_INFERENCE_REQUEST (0x03)
   ```

3. **PID Parameters Response** (ESP32 → Server):
   ```
   [Kp: 4 bytes float]
   [Ki: 4 bytes float]
   [Kd: 4 bytes float]
   ```

4. **New PID Constants** (Server → ESP32):
   ```
   MSG_TYPE_NEW_PID_CONSTANTS (0x02)
   [Kp: 4 bytes float]
   [Ki: 4 bytes float]
   [Kd: 4 bytes float]
   ```

### Motor Identification Protocol (Port 8889)

**Data Structure**:
```c
typedef struct {
    uint32_t timestamp_ms;
    float pwm_duty;              // PWM input [0.0-1.0]
    float motor_velocity;        // Measured velocity [rad/s]
} MOTOR_IDENT_SAMPLE;
```

---

## Performance Metrics

The RNN autotuner evaluates PID performance using two key metrics:

### 1. Oscillation Score
- Measures tracking stability
- Calculates variance around setpoint
- Lower is better (less oscillation)
- Formula: `σ²(error) / σ²(setpoint)`

### 2. Steady-State Error
- Measures final tracking accuracy
- Uses last 20% of data samples
- Mean absolute error in steady state
- Lower is better (less bias)

### 3. Combined Score
- Weighted combination: `0.7 × oscillation + 0.3 × steady_state`
- Used to determine convergence
- Training target for LSTM model

---

## RNN Inference Mode (MODE 0)

When `MODE = 0`, the server uses a trained LSTM model:

**Model Architecture**:
- Input: 100 samples × 15 features (motor states, setpoints, errors)
- LSTM Layer 1: 64 units, tanh activation
- LSTM Layer 2: 32 units, tanh activation
- Dense Layer: 3 outputs (Kp, Ki, Kd) with ReLU activation

**Inference Process**:
1. Receive telemetry batch (100 samples)
2. Build sequence using `build_sequences()`
3. Normalize input features
4. Run LSTM forward pass
5. Denormalize PID outputs
6. Evaluate performance metrics
7. Send PID constants to ESP32
8. Repeat until converged (default 2 iterations)

**Model Files**:
- `models/lstm64tanh_32relu_exp_1401.h5` (recommended)
- `models/lstm128tanh_32relu_exp_1401.h5` (larger, slower)
- `models/lstm64tanh_64relu_exp_1401.h5` (alternative)

---

## Classical Constants Mode (MODE 1)

When `MODE = 1`, the server returns predefined PID constants:

```python
DEFAULT_KP = 0.5
DEFAULT_KI = 0.1
DEFAULT_KD = 0.01
```

Use this mode for:
- Testing without trained models
- Baseline performance comparison
- Debugging communication issues

---

## Workflow Example

**Complete Autotuning Session**:

1. **Start Server**:
   ```bash
   python3 python_server.py
   ```

2. **ESP32 Boot**: Robot connects to server via WiFi

3. **Initial Movement**: Robot executes predefined movement sequence

4. **Telemetry Stream**: ESP32 sends batches (100 samples each)

5. **Performance Evaluation**: Server calculates metrics

6. **RNN Inference**: LSTM predicts better PID constants

7. **Parameter Update**: Server sends new Kp, Ki, Kd to ESP32

8. **Iteration**: Repeat steps 3-7 until convergence (default 2 runs)

9. **Visualization**: Server generates before/after comparison plots:
   - Motor tracking accuracy
   - Error evolution
   - Trajectory reconstruction
   - Performance score comparison

10. **Data Saving**: Logs saved to `autotuning_logs_new/` with:
    - CSV telemetry data
    - Binary raw data
    - PNG visualization
    - JSON metadata

---

## Dependencies

Install required Python packages:

```bash
pip3 install numpy matplotlib tensorflow keras scikit-learn
```

**Package Versions**:
- Python >= 3.8
- NumPy >= 1.19
- TensorFlow >= 2.4
- Keras >= 2.4
- Matplotlib >= 3.3
- scikit-learn >= 0.24

---

## Troubleshooting

### Server Won't Start
- **Port already in use**: Change `PORT` in script or kill existing process
- **Model not found**: Verify `.h5` file exists in `models/` directory
- **Import errors**: Install missing packages with pip

### No Data Received
- **Network issues**: Check ESP32 WiFi connection and server IP
- **Firewall**: Allow Python on port 8888/8889
- **Wrong port**: Verify ESP32 and server use matching ports

### Poor Tuning Results
- **Insufficient data**: Increase `TELEMETRY_SAMPLE_WINDOW` (default 100)
- **Wrong model**: Try different LSTM models in `models/`
- **Movement too short**: Ensure robot completes full trajectory
- **Noise**: Check sensor calibration and mechanical issues

### Data Quality Issues
- **CRC errors**: Check for WiFi interference or packet corruption
- **Missing samples**: Increase ESP32 buffer size or reduce sampling rate
- **Timestamp jumps**: Verify FreeRTOS tick configuration

---

## File Structure

```
pid-autotuning-firmware/
├── python_server.py           # Main RNN autotuner server
├── data_logger_server.py      # Data logging server
├── motor_ident_server.py      # Motor identification server
├── analyze_telemetry.py       # Post-analysis visualization tool
├── view_motor_ident.py        # Motor ident result viewer
├── requirements.txt           # Python dependencies
├── models/                    # Trained LSTM models (.h5)
├── autotuning_logs_new/       # RNN autotuning results
├── telemetry_logs/            # Raw data logs
└── motor_ident_logs/          # Motor identification data
```

---

## Authors

- Nelson (Firmware & RNN Integration)
- Developed for ESP32-S3 Omniwheel Robot Platform

## References

- LSTM Architecture: Hochreiter & Schmidhuber (1997)
- PID Control: Åström & Murray (2008)
- TensorFlow/Keras: https://www.tensorflow.org/

---

## Protocol Details

Both servers use the same protocol to communicate with the ESP32 firmware:

### Message Header (8 bytes):
- `type` (1 byte): Message type (0x01 = DATA_BATCH, 0x02 = PID_RESPONSE, 0x03 = ACK)
- `version` (1 byte): Protocol version
- `length` (2 bytes): Payload length
- `checksum` (4 bytes): CRC32 checksum of payload

### Data Batch Payload:
- `sample_count` (2 bytes): Number of samples in batch
- `sequence_number` (2 bytes): Batch sequence number
- Samples array (52 bytes each):
  - `timestamp_ms` (4 bytes)
  - `motor_speed[3]` (12 bytes)
  - `motor_current[3]` (12 bytes)
  - `target_speed[3]` (12 bytes)
  - `position_x` (4 bytes)
  - `position_y` (4 bytes)
  - `heading` (4 bytes)

---

## Requirements

```bash
pip install numpy
```

---

## Configuration

You can modify these constants in the server files:

- `PORT`: Server port (default: 8888)
- `TELEMETRY_SAMPLE_WINDOW`: Samples per batch (default: 100)
- `LOG_DIR`: Directory for log files (default: "telemetry_logs")

---

## Running Multiple Servers

If you want to run both servers simultaneously (for development/testing), you'll need to use different ports:

```python
# In data_logger_server.py, change the port
PORT = 8889  # Different port

# Then run both:
python3 python_server.py       # Port 8888 (PID autotuning)
python3 data_logger_server.py  # Port 8889 (Data logging)
```

And configure your ESP32 to send to the appropriate port based on your needs.
