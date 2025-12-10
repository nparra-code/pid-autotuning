# Telemetry Server Documentation

This directory contains two Python servers for handling robot telemetry data:

## 1. `python_server.py` - PID Autotuner Server

Receives robot telemetry data, runs RNN inference, and returns PID constants.

### Features:
- Receives batches of robot telemetry samples
- Runs RNN inference (mock implementation included)
- Sends back optimized PID constants
- Validates data integrity with CRC32 checksums

### Usage:
```bash
python3 python_server.py
```

The server listens on port 8888 by default.

---

## 2. `data_logger_server.py` - Data Logger Server

Receives robot telemetry data and saves it to local files for analysis and debugging.

### Features:
- Logs all received telemetry data to CSV and binary formats
- Creates session metadata with timestamps and statistics
- Validates data integrity with CRC32 checksums
- Sends acknowledgments back to the client

### Usage:
```bash
python3 data_logger_server.py
```

### Output Files:

All logs are saved in the `telemetry_logs/` directory with timestamps:

1. **CSV File** (`telemetry_YYYYMMDD_HHMMSS.csv`)
   - Human-readable format
   - Contains all telemetry samples with headers
   - Easy to import into Excel, MATLAB, Python for analysis
   - Columns:
     - timestamp_ms
     - motor_speed_0, motor_speed_1, motor_speed_2
     - motor_current_0, motor_current_1, motor_current_2
     - target_speed_0, target_speed_1, target_speed_2
     - position_x, position_y, heading

2. **Binary File** (`telemetry_YYYYMMDD_HHMMSS_raw.bin`)
   - Raw binary format (exact data as received)
   - Can be used for exact replay or further processing
   - Preserves full precision

3. **Metadata File** (`telemetry_YYYYMMDD_HHMMSS_meta.json`)
   - Session information
   - Start/end timestamps
   - Total samples and batches
   - Duration
   - File references

### Example Analysis:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load telemetry data
df = pd.read_csv('telemetry_logs/telemetry_20251210_143025.csv')

# Plot motor speeds
plt.figure(figsize=(12, 6))
plt.plot(df['timestamp_ms'], df['motor_speed_0'], label='Motor 0')
plt.plot(df['timestamp_ms'], df['motor_speed_1'], label='Motor 1')
plt.plot(df['timestamp_ms'], df['motor_speed_2'], label='Motor 2')
plt.xlabel('Time (ms)')
plt.ylabel('Speed')
plt.legend()
plt.title('Motor Speeds Over Time')
plt.show()

# Plot position trajectory
plt.figure(figsize=(8, 8))
plt.plot(df['position_x'], df['position_y'])
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Trajectory')
plt.axis('equal')
plt.show()
```

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
