# Telemetry Data Structure Update

## Summary
Updated the data logger server and analysis scripts to match the new telemetry data structure from the ESP32 firmware.

## New Data Structure

### Robot Sample Format
```c
typedef struct {
    uint32_t timestamp_ms;      // Timestamp in milliseconds
    float motor_state[3];       // Motor states (3 wheels)
    float motor_setpoint[3];    // Motor setpoints
    float errors[3][3];         // Error history for each motor [motor][time]
} robot_sample_t;
```

**Total Size:** 1 uint32 (4 bytes) + 15 floats (60 bytes) = 64 bytes per sample

### Error History Format
- `errors[motor][0]` - Current error (k)
- `errors[motor][1]` - Previous error (k-1)
- `errors[motor][2]` - Two steps back (k-2)

## Updated Files

### 1. `data_logger_server.py`
**Changes:**
- Updated `ROBOT_SAMPLE_STRUCT` from `'<I12f'` to `'<I15f'`
- Updated CSV header to match new data fields:
  - `motor_state_0/1/2` - Current motor states
  - `motor_setpoint_0/1/2` - Target setpoints
  - `error_0/1/2_k` - Current errors (k)
  - `error_0/1/2_k1` - Previous errors (k-1)
  - `error_0/1/2_k2` - Two steps back errors (k-2)

**Removed:**
- Old fields: `motor_speed`, `motor_current`, `position_x/y`, `heading`

### 2. `analyze_telemetry.py`
**Changes:**
- Replaced `plot_motor_speeds()` with `plot_motor_states()` - Shows motor states vs setpoints
- Replaced `plot_motor_currents()` with `plot_errors()` - Shows PID error history for all motors
- Replaced `plot_trajectory()` and `plot_heading()` with `plot_error_magnitude()` - Shows absolute error over time
- Updated `calculate_tracking_error()` to use recorded error values instead of calculating from speed difference
- Updated `print_statistics()` to show motor states, setpoints, and comprehensive error statistics

**New Plots Generated:**
1. `motor_states.png` - Motor states and setpoints over time (3 subplots)
2. `pid_errors.png` - Error history (k, k-1, k-2) for each motor (3 subplots)
3. `error_magnitude.png` - Absolute error magnitude comparison

**New Statistics:**
- Mean Error (shows bias)
- Standard Deviation
- MAE (Mean Absolute Error)
- RMSE (Root Mean Square Error)
- Max Error

## Testing

### Start the Data Logger Server
```bash
python3 data_logger_server.py
```

### Flash and Monitor Firmware
```bash
idf.py flash monitor
```

### Analyze Collected Data
```bash
python3 analyze_telemetry.py telemetry_logs/telemetry_YYYYMMDD_HHMMSS.csv
```

## Data Flow

1. **Firmware** (`firmware_main.c`):
   - `read_robot_sensors()` collects motor states, setpoints, and error history
   - `telemetry_send_batch()` sends 100 samples to server

2. **Data Logger Server** (`data_logger_server.py`):
   - Receives binary data batches
   - Unpacks 15 floats per sample
   - Logs to CSV with proper headers
   - Sends ACK back to firmware

3. **Analysis Script** (`analyze_telemetry.py`):
   - Loads CSV data
   - Generates plots showing PID performance
   - Calculates error statistics for tuning evaluation

## Notes

- Sample rate: ~100Hz (10ms interval, configured in firmware)
- Batch size: 100 samples per transmission
- Each batch includes sequence number for tracking
- CRC32 checksum for data integrity
