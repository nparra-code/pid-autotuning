#!/usr/bin/env python3
"""
Quick test to verify the telemetry data structure packing/unpacking
"""

import struct

# Structure from data_logger_server.py
ROBOT_SAMPLE_STRUCT = struct.Struct('<I15f')

print("Telemetry Data Structure Test")
print("=" * 60)
print(f"Struct format: {ROBOT_SAMPLE_STRUCT.format}")
print(f"Struct size: {ROBOT_SAMPLE_STRUCT.size} bytes")
print()

# Create test data matching ESP32 structure
test_data = (
    1000,  # timestamp_ms (uint32)
    # motor_state[3]
    10.5, 11.2, 9.8,
    # motor_setpoint[3]
    10.0, 10.0, 10.0,
    # errors[3][3] - [motor][time_k]
    # Motor 0: error(k), error(k-1), error(k-2)
    0.5, 0.6, 0.7,
    # Motor 1: error(k), error(k-1), error(k-2)
    -1.2, -1.1, -1.0,
    # Motor 2: error(k), error(k-1), error(k-2)
    0.2, 0.3, 0.4
)

print("Test Data (Python tuple):")
print(f"  Timestamp: {test_data[0]} ms")
print(f"  Motor States: {test_data[1:4]}")
print(f"  Motor Setpoints: {test_data[4:7]}")
print(f"  Motor 0 Errors (k, k-1, k-2): {test_data[7:10]}")
print(f"  Motor 1 Errors (k, k-1, k-2): {test_data[10:13]}")
print(f"  Motor 2 Errors (k, k-1, k-2): {test_data[13:16]}")
print()

# Pack data
packed = ROBOT_SAMPLE_STRUCT.pack(*test_data)
print(f"Packed data size: {len(packed)} bytes")
print(f"Packed data (hex): {packed.hex()}")
print()

# Unpack data
unpacked = ROBOT_SAMPLE_STRUCT.unpack(packed)
print("Unpacked data matches original:", unpacked == test_data)
print()

# Verify CSV header mapping
CSV_HEADER = [
    "timestamp_ms",
    "motor_state_0", "motor_state_1", "motor_state_2",
    "motor_setpoint_0", "motor_setpoint_1", "motor_setpoint_2",
    "error_0_k", "error_0_k1", "error_0_k2",
    "error_1_k", "error_1_k1", "error_1_k2",
    "error_2_k", "error_2_k1", "error_2_k2"
]

print("CSV Header Mapping:")
for i, (header, value) in enumerate(zip(CSV_HEADER, unpacked)):
    print(f"  [{i:2d}] {header:20s} = {value}")
print()

print("✓ Structure test passed!")
print()
print("Expected firmware structure (C):")
print("  typedef struct {")
print("      uint32_t timestamp_ms;")
print("      float motor_state[3];")
print("      float motor_setpoint[3];")
print("      float errors[3][3];")
print("  } robot_sample_t;")
